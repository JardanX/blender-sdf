/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <utility>

#include "MEM_guardedalloc.h"

#include "DNA_material_types.h"
#include "DNA_nurb_body_types.h"
#include "DNA_object_types.h"

#include "BLI_array.hh"
#include "BLI_listbase.h"
#include "BLI_map.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_anim_data.hh"
#include "BKE_geometry_set.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_mesh.hh"
#include "BKE_nurb_body.hh"
#include "BKE_object.hh"
#include "BKE_object_types.hh"

#include "BLT_translation.hh"

#include "BLO_read_write.hh"

#ifdef WITH_OPENCASCADE
#  include <BRepBuilderAPI_Transform.hxx>
#  include <BRep_Tool.hxx>
#  include <BRepAlgoAPI_Common.hxx>
#  include <BRepAlgoAPI_Cut.hxx>
#  include <BRepAlgoAPI_Fuse.hxx>
#  include <BRepFilletAPI_MakeChamfer.hxx>
#  include <BRepFilletAPI_MakeFillet.hxx>
#  include <BRepLib_ToolTriangulatedShape.hxx>
#  include <BRepMesh_IncrementalMesh.hxx>
#  include <BRepPrimAPI_MakeCylinder.hxx>
#  include <Geom_Curve.hxx>
#  include <Geom_CylindricalSurface.hxx>
#  include <Geom_Surface.hxx>
#  include <Poly_PolygonOnTriangulation.hxx>
#  include <Poly_Triangle.hxx>
#  include <Poly_Triangulation.hxx>
#  include <Standard_Failure.hxx>
#  include <TopExp.hxx>
#  include <TopExp_Explorer.hxx>
#  include <TopAbs_Orientation.hxx>
#  include <TopLoc_Location.hxx>
#  include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#  include <TopTools_IndexedMapOfShape.hxx>
#  include <TopTools_ListOfShape.hxx>
#  include <TopoDS.hxx>
#  include <TopoDS_Edge.hxx>
#  include <TopoDS_Face.hxx>
#  include <TopoDS_Shape.hxx>
#  include <gp_Ax2.hxx>
#  include <gp_Dir.hxx>
#  include <gp_Pnt.hxx>
#  include <gp_Trsf.hxx>
#  include <gp_Vec.hxx>
#endif

namespace blender {

static uint64_t nurb_body_edge_mask_for_index(const int edge_index)
{
  return (edge_index >= 0 && edge_index < 64) ? (uint64_t(1) << uint(edge_index)) : uint64_t(0);
}

static void nurb_body_materialize_edge_bevel_radii(const uint64_t bevel_edges,
                                                   const float fallback_radius,
                                                   float bevel_radii[64])
{
  if (fallback_radius <= 0.0f) {
    return;
  }
  for (int i = 0; i < 64; i++) {
    if ((bevel_edges & nurb_body_edge_mask_for_index(i)) != 0 && bevel_radii[i] > 0.0f) {
      return;
    }
  }
  for (int i = 0; i < 64; i++) {
    if ((bevel_edges & nurb_body_edge_mask_for_index(i)) != 0 && bevel_radii[i] <= 0.0f) {
      bevel_radii[i] = fallback_radius;
    }
  }
}

static void nurb_body_normalize_edge_bevel_order(const uint64_t bevel_edges,
                                                 int bevel_order[64],
                                                 int &bevel_order_next)
{
  if (bevel_edges == 0) {
    std::fill_n(bevel_order, 64, 0);
    bevel_order_next = 1;
    return;
  }

  int max_order = 0;
  for (int i = 0; i < 64; i++) {
    if ((bevel_edges & nurb_body_edge_mask_for_index(i)) == 0) {
      bevel_order[i] = 0;
      continue;
    }
    if (bevel_order[i] < 0) {
      bevel_order[i] = 0;
    }
    max_order = std::max(max_order, bevel_order[i]);
  }

  int next_order = std::max(bevel_order_next, max_order + 1);
  if (next_order <= 0) {
    next_order = 1;
  }
  for (int i = 0; i < 64; i++) {
    if ((bevel_edges & nurb_body_edge_mask_for_index(i)) != 0 && bevel_order[i] == 0) {
      bevel_order[i] = next_order++;
    }
  }
  bevel_order_next = next_order;
}

static void nurb_body_init_data(ID *id)
{
  NurbBody *body = id_cast<NurbBody *>(id);
  INIT_DEFAULT_STRUCT_AFTER(body, id);
}

static void nurb_body_copy_data(Main * /*bmain*/,
                                std::optional<Library *> /*owner_library*/,
                                ID *id_dst,
                                const ID *id_src,
                                const int /*flag*/)
{
  NurbBody *body_dst = reinterpret_cast<NurbBody *>(id_dst);
  const NurbBody *body_src = reinterpret_cast<const NurbBody *>(id_src);
  body_dst->mat = MEM_dupalloc(body_src->mat);
  BLI_duplicatelist(&body_dst->boolean_ops, &body_src->boolean_ops);
}

static void nurb_body_free_data(ID *id)
{
  NurbBody *body = reinterpret_cast<NurbBody *>(id);
  BKE_animdata_free(&body->id, false);
  BLI_freelistN(&body->boolean_ops);
  MEM_SAFE_DELETE(body->mat);
}

static void nurb_body_foreach_id(ID *id, LibraryForeachIDData *data)
{
  NurbBody *body = reinterpret_cast<NurbBody *>(id);
  for (int i = 0; i < body->totcol; i++) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, body->mat[i], IDWALK_CB_USER);
  }
}

static void nurb_body_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  NurbBody *body = reinterpret_cast<NurbBody *>(id);

  writer->write_id_struct(id_address, body);
  BKE_id_blend_write(writer, &body->id);
  BLO_write_pointer_array(writer, body->totcol, body->mat);
  writer->write_struct_list_by_id(dna::sdna_struct_id_get<NurbBodyBooleanOp>(),
                                  &body->boolean_ops);
}

static void nurb_body_blend_read_data(BlendDataReader *reader, ID *id)
{
  NurbBody *body = reinterpret_cast<NurbBody *>(id);
  BLO_read_pointer_array(reader, body->totcol, reinterpret_cast<void **>(&body->mat));
  BLO_read_struct_list(reader, NurbBodyBooleanOp, &body->boolean_ops);
  body->hovered_edge = -1;
  body->surface_hovered_edge = -1;
  if (body->selected_edges == 0 && body->selected_edge >= 0) {
    body->selected_edges = nurb_body_edge_mask_for_index(body->selected_edge);
  }
  if (body->bevel_edges == 0 && body->bevel_edge >= 0) {
    body->bevel_edges = nurb_body_edge_mask_for_index(body->bevel_edge);
  }
  if (body->bevel_radius <= 0.0f || body->bevel_edge < -1) {
    body->bevel_edge = -1;
    body->bevel_edges = 0;
    body->chamfer_edges = 0;
  }
  if (body->surface_bevel_radius <= 0.0f || body->surface_bevel_edge < -1) {
    body->surface_bevel_edge = -1;
    body->surface_bevel_edges = 0;
    body->surface_chamfer_edges = 0;
  }
  if (!ELEM(body->bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER)) {
    body->bevel_type = NURB_BODY_BEVEL_FILLET;
  }
  if (!ELEM(body->surface_bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER)) {
    body->surface_bevel_type = NURB_BODY_BEVEL_FILLET;
  }
  if (body->bevel_type == NURB_BODY_BEVEL_CHAMFER && body->chamfer_edges == 0 &&
      body->bevel_edges != 0)
  {
    body->chamfer_edges = body->bevel_edges;
  }
  if (body->surface_bevel_type == NURB_BODY_BEVEL_CHAMFER &&
      body->surface_chamfer_edges == 0 && body->surface_bevel_edges != 0)
  {
    body->surface_chamfer_edges = body->surface_bevel_edges;
  }
  nurb_body_materialize_edge_bevel_radii(
      body->bevel_edges, body->bevel_radius, body->bevel_radii);
  nurb_body_materialize_edge_bevel_radii(
      body->surface_bevel_edges, body->surface_bevel_radius, body->surface_bevel_radii);
  body->chamfer_edges &= body->bevel_edges;
  body->surface_chamfer_edges &= body->surface_bevel_edges;
  nurb_body_normalize_edge_bevel_order(
      body->bevel_edges, body->bevel_order, body->bevel_order_next);
  nurb_body_normalize_edge_bevel_order(body->surface_bevel_edges,
                                       body->surface_bevel_order,
                                       body->surface_bevel_order_next);
  if (!ELEM(body->select_mode,
            NURB_BODY_SELECT_MODE_EDGE,
            NURB_BODY_SELECT_MODE_FACE,
            NURB_BODY_SELECT_MODE_OBJECT))
  {
    body->select_mode = NURB_BODY_SELECT_MODE_EDGE;
  }
  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body->boolean_ops.first); op;
       op = op->next)
  {
    op->hovered_edge = -1;
    if (op->selected_edge < -1) {
      op->selected_edge = -1;
    }
    if (op->selected_edges == 0 && op->selected_edge >= 0) {
      op->selected_edges = nurb_body_edge_mask_for_index(op->selected_edge);
    }
    if (op->bevel_edges == 0 && op->bevel_edge >= 0) {
      op->bevel_edges = nurb_body_edge_mask_for_index(op->bevel_edge);
    }
    if (op->bevel_radius <= 0.0f || op->bevel_edge < -1) {
      op->bevel_edge = -1;
      op->bevel_edges = 0;
      op->chamfer_edges = 0;
    }
    if (!ELEM(op->bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER)) {
      op->bevel_type = NURB_BODY_BEVEL_FILLET;
    }
    if (op->bevel_type == NURB_BODY_BEVEL_CHAMFER && op->chamfer_edges == 0 &&
        op->bevel_edges != 0)
    {
      op->chamfer_edges = op->bevel_edges;
    }
    nurb_body_materialize_edge_bevel_radii(
        op->bevel_edges, op->bevel_radius, op->bevel_radii);
    op->chamfer_edges &= op->bevel_edges;
    nurb_body_normalize_edge_bevel_order(
        op->bevel_edges, op->bevel_order, op->bevel_order_next);
  }
}

IDTypeInfo IDType_ID_NB = {
    /*id_code*/ NurbBody::id_type,
    /*id_filter*/ FILTER_ID_NB,
    /*dependencies_id_types*/ FILTER_ID_MA,
    /*main_listbase_index*/ INDEX_ID_NB,
    /*struct_size*/ sizeof(NurbBody),
    /*name*/ "NurbBody",
    /*name_plural*/ N_("nurb bodies"),
    /*translation_context*/ BLT_I18NCONTEXT_ID_ID,
    /*flags*/ IDTYPE_FLAGS_APPEND_IS_REUSABLE,
    /*asset_type_info*/ nullptr,

    /*init_data*/ nurb_body_init_data,
    /*copy_data*/ nurb_body_copy_data,
    /*free_data*/ nurb_body_free_data,
    /*make_local*/ nullptr,
    /*foreach_id*/ nurb_body_foreach_id,
    /*foreach_cache*/ nullptr,
    /*foreach_path*/ nullptr,
    /*foreach_working_space_color*/ nullptr,
    /*owner_pointer_get*/ nullptr,

    /*blend_write*/ nurb_body_blend_write,
    /*blend_read_data*/ nurb_body_blend_read_data,
    /*blend_read_after_liblink*/ nullptr,

    /*blend_read_undo_preserve*/ nullptr,

    /*lib_override_apply_post*/ nullptr,
};

NurbBody *BKE_nurb_body_add(Main *bmain, const char *name)
{
  return BKE_id_new<NurbBody>(bmain, name);
}

#ifdef WITH_OPENCASCADE

static TopoDS_Shape make_centered_cylinder(const gp_Dir &axis,
                                           const float radius,
                                           const float depth,
                                           const float3 center)
{
  gp_Pnt base(center.x, center.y, center.z);
  const double half_depth = double(depth) * 0.5;
  base.Translate(gp_Vec(-axis.X() * half_depth, -axis.Y() * half_depth, -axis.Z() * half_depth));
  return BRepPrimAPI_MakeCylinder(gp_Ax2(base, axis), radius, depth).Shape();
}

static TopoDS_Shape make_body_primitive_shape(const NurbBody &body)
{
  return make_centered_cylinder(gp_Dir(1.0, 0.0, 0.0),
                                std::max(body.radius, 0.001f),
                                std::max(body.depth, 0.001f),
                                float3(0.0f));
}

static TopoDS_Shape transform_shape(const TopoDS_Shape &shape, const float mat[4][4])
{
  gp_Trsf transform;
  transform.SetValues(mat[0][0],
                      mat[1][0],
                      mat[2][0],
                      mat[3][0],
                      mat[0][1],
                      mat[1][1],
                      mat[2][1],
                      mat[3][1],
                      mat[0][2],
                      mat[1][2],
                      mat[2][2],
                      mat[3][2]);
  return BRepBuilderAPI_Transform(shape, transform, true).Shape();
}

static TopoDS_Shape make_boolean_op_tool_shape(const NurbBodyBooleanOp &op)
{
  TopoDS_Shape shape = make_centered_cylinder(gp_Dir(1.0, 0.0, 0.0),
                                              std::max(op.operand_radius, 0.001f),
                                              std::max(op.operand_depth, 0.001f),
                                              float3(0.0f));
  return transform_shape(shape, op.operand_to_target);
}

static bool cylindrical_face_radius(const TopoDS_Face &face, double &r_radius)
{
  Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
  Handle(Geom_CylindricalSurface) cylinder = Handle(Geom_CylindricalSurface)::DownCast(surface);
  if (cylinder.IsNull()) {
    return false;
  }
  r_radius = cylinder->Radius();
  return true;
}

static bool topological_edge_is_visible_outline(
    const TopoDS_Edge &edge,
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces);

static void triangulate_shape_for_preview(const TopoDS_Shape &shape, const NurbBody &body)
{
  const double linear_deflection = std::max(body.tessellation_deflection, 0.0001f);
  const double angular_deflection = std::clamp(
      double(body.tessellation_angle), 0.01, 3.14159265358979323846);
  BRepMesh_IncrementalMesh(shape, linear_deflection, false, angular_deflection, true);
}

static Vector<TopoDS_Edge> find_cut_edges(
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces,
    const float body_radius,
    const float cutter_radius)
{
  Vector<TopoDS_Edge> intersection_edges;
  Vector<TopoDS_Edge> cutter_rim_edges;

  const double body_tol = std::max(0.001, double(body_radius) * 0.002);
  const double cutter_tol = std::max(0.001, double(cutter_radius) * 0.002);

  for (int i = 1; i <= edge_faces.Extent(); i++) {
    bool touches_body = false;
    bool touches_cutter = false;
    const TopTools_ListOfShape &faces = edge_faces.FindFromIndex(i);
    for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) {
      double radius = 0.0;
      if (!cylindrical_face_radius(TopoDS::Face(it.Value()), radius)) {
        continue;
      }
      touches_body |= std::abs(radius - double(body_radius)) <= body_tol;
      touches_cutter |= std::abs(radius - double(cutter_radius)) <= cutter_tol;
    }
    if (touches_body && touches_cutter) {
      intersection_edges.append(TopoDS::Edge(edge_faces.FindKey(i)));
    }
    else if (touches_cutter) {
      const TopoDS_Edge edge = TopoDS::Edge(edge_faces.FindKey(i));
      if (topological_edge_is_visible_outline(edge, edge_faces)) {
        cutter_rim_edges.append(edge);
      }
    }
  }
  intersection_edges.extend(cutter_rim_edges);
  return intersection_edges;
}

static Vector<TopoDS_Edge> find_cut_edges(const TopoDS_Shape &shape,
                                          const float body_radius,
                                          const float cutter_radius)
{
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  return find_cut_edges(edge_faces, body_radius, cutter_radius);
}

static bool append_edge_polyline_local(const TopoDS_Edge &edge,
                                       const int samples,
                                       Vector<float3> &r_points)
{
  if (BRep_Tool::Degenerated(edge)) {
    return false;
  }

  Standard_Real first = 0.0;
  Standard_Real last = 0.0;
  TopLoc_Location location;
  Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, location, first, last);
  if (curve.IsNull() || !std::isfinite(double(first)) || !std::isfinite(double(last)) ||
      first == last)
  {
    return false;
  }

  const gp_Trsf curve_transform = location.Transformation();
  const int sample_count = std::max(samples, 2);
  const int start_points = r_points.size();
  for (int i = 0; i <= sample_count; i++) {
    const double t = double(first) + (double(last) - double(first)) *
                                         (double(i) / double(sample_count));
    gp_Pnt point;
    curve->D0(t, point);
    point.Transform(curve_transform);

    r_points.append(float3(float(point.X()), float(point.Y()), float(point.Z())));
  }
  return r_points.size() > start_points + 1;
}

static bool append_tessellated_edge_polyline_local(
    const TopoDS_Edge &edge,
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces,
    Vector<float3> &r_points)
{
  if (BRep_Tool::Degenerated(edge)) {
    return false;
  }

  const TopTools_ListOfShape *faces = edge_faces.Seek(edge);
  if (faces == nullptr) {
    return false;
  }

  for (TopTools_ListIteratorOfListOfShape it(*faces); it.More(); it.Next()) {
    const TopoDS_Face face = TopoDS::Face(it.Value());
    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull()) {
      continue;
    }

    const Handle(Poly_PolygonOnTriangulation) polygon =
        BRep_Tool::PolygonOnTriangulation(edge, triangulation, location);
    if (polygon.IsNull() || polygon->NbNodes() < 2) {
      continue;
    }

    const gp_Trsf transform = location.Transformation();
    const int start_points = r_points.size();
    for (int i = 1; i <= polygon->NbNodes(); i++) {
      const int node = polygon->Node(i);
      if (node < 1 || node > triangulation->NbNodes()) {
        continue;
      }
      gp_Pnt point = triangulation->Node(node);
      point.Transform(transform);

      r_points.append(float3(float(point.X()), float(point.Y()), float(point.Z())));
    }
    if (r_points.size() > start_points + 1) {
      return true;
    }
  }

  return false;
}

struct NurbBodySelectableEdgeRef {
  const NurbBodyBooleanOp *op = nullptr;
  int edge_index = -1;
  int flag = 0;
  float threshold = 0.0f;
  Vector<float3> points;
};

static float dist_squared_to_polyline_v3(const float3 &point, const Span<float3> polyline)
{
  if (polyline.size() < 2) {
    return FLT_MAX;
  }

  float best_dist_sq = FLT_MAX;
  for (int i = 1; i < polyline.size(); i++) {
    best_dist_sq = std::min(best_dist_sq,
                            dist_squared_to_line_segment_v3(point, polyline[i - 1], polyline[i]));
  }
  return best_dist_sq;
}

static const NurbBodySelectableEdgeRef *find_near_selectable_edge_ref(
    const Vector<float3> &points, const Span<NurbBodySelectableEdgeRef> selectable_refs)
{
  if (points.size() < 2 || selectable_refs.is_empty()) {
    return nullptr;
  }

  const float3 samples[3] = {
      points.first(),
      points[points.size() / 2],
      points.last(),
  };
  for (const NurbBodySelectableEdgeRef &ref : selectable_refs) {
    const float threshold_sq = ref.threshold * ref.threshold;
    bool near_ref = true;
    for (const float3 &sample : samples) {
      if (dist_squared_to_polyline_v3(sample, ref.points) > threshold_sq) {
        near_ref = false;
        break;
      }
    }
    if (near_ref) {
      return &ref;
    }
  }
  return nullptr;
}

static int unique_edge_face_count(const TopTools_ListOfShape &faces)
{
  TopTools_IndexedMapOfShape unique_faces;
  for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) {
    unique_faces.Add(it.Value());
  }
  return unique_faces.Extent();
}

static bool topological_edge_is_visible_outline(
    const TopoDS_Edge &edge,
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces)
{
  if (BRep_Tool::Degenerated(edge)) {
    return false;
  }

  const TopTools_ListOfShape *faces = edge_faces.Seek(edge);
  if (faces == nullptr) {
    return true;
  }

  TopTools_IndexedMapOfShape unique_faces;
  for (TopTools_ListIteratorOfListOfShape it(*faces); it.More(); it.Next()) {
    unique_faces.Add(it.Value());
  }
  if (unique_faces.Extent() == 1) {
    const TopoDS_Face face = TopoDS::Face(unique_faces.FindKey(1));
    return !BRep_Tool::IsClosed(edge, face);
  }

  return unique_faces.Extent() > 1;
}

static void append_shape_surface_edge_polylines(const TopoDS_Shape &shape,
                                                const int samples_per_edge,
                                                const Span<NurbBodySelectableEdgeRef> selectable_refs,
                                                Vector<NurbBodyEdgePolyline> &r_polylines)
{
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);

  for (int i = 1; i <= edge_faces.Extent(); i++) {
    if (unique_edge_face_count(edge_faces.FindFromIndex(i)) < 2) {
      continue;
    }

    NurbBodyEdgePolyline polyline;
    polyline.flag = NURB_BODY_EDGE_POLYLINE_SURFACE;
    const TopoDS_Edge edge = TopoDS::Edge(edge_faces.FindKey(i));
    if (!topological_edge_is_visible_outline(edge, edge_faces)) {
      continue;
    }
    if (append_tessellated_edge_polyline_local(edge, edge_faces, polyline.points) ||
        append_edge_polyline_local(edge, samples_per_edge, polyline.points))
    {
      if (const NurbBodySelectableEdgeRef *ref = find_near_selectable_edge_ref(polyline.points,
                                                                               selectable_refs))
      {
        polyline.op = ref->op;
        polyline.edge_index = ref->edge_index;
        polyline.flag |= ref->flag | NURB_BODY_EDGE_POLYLINE_SELECTABLE;
      }
      else if (i <= 64) {
        polyline.edge_index = i - 1;
        polyline.flag |= NURB_BODY_EDGE_POLYLINE_FINAL | NURB_BODY_EDGE_POLYLINE_SELECTABLE;
      }
      r_polylines.append(std::move(polyline));
    }
  }
}

static Vector<TopoDS_Edge> find_selectable_surface_edges(
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces)
{
  Vector<TopoDS_Edge> edges;
  for (int i = 1; i <= edge_faces.Extent(); i++) {
    if (unique_edge_face_count(edge_faces.FindFromIndex(i)) < 2) {
      continue;
    }
    const TopoDS_Edge edge = TopoDS::Edge(edge_faces.FindKey(i));
    if (topological_edge_is_visible_outline(edge, edge_faces)) {
      edges.append(edge);
    }
  }
  return edges;
}

static TopoDS_Shape apply_boolean_operation(const TopoDS_Shape &base,
                                            const TopoDS_Shape &tool,
                                            const int operation)
{
  switch (operation) {
    case NURB_BODY_BOOLEAN_UNION:
      return BRepAlgoAPI_Fuse(base, tool).Shape();
    case NURB_BODY_BOOLEAN_INTERSECT:
      return BRepAlgoAPI_Common(base, tool).Shape();
    case NURB_BODY_BOOLEAN_DIFFERENCE:
    default:
      return BRepAlgoAPI_Cut(base, tool).Shape();
  }
}

static Vector<TopoDS_Edge> find_selectable_surface_edges(const TopoDS_Shape &shape)
{
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  return find_selectable_surface_edges(edge_faces);
}

static float bevel_radius_for_edge(const float fallback_radius,
                                   const uint64_t bevel_edges,
                                   const float *edge_radii,
                                   const int edge_index)
{
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(edge_index);
  if (edge_mask != 0 && bevel_edges != 0) {
    if ((bevel_edges & edge_mask) == 0) {
      return 0.0f;
    }
    return (edge_index < 64 && edge_radii != nullptr && edge_radii[edge_index] > 0.0f) ?
               edge_radii[edge_index] :
               0.0f;
  }
  if (edge_index >= 0 && edge_index < 64 && edge_radii != nullptr && edge_radii[edge_index] > 0.0f)
  {
    return edge_radii[edge_index];
  }
  return fallback_radius;
}

static void append_selectable_edge_refs(const Vector<TopoDS_Edge> &edges,
                                        const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces,
                                        const NurbBodyBooleanOp *op,
                                        const int flag,
                                        const int samples_per_edge,
                                        const float base_threshold,
                                        const uint64_t blended_edges,
                                        const float blend_radius,
                                        const float *blend_radii,
                                        Vector<NurbBodySelectableEdgeRef> &r_refs)
{
  for (const int i : edges.index_range()) {
    const uint64_t edge_mask = nurb_body_edge_mask_for_index(i);
    const float edge_blend_radius = bevel_radius_for_edge(
        blend_radius, blended_edges, blend_radii, i);
    const bool edge_is_blended = edge_blend_radius > 0.0f && edge_mask != 0 &&
                                 (blended_edges & edge_mask) != 0;
    if (edge_is_blended) {
      continue;
    }
    NurbBodySelectableEdgeRef ref;
    ref.op = op;
    ref.edge_index = i;
    ref.flag = flag;
    ref.threshold = base_threshold + std::max(0.0f, edge_blend_radius) * 2.0f;
    if ((append_tessellated_edge_polyline_local(edges[i], edge_faces, ref.points) ||
         append_edge_polyline_local(edges[i], samples_per_edge, ref.points)) &&
        ref.points.size() >= 2)
    {
      r_refs.append(std::move(ref));
    }
  }
}

static void append_selectable_edge_ref_polylines(const Span<NurbBodySelectableEdgeRef> selectable_refs,
                                                 Vector<NurbBodyEdgePolyline> &r_polylines)
{
  for (const NurbBodySelectableEdgeRef &ref : selectable_refs) {
    if (ref.points.size() < 2 || ref.edge_index < 0) {
      continue;
    }

    NurbBodyEdgePolyline hit_polyline;
    hit_polyline.op = ref.op;
    hit_polyline.edge_index = ref.edge_index;
    hit_polyline.flag = ref.flag | NURB_BODY_EDGE_POLYLINE_SELECTABLE;
    hit_polyline.points = ref.points;
    r_polylines.append(std::move(hit_polyline));
  }
}

static bool selected_edge_for_blend(const uint64_t bevel_edges,
                                    const uint64_t selected_edges,
                                    const int bevel_edge,
                                    const int selected_edge,
                                    const int edge_index,
                                    const int edges_num)
{
  if (edges_num <= 0) {
    return false;
  }
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(edge_index);
  if (edge_mask != 0) {
    if (bevel_edges != 0) {
      return (bevel_edges & edge_mask) != 0;
    }
    if (selected_edges != 0) {
      return (selected_edges & edge_mask) != 0;
    }
  }
  const int active_edge = bevel_edge >= 0 ? bevel_edge : selected_edge;
  if (active_edge < 0) {
    return edge_index == 0;
  }
  return edge_index == active_edge;
}

static int bevel_type_for_edge(const int fallback_type,
                               const uint64_t bevel_edges,
                               const uint64_t chamfer_edges,
                               const int bevel_edge,
                               const int edge_index)
{
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(edge_index);
  if (edge_mask != 0 && (bevel_edges & edge_mask) != 0) {
    return (chamfer_edges & edge_mask) != 0 ? NURB_BODY_BEVEL_CHAMFER :
                                             NURB_BODY_BEVEL_FILLET;
  }
  if (bevel_edges == 0 && bevel_edge == edge_index) {
    return fallback_type;
  }
  return NURB_BODY_BEVEL_FILLET;
}

struct NurbBodyBlendStep {
  int edge_index = -1;
  int order = 0;
  int bevel_type = NURB_BODY_BEVEL_FILLET;
  float radius = 0.0f;
};

static int bevel_order_for_edge(const int *bevel_order, const int edge_index)
{
  if (edge_index >= 0 && edge_index < 64 && bevel_order != nullptr && bevel_order[edge_index] > 0)
  {
    return bevel_order[edge_index];
  }
  return edge_index + 1;
}

static bool edge_radii_contains_positive(const float *edge_radii, const uint64_t edge_mask)
{
  if (edge_radii == nullptr) {
    return false;
  }
  for (int i = 0; i < 64; i++) {
    if ((edge_mask & nurb_body_edge_mask_for_index(i)) != 0 && edge_radii[i] > 0.0f) {
      return true;
    }
  }
  return false;
}

static bool blend_settings_may_change_shape(const float bevel_radius,
                                            const float *bevel_radii,
                                            const uint64_t bevel_edges,
                                            const uint64_t selected_edges,
                                            const int bevel_edge,
                                            const int selected_edge)
{
  if (bevel_edges != 0) {
    return bevel_radius > 0.0f || edge_radii_contains_positive(bevel_radii, bevel_edges);
  }

  if (bevel_radius > 0.0f) {
    return true;
  }

  if (selected_edges != 0) {
    return edge_radii_contains_positive(bevel_radii, selected_edges);
  }

  const int active_edge = bevel_edge >= 0 ? bevel_edge : selected_edge;
  const uint64_t active_edge_mask = active_edge >= 0 ? nurb_body_edge_mask_for_index(active_edge) :
                                                       nurb_body_edge_mask_for_index(0);
  return edge_radii_contains_positive(bevel_radii, active_edge_mask);
}

static Vector<NurbBodyBlendStep> sorted_blend_steps(const float bevel_radius,
                                                    const float *bevel_radii,
                                                    const int fallback_bevel_type,
                                                    const uint64_t bevel_edges,
                                                    const uint64_t chamfer_edges,
                                                    const uint64_t selected_edges,
                                                    const int bevel_edge,
                                                    const int selected_edge,
                                                    const int *bevel_order,
                                                    const int edges_num)
{
  Vector<NurbBodyBlendStep> steps;
  for (int i = 0; i < edges_num && i < 64; i++) {
    if (!selected_edge_for_blend(
            bevel_edges, selected_edges, bevel_edge, selected_edge, i, edges_num))
    {
      continue;
    }
    const float radius = bevel_radius_for_edge(bevel_radius, bevel_edges, bevel_radii, i);
    if (radius <= 0.0f) {
      continue;
    }

    NurbBodyBlendStep step;
    step.edge_index = i;
    step.order = bevel_order_for_edge(bevel_order, i);
    step.bevel_type = bevel_type_for_edge(
        fallback_bevel_type, bevel_edges, chamfer_edges, bevel_edge, i);
    step.radius = radius;
    steps.append(step);
  }

  std::sort(steps.begin(), steps.end(), [](const NurbBodyBlendStep &a,
                                           const NurbBodyBlendStep &b) {
    if (a.order == b.order) {
      return a.edge_index < b.edge_index;
    }
    return a.order < b.order;
  });
  return steps;
}

static Vector<Vector<float3>> reference_points_for_edges(const Vector<TopoDS_Edge> &edges)
{
  Vector<Vector<float3>> references;
  references.reserve(edges.size());
  for (const TopoDS_Edge &edge : edges) {
    Vector<float3> points;
    append_edge_polyline_local(edge, 48, points);
    references.append(std::move(points));
  }
  return references;
}

static int find_current_edge_by_reference(const Vector<TopoDS_Edge> &edges,
                                          const int fallback_index,
                                          const Span<float3> reference_points)
{
  if (reference_points.size() < 2) {
    return fallback_index >= 0 && fallback_index < edges.size() ? fallback_index : -1;
  }

  float reference_length = 0.0f;
  for (int i = 1; i < reference_points.size(); i++) {
    const float3 delta = reference_points[i] - reference_points[i - 1];
    reference_length += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  }
  const float threshold = std::max(reference_length * 0.02f, 0.001f);

  int best_index = -1;
  float best_dist_sq = FLT_MAX;
  for (const int i : edges.index_range()) {
    Vector<float3> points;
    if (!append_edge_polyline_local(edges[i], 48, points) || points.size() < 2) {
      continue;
    }

    float dist_sq = 0.0f;
    for (const float3 &point : reference_points) {
      dist_sq += dist_squared_to_polyline_v3(point, points);
    }
    dist_sq /= float(reference_points.size());
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_index = i;
    }
  }
  if (best_index != -1 && best_dist_sq <= threshold * threshold) {
    return best_index;
  }
  return -1;
}

static TopoDS_Shape apply_single_edge_blend(const TopoDS_Shape &shape,
                                            const float radius_limit,
                                            const float bevel_radius,
                                            const int bevel_type,
                                            const TopoDS_Edge &edge,
                                            bool &r_applied)
{
  r_applied = false;
  const double radius = std::min(double(bevel_radius),
                                 double(std::max(radius_limit, 0.001f)) * 0.95);
  if (radius <= 0.0) {
    return shape;
  }

  if (bevel_type == NURB_BODY_BEVEL_CHAMFER) {
    BRepFilletAPI_MakeChamfer chamfer(shape);
    chamfer.Add(radius, edge);
    chamfer.Build();
    if (chamfer.IsDone()) {
      r_applied = true;
      return chamfer.Shape();
    }
    return shape;
  }

  BRepFilletAPI_MakeFillet fillet(shape);
  fillet.Add(radius, edge);
  fillet.Build();
  if (fillet.IsDone()) {
    r_applied = true;
    return fillet.Shape();
  }
  return shape;
}

template<typename EdgeRefinder>
static TopoDS_Shape apply_edge_blend(const TopoDS_Shape &shape,
                                     const float radius_limit,
                                     const float bevel_radius,
                                     const float *bevel_radii,
                                     const int bevel_type,
                                     const uint64_t bevel_edges,
                                     const uint64_t chamfer_edges,
                                     const int *bevel_order,
                                     const uint64_t selected_edges,
                                     const int bevel_edge,
                                     const int selected_edge,
                                     const Vector<TopoDS_Edge> &edges,
                                     EdgeRefinder &&refind_edges)
{
  const Vector<NurbBodyBlendStep> steps = sorted_blend_steps(bevel_radius,
                                                             bevel_radii,
                                                             bevel_type,
                                                             bevel_edges,
                                                             chamfer_edges,
                                                             selected_edges,
                                                             bevel_edge,
                                                             selected_edge,
                                                             bevel_order,
                                                             edges.size());
  if (steps.is_empty()) {
    return shape;
  }

  TopoDS_Shape result = shape;
  Vector<TopoDS_Edge> current_edges = edges;
  Vector<Vector<float3>> edge_references;
  bool edge_references_ready = false;
  bool current_edges_match_input = true;
  for (const NurbBodyBlendStep &step : steps) {
    if (current_edges.is_empty() || step.edge_index < 0) {
      continue;
    }
    int current_edge = -1;
    if (current_edges_match_input && step.edge_index < current_edges.size()) {
      current_edge = step.edge_index;
    }
    else {
      if (!edge_references_ready) {
        edge_references = reference_points_for_edges(edges);
        edge_references_ready = true;
      }
    }
    if (current_edge < 0 && step.edge_index < edge_references.size()) {
      current_edge = find_current_edge_by_reference(current_edges,
                                                    step.edge_index,
                                                    edge_references[step.edge_index]);
    }
    if (current_edge < 0 && step.edge_index >= 0 && step.edge_index < current_edges.size()) {
      current_edge = step.edge_index;
    }
    if (current_edge < 0 || current_edge >= current_edges.size()) {
      continue;
    }

    bool applied = false;
    result = apply_single_edge_blend(
        result, radius_limit, step.radius, step.bevel_type, current_edges[current_edge], applied);
    if (applied) {
      current_edges = refind_edges(result);
      current_edges_match_input = false;
    }
  }
  return result;
}

static TopoDS_Shape evaluate_shape(const NurbBody &body, const Object *object)
{
  TopoDS_Shape result = make_body_primitive_shape(body);
  if (blend_settings_may_change_shape(body.bevel_radius,
                                      body.bevel_radii,
                                      body.bevel_edges,
                                      body.selected_edges,
                                      body.bevel_edge,
                                      body.selected_edge))
  {
    const Vector<TopoDS_Edge> body_edges = find_selectable_surface_edges(result);
    result = apply_edge_blend(result,
                              body.radius,
                              body.bevel_radius,
                              body.bevel_radii,
                              body.bevel_type,
                              body.bevel_edges,
                              body.chamfer_edges,
                              body.bevel_order,
                              body.selected_edges,
                              body.bevel_edge,
                              body.selected_edge,
                              body_edges,
                              [](const TopoDS_Shape &shape) {
                                return find_selectable_surface_edges(shape);
                              });
  }

  if (object != nullptr && !BLI_listbase_is_empty(&body.boolean_ops)) {
    for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(
             body.boolean_ops.first);
         op;
         op = op->next)
    {
      if (op->operand_radius <= 0.0f || op->operand_depth <= 0.0f) {
        continue;
      }

      const TopoDS_Shape tool = make_boolean_op_tool_shape(*op);
      result = apply_boolean_operation(result, tool, op->operation);
      if (blend_settings_may_change_shape(op->bevel_radius,
                                          op->bevel_radii,
                                          op->bevel_edges,
                                          op->selected_edges,
                                          op->bevel_edge,
                                          op->selected_edge))
      {
        const float operand_radius = std::max(op->operand_radius, 0.001f);
        const Vector<TopoDS_Edge> cut_edges = find_cut_edges(result, body.radius, operand_radius);
        result = apply_edge_blend(result,
                                  operand_radius,
                                  op->bevel_radius,
                                  op->bevel_radii,
                                  op->bevel_type,
                                  op->bevel_edges,
                                  op->chamfer_edges,
                                  op->bevel_order,
                                  op->selected_edges,
                                  op->bevel_edge,
                                  op->selected_edge,
                                  cut_edges,
                                  [&](const TopoDS_Shape &shape) {
                                    return find_cut_edges(shape, body.radius, operand_radius);
                                  });
      }
    }
  }

  if (blend_settings_may_change_shape(body.surface_bevel_radius,
                                      body.surface_bevel_radii,
                                      body.surface_bevel_edges,
                                      body.surface_selected_edges,
                                      body.surface_bevel_edge,
                                      body.surface_selected_edge))
  {
    const Vector<TopoDS_Edge> surface_edges = find_selectable_surface_edges(result);
    result = apply_edge_blend(result,
                              std::max(body.radius, 0.001f),
                              body.surface_bevel_radius,
                              body.surface_bevel_radii,
                              body.surface_bevel_type,
                              body.surface_bevel_edges,
                              body.surface_chamfer_edges,
                              body.surface_bevel_order,
                              body.surface_selected_edges,
                              body.surface_bevel_edge,
                              body.surface_selected_edge,
                              surface_edges,
                              [](const TopoDS_Shape &shape) {
                                return find_selectable_surface_edges(shape);
                              });
  }

  return result;
}

static void sample_boolean_edge_polylines(const NurbBody &body,
                                          const int samples_per_edge,
                                          Vector<NurbBodyEdgePolyline> &r_polylines)
{
  TopoDS_Shape result = make_body_primitive_shape(body);
  r_polylines.clear();
  Vector<NurbBodySelectableEdgeRef> selectable_refs;
  const float base_threshold = std::max(0.05f, body.tessellation_deflection * 8.0f);

  triangulate_shape_for_preview(result, body);
  TopTools_IndexedDataMapOfShapeListOfShape body_edge_faces;
  TopExp::MapShapesAndAncestors(result, TopAbs_EDGE, TopAbs_FACE, body_edge_faces);
  const Vector<TopoDS_Edge> body_edges = find_selectable_surface_edges(body_edge_faces);
  const uint64_t body_blended_edges = body.bevel_edges != 0 ?
                                          body.bevel_edges :
                                          nurb_body_edge_mask_for_index(body.bevel_edge);
  append_selectable_edge_refs(body_edges,
                              body_edge_faces,
                              nullptr,
                              NURB_BODY_EDGE_POLYLINE_BODY,
                              samples_per_edge,
                              base_threshold,
                              body_blended_edges,
                              body.bevel_radius,
                              body.bevel_radii,
                              selectable_refs);
  result = apply_edge_blend(result,
                            body.radius,
                            body.bevel_radius,
                            body.bevel_radii,
                            body.bevel_type,
                            body.bevel_edges,
                            body.chamfer_edges,
                            body.bevel_order,
                            body.selected_edges,
                            body.bevel_edge,
                            body.selected_edge,
                            body_edges,
                            [](const TopoDS_Shape &shape) {
                              return find_selectable_surface_edges(shape);
                            });

  for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(body.boolean_ops.first);
       op;
       op = op->next)
  {
    if (op->operand_radius <= 0.0f || op->operand_depth <= 0.0f) {
      continue;
    }

    const TopoDS_Shape tool = make_boolean_op_tool_shape(*op);
    result = apply_boolean_operation(result, tool, op->operation);
    triangulate_shape_for_preview(result, body);

    const float operand_radius = std::max(op->operand_radius, 0.001f);
    TopTools_IndexedDataMapOfShapeListOfShape op_edge_faces;
    TopExp::MapShapesAndAncestors(result, TopAbs_EDGE, TopAbs_FACE, op_edge_faces);
    const Vector<TopoDS_Edge> cut_edges = find_cut_edges(op_edge_faces, body.radius, operand_radius);
    const uint64_t op_blended_edges = op->bevel_edges != 0 ?
                                          op->bevel_edges :
                                          nurb_body_edge_mask_for_index(op->bevel_edge);
    append_selectable_edge_refs(cut_edges,
                                op_edge_faces,
                                op,
                                0,
                                samples_per_edge,
                                base_threshold,
                                op_blended_edges,
                                op->bevel_radius,
                                op->bevel_radii,
                                selectable_refs);

    result = apply_edge_blend(result,
                              operand_radius,
                              op->bevel_radius,
                              op->bevel_radii,
                              op->bevel_type,
                              op->bevel_edges,
                              op->chamfer_edges,
                              op->bevel_order,
                              op->selected_edges,
                              op->bevel_edge,
                              op->selected_edge,
                              cut_edges,
                              [&](const TopoDS_Shape &shape) {
                                return find_cut_edges(shape, body.radius, operand_radius);
                              });
  }

  triangulate_shape_for_preview(result, body);
  TopTools_IndexedDataMapOfShapeListOfShape surface_edge_faces;
  TopExp::MapShapesAndAncestors(result, TopAbs_EDGE, TopAbs_FACE, surface_edge_faces);
  const Vector<TopoDS_Edge> surface_edges = find_selectable_surface_edges(surface_edge_faces);
  const uint64_t surface_blended_edges = body.surface_bevel_edges != 0 ?
                                             body.surface_bevel_edges :
                                             nurb_body_edge_mask_for_index(body.surface_bevel_edge);
  append_selectable_edge_refs(surface_edges,
                              surface_edge_faces,
                              nullptr,
                              NURB_BODY_EDGE_POLYLINE_FINAL,
                              samples_per_edge,
                              base_threshold,
                              surface_blended_edges,
                              body.surface_bevel_radius,
                              body.surface_bevel_radii,
                              selectable_refs);
  result = apply_edge_blend(result,
                            std::max(body.radius, 0.001f),
                            body.surface_bevel_radius,
                            body.surface_bevel_radii,
                            body.surface_bevel_type,
                            body.surface_bevel_edges,
                            body.surface_chamfer_edges,
                            body.surface_bevel_order,
                            body.surface_selected_edges,
                            body.surface_bevel_edge,
                            body.surface_selected_edge,
                            surface_edges,
                            [](const TopoDS_Shape &shape) {
                              return find_selectable_surface_edges(shape);
                            });

  triangulate_shape_for_preview(result, body);
  append_shape_surface_edge_polylines(result, samples_per_edge, selectable_refs, r_polylines);
  append_selectable_edge_ref_polylines(selectable_refs, r_polylines);
}

static int64_t edge_key(const int v1, const int v2)
{
  const uint64_t a = uint64_t(std::min(v1, v2));
  const uint64_t b = uint64_t(std::max(v1, v2));
  return int64_t((a << 32) | b);
}

struct NurbBodyMeshFace {
  int verts[4] = {};
  float3 normals[4] = {};
  int len = 0;
};

struct NurbBodyMeshTriangle {
  int3 verts = {};
  float3 normals[3] = {};
  int face_index = -1;
};

static void append_triangle_mesh_face(Vector<NurbBodyMeshFace> &faces,
                                      const NurbBodyMeshTriangle &tri)
{
  NurbBodyMeshFace face;
  face.verts[0] = tri.verts[0];
  face.verts[1] = tri.verts[1];
  face.verts[2] = tri.verts[2];
  face.normals[0] = tri.normals[0];
  face.normals[1] = tri.normals[1];
  face.normals[2] = tri.normals[2];
  face.len = 3;
  faces.append(face);
}

static bool same_unordered_edge(const int a1, const int a2, const int b1, const int b2)
{
  return (a1 == b1 && a2 == b2) || (a1 == b2 && a2 == b1);
}

static bool triangle_contains_vertex(const int3 &tri, const int vertex)
{
  return tri[0] == vertex || tri[1] == vertex || tri[2] == vertex;
}

static float3 normal_for_triangle_vertex(const NurbBodyMeshTriangle &tri, const int vertex)
{
  float3 normal(0.0f);
  int count = 0;
  for (int i = 0; i < 3; i++) {
    if (tri.verts[i] == vertex) {
      normal += tri.normals[i];
      count++;
    }
  }
  if (count > 1) {
    normal *= 1.0f / float(count);
  }
  if (normalize_v3(normal) == 0.0f) {
    normal = float3(0.0f, 0.0f, 1.0f);
  }
  return normal;
}

static float3 normal_for_quad_vertex(const NurbBodyMeshTriangle &tri_a,
                                     const NurbBodyMeshTriangle &tri_b,
                                     const int vertex)
{
  float3 normal(0.0f);
  int count = 0;
  for (int i = 0; i < 3; i++) {
    if (tri_a.verts[i] == vertex) {
      normal += tri_a.normals[i];
      count++;
    }
    if (tri_b.verts[i] == vertex) {
      normal += tri_b.normals[i];
      count++;
    }
  }
  if (count > 1) {
    normal *= 1.0f / float(count);
  }
  if (normalize_v3(normal) == 0.0f) {
    return normal_for_triangle_vertex(tri_a, vertex);
  }
  return normal;
}

static bool try_make_quad_from_tri_pair(const NurbBodyMeshTriangle &mesh_tri_a,
                                        const NurbBodyMeshTriangle &mesh_tri_b,
                                        const Span<float3> positions,
                                        NurbBodyMeshFace &r_face)
{
  const int3 &tri_a = mesh_tri_a.verts;
  const int3 &tri_b = mesh_tri_b.verts;
  int shared[2] = {};
  int shared_num = 0;
  for (int i = 0; i < 3; i++) {
    const int vertex = tri_a[i];
    if (triangle_contains_vertex(tri_b, vertex)) {
      if (shared_num == 2) {
        return false;
      }
      shared[shared_num++] = vertex;
    }
  }
  if (shared_num != 2) {
    return false;
  }

  const int2 tri_edges[6] = {
      int2(tri_a[0], tri_a[1]),
      int2(tri_a[1], tri_a[2]),
      int2(tri_a[2], tri_a[0]),
      int2(tri_b[0], tri_b[1]),
      int2(tri_b[1], tri_b[2]),
      int2(tri_b[2], tri_b[0]),
  };

  int2 boundary_edges[4];
  int boundary_num = 0;
  for (const int2 &edge : tri_edges) {
    if (same_unordered_edge(edge[0], edge[1], shared[0], shared[1])) {
      continue;
    }
    if (boundary_num == 4) {
      return false;
    }
    boundary_edges[boundary_num++] = edge;
  }
  if (boundary_num != 4) {
    return false;
  }

  int loop[4] = {boundary_edges[0][0], boundary_edges[0][1], -1, -1};
  bool used_edges[4] = {true, false, false, false};
  int current = loop[1];
  for (int step = 2; step < 4; step++) {
    bool found = false;
    for (int edge_i = 1; edge_i < 4; edge_i++) {
      if (used_edges[edge_i]) {
        continue;
      }
      const int2 &edge = boundary_edges[edge_i];
      if (edge[0] == current || edge[1] == current) {
        loop[step] = (edge[0] == current) ? edge[1] : edge[0];
        current = loop[step];
        used_edges[edge_i] = true;
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }

  bool closes = false;
  for (int edge_i = 1; edge_i < 4; edge_i++) {
    if (!used_edges[edge_i]) {
      const int2 &edge = boundary_edges[edge_i];
      closes = same_unordered_edge(edge[0], edge[1], current, loop[0]);
      break;
    }
  }
  if (!closes) {
    return false;
  }

  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (loop[i] == loop[j]) {
        return false;
      }
    }
  }

  float tri_no[3];
  float quad_no[3];
  if (normal_tri_v3(tri_no, positions[tri_a[0]], positions[tri_a[1]], positions[tri_a[2]]) ==
      0.0f)
  {
    return false;
  }
  if (normal_quad_v3(
          quad_no, positions[loop[0]], positions[loop[1]], positions[loop[2]], positions[loop[3]]) ==
      0.0f)
  {
    return false;
  }
  if (dot_v3v3(tri_no, quad_no) < 0.0f) {
    std::swap(loop[1], loop[3]);
  }
  if (!is_quad_convex_v3(
          positions[loop[0]], positions[loop[1]], positions[loop[2]], positions[loop[3]]))
  {
    return false;
  }

  r_face.len = 4;
  r_face.verts[0] = loop[0];
  r_face.verts[1] = loop[1];
  r_face.verts[2] = loop[2];
  r_face.verts[3] = loop[3];
  for (int i = 0; i < 4; i++) {
    r_face.normals[i] = normal_for_quad_vertex(mesh_tri_a, mesh_tri_b, loop[i]);
  }
  return true;
}

static void build_mesh_faces_from_triangles(const Span<NurbBodyMeshTriangle> triangles,
                                            const Span<float3> positions,
                                            const bool triangulate_mesh,
                                            Vector<NurbBodyMeshFace> &r_faces)
{
  if (triangulate_mesh) {
    for (const NurbBodyMeshTriangle &tri : triangles) {
      append_triangle_mesh_face(r_faces, tri);
    }
    return;
  }

  Array<bool> used(triangles.size(), false);
  Array<int64_t> paired_triangles(triangles.size(), -1);
  Map<std::pair<int, int64_t>, int64_t> triangle_by_face_edge;
  triangle_by_face_edge.reserve(triangles.size() * 3);

  for (const int64_t i : triangles.index_range()) {
    const NurbBodyMeshTriangle &tri = triangles[i];
    for (int edge_i = 0; edge_i < 3 && paired_triangles[i] == -1; edge_i++) {
      const int v1 = tri.verts[edge_i];
      const int v2 = tri.verts[(edge_i + 1) % 3];
      const std::pair<int, int64_t> key(tri.face_index, edge_key(v1, v2));
      if (const int64_t *other_tri = triangle_by_face_edge.lookup_ptr(key)) {
        if (paired_triangles[*other_tri] == -1) {
          NurbBodyMeshFace quad;
          if (try_make_quad_from_tri_pair(triangles[*other_tri], tri, positions, quad)) {
            paired_triangles[*other_tri] = i;
            paired_triangles[i] = *other_tri;
          }
        }
      }
      else {
        triangle_by_face_edge.add(key, i);
      }
    }
  }

  for (const int64_t i : triangles.index_range()) {
    if (used[i]) {
      continue;
    }
    const int64_t paired_tri = paired_triangles[i];
    if (paired_tri > i) {
      NurbBodyMeshFace quad;
      if (try_make_quad_from_tri_pair(triangles[i], triangles[paired_tri], positions, quad)) {
        used[i] = true;
        used[paired_tri] = true;
        r_faces.append(quad);
        continue;
      }
    }
    if (paired_tri < i && paired_tri >= 0) {
      used[i] = true;
      continue;
    }
    used[i] = true;
    append_triangle_mesh_face(r_faces, triangles[i]);
  }
}

static void write_custom_corner_normals(Mesh &mesh, const Span<NurbBodyMeshFace> faces)
{
  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  bke::SpanAttributeWriter<float3> custom_normals =
      attributes.lookup_or_add_for_write_only_span<float3>("custom_normal",
                                                           bke::AttrDomain::Corner);
  if (!custom_normals) {
    return;
  }

  int corner_index = 0;
  for (const NurbBodyMeshFace &face : faces) {
    for (int corner = 0; corner < face.len; corner++) {
      custom_normals.span[corner_index++] = face.normals[corner];
    }
  }
  custom_normals.finish();
  mesh.tag_custom_normals_changed();
}

static float3 normal_from_triangulation_node(const Handle(Poly_Triangulation) &triangulation,
                                             const int node_index,
                                             const gp_Trsf &transform,
                                             const bool reverse_face)
{
  if (!triangulation->HasNormals()) {
    return float3(0.0f);
  }

  const gp_Dir node_normal = triangulation->Normal(node_index);
  gp_Vec normal(node_normal.X(), node_normal.Y(), node_normal.Z());
  normal.Transform(transform);

  float3 result(float(normal.X()), float(normal.Y()), float(normal.Z()));
  if (reverse_face) {
    result *= -1.0f;
  }
  if (normalize_v3(result) == 0.0f) {
    return float3(0.0f);
  }
  return result;
}

static float3 fallback_triangle_normal(const Span<float3> node_positions,
                                       const int3 &source_nodes,
                                       const bool reverse_face)
{
  float3 normal;
  if (reverse_face) {
    normal_tri_v3(normal,
                  node_positions[source_nodes[0] - 1],
                  node_positions[source_nodes[2] - 1],
                  node_positions[source_nodes[1] - 1]);
  }
  else {
    normal_tri_v3(normal,
                  node_positions[source_nodes[0] - 1],
                  node_positions[source_nodes[1] - 1],
                  node_positions[source_nodes[2] - 1]);
  }
  if (normalize_v3(normal) == 0.0f) {
    normal = float3(0.0f, 0.0f, 1.0f);
  }
  return normal;
}

static Mesh *mesh_from_shape(const TopoDS_Shape &shape, const NurbBody &body)
{
  triangulate_shape_for_preview(shape, body);

  const bool triangulate_mesh = (body.flag & NURB_BODY_TRIANGULATE_MESH) != 0;
  const bool use_smooth_shading = (body.flag & NURB_BODY_SMOOTH_SHADING) != 0;
  Vector<float3> positions;
  Vector<NurbBodyMeshTriangle> triangles;

  int estimated_nodes = 0;
  int estimated_triangles = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
    if (!triangulation.IsNull()) {
      estimated_nodes += triangulation->NbNodes();
      estimated_triangles += triangulation->NbTriangles();
    }
  }
  positions.reserve(estimated_nodes);
  triangles.reserve(estimated_triangles);

  auto add_position = [&](const float3 &position) -> int {
    const int index = int(positions.size());
    positions.append(position);
    return index;
  };

  int shape_face_index = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next(), shape_face_index++)
  {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull()) {
      continue;
    }

    const gp_Trsf transform = location.Transformation();
    const bool reverse_face = face.Orientation() == TopAbs_REVERSED;
    if (use_smooth_shading && !triangulation->HasNormals()) {
      BRepLib_ToolTriangulatedShape::ComputeNormals(face, triangulation);
    }

    Vector<int> node_indices;
    Vector<float3> node_positions;
    Vector<float3> node_normals;
    node_indices.reserve(triangulation->NbNodes());
    node_positions.reserve(triangulation->NbNodes());
    node_normals.reserve(triangulation->NbNodes());
    for (int i = 1; i <= triangulation->NbNodes(); i++) {
      gp_Pnt point = triangulation->Node(i);
      point.Transform(transform);
      const float3 position(float(point.X()), float(point.Y()), float(point.Z()));
      node_positions.append(position);
      node_indices.append(add_position(position));
      node_normals.append(use_smooth_shading ?
                              normal_from_triangulation_node(
                                  triangulation, i, transform, reverse_face) :
                              float3(0.0f));
    }

    for (int i = 1; i <= triangulation->NbTriangles(); i++) {
      int v1, v2, v3;
      triangulation->Triangle(i).Get(v1, v2, v3);
      int3 tri_verts(node_indices[v1 - 1], node_indices[v2 - 1], node_indices[v3 - 1]);
      if (tri_verts[0] == tri_verts[1] || tri_verts[0] == tri_verts[2] ||
          tri_verts[1] == tri_verts[2])
      {
        continue;
      }
      float3 tri_normals[3] = {
          node_normals[v1 - 1],
          node_normals[v2 - 1],
          node_normals[v3 - 1],
      };
      if (reverse_face) {
        std::swap(tri_verts[1], tri_verts[2]);
        std::swap(tri_normals[1], tri_normals[2]);
      }
      if (use_smooth_shading &&
          (is_zero_v3(tri_normals[0]) || is_zero_v3(tri_normals[1]) ||
           is_zero_v3(tri_normals[2])))
      {
        const float3 fallback_normal = fallback_triangle_normal(
            node_positions, int3(v1, v2, v3), reverse_face);
        for (int corner = 0; corner < 3; corner++) {
          if (is_zero_v3(tri_normals[corner])) {
            tri_normals[corner] = fallback_normal;
          }
        }
      }

      NurbBodyMeshTriangle mesh_triangle;
      mesh_triangle.verts = tri_verts;
      mesh_triangle.normals[0] = tri_normals[0];
      mesh_triangle.normals[1] = tri_normals[1];
      mesh_triangle.normals[2] = tri_normals[2];
      mesh_triangle.face_index = shape_face_index;
      triangles.append(mesh_triangle);
    }
  }

  const Span<float3> final_positions = positions.as_span();
  Vector<NurbBodyMeshFace> faces;
  build_mesh_faces_from_triangles(triangles, final_positions, triangulate_mesh, faces);

  Map<int64_t, int> edge_index_by_key;
  Vector<int2> edges;
  Vector<int> face_offsets_data;
  Vector<int> corner_verts;
  Vector<int> corner_edges;
  face_offsets_data.reserve(faces.size() + 1);
  corner_verts.reserve(faces.size() * 4);
  corner_edges.reserve(faces.size() * 4);

  auto edge_index = [&](const int v1, const int v2) {
    const int64_t key = edge_key(v1, v2);
    return edge_index_by_key.lookup_or_add_cb(key, [&]() {
      const int index = edges.size();
      edges.append(int2(v1, v2));
      return index;
    });
  };

  face_offsets_data.append(0);
  for (const NurbBodyMeshFace &face : faces) {
    for (int corner = 0; corner < face.len; corner++) {
      const int v1 = face.verts[corner];
      const int v2 = face.verts[(corner + 1) % face.len];
      corner_verts.append(v1);
      corner_edges.append(edge_index(v1, v2));
    }
    face_offsets_data.append(corner_verts.size());
  }

  Mesh *mesh = BKE_mesh_new_nomain(
      final_positions.size(), edges.size(), faces.size(), corner_verts.size());
  mesh->vert_positions_for_write().copy_from(final_positions);
  mesh->edges_for_write().copy_from(edges);

  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  MutableSpan<int> mesh_corner_verts = mesh->corner_verts_for_write();
  MutableSpan<int> mesh_corner_edges = mesh->corner_edges_for_write();

  face_offsets.copy_from(face_offsets_data);
  mesh_corner_verts.copy_from(corner_verts);
  mesh_corner_edges.copy_from(corner_edges);

  bke::mesh_smooth_set(*mesh, use_smooth_shading);
  if (use_smooth_shading) {
    write_custom_corner_normals(*mesh, faces);
  }
  mesh->tag_loose_verts_none();
  mesh->tag_loose_edges_none();
  mesh->tag_overlapping_none();
  mesh->tag_topology_changed();

  return mesh;
}

static void nurb_body_hash_bytes(uint64_t &hash, const void *data, const size_t size)
{
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < size; i++) {
    hash ^= uint64_t(bytes[i]);
    hash *= 1099511628211ull;
  }
}

template<typename T> static void nurb_body_hash_value(uint64_t &hash, const T &value)
{
  nurb_body_hash_bytes(hash, &value, sizeof(T));
}

static uint64_t nurb_body_edge_polyline_cache_key(const NurbBody &body,
                                                  const Object & /*object*/,
                                                  const int samples_per_edge)
{
  uint64_t hash = 1469598103934665603ull;
  nurb_body_hash_value(hash, samples_per_edge);
  nurb_body_hash_value(hash, body.primitive);
  nurb_body_hash_value(hash, body.radius);
  nurb_body_hash_value(hash, body.depth);
  nurb_body_hash_value(hash, body.tessellation_deflection);
  nurb_body_hash_value(hash, body.tessellation_angle);
  nurb_body_hash_value(hash, body.bevel_edge);
  nurb_body_hash_value(hash, body.bevel_edges);
  nurb_body_hash_value(hash, body.chamfer_edges);
  nurb_body_hash_value(hash, body.surface_bevel_edge);
  nurb_body_hash_value(hash, body.surface_bevel_edges);
  nurb_body_hash_value(hash, body.surface_chamfer_edges);
  nurb_body_hash_value(hash, body.bevel_type);
  nurb_body_hash_value(hash, body.surface_bevel_type);
  nurb_body_hash_value(hash, body.bevel_radius);
  nurb_body_hash_value(hash, body.surface_bevel_radius);
  nurb_body_hash_bytes(hash, body.bevel_radii, sizeof(body.bevel_radii));
  nurb_body_hash_bytes(hash, body.surface_bevel_radii, sizeof(body.surface_bevel_radii));
  nurb_body_hash_bytes(hash, body.bevel_order, sizeof(body.bevel_order));
  nurb_body_hash_bytes(hash, body.surface_bevel_order, sizeof(body.surface_bevel_order));
  if (body.bevel_radius > 0.0f && body.bevel_edges == 0 && body.bevel_edge < 0) {
    nurb_body_hash_value(hash, body.selected_edges);
    nurb_body_hash_value(hash, body.selected_edge);
  }
  if (body.surface_bevel_radius > 0.0f && body.surface_bevel_edges == 0 &&
      body.surface_bevel_edge < 0)
  {
    nurb_body_hash_value(hash, body.surface_selected_edges);
    nurb_body_hash_value(hash, body.surface_selected_edge);
  }

  int op_count = 0;
  for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(body.boolean_ops.first);
       op;
       op = op->next)
  {
    op_count++;
    nurb_body_hash_value(hash, op->operation);
    nurb_body_hash_value(hash, op->primitive);
    nurb_body_hash_value(hash, op->bevel_edge);
    nurb_body_hash_value(hash, op->bevel_edges);
    nurb_body_hash_value(hash, op->chamfer_edges);
    nurb_body_hash_value(hash, op->bevel_type);
    nurb_body_hash_value(hash, op->bevel_radius);
    nurb_body_hash_bytes(hash, op->bevel_radii, sizeof(op->bevel_radii));
    nurb_body_hash_bytes(hash, op->bevel_order, sizeof(op->bevel_order));
    if (op->bevel_radius > 0.0f && op->bevel_edges == 0 && op->bevel_edge < 0) {
      nurb_body_hash_value(hash, op->selected_edges);
      nurb_body_hash_value(hash, op->selected_edge);
    }
    nurb_body_hash_value(hash, op->operand_radius);
    nurb_body_hash_value(hash, op->operand_depth);
    nurb_body_hash_bytes(hash, op->operand_to_target, sizeof(op->operand_to_target));
  }
  nurb_body_hash_value(hash, op_count);
  return hash;
}

struct NurbBodyEdgePolylineCache {
  const Object *object = nullptr;
  uint64_t key = 0;
  Vector<NurbBodyEdgePolyline> polylines;
};

static Vector<NurbBodyEdgePolylineCache> g_nurb_body_edge_polyline_caches;

static NurbBodyEdgePolylineCache *nurb_body_edge_polyline_cache_find(const Object *object)
{
  for (NurbBodyEdgePolylineCache &cache : g_nurb_body_edge_polyline_caches) {
    if (cache.object == object) {
      return &cache;
    }
  }
  return nullptr;
}

static NurbBodyEdgePolylineCache &nurb_body_edge_polyline_cache_ensure(const Object *object)
{
  if (NurbBodyEdgePolylineCache *cache = nurb_body_edge_polyline_cache_find(object)) {
    return *cache;
  }

  constexpr int max_cached_objects = 64;
  if (g_nurb_body_edge_polyline_caches.size() >= max_cached_objects) {
    g_nurb_body_edge_polyline_caches.remove_and_reorder(0);
  }

  NurbBodyEdgePolylineCache cache;
  cache.object = object;
  g_nurb_body_edge_polyline_caches.append(std::move(cache));
  return g_nurb_body_edge_polyline_caches.last();
}

#endif

Mesh *BKE_nurb_body_to_mesh(const NurbBody *body, const Object *object)
{
#ifdef WITH_OPENCASCADE
  try {
    return mesh_from_shape(evaluate_shape(*body, object), *body);
  }
  catch (Standard_Failure const &) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
#else
  UNUSED_VARS(body);
  return BKE_mesh_new_nomain(0, 0, 0, 0);
#endif
}

void BKE_nurb_body_boolean_edge_polylines(const Object *object,
                                          Vector<NurbBodyEdgePolyline> &r_polylines,
                                          const int samples_per_edge)
{
  r_polylines.clear();
  const Span<NurbBodyEdgePolyline> cached_polylines =
      BKE_nurb_body_boolean_edge_polylines_cached(object, samples_per_edge);
  r_polylines.extend(cached_polylines.data(), cached_polylines.size());
}

Span<NurbBodyEdgePolyline> BKE_nurb_body_boolean_edge_polylines_cached(
    const Object *object, const int samples_per_edge)
{
#ifdef WITH_OPENCASCADE
  if (object == nullptr || object->type != OB_NURB_BODY || object->data == nullptr) {
    return {};
  }

  const NurbBody *body = id_cast<const NurbBody *>(object->data);
  const uint64_t cache_key = nurb_body_edge_polyline_cache_key(
      *body, *object, samples_per_edge);
  NurbBodyEdgePolylineCache &cache = nurb_body_edge_polyline_cache_ensure(object);
  if (cache.key == cache_key) {
    return cache.polylines.as_span();
  }

  try {
    cache.polylines.clear();
    sample_boolean_edge_polylines(*body, samples_per_edge, cache.polylines);
    cache.key = cache_key;
    return cache.polylines.as_span();
  }
  catch (Standard_Failure const &) {
    cache.key = 0;
    cache.polylines.clear();
    return {};
  }
#else
  UNUSED_VARS(object, samples_per_edge);
  return {};
#endif
}

uint64_t BKE_nurb_body_boolean_edge_polylines_cache_key(const Object *object,
                                                        const int samples_per_edge)
{
#ifdef WITH_OPENCASCADE
  if (object == nullptr || object->type != OB_NURB_BODY || object->data == nullptr) {
    return 0;
  }

  const NurbBody *body = id_cast<const NurbBody *>(object->data);
  return nurb_body_edge_polyline_cache_key(*body, *object, samples_per_edge);
#else
  UNUSED_VARS(object, samples_per_edge);
  return 0;
#endif
}

void BKE_nurb_body_data_update(Depsgraph * /*depsgraph*/, Scene * /*scene*/, Object *ob)
{
  NurbBody *body = id_cast<NurbBody *>(ob->data);
  Mesh *mesh = BKE_nurb_body_to_mesh(body, ob);
  BKE_object_eval_assign_data(ob, &mesh->id, true);

  bke::GeometrySet *geometry_set = new bke::GeometrySet();
  geometry_set->replace_mesh(mesh, bke::GeometryOwnershipType::Editable);
  ob->runtime->geometry_set_eval = geometry_set;
}

}  // namespace blender
