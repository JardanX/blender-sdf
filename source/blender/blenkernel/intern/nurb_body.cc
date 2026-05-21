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
#include <cstdio>
#include <cstdlib>
#include <utility>

#include "MEM_guardedalloc.h"

#include "DNA_material_types.h"
#include "DNA_nurb_body_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BLI_array.hh"
#include "BLI_listbase.h"
#include "BLI_map.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_time.h"
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
#  include <BOPAlgo_Operation.hxx>
#  include <BRepBuilderAPI_GTransform.hxx>
#  include <BRepBuilderAPI_Transform.hxx>
#  include <BRep_Tool.hxx>
#  include <BRepAlgoAPI_Common.hxx>
#  include <BRepAlgoAPI_Cut.hxx>
#  include <BRepAlgoAPI_Fuse.hxx>
#  include <BRepFilletAPI_MakeChamfer.hxx>
#  include <BRepFilletAPI_MakeFillet.hxx>
#  include <BRepLib.hxx>
#  include <BRepLib_ToolTriangulatedShape.hxx>
#  include <BRepMesh_IncrementalMesh.hxx>
#  include <BRepPrimAPI_MakeBox.hxx>
#  include <BRepPrimAPI_MakeCone.hxx>
#  include <BRepPrimAPI_MakeCylinder.hxx>
#  include <BRepPrimAPI_MakeSphere.hxx>
#  include <BRepPrimAPI_MakeTorus.hxx>
#  include <BRepPrimAPI_MakeWedge.hxx>
#  include <BRepTools.hxx>
#  include <ChFi3d_FilletShape.hxx>
#  include <Geom_Circle.hxx>
#  include <Geom_Curve.hxx>
#  include <Geom_Line.hxx>
#  include <Geom_Surface.hxx>
#  include <Poly_PolygonOnTriangulation.hxx>
#  include <Poly_Triangle.hxx>
#  include <Poly_Triangulation.hxx>
#  include <Standard_Failure.hxx>
#  include <NCollection_List.hxx>
#  include <TopExp.hxx>
#  include <TopExp_Explorer.hxx>
#  include <TopAbs_Orientation.hxx>
#  include <TopLoc_Location.hxx>
#  include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#  include <TopTools_ListOfShape.hxx>
#  include <TopoDS.hxx>
#  include <TopoDS_Edge.hxx>
#  include <TopoDS_Face.hxx>
#  include <TopoDS_Shape.hxx>
#  include <TopoDS_Vertex.hxx>
#  include <gp_Ax2.hxx>
#  include <gp_Dir.hxx>
#  include <gp_GTrsf.hxx>
#  include <gp_Pnt.hxx>
#  include <gp_Trsf.hxx>
#  include <gp_Vec.hxx>
#endif

namespace blender {

static uint64_t nurb_body_edge_mask_for_index(const int edge_index)
{
  return (edge_index >= 0 && edge_index < 64) ? (uint64_t(1) << uint(edge_index)) : uint64_t(0);
}

static bool nurb_body_primitive_is_valid(const int primitive)
{
  return ELEM(primitive,
              NURB_BODY_PRIMITIVE_BOX,
              NURB_BODY_PRIMITIVE_SPHERE,
              NURB_BODY_PRIMITIVE_CYLINDER,
              NURB_BODY_PRIMITIVE_CONE,
              NURB_BODY_PRIMITIVE_TORUS,
              NURB_BODY_PRIMITIVE_WEDGE);
}

static void nurb_body_sanitize_dimensions(float dimensions[3])
{
  for (int i = 0; i < 3; i++) {
    if (dimensions[i] <= 0.0f) {
      dimensions[i] = 2.0f;
    }
  }
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

static void nurb_body_clear_selection_state(NurbBody &body)
{
  body.selected_edges = 0;
  body.selected_edge = -1;
  body.hovered_edge = -1;
  body.surface_selected_edges = 0;
  body.surface_selected_edge = -1;
  body.surface_hovered_edge = -1;
  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first); op;
       op = op->next)
  {
    op->selected_edges = 0;
    op->selected_edge = -1;
    op->hovered_edge = -1;
    op->flag &= ~(NURB_BODY_BOOLEAN_OP_SELECTED | NURB_BODY_BOOLEAN_OP_HOVERED);
  }
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
  nurb_body_clear_selection_state(*body_dst);
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
  body->flag &= ~(NURB_BODY_MERGE_VERTICES | NURB_BODY_AUTO_CREASE_SHARP_EDGES |
                  NURB_BODY_FAST_BEVEL_PREVIEW);
  if (!nurb_body_primitive_is_valid(body->primitive)) {
    body->primitive = NURB_BODY_PRIMITIVE_CYLINDER;
  }
  if (body->minor_radius <= 0.0f) {
    body->minor_radius = 0.25f;
  }
  nurb_body_sanitize_dimensions(body->dimensions);
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
  for (int i = 0; i < 64; i++) {
    if (((body->surface_bevel_edges | body->surface_selected_edges) &
         nurb_body_edge_mask_for_index(i)) == 0)
    {
      body->surface_edge_keys[i] = 0;
    }
  }
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
    if (!nurb_body_primitive_is_valid(op->primitive)) {
      op->primitive = NURB_BODY_PRIMITIVE_CYLINDER;
    }
    if (op->operand_minor_radius <= 0.0f) {
      op->operand_minor_radius = 0.25f;
    }
    nurb_body_sanitize_dimensions(op->operand_dimensions);
    if (op->operand_scale[0] == 0.0f && op->operand_scale[1] == 0.0f &&
        op->operand_scale[2] == 0.0f)
    {
      op->operand_scale[0] = 1.0f;
      op->operand_scale[1] = 1.0f;
      op->operand_scale[2] = 1.0f;
    }
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

    if (op->operand_selected_edge < -1) {
      op->operand_selected_edge = -1;
    }
    if (op->operand_selected_edges == 0 && op->operand_selected_edge >= 0) {
      op->operand_selected_edges = nurb_body_edge_mask_for_index(op->operand_selected_edge);
    }
    if (op->operand_bevel_edges == 0 && op->operand_bevel_edge >= 0) {
      op->operand_bevel_edges = nurb_body_edge_mask_for_index(op->operand_bevel_edge);
    }
    if (op->operand_surface_selected_edges == 0 && op->operand_surface_selected_edge >= 0) {
      op->operand_surface_selected_edges = nurb_body_edge_mask_for_index(
          op->operand_surface_selected_edge);
    }
    if (op->operand_surface_bevel_edges == 0 && op->operand_surface_bevel_edge >= 0) {
      op->operand_surface_bevel_edges = nurb_body_edge_mask_for_index(
          op->operand_surface_bevel_edge);
    }
    if (op->operand_bevel_radius <= 0.0f || op->operand_bevel_edge < -1) {
      op->operand_bevel_edge = -1;
      op->operand_bevel_edges = 0;
      op->operand_chamfer_edges = 0;
    }
    if (op->operand_surface_bevel_radius <= 0.0f || op->operand_surface_bevel_edge < -1) {
      op->operand_surface_bevel_edge = -1;
      op->operand_surface_bevel_edges = 0;
      op->operand_surface_chamfer_edges = 0;
    }
    if (!ELEM(op->operand_bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER)) {
      op->operand_bevel_type = NURB_BODY_BEVEL_FILLET;
    }
    if (!ELEM(op->operand_surface_bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER))
    {
      op->operand_surface_bevel_type = NURB_BODY_BEVEL_FILLET;
    }
    if (op->operand_bevel_type == NURB_BODY_BEVEL_CHAMFER &&
        op->operand_chamfer_edges == 0 && op->operand_bevel_edges != 0)
    {
      op->operand_chamfer_edges = op->operand_bevel_edges;
    }
    if (op->operand_surface_bevel_type == NURB_BODY_BEVEL_CHAMFER &&
        op->operand_surface_chamfer_edges == 0 && op->operand_surface_bevel_edges != 0)
    {
      op->operand_surface_chamfer_edges = op->operand_surface_bevel_edges;
    }
    nurb_body_materialize_edge_bevel_radii(
        op->operand_bevel_edges, op->operand_bevel_radius, op->operand_bevel_radii);
    nurb_body_materialize_edge_bevel_radii(op->operand_surface_bevel_edges,
                                           op->operand_surface_bevel_radius,
                                           op->operand_surface_bevel_radii);
    op->operand_chamfer_edges &= op->operand_bevel_edges;
    op->operand_surface_chamfer_edges &= op->operand_surface_bevel_edges;
    for (int i = 0; i < 64; i++) {
      if (((op->operand_surface_bevel_edges | op->operand_surface_selected_edges) &
           nurb_body_edge_mask_for_index(i)) == 0)
      {
        op->operand_surface_edge_keys[i] = 0;
      }
    }
    nurb_body_normalize_edge_bevel_order(
        op->operand_bevel_edges, op->operand_bevel_order, op->operand_bevel_order_next);
    nurb_body_normalize_edge_bevel_order(op->operand_surface_bevel_edges,
                                         op->operand_surface_bevel_order,
                                         op->operand_surface_bevel_order_next);
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

static float nurb_body_safe_dimension(const float value)
{
  return std::max(std::abs(value), 0.001f);
}

static float nurb_body_safe_radius(const float value)
{
  return std::max(value, 0.001f);
}

static float3 nurb_body_safe_dimensions(const float dimensions[3])
{
  float3 result(nurb_body_safe_dimension(dimensions[0]),
                nurb_body_safe_dimension(dimensions[1]),
                nurb_body_safe_dimension(dimensions[2]));
  if (result.x == 0.001f && result.y == 0.001f && result.z == 0.001f) {
    result = float3(2.0f);
  }
  return result;
}

static gp_Pnt nurb_body_gp_point(const float3 center)
{
  return gp_Pnt(center.x, center.y, center.z);
}

static gp_Ax2 nurb_body_blender_z_axis(const float3 center)
{
  return gp_Ax2(nurb_body_gp_point(center), gp_Dir(0.0, 0.0, 1.0));
}

static TopoDS_Shape make_centered_box(const float dimensions[3], const float3 center)
{
  const float3 size = nurb_body_safe_dimensions(dimensions);
  const gp_Pnt corner(center.x - size.x * 0.5f,
                      center.y - size.y * 0.5f,
                      center.z - size.z * 0.5f);
  return BRepPrimAPI_MakeBox(corner, size.x, size.y, size.z).Shape();
}

static TopoDS_Shape make_centered_sphere(const float radius, const float3 center)
{
  return BRepPrimAPI_MakeSphere(nurb_body_gp_point(center), nurb_body_safe_radius(radius)).Shape();
}

static TopoDS_Shape make_centered_cylinder(const float radius,
                                           const float depth,
                                           const float3 center)
{
  const float safe_depth = nurb_body_safe_dimension(depth);
  const gp_Pnt base(center.x, center.y, center.z - safe_depth * 0.5f);
  return BRepPrimAPI_MakeCylinder(gp_Ax2(base, gp_Dir(0.0, 0.0, 1.0)),
                                  nurb_body_safe_radius(radius),
                                  safe_depth)
      .Shape();
}

static TopoDS_Shape make_centered_cone(const float radius, const float depth, const float3 center)
{
  const float safe_depth = nurb_body_safe_dimension(depth);
  const gp_Pnt base(center.x, center.y, center.z - safe_depth * 0.5f);
  return BRepPrimAPI_MakeCone(
             gp_Ax2(base, gp_Dir(0.0, 0.0, 1.0)), nurb_body_safe_radius(radius), 0.0, safe_depth)
      .Shape();
}

static TopoDS_Shape make_centered_torus(const float major_radius,
                                        const float minor_radius,
                                        const float3 center)
{
  const float safe_minor_radius = nurb_body_safe_radius(minor_radius);
  const float safe_major_radius = std::max(major_radius, safe_minor_radius + 0.001f);
  return BRepPrimAPI_MakeTorus(
             nurb_body_blender_z_axis(center), safe_major_radius, safe_minor_radius)
      .Shape();
}

static TopoDS_Shape make_centered_wedge(const float dimensions[3], const float3 center)
{
  const float3 size = nurb_body_safe_dimensions(dimensions);
  const gp_Pnt corner(center.x - size.x * 0.5f,
                      center.y - size.y * 0.5f,
                      center.z - size.z * 0.5f);
  return BRepPrimAPI_MakeWedge(gp_Ax2(corner, gp_Dir(0.0, 0.0, 1.0)),
                               size.x,
                               size.y,
                               size.z,
                               size.x * 0.5f)
      .Shape();
}

static TopoDS_Shape make_primitive_shape(const int primitive,
                                         const float radius,
                                         const float depth,
                                         const float minor_radius,
                                         const float dimensions[3],
                                         const float3 center)
{
  switch (primitive) {
    case NURB_BODY_PRIMITIVE_BOX:
      return make_centered_box(dimensions, center);
    case NURB_BODY_PRIMITIVE_SPHERE:
      return make_centered_sphere(radius, center);
    case NURB_BODY_PRIMITIVE_CONE:
      return make_centered_cone(radius, depth, center);
    case NURB_BODY_PRIMITIVE_TORUS:
      return make_centered_torus(radius, minor_radius, center);
    case NURB_BODY_PRIMITIVE_WEDGE:
      return make_centered_wedge(dimensions, center);
    case NURB_BODY_PRIMITIVE_CYLINDER:
    default:
      return make_centered_cylinder(radius, depth, center);
  }
}

static TopoDS_Shape make_body_primitive_shape(const NurbBody &body)
{
  return make_primitive_shape(body.primitive,
                              body.radius,
                              body.depth,
                              body.minor_radius,
                              body.dimensions,
                              float3(0.0f));
}

static TopoDS_Shape transform_shape(const TopoDS_Shape &shape, const float mat[4][4])
{
  float size[3];
  mat4_to_size(size, mat);
  const bool needs_general_transform = std::abs(size[0] - size[1]) > 1.0e-5f ||
                                       std::abs(size[0] - size[2]) > 1.0e-5f;
  if (needs_general_transform) {
    gp_GTrsf transform;
    transform.SetValue(1, 1, mat[0][0]);
    transform.SetValue(1, 2, mat[1][0]);
    transform.SetValue(1, 3, mat[2][0]);
    transform.SetValue(1, 4, mat[3][0]);
    transform.SetValue(2, 1, mat[0][1]);
    transform.SetValue(2, 2, mat[1][1]);
    transform.SetValue(2, 3, mat[2][1]);
    transform.SetValue(2, 4, mat[3][1]);
    transform.SetValue(3, 1, mat[0][2]);
    transform.SetValue(3, 2, mat[1][2]);
    transform.SetValue(3, 3, mat[2][2]);
    transform.SetValue(3, 4, mat[3][2]);
    return BRepBuilderAPI_GTransform(shape, transform, true).Shape();
  }

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

static TopoDS_Shape make_boolean_op_primitive_shape(const NurbBodyBooleanOp &op)
{
  return make_primitive_shape(op.primitive,
                              op.operand_radius,
                              op.operand_depth,
                              op.operand_minor_radius,
                              op.operand_dimensions,
                              float3(0.0f));
}

static float nurb_body_safe_scale_axis(const float scale)
{
  return std::max(std::abs(scale), 0.001f);
}

static float nurb_body_signed_scale_axis(const float scale)
{
  if (std::abs(scale) >= 0.001f) {
    return scale;
  }
  return scale < 0.0f ? -0.001f : 0.001f;
}

static float nurb_body_boolean_op_axis_scale(const NurbBodyBooleanOp &op)
{
  return nurb_body_safe_scale_axis(op.operand_scale[2] == 0.0f ? 1.0f :
                                                               op.operand_scale[2]);
}

static float nurb_body_boolean_op_radial_scale(const NurbBodyBooleanOp &op)
{
  const float x_scale = nurb_body_safe_scale_axis(op.operand_scale[0] == 0.0f ?
                                                     1.0f :
                                                     op.operand_scale[0]);
  const float y_scale = nurb_body_safe_scale_axis(op.operand_scale[1] == 0.0f ?
                                                     1.0f :
                                                     op.operand_scale[1]);
  return (x_scale + y_scale) * 0.5f;
}

static bool nurb_body_boolean_op_has_non_unit_scale(const NurbBodyBooleanOp &op)
{
  return std::abs(nurb_body_boolean_op_axis_scale(op) - 1.0f) > 1.0e-5f ||
         std::abs(nurb_body_boolean_op_radial_scale(op) - 1.0f) > 1.0e-5f;
}

static float nurb_body_boolean_op_scaled_radius(const NurbBodyBooleanOp &op)
{
  return std::max(op.operand_radius * nurb_body_boolean_op_radial_scale(op), 0.001f);
}

static float nurb_body_boolean_op_scaled_depth(const NurbBodyBooleanOp &op)
{
  return std::max(op.operand_depth * nurb_body_boolean_op_axis_scale(op), 0.001f);
}

static float nurb_body_primitive_blend_radius_limit(const int primitive,
                                                    const float radius,
                                                    const float depth,
                                                    const float minor_radius,
                                                    const float dimensions[3])
{
  const float safe_radius = std::max(radius, 0.001f);
  const float safe_depth = std::max(depth, 0.001f);
  const float safe_minor_radius = std::max(minor_radius, 0.001f);
  const float safe_dimensions[3] = {
      std::max(dimensions[0], 0.001f),
      std::max(dimensions[1], 0.001f),
      std::max(dimensions[2], 0.001f),
  };

  switch (primitive) {
    case NURB_BODY_PRIMITIVE_BOX:
    case NURB_BODY_PRIMITIVE_WEDGE: {
      const float min_dimension = std::min(std::min(safe_dimensions[0], safe_dimensions[1]),
                                           safe_dimensions[2]);
      return std::max(min_dimension * 0.5f, 0.001f);
    }
    case NURB_BODY_PRIMITIVE_TORUS:
      return safe_minor_radius;
    case NURB_BODY_PRIMITIVE_CYLINDER:
    case NURB_BODY_PRIMITIVE_CONE:
      return std::max(std::min(safe_radius, safe_depth * 0.5f), 0.001f);
    case NURB_BODY_PRIMITIVE_SPHERE:
    default:
      return safe_radius;
  }
}

static float nurb_body_blend_radius_limit(const NurbBody &body)
{
  return nurb_body_primitive_blend_radius_limit(
      body.primitive, body.radius, body.depth, body.minor_radius, body.dimensions);
}

static void scaled_edge_radii(const float src[64], const float scale, float dst[64])
{
  for (int i = 0; i < 64; i++) {
    dst[i] = src[i] * scale;
  }
}

static void scaled_primitive_dimensions(const NurbBodyBooleanOp &op, float r_dimensions[3])
{
  r_dimensions[0] = nurb_body_safe_dimension(
      op.operand_dimensions[0] * nurb_body_safe_scale_axis(op.operand_scale[0] == 0.0f ?
                                                               1.0f :
                                                               op.operand_scale[0]));
  r_dimensions[1] = nurb_body_safe_dimension(
      op.operand_dimensions[1] * nurb_body_safe_scale_axis(op.operand_scale[1] == 0.0f ?
                                                               1.0f :
                                                               op.operand_scale[1]));
  r_dimensions[2] = nurb_body_safe_dimension(
      op.operand_dimensions[2] * nurb_body_safe_scale_axis(op.operand_scale[2] == 0.0f ?
                                                               1.0f :
                                                               op.operand_scale[2]));
}

static float nurb_body_boolean_op_scaled_blend_radius_limit(const NurbBodyBooleanOp &op)
{
  float operand_dimensions[3];
  scaled_primitive_dimensions(op, operand_dimensions);
  return nurb_body_primitive_blend_radius_limit(op.primitive,
                                                nurb_body_boolean_op_scaled_radius(op),
                                                nurb_body_boolean_op_scaled_depth(op),
                                                op.operand_minor_radius *
                                                    nurb_body_boolean_op_radial_scale(op),
                                                operand_dimensions);
}

static void nurb_body_boolean_op_transform_matrix(const NurbBodyBooleanOp &op, float r_mat[4][4])
{
  copy_m4_m4(r_mat, op.operand_to_target);

  if (ELEM(op.primitive, NURB_BODY_PRIMITIVE_BOX, NURB_BODY_PRIMITIVE_WEDGE)) {
    const float residual_scale[3] = {
        nurb_body_signed_scale_axis(op.operand_scale[0] == 0.0f ? 1.0f : op.operand_scale[0]) <
                0.0f ?
            -1.0f :
            1.0f,
        nurb_body_signed_scale_axis(op.operand_scale[1] == 0.0f ? 1.0f : op.operand_scale[1]) <
                0.0f ?
            -1.0f :
            1.0f,
        nurb_body_signed_scale_axis(op.operand_scale[2] == 0.0f ? 1.0f : op.operand_scale[2]) <
                0.0f ?
            -1.0f :
            1.0f,
    };
    mul_v3_fl(r_mat[0], residual_scale[0]);
    mul_v3_fl(r_mat[1], residual_scale[1]);
    mul_v3_fl(r_mat[2], residual_scale[2]);
    return;
  }

  const float axis_scale = nurb_body_boolean_op_axis_scale(op);
  const float radial_scale = nurb_body_boolean_op_radial_scale(op);
  /* Radius/depth and operand bevel radii are already baked with axis/radial scale. Keep that
   * existing bookkeeping, then apply the leftover anisotropic object scale to the final tool. */
  const float residual_scale[3] = {
      nurb_body_signed_scale_axis(op.operand_scale[0] == 0.0f ? 1.0f : op.operand_scale[0]) /
          radial_scale,
      nurb_body_signed_scale_axis(op.operand_scale[1] == 0.0f ? 1.0f : op.operand_scale[1]) /
          radial_scale,
      nurb_body_signed_scale_axis(op.operand_scale[2] == 0.0f ? 1.0f : op.operand_scale[2]) /
          axis_scale,
  };

  mul_v3_fl(r_mat[0], residual_scale[0]);
  mul_v3_fl(r_mat[1], residual_scale[1]);
  mul_v3_fl(r_mat[2], residual_scale[2]);
}

static bool topological_edge_is_visible_outline(
    const TopoDS_Edge &edge,
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces);

static bool shape_has_unmeshed_faces(const TopoDS_Shape &shape)
{
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull() || triangulation->NbNodes() == 0 ||
        triangulation->NbTriangles() == 0)
    {
      return true;
    }
  }
  return false;
}

static void triangulate_shape_for_preview(const TopoDS_Shape &shape,
                                          const NurbBody &body,
                                          const bool force_rebuild = true)
{
  if (!force_rebuild && !shape_has_unmeshed_faces(shape)) {
    return;
  }

  constexpr double min_linear_deflection = 1.0e-5;
  constexpr double retry_angle = 0.05235987755982989; /* 3 degrees. */

  const double requested_linear = std::max(double(body.tessellation_deflection),
                                           min_linear_deflection);
  const double linear_deflection = requested_linear;

  const double requested_angle = std::clamp(
      double(body.tessellation_angle), 0.01, 3.14159265358979323846);
  const double angular_deflection = requested_angle;

  BRepTools::Clean(shape);
  BRepMesh_IncrementalMesh(shape, linear_deflection, false, angular_deflection, true);
  if (shape_has_unmeshed_faces(shape)) {
    BRepTools::Clean(shape);
    BRepMesh_IncrementalMesh(shape,
                             std::max(linear_deflection * 0.35, min_linear_deflection),
                             false,
                             std::min(angular_deflection, retry_angle),
                             true);
  }
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
  int sample_count = std::max(samples, 2);
  if (!Handle(Geom_Line)::DownCast(curve).IsNull()) {
    sample_count = 1;
  }
  else if (!Handle(Geom_Circle)::DownCast(curve).IsNull()) {
    constexpr double two_pi = 6.28318530717958647692;
    const double angle_span = std::abs(double(last) - double(first));
    const int min_circle_samples = std::min(8, sample_count);
    sample_count = std::clamp(int(std::ceil((angle_span / two_pi) * double(sample_count))),
                              min_circle_samples,
                              sample_count);
  }
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
  uint64_t edge_key = 0;
  float threshold = 0.0f;
  Vector<float3> points;
};

struct NurbBodySurfaceEdgeEntry {
  TopoDS_Edge edge;
  uint64_t edge_key = 0;
  float length = 0.0f;
  Vector<float3> points;
};

struct NurbBodyEdgeReference {
  float length = 0.0f;
  Vector<float3> points;
};

static void hash_quantized_value(uint64_t &hash, const double value, const double scale)
{
  const int64_t quantized = int64_t(std::llround(value * scale));
  hash ^= uint64_t(quantized);
  hash *= 1099511628211ull;
}

static uint64_t surface_edge_geometry_key_for_points(const Span<float3> points)
{
  if (points.size() < 2) {
    return 0;
  }

  double center[3] = {};
  double length = 0.0;
  const float3 close_delta = points.last() - points.first();
  const bool closed = (double(close_delta.x) * double(close_delta.x) +
                       double(close_delta.y) * double(close_delta.y) +
                       double(close_delta.z) * double(close_delta.z)) < 1.0e-12;
  const int point_count = points.size() - (closed ? 1 : 0);
  if (point_count < 2) {
    return 0;
  }

  for (int i = 0; i < point_count; i++) {
    const float3 &point = points[i];
    const double point_values[3] = {double(point.x), double(point.y), double(point.z)};
    for (int axis = 0; axis < 3; axis++) {
      center[axis] += point_values[axis];
    }
  }
  const double inv_count = 1.0 / double(point_count);
  for (int axis = 0; axis < 3; axis++) {
    center[axis] *= inv_count;
  }

  for (int i = 1; i < points.size(); i++) {
    const float3 delta = points[i] - points[i - 1];
    length += std::sqrt(double(delta.x) * double(delta.x) +
                        double(delta.y) * double(delta.y) +
                        double(delta.z) * double(delta.z));
  }

  double covariance[6] = {};
  for (int i = 0; i < point_count; i++) {
    const float3 &point = points[i];
    const double x = double(point.x) - center[0];
    const double y = double(point.y) - center[1];
    const double z = double(point.z) - center[2];
    covariance[0] += x * x;
    covariance[1] += y * y;
    covariance[2] += z * z;
    covariance[3] += x * y;
    covariance[4] += x * z;
    covariance[5] += y * z;
  }
  for (double &value : covariance) {
    value *= inv_count;
  }

  uint64_t hash = 1469598103934665603ull;
  constexpr double quantize_scale = 10000.0;
  for (int axis = 0; axis < 3; axis++) {
    hash_quantized_value(hash, center[axis], quantize_scale);
  }
  hash_quantized_value(hash, length, quantize_scale);
  for (const double value : covariance) {
    hash_quantized_value(hash, value, quantize_scale);
  }
  return hash != 0 ? hash : 1;
}

static bool surface_edge_catalog_has_key(const Span<NurbBodySurfaceEdgeEntry> edges,
                                         const uint64_t edge_key)
{
  if (edge_key == 0) {
    return false;
  }

  for (const NurbBodySurfaceEdgeEntry &entry : edges) {
    if (entry.edge_key == edge_key) {
      return true;
    }
  }
  return false;
}

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

static float polyline_length(const Span<float3> points)
{
  float length = 0.0f;
  for (int i = 1; i < points.size(); i++) {
    const float3 delta = points[i] - points[i - 1];
    length += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  }
  return length;
}

static int find_exact_selectable_edge_ref_index(
    const uint64_t edge_key, const Span<NurbBodySelectableEdgeRef> selectable_refs)
{
  if (edge_key == 0 || selectable_refs.is_empty()) {
    return -1;
  }

  for (const int ref_i : selectable_refs.index_range()) {
    const NurbBodySelectableEdgeRef &ref = selectable_refs[ref_i];
    if (ref.edge_key == edge_key) {
      return ref_i;
    }
  }
  return -1;
}

static Vector<TopoDS_Face> unique_edge_faces(const TopTools_ListOfShape &faces)
{
  Vector<TopoDS_Face> unique_faces;
  for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) {
    const TopoDS_Face face = TopoDS::Face(it.Value());
    bool already_added = false;
    for (const TopoDS_Face &unique_face : unique_faces) {
      if (face.IsSame(unique_face)) {
        already_added = true;
        break;
      }
    }
    if (!already_added) {
      unique_faces.append(face);
    }
  }
  return unique_faces;
}

static int unique_edge_face_count(const TopTools_ListOfShape &faces)
{
  return unique_edge_faces(faces).size();
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

  const Vector<TopoDS_Face> unique_faces = unique_edge_faces(*faces);
  if (unique_faces.size() == 1) {
    const TopoDS_Face face = unique_faces.first();
    return !BRep_Tool::IsClosed(edge, face);
  }

  return unique_faces.size() > 1;
}

static uint64_t edge_geometry_key(
    const TopoDS_Edge &edge,
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces)
{
  Vector<float3> points;
  if (!(append_edge_polyline_local(edge, 32, points) ||
        append_tessellated_edge_polyline_local(edge, edge_faces, points)))
  {
    return 0;
  }
  return surface_edge_geometry_key_for_points(points.as_span());
}

static bool edge_geometry_key_in_span(const Span<uint64_t> edge_keys, const uint64_t edge_key)
{
  if (edge_key == 0) {
    return false;
  }
  for (const uint64_t existing_key : edge_keys) {
    if (existing_key == edge_key) {
      return true;
    }
  }
  return false;
}

static Vector<uint64_t> selectable_edge_geometry_keys(const TopoDS_Shape &shape)
{
  Vector<uint64_t> edge_keys;
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  for (int i = 1; i <= edge_faces.Extent(); i++) {
    if (unique_edge_face_count(edge_faces.FindFromIndex(i)) < 2) {
      continue;
    }
    const TopoDS_Edge edge = TopoDS::Edge(edge_faces.FindKey(i));
    if (!topological_edge_is_visible_outline(edge, edge_faces)) {
      continue;
    }
    const uint64_t edge_key = edge_geometry_key(edge, edge_faces);
    if (edge_key != 0 && !edge_geometry_key_in_span(edge_keys.as_span(), edge_key)) {
      edge_keys.append(edge_key);
    }
  }
  return edge_keys;
}

static Vector<TopoDS_Edge> find_boolean_output_edges(
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces,
    const Span<uint64_t> pre_boolean_edge_keys)
{
  Vector<TopoDS_Edge> edges;
  Vector<uint64_t> emitted_edge_keys;
  for (int i = 1; i <= edge_faces.Extent() && edges.size() < 64; i++) {
    if (unique_edge_face_count(edge_faces.FindFromIndex(i)) < 2) {
      continue;
    }
    const TopoDS_Edge edge = TopoDS::Edge(edge_faces.FindKey(i));
    if (!topological_edge_is_visible_outline(edge, edge_faces)) {
      continue;
    }
    const uint64_t edge_key = edge_geometry_key(edge, edge_faces);
    if (edge_key == 0 || edge_geometry_key_in_span(pre_boolean_edge_keys, edge_key) ||
        edge_geometry_key_in_span(emitted_edge_keys.as_span(), edge_key))
    {
      continue;
    }
    emitted_edge_keys.append(edge_key);
    edges.append(edge);
  }
  return edges;
}

static Vector<TopoDS_Edge> find_boolean_output_edges(
    const TopoDS_Shape &shape,
    const Span<uint64_t> pre_boolean_edge_keys)
{
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  return find_boolean_output_edges(edge_faces, pre_boolean_edge_keys);
}

static Vector<NurbBodySurfaceEdgeEntry> find_selectable_surface_edge_catalog(
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces,
    const int samples_per_edge)
{
  Vector<NurbBodySurfaceEdgeEntry> edges;
  for (int i = 1; i <= edge_faces.Extent(); i++) {
    if (unique_edge_face_count(edge_faces.FindFromIndex(i)) < 2) {
      continue;
    }
    const TopoDS_Edge edge = TopoDS::Edge(edge_faces.FindKey(i));
    if (!topological_edge_is_visible_outline(edge, edge_faces)) {
      continue;
    }

    NurbBodySurfaceEdgeEntry entry;
    entry.edge = edge;
    if (!(append_tessellated_edge_polyline_local(edge, edge_faces, entry.points) ||
          append_edge_polyline_local(edge, samples_per_edge, entry.points)) ||
        entry.points.size() < 2)
    {
      continue;
    }
    entry.length = polyline_length(entry.points.as_span());
    Vector<float3> key_points;
    if (!append_edge_polyline_local(edge, 32, key_points)) {
      key_points = entry.points;
    }
    entry.edge_key = surface_edge_geometry_key_for_points(key_points);
    if (surface_edge_catalog_has_key(edges.as_span(), entry.edge_key)) {
      continue;
    }
    edges.append(std::move(entry));
  }
  return edges;
}

static Vector<NurbBodySurfaceEdgeEntry> find_selectable_surface_edge_catalog(
    const TopoDS_Shape &shape,
    const int samples_per_edge)
{
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  return find_selectable_surface_edge_catalog(edge_faces, samples_per_edge);
}

static int find_surface_edge_slot_for_key(const NurbBody &body,
                                          const uint64_t edge_key,
                                          const uint64_t slot_mask)
{
  if (edge_key == 0) {
    return -1;
  }
  for (int i = 0; i < 64; i++) {
    if ((slot_mask & nurb_body_edge_mask_for_index(i)) != 0 &&
        body.surface_edge_keys[i] == edge_key)
    {
      return i;
    }
  }
  return -1;
}

static int find_catalog_edge_for_key(const Span<NurbBodySurfaceEdgeEntry> catalog,
                                     const uint64_t edge_key)
{
  if (edge_key == 0) {
    return -1;
  }
  for (const int i : catalog.index_range()) {
    if (catalog[i].edge_key == edge_key) {
      return i;
    }
  }
  return -1;
}

static Vector<TopoDS_Edge> find_selectable_surface_edges_indexed(
    const TopTools_IndexedDataMapOfShapeListOfShape &edge_faces,
    const int samples_per_edge,
    const NurbBody &body)
{
  Vector<TopoDS_Edge> edges;
  edges.resize(64);
  bool used_slots[64] = {};
  const uint64_t reserved_slots = body.surface_bevel_edges | body.surface_selected_edges;
  for (int i = 0; i < 64; i++) {
    used_slots[i] = (reserved_slots & nurb_body_edge_mask_for_index(i)) != 0;
  }

  const Vector<NurbBodySurfaceEdgeEntry> catalog = find_selectable_surface_edge_catalog(
      edge_faces, samples_per_edge);
  for (const NurbBodySurfaceEdgeEntry &entry : catalog) {
    if (entry.edge_key == 0) {
      continue;
    }

    int edge_index = find_surface_edge_slot_for_key(body, entry.edge_key, reserved_slots);
    if (edge_index == -1) {
      for (int i = 0; i < 64; i++) {
        if (!used_slots[i]) {
          edge_index = i;
          break;
        }
      }
    }
    if (edge_index == -1) {
      continue;
    }
    used_slots[edge_index] = true;
    edges[edge_index] = entry.edge;
  }
  return edges;
}

static void append_shape_surface_edge_polylines(const TopoDS_Shape &shape,
                                                const int samples_per_edge,
                                                const Span<NurbBodySelectableEdgeRef> selectable_refs,
                                                const NurbBody &body,
                                                Vector<NurbBodyEdgePolyline> &r_polylines)
{
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  bool final_slots_used[64] = {};
  const uint64_t reserved_slots = body.surface_bevel_edges | body.surface_selected_edges;
  for (int i = 0; i < 64; i++) {
    final_slots_used[i] = (reserved_slots & nurb_body_edge_mask_for_index(i)) != 0;
  }

  const Vector<NurbBodySurfaceEdgeEntry> catalog = find_selectable_surface_edge_catalog(
      edge_faces, samples_per_edge);
  for (const NurbBodySurfaceEdgeEntry &entry : catalog) {
    NurbBodyEdgePolyline polyline;
    polyline.flag = NURB_BODY_EDGE_POLYLINE_SURFACE;
    polyline.edge_key = entry.edge_key;
    polyline.points = entry.points;
    if (entry.edge_key != 0) {
      const int ref_i = find_exact_selectable_edge_ref_index(entry.edge_key, selectable_refs);
      if (ref_i != -1)
      {
        const NurbBodySelectableEdgeRef *ref = &selectable_refs[ref_i];
        polyline.op = ref->op;
        polyline.edge_index = ref->edge_index;
        polyline.edge_key = ref->edge_key;
        polyline.flag |= ref->flag | NURB_BODY_EDGE_POLYLINE_SELECTABLE;
        if ((ref->flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0 && ref->edge_index >= 0 &&
            ref->edge_index < 64)
        {
          final_slots_used[ref->edge_index] = true;
        }
      }
      else {
        int assigned_edge_index = find_surface_edge_slot_for_key(body,
                                                                 entry.edge_key,
                                                                 reserved_slots);
        if (assigned_edge_index == -1) {
          for (int edge_index = 0; edge_index < 64; edge_index++) {
            if (!final_slots_used[edge_index]) {
              assigned_edge_index = edge_index;
              break;
            }
          }
        }
        if (assigned_edge_index != -1) {
          final_slots_used[assigned_edge_index] = true;
          polyline.edge_index = assigned_edge_index;
          polyline.flag |= NURB_BODY_EDGE_POLYLINE_FINAL | NURB_BODY_EDGE_POLYLINE_SELECTABLE;
        }
      }
    }
    r_polylines.append(std::move(polyline));
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

static bool run_boolean_builder_attempt(BRepAlgoAPI_BooleanOperation &builder,
                                        const TopoDS_Shape &base,
                                        const TopoDS_Shape &tool,
                                        const BOPAlgo_Operation operation,
                                        const bool run_parallel,
                                        const bool use_obb,
                                        const double fuzzy_value,
                                        TopoDS_Shape &r_result)
{
  NCollection_List<TopoDS_Shape> arguments;
  NCollection_List<TopoDS_Shape> tools;
  arguments.Append(base);
  tools.Append(tool);
  builder.SetArguments(arguments);
  builder.SetTools(tools);
  builder.SetOperation(operation);
  builder.SetRunParallel(run_parallel);
  builder.SetUseOBB(use_obb);
  builder.SetCheckInverted(false);
  builder.SetFuzzyValue(fuzzy_value);
  try {
    builder.Build();
  }
  catch (Standard_Failure const &) {
    return false;
  }
  if (builder.HasErrors()) {
    return false;
  }
  TopoDS_Shape result;
  try {
    result = builder.Shape();
  }
  catch (Standard_Failure const &) {
    return false;
  }
  if (result.IsNull()) {
    return false;
  }
  r_result = result;
  return true;
}

template<typename BooleanBuilder>
static TopoDS_Shape run_boolean_builder(const TopoDS_Shape &base,
                                        const TopoDS_Shape &tool,
                                        const BOPAlgo_Operation operation)
{
  struct BooleanAttempt {
    bool run_parallel;
    bool use_obb;
    double fuzzy_value;
  };
  const BooleanAttempt attempts[] = {
      {true, true, 0.0},
      {false, true, 0.0},
      {false, false, 1.0e-7},
      {false, false, 1.0e-5},
  };

  for (const BooleanAttempt &attempt : attempts) {
    BooleanBuilder builder;
    TopoDS_Shape result;
    if (run_boolean_builder_attempt(builder,
                                    base,
                                    tool,
                                    operation,
                                    attempt.run_parallel,
                                    attempt.use_obb,
                                    attempt.fuzzy_value,
                                    result))
    {
      return result;
    }
  }
  return base;
}

static TopoDS_Shape apply_boolean_operation(const TopoDS_Shape &base,
                                            const TopoDS_Shape &tool,
                                            const int operation)
{
  try {
    switch (operation) {
      case NURB_BODY_BOOLEAN_UNION: {
        return run_boolean_builder<BRepAlgoAPI_Fuse>(base, tool, BOPAlgo_FUSE);
      }
      case NURB_BODY_BOOLEAN_INTERSECT: {
        return run_boolean_builder<BRepAlgoAPI_Common>(base, tool, BOPAlgo_COMMON);
      }
      case NURB_BODY_BOOLEAN_DIFFERENCE:
      default: {
        return run_boolean_builder<BRepAlgoAPI_Cut>(base, tool, BOPAlgo_CUT);
      }
    }
  }
  catch (Standard_Failure const &) {
    return base;
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
                                        const uint64_t hidden_edges,
                                        const uint64_t radius_edges,
                                        const float blend_radius,
                                        const float *blend_radii,
                                        Vector<NurbBodySelectableEdgeRef> &r_refs)
{
  for (const int i : edges.index_range()) {
    if (edges[i].IsNull()) {
      continue;
    }
    const uint64_t edge_mask = nurb_body_edge_mask_for_index(i);
    const float edge_blend_radius = bevel_radius_for_edge(
        blend_radius, radius_edges, blend_radii, i);
    const bool edge_is_blended = edge_blend_radius > 0.0f && edge_mask != 0 &&
                                 (hidden_edges & edge_mask) != 0;
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
      Vector<float3> key_points;
      if (!append_edge_polyline_local(edges[i], 32, key_points)) {
        key_points = ref.points;
      }
      ref.edge_key = surface_edge_geometry_key_for_points(key_points);
      if (ref.edge_key == 0) {
        continue;
      }
      if (find_exact_selectable_edge_ref_index(ref.edge_key, r_refs.as_span()) != -1) {
        continue;
      }
      r_refs.append(std::move(ref));
    }
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
    return false;
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

struct NurbBodyBlendCandidate {
  TopoDS_Edge edge;
  double radius = 0.0;
  int source_index = -1;
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

  if (selected_edges != 0) {
    return bevel_radius > 0.0f || edge_radii_contains_positive(bevel_radii, selected_edges);
  }

  const int active_edge = bevel_edge >= 0 ? bevel_edge : selected_edge;
  if (active_edge < 0) {
    return false;
  }
  const uint64_t active_edge_mask = nurb_body_edge_mask_for_index(active_edge);
  return bevel_radius > 0.0f || edge_radii_contains_positive(bevel_radii, active_edge_mask);
}

static bool blend_selection_affects_shape(const float bevel_radius,
                                          const float *bevel_radii,
                                          const uint64_t bevel_edges,
                                          const uint64_t selected_edges,
                                          const int bevel_edge,
                                          const int selected_edge)
{
  if (bevel_edges != 0) {
    return false;
  }
  if (selected_edges != 0) {
    return bevel_radius > 0.0f || edge_radii_contains_positive(bevel_radii, selected_edges);
  }
  if (bevel_edge >= 0 || selected_edge < 0) {
    return false;
  }
  return bevel_radius > 0.0f ||
         edge_radii_contains_positive(bevel_radii, nurb_body_edge_mask_for_index(selected_edge));
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

static bool blend_steps_can_share_builder(const NurbBodyBlendStep &a,
                                          const NurbBodyBlendStep &b)
{
  return a.bevel_type == b.bevel_type;
}

static int active_blend_edge_index(const int bevel_edge, const int selected_edge)
{
  return bevel_edge >= 0 ? bevel_edge : selected_edge;
}

static int active_blend_step_position(const Span<NurbBodyBlendStep> steps,
                                      const int active_edge)
{
  if (active_edge < 0) {
    return -1;
  }
  for (const int i : steps.index_range()) {
    if (steps[i].edge_index == active_edge) {
      return i;
    }
  }
  return -1;
}

static Vector<NurbBodyBlendStep> stable_blend_steps_without_active(
    const Span<NurbBodyBlendStep> steps, const int active_pos)
{
  Vector<NurbBodyBlendStep> stable_steps;
  if (steps.size() > 1) {
    stable_steps.reserve(steps.size() - 1);
  }
  for (const int i : steps.index_range()) {
    if (i != active_pos) {
      stable_steps.append(steps[i]);
    }
  }
  return stable_steps;
}

static NurbBodyEdgeReference edge_reference_for_blend(const TopoDS_Edge &edge)
{
  NurbBodyEdgeReference reference;
  if (!edge.IsNull() && append_edge_polyline_local(edge, 16, reference.points) &&
      reference.points.size() >= 2)
  {
    reference.length = polyline_length(reference.points.as_span());
  }
  else {
    reference.points.clear();
  }
  return reference;
}

static Vector<NurbBodyEdgeReference> edge_references_for_blend(
    const Vector<TopoDS_Edge> &edges)
{
  Vector<NurbBodyEdgeReference> references;
  references.reserve(edges.size());
  for (const TopoDS_Edge &edge : edges) {
    references.append(edge_reference_for_blend(edge));
  }
  return references;
}

static int find_current_edge_by_reference(const Span<NurbBodyEdgeReference> edge_references,
                                          const int fallback_index,
                                          const NurbBodyEdgeReference &reference)
{
  if (reference.points.size() < 2) {
    return fallback_index >= 0 && fallback_index < edge_references.size() ? fallback_index : -1;
  }

  const float threshold = std::max(reference.length * 0.02f, 0.001f);

  int best_index = -1;
  float best_dist_sq = FLT_MAX;
  for (const int i : edge_references.index_range()) {
    const Span<float3> points = edge_references[i].points.as_span();
    if (points.size() < 2) {
      continue;
    }

    float dist_sq = 0.0f;
    for (const float3 &point : reference.points) {
      dist_sq += dist_squared_to_polyline_v3(point, points);
    }
    dist_sq /= float(reference.points.size());
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

static double safe_blend_radius_limit(const float radius_limit)
{
  const double base_limit = double(std::max(radius_limit, 0.001f)) * 0.995;
  return std::max(base_limit, 1.0e-6);
}

static bool shape_can_be_previewed(const TopoDS_Shape &shape)
{
  if (shape.IsNull()) {
    return false;
  }

  TopExp_Explorer face_explorer(shape, TopAbs_FACE);
  if (!face_explorer.More()) {
    return false;
  }

  try {
    BRepTools::Clean(shape);
    BRepMesh_IncrementalMesh(shape, 0.05, false, 0.5, true);
    if (shape_has_unmeshed_faces(shape)) {
      BRepTools::Clean(shape);
      BRepMesh_IncrementalMesh(shape, 0.005, false, 0.05235987755982989, true);
    }
  }
  catch (Standard_Failure const &) {
    return false;
  }

  return !shape_has_unmeshed_faces(shape);
}

static void prepare_edge_for_blend(const TopoDS_Edge &edge)
{
  if (edge.IsNull()) {
    return;
  }

  try {
    BRepLib::BuildCurve3d(edge, 1.0e-5);
    if (!BRep_Tool::SameParameter(edge)) {
      BRepLib::SameParameter(edge, 1.0e-5);
    }
  }
  catch (Standard_Failure const &) {
    /* Keep the original edge if OCCT cannot repair parameterization quickly. */
  }
}

static bool nurb_body_profile_blend_enabled()
{
  static const bool enabled = std::getenv("NURB_BODY_PROFILE_BLEND") != nullptr;
  return enabled;
}

static int first_blend_candidate_source_index(const Span<NurbBodyBlendCandidate> edges)
{
  for (const NurbBodyBlendCandidate &candidate : edges) {
    if (candidate.source_index >= 0) {
      return candidate.source_index;
    }
  }
  return -1;
}

static void blend_candidate_radius_range(const Span<NurbBodyBlendCandidate> edges,
                                         double &r_min_radius,
                                         double &r_max_radius)
{
  r_min_radius = DBL_MAX;
  r_max_radius = 0.0;
  for (const NurbBodyBlendCandidate &candidate : edges) {
    if (candidate.radius <= 0.0) {
      continue;
    }
    r_min_radius = std::min(r_min_radius, candidate.radius);
    r_max_radius = std::max(r_max_radius, candidate.radius);
  }
  if (r_min_radius == DBL_MAX) {
    r_min_radius = 0.0;
  }
}

static void profile_blend_build(const Span<NurbBodyBlendCandidate> edges,
                                const int bevel_type,
                                const bool prepare_edges,
                                const bool done,
                                const double seconds)
{
  if (!nurb_body_profile_blend_enabled()) {
    return;
  }
  if (done && seconds < 0.005) {
    return;
  }
  double min_radius = 0.0;
  double max_radius = 0.0;
  blend_candidate_radius_range(edges, min_radius, max_radius);
  std::fprintf(stderr,
               "NURB_BODY_BLEND_PROFILE type=%s edges=%d first_index=%d prepare=%d "
               "done=%d time_ms=%.3f radius_min=%.6f radius_max=%.6f\n",
               bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
               int(edges.size()),
               first_blend_candidate_source_index(edges),
               int(prepare_edges),
               int(done),
               seconds * 1000.0,
               min_radius,
               max_radius);
}

static Vector<NurbBodyBlendCandidate> blend_candidates_for_edges(const Span<TopoDS_Edge> edges,
                                                                 const double radius)
{
  Vector<NurbBodyBlendCandidate> candidates;
  candidates.reserve(edges.size());
  for (const TopoDS_Edge &edge : edges) {
    NurbBodyBlendCandidate candidate;
    candidate.edge = edge;
    candidate.radius = radius;
    candidates.append(candidate);
  }
  return candidates;
}

static bool try_edge_set_blend(const TopoDS_Shape &shape,
                               const int bevel_type,
                               const Span<NurbBodyBlendCandidate> edges,
                               const bool validate_shape,
                               const bool prepare_edges,
                               const ChFi3d_FilletShape fillet_shape,
                               TopoDS_Shape &r_shape)
{
  if (edges.is_empty()) {
    return false;
  }

  try {
    TopoDS_Shape blended_shape;
    if (bevel_type == NURB_BODY_BEVEL_CHAMFER) {
      BRepFilletAPI_MakeChamfer chamfer(shape);
      bool added_edge = false;
      Vector<int> contours;
      for (const NurbBodyBlendCandidate &candidate : edges) {
        if (candidate.radius <= 0.0 || candidate.edge.IsNull() ||
            chamfer.Contour(candidate.edge) != 0)
        {
          continue;
        }
        if (prepare_edges) {
          prepare_edge_for_blend(candidate.edge);
        }
        chamfer.Add(candidate.radius, candidate.edge);
        added_edge = true;
      }
      if (!added_edge) {
        return false;
      }
      for (const NurbBodyBlendCandidate &candidate : edges) {
        const int contour = chamfer.Contour(candidate.edge);
        if (contour <= 0 || contours.contains(contour)) {
          continue;
        }
        contours.append(contour);
      }
      if (contours.is_empty()) {
        return false;
      }
      const double build_start = nurb_body_profile_blend_enabled() ? BLI_time_now_seconds() : 0.0;
      chamfer.Build();
      const bool done = chamfer.IsDone();
      if (nurb_body_profile_blend_enabled()) {
        profile_blend_build(edges,
                            bevel_type,
                            prepare_edges,
                            done,
                            BLI_time_now_seconds() - build_start);
      }
      if (!done) {
        return false;
      }
      blended_shape = chamfer.Shape();
    }
    else {
      BRepFilletAPI_MakeFillet fillet(shape, fillet_shape);
      bool added_edge = false;
      Vector<int> contours;
      for (const NurbBodyBlendCandidate &candidate : edges) {
        if (candidate.radius <= 0.0 || candidate.edge.IsNull() ||
            fillet.Contour(candidate.edge) != 0)
        {
          continue;
        }
        if (prepare_edges) {
          prepare_edge_for_blend(candidate.edge);
        }
        fillet.Add(candidate.radius, candidate.edge);
        added_edge = true;
      }
      if (!added_edge) {
        return false;
      }
      for (const NurbBodyBlendCandidate &candidate : edges) {
        const int contour = fillet.Contour(candidate.edge);
        if (contour <= 0 || contours.contains(contour)) {
          continue;
        }
        contours.append(contour);
      }
      if (contours.is_empty()) {
        return false;
      }
      const double build_start = nurb_body_profile_blend_enabled() ? BLI_time_now_seconds() : 0.0;
      fillet.Build();
      const bool done = fillet.IsDone();
      if (nurb_body_profile_blend_enabled()) {
        profile_blend_build(edges,
                            bevel_type,
                            prepare_edges,
                            done,
                            BLI_time_now_seconds() - build_start);
      }
      if (!done) {
        return false;
      }
      if (fillet.NbFaultyContours() > 0 || fillet.NbFaultyVertices() > 0) {
        return false;
      }
      blended_shape = fillet.Shape();
    }

    if (validate_shape && !shape_can_be_previewed(blended_shape)) {
      return false;
    }
    if (blended_shape.IsNull() || blended_shape.IsSame(shape)) {
      return false;
    }

    r_shape = blended_shape;
    return true;
  }
  catch (Standard_Failure const &) {
    return false;
  }
}

static bool try_edge_set_blend(const TopoDS_Shape &shape,
                               const double radius,
                               const int bevel_type,
                               const Span<TopoDS_Edge> edges,
                               const bool validate_shape,
                               const bool prepare_edges,
                               const ChFi3d_FilletShape fillet_shape,
                               TopoDS_Shape &r_shape)
{
  if (radius <= 0.0) {
    return false;
  }
  const Vector<NurbBodyBlendCandidate> candidates = blend_candidates_for_edges(edges, radius);
  return try_edge_set_blend(shape,
                            bevel_type,
                            candidates.as_span(),
                            validate_shape,
                            prepare_edges,
                            fillet_shape,
                            r_shape);
}

static bool try_single_edge_blend(const TopoDS_Shape &shape,
                                  const double radius,
                                  const int bevel_type,
                                  const TopoDS_Edge &edge,
                                  const int source_index,
                                  const bool validate_shape,
                                  const bool prepare_edges,
                                  const ChFi3d_FilletShape fillet_shape,
                                  TopoDS_Shape &r_shape)
{
  Vector<NurbBodyBlendCandidate> edges;
  NurbBodyBlendCandidate candidate;
  candidate.edge = edge;
  candidate.radius = radius;
  candidate.source_index = source_index;
  edges.append(candidate);
  return try_edge_set_blend(shape,
                            bevel_type,
                            edges.as_span(),
                            validate_shape,
                            prepare_edges,
                            fillet_shape,
                            r_shape);
}

static double edge_length_for_blend(const TopoDS_Edge &edge)
{
  Vector<float3> points;
  if (edge.IsNull() || !append_edge_polyline_local(edge, 16, points) || points.size() < 2) {
    return 0.0;
  }
  return double(polyline_length(points.as_span()));
}

static bool edge_direction_away_from_vertex(const TopoDS_Edge &edge,
                                            const TopoDS_Vertex &vertex,
                                            gp_Vec &r_direction)
{
  TopoDS_Vertex first;
  TopoDS_Vertex last;
  TopExp::Vertices(edge, first, last);
  if (first.IsNull() || last.IsNull() || vertex.IsNull()) {
    return false;
  }

  TopoDS_Vertex other;
  if (first.IsSame(vertex)) {
    other = last;
  }
  else if (last.IsSame(vertex)) {
    other = first;
  }
  else {
    return false;
  }

  const gp_Pnt vertex_point = BRep_Tool::Pnt(vertex);
  const gp_Pnt other_point = BRep_Tool::Pnt(other);
  r_direction = gp_Vec(vertex_point, other_point);
  if (r_direction.SquareMagnitude() <= 1.0e-18) {
    return false;
  }
  r_direction.Normalize();
  return true;
}

static bool edge_already_listed(const Span<TopoDS_Edge> edges, const TopoDS_Edge &edge)
{
  for (const TopoDS_Edge &listed_edge : edges) {
    if (listed_edge.IsSame(edge)) {
      return true;
    }
  }
  return false;
}

static bool edge_face_lists_share_face(const TopTools_ListOfShape *a,
                                       const TopTools_ListOfShape *b)
{
  if (a == nullptr || b == nullptr) {
    return false;
  }
  for (TopTools_ListIteratorOfListOfShape a_it(*a); a_it.More(); a_it.Next()) {
    const TopoDS_Face a_face = TopoDS::Face(a_it.Value());
    for (TopTools_ListIteratorOfListOfShape b_it(*b); b_it.More(); b_it.Next()) {
      if (a_face.IsSame(TopoDS::Face(b_it.Value()))) {
        return true;
      }
    }
  }
  return false;
}

static Vector<TopoDS_Edge> connected_sharp_blend_edges(const TopoDS_Shape &shape,
                                                       const TopoDS_Edge &edge,
                                                       const double radius)
{
  Vector<TopoDS_Edge> result;
  if (shape.IsNull() || edge.IsNull()) {
    return result;
  }

  result.append(edge);

  TopoDS_Vertex first;
  TopoDS_Vertex last;
  TopExp::Vertices(edge, first, last);
  if (first.IsNull() || last.IsNull()) {
    return result;
  }

  TopTools_IndexedDataMapOfShapeListOfShape vertex_edges;
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertex_edges);
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);

  const TopTools_ListOfShape *selected_faces = edge_faces.Seek(edge);
  const double selected_length = edge_length_for_blend(edge);
  const double short_edge_limit = std::max(radius * 4.0, selected_length * 0.15);
  const double shared_face_limit = std::max(radius * 6.0, selected_length * 0.5);
  const TopoDS_Vertex endpoints[2] = {first, last};
  for (const TopoDS_Vertex &vertex : endpoints) {
    const TopTools_ListOfShape *incident_edges = vertex_edges.Seek(vertex);
    if (incident_edges == nullptr) {
      continue;
    }

    gp_Vec selected_direction;
    const bool has_selected_direction = edge_direction_away_from_vertex(edge,
                                                                        vertex,
                                                                        selected_direction);
    for (TopTools_ListIteratorOfListOfShape it(*incident_edges); it.More(); it.Next()) {
      const TopoDS_Edge candidate = TopoDS::Edge(it.Value());
      if (candidate.IsNull() || candidate.IsSame(edge) ||
          edge_already_listed(result.as_span(), candidate) || BRep_Tool::Degenerated(candidate))
      {
        continue;
      }

      const TopTools_ListOfShape *candidate_faces = edge_faces.Seek(candidate);
      if (candidate_faces == nullptr || unique_edge_face_count(*candidate_faces) < 2 ||
          !topological_edge_is_visible_outline(candidate, edge_faces))
      {
        continue;
      }

      const double candidate_length = edge_length_for_blend(candidate);
      if (candidate_length <= 0.0) {
        continue;
      }

      bool include_candidate = candidate_length <= short_edge_limit;
      if (!include_candidate && edge_face_lists_share_face(selected_faces, candidate_faces)) {
        include_candidate = candidate_length <= shared_face_limit;
      }
      if (!include_candidate && has_selected_direction) {
        gp_Vec candidate_direction;
        if (edge_direction_away_from_vertex(candidate, vertex, candidate_direction)) {
          include_candidate = std::abs(selected_direction.Dot(candidate_direction)) >= 0.94;
        }
      }
      if (!include_candidate) {
        continue;
      }

      result.append(candidate);
      if (result.size() >= 8) {
        return result;
      }
    }
  }

  return result;
}

static TopoDS_Shape apply_single_edge_blend(const TopoDS_Shape &shape,
                                            const float bevel_radius,
                                            const int bevel_type,
                                            const TopoDS_Edge &edge,
                                            const int source_index,
                                            const bool validate_shape,
                                            const bool fast_preview,
                                            const double hard_limit,
                                            bool &r_applied)
{
  r_applied = false;
  const double radius = std::min(double(bevel_radius), hard_limit);
  if (radius <= 0.0) {
    return shape;
  }
  const bool validate_blended_shape = validate_shape;
  const ChFi3d_FilletShape fillet_shape = fast_preview ? ChFi3d_Polynomial : ChFi3d_Rational;

  Vector<TopoDS_Edge> connected_edges;
  bool connected_edges_ready = false;
  auto try_blend_radius = [&](const double test_radius, TopoDS_Shape &r_shape) {
    if (try_single_edge_blend(shape,
                              test_radius,
                              bevel_type,
                              edge,
                              source_index,
                              validate_blended_shape,
                              false,
                              fillet_shape,
                              r_shape))
    {
      return true;
    }
    if (try_single_edge_blend(shape,
                              test_radius,
                              bevel_type,
                              edge,
                              source_index,
                              validate_blended_shape,
                              true,
                              fillet_shape,
                              r_shape))
    {
      return true;
    }
    if (!connected_edges_ready) {
      connected_edges = connected_sharp_blend_edges(shape, edge, test_radius);
      connected_edges_ready = true;
    }
    return connected_edges.size() > 1 &&
           try_edge_set_blend(shape,
                              test_radius,
                              bevel_type,
                              connected_edges.as_span(),
                              validate_blended_shape,
                              true,
                              fillet_shape,
                              r_shape);
  };

  TopoDS_Shape blended_shape;
  if (try_blend_radius(radius, blended_shape)) {
    r_applied = true;
    return blended_shape;
  }

  const double min_radius = std::max(std::min(radius, hard_limit) * 1.0e-4, 1.0e-6);
  const int bracket_steps = fast_preview ? 6 : 8;
  const int refine_steps = fast_preview ? 7 : 10;
  double low_radius = 0.0;
  double high_radius = radius;
  TopoDS_Shape best_shape;
  bool found_valid_radius = false;

  for (int i = 0; i < bracket_steps; i++) {
    const double test_radius = high_radius * 0.5;
    if (test_radius <= min_radius) {
      break;
    }
    TopoDS_Shape test_shape;
    if (try_blend_radius(test_radius, test_shape)) {
      low_radius = test_radius;
      best_shape = test_shape;
      found_valid_radius = true;
      break;
    }
    high_radius = test_radius;
  }

  if (!found_valid_radius) {
    return shape;
  }

  for (int i = 0; i < refine_steps; i++) {
    if (high_radius - low_radius <= min_radius) {
      break;
    }
    const double test_radius = (low_radius + high_radius) * 0.5;
    TopoDS_Shape test_shape;
    if (try_blend_radius(test_radius, test_shape)) {
      low_radius = test_radius;
      best_shape = test_shape;
    }
    else {
      high_radius = test_radius;
    }
  }

  r_applied = true;
  return best_shape;
}

template<typename EdgeRefinder>
static TopoDS_Shape apply_edge_blend_steps(const TopoDS_Shape &shape,
                                           const float radius_limit,
                                           const Span<NurbBodyBlendStep> steps,
                                           const Vector<TopoDS_Edge> &reference_edges,
                                           const Vector<TopoDS_Edge> &initial_current_edges,
                                           const bool initial_edges_match_reference,
                                           const bool validate_shape,
                                           const bool fast_preview,
                                           EdgeRefinder &&refind_edges)
{
  if (steps.is_empty()) {
    return shape;
  }

  TopoDS_Shape result = shape;
  Vector<TopoDS_Edge> current_edges = initial_current_edges;
  Vector<NurbBodyEdgeReference> edge_references;
  Vector<NurbBodyEdgeReference> current_edge_references;
  Vector<char> current_edge_reference_ready;
  bool edge_references_ready = false;
  bool all_current_edge_references_ready = false;
  bool current_edges_match_input = initial_edges_match_reference;
  const ChFi3d_FilletShape fillet_shape = fast_preview ? ChFi3d_Polynomial : ChFi3d_Rational;
  auto reset_current_edge_references = [&]() {
    current_edge_references.clear();
    current_edge_reference_ready.clear();
    all_current_edge_references_ready = false;
  };
  auto ensure_current_edge_reference_storage = [&]() {
    if (current_edge_references.size() != current_edges.size()) {
      current_edge_references.resize(current_edges.size());
      current_edge_reference_ready.resize(current_edges.size());
      for (char &ready : current_edge_reference_ready) {
        ready = false;
      }
      all_current_edge_references_ready = false;
    }
  };
  auto current_edge_reference = [&](const int edge_index) -> const NurbBodyEdgeReference & {
    ensure_current_edge_reference_storage();
    if (!current_edge_reference_ready[edge_index]) {
      current_edge_references[edge_index] = edge_reference_for_blend(current_edges[edge_index]);
      current_edge_reference_ready[edge_index] = true;
    }
    return current_edge_references[edge_index];
  };
  auto ensure_current_edge_references = [&]() {
    if (!all_current_edge_references_ready) {
      ensure_current_edge_reference_storage();
      for (const int edge_index : current_edges.index_range()) {
        current_edge_reference(edge_index);
      }
      all_current_edge_references_ready = true;
    }
  };
  auto current_edge_for_step = [&](const NurbBodyBlendStep &step) {
    if (current_edges_match_input && step.edge_index >= 0 && step.edge_index < current_edges.size())
    {
      return current_edges[step.edge_index].IsNull() ? -1 : step.edge_index;
    }
    if (!edge_references_ready) {
      edge_references = edge_references_for_blend(reference_edges);
      edge_references_ready = true;
    }
    if (step.edge_index < 0 || step.edge_index >= edge_references.size()) {
      return -1;
    }
    ensure_current_edge_references();
    return find_current_edge_by_reference(current_edge_references.as_span(),
                                          step.edge_index,
                                          edge_references[step.edge_index]);
  };
  for (int step_i = 0; step_i < steps.size();) {
    const NurbBodyBlendStep &step = steps[step_i];
    if (current_edges.is_empty() || step.edge_index < 0) {
      step_i++;
      continue;
    }

    Vector<TopoDS_Edge> batch_edges;
    Vector<NurbBodyBlendCandidate> batch_candidates;
    double single_hard_limit = -1.0;
    int batch_end = step_i;
    for (; batch_end < steps.size(); batch_end++) {
      const NurbBodyBlendStep &candidate = steps[batch_end];
      if (!blend_steps_can_share_builder(step, candidate)) {
        break;
      }
      const int candidate_current_edge = current_edge_for_step(candidate);
      if (candidate_current_edge < 0 || candidate_current_edge >= current_edges.size()) {
        break;
      }
      const TopoDS_Edge &candidate_edge = current_edges[candidate_current_edge];
      if (candidate_edge.IsNull() || edge_already_listed(batch_edges.as_span(), candidate_edge)) {
        break;
      }
      const double hard_limit = safe_blend_radius_limit(radius_limit);
      if (batch_edges.is_empty()) {
        single_hard_limit = hard_limit;
      }
      const double candidate_radius = std::min(double(candidate.radius), hard_limit);
      if (candidate_radius <= 0.0) {
        break;
      }
      batch_edges.append(candidate_edge);
      NurbBodyBlendCandidate blend_candidate;
      blend_candidate.edge = candidate_edge;
      blend_candidate.radius = candidate_radius;
      blend_candidate.source_index = candidate.edge_index;
      batch_candidates.append(blend_candidate);
    }
    if (fast_preview && batch_candidates.size() > 1) {
      TopoDS_Shape batch_shape;
      const bool batch_applied = try_edge_set_blend(result,
                                                    step.bevel_type,
                                                    batch_candidates.as_span(),
                                                    validate_shape,
                                                    false,
                                                    fillet_shape,
                                                    batch_shape) ||
                                 (!fast_preview &&
                                  try_edge_set_blend(result,
                                                     step.bevel_type,
                                                     batch_candidates.as_span(),
                                                     validate_shape,
                                                     true,
                                                     fillet_shape,
                                                     batch_shape));
      if (batch_applied) {
        result = batch_shape;
        if (batch_end < steps.size()) {
          current_edges = refind_edges(result);
          reset_current_edge_references();
          current_edges_match_input = false;
        }
        step_i = batch_end;
        continue;
      }
    }

    const int current_edge = current_edge_for_step(step);
    if (current_edge < 0 || current_edge >= current_edges.size() ||
        current_edges[current_edge].IsNull())
    {
      step_i++;
      continue;
    }

    bool applied = false;
    result = apply_single_edge_blend(
        result,
        step.radius,
        step.bevel_type,
        current_edges[current_edge],
        step.edge_index,
        validate_shape,
        fast_preview,
        single_hard_limit > 0.0 ? single_hard_limit :
                                  safe_blend_radius_limit(radius_limit),
        applied);
    if (applied) {
      if (step_i + 1 < steps.size()) {
        current_edges = refind_edges(result);
        reset_current_edge_references();
        current_edges_match_input = false;
      }
    }
    step_i++;
  }
  return result;
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
                                     const bool validate_shape,
                                     const bool fast_preview,
                                     EdgeRefinder &&refind_edges)
{
  Vector<NurbBodyBlendStep> steps = sorted_blend_steps(bevel_radius,
                                                       bevel_radii,
                                                       bevel_type,
                                                       bevel_edges,
                                                       chamfer_edges,
                                                       selected_edges,
                                                       bevel_edge,
                                                       selected_edge,
                                                       bevel_order,
                                                       edges.size());
  return apply_edge_blend_steps(shape,
                                radius_limit,
                                steps.as_span(),
                                edges,
                                edges,
                                true,
                                validate_shape,
                                fast_preview,
                                std::forward<EdgeRefinder>(refind_edges));
}

static TopoDS_Shape apply_stable_surface_edge_blend_steps(
    const TopoDS_Shape &shape,
    const float radius_limit,
    const Span<NurbBodyBlendStep> steps,
    const uint64_t *edge_keys,
    const int samples_per_edge,
    const bool validate_shape,
    const bool fast_preview,
    uint64_t *r_applied_edges,
    const Vector<NurbBodySurfaceEdgeEntry> *initial_edges)
{
  if (r_applied_edges != nullptr) {
    *r_applied_edges = 0;
  }
  if (steps.is_empty()) {
    return shape;
  }

  TopoDS_Shape result = shape;
  Vector<NurbBodySurfaceEdgeEntry> current_edges =
      initial_edges != nullptr ? *initial_edges :
                                 find_selectable_surface_edge_catalog(result, samples_per_edge);
  const ChFi3d_FilletShape fillet_shape = fast_preview ? ChFi3d_Polynomial : ChFi3d_Rational;
  for (int step_i = 0; step_i < steps.size();) {
    const NurbBodyBlendStep &step = steps[step_i];
    if (step.edge_index < 0 || step.edge_index >= 64 || edge_keys == nullptr ||
        edge_keys[step.edge_index] == 0) {
      step_i++;
      continue;
    }
    const int current_edge = find_catalog_edge_for_key(current_edges, edge_keys[step.edge_index]);
    if (current_edge == -1) {
      step_i++;
      continue;
    }
    Vector<TopoDS_Edge> batch_edges;
    Vector<NurbBodyBlendCandidate> batch_candidates;
    Vector<int> batch_edge_indices;
    double single_hard_limit = -1.0;
    int batch_end = step_i;
    for (; batch_end < steps.size(); batch_end++) {
      const NurbBodyBlendStep &candidate = steps[batch_end];
      if (!blend_steps_can_share_builder(step, candidate)) {
        break;
      }
      if (candidate.edge_index < 0 || candidate.edge_index >= 64 || edge_keys == nullptr ||
          edge_keys[candidate.edge_index] == 0)
      {
        break;
      }
      const int candidate_current_edge = find_catalog_edge_for_key(
          current_edges, edge_keys[candidate.edge_index]);
      if (candidate_current_edge == -1) {
        break;
      }
      const NurbBodySurfaceEdgeEntry &candidate_entry = current_edges[candidate_current_edge];
      const TopoDS_Edge &candidate_edge = candidate_entry.edge;
      if (candidate_edge.IsNull() || edge_already_listed(batch_edges.as_span(), candidate_edge)) {
        break;
      }
      const double hard_limit = safe_blend_radius_limit(radius_limit);
      if (batch_edges.is_empty()) {
        single_hard_limit = hard_limit;
      }
      const double candidate_radius = std::min(double(candidate.radius), hard_limit);
      if (candidate_radius <= 0.0) {
        break;
      }
      batch_edges.append(candidate_edge);
      NurbBodyBlendCandidate blend_candidate;
      blend_candidate.edge = candidate_edge;
      blend_candidate.radius = candidate_radius;
      blend_candidate.source_index = candidate.edge_index;
      batch_candidates.append(blend_candidate);
      batch_edge_indices.append(candidate.edge_index);
    }
    if (fast_preview && batch_candidates.size() > 1) {
      TopoDS_Shape batch_shape;
      const bool batch_applied = try_edge_set_blend(result,
                                                    step.bevel_type,
                                                    batch_candidates.as_span(),
                                                    validate_shape,
                                                    false,
                                                    fillet_shape,
                                                    batch_shape) ||
                                 (!fast_preview &&
                                  try_edge_set_blend(result,
                                                     step.bevel_type,
                                                     batch_candidates.as_span(),
                                                     validate_shape,
                                                     true,
                                                     fillet_shape,
                                                     batch_shape));
      if (batch_applied) {
        result = batch_shape;
        if (r_applied_edges != nullptr) {
          for (const int edge_index : batch_edge_indices) {
            *r_applied_edges |= nurb_body_edge_mask_for_index(edge_index);
          }
        }
        if (batch_end < steps.size()) {
          current_edges = find_selectable_surface_edge_catalog(result, samples_per_edge);
        }
        step_i = batch_end;
        continue;
      }
    }

    bool applied = false;
    result = apply_single_edge_blend(result,
                                     step.radius,
                                     step.bevel_type,
                                     current_edges[current_edge].edge,
                                     step.edge_index,
                                     validate_shape,
                                     fast_preview,
                                     single_hard_limit > 0.0 ?
                                         single_hard_limit :
                                         safe_blend_radius_limit(radius_limit),
                                     applied);
    if (applied) {
      if (r_applied_edges != nullptr) {
        *r_applied_edges |= nurb_body_edge_mask_for_index(step.edge_index);
      }
      if (step_i + 1 < steps.size()) {
        current_edges = find_selectable_surface_edge_catalog(result, samples_per_edge);
      }
    }
    step_i++;
  }
  return result;
}

static TopoDS_Shape apply_stable_surface_edge_blend(const TopoDS_Shape &shape,
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
                                                    const uint64_t *edge_keys,
                                                    const int samples_per_edge,
                                                    const bool validate_shape,
                                                    const bool fast_preview,
                                                    uint64_t *r_applied_edges)
{
  Vector<NurbBodyBlendStep> steps = sorted_blend_steps(bevel_radius,
                                                       bevel_radii,
                                                       bevel_type,
                                                       bevel_edges,
                                                       chamfer_edges,
                                                       selected_edges,
                                                       bevel_edge,
                                                       selected_edge,
                                                       bevel_order,
                                                       64);
  return apply_stable_surface_edge_blend_steps(shape,
                                               radius_limit,
                                               steps.as_span(),
                                               edge_keys,
                                               samples_per_edge,
                                               validate_shape,
                                               fast_preview,
                                               r_applied_edges,
                                               nullptr);
}

static TopoDS_Shape make_boolean_op_tool_shape(const NurbBodyBooleanOp &op,
                                               const bool validate_shape,
                                               const bool fast_preview)
{
  const float operand_radius = nurb_body_boolean_op_scaled_radius(op);
  const float radial_scale = nurb_body_boolean_op_radial_scale(op);
  const float operand_blend_radius_limit = nurb_body_boolean_op_scaled_blend_radius_limit(op);

  float operand_bevel_radii[64];
  float operand_surface_bevel_radii[64];
  float operand_dimensions[3];
  scaled_edge_radii(op.operand_bevel_radii, radial_scale, operand_bevel_radii);
  scaled_edge_radii(op.operand_surface_bevel_radii, radial_scale, operand_surface_bevel_radii);
  scaled_primitive_dimensions(op, operand_dimensions);

  TopoDS_Shape shape = make_primitive_shape(op.primitive,
                                            operand_radius,
                                            nurb_body_boolean_op_scaled_depth(op),
                                            op.operand_minor_radius * radial_scale,
                                            operand_dimensions,
                                            float3(0.0f));

  const float operand_bevel_radius = op.operand_bevel_radius * radial_scale;
  if (blend_settings_may_change_shape(operand_bevel_radius,
                                      operand_bevel_radii,
                                      op.operand_bevel_edges,
                                      op.operand_selected_edges,
                                      op.operand_bevel_edge,
                                      op.operand_selected_edge))
  {
    const Vector<TopoDS_Edge> body_edges = find_selectable_surface_edges(shape);
    shape = apply_edge_blend(shape,
                             operand_blend_radius_limit,
                             operand_bevel_radius,
                             operand_bevel_radii,
                             op.operand_bevel_type,
                             op.operand_bevel_edges,
                             op.operand_chamfer_edges,
                             op.operand_bevel_order,
                             op.operand_selected_edges,
                             op.operand_bevel_edge,
                             op.operand_selected_edge,
                             body_edges,
                             validate_shape,
                             fast_preview,
                             [](const TopoDS_Shape &shape) {
                               return find_selectable_surface_edges(shape);
                             });
  }

  const float operand_surface_bevel_radius = op.operand_surface_bevel_radius * radial_scale;
  if (blend_settings_may_change_shape(operand_surface_bevel_radius,
                                      operand_surface_bevel_radii,
                                      op.operand_surface_bevel_edges,
                                      op.operand_surface_selected_edges,
                                      op.operand_surface_bevel_edge,
                                      op.operand_surface_selected_edge))
  {
    uint64_t surface_edge_keys[64];
    std::copy_n(op.operand_surface_edge_keys, 64, surface_edge_keys);
    if (nurb_body_boolean_op_has_non_unit_scale(op)) {
      TopoDS_Shape unscaled_shape = make_boolean_op_primitive_shape(op);
      if (blend_settings_may_change_shape(op.operand_bevel_radius,
                                          op.operand_bevel_radii,
                                          op.operand_bevel_edges,
                                          op.operand_selected_edges,
                                          op.operand_bevel_edge,
                                          op.operand_selected_edge))
      {
        const Vector<TopoDS_Edge> unscaled_body_edges = find_selectable_surface_edges(
            unscaled_shape);
        unscaled_shape = apply_edge_blend(unscaled_shape,
                                          nurb_body_primitive_blend_radius_limit(
                                              op.primitive,
                                              op.operand_radius,
                                              op.operand_depth,
                                              op.operand_minor_radius,
                                              op.operand_dimensions),
                                          op.operand_bevel_radius,
                                          op.operand_bevel_radii,
                                          op.operand_bevel_type,
                                          op.operand_bevel_edges,
                                          op.operand_chamfer_edges,
                                          op.operand_bevel_order,
                                          op.operand_selected_edges,
                                          op.operand_bevel_edge,
                                          op.operand_selected_edge,
                                          unscaled_body_edges,
                                          validate_shape,
                                          fast_preview,
                                          [](const TopoDS_Shape &shape) {
                                            return find_selectable_surface_edges(shape);
                                          });
      }

      const Vector<NurbBodySurfaceEdgeEntry> unscaled_edges =
          find_selectable_surface_edge_catalog(unscaled_shape, 64);
      const Vector<NurbBodySurfaceEdgeEntry> scaled_edges =
          find_selectable_surface_edge_catalog(shape, 64);
      for (int i = 0; i < 64; i++) {
        if (op.operand_surface_edge_keys[i] == 0) {
          continue;
        }
        const int unscaled_edge = find_catalog_edge_for_key(unscaled_edges,
                                                           op.operand_surface_edge_keys[i]);
        if (unscaled_edge >= 0 && unscaled_edge < scaled_edges.size() &&
            scaled_edges[unscaled_edge].edge_key != 0)
        {
          surface_edge_keys[i] = scaled_edges[unscaled_edge].edge_key;
        }
      }
    }

    shape = apply_stable_surface_edge_blend(shape,
                                            operand_blend_radius_limit,
                                            operand_surface_bevel_radius,
                                            operand_surface_bevel_radii,
                                            op.operand_surface_bevel_type,
                                            op.operand_surface_bevel_edges,
                                            op.operand_surface_chamfer_edges,
                                            op.operand_surface_bevel_order,
                                            op.operand_surface_selected_edges,
                                            op.operand_surface_bevel_edge,
                                            op.operand_surface_selected_edge,
                                            surface_edge_keys,
                                            64,
                                            validate_shape,
                                            fast_preview,
                                            nullptr);
  }

  float operand_transform[4][4];
  nurb_body_boolean_op_transform_matrix(op, operand_transform);
  return transform_shape(shape, operand_transform);
}

static void nurb_body_stage_hash_bytes(uint64_t &hash, const void *data, const size_t size)
{
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < size; i++) {
    hash ^= uint64_t(bytes[i]);
    hash *= 1099511628211ull;
  }
}

template<typename T> static void nurb_body_stage_hash_value(uint64_t &hash, const T &value)
{
  nurb_body_stage_hash_bytes(hash, &value, sizeof(T));
}

static void nurb_body_stage_hash_body_base(uint64_t &hash,
                                           const NurbBody &body,
                                           const bool include_surface_blend)
{
  nurb_body_stage_hash_value(hash, body.primitive);
  nurb_body_stage_hash_value(hash, body.radius);
  nurb_body_stage_hash_value(hash, body.depth);
  nurb_body_stage_hash_value(hash, body.minor_radius);
  nurb_body_stage_hash_bytes(hash, body.dimensions, sizeof(body.dimensions));
  nurb_body_stage_hash_value(hash, body.flag & NURB_BODY_FAST_BEVEL_PREVIEW);
  nurb_body_stage_hash_value(hash, body.bevel_edges);
  if (body.bevel_edges == 0) {
    nurb_body_stage_hash_value(hash, body.bevel_edge);
  }
  nurb_body_stage_hash_value(hash, body.chamfer_edges);
  nurb_body_stage_hash_value(hash, body.bevel_type);
  nurb_body_stage_hash_value(hash, body.bevel_radius);
  nurb_body_stage_hash_bytes(hash, body.bevel_radii, sizeof(body.bevel_radii));
  nurb_body_stage_hash_bytes(hash, body.bevel_order, sizeof(body.bevel_order));
  if (blend_selection_affects_shape(body.bevel_radius,
                                    body.bevel_radii,
                                    body.bevel_edges,
                                    body.selected_edges,
                                    body.bevel_edge,
                                    body.selected_edge))
  {
    nurb_body_stage_hash_value(hash, body.selected_edges);
    nurb_body_stage_hash_value(hash, body.selected_edge);
  }

  if (!include_surface_blend) {
    return;
  }

  nurb_body_stage_hash_value(hash, body.surface_bevel_edges);
  if (body.surface_bevel_edges == 0) {
    nurb_body_stage_hash_value(hash, body.surface_bevel_edge);
  }
  nurb_body_stage_hash_value(hash, body.surface_chamfer_edges);
  nurb_body_stage_hash_value(hash, body.surface_bevel_type);
  nurb_body_stage_hash_value(hash, body.surface_bevel_radius);
  nurb_body_stage_hash_bytes(hash, body.surface_bevel_radii, sizeof(body.surface_bevel_radii));
  nurb_body_stage_hash_bytes(hash, body.surface_bevel_order, sizeof(body.surface_bevel_order));
  nurb_body_stage_hash_bytes(hash, body.surface_edge_keys, sizeof(body.surface_edge_keys));
  if (blend_selection_affects_shape(body.surface_bevel_radius,
                                    body.surface_bevel_radii,
                                    body.surface_bevel_edges,
                                    body.surface_selected_edges,
                                    body.surface_bevel_edge,
                                    body.surface_selected_edge))
  {
    nurb_body_stage_hash_value(hash, body.surface_selected_edges);
    nurb_body_stage_hash_value(hash, body.surface_selected_edge);
  }
}

static void nurb_body_stage_hash_boolean_tool(uint64_t &hash, const NurbBodyBooleanOp &op)
{
  nurb_body_stage_hash_value(hash, op.operation);
  nurb_body_stage_hash_value(hash, op.primitive);
  nurb_body_stage_hash_value(hash, op.operand_radius);
  nurb_body_stage_hash_value(hash, op.operand_depth);
  nurb_body_stage_hash_value(hash, op.operand_minor_radius);
  nurb_body_stage_hash_bytes(hash, op.operand_dimensions, sizeof(op.operand_dimensions));
  nurb_body_stage_hash_bytes(hash, op.operand_to_target, sizeof(op.operand_to_target));
  nurb_body_stage_hash_bytes(hash, op.operand_scale, sizeof(op.operand_scale));

  nurb_body_stage_hash_value(hash, op.operand_bevel_edges);
  nurb_body_stage_hash_value(hash, op.operand_chamfer_edges);
  nurb_body_stage_hash_value(hash, op.operand_surface_bevel_edges);
  nurb_body_stage_hash_value(hash, op.operand_surface_chamfer_edges);
  if (op.operand_bevel_edges == 0) {
    nurb_body_stage_hash_value(hash, op.operand_bevel_edge);
  }
  nurb_body_stage_hash_value(hash, op.operand_bevel_type);
  if (op.operand_surface_bevel_edges == 0) {
    nurb_body_stage_hash_value(hash, op.operand_surface_bevel_edge);
  }
  nurb_body_stage_hash_value(hash, op.operand_surface_bevel_type);
  nurb_body_stage_hash_value(hash, op.operand_bevel_radius);
  nurb_body_stage_hash_value(hash, op.operand_surface_bevel_radius);
  nurb_body_stage_hash_bytes(hash, op.operand_bevel_radii, sizeof(op.operand_bevel_radii));
  nurb_body_stage_hash_bytes(
      hash, op.operand_surface_bevel_radii, sizeof(op.operand_surface_bevel_radii));
  nurb_body_stage_hash_bytes(hash, op.operand_bevel_order, sizeof(op.operand_bevel_order));
  nurb_body_stage_hash_bytes(
      hash, op.operand_surface_bevel_order, sizeof(op.operand_surface_bevel_order));
  nurb_body_stage_hash_bytes(
      hash, op.operand_surface_edge_keys, sizeof(op.operand_surface_edge_keys));

  if (blend_selection_affects_shape(op.operand_bevel_radius,
                                    op.operand_bevel_radii,
                                    op.operand_bevel_edges,
                                    op.operand_selected_edges,
                                    op.operand_bevel_edge,
                                    op.operand_selected_edge))
  {
    nurb_body_stage_hash_value(hash, op.operand_selected_edges);
    nurb_body_stage_hash_value(hash, op.operand_selected_edge);
  }
  if (blend_selection_affects_shape(op.operand_surface_bevel_radius,
                                    op.operand_surface_bevel_radii,
                                    op.operand_surface_bevel_edges,
                                    op.operand_surface_selected_edges,
                                    op.operand_surface_bevel_edge,
                                    op.operand_surface_selected_edge))
  {
    nurb_body_stage_hash_value(hash, op.operand_surface_selected_edges);
    nurb_body_stage_hash_value(hash, op.operand_surface_selected_edge);
  }
}

static void nurb_body_stage_hash_boolean_output_blend(uint64_t &hash,
                                                      const NurbBodyBooleanOp &op)
{
  nurb_body_stage_hash_value(hash, op.bevel_edges);
  nurb_body_stage_hash_value(hash, op.chamfer_edges);
  if (op.bevel_edges == 0) {
    nurb_body_stage_hash_value(hash, op.bevel_edge);
  }
  nurb_body_stage_hash_value(hash, op.bevel_type);
  nurb_body_stage_hash_value(hash, op.bevel_radius);
  nurb_body_stage_hash_bytes(hash, op.bevel_radii, sizeof(op.bevel_radii));
  nurb_body_stage_hash_bytes(hash, op.bevel_order, sizeof(op.bevel_order));
  if (blend_selection_affects_shape(op.bevel_radius,
                                    op.bevel_radii,
                                    op.bevel_edges,
                                    op.selected_edges,
                                    op.bevel_edge,
                                    op.selected_edge))
  {
    nurb_body_stage_hash_value(hash, op.selected_edges);
    nurb_body_stage_hash_value(hash, op.selected_edge);
  }
}

static bool nurb_body_stage_hash_edge_blend_without_active(uint64_t &hash,
                                                           const float bevel_radius,
                                                           const float *bevel_radii,
                                                           const int bevel_type,
                                                           const uint64_t bevel_edges,
                                                           const uint64_t chamfer_edges,
                                                           const int *bevel_order,
                                                           const uint64_t selected_edges,
                                                           const int bevel_edge,
                                                           const int selected_edge,
                                                           const int edges_num)
{
  Vector<NurbBodyBlendStep> steps = sorted_blend_steps(bevel_radius,
                                                       bevel_radii,
                                                       bevel_type,
                                                       bevel_edges,
                                                       chamfer_edges,
                                                       selected_edges,
                                                       bevel_edge,
                                                       selected_edge,
                                                       bevel_order,
                                                       edges_num);
  const int active_edge = active_blend_edge_index(bevel_edge, selected_edge);
  const int active_pos = active_blend_step_position(steps.as_span(), active_edge);
  if (active_pos == -1) {
    return false;
  }

  const Vector<NurbBodyBlendStep> stable_steps = stable_blend_steps_without_active(
      steps.as_span(), active_pos);
  nurb_body_stage_hash_value(hash, stable_steps.size());
  for (const NurbBodyBlendStep &step : stable_steps) {
    nurb_body_stage_hash_value(hash, step.edge_index);
    nurb_body_stage_hash_value(hash, step.order);
    nurb_body_stage_hash_value(hash, step.bevel_type);
    nurb_body_stage_hash_value(hash, step.radius);
  }
  return true;
}

static uint64_t nurb_body_body_edge_blend_base_cache_key(const NurbBody &body,
                                                         const bool fast_preview)
{
  uint64_t hash = 1469598103934665603ull;
  const uint64_t domain = fast_preview ? 0x72ad211a5d037a91ull : 0x366a820fe9e37f4dull;
  nurb_body_stage_hash_value(hash, domain);
  nurb_body_stage_hash_value(hash, body.primitive);
  nurb_body_stage_hash_value(hash, body.radius);
  nurb_body_stage_hash_value(hash, body.depth);
  nurb_body_stage_hash_value(hash, body.minor_radius);
  nurb_body_stage_hash_bytes(hash, body.dimensions, sizeof(body.dimensions));
  if (!nurb_body_stage_hash_edge_blend_without_active(hash,
                                                      body.bevel_radius,
                                                      body.bevel_radii,
                                                      body.bevel_type,
                                                      body.bevel_edges,
                                                      body.chamfer_edges,
                                                      body.bevel_order,
                                                      body.selected_edges,
                                                      body.bevel_edge,
                                                      body.selected_edge,
                                                      64))
  {
    return 0;
  }
  return hash;
}

static uint64_t nurb_body_pre_boolean_output_blend_cache_key(
    const NurbBody &body, const NurbBodyBooleanOp *target_op);

static uint64_t nurb_body_boolean_output_blend_base_cache_key(const NurbBody &body,
                                                              const NurbBodyBooleanOp &op,
                                                              const bool fast_preview)
{
  const uint64_t pre_output_key = nurb_body_pre_boolean_output_blend_cache_key(body, &op);
  if (pre_output_key == 0) {
    return 0;
  }
  uint64_t hash = 1469598103934665603ull;
  const uint64_t domain = fast_preview ? 0x43b4f98e12b7df23ull : 0x6ef38dd64bc93423ull;
  nurb_body_stage_hash_value(hash, domain);
  nurb_body_stage_hash_value(hash, pre_output_key);
  if (!nurb_body_stage_hash_edge_blend_without_active(hash,
                                                      op.bevel_radius,
                                                      op.bevel_radii,
                                                      op.bevel_type,
                                                      op.bevel_edges,
                                                      op.chamfer_edges,
                                                      op.bevel_order,
                                                      op.selected_edges,
                                                      op.bevel_edge,
                                                      op.selected_edge,
                                                      64))
  {
    return 0;
  }
  return hash;
}

static uint64_t nurb_body_pre_boolean_output_blend_cache_key(
    const NurbBody &body, const NurbBodyBooleanOp *target_op)
{
  uint64_t hash = 1469598103934665603ull;
  constexpr uint64_t domain = 0x9ad0608f03a8f54bull;
  nurb_body_stage_hash_value(hash, domain);
  nurb_body_stage_hash_body_base(hash, body, true);

  int op_count = 0;
  for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(body.boolean_ops.first);
       op;
       op = op->next)
  {
    op_count++;
    nurb_body_stage_hash_boolean_tool(hash, *op);
    if (op == target_op) {
      break;
    }
    nurb_body_stage_hash_boolean_output_blend(hash, *op);
  }
  nurb_body_stage_hash_value(hash, op_count);
  return hash;
}

static uint64_t nurb_body_pre_surface_blend_cache_key(const NurbBody &body)
{
  uint64_t hash = 1469598103934665603ull;
  constexpr uint64_t domain = 0x187a36b39fe72ac5ull;
  nurb_body_stage_hash_value(hash, domain);
  nurb_body_stage_hash_body_base(hash, body, false);

  int op_count = 0;
  for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(body.boolean_ops.first);
       op;
       op = op->next)
  {
    op_count++;
    nurb_body_stage_hash_boolean_tool(hash, *op);
    nurb_body_stage_hash_boolean_output_blend(hash, *op);
  }
  nurb_body_stage_hash_value(hash, op_count);
  return hash;
}

static uint64_t nurb_body_surface_edge_blend_base_cache_key(const NurbBody &body,
                                                            const bool fast_preview)
{
  const uint64_t pre_surface_key = nurb_body_pre_surface_blend_cache_key(body);
  if (pre_surface_key == 0) {
    return 0;
  }
  uint64_t hash = 1469598103934665603ull;
  const uint64_t domain = fast_preview ? 0xbd76169c080d2b51ull : 0xaf38bc449d2d8d17ull;
  nurb_body_stage_hash_value(hash, domain);
  nurb_body_stage_hash_value(hash, pre_surface_key);
  if (!nurb_body_stage_hash_edge_blend_without_active(hash,
                                                      body.surface_bevel_radius,
                                                      body.surface_bevel_radii,
                                                      body.surface_bevel_type,
                                                      body.surface_bevel_edges,
                                                      body.surface_chamfer_edges,
                                                      body.surface_bevel_order,
                                                      body.surface_selected_edges,
                                                      body.surface_bevel_edge,
                                                      body.surface_selected_edge,
                                                      64))
  {
    return 0;
  }
  return hash;
}

static bool nurb_body_stage_shape_cache_find(const Object *object,
                                             const uint64_t key,
                                             TopoDS_Shape &r_shape);
static bool nurb_body_stage_shape_cache_find_edges(const Object *object,
                                                   const uint64_t key,
                                                   TopoDS_Shape &r_shape,
                                                   Vector<TopoDS_Edge> &r_edges);
static bool nurb_body_stage_shape_cache_find_surface_edges(
    const Object *object,
    const uint64_t key,
    TopoDS_Shape &r_shape,
    Vector<NurbBodySurfaceEdgeEntry> &r_edges);
static void nurb_body_stage_shape_cache_store(const Object *object,
                                              const uint64_t key,
                                              const TopoDS_Shape &shape);
static void nurb_body_stage_shape_cache_store_edges(const Object *object,
                                                    const uint64_t key,
                                                    const TopoDS_Shape &shape,
                                                    const Span<TopoDS_Edge> edges);
static void nurb_body_stage_shape_cache_store_surface_edges(
    const Object *object,
    const uint64_t key,
    const TopoDS_Shape &shape,
    const Span<NurbBodySurfaceEdgeEntry> edges);

template<typename EdgeRefinder>
static TopoDS_Shape apply_edge_blend_local_preview(const TopoDS_Shape &shape,
                                                   const Object *object,
                                                   const uint64_t preview_base_key,
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
                                                   const bool validate_shape,
                                                   const bool fast_preview,
                                                   EdgeRefinder &&refind_edges)
{
  Vector<NurbBodyBlendStep> steps = sorted_blend_steps(bevel_radius,
                                                       bevel_radii,
                                                       bevel_type,
                                                       bevel_edges,
                                                       chamfer_edges,
                                                       selected_edges,
                                                       bevel_edge,
                                                       selected_edge,
                                                       bevel_order,
                                                       edges.size());
  const int active_edge = active_blend_edge_index(bevel_edge, selected_edge);
  const int active_pos = active_blend_step_position(steps.as_span(), active_edge);
  const bool active_is_last_step = active_pos != -1 && active_pos + 1 == int(steps.size());
  const bool can_cache_stable_base = fast_preview || active_is_last_step;
  if (active_pos == -1 || !can_cache_stable_base || object == nullptr || preview_base_key == 0) {
    return apply_edge_blend_steps(shape,
                                  radius_limit,
                                  steps.as_span(),
                                  edges,
                                  edges,
                                  true,
                                  validate_shape,
                                  fast_preview,
                                  refind_edges);
  }

  TopoDS_Shape active_base = shape;
  Vector<TopoDS_Edge> current_edges = edges;
  bool current_edges_match_input = true;
  const Vector<NurbBodyBlendStep> stable_steps = stable_blend_steps_without_active(
      steps.as_span(), active_pos);

  if (!stable_steps.is_empty()) {
    TopoDS_Shape cached_base;
    Vector<TopoDS_Edge> cached_edges;
    if (nurb_body_stage_shape_cache_find_edges(
            object, preview_base_key, cached_base, cached_edges))
    {
      active_base = cached_base;
      current_edges = std::move(cached_edges);
    }
    else {
      if (nurb_body_stage_shape_cache_find(object, preview_base_key, cached_base)) {
        active_base = cached_base;
      }
      else {
        active_base = apply_edge_blend_steps(shape,
                                             radius_limit,
                                             stable_steps.as_span(),
                                             edges,
                                             edges,
                                             true,
                                             validate_shape,
                                             true,
                                             refind_edges);
      }
      current_edges = refind_edges(active_base);
      nurb_body_stage_shape_cache_store_edges(
          object, preview_base_key, active_base, current_edges.as_span());
    }
    current_edges_match_input = false;
  }

  Vector<NurbBodyBlendStep> active_steps;
  active_steps.append(steps[active_pos]);
  return apply_edge_blend_steps(active_base,
                                radius_limit,
                                active_steps.as_span(),
                                edges,
                                current_edges,
                                current_edges_match_input,
                                validate_shape,
                                fast_preview,
                                refind_edges);
}

static TopoDS_Shape apply_stable_surface_edge_blend_local_preview(
    const TopoDS_Shape &shape,
    const Object *object,
    const uint64_t preview_base_key,
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
    const uint64_t *edge_keys,
    const int samples_per_edge,
    const bool validate_shape,
    const bool fast_preview,
    uint64_t *r_applied_edges)
{
  Vector<NurbBodyBlendStep> steps = sorted_blend_steps(bevel_radius,
                                                       bevel_radii,
                                                       bevel_type,
                                                       bevel_edges,
                                                       chamfer_edges,
                                                       selected_edges,
                                                       bevel_edge,
                                                       selected_edge,
                                                       bevel_order,
                                                       64);
  const int active_edge = active_blend_edge_index(bevel_edge, selected_edge);
  const int active_pos = active_blend_step_position(steps.as_span(), active_edge);
  const bool active_is_last_step = active_pos != -1 && active_pos + 1 == int(steps.size());
  const bool can_cache_stable_base = fast_preview || active_is_last_step;
  if (active_pos == -1 || !can_cache_stable_base || object == nullptr || preview_base_key == 0) {
    return apply_stable_surface_edge_blend_steps(shape,
                                                 radius_limit,
                                                 steps.as_span(),
                                                 edge_keys,
                                                 samples_per_edge,
                                                 validate_shape,
                                                 fast_preview,
                                                 r_applied_edges,
                                                 nullptr);
  }

  TopoDS_Shape active_base = shape;
  Vector<NurbBodySurfaceEdgeEntry> current_edges;
  uint64_t stable_applied_edges = 0;
  const Vector<NurbBodyBlendStep> stable_steps = stable_blend_steps_without_active(
      steps.as_span(), active_pos);

  if (!stable_steps.is_empty()) {
    for (const NurbBodyBlendStep &step : stable_steps) {
      stable_applied_edges |= nurb_body_edge_mask_for_index(step.edge_index);
    }

    TopoDS_Shape cached_base;
    Vector<NurbBodySurfaceEdgeEntry> cached_edges;
    if (nurb_body_stage_shape_cache_find_surface_edges(
            object, preview_base_key, cached_base, cached_edges))
    {
      active_base = cached_base;
      current_edges = std::move(cached_edges);
    }
    else {
      if (nurb_body_stage_shape_cache_find(object, preview_base_key, cached_base)) {
        active_base = cached_base;
      }
      else {
        active_base = apply_stable_surface_edge_blend_steps(shape,
                                                            radius_limit,
                                                            stable_steps.as_span(),
                                                            edge_keys,
                                                            samples_per_edge,
                                                            validate_shape,
                                                            true,
                                                            nullptr,
                                                            nullptr);
      }
      current_edges = find_selectable_surface_edge_catalog(active_base, samples_per_edge);
      nurb_body_stage_shape_cache_store_surface_edges(
          object, preview_base_key, active_base, current_edges.as_span());
    }
  }

  Vector<NurbBodyBlendStep> active_steps;
  active_steps.append(steps[active_pos]);
  uint64_t active_applied_edges = 0;
  TopoDS_Shape result = apply_stable_surface_edge_blend_steps(
      active_base,
      radius_limit,
      active_steps.as_span(),
      edge_keys,
      samples_per_edge,
      validate_shape,
      fast_preview,
      r_applied_edges != nullptr ? &active_applied_edges : nullptr,
      !stable_steps.is_empty() ? &current_edges : nullptr);
  if (r_applied_edges != nullptr) {
    *r_applied_edges = stable_applied_edges | active_applied_edges;
  }
  return result;
}

static TopoDS_Shape evaluate_shape(const NurbBody &body, const Object *object)
{
  TopoDS_Shape result = make_body_primitive_shape(body);
  const bool fast_preview = (body.flag & NURB_BODY_FAST_BEVEL_PREVIEW) != 0;
  if (blend_settings_may_change_shape(body.bevel_radius,
                                      body.bevel_radii,
                                      body.bevel_edges,
                                      body.selected_edges,
                                      body.bevel_edge,
                                      body.selected_edge))
  {
    const Vector<TopoDS_Edge> body_edges = find_selectable_surface_edges(result);
    const uint64_t preview_base_key = nurb_body_body_edge_blend_base_cache_key(body,
                                                                               fast_preview);
    result = apply_edge_blend_local_preview(result,
                                            object,
                                            preview_base_key,
                                            nurb_body_blend_radius_limit(body),
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
                                            true,
                                            fast_preview,
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
      const TopoDS_Shape pre_boolean_shape = result;
      const bool output_blend_requested = blend_settings_may_change_shape(op->bevel_radius,
                                                                          op->bevel_radii,
                                                                          op->bevel_edges,
                                                                          op->selected_edges,
                                                                          op->bevel_edge,
                                                                          op->selected_edge);

      const uint64_t pre_output_blend_key = nurb_body_pre_boolean_output_blend_cache_key(body,
                                                                                        op);
      TopoDS_Shape cached_pre_output_blend;
      if (nurb_body_stage_shape_cache_find(object, pre_output_blend_key, cached_pre_output_blend))
      {
        result = cached_pre_output_blend;
      }
      else {
        const TopoDS_Shape tool = make_boolean_op_tool_shape(*op, true, fast_preview);
        result = apply_boolean_operation(result, tool, op->operation);
        nurb_body_stage_shape_cache_store(object, pre_output_blend_key, result);
      }
      if (output_blend_requested) {
        const Vector<uint64_t> pre_boolean_edge_keys = selectable_edge_geometry_keys(
            pre_boolean_shape);
        const float output_blend_radius_limit = std::max(
            nurb_body_blend_radius_limit(body),
            nurb_body_boolean_op_scaled_blend_radius_limit(*op));
        const Vector<TopoDS_Edge> cut_edges = find_boolean_output_edges(
            result, pre_boolean_edge_keys.as_span());
        const uint64_t preview_base_key = nurb_body_boolean_output_blend_base_cache_key(
            body, *op, fast_preview);
        result = apply_edge_blend_local_preview(result,
                                                object,
                                                preview_base_key,
                                                output_blend_radius_limit,
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
                                                true,
                                                fast_preview,
                                                [&](const TopoDS_Shape &shape) {
                                                  return find_boolean_output_edges(
                                                      shape, pre_boolean_edge_keys.as_span());
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
    if (object != nullptr) {
      const uint64_t pre_surface_key = nurb_body_pre_surface_blend_cache_key(body);
      TopoDS_Shape cached_pre_surface;
      if (nurb_body_stage_shape_cache_find(object, pre_surface_key, cached_pre_surface)) {
        result = cached_pre_surface;
      }
      else {
        nurb_body_stage_shape_cache_store(object, pre_surface_key, result);
      }
    }
    const uint64_t preview_base_key = nurb_body_surface_edge_blend_base_cache_key(body,
                                                                                  fast_preview);
    result = apply_stable_surface_edge_blend_local_preview(result,
                                                           object,
                                                           preview_base_key,
                                                           nurb_body_blend_radius_limit(body),
                                                           body.surface_bevel_radius,
                                                           body.surface_bevel_radii,
                                                           body.surface_bevel_type,
                                                           body.surface_bevel_edges,
                                                           body.surface_chamfer_edges,
                                                           body.surface_bevel_order,
                                                           body.surface_selected_edges,
                                                           body.surface_bevel_edge,
                                                           body.surface_selected_edge,
                                                           body.surface_edge_keys,
                                                           64,
                                                           true,
                                                           fast_preview,
                                                           nullptr);
  }

  if (!result.IsNull()) {
    BRepTools::Clean(result);
  }
  return result;
}

static void nurb_body_evaluated_shape_cache_store(const NurbBody &body,
                                                  const Object *object,
                                                  const TopoDS_Shape &shape);

static void sample_boolean_edge_polylines(const NurbBody &body,
                                          const Object *object,
                                          const int samples_per_edge,
                                          Vector<NurbBodyEdgePolyline> &r_polylines)
{
  TopoDS_Shape result = make_body_primitive_shape(body);
  const bool fast_preview = (body.flag & NURB_BODY_FAST_BEVEL_PREVIEW) != 0;
  r_polylines.clear();
  Vector<NurbBodySelectableEdgeRef> selectable_refs;
  const float base_threshold = std::max(0.05f, body.tessellation_deflection * 8.0f);

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
                              body_blended_edges,
                              body.bevel_radius,
                              body.bevel_radii,
                              selectable_refs);
  const uint64_t body_preview_base_key = nurb_body_body_edge_blend_base_cache_key(body,
                                                                                  fast_preview);
  result = apply_edge_blend_local_preview(result,
                                          object,
                                          body_preview_base_key,
                                          nurb_body_blend_radius_limit(body),
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
                                          true,
                                          fast_preview,
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
    const TopoDS_Shape pre_boolean_shape = result;
    const Vector<uint64_t> pre_boolean_edge_keys = selectable_edge_geometry_keys(
        pre_boolean_shape);
    const float output_blend_radius_limit = std::max(
        nurb_body_blend_radius_limit(body), nurb_body_boolean_op_scaled_blend_radius_limit(*op));

    const uint64_t pre_output_blend_key = object != nullptr ?
                                              nurb_body_pre_boolean_output_blend_cache_key(body,
                                                                                          op) :
                                              0;
    TopoDS_Shape cached_pre_output_blend;
    if (nurb_body_stage_shape_cache_find(object, pre_output_blend_key, cached_pre_output_blend)) {
      result = cached_pre_output_blend;
    }
    else {
      const TopoDS_Shape tool = make_boolean_op_tool_shape(*op, true, fast_preview);
      result = apply_boolean_operation(result, tool, op->operation);
      nurb_body_stage_shape_cache_store(object, pre_output_blend_key, result);
    }

    TopTools_IndexedDataMapOfShapeListOfShape op_edge_faces;
    TopExp::MapShapesAndAncestors(result, TopAbs_EDGE, TopAbs_FACE, op_edge_faces);
    const Vector<TopoDS_Edge> cut_edges = find_boolean_output_edges(
        op_edge_faces, pre_boolean_edge_keys.as_span());
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
                                op_blended_edges,
                                op->bevel_radius,
                                op->bevel_radii,
                                selectable_refs);

    const uint64_t preview_base_key = nurb_body_boolean_output_blend_base_cache_key(body,
                                                                                   *op,
                                                                                   fast_preview);
    result = apply_edge_blend_local_preview(result,
                                            object,
                                            preview_base_key,
                                            output_blend_radius_limit,
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
                                            true,
                                            fast_preview,
                                            [&](const TopoDS_Shape &shape) {
                                              return find_boolean_output_edges(
                                                  shape, pre_boolean_edge_keys.as_span());
                                            });
  }

  uint64_t surface_requested_edges = body.surface_bevel_edges;
  if (surface_requested_edges == 0) {
    surface_requested_edges = body.surface_bevel_edge >= 0 ?
                                  nurb_body_edge_mask_for_index(body.surface_bevel_edge) :
                                  body.surface_selected_edges;
  }
  uint64_t surface_applied_edges = 0;
  if (blend_settings_may_change_shape(body.surface_bevel_radius,
                                      body.surface_bevel_radii,
                                      body.surface_bevel_edges,
                                      body.surface_selected_edges,
                                      body.surface_bevel_edge,
                                      body.surface_selected_edge))
  {
    if (object != nullptr) {
      const uint64_t pre_surface_key = nurb_body_pre_surface_blend_cache_key(body);
      TopoDS_Shape cached_pre_surface;
      if (nurb_body_stage_shape_cache_find(object, pre_surface_key, cached_pre_surface)) {
        result = cached_pre_surface;
      }
      else {
        nurb_body_stage_shape_cache_store(object, pre_surface_key, result);
      }
    }
    TopTools_IndexedDataMapOfShapeListOfShape surface_edge_faces;
    TopExp::MapShapesAndAncestors(result, TopAbs_EDGE, TopAbs_FACE, surface_edge_faces);
    const Vector<TopoDS_Edge> surface_edges = find_selectable_surface_edges_indexed(
        surface_edge_faces, samples_per_edge, body);
    const uint64_t preview_base_key = nurb_body_surface_edge_blend_base_cache_key(body,
                                                                                  fast_preview);
    result = apply_stable_surface_edge_blend_local_preview(result,
                                                           object,
                                                           preview_base_key,
                                                           nurb_body_blend_radius_limit(body),
                                                           body.surface_bevel_radius,
                                                           body.surface_bevel_radii,
                                                           body.surface_bevel_type,
                                                           body.surface_bevel_edges,
                                                           body.surface_chamfer_edges,
                                                           body.surface_bevel_order,
                                                           body.surface_selected_edges,
                                                           body.surface_bevel_edge,
                                                           body.surface_selected_edge,
                                                           body.surface_edge_keys,
                                                           samples_per_edge,
                                                           true,
                                                           fast_preview,
                                                           &surface_applied_edges);
    append_selectable_edge_refs(surface_edges,
                                surface_edge_faces,
                                nullptr,
                                NURB_BODY_EDGE_POLYLINE_FINAL,
                                samples_per_edge,
                                base_threshold,
                                surface_applied_edges,
                                surface_requested_edges,
                                body.surface_bevel_radius,
                                body.surface_bevel_radii,
                                selectable_refs);
  }

  triangulate_shape_for_preview(result, body);
  append_shape_surface_edge_polylines(result, samples_per_edge, selectable_refs, body, r_polylines);
  nurb_body_evaluated_shape_cache_store(body, object, result);
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
  triangulate_shape_for_preview(shape, body, false);

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

static int nurb_body_effective_edge_polyline_samples(const NurbBody &body,
                                                     const int samples_per_edge)
{
  UNUSED_VARS(body);
  const int samples = std::max(samples_per_edge, 2);
  return samples;
}

static uint64_t nurb_body_geometry_cache_key(const NurbBody &body,
                                             const Object & /*object*/,
                                             const int samples_per_edge,
                                             const bool include_viewport_mesh_settings)
{
  uint64_t hash = 1469598103934665603ull;
  if (samples_per_edge >= 0) {
    nurb_body_hash_value(hash, samples_per_edge);
  }
  nurb_body_hash_value(hash, body.primitive);
  nurb_body_hash_value(hash, body.radius);
  nurb_body_hash_value(hash, body.depth);
  nurb_body_hash_value(hash, body.minor_radius);
  nurb_body_hash_bytes(hash, body.dimensions, sizeof(body.dimensions));
  nurb_body_hash_value(hash, body.flag & NURB_BODY_FAST_BEVEL_PREVIEW);
  if (include_viewport_mesh_settings) {
    nurb_body_hash_value(hash, body.tessellation_deflection);
    nurb_body_hash_value(hash, body.tessellation_angle);
  }
  nurb_body_hash_value(hash, body.bevel_edges);
  if (body.bevel_edges == 0) {
    nurb_body_hash_value(hash, body.bevel_edge);
  }
  nurb_body_hash_value(hash, body.chamfer_edges);
  nurb_body_hash_value(hash, body.surface_bevel_edges);
  if (body.surface_bevel_edges == 0) {
    nurb_body_hash_value(hash, body.surface_bevel_edge);
  }
  nurb_body_hash_value(hash, body.surface_chamfer_edges);
  nurb_body_hash_value(hash, body.bevel_type);
  nurb_body_hash_value(hash, body.surface_bevel_type);
  nurb_body_hash_value(hash, body.bevel_radius);
  nurb_body_hash_value(hash, body.surface_bevel_radius);
  nurb_body_hash_bytes(hash, body.bevel_radii, sizeof(body.bevel_radii));
  nurb_body_hash_bytes(hash, body.surface_bevel_radii, sizeof(body.surface_bevel_radii));
  nurb_body_hash_bytes(hash, body.bevel_order, sizeof(body.bevel_order));
  nurb_body_hash_bytes(hash, body.surface_bevel_order, sizeof(body.surface_bevel_order));
  nurb_body_hash_bytes(hash, body.surface_edge_keys, sizeof(body.surface_edge_keys));
  if (blend_selection_affects_shape(body.bevel_radius,
                                    body.bevel_radii,
                                    body.bevel_edges,
                                    body.selected_edges,
                                    body.bevel_edge,
                                    body.selected_edge))
  {
    nurb_body_hash_value(hash, body.selected_edges);
    nurb_body_hash_value(hash, body.selected_edge);
  }
  if (blend_selection_affects_shape(body.surface_bevel_radius,
                                    body.surface_bevel_radii,
                                    body.surface_bevel_edges,
                                    body.surface_selected_edges,
                                    body.surface_bevel_edge,
                                    body.surface_selected_edge))
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
    nurb_body_hash_value(hash, op->bevel_edges);
    if (op->bevel_edges == 0) {
      nurb_body_hash_value(hash, op->bevel_edge);
    }
    nurb_body_hash_value(hash, op->chamfer_edges);
    nurb_body_hash_value(hash, op->bevel_type);
    nurb_body_hash_value(hash, op->bevel_radius);
    nurb_body_hash_bytes(hash, op->bevel_radii, sizeof(op->bevel_radii));
    nurb_body_hash_bytes(hash, op->bevel_order, sizeof(op->bevel_order));
    if (blend_selection_affects_shape(op->bevel_radius,
                                      op->bevel_radii,
                                      op->bevel_edges,
                                      op->selected_edges,
                                      op->bevel_edge,
                                      op->selected_edge))
    {
      nurb_body_hash_value(hash, op->selected_edges);
      nurb_body_hash_value(hash, op->selected_edge);
    }
    nurb_body_hash_value(hash, op->operand_radius);
    nurb_body_hash_value(hash, op->operand_depth);
    nurb_body_hash_value(hash, op->operand_minor_radius);
    nurb_body_hash_bytes(hash, op->operand_dimensions, sizeof(op->operand_dimensions));
    nurb_body_hash_bytes(hash, op->operand_to_target, sizeof(op->operand_to_target));
    nurb_body_hash_bytes(hash, op->operand_scale, sizeof(op->operand_scale));
    nurb_body_hash_value(hash, op->operand_bevel_edges);
    nurb_body_hash_value(hash, op->operand_chamfer_edges);
    nurb_body_hash_value(hash, op->operand_surface_bevel_edges);
    nurb_body_hash_value(hash, op->operand_surface_chamfer_edges);
    if (op->operand_bevel_edges == 0) {
      nurb_body_hash_value(hash, op->operand_bevel_edge);
    }
    nurb_body_hash_value(hash, op->operand_bevel_type);
    if (op->operand_surface_bevel_edges == 0) {
      nurb_body_hash_value(hash, op->operand_surface_bevel_edge);
    }
    nurb_body_hash_value(hash, op->operand_surface_bevel_type);
    nurb_body_hash_value(hash, op->operand_bevel_radius);
    nurb_body_hash_value(hash, op->operand_surface_bevel_radius);
    nurb_body_hash_bytes(hash, op->operand_bevel_radii, sizeof(op->operand_bevel_radii));
    nurb_body_hash_bytes(hash,
                         op->operand_surface_bevel_radii,
                         sizeof(op->operand_surface_bevel_radii));
    nurb_body_hash_bytes(hash, op->operand_bevel_order, sizeof(op->operand_bevel_order));
    nurb_body_hash_bytes(hash,
                         op->operand_surface_bevel_order,
                         sizeof(op->operand_surface_bevel_order));
    nurb_body_hash_bytes(hash,
                         op->operand_surface_edge_keys,
                         sizeof(op->operand_surface_edge_keys));
    if (blend_selection_affects_shape(op->operand_bevel_radius,
                                      op->operand_bevel_radii,
                                      op->operand_bevel_edges,
                                      op->operand_selected_edges,
                                      op->operand_bevel_edge,
                                      op->operand_selected_edge))
    {
      nurb_body_hash_value(hash, op->operand_selected_edges);
      nurb_body_hash_value(hash, op->operand_selected_edge);
    }
    if (blend_selection_affects_shape(op->operand_surface_bevel_radius,
                                      op->operand_surface_bevel_radii,
                                      op->operand_surface_bevel_edges,
                                      op->operand_surface_selected_edges,
                                      op->operand_surface_bevel_edge,
                                      op->operand_surface_selected_edge))
    {
      nurb_body_hash_value(hash, op->operand_surface_selected_edges);
      nurb_body_hash_value(hash, op->operand_surface_selected_edge);
    }
  }
  nurb_body_hash_value(hash, op_count);
  return hash;
}

static uint64_t nurb_body_shape_cache_key(const NurbBody &body, const Object &object)
{
  return nurb_body_geometry_cache_key(body, object, -1, true);
}

static uint64_t nurb_body_edge_polyline_cache_key(const NurbBody &body,
                                                  const Object &object,
                                                  const int samples_per_edge)
{
  return nurb_body_geometry_cache_key(body, object, samples_per_edge, true);
}

struct NurbBodyEdgePolylineCache {
  const Object *object = nullptr;
  uint64_t key = 0;
  Vector<NurbBodyEdgePolyline> polylines;
};

struct NurbBodyEvaluatedShapeCache {
  const Object *object = nullptr;
  uint64_t key = 0;
  TopoDS_Shape shape;
};

struct NurbBodyStageShapeCache {
  const Object *object = nullptr;
  uint64_t key = 0;
  TopoDS_Shape shape;
  Vector<TopoDS_Edge> edges;
  Vector<NurbBodySurfaceEdgeEntry> surface_edges;
};

struct NurbBodyHoveredEdgeKey {
  const Object *object = nullptr;
  const NurbBodyBooleanOp *op = nullptr;
  int edge_index = -1;
  int flag = 0;
  uint64_t edge_key = 0;
};

static Vector<NurbBodyEvaluatedShapeCache> g_nurb_body_evaluated_shape_caches;
static Vector<NurbBodyStageShapeCache> g_nurb_body_stage_shape_caches;
static Vector<NurbBodyEdgePolylineCache> g_nurb_body_edge_polyline_caches;
static Vector<NurbBodyHoveredEdgeKey> g_nurb_body_hovered_edge_keys;

static uint64_t nurb_body_evaluated_shape_cache_key(const NurbBody &body, const Object &object)
{
  return nurb_body_shape_cache_key(body, object);
}

static NurbBodyEvaluatedShapeCache *nurb_body_evaluated_shape_cache_find(const Object *object)
{
  for (NurbBodyEvaluatedShapeCache &cache : g_nurb_body_evaluated_shape_caches) {
    if (cache.object == object) {
      return &cache;
    }
  }
  return nullptr;
}

static NurbBodyEvaluatedShapeCache &nurb_body_evaluated_shape_cache_ensure(const Object *object)
{
  if (NurbBodyEvaluatedShapeCache *cache = nurb_body_evaluated_shape_cache_find(object)) {
    return *cache;
  }

  constexpr int max_cached_objects = 64;
  if (g_nurb_body_evaluated_shape_caches.size() >= max_cached_objects) {
    g_nurb_body_evaluated_shape_caches.remove_and_reorder(0);
  }

  NurbBodyEvaluatedShapeCache cache;
  cache.object = object;
  g_nurb_body_evaluated_shape_caches.append(std::move(cache));
  return g_nurb_body_evaluated_shape_caches.last();
}

static TopoDS_Shape nurb_body_evaluate_shape_cached(const NurbBody &body, const Object *object)
{
  if (object == nullptr) {
    return evaluate_shape(body, object);
  }

  const uint64_t key = nurb_body_evaluated_shape_cache_key(body, *object);
  NurbBodyEvaluatedShapeCache &cache = nurb_body_evaluated_shape_cache_ensure(object);
  if (cache.key == key && !cache.shape.IsNull()) {
    return cache.shape;
  }

  cache.shape = evaluate_shape(body, object);
  cache.key = key;
  return cache.shape;
}

static void nurb_body_evaluated_shape_cache_store(const NurbBody &body,
                                                  const Object *object,
                                                  const TopoDS_Shape &shape)
{
  if (object == nullptr || shape.IsNull()) {
    return;
  }

  NurbBodyEvaluatedShapeCache &cache = nurb_body_evaluated_shape_cache_ensure(object);
  cache.shape = shape;
  cache.key = nurb_body_evaluated_shape_cache_key(body, *object);
}

static NurbBodyStageShapeCache *nurb_body_stage_shape_cache_find_entry(const Object *object,
                                                                       const uint64_t key)
{
  if (object == nullptr || key == 0) {
    return nullptr;
  }
  for (NurbBodyStageShapeCache &cache : g_nurb_body_stage_shape_caches) {
    if (cache.object == object && cache.key == key) {
      return &cache;
    }
  }
  return nullptr;
}

static bool nurb_body_stage_shape_cache_find(const Object *object,
                                             const uint64_t key,
                                             TopoDS_Shape &r_shape)
{
  if (NurbBodyStageShapeCache *cache = nurb_body_stage_shape_cache_find_entry(object, key)) {
    if (!cache->shape.IsNull()) {
      r_shape = cache->shape;
      return true;
    }
  }
  return false;
}

static bool nurb_body_stage_shape_cache_find_edges(const Object *object,
                                                   const uint64_t key,
                                                   TopoDS_Shape &r_shape,
                                                   Vector<TopoDS_Edge> &r_edges)
{
  if (NurbBodyStageShapeCache *cache = nurb_body_stage_shape_cache_find_entry(object, key)) {
    if (!cache->shape.IsNull() && !cache->edges.is_empty()) {
      r_shape = cache->shape;
      r_edges = cache->edges;
      return true;
    }
  }
  return false;
}

static bool nurb_body_stage_shape_cache_find_surface_edges(
    const Object *object,
    const uint64_t key,
    TopoDS_Shape &r_shape,
    Vector<NurbBodySurfaceEdgeEntry> &r_edges)
{
  if (NurbBodyStageShapeCache *cache = nurb_body_stage_shape_cache_find_entry(object, key)) {
    if (!cache->shape.IsNull() && !cache->surface_edges.is_empty()) {
      r_shape = cache->shape;
      r_edges = cache->surface_edges;
      return true;
    }
  }
  return false;
}

static void nurb_body_stage_shape_cache_store(const Object *object,
                                              const uint64_t key,
                                              const TopoDS_Shape &shape)
{
  if (object == nullptr || key == 0 || shape.IsNull()) {
    return;
  }
  if (NurbBodyStageShapeCache *cache = nurb_body_stage_shape_cache_find_entry(object, key)) {
    cache->shape = shape;
    cache->edges.clear();
    cache->surface_edges.clear();
    return;
  }

  constexpr int max_stage_cache_entries = 192;
  if (g_nurb_body_stage_shape_caches.size() >= max_stage_cache_entries) {
    g_nurb_body_stage_shape_caches.remove_and_reorder(0);
  }

  NurbBodyStageShapeCache cache;
  cache.object = object;
  cache.key = key;
  cache.shape = shape;
  g_nurb_body_stage_shape_caches.append(std::move(cache));
}

static void nurb_body_stage_shape_cache_store_edges(const Object *object,
                                                    const uint64_t key,
                                                    const TopoDS_Shape &shape,
                                                    const Span<TopoDS_Edge> edges)
{
  if (object == nullptr || key == 0 || shape.IsNull()) {
    return;
  }
  if (NurbBodyStageShapeCache *cache = nurb_body_stage_shape_cache_find_entry(object, key)) {
    cache->shape = shape;
    cache->edges.clear();
    cache->edges.extend(edges.data(), edges.size());
    cache->surface_edges.clear();
    return;
  }

  constexpr int max_stage_cache_entries = 192;
  if (g_nurb_body_stage_shape_caches.size() >= max_stage_cache_entries) {
    g_nurb_body_stage_shape_caches.remove_and_reorder(0);
  }

  NurbBodyStageShapeCache cache;
  cache.object = object;
  cache.key = key;
  cache.shape = shape;
  cache.edges.extend(edges.data(), edges.size());
  g_nurb_body_stage_shape_caches.append(std::move(cache));
}

static void nurb_body_stage_shape_cache_store_surface_edges(
    const Object *object,
    const uint64_t key,
    const TopoDS_Shape &shape,
    const Span<NurbBodySurfaceEdgeEntry> edges)
{
  if (object == nullptr || key == 0 || shape.IsNull()) {
    return;
  }
  if (NurbBodyStageShapeCache *cache = nurb_body_stage_shape_cache_find_entry(object, key)) {
    cache->shape = shape;
    cache->edges.clear();
    cache->surface_edges.clear();
    cache->surface_edges.extend(edges.data(), edges.size());
    return;
  }

  constexpr int max_stage_cache_entries = 192;
  if (g_nurb_body_stage_shape_caches.size() >= max_stage_cache_entries) {
    g_nurb_body_stage_shape_caches.remove_and_reorder(0);
  }

  NurbBodyStageShapeCache cache;
  cache.object = object;
  cache.key = key;
  cache.shape = shape;
  cache.surface_edges.extend(edges.data(), edges.size());
  g_nurb_body_stage_shape_caches.append(std::move(cache));
}

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

static int hovered_edge_domain_from_flag(const int flag)
{
  if ((flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0) {
    return NURB_BODY_EDGE_POLYLINE_BODY;
  }
  if ((flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0) {
    return NURB_BODY_EDGE_POLYLINE_FINAL;
  }
  return 0;
}

static NurbBodyHoveredEdgeKey *nurb_body_hovered_edge_key_find(const Object *object)
{
  for (NurbBodyHoveredEdgeKey &hover_key : g_nurb_body_hovered_edge_keys) {
    if (hover_key.object == object) {
      return &hover_key;
    }
  }
  return nullptr;
}

#endif

Mesh *BKE_nurb_body_to_mesh(const NurbBody *body, const Object *object)
{
#ifdef WITH_OPENCASCADE
  try {
    TopoDS_Shape shape = nurb_body_evaluate_shape_cached(*body, object);
    Mesh *mesh = mesh_from_shape(shape, *body);
    nurb_body_evaluated_shape_cache_store(*body, object, shape);
    return mesh;
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
  const int effective_samples = nurb_body_effective_edge_polyline_samples(*body,
                                                                          samples_per_edge);
  const uint64_t cache_key = nurb_body_edge_polyline_cache_key(*body, *object, effective_samples);
  NurbBodyEdgePolylineCache &cache = nurb_body_edge_polyline_cache_ensure(object);
  if (cache.key == cache_key) {
    return cache.polylines.as_span();
  }

  try {
    cache.polylines.clear();
    sample_boolean_edge_polylines(*body, object, effective_samples, cache.polylines);
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
  const int effective_samples = nurb_body_effective_edge_polyline_samples(*body,
                                                                          samples_per_edge);
  return nurb_body_edge_polyline_cache_key(*body, *object, effective_samples);
#else
  UNUSED_VARS(object, samples_per_edge);
  return 0;
#endif
}

bool BKE_nurb_body_hovered_edge_key_set(const Object *object,
                                        const NurbBodyBooleanOp *op,
                                        const int edge_index,
                                        const int flag,
                                        const uint64_t edge_key)
{
#ifdef WITH_OPENCASCADE
  if (object == nullptr || edge_index < 0 || edge_key == 0) {
    return BKE_nurb_body_hovered_edge_key_clear(object);
  }

  const int domain = hovered_edge_domain_from_flag(flag);
  if (NurbBodyHoveredEdgeKey *hover_key = nurb_body_hovered_edge_key_find(object)) {
    const bool changed = hover_key->op != op || hover_key->edge_index != edge_index ||
                         hover_key->flag != domain || hover_key->edge_key != edge_key;
    hover_key->op = op;
    hover_key->edge_index = edge_index;
    hover_key->flag = domain;
    hover_key->edge_key = edge_key;
    return changed;
  }

  NurbBodyHoveredEdgeKey hover_key;
  hover_key.object = object;
  hover_key.op = op;
  hover_key.edge_index = edge_index;
  hover_key.flag = domain;
  hover_key.edge_key = edge_key;
  g_nurb_body_hovered_edge_keys.append(hover_key);
  return true;
#else
  UNUSED_VARS(object, op, edge_index, flag, edge_key);
  return false;
#endif
}

bool BKE_nurb_body_hovered_edge_key_clear(const Object *object)
{
#ifdef WITH_OPENCASCADE
  bool changed = false;
  for (int i = int(g_nurb_body_hovered_edge_keys.size()) - 1; i >= 0; i--) {
    if (g_nurb_body_hovered_edge_keys[i].object == object) {
      g_nurb_body_hovered_edge_keys.remove_and_reorder(i);
      changed = true;
    }
  }
  return changed;
#else
  UNUSED_VARS(object);
  return false;
#endif
}

uint64_t BKE_nurb_body_hovered_edge_key_get(const Object *object)
{
#ifdef WITH_OPENCASCADE
  if (const NurbBodyHoveredEdgeKey *hover_key = nurb_body_hovered_edge_key_find(object)) {
    return hover_key->edge_key;
  }
  return 0;
#else
  UNUSED_VARS(object);
  return 0;
#endif
}

bool BKE_nurb_body_hovered_edge_key_matches(const Object *object,
                                            const NurbBodyEdgePolyline &polyline)
{
#ifdef WITH_OPENCASCADE
  const NurbBodyHoveredEdgeKey *hover_key = nurb_body_hovered_edge_key_find(object);
  if (hover_key == nullptr) {
    return false;
  }

  return hover_key->op == polyline.op && hover_key->edge_index == polyline.edge_index &&
         hover_key->flag == hovered_edge_domain_from_flag(polyline.flag) &&
         hover_key->edge_key != 0 && hover_key->edge_key == polyline.edge_key;
#else
  UNUSED_VARS(object, polyline);
  return false;
#endif
}

static void nurb_body_apply_viewport_tool_settings(NurbBody &body, const Scene *scene)
{
  if (scene == nullptr || scene->toolsettings == nullptr) {
    return;
  }

  const ToolSettings &tool_settings = *scene->toolsettings;
  const bool missing_legacy_tool_values =
      tool_settings.nurb_body_tessellation_deflection <= 0.0f &&
      tool_settings.nurb_body_tessellation_angle <= 0.0f &&
      tool_settings.nurb_body_viewport_flag == 0;
  const float deflection = tool_settings.nurb_body_tessellation_deflection > 0.0f ?
                               tool_settings.nurb_body_tessellation_deflection :
                               0.01f;
  const float angle = tool_settings.nurb_body_tessellation_angle > 0.0f ?
                          tool_settings.nurb_body_tessellation_angle :
                          0.558505f;
  int viewport_flags = tool_settings.nurb_body_viewport_flag &
                       (NURB_BODY_TRIANGULATE_MESH | NURB_BODY_SMOOTH_SHADING);
  if (missing_legacy_tool_values) {
    viewport_flags = NURB_BODY_TRIANGULATE_MESH | NURB_BODY_SMOOTH_SHADING;
  }

  body.tessellation_deflection = std::max(deflection, 0.0001f);
  body.tessellation_angle = std::clamp(angle, 0.01f, 3.14159f);
  body.flag &= ~(NURB_BODY_TRIANGULATE_MESH | NURB_BODY_SMOOTH_SHADING);
  body.flag |= viewport_flags;
}

void BKE_nurb_body_data_update(Depsgraph * /*depsgraph*/, Scene *scene, Object *ob)
{
  NurbBody *body = id_cast<NurbBody *>(ob->data);
  nurb_body_apply_viewport_tool_settings(*body, scene);
  Mesh *mesh = BKE_nurb_body_to_mesh(body, ob);
  BKE_object_eval_assign_data(ob, &mesh->id, true);

  bke::GeometrySet *geometry_set = new bke::GeometrySet();
  geometry_set->replace_mesh(mesh, bke::GeometryOwnershipType::Editable);
  ob->runtime->geometry_set_eval = geometry_set;
}

}  // namespace blender
