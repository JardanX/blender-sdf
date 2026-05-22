/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
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
#  include <BRepAdaptor_Surface.hxx>
#  include <BRep_Builder.hxx>
#  include <BRepCheck_Analyzer.hxx>
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
#  include <GeomAbs_SurfaceType.hxx>
#  include <Geom_Circle.hxx>
#  include <Geom_Curve.hxx>
#  include <Geom_Line.hxx>
#  include <Geom_Surface.hxx>
#  include <IMeshTools_Parameters.hxx>
#  include <Poly_PolygonOnTriangulation.hxx>
#  include <Poly_Triangle.hxx>
#  include <Poly_Triangulation.hxx>
#  include <ShapeFix_Shape.hxx>
#  include <Standard_Failure.hxx>
#  include <NCollection_List.hxx>
#  include <TopExp.hxx>
#  include <TopExp_Explorer.hxx>
#  include <TopAbs_Orientation.hxx>
#  include <TopLoc_Location.hxx>
#  include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#  include <TopTools_ListOfShape.hxx>
#  include <TopoDS.hxx>
#  include <TopoDS_Compound.hxx>
#  include <TopoDS_Edge.hxx>
#  include <TopoDS_Face.hxx>
#  include <TopoDS_Shape.hxx>
#  include <TopoDS_Solid.hxx>
#  include <TopoDS_Vertex.hxx>
#  include <gp_Ax2.hxx>
#  include <gp_Dir.hxx>
#  include <gp_GTrsf.hxx>
#  include <gp_Pnt.hxx>
#  include <gp_Pnt2d.hxx>
#  include <gp_Trsf.hxx>
#  include <gp_Vec.hxx>
#endif

namespace blender {

struct NurbBodyBevelDebugDragState {
  uint64_t tick_id = 0;
  uint64_t active_mask = 0;
  float radius = 0.0f;
  int edge_index = -1;
  int domain = 0;
  double tick_time = 0.0;
};

static NurbBodyBevelDebugDragState g_nurb_body_bevel_debug_drag;

bool BKE_nurb_body_debug_bevel_enabled()
{
  static const bool enabled = std::getenv("NURB_BODY_DEBUG_BEVEL") != nullptr;
  return enabled;
}

void BKE_nurb_body_debug_bevel_set_drag_tick(const uint64_t tick_id,
                                             const float radius,
                                             const int edge_index,
                                             const int domain,
                                             const uint64_t active_mask)
{
  if (!BKE_nurb_body_debug_bevel_enabled()) {
    return;
  }

  g_nurb_body_bevel_debug_drag.tick_id = tick_id;
  g_nurb_body_bevel_debug_drag.active_mask = active_mask;
  g_nurb_body_bevel_debug_drag.radius = radius;
  g_nurb_body_bevel_debug_drag.edge_index = edge_index;
  g_nurb_body_bevel_debug_drag.domain = domain;
  g_nurb_body_bevel_debug_drag.tick_time = BLI_time_now_seconds();
  std::fprintf(stderr,
               "\n----START---- NURB_BODY_BEVEL_TICK tick=%llu domain=%d edge=%d "
               "active_mask=%llu radius=%.8f time=%.6f\n",
               static_cast<unsigned long long>(tick_id),
               domain,
               edge_index,
               static_cast<unsigned long long>(active_mask),
               double(radius),
               g_nurb_body_bevel_debug_drag.tick_time);
  std::fprintf(stderr,
               "NURB_BODY_BEVEL_KERNEL tick=%llu stage=set_drag_tick domain=%d edge=%d "
               "active_mask=%llu radius=%.8f time=%.6f\n",
               static_cast<unsigned long long>(tick_id),
               domain,
               edge_index,
               static_cast<unsigned long long>(active_mask),
               double(radius),
               g_nurb_body_bevel_debug_drag.tick_time);
  std::fflush(stderr);
}

void BKE_nurb_body_debug_bevel_end_drag_tick(const char *reason)
{
  if (!BKE_nurb_body_debug_bevel_enabled() || g_nurb_body_bevel_debug_drag.tick_id == 0) {
    return;
  }
  const double now = BLI_time_now_seconds();
  std::fprintf(stderr,
               "----END---- NURB_BODY_BEVEL_TICK tick=%llu domain=%d edge=%d "
               "active_mask=%llu radius=%.8f reason=%s total_ms=%.3f time=%.6f\n\n",
               static_cast<unsigned long long>(g_nurb_body_bevel_debug_drag.tick_id),
               g_nurb_body_bevel_debug_drag.domain,
               g_nurb_body_bevel_debug_drag.edge_index,
               static_cast<unsigned long long>(g_nurb_body_bevel_debug_drag.active_mask),
               double(g_nurb_body_bevel_debug_drag.radius),
               reason != nullptr ? reason : "unknown",
               (now - g_nurb_body_bevel_debug_drag.tick_time) * 1000.0,
               now);
  std::fflush(stderr);
  g_nurb_body_bevel_debug_drag = {};
}

static bool nurb_body_debug_bevel_active()
{
  return BKE_nurb_body_debug_bevel_enabled() && g_nurb_body_bevel_debug_drag.tick_id != 0;
}

static bool nurb_body_modal_bevel_preview_active(const NurbBody &body)
{
  return (body.flag & NURB_BODY_FAST_BEVEL_PREVIEW) != 0 || nurb_body_debug_bevel_active();
}

static double nurb_body_debug_now()
{
  return BLI_time_now_seconds();
}

static void nurb_body_debug_bevel_log(const char *stage, const char *format, ...)
{
  if (!nurb_body_debug_bevel_active()) {
    return;
  }

  std::fprintf(stderr,
               "NURB_BODY_BEVEL_KERNEL tick=%llu domain=%d edge=%d radius=%.8f stage=%s ",
               static_cast<unsigned long long>(g_nurb_body_bevel_debug_drag.tick_id),
               g_nurb_body_bevel_debug_drag.domain,
               g_nurb_body_bevel_debug_drag.edge_index,
               double(g_nurb_body_bevel_debug_drag.radius),
               stage);
  va_list args;
  va_start(args, format);
  std::vfprintf(stderr, format, args);
  va_end(args);
  std::fprintf(stderr, "\n");
  std::fflush(stderr);
}

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

static uint64_t edge_mask_with_positive_radii(const float *edge_radii, const uint64_t edge_mask)
{
  if (edge_radii == nullptr) {
    return 0;
  }
  uint64_t positive_edges = 0;
  for (int i = 0; i < 64; i++) {
    const uint64_t mask = nurb_body_edge_mask_for_index(i);
    if ((edge_mask & mask) != 0 && edge_radii[i] > 0.0f) {
      positive_edges |= mask;
    }
  }
  return positive_edges;
}

static bool edge_radii_contains_positive(const float *edge_radii, const uint64_t edge_mask)
{
  return edge_mask_with_positive_radii(edge_radii, edge_mask) != 0;
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

static void nurb_body_sanitize_edge_blend_state(const int bevel_type,
                                                float &bevel_radius,
                                                int &bevel_edge,
                                                uint64_t &bevel_edges,
                                                uint64_t &chamfer_edges,
                                                float bevel_radii[64],
                                                int bevel_order[64],
                                                int &bevel_order_next)
{
  if (bevel_edge < -1) {
    bevel_edge = -1;
  }

  if (bevel_edges == 0 && bevel_edge >= 0) {
    const uint64_t edge_mask = nurb_body_edge_mask_for_index(bevel_edge);
    if (bevel_radius > 0.0f || edge_radii_contains_positive(bevel_radii, edge_mask)) {
      bevel_edges = edge_mask;
    }
  }

  if (bevel_edges != 0 && bevel_radius <= 0.0f) {
    bevel_edges = edge_mask_with_positive_radii(bevel_radii, bevel_edges);
  }

  if (bevel_edges == 0) {
    bevel_edge = -1;
    chamfer_edges = 0;
    nurb_body_normalize_edge_bevel_order(bevel_edges, bevel_order, bevel_order_next);
    return;
  }

  if (bevel_type == NURB_BODY_BEVEL_CHAMFER && chamfer_edges == 0) {
    chamfer_edges = bevel_edges;
  }
  nurb_body_materialize_edge_bevel_radii(bevel_edges, bevel_radius, bevel_radii);
  chamfer_edges &= bevel_edges;
  nurb_body_normalize_edge_bevel_order(bevel_edges, bevel_order, bevel_order_next);
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

#ifdef WITH_OPENCASCADE
static void nurb_body_global_caches_remove_body(const NurbBody *body);
#endif

static void nurb_body_free_data(ID *id)
{
  NurbBody *body = reinterpret_cast<NurbBody *>(id);
#ifdef WITH_OPENCASCADE
  nurb_body_global_caches_remove_body(body);
#endif
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
  if (!ELEM(body->bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER)) {
    body->bevel_type = NURB_BODY_BEVEL_FILLET;
  }
  if (!ELEM(body->surface_bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER)) {
    body->surface_bevel_type = NURB_BODY_BEVEL_FILLET;
  }
  nurb_body_sanitize_edge_blend_state(body->bevel_type,
                                      body->bevel_radius,
                                      body->bevel_edge,
                                      body->bevel_edges,
                                      body->chamfer_edges,
                                      body->bevel_radii,
                                      body->bevel_order,
                                      body->bevel_order_next);
  nurb_body_sanitize_edge_blend_state(body->surface_bevel_type,
                                      body->surface_bevel_radius,
                                      body->surface_bevel_edge,
                                      body->surface_bevel_edges,
                                      body->surface_chamfer_edges,
                                      body->surface_bevel_radii,
                                      body->surface_bevel_order,
                                      body->surface_bevel_order_next);
  for (int i = 0; i < 64; i++) {
    if (((body->surface_bevel_edges | body->surface_selected_edges) &
         nurb_body_edge_mask_for_index(i)) == 0)
    {
      body->surface_edge_keys[i] = 0;
    }
  }
  if (!ELEM(body->select_mode,
            NURB_BODY_SELECT_MODE_EDGE,
            NURB_BODY_SELECT_MODE_FACE,
            NURB_BODY_SELECT_MODE_OBJECT))
  {
    body->select_mode = NURB_BODY_SELECT_MODE_EDGE;
  }
  if ((body->flag & NURB_BODY_TRIANGULATE_MESH) != 0) {
    body->tessellation_topology = NURB_BODY_TESSELLATION_TRIS;
    body->flag &= ~NURB_BODY_TRIANGULATE_MESH;
  }
  if (!ELEM(body->tessellation_topology,
            NURB_BODY_TESSELLATION_TRIS,
            NURB_BODY_TESSELLATION_QUADS,
            NURB_BODY_TESSELLATION_NGONS))
  {
    body->tessellation_topology = NURB_BODY_TESSELLATION_NGONS;
  }
  body->tessellation_deflection = std::max(body->tessellation_deflection, 0.0001f);
  body->tessellation_angle = std::clamp(body->tessellation_angle, 0.01f, 3.14159f);
  body->tessellation_face_deflection =
      std::max(body->tessellation_face_deflection > 0.0f ?
                   body->tessellation_face_deflection :
                   body->tessellation_deflection,
               0.0001f);
  body->tessellation_face_angle = std::clamp(body->tessellation_face_angle > 0.0f ?
                                                body->tessellation_face_angle :
                                                body->tessellation_angle,
                                            0.01f,
                                            3.14159f);
  body->tessellation_density = std::clamp(body->tessellation_density, 0.0f, 1.0f);
  body->tessellation_min_width = std::max(body->tessellation_min_width, 0.0f);
  body->tessellation_plane_angle = std::clamp(body->tessellation_plane_angle,
                                             0.01f,
                                             3.14159f);
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
    if (!ELEM(op->bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER)) {
      op->bevel_type = NURB_BODY_BEVEL_FILLET;
    }
    nurb_body_sanitize_edge_blend_state(op->bevel_type,
                                        op->bevel_radius,
                                        op->bevel_edge,
                                        op->bevel_edges,
                                        op->chamfer_edges,
                                        op->bevel_radii,
                                        op->bevel_order,
                                        op->bevel_order_next);

    if (op->operand_selected_edge < -1) {
      op->operand_selected_edge = -1;
    }
    if (op->operand_selected_edges == 0 && op->operand_selected_edge >= 0) {
      op->operand_selected_edges = nurb_body_edge_mask_for_index(op->operand_selected_edge);
    }
    if (op->operand_surface_selected_edges == 0 && op->operand_surface_selected_edge >= 0) {
      op->operand_surface_selected_edges = nurb_body_edge_mask_for_index(
          op->operand_surface_selected_edge);
    }
    if (!ELEM(op->operand_bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER)) {
      op->operand_bevel_type = NURB_BODY_BEVEL_FILLET;
    }
    if (!ELEM(op->operand_surface_bevel_type, NURB_BODY_BEVEL_FILLET, NURB_BODY_BEVEL_CHAMFER))
    {
      op->operand_surface_bevel_type = NURB_BODY_BEVEL_FILLET;
    }
    nurb_body_sanitize_edge_blend_state(op->operand_bevel_type,
                                        op->operand_bevel_radius,
                                        op->operand_bevel_edge,
                                        op->operand_bevel_edges,
                                        op->operand_chamfer_edges,
                                        op->operand_bevel_radii,
                                        op->operand_bevel_order,
                                        op->operand_bevel_order_next);
    nurb_body_sanitize_edge_blend_state(op->operand_surface_bevel_type,
                                        op->operand_surface_bevel_radius,
                                        op->operand_surface_bevel_edge,
                                        op->operand_surface_bevel_edges,
                                        op->operand_surface_chamfer_edges,
                                        op->operand_surface_bevel_radii,
                                        op->operand_surface_bevel_order,
                                        op->operand_surface_bevel_order_next);
    for (int i = 0; i < 64; i++) {
      if (((op->operand_surface_bevel_edges | op->operand_surface_selected_edges) &
           nurb_body_edge_mask_for_index(i)) == 0)
      {
        op->operand_surface_edge_keys[i] = 0;
      }
    }
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

struct NurbBodyShapeBoundaryStats {
  int edges = 0;
  int manifold_edges = 0;
  int seam_edges = 0;
  int open_edges = 0;
  int nonmanifold_edges = 0;
};

static int shape_unique_face_count_for_edge(const TopTools_ListOfShape &faces,
                                            const TopoDS_Edge &edge,
                                            bool &r_closed_seam)
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

  r_closed_seam = unique_faces.size() == 1 && BRep_Tool::IsClosed(edge, unique_faces.first());
  return unique_faces.size();
}

static NurbBodyShapeBoundaryStats shape_boundary_stats(const TopoDS_Shape &shape)
{
  NurbBodyShapeBoundaryStats stats;
  if (shape.IsNull()) {
    return stats;
  }

  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  stats.edges = edge_faces.Extent();
  for (int i = 1; i <= edge_faces.Extent(); i++) {
    const TopoDS_Edge edge = TopoDS::Edge(edge_faces.FindKey(i));
    if (edge.IsNull() || BRep_Tool::Degenerated(edge)) {
      continue;
    }

    bool closed_seam = false;
    const int face_count = shape_unique_face_count_for_edge(edge_faces.FindFromIndex(i),
                                                            edge,
                                                            closed_seam);
    if (face_count == 2) {
      stats.manifold_edges++;
    }
    else if (face_count == 1 && closed_seam) {
      stats.seam_edges++;
    }
    else if (face_count < 2) {
      stats.open_edges++;
    }
    else {
      stats.nonmanifold_edges++;
    }
  }
  return stats;
}

static bool shape_has_closed_surface_boundary(const TopoDS_Shape &shape)
{
  const NurbBodyShapeBoundaryStats stats = shape_boundary_stats(shape);
  return stats.edges > 0 && stats.open_edges == 0 && stats.nonmanifold_edges == 0 &&
         (stats.manifold_edges > 0 || stats.seam_edges > 0);
}

static int shape_solid_count(const TopoDS_Shape &shape)
{
  int solids = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
    solids++;
  }
  return solids;
}

static bool shape_passes_fast_integrity_check(const TopoDS_Shape &shape,
                                              const bool geometry_controls)
{
  if (shape.IsNull()) {
    return false;
  }

  int face_count = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    face_count++;
  }
  if (face_count == 0) {
    return false;
  }

  int edge_count = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_EDGE); explorer.More(); explorer.Next()) {
    edge_count++;
  }
  if (edge_count == 0) {
    return false;
  }
  if (!shape_has_closed_surface_boundary(shape)) {
    return false;
  }

  try {
    BRepCheck_Analyzer analyzer(shape, geometry_controls, false, false);
    return analyzer.IsValid();
  }
  catch (Standard_Failure const &) {
    return false;
  }
}

static bool shape_passes_fast_blend_integrity_check(const TopoDS_Shape &shape)
{
  return shape_passes_fast_integrity_check(shape, true);
}

static bool shape_passes_boolean_result_check(const TopoDS_Shape &shape)
{
  if (shape_solid_count(shape) == 0) {
    return false;
  }
  return shape_passes_fast_integrity_check(shape, true);
}

static void shape_triangulation_stats(const TopoDS_Shape &shape,
                                      int &r_nodes,
                                      int &r_triangles,
                                      int &r_faces)
{
  r_nodes = 0;
  r_triangles = 0;
  r_faces = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    r_faces++;
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
    if (!triangulation.IsNull()) {
      r_nodes += triangulation->NbNodes();
      r_triangles += triangulation->NbTriangles();
    }
  }
}

static void triangulate_shape_for_preview(const TopoDS_Shape &shape,
                                          const NurbBody &body,
                                          const bool force_rebuild = true)
{
  constexpr int stale_triangle_budget = 50000;
  const bool debug_mesh = nurb_body_debug_bevel_active();
  int existing_nodes = 0;
  int existing_triangles = 0;
  int existing_faces = 0;
  if (debug_mesh && !force_rebuild) {
    shape_triangulation_stats(shape, existing_nodes, existing_triangles, existing_faces);
  }

  constexpr double min_linear_deflection = 1.0e-5;
  const double density = std::clamp(double(body.tessellation_density), 0.0, 1.0);
  const double density_scale = std::max(0.35, 1.0 - density * 0.65);
  const double linear_deflection = std::max(double(body.tessellation_deflection) * density_scale,
                                            min_linear_deflection);
  const double face_deflection = std::max(double(body.tessellation_face_deflection) *
                                              density_scale,
                                          min_linear_deflection);
  const double angular_deflection = std::clamp(double(body.tessellation_angle) * density_scale,
                                               0.01,
                                               3.14159265358979323846);
  const double face_angle = std::clamp(double(body.tessellation_face_angle) * density_scale,
                                       0.01,
                                       3.14159265358979323846);

  IMeshTools_Parameters parameters;
  parameters.MeshAlgo = IMeshTools_MeshAlgoType_Delabella;
  parameters.Deflection = linear_deflection;
  parameters.DeflectionInterior = face_deflection;
  parameters.Angle = angular_deflection;
  parameters.AngleInterior = face_angle;
  parameters.MinSize = body.tessellation_min_width > 0.0f ?
                           double(body.tessellation_min_width) :
                           std::max(std::min(linear_deflection, face_deflection) *
                                        IMeshTools_Parameters::RelMinSize(),
                                    1.0e-7);
  parameters.InParallel = true;
  parameters.Relative = false;
  parameters.InternalVerticesMode = true;
  parameters.ControlSurfaceDeflection = true;
  parameters.EnableControlSurfaceDeflectionAllSurfaces = false;
  parameters.CleanModel = true;
  parameters.AdjustMinSize = true;
  parameters.ForceFaceDeflection = false;
  parameters.AllowQualityDecrease = false;

  BRepTools::Clean(shape);
  BRepMesh_IncrementalMesh(shape, parameters);
  if (debug_mesh && shape_has_unmeshed_faces(shape)) {
    nurb_body_debug_bevel_log("triangulate_unmeshed_with_user_settings",
                              "edge_deflection=%.6f face_deflection=%.6f edge_angle=%.6f "
                              "face_angle=%.6f",
                              linear_deflection,
                              face_deflection,
                              angular_deflection,
                              face_angle);
  }
  if (debug_mesh && !force_rebuild && existing_triangles > stale_triangle_budget) {
    int nodes = 0;
    int triangles = 0;
    int faces = 0;
    shape_triangulation_stats(shape, nodes, triangles, faces);
    nurb_body_debug_bevel_log("triangulate_cleaned_pathology",
                              "old_nodes=%d old_tris=%d old_faces=%d nodes=%d tris=%d "
                              "faces=%d edge_deflection=%.6f face_deflection=%.6f "
                              "edge_angle=%.6f face_angle=%.6f",
                              existing_nodes,
                              existing_triangles,
                              existing_faces,
                              nodes,
                              triangles,
                              faces,
                              linear_deflection,
                              face_deflection,
                              angular_deflection,
                              face_angle);
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

static TopoDS_Shape heal_boolean_result_shape(const TopoDS_Shape &shape)
{
  if (shape.IsNull()) {
    return shape;
  }

  TopoDS_Shape result = shape;
  try {
    ShapeFix_Shape fixer(result);
    fixer.SetPrecision(1.0e-6);
    fixer.SetMinTolerance(1.0e-7);
    fixer.SetMaxTolerance(1.0e-4);
    fixer.Perform();
    const TopoDS_Shape fixed_shape = fixer.Shape();
    if (!fixed_shape.IsNull()) {
      result = fixed_shape;
    }
  }
  catch (Standard_Failure const &) {
    /* Keep the original boolean result and let validation decide. */
  }

  try {
    BRepLib::SameParameter(result, 1.0e-5, true);
    BRepLib::EncodeRegularity(result);
    BRepTools::Clean(result);
  }
  catch (Standard_Failure const &) {
    /* Keep the best available shape and let validation decide. */
  }

  Vector<TopoDS_Solid> solids;
  for (TopExp_Explorer explorer(result, TopAbs_SOLID); explorer.More(); explorer.Next()) {
    TopoDS_Solid solid = TopoDS::Solid(explorer.Current());
    if (solid.IsNull()) {
      continue;
    }
    try {
      BRepLib::OrientClosedSolid(solid);
    }
    catch (Standard_Failure const &) {
      /* Keep the solid orientation returned by OCCT if orientation repair fails. */
    }
    solids.append(solid);
  }
  if (solids.size() == 1) {
    result = solids.first();
  }
  else if (solids.size() > 1) {
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (const TopoDS_Solid &solid : solids) {
      builder.Add(compound, solid);
    }
    result = compound;
  }
  return result;
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
  builder.SetNonDestructive(true);
  builder.SetCheckInverted(true);
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
  try {
    builder.SimplifyResult(true, true);
  }
  catch (Standard_Failure const &) {
    /* Keep the raw boolean result if same-domain simplification fails. */
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
  result = heal_boolean_result_shape(result);
  if (result.IsNull()) {
    return false;
  }
  if (!shape_passes_boolean_result_check(result)) {
    const NurbBodyShapeBoundaryStats stats = shape_boundary_stats(result);
    nurb_body_debug_bevel_log("boolean_result_invalid",
                              "operation=%d parallel=%d obb=%d fuzzy=%.8g solids=%d "
                              "edges=%d manifold=%d seam=%d open=%d nonmanifold=%d",
                              int(operation),
                              int(run_parallel),
                              int(use_obb),
                              fuzzy_value,
                              shape_solid_count(result),
                              stats.edges,
                              stats.manifold_edges,
                              stats.seam_edges,
                              stats.open_edges,
                              stats.nonmanifold_edges);
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
  nurb_body_debug_bevel_log("boolean_all_attempts_failed",
                            "operation=%d base_null=%d tool_null=%d",
                            int(operation),
                            int(base.IsNull()),
                            int(tool.IsNull()));
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
  UNUSED_VARS(bevel_radius, bevel_radii, bevel_edges, selected_edges, bevel_edge, selected_edge);
  return false;
}

static float uniform_selected_blend_radius(const float bevel_radius,
                                           const uint64_t bevel_edges,
                                           const float *bevel_radii,
                                           const uint64_t selected_edges,
                                           const int bevel_edge,
                                           const int selected_edge)
{
  if (selected_edges == 0) {
    return 0.0f;
  }

  const int preferred_edge = bevel_edge >= 0 ? bevel_edge : selected_edge;
  const uint64_t preferred_mask = nurb_body_edge_mask_for_index(preferred_edge);
  if ((selected_edges & preferred_mask) != 0) {
    const float preferred_radius = bevel_radius_for_edge(
        bevel_radius, bevel_edges, bevel_radii, preferred_edge);
    if (preferred_radius > 0.0f) {
      return preferred_radius;
    }
  }

  for (int i = 0; i < 64; i++) {
    if ((selected_edges & nurb_body_edge_mask_for_index(i)) == 0) {
      continue;
    }
    const float radius = bevel_radius_for_edge(bevel_radius, bevel_edges, bevel_radii, i);
    if (radius > 0.0f) {
      return radius;
    }
  }

  return bevel_radius > 0.0f ? bevel_radius : 0.0f;
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
  const float selected_uniform_radius = uniform_selected_blend_radius(bevel_radius,
                                                                     bevel_edges,
                                                                     bevel_radii,
                                                                     selected_edges,
                                                                     bevel_edge,
                                                                     selected_edge);
  for (int i = 0; i < edges_num && i < 64; i++) {
    if (!selected_edge_for_blend(
            bevel_edges, selected_edges, bevel_edge, selected_edge, i, edges_num))
    {
      continue;
    }
    const uint64_t edge_mask = nurb_body_edge_mask_for_index(i);
    const bool selected_step = (selected_edges & edge_mask) != 0;
    const float radius = (selected_step && selected_uniform_radius > 0.0f) ?
                             selected_uniform_radius :
                             bevel_radius_for_edge(bevel_radius, bevel_edges, bevel_radii, i);
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

static uint64_t active_blend_edge_mask(const uint64_t selected_edges,
                                       const uint64_t bevel_edges,
                                       const int bevel_edge,
                                       const int selected_edge)
{
  if (selected_edges != 0) {
    return selected_edges;
  }
  const int active_edge = active_blend_edge_index(bevel_edge, selected_edge);
  const uint64_t active_mask = nurb_body_edge_mask_for_index(active_edge);
  if (active_mask != 0 && (bevel_edges & active_mask) != 0) {
    return active_mask;
  }
  return 0;
}

static int active_blend_step_position(const Span<NurbBodyBlendStep> steps,
                                      const uint64_t active_mask)
{
  if (active_mask == 0) {
    return -1;
  }
  for (const int i : steps.index_range()) {
    if ((active_mask & nurb_body_edge_mask_for_index(steps[i].edge_index)) != 0) {
      return i;
    }
  }
  return -1;
}

static bool active_blend_steps_are_last(const Span<NurbBodyBlendStep> steps,
                                        const uint64_t active_mask)
{
  bool found_active = false;
  bool found_stable_after_active = false;
  for (const NurbBodyBlendStep &step : steps) {
    const bool active = (active_mask & nurb_body_edge_mask_for_index(step.edge_index)) != 0;
    found_active |= active;
    if (found_active && !active) {
      found_stable_after_active = true;
    }
  }
  return found_active && !found_stable_after_active;
}

static Vector<NurbBodyBlendStep> blend_steps_for_active_mask(
    const Span<NurbBodyBlendStep> steps, const uint64_t active_mask)
{
  Vector<NurbBodyBlendStep> active_steps;
  for (const NurbBodyBlendStep &step : steps) {
    if ((active_mask & nurb_body_edge_mask_for_index(step.edge_index)) != 0) {
      active_steps.append(step);
    }
  }
  return active_steps;
}

static Vector<NurbBodyBlendStep> stable_blend_steps_without_active(
    const Span<NurbBodyBlendStep> steps, const uint64_t active_mask)
{
  Vector<NurbBodyBlendStep> stable_steps;
  stable_steps.reserve(steps.size());
  for (const NurbBodyBlendStep &step : steps) {
    if ((active_mask & nurb_body_edge_mask_for_index(step.edge_index)) == 0) {
      stable_steps.append(step);
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

static int find_current_edge_by_reference_threshold(
    const Span<NurbBodyEdgeReference> edge_references,
    const int fallback_index,
    const NurbBodyEdgeReference &reference,
    const float relative_threshold,
    const float absolute_threshold)
{
  if (reference.points.size() < 2) {
    return fallback_index >= 0 && fallback_index < edge_references.size() ? fallback_index : -1;
  }

  const float threshold = std::max(reference.length * relative_threshold, absolute_threshold);

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
  return fallback_index >= 0 && fallback_index < edge_references.size() ? fallback_index : -1;
}

static int find_current_edge_by_reference(const Span<NurbBodyEdgeReference> edge_references,
                                          const int fallback_index,
                                          const NurbBodyEdgeReference &reference)
{
  return find_current_edge_by_reference_threshold(edge_references, fallback_index, reference,
                                                  0.02f, 0.001f);
}

static NurbBodyEdgeReference surface_edge_entry_reference_for_blend(
    const NurbBodySurfaceEdgeEntry &entry)
{
  NurbBodyEdgeReference reference;
  reference.points = entry.points;
  reference.length = entry.length > 0.0f ? entry.length : polyline_length(entry.points.as_span());
  return reference;
}

static Vector<NurbBodyEdgeReference> surface_catalog_references_for_blend(
    const Span<NurbBodySurfaceEdgeEntry> catalog)
{
  Vector<NurbBodyEdgeReference> references;
  references.reserve(catalog.size());
  for (const NurbBodySurfaceEdgeEntry &entry : catalog) {
    references.append(surface_edge_entry_reference_for_blend(entry));
  }
  return references;
}

static int find_catalog_edge_for_key_or_reference(
    const Span<NurbBodySurfaceEdgeEntry> catalog,
    const uint64_t edge_key,
    const NurbBodyEdgeReference &reference)
{
  const int key_match = find_catalog_edge_for_key(catalog, edge_key);
  if (key_match != -1) {
    return key_match;
  }
  const Vector<NurbBodyEdgeReference> catalog_references =
      surface_catalog_references_for_blend(catalog);
  return find_current_edge_by_reference_threshold(
      catalog_references.as_span(), -1, reference, 0.12f, 0.005f);
}

static double safe_blend_radius_limit(const float radius_limit)
{
  return std::max(double(radius_limit), 1.0e-6);
}

static bool shape_can_be_previewed_in_place(TopoDS_Shape &shape)
{
  if (shape.IsNull()) {
    return false;
  }

  TopExp_Explorer face_explorer(shape, TopAbs_FACE);
  if (!face_explorer.More()) {
    return false;
  }

  try {
    return shape_passes_fast_blend_integrity_check(shape);
  }
  catch (Standard_Failure const &) {
    return false;
  }
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

struct NurbBodySingleFilletBuilderCache {
  TopoDS_Shape shape;
  TopoDS_Edge edge;
  ChFi3d_FilletShape fillet_shape = ChFi3d_QuasiAngular;
  std::unique_ptr<BRepFilletAPI_MakeFillet> builder;
};

struct NurbBodyLastGoodSingleBlend {
  TopoDS_Shape base_shape;
  TopoDS_Edge edge;
  TopoDS_Shape blended_shape;
  int bevel_type = NURB_BODY_BEVEL_FILLET;
  int source_index = -1;
  double radius = 0.0;
  double failed_radius = 0.0;
};

struct NurbBodyLastGoodGroupBlend {
  TopoDS_Shape base_shape;
  TopoDS_Shape blended_shape;
  uint64_t source_mask = 0;
  int bevel_type = NURB_BODY_BEVEL_FILLET;
  double radius = 0.0;
};

static void single_fillet_builder_cache_clear(NurbBodySingleFilletBuilderCache &cache)
{
  cache.builder.reset();
  cache.shape = TopoDS_Shape();
  cache.edge = TopoDS_Edge();
  cache.fillet_shape = ChFi3d_QuasiAngular;
}

static void last_good_single_blend_store(NurbBodyLastGoodSingleBlend &cache,
                                         const TopoDS_Shape &base_shape,
                                         const TopoDS_Edge &edge,
                                         const TopoDS_Shape &blended_shape,
                                         const int bevel_type,
                                         const int source_index,
                                         const double radius,
                                         const double failed_radius = 0.0)
{
  if (base_shape.IsNull() || edge.IsNull() || blended_shape.IsNull() || radius <= 0.0) {
    return;
  }
  cache.base_shape = base_shape;
  cache.edge = edge;
  cache.blended_shape = blended_shape;
  cache.bevel_type = bevel_type;
  cache.source_index = source_index;
  cache.radius = radius;
  cache.failed_radius = failed_radius > radius ? failed_radius : 0.0;
}

static bool last_good_single_blend_matches(const NurbBodyLastGoodSingleBlend &cache,
                                           const TopoDS_Shape &base_shape,
                                           const TopoDS_Edge &edge,
                                           const int bevel_type,
                                           const int source_index)
{
  if (cache.base_shape.IsNull() || cache.edge.IsNull() || cache.blended_shape.IsNull() ||
      cache.radius <= 0.0)
  {
    return false;
  }
  if (cache.bevel_type != bevel_type || cache.source_index != source_index ||
      !cache.base_shape.IsSame(base_shape) || !cache.edge.IsSame(edge))
  {
    return false;
  }
  return true;
}

static void last_good_single_blend_note_failure(NurbBodyLastGoodSingleBlend &cache,
                                                const TopoDS_Shape &base_shape,
                                                const TopoDS_Edge &edge,
                                                const int bevel_type,
                                                const int source_index,
                                                const double failed_radius)
{
  if (failed_radius <= 0.0 ||
      !last_good_single_blend_matches(cache, base_shape, edge, bevel_type, source_index))
  {
    return;
  }
  if (failed_radius <= cache.radius) {
    return;
  }
  if (cache.failed_radius <= 0.0 || failed_radius < cache.failed_radius) {
    cache.failed_radius = failed_radius;
  }
}

static bool last_good_single_blend_find(const NurbBodyLastGoodSingleBlend &cache,
                                        const TopoDS_Shape &base_shape,
                                        const TopoDS_Edge &edge,
                                        const int bevel_type,
                                        const int source_index,
                                        const double requested_radius,
                                        TopoDS_Shape &r_shape)
{
  if (requested_radius <= 0.0 ||
      !last_good_single_blend_matches(cache, base_shape, edge, bevel_type, source_index))
  {
    return false;
  }
  if (cache.radius > requested_radius) {
    return false;
  }
  r_shape = cache.blended_shape;
  return true;
}

static uint64_t blend_step_source_mask(const Span<NurbBodyBlendStep> steps)
{
  uint64_t source_mask = 0;
  for (const NurbBodyBlendStep &step : steps) {
    source_mask |= nurb_body_edge_mask_for_index(step.edge_index);
  }
  return source_mask;
}

static void last_good_group_blend_store(NurbBodyLastGoodGroupBlend &cache,
                                        const TopoDS_Shape &base_shape,
                                        const TopoDS_Shape &blended_shape,
                                        const uint64_t source_mask,
                                        const int bevel_type,
                                        const double radius)
{
  if (base_shape.IsNull() || blended_shape.IsNull() || source_mask == 0 || radius <= 0.0) {
    return;
  }
  cache.base_shape = base_shape;
  cache.blended_shape = blended_shape;
  cache.source_mask = source_mask;
  cache.bevel_type = bevel_type;
  cache.radius = radius;
}

static bool last_good_group_blend_find(const NurbBodyLastGoodGroupBlend &cache,
                                       const TopoDS_Shape &base_shape,
                                       const uint64_t source_mask,
                                       const int bevel_type,
                                       const double requested_radius,
                                       TopoDS_Shape &r_shape)
{
  if (requested_radius <= 0.0 || source_mask == 0 || cache.base_shape.IsNull() ||
      cache.blended_shape.IsNull() || cache.radius <= 0.0)
  {
    return false;
  }
  if (cache.bevel_type != bevel_type || cache.source_mask != source_mask ||
      !cache.base_shape.IsSame(base_shape) || cache.radius > requested_radius)
  {
    return false;
  }
  r_shape = cache.blended_shape;
  return true;
}

[[maybe_unused]] static bool try_cached_single_fillet_blend(
    const TopoDS_Shape &shape,
    const NurbBodyBlendCandidate &candidate,
    const bool validate_shape,
    const ChFi3d_FilletShape fillet_shape,
    TopoDS_Shape &r_shape)
{
  if (shape.IsNull() || candidate.edge.IsNull() || candidate.radius <= 0.0) {
    return false;
  }

  thread_local NurbBodySingleFilletBuilderCache cache;
  try {
    const double total_start = nurb_body_debug_now();
    const bool can_reuse = cache.builder != nullptr && cache.shape.IsSame(shape) &&
                           cache.edge.IsSame(candidate.edge) &&
                           cache.fillet_shape == fillet_shape;
    nurb_body_debug_bevel_log("single_fillet_cache_begin",
                              "cache_hit=%d radius=%.8f validate=%d shape_null=%d edge_null=%d",
                              int(can_reuse),
                              candidate.radius,
                              int(validate_shape),
                              int(shape.IsNull()),
                              int(candidate.edge.IsNull()));
    if (!can_reuse) {
      single_fillet_builder_cache_clear(cache);
      cache.shape = shape;
      cache.edge = candidate.edge;
      cache.fillet_shape = fillet_shape;
      cache.builder = std::make_unique<BRepFilletAPI_MakeFillet>(shape, fillet_shape);
      cache.builder->Add(candidate.radius, candidate.edge);
    }
    else {
      cache.builder->Reset();
      const int contour = cache.builder->Contour(candidate.edge);
      if (contour <= 0) {
        cache.builder->Add(candidate.radius, candidate.edge);
      }
      else {
        cache.builder->SetRadius(candidate.radius, contour, candidate.edge);
      }
    }

    const double build_start = nurb_body_profile_blend_enabled() ? BLI_time_now_seconds() : 0.0;
    const double debug_build_start = nurb_body_debug_now();
    cache.builder->Build();
    const double build_ms = (nurb_body_debug_now() - debug_build_start) * 1000.0;
    const bool done = cache.builder->IsDone();
    if (nurb_body_profile_blend_enabled()) {
      NurbBodyBlendCandidate profile_candidate = candidate;
      profile_blend_build(Span<NurbBodyBlendCandidate>(&profile_candidate, 1),
                          NURB_BODY_BEVEL_FILLET,
                          false,
                          done,
                          BLI_time_now_seconds() - build_start);
    }
    if (!done || cache.builder->NbFaultyContours() > 0 ||
        cache.builder->NbFaultyVertices() > 0)
    {
      nurb_body_debug_bevel_log("single_fillet_cache_build_failed",
                                "cache_hit=%d done=%d faulty_contours=%d faulty_vertices=%d "
                                "build_ms=%.3f total_ms=%.3f",
                                int(can_reuse),
                                int(done),
                                cache.builder->NbFaultyContours(),
                                cache.builder->NbFaultyVertices(),
                                build_ms,
                                (nurb_body_debug_now() - total_start) * 1000.0);
      single_fillet_builder_cache_clear(cache);
      return false;
    }

    TopoDS_Shape blended_shape = cache.builder->Shape();
    if (blended_shape.IsNull() || blended_shape.IsSame(shape)) {
      nurb_body_debug_bevel_log("single_fillet_cache_shape_rejected",
                                "cache_hit=%d null=%d same=%d build_ms=%.3f total_ms=%.3f",
                                int(can_reuse),
                                int(blended_shape.IsNull()),
                                int(blended_shape.IsSame(shape)),
                                build_ms,
                                (nurb_body_debug_now() - total_start) * 1000.0);
      single_fillet_builder_cache_clear(cache);
      return false;
    }
    double validate_ms = 0.0;
    if (validate_shape) {
      const double validate_start = nurb_body_debug_now();
      const bool previewable = shape_can_be_previewed_in_place(blended_shape);
      validate_ms = (nurb_body_debug_now() - validate_start) * 1000.0;
      if (!previewable) {
        nurb_body_debug_bevel_log("single_fillet_cache_validation_failed",
                                  "cache_hit=%d build_ms=%.3f validate_ms=%.3f total_ms=%.3f",
                                  int(can_reuse),
                                  build_ms,
                                  validate_ms,
                                  (nurb_body_debug_now() - total_start) * 1000.0);
        single_fillet_builder_cache_clear(cache);
        return false;
      }
    }
    else {
      const double validate_start = nurb_body_debug_now();
      const bool previewable = shape_passes_fast_blend_integrity_check(blended_shape);
      validate_ms = (nurb_body_debug_now() - validate_start) * 1000.0;
      if (!previewable) {
        nurb_body_debug_bevel_log("single_fillet_cache_integrity_failed",
                                  "cache_hit=%d build_ms=%.3f integrity_ms=%.3f total_ms=%.3f",
                                  int(can_reuse),
                                  build_ms,
                                  validate_ms,
                                  (nurb_body_debug_now() - total_start) * 1000.0);
        single_fillet_builder_cache_clear(cache);
        return false;
      }
    }

    r_shape = blended_shape;
    nurb_body_debug_bevel_log("single_fillet_cache_done",
                              "cache_hit=%d build_ms=%.3f validate_ms=%.3f total_ms=%.3f",
                              int(can_reuse),
                              build_ms,
                              validate_ms,
                              (nurb_body_debug_now() - total_start) * 1000.0);
    return true;
  }
  catch (Standard_Failure const &) {
    nurb_body_debug_bevel_log("single_fillet_cache_exception", "radius=%.8f", candidate.radius);
    single_fillet_builder_cache_clear(cache);
    return false;
  }
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
  double min_radius = 0.0;
  double max_radius = 0.0;
  blend_candidate_radius_range(edges, min_radius, max_radius);
  const double total_start = nurb_body_debug_now();
  nurb_body_debug_bevel_log("try_edge_set_begin",
                            "type=%s edges=%d prepare=%d validate=%d fillet_shape=%d "
                            "radius_min=%.8f radius_max=%.8f",
                            bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
                            int(edges.size()),
                            int(prepare_edges),
                            int(validate_shape),
                            int(fillet_shape),
                            min_radius,
                            max_radius);
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
      if (!validate_shape) {
        const double simulate_start = nurb_body_debug_now();
        for (const int contour : contours) {
          chamfer.Simulate(contour);
          if (chamfer.NbSurf(contour) <= 0) {
            nurb_body_debug_bevel_log("try_edge_set_simulation_failed",
                                      "type=chamfer edges=%d prepare=%d contour=%d "
                                      "simulate_ms=%.3f total_ms=%.3f",
                                      int(edges.size()),
                                      int(prepare_edges),
                                      contour,
                                      (nurb_body_debug_now() - simulate_start) * 1000.0,
                                      (nurb_body_debug_now() - total_start) * 1000.0);
            return false;
          }
        }
      }
      const double build_start = nurb_body_profile_blend_enabled() ? BLI_time_now_seconds() : 0.0;
      const double debug_build_start = nurb_body_debug_now();
      chamfer.Build();
      const double build_ms = (nurb_body_debug_now() - debug_build_start) * 1000.0;
      const bool done = chamfer.IsDone();
      if (nurb_body_profile_blend_enabled()) {
        profile_blend_build(edges,
                            bevel_type,
                            prepare_edges,
                            done,
                            BLI_time_now_seconds() - build_start);
      }
      if (!done) {
        nurb_body_debug_bevel_log("try_edge_set_build_failed",
                                  "type=chamfer edges=%d prepare=%d build_ms=%.3f total_ms=%.3f",
                                  int(edges.size()),
                                  int(prepare_edges),
                                  build_ms,
                                  (nurb_body_debug_now() - total_start) * 1000.0);
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
      if (!validate_shape) {
        const double simulate_start = nurb_body_debug_now();
        for (const int contour : contours) {
          fillet.Simulate(contour);
          if (fillet.NbSurf(contour) <= 0) {
            nurb_body_debug_bevel_log("try_edge_set_simulation_failed",
                                      "type=fillet edges=%d prepare=%d contour=%d "
                                      "simulate_ms=%.3f total_ms=%.3f",
                                      int(edges.size()),
                                      int(prepare_edges),
                                      contour,
                                      (nurb_body_debug_now() - simulate_start) * 1000.0,
                                      (nurb_body_debug_now() - total_start) * 1000.0);
            return false;
          }
        }
      }
      const double build_start = nurb_body_profile_blend_enabled() ? BLI_time_now_seconds() : 0.0;
      const double debug_build_start = nurb_body_debug_now();
      fillet.Build();
      const double build_ms = (nurb_body_debug_now() - debug_build_start) * 1000.0;
      const bool done = fillet.IsDone();
      if (nurb_body_profile_blend_enabled()) {
        profile_blend_build(edges,
                            bevel_type,
                            prepare_edges,
                            done,
                            BLI_time_now_seconds() - build_start);
      }
      if (!done) {
        nurb_body_debug_bevel_log("try_edge_set_build_failed",
                                  "type=fillet edges=%d prepare=%d build_ms=%.3f total_ms=%.3f",
                                  int(edges.size()),
                                  int(prepare_edges),
                                  build_ms,
                                  (nurb_body_debug_now() - total_start) * 1000.0);
        return false;
      }
      if (fillet.NbFaultyContours() > 0 || fillet.NbFaultyVertices() > 0) {
        nurb_body_debug_bevel_log("try_edge_set_faulty",
                                  "type=fillet edges=%d prepare=%d faulty_contours=%d "
                                  "faulty_vertices=%d build_ms=%.3f total_ms=%.3f",
                                  int(edges.size()),
                                  int(prepare_edges),
                                  fillet.NbFaultyContours(),
                                  fillet.NbFaultyVertices(),
                                  build_ms,
                                  (nurb_body_debug_now() - total_start) * 1000.0);
        return false;
      }
      if (blended_shape.IsNull()) {
        blended_shape = fillet.Shape();
      }
    }

    if (blended_shape.IsNull() || blended_shape.IsSame(shape)) {
      nurb_body_debug_bevel_log("try_edge_set_shape_rejected",
                                "type=%s edges=%d null=%d same=%d total_ms=%.3f",
                                bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
                                int(edges.size()),
                                int(blended_shape.IsNull()),
                                int(blended_shape.IsSame(shape)),
                                (nurb_body_debug_now() - total_start) * 1000.0);
      return false;
    }
    double validate_ms = 0.0;
    if (validate_shape) {
      const double validate_start = nurb_body_debug_now();
      const bool previewable = shape_can_be_previewed_in_place(blended_shape);
      validate_ms = (nurb_body_debug_now() - validate_start) * 1000.0;
      if (!previewable) {
        nurb_body_debug_bevel_log("try_edge_set_validation_failed",
                                  "type=%s edges=%d validate_ms=%.3f total_ms=%.3f",
                                  bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
                                  int(edges.size()),
                                  validate_ms,
                                  (nurb_body_debug_now() - total_start) * 1000.0);
        return false;
      }
    }
    else {
      const double validate_start = nurb_body_debug_now();
      const bool previewable = shape_passes_fast_blend_integrity_check(blended_shape);
      validate_ms = (nurb_body_debug_now() - validate_start) * 1000.0;
      if (!previewable) {
        nurb_body_debug_bevel_log("try_edge_set_integrity_failed",
                                  "type=%s edges=%d prepare=%d integrity_ms=%.3f total_ms=%.3f",
                                  bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
                                  int(edges.size()),
                                  int(prepare_edges),
                                  validate_ms,
                                  (nurb_body_debug_now() - total_start) * 1000.0);
        return false;
      }
    }

    r_shape = blended_shape;
    nurb_body_debug_bevel_log("try_edge_set_done",
                              "type=%s edges=%d prepare=%d validate_ms=%.3f total_ms=%.3f",
                              bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
                              int(edges.size()),
                              int(prepare_edges),
                              validate_ms,
                              (nurb_body_debug_now() - total_start) * 1000.0);
    return true;
  }
  catch (Standard_Failure const &) {
    nurb_body_debug_bevel_log("try_edge_set_exception",
                              "type=%s edges=%d prepare=%d total_ms=%.3f",
                              bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
                              int(edges.size()),
                              int(prepare_edges),
                              (nurb_body_debug_now() - total_start) * 1000.0);
    return false;
  }
}

static bool try_edge_set_blend_prepared(const TopoDS_Shape &shape,
                                        const int bevel_type,
                                        const Span<NurbBodyBlendCandidate> candidates,
                                        const bool validate_shape,
                                        const ChFi3d_FilletShape fillet_shape,
                                        TopoDS_Shape &r_shape)
{
  return try_edge_set_blend(
             shape, bevel_type, candidates, validate_shape, false, fillet_shape, r_shape) ||
         try_edge_set_blend(
             shape, bevel_type, candidates, validate_shape, true, fillet_shape, r_shape);
}

static Vector<NurbBodyBlendCandidate> uniform_blend_candidates(
    const Span<NurbBodyBlendCandidate> candidates)
{
  Vector<NurbBodyBlendCandidate> uniform_candidates;
  uniform_candidates.reserve(candidates.size());
  if (candidates.is_empty()) {
    return uniform_candidates;
  }

  const double uniform_radius = candidates[0].radius;
  for (const NurbBodyBlendCandidate &candidate : candidates) {
    NurbBodyBlendCandidate uniform_candidate = candidate;
    uniform_candidate.radius = uniform_radius;
    uniform_candidates.append(uniform_candidate);
  }
  return uniform_candidates;
}

static bool try_uniform_edge_set_blend_with_fallback(
    const TopoDS_Shape &shape,
    const int bevel_type,
    const Span<NurbBodyBlendCandidate> candidates,
    const bool validate_shape,
    const ChFi3d_FilletShape fillet_shape,
    TopoDS_Shape &r_shape)
{
  if (candidates.is_empty()) {
    return false;
  }
  if (bevel_type == NURB_BODY_BEVEL_FILLET && candidates.size() > 1) {
    return false;
  }

  Vector<NurbBodyBlendCandidate> uniform_candidates = uniform_blend_candidates(candidates);
  if (try_edge_set_blend_prepared(
          shape, bevel_type, uniform_candidates.as_span(), validate_shape, fillet_shape, r_shape))
  {
    return true;
  }

  const double requested_radius = uniform_candidates[0].radius;
  constexpr double fallback_scales[] = {0.98, 0.95, 0.90, 0.80, 0.65, 0.50, 0.35, 0.20};
  for (const double scale : fallback_scales) {
    const double fallback_radius = requested_radius * scale;
    if (fallback_radius <= 1.0e-6 || fallback_radius >= requested_radius) {
      continue;
    }
    for (NurbBodyBlendCandidate &candidate : uniform_candidates) {
      candidate.radius = fallback_radius;
    }
    if (try_edge_set_blend_prepared(
            shape, bevel_type, uniform_candidates.as_span(), validate_shape, fillet_shape, r_shape))
    {
      nurb_body_debug_bevel_log("try_uniform_edge_set_fallback_success",
                                "type=%s edges=%d requested=%.8f fallback=%.8f scale=%.3f",
                                bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
                                int(candidates.size()),
                                requested_radius,
                                fallback_radius,
                                scale);
      return true;
    }
  }
  return false;
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

static bool edge_already_listed(const Span<TopoDS_Edge> edges, const TopoDS_Edge &edge)
{
  for (const TopoDS_Edge &listed_edge : edges) {
    if (listed_edge.IsSame(edge)) {
      return true;
    }
  }
  return false;
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
  const double total_start = nurb_body_debug_now();
  const double radius = std::min(double(bevel_radius), hard_limit);
  const bool validate_blended_shape = validate_shape;
  nurb_body_debug_bevel_log("apply_single_begin",
                            "requested=%.8f radius=%.8f hard_limit=%.8f type=%s "
                            "source_index=%d validate=%d fast_preview=%d",
                            double(bevel_radius),
                            radius,
                            hard_limit,
                            bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" : "fillet",
                            source_index,
                            int(validate_blended_shape),
                            int(fast_preview));
  if (radius <= 0.0) {
    nurb_body_debug_bevel_log("apply_single_skip_zero",
                              "requested=%.8f hard_limit=%.8f total_ms=%.3f",
                              double(bevel_radius),
                              hard_limit,
                              (nurb_body_debug_now() - total_start) * 1000.0);
    return shape;
  }
  const ChFi3d_FilletShape fillet_shape = ChFi3d_QuasiAngular;
  thread_local NurbBodyLastGoodSingleBlend last_good_blend;

  auto try_blend_radius = [&](const double test_radius, TopoDS_Shape &r_shape) {
    return try_single_edge_blend(shape,
                                 test_radius,
                                 bevel_type,
                                 edge,
                                 source_index,
                                 validate_blended_shape,
                                 false,
                                 fillet_shape,
                                 r_shape) ||
           try_single_edge_blend(shape,
                                 test_radius,
                                 bevel_type,
                                 edge,
                                 source_index,
                                 validate_blended_shape,
                                 true,
                                 fillet_shape,
                                 r_shape);
  };

  TopoDS_Shape blended_shape;
  if (try_blend_radius(radius, blended_shape)) {
    r_applied = true;
    last_good_single_blend_store(
        last_good_blend, shape, edge, blended_shape, bevel_type, source_index, radius);
    nurb_body_debug_bevel_log("apply_single_direct_success",
                              "radius=%.8f total_ms=%.3f",
                              radius,
                              (nurb_body_debug_now() - total_start) * 1000.0);
    return blended_shape;
  }

  const double fallback_radii[] = {
      radius * 0.90,
      radius * 0.75,
      radius * 0.50,
      radius * 0.25,
      radius * 0.10,
      radius * 0.05,
  };
  for (const double fallback_radius : fallback_radii) {
    if (fallback_radius <= 1.0e-6 || fallback_radius >= radius) {
      continue;
    }
    if (try_blend_radius(fallback_radius, blended_shape)) {
      r_applied = true;
      last_good_single_blend_store(last_good_blend,
                                   shape,
                                   edge,
                                   blended_shape,
                                   bevel_type,
                                   source_index,
                                   fallback_radius,
                                   radius);
      nurb_body_debug_bevel_log("apply_single_fallback_success",
                                "requested=%.8f fallback=%.8f total_ms=%.3f",
                                radius,
                                fallback_radius,
                                (nurb_body_debug_now() - total_start) * 1000.0);
      return blended_shape;
    }
  }

  last_good_single_blend_note_failure(
      last_good_blend, shape, edge, bevel_type, source_index, radius);

  TopoDS_Shape last_good_shape;
  if (last_good_single_blend_find(last_good_blend,
                                  shape,
                                  edge,
                                  bevel_type,
                                  source_index,
                                  radius,
                                  last_good_shape))
  {
    r_applied = true;
    nurb_body_debug_bevel_log("apply_single_failed_use_last_good",
                              "requested=%.8f cached_radius=%.8f failed_radius=%.8f total_ms=%.3f",
                              radius,
                              last_good_blend.radius,
                              last_good_blend.failed_radius,
                              (nurb_body_debug_now() - total_start) * 1000.0);
    return last_good_shape;
  }

  nurb_body_debug_bevel_log("apply_single_failed_keep_base",
                            "radius=%.8f total_ms=%.3f",
                            radius,
                            (nurb_body_debug_now() - total_start) * 1000.0);
  return shape;
}

template<typename EdgeRefinder>
static bool try_uniform_sequential_edge_group(const TopoDS_Shape &shape,
                                              const float radius_limit,
                                              const Span<NurbBodyBlendStep> steps,
                                              const Vector<TopoDS_Edge> &reference_edges,
                                              const Vector<TopoDS_Edge> &initial_current_edges,
                                              const bool initial_edges_match_reference,
                                              const bool validate_shape,
                                              const bool fast_preview,
                                              const ChFi3d_FilletShape fillet_shape,
                                              EdgeRefinder &&refind_edges,
                                              TopoDS_Shape &r_shape)
{
  if (steps.size() < 2) {
    return false;
  }

  const double requested_radius = std::min(double(steps[0].radius),
                                           safe_blend_radius_limit(radius_limit));
  if (requested_radius <= 0.0) {
    return false;
  }

  thread_local NurbBodyLastGoodGroupBlend last_good_group;
  const uint64_t source_mask = blend_step_source_mask(steps);
  constexpr double preview_fallback_scales[] = {1.0};
  constexpr double final_fallback_scales[] = {1.0, 0.98, 0.95, 0.90, 0.80, 0.65};
  const double *fallback_scales = fast_preview ? preview_fallback_scales :
                                                final_fallback_scales;
  const int fallback_scales_num = fast_preview ?
                                      int(sizeof(preview_fallback_scales) /
                                          sizeof(preview_fallback_scales[0])) :
                                      int(sizeof(final_fallback_scales) /
                                          sizeof(final_fallback_scales[0]));
  for (int scale_i = 0; scale_i < fallback_scales_num; scale_i++) {
    const double scale = fallback_scales[scale_i];
    const double radius = requested_radius * scale;
    if (radius <= 1.0e-6) {
      continue;
    }

    TopoDS_Shape group_shape = shape;
    Vector<TopoDS_Edge> current_edges = initial_current_edges;
    Vector<NurbBodyEdgeReference> edge_references;
    Vector<NurbBodyEdgeReference> current_edge_references;
    Vector<char> current_edge_reference_ready;
    bool edge_references_ready = false;
    bool all_current_edge_references_ready = false;
    bool current_edges_match_input = initial_edges_match_reference;

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
      if (current_edges_match_input && step.edge_index >= 0 &&
          step.edge_index < current_edges.size())
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

    bool group_applied = true;
    for (const int step_i : steps.index_range()) {
      const NurbBodyBlendStep &step = steps[step_i];
      const int current_edge = current_edge_for_step(step);
      if (current_edge < 0 || current_edge >= current_edges.size() ||
          current_edges[current_edge].IsNull())
      {
        group_applied = false;
        break;
      }

      TopoDS_Shape next_shape;
      const bool step_applied =
          try_single_edge_blend(group_shape,
                                radius,
                                step.bevel_type,
                                current_edges[current_edge],
                                step.edge_index,
                                validate_shape,
                                false,
                                fillet_shape,
                                next_shape) ||
          (!fast_preview && try_single_edge_blend(group_shape,
                                                  radius,
                                                  step.bevel_type,
                                                  current_edges[current_edge],
                                                  step.edge_index,
                                                  validate_shape,
                                                  true,
                                                  fillet_shape,
                                                  next_shape));
      if (!step_applied)
      {
        group_applied = false;
        break;
      }

      group_shape = next_shape;
      if (step_i + 1 < steps.size()) {
        current_edges = refind_edges(group_shape);
        reset_current_edge_references();
        current_edges_match_input = false;
      }
    }

    if (group_applied) {
      r_shape = group_shape;
      last_good_group_blend_store(
          last_good_group, shape, group_shape, source_mask, NURB_BODY_BEVEL_FILLET, radius);
      if (scale < 1.0) {
        nurb_body_debug_bevel_log("uniform_sequential_group_fallback_success",
                                  "type=fillet edges=%d requested=%.8f fallback=%.8f scale=%.3f",
                                  int(steps.size()),
                                  requested_radius,
                                  radius,
                                  scale);
      }
      return true;
    }

    if (fast_preview &&
        last_good_group_blend_find(
            last_good_group, shape, source_mask, NURB_BODY_BEVEL_FILLET, requested_radius, r_shape))
    {
      nurb_body_debug_bevel_log("uniform_sequential_group_use_last_good",
                                "type=fillet edges=%d requested=%.8f cached=%.8f",
                                int(steps.size()),
                                requested_radius,
                                last_good_group.radius);
      return true;
    }
  }

  return false;
}

static bool try_uniform_sequential_surface_edge_group(
    const TopoDS_Shape &shape,
    const float radius_limit,
    const Span<NurbBodyBlendStep> steps,
    const uint64_t *edge_keys,
    const int samples_per_edge,
    const bool validate_shape,
    const bool fast_preview,
    const ChFi3d_FilletShape fillet_shape,
    const Vector<NurbBodySurfaceEdgeEntry> *initial_edges,
    TopoDS_Shape &r_shape)
{
  if (steps.size() < 2 || edge_keys == nullptr) {
    return false;
  }

  const double requested_radius = std::min(double(steps[0].radius),
                                           safe_blend_radius_limit(radius_limit));
  if (requested_radius <= 0.0) {
    return false;
  }

  thread_local NurbBodyLastGoodGroupBlend last_good_group;
  const uint64_t source_mask = blend_step_source_mask(steps);
  constexpr double preview_fallback_scales[] = {1.0};
  constexpr double final_fallback_scales[] = {1.0, 0.98, 0.95, 0.90, 0.80, 0.65};
  const double *fallback_scales = fast_preview ? preview_fallback_scales :
                                                final_fallback_scales;
  const int fallback_scales_num = fast_preview ?
                                      int(sizeof(preview_fallback_scales) /
                                          sizeof(preview_fallback_scales[0])) :
                                      int(sizeof(final_fallback_scales) /
                                          sizeof(final_fallback_scales[0]));
  for (int scale_i = 0; scale_i < fallback_scales_num; scale_i++) {
    const double scale = fallback_scales[scale_i];
    const double radius = requested_radius * scale;
    if (radius <= 1.0e-6) {
      continue;
    }

    TopoDS_Shape group_shape = shape;
    Vector<NurbBodySurfaceEdgeEntry> current_edges =
        initial_edges != nullptr ? *initial_edges :
                                   find_selectable_surface_edge_catalog(group_shape,
                                                                        samples_per_edge);
    NurbBodyEdgeReference original_edge_references[64];
    bool original_edge_reference_ready[64] = {};
    auto original_edge_reference = [&](const int edge_index) -> const NurbBodyEdgeReference & {
      if (edge_index < 0 || edge_index >= 64) {
        static const NurbBodyEdgeReference empty_reference;
        return empty_reference;
      }
      if (!original_edge_reference_ready[edge_index]) {
        original_edge_reference_ready[edge_index] = true;
        if (edge_keys[edge_index] != 0) {
          const int catalog_edge = find_catalog_edge_for_key(current_edges, edge_keys[edge_index]);
          if (catalog_edge != -1) {
            original_edge_references[edge_index] = surface_edge_entry_reference_for_blend(
                current_edges[catalog_edge]);
          }
        }
      }
      return original_edge_references[edge_index];
    };
    auto current_edge_for_step = [&](const NurbBodyBlendStep &step) {
      if (step.edge_index < 0 || step.edge_index >= 64 || edge_keys[step.edge_index] == 0) {
        return -1;
      }
      return find_catalog_edge_for_key_or_reference(current_edges,
                                                    edge_keys[step.edge_index],
                                                    original_edge_reference(step.edge_index));
    };
    for (const NurbBodyBlendStep &step : steps) {
      original_edge_reference(step.edge_index);
    }

    bool group_applied = true;
    for (const int step_i : steps.index_range()) {
      const NurbBodyBlendStep &step = steps[step_i];
      const int current_edge = current_edge_for_step(step);
      if (current_edge == -1) {
        group_applied = false;
        break;
      }

      TopoDS_Shape next_shape;
      const bool step_applied =
          try_single_edge_blend(group_shape,
                                radius,
                                step.bevel_type,
                                current_edges[current_edge].edge,
                                step.edge_index,
                                validate_shape,
                                false,
                                fillet_shape,
                                next_shape) ||
          (!fast_preview && try_single_edge_blend(group_shape,
                                                  radius,
                                                  step.bevel_type,
                                                  current_edges[current_edge].edge,
                                                  step.edge_index,
                                                  validate_shape,
                                                  true,
                                                  fillet_shape,
                                                  next_shape));
      if (!step_applied)
      {
        group_applied = false;
        break;
      }

      group_shape = next_shape;
      if (step_i + 1 < steps.size()) {
        current_edges = find_selectable_surface_edge_catalog(group_shape, samples_per_edge);
      }
    }

    if (group_applied) {
      r_shape = group_shape;
      last_good_group_blend_store(
          last_good_group, shape, group_shape, source_mask, NURB_BODY_BEVEL_FILLET, radius);
      if (scale < 1.0) {
        nurb_body_debug_bevel_log("uniform_surface_group_fallback_success",
                                  "type=fillet edges=%d requested=%.8f fallback=%.8f scale=%.3f",
                                  int(steps.size()),
                                  requested_radius,
                                  radius,
                                  scale);
      }
      return true;
    }

    if (fast_preview &&
        last_good_group_blend_find(
            last_good_group, shape, source_mask, NURB_BODY_BEVEL_FILLET, requested_radius, r_shape))
    {
      nurb_body_debug_bevel_log("uniform_surface_group_use_last_good",
                                "type=fillet edges=%d requested=%.8f cached=%.8f",
                                int(steps.size()),
                                requested_radius,
                                last_good_group.radius);
      return true;
    }
  }

  return false;
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
  const ChFi3d_FilletShape fillet_shape = ChFi3d_QuasiAngular;
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
    if (batch_candidates.size() > 1) {
      TopoDS_Shape batch_shape;
      const Span<NurbBodyBlendStep> batch_steps = steps.slice(step_i, batch_end - step_i);
      const bool batch_applied =
          step.bevel_type == NURB_BODY_BEVEL_FILLET ?
              try_uniform_sequential_edge_group(result,
                                                radius_limit,
                                                batch_steps,
                                                reference_edges,
                                                current_edges,
                                                current_edges_match_input,
                                                validate_shape,
                                                fast_preview,
                                                fillet_shape,
                                                refind_edges,
                                                batch_shape) :
              try_uniform_edge_set_blend_with_fallback(result,
                                                       step.bevel_type,
                                                       batch_candidates.as_span(),
                                                       validate_shape,
                                                       fillet_shape,
                                                       batch_shape);
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
      nurb_body_debug_bevel_log("edge_blend_unified_batch_failed",
                                "type=%s edges=%d skipped_per_edge_fallback=1 kept_base=1",
                                step.bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" :
                                                                              "fillet",
                                int(batch_candidates.size()));
      step_i = batch_end;
      continue;
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
  NurbBodyEdgeReference original_edge_references[64];
  bool original_edge_reference_ready[64] = {};
  auto original_edge_reference = [&](const int edge_index) -> const NurbBodyEdgeReference & {
    if (edge_index < 0 || edge_index >= 64) {
      static const NurbBodyEdgeReference empty_reference;
      return empty_reference;
    }
    if (!original_edge_reference_ready[edge_index]) {
      original_edge_reference_ready[edge_index] = true;
      if (edge_keys != nullptr && edge_keys[edge_index] != 0) {
        const int catalog_edge = find_catalog_edge_for_key(current_edges, edge_keys[edge_index]);
        if (catalog_edge != -1) {
          original_edge_references[edge_index] = surface_edge_entry_reference_for_blend(
              current_edges[catalog_edge]);
        }
      }
    }
    return original_edge_references[edge_index];
  };
  auto current_edge_for_surface_step = [&](const NurbBodyBlendStep &blend_step) {
    if (blend_step.edge_index < 0 || blend_step.edge_index >= 64 || edge_keys == nullptr ||
        edge_keys[blend_step.edge_index] == 0)
    {
      return -1;
    }
    return find_catalog_edge_for_key_or_reference(current_edges,
                                                  edge_keys[blend_step.edge_index],
                                                  original_edge_reference(blend_step.edge_index));
  };
  for (const NurbBodyBlendStep &step : steps) {
    original_edge_reference(step.edge_index);
  }
  const ChFi3d_FilletShape fillet_shape = ChFi3d_QuasiAngular;
  for (int step_i = 0; step_i < steps.size();) {
    const NurbBodyBlendStep &step = steps[step_i];
    const int current_edge = current_edge_for_surface_step(step);
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
      const int candidate_current_edge = current_edge_for_surface_step(candidate);
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
    if (batch_candidates.size() > 1) {
      TopoDS_Shape batch_shape;
      const Span<NurbBodyBlendStep> batch_steps = steps.slice(step_i, batch_end - step_i);
      const bool batch_applied =
          step.bevel_type == NURB_BODY_BEVEL_FILLET ?
              try_uniform_sequential_surface_edge_group(result,
                                                        radius_limit,
                                                        batch_steps,
                                                        edge_keys,
                                                        samples_per_edge,
                                                        validate_shape,
                                                        fast_preview,
                                                        fillet_shape,
                                                        &current_edges,
                                                        batch_shape) :
              try_uniform_edge_set_blend_with_fallback(result,
                                                       step.bevel_type,
                                                       batch_candidates.as_span(),
                                                       validate_shape,
                                                       fillet_shape,
                                                       batch_shape);
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
      nurb_body_debug_bevel_log("surface_blend_unified_batch_failed",
                                "type=%s edges=%d skipped_per_edge_fallback=1 kept_base=1",
                                step.bevel_type == NURB_BODY_BEVEL_CHAMFER ? "chamfer" :
                                                                              "fillet",
                                int(batch_candidates.size()));
      step_i = batch_end;
      continue;
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
  const uint64_t operand_selected_edges = fast_preview ? op.operand_selected_edges : 0;
  const int operand_selected_edge = fast_preview ? op.operand_selected_edge : -1;
  const uint64_t operand_surface_selected_edges = fast_preview ?
                                                     op.operand_surface_selected_edges :
                                                     0;
  const int operand_surface_selected_edge = fast_preview ? op.operand_surface_selected_edge : -1;

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
                                      operand_selected_edges,
                                      op.operand_bevel_edge,
                                      operand_selected_edge))
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
                             operand_selected_edges,
                             op.operand_bevel_edge,
                             operand_selected_edge,
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
                                      operand_surface_selected_edges,
                                      op.operand_surface_bevel_edge,
                                      operand_surface_selected_edge))
  {
    uint64_t surface_edge_keys[64];
    std::copy_n(op.operand_surface_edge_keys, 64, surface_edge_keys);
    if (nurb_body_boolean_op_has_non_unit_scale(op)) {
      TopoDS_Shape unscaled_shape = make_boolean_op_primitive_shape(op);
      if (blend_settings_may_change_shape(op.operand_bevel_radius,
                                          op.operand_bevel_radii,
                                          op.operand_bevel_edges,
                                          operand_selected_edges,
                                          op.operand_bevel_edge,
                                          operand_selected_edge))
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
                                          operand_selected_edges,
                                          op.operand_bevel_edge,
                                          operand_selected_edge,
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
                                            operand_surface_selected_edges,
                                            op.operand_surface_bevel_edge,
                                            operand_surface_selected_edge,
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

static bool nurb_body_boolean_op_is_surface_blend_stage(const NurbBodyBooleanOp &op)
{
  return op.operation == NURB_BODY_BOOLEAN_SURFACE_BLEND_STAGE;
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
  if (nurb_body_boolean_op_is_surface_blend_stage(op)) {
    nurb_body_stage_hash_value(hash, op.bevel_edges);
    nurb_body_stage_hash_value(hash, op.chamfer_edges);
    nurb_body_stage_hash_value(hash, op.bevel_type);
    nurb_body_stage_hash_value(hash, op.bevel_radius);
    nurb_body_stage_hash_bytes(hash, op.bevel_radii, sizeof(op.bevel_radii));
    nurb_body_stage_hash_bytes(hash, op.bevel_order, sizeof(op.bevel_order));
    nurb_body_stage_hash_bytes(
        hash, op.operand_surface_edge_keys, sizeof(op.operand_surface_edge_keys));
    return;
  }
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
  if (nurb_body_boolean_op_is_surface_blend_stage(op)) {
    return;
  }
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
  const uint64_t active_mask = active_blend_edge_mask(selected_edges,
                                                      bevel_edges,
                                                      bevel_edge,
                                                      selected_edge);
  if (active_mask != 0) {
    const int active_pos = active_blend_step_position(steps.as_span(), active_mask);
    if (active_pos == -1) {
      return false;
    }
  }

  const Vector<NurbBodyBlendStep> stable_steps = active_mask != 0 ?
                                                     stable_blend_steps_without_active(
                                                         steps.as_span(), active_mask) :
                                                     steps;
  nurb_body_stage_hash_value(hash, stable_steps.size());
  for (const NurbBodyBlendStep &step : stable_steps) {
    nurb_body_stage_hash_value(hash, step.edge_index);
    nurb_body_stage_hash_value(hash, step.order);
    nurb_body_stage_hash_value(hash, step.bevel_type);
    nurb_body_stage_hash_value(hash, step.radius);
  }
  return true;
}

static uint64_t nurb_body_body_edge_blend_base_cache_key(const NurbBody &body)
{
  uint64_t hash = 1469598103934665603ull;
  constexpr uint64_t domain = 0x366a820fe9e37f4dull;
  const bool modal_preview = nurb_body_modal_bevel_preview_active(body);
  const uint64_t selected_edges = modal_preview ? body.selected_edges : 0;
  const int selected_edge = modal_preview ? body.selected_edge : -1;
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
                                                      selected_edges,
                                                      body.bevel_edge,
                                                      selected_edge,
                                                      64))
  {
    return 0;
  }
  return hash;
}

static uint64_t nurb_body_pre_boolean_output_blend_cache_key(
    const NurbBody &body, const NurbBodyBooleanOp *target_op);

static uint64_t nurb_body_boolean_output_blend_base_cache_key(const NurbBody &body,
                                                              const NurbBodyBooleanOp &op)
{
  const uint64_t pre_output_key = nurb_body_pre_boolean_output_blend_cache_key(body, &op);
  if (pre_output_key == 0) {
    return 0;
  }
  uint64_t hash = 1469598103934665603ull;
  constexpr uint64_t domain = 0x6ef38dd64bc93423ull;
  const bool modal_preview = nurb_body_modal_bevel_preview_active(body);
  const uint64_t selected_edges = modal_preview ? op.selected_edges : 0;
  const int selected_edge = modal_preview ? op.selected_edge : -1;
  nurb_body_stage_hash_value(hash, domain);
  nurb_body_stage_hash_value(hash, pre_output_key);
  if (!nurb_body_stage_hash_edge_blend_without_active(hash,
                                                      op.bevel_radius,
                                                      op.bevel_radii,
                                                      op.bevel_type,
                                                      op.bevel_edges,
                                                      op.chamfer_edges,
                                                      op.bevel_order,
                                                      selected_edges,
                                                      op.bevel_edge,
                                                      selected_edge,
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
  nurb_body_stage_hash_body_base(hash, body, false);

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

static uint64_t nurb_body_surface_edge_blend_base_cache_key(const NurbBody &body)
{
  const uint64_t pre_surface_key = nurb_body_pre_surface_blend_cache_key(body);
  if (pre_surface_key == 0) {
    return 0;
  }
  uint64_t hash = 1469598103934665603ull;
  constexpr uint64_t domain = 0xaf38bc449d2d8d17ull;
  const bool modal_preview = nurb_body_modal_bevel_preview_active(body);
  const uint64_t selected_edges = modal_preview ? body.surface_selected_edges : 0;
  const int selected_edge = modal_preview ? body.surface_selected_edge : -1;
  nurb_body_stage_hash_value(hash, domain);
  nurb_body_stage_hash_value(hash, pre_surface_key);
  if (!nurb_body_stage_hash_edge_blend_without_active(hash,
                                                      body.surface_bevel_radius,
                                                      body.surface_bevel_radii,
                                                      body.surface_bevel_type,
                                                      body.surface_bevel_edges,
                                                      body.surface_chamfer_edges,
                                                      body.surface_bevel_order,
                                                      selected_edges,
                                                      body.surface_bevel_edge,
                                                      selected_edge,
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
  const uint64_t active_mask = active_blend_edge_mask(selected_edges,
                                                      bevel_edges,
                                                      bevel_edge,
                                                      selected_edge);
  const int active_pos = active_blend_step_position(steps.as_span(), active_mask);
  const bool active_is_last_step = active_blend_steps_are_last(steps.as_span(), active_mask);
  const bool can_cache_stable_base = active_is_last_step;
  const Vector<NurbBodyBlendStep> stable_steps = stable_blend_steps_without_active(
      steps.as_span(), active_mask);
  const Vector<NurbBodyBlendStep> active_steps = blend_steps_for_active_mask(steps.as_span(),
                                                                             active_mask);
  nurb_body_debug_bevel_log("edge_blend_local_preview_begin",
                            "steps=%d stable_steps=%d active_steps=%d active_edge=%d "
                            "active_mask=%llu selected_edges=%llu bevel_edges=%llu "
                            "chamfer_edges=%llu active_pos=%d active_last=%d "
                            "can_cache=%d key=%llu fast_preview=%d validate=%d",
                            int(steps.size()),
                            int(stable_steps.size()),
                            int(active_steps.size()),
                            active_edge,
                            static_cast<unsigned long long>(active_mask),
                            static_cast<unsigned long long>(selected_edges),
                            static_cast<unsigned long long>(bevel_edges),
                            static_cast<unsigned long long>(chamfer_edges),
                            active_pos,
                            int(active_is_last_step),
                            int(can_cache_stable_base),
                            static_cast<unsigned long long>(preview_base_key),
                            int(fast_preview),
                            int(validate_shape));
  if (active_mask == 0 && object != nullptr && preview_base_key != 0) {
    TopoDS_Shape cached_committed;
    if (nurb_body_stage_shape_cache_find(object, preview_base_key, cached_committed)) {
      nurb_body_debug_bevel_log("edge_blend_committed_cache_hit",
                                "key=%llu steps=%d",
                                static_cast<unsigned long long>(preview_base_key),
                                int(steps.size()));
      return cached_committed;
    }

    const double committed_start = nurb_body_debug_now();
    TopoDS_Shape committed_shape = apply_edge_blend_steps(shape,
                                                          radius_limit,
                                                          steps.as_span(),
                                                          edges,
                                                          edges,
                                                          true,
                                                          validate_shape,
                                                          false,
                                                          refind_edges);
    nurb_body_stage_shape_cache_store(object, preview_base_key, committed_shape);
    nurb_body_debug_bevel_log("edge_blend_committed_cache_store",
                              "key=%llu steps=%d ms=%.3f",
                              static_cast<unsigned long long>(preview_base_key),
                              int(steps.size()),
                              (nurb_body_debug_now() - committed_start) * 1000.0);
    return committed_shape;
  }
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

  if (!stable_steps.is_empty()) {
    TopoDS_Shape cached_base;
    Vector<TopoDS_Edge> cached_edges;
    if (nurb_body_stage_shape_cache_find_edges(
            object, preview_base_key, cached_base, cached_edges))
    {
      nurb_body_debug_bevel_log("edge_blend_local_preview_base_cache_hit",
                                "key=%llu cached_edges=%d",
                                static_cast<unsigned long long>(preview_base_key),
                                int(cached_edges.size()));
      active_base = cached_base;
      current_edges = std::move(cached_edges);
    }
    else {
      if (nurb_body_stage_shape_cache_find(object, preview_base_key, cached_base)) {
        nurb_body_debug_bevel_log("edge_blend_local_preview_shape_cache_hit",
                                  "key=%llu",
                                  static_cast<unsigned long long>(preview_base_key));
        active_base = cached_base;
      }
      else {
        const double stable_start = nurb_body_debug_now();
        active_base = apply_edge_blend_steps(shape,
                                             radius_limit,
                                             stable_steps.as_span(),
                                             edges,
                                             edges,
                                             true,
                                             validate_shape,
                                             false,
                                             refind_edges);
        nurb_body_debug_bevel_log("edge_blend_local_preview_stable_built",
                                  "stable_steps=%d ms=%.3f",
                                  int(stable_steps.size()),
                                  (nurb_body_debug_now() - stable_start) * 1000.0);
      }
      const double refind_start = nurb_body_debug_now();
      current_edges = refind_edges(active_base);
      nurb_body_debug_bevel_log("edge_blend_local_preview_refind",
                                "edges=%d ms=%.3f",
                                int(current_edges.size()),
                                (nurb_body_debug_now() - refind_start) * 1000.0);
      nurb_body_stage_shape_cache_store_edges(
          object, preview_base_key, active_base, current_edges.as_span());
    }
    current_edges_match_input = false;
  }

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
  const uint64_t active_mask = active_blend_edge_mask(selected_edges,
                                                      bevel_edges,
                                                      bevel_edge,
                                                      selected_edge);
  const int active_pos = active_blend_step_position(steps.as_span(), active_mask);
  const bool active_is_last_step = active_blend_steps_are_last(steps.as_span(), active_mask);
  const bool can_cache_stable_base = active_is_last_step;
  const Vector<NurbBodyBlendStep> stable_steps = stable_blend_steps_without_active(
      steps.as_span(), active_mask);
  const Vector<NurbBodyBlendStep> active_steps = blend_steps_for_active_mask(steps.as_span(),
                                                                             active_mask);
  nurb_body_debug_bevel_log("surface_blend_local_preview_begin",
                            "steps=%d stable_steps=%d active_steps=%d active_edge=%d "
                            "active_mask=%llu selected_edges=%llu bevel_edges=%llu "
                            "chamfer_edges=%llu active_pos=%d active_last=%d "
                            "can_cache=%d key=%llu fast_preview=%d validate=%d",
                            int(steps.size()),
                            int(stable_steps.size()),
                            int(active_steps.size()),
                            active_edge,
                            static_cast<unsigned long long>(active_mask),
                            static_cast<unsigned long long>(selected_edges),
                            static_cast<unsigned long long>(bevel_edges),
                            static_cast<unsigned long long>(chamfer_edges),
                            active_pos,
                            int(active_is_last_step),
                            int(can_cache_stable_base),
                            static_cast<unsigned long long>(preview_base_key),
                            int(fast_preview),
                            int(validate_shape));
  if (active_mask == 0 && object != nullptr && preview_base_key != 0) {
    if (r_applied_edges != nullptr) {
      *r_applied_edges = 0;
      for (const NurbBodyBlendStep &step : steps) {
        *r_applied_edges |= nurb_body_edge_mask_for_index(step.edge_index);
      }
    }

    TopoDS_Shape cached_committed;
    if (nurb_body_stage_shape_cache_find(object, preview_base_key, cached_committed)) {
      nurb_body_debug_bevel_log("surface_blend_committed_cache_hit",
                                "key=%llu steps=%d",
                                static_cast<unsigned long long>(preview_base_key),
                                int(steps.size()));
      return cached_committed;
    }

    const double committed_start = nurb_body_debug_now();
    TopoDS_Shape committed_shape = apply_stable_surface_edge_blend_steps(shape,
                                                                         radius_limit,
                                                                         steps.as_span(),
                                                                         edge_keys,
                                                                         samples_per_edge,
                                                                         validate_shape,
                                                                         false,
                                                                         nullptr,
                                                                         nullptr);
    nurb_body_stage_shape_cache_store(object, preview_base_key, committed_shape);
    nurb_body_debug_bevel_log("surface_blend_committed_cache_store",
                              "key=%llu steps=%d ms=%.3f",
                              static_cast<unsigned long long>(preview_base_key),
                              int(steps.size()),
                              (nurb_body_debug_now() - committed_start) * 1000.0);
    return committed_shape;
  }
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

  if (!stable_steps.is_empty()) {
    for (const NurbBodyBlendStep &step : stable_steps) {
      stable_applied_edges |= nurb_body_edge_mask_for_index(step.edge_index);
    }

    TopoDS_Shape cached_base;
    Vector<NurbBodySurfaceEdgeEntry> cached_edges;
    if (nurb_body_stage_shape_cache_find_surface_edges(
            object, preview_base_key, cached_base, cached_edges))
    {
      nurb_body_debug_bevel_log("surface_blend_local_preview_base_cache_hit",
                                "key=%llu cached_edges=%d",
                                static_cast<unsigned long long>(preview_base_key),
                                int(cached_edges.size()));
      active_base = cached_base;
      current_edges = std::move(cached_edges);
    }
    else {
      if (nurb_body_stage_shape_cache_find(object, preview_base_key, cached_base)) {
        nurb_body_debug_bevel_log("surface_blend_local_preview_shape_cache_hit",
                                  "key=%llu",
                                  static_cast<unsigned long long>(preview_base_key));
        active_base = cached_base;
      }
      else {
        const double stable_start = nurb_body_debug_now();
        active_base = apply_stable_surface_edge_blend_steps(shape,
                                                            radius_limit,
                                                            stable_steps.as_span(),
                                                            edge_keys,
                                                            samples_per_edge,
                                                            validate_shape,
                                                            false,
                                                            nullptr,
                                                            nullptr);
        nurb_body_debug_bevel_log("surface_blend_local_preview_stable_built",
                                  "stable_steps=%d ms=%.3f",
                                  int(stable_steps.size()),
                                  (nurb_body_debug_now() - stable_start) * 1000.0);
      }
      const double refind_start = nurb_body_debug_now();
      current_edges = find_selectable_surface_edge_catalog(active_base, samples_per_edge);
      nurb_body_debug_bevel_log("surface_blend_local_preview_refind",
                                "edges=%d ms=%.3f",
                                int(current_edges.size()),
                                (nurb_body_debug_now() - refind_start) * 1000.0);
      nurb_body_stage_shape_cache_store_surface_edges(
          object, preview_base_key, active_base, current_edges.as_span());
    }
  }

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

static TopoDS_Shape apply_boolean_surface_blend_stage(const NurbBody &body,
                                                      const Object *object,
                                                      const NurbBodyBooleanOp &op,
                                                      const TopoDS_Shape &shape,
                                                      const bool validate_shape)
{
  const uint64_t stage_key = object != nullptr ?
                                 nurb_body_pre_boolean_output_blend_cache_key(body, &op) :
                                 0;
  TopoDS_Shape cached_stage;
  if (nurb_body_stage_shape_cache_find(object, stage_key, cached_stage)) {
    nurb_body_debug_bevel_log("surface_stage_cache_hit",
                              "op=%p key=%llu",
                              static_cast<const void *>(&op),
                              static_cast<unsigned long long>(stage_key));
    return cached_stage;
  }

  const double stage_start = nurb_body_debug_now();
  TopoDS_Shape result = apply_stable_surface_edge_blend(shape,
                                                        nurb_body_blend_radius_limit(body),
                                                        op.bevel_radius,
                                                        op.bevel_radii,
                                                        op.bevel_type,
                                                        op.bevel_edges,
                                                        op.chamfer_edges,
                                                        op.bevel_order,
                                                        0,
                                                        op.bevel_edge,
                                                        -1,
                                                        op.operand_surface_edge_keys,
                                                        64,
                                                        validate_shape,
                                                        false,
                                                        nullptr);
  nurb_body_stage_shape_cache_store(object, stage_key, result);
  nurb_body_debug_bevel_log("surface_stage_cache_store",
                            "op=%p key=%llu ms=%.3f",
                            static_cast<const void *>(&op),
                            static_cast<unsigned long long>(stage_key),
                            (nurb_body_debug_now() - stage_start) * 1000.0);
  return result;
}

static TopoDS_Shape evaluate_shape(const NurbBody &body, const Object *object)
{
  const double total_start = nurb_body_debug_now();
  TopoDS_Shape result = make_body_primitive_shape(body);
  nurb_body_debug_bevel_log("evaluate_make_body",
                            "object=%p primitive=%d ms=%.3f",
                            static_cast<const void *>(object),
                            body.primitive,
                            (nurb_body_debug_now() - total_start) * 1000.0);
  const bool fast_preview = nurb_body_modal_bevel_preview_active(body);
  const uint64_t body_selected_edges = fast_preview ? body.selected_edges : 0;
  const int body_selected_edge = fast_preview ? body.selected_edge : -1;
  const uint64_t surface_selected_edges = fast_preview ? body.surface_selected_edges : 0;
  const int surface_selected_edge = fast_preview ? body.surface_selected_edge : -1;
  if (blend_settings_may_change_shape(body.bevel_radius,
                                      body.bevel_radii,
                                      body.bevel_edges,
                                      body_selected_edges,
                                      body.bevel_edge,
                                      body_selected_edge))
  {
    const double body_blend_start = nurb_body_debug_now();
    const Vector<TopoDS_Edge> body_edges = find_selectable_surface_edges(result);
    nurb_body_debug_bevel_log("evaluate_body_edges",
                              "edges=%d ms=%.3f",
                              int(body_edges.size()),
                              (nurb_body_debug_now() - body_blend_start) * 1000.0);
    const uint64_t preview_base_key = nurb_body_body_edge_blend_base_cache_key(body);
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
                                            body_selected_edges,
                                            body.bevel_edge,
                                            body_selected_edge,
                                            body_edges,
                                            true,
                                            fast_preview,
                                            [](const TopoDS_Shape &shape) {
                                              return find_selectable_surface_edges(shape);
                                            });
    nurb_body_debug_bevel_log("evaluate_body_blend_done",
                              "edges=%d key=%llu ms=%.3f",
                              int(body_edges.size()),
                              static_cast<unsigned long long>(preview_base_key),
                              (nurb_body_debug_now() - body_blend_start) * 1000.0);
  }

  if (object != nullptr && !BLI_listbase_is_empty(&body.boolean_ops)) {
    for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(
             body.boolean_ops.first);
         op;
         op = op->next)
    {
      if (nurb_body_boolean_op_is_surface_blend_stage(*op)) {
        const double stage_start = nurb_body_debug_now();
        result = apply_boolean_surface_blend_stage(body, object, *op, result, true);
        nurb_body_debug_bevel_log("evaluate_surface_stage_done",
                                  "op=%p total_ms=%.3f",
                                  static_cast<const void *>(op),
                                  (nurb_body_debug_now() - stage_start) * 1000.0);
        continue;
      }
      if (op->operand_radius <= 0.0f || op->operand_depth <= 0.0f) {
        continue;
      }
      const double boolean_start = nurb_body_debug_now();
      const TopoDS_Shape pre_boolean_shape = result;
      const uint64_t op_selected_edges = fast_preview ? op->selected_edges : 0;
      const int op_selected_edge = fast_preview ? op->selected_edge : -1;
      const bool output_blend_requested = blend_settings_may_change_shape(op->bevel_radius,
                                                                          op->bevel_radii,
                                                                          op->bevel_edges,
                                                                          op_selected_edges,
                                                                          op->bevel_edge,
                                                                          op_selected_edge);

      const uint64_t pre_output_blend_key = nurb_body_pre_boolean_output_blend_cache_key(body,
                                                                                        op);
      TopoDS_Shape cached_pre_output_blend;
      if (nurb_body_stage_shape_cache_find(object, pre_output_blend_key, cached_pre_output_blend))
      {
        result = cached_pre_output_blend;
        nurb_body_debug_bevel_log("evaluate_boolean_pre_cache_hit",
                                  "op=%p key=%llu ms=%.3f",
                                  static_cast<const void *>(op),
                                  static_cast<unsigned long long>(pre_output_blend_key),
                                  (nurb_body_debug_now() - boolean_start) * 1000.0);
      }
      else {
        const double tool_start = nurb_body_debug_now();
        const TopoDS_Shape tool = make_boolean_op_tool_shape(*op, true, fast_preview);
        nurb_body_debug_bevel_log("evaluate_boolean_tool",
                                  "op=%p ms=%.3f",
                                  static_cast<const void *>(op),
                                  (nurb_body_debug_now() - tool_start) * 1000.0);
        const double op_start = nurb_body_debug_now();
        result = apply_boolean_operation(result, tool, op->operation);
        nurb_body_debug_bevel_log("evaluate_boolean_apply",
                                  "op=%p operation=%d ms=%.3f",
                                  static_cast<const void *>(op),
                                  op->operation,
                                  (nurb_body_debug_now() - op_start) * 1000.0);
        nurb_body_stage_shape_cache_store(object, pre_output_blend_key, result);
      }
      if (output_blend_requested) {
        const double output_start = nurb_body_debug_now();
        const Vector<uint64_t> pre_boolean_edge_keys = selectable_edge_geometry_keys(
            pre_boolean_shape);
        nurb_body_debug_bevel_log("evaluate_boolean_pre_keys",
                                  "op=%p keys=%d ms=%.3f",
                                  static_cast<const void *>(op),
                                  int(pre_boolean_edge_keys.size()),
                                  (nurb_body_debug_now() - output_start) * 1000.0);
        const float output_blend_radius_limit = std::max(
            nurb_body_blend_radius_limit(body),
            nurb_body_boolean_op_scaled_blend_radius_limit(*op));
        const double cut_edges_start = nurb_body_debug_now();
        const Vector<TopoDS_Edge> cut_edges = find_boolean_output_edges(
            result, pre_boolean_edge_keys.as_span());
        nurb_body_debug_bevel_log("evaluate_boolean_cut_edges",
                                  "op=%p cut_edges=%d ms=%.3f",
                                  static_cast<const void *>(op),
                                  int(cut_edges.size()),
                                  (nurb_body_debug_now() - cut_edges_start) * 1000.0);
        const uint64_t preview_base_key = nurb_body_boolean_output_blend_base_cache_key(body,
                                                                                       *op);
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
                                                op_selected_edges,
                                                op->bevel_edge,
                                                op_selected_edge,
                                                cut_edges,
                                                true,
                                                fast_preview,
                                                [&](const TopoDS_Shape &shape) {
                                                  return find_boolean_output_edges(
                                                      shape, pre_boolean_edge_keys.as_span());
                                                });
        nurb_body_debug_bevel_log("evaluate_boolean_output_blend_done",
                                  "op=%p cut_edges=%d key=%llu ms=%.3f",
                                  static_cast<const void *>(op),
                                  int(cut_edges.size()),
                                  static_cast<unsigned long long>(preview_base_key),
                                  (nurb_body_debug_now() - output_start) * 1000.0);
      }
      nurb_body_debug_bevel_log("evaluate_boolean_done",
                                "op=%p total_ms=%.3f",
                                static_cast<const void *>(op),
                                (nurb_body_debug_now() - boolean_start) * 1000.0);
    }
  }

  if (blend_settings_may_change_shape(body.surface_bevel_radius,
                                      body.surface_bevel_radii,
                                      body.surface_bevel_edges,
                                      surface_selected_edges,
                                      body.surface_bevel_edge,
                                      surface_selected_edge))
  {
    const double surface_start = nurb_body_debug_now();
    if (object != nullptr) {
      const uint64_t pre_surface_key = nurb_body_pre_surface_blend_cache_key(body);
      TopoDS_Shape cached_pre_surface;
      if (nurb_body_stage_shape_cache_find(object, pre_surface_key, cached_pre_surface)) {
        result = cached_pre_surface;
        nurb_body_debug_bevel_log("evaluate_surface_pre_cache_hit",
                                  "key=%llu ms=%.3f",
                                  static_cast<unsigned long long>(pre_surface_key),
                                  (nurb_body_debug_now() - surface_start) * 1000.0);
      }
      else {
        nurb_body_stage_shape_cache_store(object, pre_surface_key, result);
      }
    }
    const uint64_t preview_base_key = nurb_body_surface_edge_blend_base_cache_key(body);
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
                                                           surface_selected_edges,
                                                           body.surface_bevel_edge,
                                                           surface_selected_edge,
                                                           body.surface_edge_keys,
                                                           64,
                                                           true,
                                                           fast_preview,
                                                           nullptr);
    nurb_body_debug_bevel_log("evaluate_surface_blend_done",
                              "key=%llu ms=%.3f",
                              static_cast<unsigned long long>(preview_base_key),
                              (nurb_body_debug_now() - surface_start) * 1000.0);
  }

  if (!result.IsNull()) {
    const double clean_start = nurb_body_debug_now();
    BRepTools::Clean(result);
    nurb_body_debug_bevel_log("evaluate_clean",
                              "ms=%.3f",
                              (nurb_body_debug_now() - clean_start) * 1000.0);
  }
  nurb_body_debug_bevel_log("evaluate_done",
                            "total_ms=%.3f",
                            (nurb_body_debug_now() - total_start) * 1000.0);
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
  const double total_start = nurb_body_debug_now();
  nurb_body_debug_bevel_log("polyline_sample_begin",
                            "object=%p samples=%d",
                            static_cast<const void *>(object),
                            samples_per_edge);
  TopoDS_Shape result = make_body_primitive_shape(body);
  const bool fast_preview = nurb_body_modal_bevel_preview_active(body);
  const uint64_t body_selected_edges = fast_preview ? body.selected_edges : 0;
  const int body_selected_edge = fast_preview ? body.selected_edge : -1;
  const uint64_t surface_selected_edges = fast_preview ? body.surface_selected_edges : 0;
  const int surface_selected_edge = fast_preview ? body.surface_selected_edge : -1;
  r_polylines.clear();
  Vector<NurbBodySelectableEdgeRef> selectable_refs;
  const float base_threshold = std::max(0.05f, body.tessellation_deflection * 8.0f);

  TopTools_IndexedDataMapOfShapeListOfShape body_edge_faces;
  const double body_refs_start = nurb_body_debug_now();
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
  nurb_body_debug_bevel_log("polyline_body_refs",
                            "body_edges=%d refs=%d ms=%.3f",
                            int(body_edges.size()),
                            int(selectable_refs.size()),
                            (nurb_body_debug_now() - body_refs_start) * 1000.0);
  const uint64_t body_preview_base_key = nurb_body_body_edge_blend_base_cache_key(body);
  const double body_blend_start = nurb_body_debug_now();
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
                                          body_selected_edges,
                                          body.bevel_edge,
                                          body_selected_edge,
                                          body_edges,
                                          true,
                                          fast_preview,
                                          [](const TopoDS_Shape &shape) {
                                            return find_selectable_surface_edges(shape);
                                          });
  nurb_body_debug_bevel_log("polyline_body_blend_done",
                            "key=%llu ms=%.3f",
                            static_cast<unsigned long long>(body_preview_base_key),
                            (nurb_body_debug_now() - body_blend_start) * 1000.0);

  for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(body.boolean_ops.first);
       op;
       op = op->next)
  {
    if (nurb_body_boolean_op_is_surface_blend_stage(*op)) {
      const double stage_start = nurb_body_debug_now();
      result = apply_boolean_surface_blend_stage(body, object, *op, result, true);
      nurb_body_debug_bevel_log("polyline_surface_stage_done",
                                "op=%p total_ms=%.3f",
                                static_cast<const void *>(op),
                                (nurb_body_debug_now() - stage_start) * 1000.0);
      continue;
    }
    if (op->operand_radius <= 0.0f || op->operand_depth <= 0.0f) {
      continue;
    }
    const double boolean_start = nurb_body_debug_now();
    const TopoDS_Shape pre_boolean_shape = result;
    const Vector<uint64_t> pre_boolean_edge_keys = selectable_edge_geometry_keys(
        pre_boolean_shape);
    const float output_blend_radius_limit = std::max(
        nurb_body_blend_radius_limit(body), nurb_body_boolean_op_scaled_blend_radius_limit(*op));

    const uint64_t pre_output_blend_key = object != nullptr ?
                                              nurb_body_pre_boolean_output_blend_cache_key(body,
                                                                                          op) :
                                              0;
    const uint64_t op_selected_edges = fast_preview ? op->selected_edges : 0;
    const int op_selected_edge = fast_preview ? op->selected_edge : -1;
    TopoDS_Shape cached_pre_output_blend;
    if (nurb_body_stage_shape_cache_find(object, pre_output_blend_key, cached_pre_output_blend)) {
      result = cached_pre_output_blend;
      nurb_body_debug_bevel_log("polyline_boolean_pre_cache_hit",
                                "op=%p key=%llu ms=%.3f",
                                static_cast<const void *>(op),
                                static_cast<unsigned long long>(pre_output_blend_key),
                                (nurb_body_debug_now() - boolean_start) * 1000.0);
    }
    else {
      const double tool_start = nurb_body_debug_now();
      const TopoDS_Shape tool = make_boolean_op_tool_shape(*op, true, fast_preview);
      nurb_body_debug_bevel_log("polyline_boolean_tool",
                                "op=%p ms=%.3f",
                                static_cast<const void *>(op),
                                (nurb_body_debug_now() - tool_start) * 1000.0);
      const double apply_start = nurb_body_debug_now();
      result = apply_boolean_operation(result, tool, op->operation);
      nurb_body_debug_bevel_log("polyline_boolean_apply",
                                "op=%p operation=%d ms=%.3f",
                                static_cast<const void *>(op),
                                op->operation,
                                (nurb_body_debug_now() - apply_start) * 1000.0);
      nurb_body_stage_shape_cache_store(object, pre_output_blend_key, result);
    }

    const double cut_start = nurb_body_debug_now();
    TopTools_IndexedDataMapOfShapeListOfShape op_edge_faces;
    TopExp::MapShapesAndAncestors(result, TopAbs_EDGE, TopAbs_FACE, op_edge_faces);
    const Vector<TopoDS_Edge> cut_edges = find_boolean_output_edges(
        op_edge_faces, pre_boolean_edge_keys.as_span());
    nurb_body_debug_bevel_log("polyline_boolean_cut_edges",
                              "op=%p cut_edges=%d ms=%.3f",
                              static_cast<const void *>(op),
                              int(cut_edges.size()),
                              (nurb_body_debug_now() - cut_start) * 1000.0);
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

    const uint64_t preview_base_key = nurb_body_boolean_output_blend_base_cache_key(body, *op);
    const double output_blend_start = nurb_body_debug_now();
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
                                            op_selected_edges,
                                            op->bevel_edge,
                                            op_selected_edge,
                                            cut_edges,
                                            true,
                                            fast_preview,
                                            [&](const TopoDS_Shape &shape) {
                                              return find_boolean_output_edges(
                                                  shape, pre_boolean_edge_keys.as_span());
                                            });
    nurb_body_debug_bevel_log("polyline_boolean_output_blend_done",
                              "op=%p key=%llu ms=%.3f",
                              static_cast<const void *>(op),
                              static_cast<unsigned long long>(preview_base_key),
                              (nurb_body_debug_now() - output_blend_start) * 1000.0);
    nurb_body_debug_bevel_log("polyline_boolean_done",
                              "op=%p total_ms=%.3f",
                              static_cast<const void *>(op),
                              (nurb_body_debug_now() - boolean_start) * 1000.0);
  }

  uint64_t surface_requested_edges = body.surface_bevel_edges;
  if (surface_requested_edges == 0) {
    surface_requested_edges = body.surface_bevel_edge >= 0 ?
                                  nurb_body_edge_mask_for_index(body.surface_bevel_edge) :
                                  surface_selected_edges;
  }
  uint64_t surface_applied_edges = 0;
  if (blend_settings_may_change_shape(body.surface_bevel_radius,
                                      body.surface_bevel_radii,
                                      body.surface_bevel_edges,
                                      surface_selected_edges,
                                      body.surface_bevel_edge,
                                      surface_selected_edge))
  {
    const double surface_start = nurb_body_debug_now();
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
    const uint64_t preview_base_key = nurb_body_surface_edge_blend_base_cache_key(body);
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
                                                           surface_selected_edges,
                                                           body.surface_bevel_edge,
                                                           surface_selected_edge,
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
    nurb_body_debug_bevel_log("polyline_surface_done",
                              "surface_edges=%d applied_mask=%llu refs=%d ms=%.3f",
                              int(surface_edges.size()),
                              static_cast<unsigned long long>(surface_applied_edges),
                              int(selectable_refs.size()),
                              (nurb_body_debug_now() - surface_start) * 1000.0);
  }

  const double append_start = nurb_body_debug_now();
  append_shape_surface_edge_polylines(result, samples_per_edge, selectable_refs, body, r_polylines);
  nurb_body_debug_bevel_log("polyline_append_surface",
                            "polylines=%d ms=%.3f",
                            int(r_polylines.size()),
                            (nurb_body_debug_now() - append_start) * 1000.0);
  nurb_body_evaluated_shape_cache_store(body, object, result);
  nurb_body_debug_bevel_log("polyline_sample_done",
                            "polylines=%d total_ms=%.3f",
                            int(r_polylines.size()),
                            (nurb_body_debug_now() - total_start) * 1000.0);
}

static void sample_evaluated_shape_edge_polylines(const NurbBody &body,
                                                  const TopoDS_Shape &shape,
                                                  const int samples_per_edge,
                                                  Vector<NurbBodyEdgePolyline> &r_polylines)
{
  const double total_start = nurb_body_debug_now();
  r_polylines.clear();
  if (shape.IsNull()) {
    nurb_body_debug_bevel_log("polyline_from_evaluated_shape_skip",
                              "shape_null=1 samples=%d",
                              samples_per_edge);
    return;
  }

  append_shape_surface_edge_polylines(shape, samples_per_edge, {}, body, r_polylines);
  nurb_body_debug_bevel_log("polyline_from_evaluated_shape_done",
                            "polylines=%d samples=%d total_ms=%.3f",
                            int(r_polylines.size()),
                            samples_per_edge,
                            (nurb_body_debug_now() - total_start) * 1000.0);
}

static int64_t edge_key(const int v1, const int v2)
{
  const uint64_t a = uint64_t(std::min(v1, v2));
  const uint64_t b = uint64_t(std::max(v1, v2));
  return int64_t((a << 32) | b);
}

struct NurbBodyMeshFace {
  Vector<int, 4> verts;
  Vector<float3, 4> normals;
};

struct NurbBodyMeshTriangle {
  int3 verts = {};
  float3 normals[3] = {};
  int face_index = -1;
  bool planar_face = false;
};

static void append_triangle_mesh_face(Vector<NurbBodyMeshFace> &faces,
                                      const NurbBodyMeshTriangle &tri)
{
  NurbBodyMeshFace face;
  face.verts.append(tri.verts[0]);
  face.verts.append(tri.verts[1]);
  face.verts.append(tri.verts[2]);
  face.normals.append(tri.normals[0]);
  face.normals.append(tri.normals[1]);
  face.normals.append(tri.normals[2]);
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

static float quad_shape_score(const int loop[4], const Span<float3> positions)
{
  float min_edge = FLT_MAX;
  float max_edge = 0.0f;
  for (int i = 0; i < 4; i++) {
    const float length = len_v3v3(positions[loop[i]], positions[loop[(i + 1) % 4]]);
    min_edge = std::min(min_edge, length);
    max_edge = std::max(max_edge, length);
  }
  if (min_edge <= 1.0e-8f) {
    return FLT_MAX;
  }

  float score = max_edge / min_edge;
  for (int i = 0; i < 4; i++) {
    float3 prev = positions[loop[(i + 3) % 4]] - positions[loop[i]];
    float3 next = positions[loop[(i + 1) % 4]] - positions[loop[i]];
    if (normalize_v3(prev) == 0.0f || normalize_v3(next) == 0.0f) {
      return FLT_MAX;
    }
    score += std::fabs(dot_v3v3(prev, next)) * 2.0f;
  }

  const float diag_a = len_v3v3(positions[loop[0]], positions[loop[2]]);
  const float diag_b = len_v3v3(positions[loop[1]], positions[loop[3]]);
  const float min_diag = std::min(diag_a, diag_b);
  const float max_diag = std::max(diag_a, diag_b);
  if (min_diag > 1.0e-8f) {
    score += (max_diag / min_diag) * 0.25f;
  }
  return score;
}

static bool try_make_quad_from_tri_pair(const NurbBodyMeshTriangle &mesh_tri_a,
                                        const NurbBodyMeshTriangle &mesh_tri_b,
                                        const Span<float3> positions,
                                        const float max_merge_angle,
                                        NurbBodyMeshFace &r_face,
                                        float *r_score = nullptr)
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
  float other_tri_no[3];
  float quad_no[3];
  if (normal_tri_v3(tri_no, positions[tri_a[0]], positions[tri_a[1]], positions[tri_a[2]]) ==
      0.0f)
  {
    return false;
  }
  if (normal_tri_v3(other_tri_no,
                    positions[tri_b[0]],
                    positions[tri_b[1]],
                    positions[tri_b[2]]) == 0.0f)
  {
    return false;
  }
  const float normal_threshold = std::cos(std::clamp(max_merge_angle, 0.01f, 3.14159f));
  if (dot_v3v3(tri_no, other_tri_no) < normal_threshold) {
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
  const float score = quad_shape_score(loop, positions);
  if (!std::isfinite(score)) {
    return false;
  }

  r_face.verts.clear();
  r_face.normals.clear();
  for (int i = 0; i < 4; i++) {
    r_face.verts.append(loop[i]);
    r_face.normals.append(normal_for_quad_vertex(mesh_tri_a, mesh_tri_b, loop[i]));
  }
  if (r_score != nullptr) {
    *r_score = score;
  }
  return true;
}

static bool triangle_geometric_normal(const NurbBodyMeshTriangle &tri,
                                      const Span<float3> positions,
                                      float3 &r_normal)
{
  if (normal_tri_v3(r_normal,
                    positions[tri.verts[0]],
                    positions[tri.verts[1]],
                    positions[tri.verts[2]]) == 0.0f)
  {
    return false;
  }
  return normalize_v3(r_normal) != 0.0f;
}

static bool add_ngon_boundary_neighbor(Map<int, int2> &neighbors,
                                       const int vertex,
                                       const int neighbor)
{
  int2 &slots = neighbors.lookup_or_add(vertex, int2(-1, -1));
  if (slots[0] == neighbor || slots[1] == neighbor) {
    return true;
  }
  if (slots[0] == -1) {
    slots[0] = neighbor;
    return true;
  }
  if (slots[1] == -1) {
    slots[1] = neighbor;
    return true;
  }
  return false;
}

static bool try_make_planar_ngon_from_triangles(const Span<NurbBodyMeshTriangle> face_triangles,
                                                const Span<float3> positions,
                                                const float plane_angle,
                                                NurbBodyMeshFace &r_face)
{
  if (face_triangles.is_empty()) {
    return false;
  }

  float3 base_normal(0.0f);
  bool found_normal = false;
  for (const NurbBodyMeshTriangle &tri : face_triangles) {
    if (!tri.planar_face) {
      return false;
    }
    if (!found_normal && triangle_geometric_normal(tri, positions, base_normal)) {
      found_normal = true;
    }
  }
  if (!found_normal) {
    return false;
  }

  const float normal_threshold = std::cos(std::clamp(plane_angle, 0.01f, 3.14159f));
  Map<int64_t, int> edge_counts;
  Map<int64_t, int2> boundary_edges_by_key;
  for (const NurbBodyMeshTriangle &tri : face_triangles) {
    float3 tri_normal(0.0f);
    if (!triangle_geometric_normal(tri, positions, tri_normal)) {
      return false;
    }
    if (dot_v3v3(base_normal, tri_normal) < normal_threshold) {
      return false;
    }

    for (int edge_i = 0; edge_i < 3; edge_i++) {
      const int v1 = tri.verts[edge_i];
      const int v2 = tri.verts[(edge_i + 1) % 3];
      const int64_t key = edge_key(v1, v2);
      if (int *count = edge_counts.lookup_ptr(key)) {
        (*count)++;
      }
      else {
        edge_counts.add(key, 1);
        boundary_edges_by_key.add(key, int2(v1, v2));
      }
    }
  }

  Vector<int2> boundary_edges;
  for (const auto &item : edge_counts.items()) {
    if (item.value == 1) {
      boundary_edges.append(boundary_edges_by_key.lookup(item.key));
    }
  }
  if (boundary_edges.size() < 3) {
    return false;
  }

  Map<int, int2> neighbors;
  neighbors.reserve(boundary_edges.size());
  for (const int2 &edge : boundary_edges) {
    if (!add_ngon_boundary_neighbor(neighbors, edge[0], edge[1]) ||
        !add_ngon_boundary_neighbor(neighbors, edge[1], edge[0]))
    {
      return false;
    }
  }
  if (neighbors.size() != boundary_edges.size()) {
    return false;
  }

  Vector<int> loop;
  loop.reserve(boundary_edges.size());
  const int start = boundary_edges[0][0];
  int previous = -1;
  int current = start;
  for (int guard = 0; guard <= boundary_edges.size(); guard++) {
    loop.append(current);
    const int2 *slots = neighbors.lookup_ptr(current);
    if (slots == nullptr || (*slots)[0] == -1 || (*slots)[1] == -1) {
      return false;
    }
    const int next = ((*slots)[0] != previous) ? (*slots)[0] : (*slots)[1];
    previous = current;
    current = next;
    if (current == start) {
      break;
    }
  }
  if (current != start || loop.size() != boundary_edges.size()) {
    return false;
  }

  float3 polygon_normal(0.0f);
  for (const int i : loop.index_range()) {
    const float3 &a = positions[loop[i]];
    const float3 &b = positions[loop[(i + 1) % loop.size()]];
    float3 edge_cross;
    cross_v3_v3v3(edge_cross, a, b);
    polygon_normal += edge_cross;
  }
  if (normalize_v3(polygon_normal) == 0.0f) {
    return false;
  }
  if (dot_v3v3(polygon_normal, base_normal) < 0.0f) {
    std::reverse(loop.begin(), loop.end());
  }

  r_face.verts.clear();
  r_face.normals.clear();
  for (const int vertex : loop) {
    float3 normal(0.0f);
    int count = 0;
    for (const NurbBodyMeshTriangle &tri : face_triangles) {
      for (int corner = 0; corner < 3; corner++) {
        if (tri.verts[corner] == vertex) {
          normal += tri.normals[corner];
          count++;
        }
      }
    }
    if (count > 0) {
      normal *= 1.0f / float(count);
    }
    if (normalize_v3(normal) == 0.0f) {
      normal = base_normal;
    }
    r_face.verts.append(vertex);
    r_face.normals.append(normal);
  }
  return true;
}

static void build_mesh_faces_from_triangles(const Span<NurbBodyMeshTriangle> triangles,
                                            const Span<float3> positions,
                                            const int topology,
                                            const float plane_angle,
                                            Vector<NurbBodyMeshFace> &r_faces)
{
  if (topology == NURB_BODY_TESSELLATION_TRIS) {
    for (const NurbBodyMeshTriangle &tri : triangles) {
      append_triangle_mesh_face(r_faces, tri);
    }
    return;
  }

  Array<bool> used(triangles.size(), false);
  if (topology == NURB_BODY_TESSELLATION_NGONS) {
    int64_t start = 0;
    while (start < triangles.size()) {
      int64_t end = start + 1;
      while (end < triangles.size() && triangles[end].face_index == triangles[start].face_index) {
        end++;
      }
      NurbBodyMeshFace ngon;
      if (try_make_planar_ngon_from_triangles(triangles.slice(start, end - start),
                                              positions,
                                              plane_angle,
                                              ngon))
      {
        for (int64_t i = start; i < end; i++) {
          used[i] = true;
        }
        r_faces.append(std::move(ngon));
      }
      start = end;
    }
  }

  Map<int64_t, int64_t> triangle_by_edge;
  triangle_by_edge.reserve(triangles.size() * 3);

  for (const int64_t i : triangles.index_range()) {
    if (used[i]) {
      continue;
    }
    const NurbBodyMeshTriangle &tri = triangles[i];
    int64_t best_other_tri = -1;
    float best_score = FLT_MAX;
    NurbBodyMeshFace best_quad;
    for (int edge_i = 0; edge_i < 3; edge_i++) {
      const int v1 = tri.verts[edge_i];
      const int v2 = tri.verts[(edge_i + 1) % 3];
      const int64_t key = edge_key(v1, v2);
      if (const int64_t *other_tri = triangle_by_edge.lookup_ptr(key)) {
        if (used[*other_tri] || triangles[*other_tri].face_index != tri.face_index) {
          continue;
        }
        NurbBodyMeshFace quad;
        float score = 0.0f;
        if (try_make_quad_from_tri_pair(
                triangles[*other_tri], tri, positions, plane_angle, quad, &score) &&
            score < best_score)
        {
          best_other_tri = *other_tri;
          best_score = score;
          best_quad = std::move(quad);
        }
      }
    }

    if (best_other_tri != -1) {
      used[best_other_tri] = true;
      used[i] = true;
      r_faces.append(std::move(best_quad));
      continue;
    }

    for (int edge_i = 0; edge_i < 3; edge_i++) {
      const int v1 = tri.verts[edge_i];
      const int v2 = tri.verts[(edge_i + 1) % 3];
      const int64_t key = edge_key(v1, v2);
      if (int64_t *stored_tri = triangle_by_edge.lookup_ptr(key)) {
        if (used[*stored_tri]) {
          *stored_tri = i;
        }
      }
      else {
        triangle_by_edge.add(key, i);
      }
    }
  }

  for (const int64_t i : triangles.index_range()) {
    if (!used[i]) {
      used[i] = true;
      append_triangle_mesh_face(r_faces, triangles[i]);
    }
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
    for (const int corner : face.verts.index_range()) {
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

static float3 normal_from_face_surface_uv(BRepAdaptor_Surface &surface,
                                          const gp_Pnt2d &uv,
                                          const bool reverse_face)
{
  try {
    gp_Pnt point;
    gp_Vec du;
    gp_Vec dv;
    surface.D1(uv.X(), uv.Y(), point, du, dv);
    gp_Vec normal = du.Crossed(dv);
    const double length_sq = normal.SquareMagnitude();
    if (length_sq <= 1.0e-24) {
      return float3(0.0f);
    }
    if (reverse_face) {
      normal.Reverse();
    }
    normal.Normalize();
    return float3(float(normal.X()), float(normal.Y()), float(normal.Z()));
  }
  catch (Standard_Failure const &) {
    return float3(0.0f);
  }
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

  const int topology = ELEM(body.tessellation_topology,
                            NURB_BODY_TESSELLATION_TRIS,
                            NURB_BODY_TESSELLATION_QUADS,
                            NURB_BODY_TESSELLATION_NGONS) ?
                           body.tessellation_topology :
                           ((body.flag & NURB_BODY_TRIANGULATE_MESH) != 0 ?
                                NURB_BODY_TESSELLATION_TRIS :
                                NURB_BODY_TESSELLATION_QUADS);
  const bool triangulate_mesh = topology == NURB_BODY_TESSELLATION_TRIS;
  const bool use_smooth_shading = (body.flag & NURB_BODY_SMOOTH_SHADING) != 0;
  Vector<float3> positions;
  Vector<NurbBodyMeshTriangle> triangles;
  positions.reserve(1024);
  triangles.reserve(2048);

  Map<uint64_t, Vector<int>> vertex_buckets;
  vertex_buckets.reserve(1024);
  const float weld_tolerance = std::clamp(body.tessellation_deflection * 0.001f,
                                          1.0e-7f,
                                          1.0e-5f);
  const float weld_tolerance_sq = weld_tolerance * weld_tolerance;
  const double inverse_weld_tolerance = 1.0 / double(weld_tolerance);
  auto position_key = [&](const float3 &position) {
    uint64_t hash = 1469598103934665603ull;
    auto hash_component = [&](const int64_t value) {
      hash ^= uint64_t(value);
      hash *= 1099511628211ull;
    };
    hash_component(int64_t(std::llround(double(position.x) * inverse_weld_tolerance)));
    hash_component(int64_t(std::llround(double(position.y) * inverse_weld_tolerance)));
    hash_component(int64_t(std::llround(double(position.z) * inverse_weld_tolerance)));
    return hash;
  };
  auto position_distance_sq = [](const float3 &a, const float3 &b) {
    const float3 delta = a - b;
    return dot_v3v3(delta, delta);
  };
  auto add_position = [&](const float3 &position) -> int {
    const uint64_t key = position_key(position);
    if (Vector<int> *bucket = vertex_buckets.lookup_ptr(key)) {
      for (const int candidate : *bucket) {
        if (position_distance_sq(positions[candidate], position) <= weld_tolerance_sq) {
          return candidate;
        }
      }
      const int index = int(positions.size());
      positions.append(position);
      bucket->append(index);
      return index;
    }
    const int index = int(positions.size());
    positions.append(position);
    Vector<int> bucket;
    bucket.append(index);
    vertex_buckets.add(key, std::move(bucket));
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
    BRepAdaptor_Surface surface;
    bool planar_face = false;
    bool use_surface_normals = false;
    try {
      BRepAdaptor_Surface face_surface(face, true);
      planar_face = face_surface.GetType() == GeomAbs_Plane;
    }
    catch (Standard_Failure const &) {
      planar_face = false;
    }
    if (use_smooth_shading && triangulation->HasUVNodes()) {
      try {
        surface.Initialize(face, true);
        use_surface_normals = true;
      }
      catch (Standard_Failure const &) {
        use_surface_normals = false;
      }
    }
    if (use_smooth_shading && !use_surface_normals && !triangulation->HasNormals()) {
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
      float3 node_normal(0.0f);
      if (use_smooth_shading) {
        if (use_surface_normals) {
          node_normal = normal_from_face_surface_uv(surface, triangulation->UVNode(i), reverse_face);
        }
        if (is_zero_v3(node_normal)) {
          node_normal = normal_from_triangulation_node(triangulation, i, transform, reverse_face);
        }
      }
      node_normals.append(node_normal);
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
      mesh_triangle.planar_face = planar_face;
      triangles.append(mesh_triangle);
    }
  }
  nurb_body_debug_bevel_log("to_mesh_triangulation_stats",
                            "nodes=%d triangles=%d modal_preview=%d",
                            int(positions.size()),
                            int(triangles.size()),
                            int(nurb_body_modal_bevel_preview_active(body)));

  Vector<NurbBodyMeshFace> faces;
  build_mesh_faces_from_triangles(triangles,
                                  positions.as_span(),
                                  triangulate_mesh ? NURB_BODY_TESSELLATION_TRIS : topology,
                                  body.tessellation_plane_angle,
                                  faces);

  Array<int> compact_vertex_index(positions.size(), -1);
  Vector<float3> final_positions;
  final_positions.reserve(positions.size());
  for (NurbBodyMeshFace &face : faces) {
    for (const int corner : face.verts.index_range()) {
      const int vertex = face.verts[corner];
      int &compact_index = compact_vertex_index[vertex];
      if (compact_index == -1) {
        compact_index = int(final_positions.size());
        final_positions.append(positions[vertex]);
      }
      face.verts[corner] = compact_index;
    }
  }

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
    for (const int corner : face.verts.index_range()) {
      const int v1 = face.verts[corner];
      const int v2 = face.verts[(corner + 1) % face.verts.size()];
      corner_verts.append(v1);
      corner_edges.append(edge_index(v1, v2));
    }
    face_offsets_data.append(corner_verts.size());
  }

  Mesh *mesh = BKE_mesh_new_nomain(
      final_positions.size(), edges.size(), faces.size(), corner_verts.size());
  mesh->vert_positions_for_write().copy_from(final_positions.as_span());
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
  const bool modal_preview = nurb_body_modal_bevel_preview_active(body);
  auto hash_surface_edge_keys = [&](const uint64_t *edge_keys, const uint64_t edge_mask) {
    nurb_body_hash_value(hash, edge_mask);
    for (int i = 0; i < 64; i++) {
      if ((edge_mask & nurb_body_edge_mask_for_index(i)) != 0) {
        nurb_body_hash_value(hash, edge_keys[i]);
      }
    }
  };
  if (samples_per_edge >= 0) {
    nurb_body_hash_value(hash, samples_per_edge);
  }
  nurb_body_hash_value(hash, body.primitive);
  nurb_body_hash_value(hash, body.radius);
  nurb_body_hash_value(hash, body.depth);
  nurb_body_hash_value(hash, body.minor_radius);
  nurb_body_hash_bytes(hash, body.dimensions, sizeof(body.dimensions));
  nurb_body_hash_value(hash, modal_preview ? NURB_BODY_FAST_BEVEL_PREVIEW : 0);
  if (include_viewport_mesh_settings) {
    nurb_body_hash_value(hash, body.flag & NURB_BODY_SMOOTH_SHADING);
    nurb_body_hash_value(hash, body.tessellation_deflection);
    nurb_body_hash_value(hash, body.tessellation_angle);
    nurb_body_hash_value(hash, body.tessellation_face_deflection);
    nurb_body_hash_value(hash, body.tessellation_face_angle);
    nurb_body_hash_value(hash, body.tessellation_density);
    nurb_body_hash_value(hash, body.tessellation_min_width);
    nurb_body_hash_value(hash, body.tessellation_plane_angle);
    nurb_body_hash_value(hash, body.tessellation_topology);
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
  hash_surface_edge_keys(body.surface_edge_keys,
                         body.surface_bevel_edges |
                             (modal_preview ? body.surface_selected_edges : 0));
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
    const uint64_t op_surface_key_mask =
        nurb_body_boolean_op_is_surface_blend_stage(*op) ?
            op->bevel_edges :
            (op->operand_surface_bevel_edges |
             (modal_preview ? op->operand_surface_selected_edges : 0));
    hash_surface_edge_keys(op->operand_surface_edge_keys, op_surface_key_mask);
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
  const NurbBody *body = nullptr;
  uint64_t key = 0;
  Vector<NurbBodyEdgePolyline> polylines;
};

struct NurbBodyEvaluatedShapeCache {
  const Object *object = nullptr;
  const NurbBody *body = nullptr;
  uint64_t key = 0;
  TopoDS_Shape shape;
};

struct NurbBodyStageShapeCache {
  const Object *object = nullptr;
  const NurbBody *body = nullptr;
  uint64_t key = 0;
  TopoDS_Shape shape;
  Vector<TopoDS_Edge> edges;
  Vector<NurbBodySurfaceEdgeEntry> surface_edges;
};

struct NurbBodyHoveredEdgeKey {
  const Object *object = nullptr;
  const NurbBody *body = nullptr;
  const NurbBodyBooleanOp *op = nullptr;
  int edge_index = -1;
  int flag = 0;
  uint64_t edge_key = 0;
};

static Vector<NurbBodyEvaluatedShapeCache> g_nurb_body_evaluated_shape_caches;
static Vector<NurbBodyStageShapeCache> g_nurb_body_stage_shape_caches;
static Vector<NurbBodyEdgePolylineCache> g_nurb_body_edge_polyline_caches;
static Vector<NurbBodyHoveredEdgeKey> g_nurb_body_hovered_edge_keys;
static Vector<NurbBodyHoveredEdgeKey> g_nurb_body_selected_edge_keys;

static const Object *nurb_body_cache_object_for_object(const Object *object)
{
  if (object == nullptr) {
    return nullptr;
  }
  if (object->id.orig_id != nullptr) {
    return id_cast<const Object *>(object->id.orig_id);
  }
  return object;
}

static const NurbBody *nurb_body_cache_body_for_object(const Object *object)
{
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  if (cache_object == nullptr || cache_object->type != OB_NURB_BODY ||
      cache_object->data == nullptr || GS(cache_object->data->name) != ID_NB)
  {
    return nullptr;
  }
  return id_cast<const NurbBody *>(cache_object->data);
}

static void nurb_body_global_caches_remove_body(const NurbBody *body)
{
  if (body == nullptr) {
    return;
  }

  for (int i = int(g_nurb_body_evaluated_shape_caches.size()) - 1; i >= 0; i--) {
    if (g_nurb_body_evaluated_shape_caches[i].body == body) {
      g_nurb_body_evaluated_shape_caches.remove_and_reorder(i);
    }
  }
  for (int i = int(g_nurb_body_stage_shape_caches.size()) - 1; i >= 0; i--) {
    if (g_nurb_body_stage_shape_caches[i].body == body) {
      g_nurb_body_stage_shape_caches.remove_and_reorder(i);
    }
  }
  for (int i = int(g_nurb_body_edge_polyline_caches.size()) - 1; i >= 0; i--) {
    if (g_nurb_body_edge_polyline_caches[i].body == body) {
      g_nurb_body_edge_polyline_caches.remove_and_reorder(i);
    }
  }
  for (int i = int(g_nurb_body_hovered_edge_keys.size()) - 1; i >= 0; i--) {
    if (g_nurb_body_hovered_edge_keys[i].body == body) {
      g_nurb_body_hovered_edge_keys.remove_and_reorder(i);
    }
  }
  for (int i = int(g_nurb_body_selected_edge_keys.size()) - 1; i >= 0; i--) {
    if (g_nurb_body_selected_edge_keys[i].body == body) {
      g_nurb_body_selected_edge_keys.remove_and_reorder(i);
    }
  }
}

static uint64_t nurb_body_evaluated_shape_cache_key(const NurbBody &body, const Object &object)
{
  return nurb_body_shape_cache_key(body, object);
}

static NurbBodyEvaluatedShapeCache *nurb_body_evaluated_shape_cache_find(const Object *object)
{
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr) {
    return nullptr;
  }
  for (NurbBodyEvaluatedShapeCache &cache : g_nurb_body_evaluated_shape_caches) {
    if (cache.object == cache_object && cache.body == body) {
      return &cache;
    }
  }
  return nullptr;
}

static NurbBodyEvaluatedShapeCache *nurb_body_evaluated_shape_cache_ensure(const Object *object)
{
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr) {
    return nullptr;
  }
  if (NurbBodyEvaluatedShapeCache *cache = nurb_body_evaluated_shape_cache_find(object)) {
    return cache;
  }

  constexpr int max_cached_objects = 64;
  if (g_nurb_body_evaluated_shape_caches.size() >= max_cached_objects) {
    g_nurb_body_evaluated_shape_caches.remove_and_reorder(0);
  }

  NurbBodyEvaluatedShapeCache cache;
  cache.object = cache_object;
  cache.body = body;
  g_nurb_body_evaluated_shape_caches.append(std::move(cache));
  return &g_nurb_body_evaluated_shape_caches.last();
}

static TopoDS_Shape nurb_body_evaluate_shape_cached(const NurbBody &body, const Object *object)
{
  if (object == nullptr) {
    nurb_body_debug_bevel_log("eval_shape_cache_bypass", "object=null");
    return evaluate_shape(body, object);
  }

  const uint64_t key = nurb_body_evaluated_shape_cache_key(body, *object);
  NurbBodyEvaluatedShapeCache *cache = nurb_body_evaluated_shape_cache_ensure(object);
  if (cache == nullptr) {
    nurb_body_debug_bevel_log("eval_shape_cache_bypass", "cache=null key=%llu",
                              static_cast<unsigned long long>(key));
    return evaluate_shape(body, object);
  }
  if (cache->key == key && !cache->shape.IsNull()) {
    nurb_body_debug_bevel_log("eval_shape_cache_hit",
                              "key=%llu",
                              static_cast<unsigned long long>(key));
    return cache->shape;
  }

  nurb_body_debug_bevel_log("eval_shape_cache_miss",
                            "old_key=%llu new_key=%llu old_shape_null=%d",
                            static_cast<unsigned long long>(cache->key),
                            static_cast<unsigned long long>(key),
                            int(cache->shape.IsNull()));
  TopoDS_Shape evaluated_shape = evaluate_shape(body, object);
  if (!evaluated_shape.IsNull() && shape_passes_fast_integrity_check(evaluated_shape, true)) {
    cache->shape = evaluated_shape;
    cache->key = key;
  }
  else {
    const NurbBodyShapeBoundaryStats stats = shape_boundary_stats(evaluated_shape);
    nurb_body_debug_bevel_log("eval_shape_cache_store_rejected",
                              "key=%llu shape_null=%d edges=%d open=%d nonmanifold=%d",
                              static_cast<unsigned long long>(key),
                              int(evaluated_shape.IsNull()),
                              stats.edges,
                              stats.open_edges,
                              stats.nonmanifold_edges);
    cache->shape = TopoDS_Shape();
    cache->key = 0;
  }
  return evaluated_shape;
}

static void nurb_body_evaluated_shape_cache_store(const NurbBody &body,
                                                  const Object *object,
                                                  const TopoDS_Shape &shape)
{
  if (object == nullptr || shape.IsNull()) {
    return;
  }
  if (!shape_passes_fast_integrity_check(shape, true)) {
    const NurbBodyShapeBoundaryStats stats = shape_boundary_stats(shape);
    nurb_body_debug_bevel_log("eval_shape_cache_prefill_rejected",
                              "edges=%d open=%d nonmanifold=%d",
                              stats.edges,
                              stats.open_edges,
                              stats.nonmanifold_edges);
    return;
  }

  NurbBodyEvaluatedShapeCache *cache = nurb_body_evaluated_shape_cache_ensure(object);
  if (cache == nullptr) {
    return;
  }
  cache->shape = shape;
  cache->key = nurb_body_evaluated_shape_cache_key(body, *object);
}

static NurbBodyStageShapeCache *nurb_body_stage_shape_cache_find_entry(const Object *object,
                                                                       const uint64_t key)
{
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr || key == 0) {
    return nullptr;
  }
  for (NurbBodyStageShapeCache &cache : g_nurb_body_stage_shape_caches) {
    if (cache.object == cache_object && cache.body == body && cache.key == key) {
      return &cache;
    }
  }
  return nullptr;
}

static bool nurb_body_shape_cache_can_store(const TopoDS_Shape &shape,
                                            const uint64_t key,
                                            const char *kind)
{
  if (shape.IsNull()) {
    return false;
  }
  if (shape_passes_fast_integrity_check(shape, true)) {
    return true;
  }

  const NurbBodyShapeBoundaryStats stats = shape_boundary_stats(shape);
  nurb_body_debug_bevel_log("shape_cache_store_rejected",
                            "kind=%s key=%llu edges=%d manifold=%d seam=%d open=%d "
                            "nonmanifold=%d",
                            kind,
                            static_cast<unsigned long long>(key),
                            stats.edges,
                            stats.manifold_edges,
                            stats.seam_edges,
                            stats.open_edges,
                            stats.nonmanifold_edges);
  return false;
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
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr || key == 0 || shape.IsNull()) {
    return;
  }
  if (!nurb_body_shape_cache_can_store(shape, key, "stage")) {
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
  cache.object = cache_object;
  cache.body = body;
  cache.key = key;
  cache.shape = shape;
  g_nurb_body_stage_shape_caches.append(std::move(cache));
}

static void nurb_body_stage_shape_cache_store_edges(const Object *object,
                                                    const uint64_t key,
                                                    const TopoDS_Shape &shape,
                                                    const Span<TopoDS_Edge> edges)
{
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr || key == 0 || shape.IsNull()) {
    return;
  }
  if (!nurb_body_shape_cache_can_store(shape, key, "stage_edges")) {
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
  cache.object = cache_object;
  cache.body = body;
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
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr || key == 0 || shape.IsNull()) {
    return;
  }
  if (!nurb_body_shape_cache_can_store(shape, key, "stage_surface_edges")) {
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
  cache.object = cache_object;
  cache.body = body;
  cache.key = key;
  cache.shape = shape;
  cache.surface_edges.extend(edges.data(), edges.size());
  g_nurb_body_stage_shape_caches.append(std::move(cache));
}

static NurbBodyEdgePolylineCache *nurb_body_edge_polyline_cache_find(const Object *object)
{
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr) {
    return nullptr;
  }
  for (NurbBodyEdgePolylineCache &cache : g_nurb_body_edge_polyline_caches) {
    if (cache.object == cache_object && cache.body == body) {
      return &cache;
    }
  }
  return nullptr;
}

static NurbBodyEdgePolylineCache *nurb_body_edge_polyline_cache_ensure(const Object *object)
{
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr) {
    return nullptr;
  }
  if (NurbBodyEdgePolylineCache *cache = nurb_body_edge_polyline_cache_find(object)) {
    return cache;
  }

  constexpr int max_cached_objects = 64;
  if (g_nurb_body_edge_polyline_caches.size() >= max_cached_objects) {
    g_nurb_body_edge_polyline_caches.remove_and_reorder(0);
  }

  NurbBodyEdgePolylineCache cache;
  cache.object = cache_object;
  cache.body = body;
  g_nurb_body_edge_polyline_caches.append(std::move(cache));
  return &g_nurb_body_edge_polyline_caches.last();
}

static bool nurb_body_edge_polyline_cache_fill_from_evaluated_shape(
    const NurbBody &body,
    const Object *object,
    const int samples_per_edge,
    const TopoDS_Shape &shape,
    NurbBodyEdgePolylineCache &cache,
    const uint64_t cache_key,
    const char *log_stage)
{
  if (!nurb_body_modal_bevel_preview_active(body) || shape.IsNull()) {
    return false;
  }

  const double polyline_start = nurb_body_debug_now();
  try {
    cache.key = 0;
    cache.polylines.clear();
    sample_evaluated_shape_edge_polylines(body, shape, samples_per_edge, cache.polylines);
    cache.key = cache_key;
    nurb_body_debug_bevel_log(log_stage,
                              "object=%p samples=%d key=%llu points=%d ms=%.3f",
                              static_cast<const void *>(object),
                              samples_per_edge,
                              static_cast<unsigned long long>(cache_key),
                              int(cache.polylines.size()),
                              (nurb_body_debug_now() - polyline_start) * 1000.0);
    return true;
  }
  catch (Standard_Failure const &) {
    cache.key = 0;
    cache.polylines.clear();
    nurb_body_debug_bevel_log(log_stage,
                              "object=%p samples=%d key=%llu exception=1 ms=%.3f",
                              static_cast<const void *>(object),
                              samples_per_edge,
                              static_cast<unsigned long long>(cache_key),
                              (nurb_body_debug_now() - polyline_start) * 1000.0);
    return false;
  }
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
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr) {
    return nullptr;
  }
  for (NurbBodyHoveredEdgeKey &hover_key : g_nurb_body_hovered_edge_keys) {
    if (hover_key.object == cache_object && hover_key.body == body) {
      return &hover_key;
    }
  }
  return nullptr;
}

static NurbBodyHoveredEdgeKey *nurb_body_selected_edge_key_find(const Object *object)
{
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr) {
    return nullptr;
  }
  for (NurbBodyHoveredEdgeKey &selected_key : g_nurb_body_selected_edge_keys) {
    if (selected_key.object == cache_object && selected_key.body == body) {
      return &selected_key;
    }
  }
  return nullptr;
}

#endif

Mesh *BKE_nurb_body_to_mesh(const NurbBody *body, const Object *object)
{
#ifdef WITH_OPENCASCADE
  try {
    const double total_start = nurb_body_debug_now();
    nurb_body_debug_bevel_log("to_mesh_begin",
                              "object=%p body=%p",
                              static_cast<const void *>(object),
                              static_cast<const void *>(body));
    const double evaluate_start = nurb_body_debug_now();
    TopoDS_Shape shape = nurb_body_evaluate_shape_cached(*body, object);
    nurb_body_debug_bevel_log("to_mesh_evaluate_shape",
                              "ms=%.3f shape_null=%d",
                              (nurb_body_debug_now() - evaluate_start) * 1000.0,
                              int(shape.IsNull()));
    const double mesh_start = nurb_body_debug_now();
    Mesh *mesh = mesh_from_shape(shape, *body);
    nurb_body_debug_bevel_log("to_mesh_mesh_from_shape",
                              "verts=%d edges=%d faces=%d ms=%.3f",
                              mesh->verts_num,
                              mesh->edges_num,
                              mesh->faces_num,
                              (nurb_body_debug_now() - mesh_start) * 1000.0);
    nurb_body_evaluated_shape_cache_store(*body, object, shape);
    if (object != nullptr) {
      const double polyline_start = nurb_body_debug_now();
      const int overlay_samples = nurb_body_effective_edge_polyline_samples(*body, 64);
      const uint64_t polyline_key = nurb_body_edge_polyline_cache_key(
          *body, *object, overlay_samples);
      NurbBodyEdgePolylineCache *polyline_cache = nurb_body_edge_polyline_cache_ensure(object);
      if (polyline_cache != nullptr && polyline_cache->key != polyline_key) {
        if (!nurb_body_edge_polyline_cache_fill_from_evaluated_shape(
                *body,
                object,
                overlay_samples,
                shape,
                *polyline_cache,
                polyline_key,
                "to_mesh_polyline_from_evaluated_shape"))
        {
          polyline_cache->key = 0;
          polyline_cache->polylines.clear();
          nurb_body_debug_bevel_log("to_mesh_polyline_prefill_skipped",
                                    "samples=%d key=%llu fast_preview=%d shape_null=%d ms=%.3f",
                                    overlay_samples,
                                    static_cast<unsigned long long>(polyline_key),
                                    int(nurb_body_modal_bevel_preview_active(*body)),
                                    int(shape.IsNull()),
                                    (nurb_body_debug_now() - polyline_start) * 1000.0);
        }
      }
      else {
        nurb_body_debug_bevel_log("to_mesh_polyline_cache_hit",
                                  "samples=%d key=%llu ms=%.3f",
                                  overlay_samples,
                                  static_cast<unsigned long long>(polyline_key),
                                  (nurb_body_debug_now() - polyline_start) * 1000.0);
      }
    }
    nurb_body_debug_bevel_log("to_mesh_done",
                              "total_ms=%.3f",
                              (nurb_body_debug_now() - total_start) * 1000.0);
    BKE_nurb_body_debug_bevel_end_drag_tick("to_mesh_done");
    return mesh;
  }
  catch (Standard_Failure const &) {
    nurb_body_debug_bevel_log("to_mesh_exception", "returning_empty_mesh=1");
    BKE_nurb_body_debug_bevel_end_drag_tick("to_mesh_exception");
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
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (body == nullptr) {
    return {};
  }

  const int effective_samples = nurb_body_effective_edge_polyline_samples(*body,
                                                                          samples_per_edge);
  const uint64_t cache_key = nurb_body_edge_polyline_cache_key(*body, *object, effective_samples);
  NurbBodyEdgePolylineCache *cache = nurb_body_edge_polyline_cache_ensure(object);
  if (cache == nullptr) {
    return {};
  }
  if (cache->key == cache_key) {
    return cache->polylines.as_span();
  }

  const bool modal_preview = nurb_body_modal_bevel_preview_active(*body);
  if (modal_preview) {
    NurbBodyEvaluatedShapeCache *shape_cache = nurb_body_evaluated_shape_cache_find(object);
    const uint64_t shape_key = nurb_body_evaluated_shape_cache_key(*body, *object);
    if (shape_cache != nullptr && shape_cache->key == shape_key && !shape_cache->shape.IsNull()) {
      if (nurb_body_edge_polyline_cache_fill_from_evaluated_shape(
              *body,
              object,
              effective_samples,
              shape_cache->shape,
              *cache,
              cache_key,
              "polyline_cache_from_evaluated_shape"))
      {
        return cache->polylines.as_span();
      }
    }
    nurb_body_debug_bevel_log("polyline_cache_evaluated_shape_miss",
                              "object=%p samples=%d cache_key=%llu shape_cache=%d stale=%d "
                              "body_fast=%d debug_active=%d",
                              static_cast<const void *>(object),
                              effective_samples,
                              static_cast<unsigned long long>(cache_key),
                              int(shape_cache != nullptr),
                              int(cache->polylines.size()),
                              int((body->flag & NURB_BODY_FAST_BEVEL_PREVIEW) != 0),
                              int(nurb_body_debug_bevel_active()));
    return cache->polylines.as_span();
  }

  try {
    cache->polylines.clear();
    sample_boolean_edge_polylines(*body, object, effective_samples, cache->polylines);
    cache->key = cache_key;
    return cache->polylines.as_span();
  }
  catch (Standard_Failure const &) {
    cache->key = 0;
    cache->polylines.clear();
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
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (body == nullptr) {
    return 0;
  }

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
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr || edge_index < 0 || edge_key == 0) {
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
  hover_key.object = cache_object;
  hover_key.body = body;
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
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  bool changed = false;
  for (int i = int(g_nurb_body_hovered_edge_keys.size()) - 1; i >= 0; i--) {
    if (g_nurb_body_hovered_edge_keys[i].object == cache_object) {
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

bool BKE_nurb_body_selected_edge_key_set(const Object *object,
                                         const NurbBodyBooleanOp *op,
                                         const int edge_index,
                                         const int flag,
                                         const uint64_t edge_key)
{
#ifdef WITH_OPENCASCADE
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr || edge_index < 0 || edge_key == 0) {
    return BKE_nurb_body_selected_edge_key_clear(object);
  }

  const int domain = hovered_edge_domain_from_flag(flag);
  for (NurbBodyHoveredEdgeKey &selected_key : g_nurb_body_selected_edge_keys) {
    if (selected_key.object != cache_object || selected_key.body != body ||
        selected_key.edge_key != edge_key)
    {
      continue;
    }
    const bool changed = selected_key.op != op || selected_key.edge_index != edge_index ||
                         selected_key.flag != domain;
    selected_key.op = op;
    selected_key.edge_index = edge_index;
    selected_key.flag = domain;
    selected_key.edge_key = edge_key;
    return changed;
  }

  NurbBodyHoveredEdgeKey selected_key;
  selected_key.object = cache_object;
  selected_key.body = body;
  selected_key.op = op;
  selected_key.edge_index = edge_index;
  selected_key.flag = domain;
  selected_key.edge_key = edge_key;
  g_nurb_body_selected_edge_keys.append(selected_key);
  return true;
#else
  UNUSED_VARS(object, op, edge_index, flag, edge_key);
  return false;
#endif
}

bool BKE_nurb_body_selected_edge_key_clear(const Object *object)
{
#ifdef WITH_OPENCASCADE
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  bool changed = false;
  for (int i = int(g_nurb_body_selected_edge_keys.size()) - 1; i >= 0; i--) {
    if (g_nurb_body_selected_edge_keys[i].object == cache_object) {
      g_nurb_body_selected_edge_keys.remove_and_reorder(i);
      changed = true;
    }
  }
  return changed;
#else
  UNUSED_VARS(object);
  return false;
#endif
}

uint64_t BKE_nurb_body_selected_edge_key_get(const Object *object)
{
#ifdef WITH_OPENCASCADE
  if (const NurbBodyHoveredEdgeKey *selected_key = nurb_body_selected_edge_key_find(object)) {
    return selected_key->edge_key;
  }
  return 0;
#else
  UNUSED_VARS(object);
  return 0;
#endif
}

bool BKE_nurb_body_selected_edge_key_matches(const Object *object,
                                             const NurbBodyEdgePolyline &polyline)
{
#ifdef WITH_OPENCASCADE
  const Object *cache_object = nurb_body_cache_object_for_object(object);
  const NurbBody *body = nurb_body_cache_body_for_object(object);
  if (cache_object == nullptr || body == nullptr || polyline.edge_key == 0) {
    return false;
  }

  for (const NurbBodyHoveredEdgeKey &selected_key : g_nurb_body_selected_edge_keys) {
    if (selected_key.object == cache_object && selected_key.body == body &&
        selected_key.edge_key == polyline.edge_key)
    {
      return true;
    }
  }
  return false;
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
                               0.004f;
  const float angle = tool_settings.nurb_body_tessellation_angle > 0.0f ?
                          tool_settings.nurb_body_tessellation_angle :
                          0.10472f;
  const float density = missing_legacy_tool_values ? 0.75f :
                                                     tool_settings.nurb_body_tessellation_density;
  const float plane_angle = tool_settings.nurb_body_tessellation_plane_angle > 0.0f ?
                                tool_settings.nurb_body_tessellation_plane_angle :
                                0.523599f;
  const float face_deflection =
      std::max(tool_settings.nurb_body_tessellation_face_deflection > 0.0f ?
                   tool_settings.nurb_body_tessellation_face_deflection :
                   deflection * 3.0f,
               deflection);
  const float face_angle = std::clamp(tool_settings.nurb_body_tessellation_face_angle > 0.0f ?
                                          tool_settings.nurb_body_tessellation_face_angle :
                                          angle * 2.0f,
                                      angle,
                                      3.14159f);
  int viewport_flags = tool_settings.nurb_body_viewport_flag & NURB_BODY_SMOOTH_SHADING;
  if (missing_legacy_tool_values) {
    viewport_flags = NURB_BODY_SMOOTH_SHADING;
  }

  body.tessellation_deflection = std::max(deflection, 0.0001f);
  body.tessellation_angle = std::clamp(angle, 0.01f, 3.14159f);
  body.tessellation_face_deflection = std::max(face_deflection, body.tessellation_deflection);
  body.tessellation_face_angle = face_angle;
  body.tessellation_density = std::clamp(density, 0.0f, 1.0f);
  body.tessellation_min_width = std::max(tool_settings.nurb_body_tessellation_min_width, 0.0f);
  body.tessellation_plane_angle = std::clamp(plane_angle, 0.01f, 3.14159f);
  body.tessellation_topology = (tool_settings.nurb_body_viewport_flag &
                                NURB_BODY_TRIANGULATE_MESH) != 0 ?
                                   NURB_BODY_TESSELLATION_TRIS :
                                   (ELEM(tool_settings.nurb_body_tessellation_topology,
                                         NURB_BODY_TESSELLATION_TRIS,
                                         NURB_BODY_TESSELLATION_QUADS,
                                         NURB_BODY_TESSELLATION_NGONS) ?
                                        tool_settings.nurb_body_tessellation_topology :
                                        NURB_BODY_TESSELLATION_NGONS);
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
