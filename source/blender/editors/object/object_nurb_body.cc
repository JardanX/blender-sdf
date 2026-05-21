/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edobj
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "MEM_guardedalloc.h"

#include "DNA_nurb_body_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"

#include "BLI_listbase.h"
#include "BLI_math_geom.h"
#include "BLI_math_base.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_nurb_body.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "DEG_depsgraph_build.hh"
#include "DEG_depsgraph.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_object.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_util.hh"
#include "ED_view3d.hh"

#include "object_intern.hh"

namespace blender::ed::object {

static Object *active_nurb_body_object(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  if (ob && ob->type == OB_NURB_BODY && ob->data) {
    return ob;
  }
  return nullptr;
}

static bool object_nurb_body_poll(bContext *C)
{
  return active_nurb_body_object(C) != nullptr;
}

static const EnumPropertyItem nurb_body_boolean_operation_items[] = {
    {NURB_BODY_BOOLEAN_DIFFERENCE, "DIFFERENCE", 0, "Difference", "Subtract selected bodies"},
    {NURB_BODY_BOOLEAN_UNION, "UNION", 0, "Union", "Join selected bodies"},
    {NURB_BODY_BOOLEAN_INTERSECT, "INTERSECT", 0, "Intersect", "Keep the common volume"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem nurb_body_bevel_type_items[] = {
    {NURB_BODY_BEVEL_FILLET, "FILLET", 0, "Fillet", "Round the selected edge"},
    {NURB_BODY_BEVEL_CHAMFER, "CHAMFER", 0, "Chamfer", "Cut a flat bevel on the selected edge"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem nurb_body_select_mode_items[] = {
    {NURB_BODY_SELECT_MODE_EDGE, "EDGE", 0, "Edge", "Select generated NURB Body edges"},
    {NURB_BODY_SELECT_MODE_FACE, "FACE", 0, "Face", "Reserve NURB Body face selection"},
    {NURB_BODY_SELECT_MODE_OBJECT, "OBJECT", 0, "Object", "Use normal object selection"},
    {0, nullptr, 0, nullptr, nullptr},
};

static uint64_t nurb_body_edge_mask_for_index(int edge_index);
static int nurb_body_first_selected_edge(uint64_t selected_edges);

static void nurb_body_boolean_ops_clear_flag(NurbBody &body, const int flag)
{
  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first); op;
       op = op->next)
  {
    op->flag &= ~flag;
    if (flag & NURB_BODY_BOOLEAN_OP_SELECTED) {
      op->selected_edges = 0;
      op->selected_edge = -1;
    }
    if (flag & NURB_BODY_BOOLEAN_OP_HOVERED) {
      op->hovered_edge = -1;
    }
  }
}

static void nurb_body_preserve_existing_bevel_edges(NurbBody &body)
{
  if (body.bevel_radius > 0.0f && body.bevel_edges == 0 && body.selected_edges != 0) {
    body.bevel_edges = body.selected_edges;
    body.bevel_edge = nurb_body_first_selected_edge(body.bevel_edges);
    if (body.bevel_type == NURB_BODY_BEVEL_CHAMFER) {
      body.chamfer_edges |= body.bevel_edges;
    }
    else {
      body.chamfer_edges &= ~body.bevel_edges;
    }
  }

  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first); op;
       op = op->next)
  {
    if (op->bevel_radius > 0.0f && op->bevel_edges == 0 && op->selected_edges != 0) {
      op->bevel_edges = op->selected_edges;
      op->bevel_edge = nurb_body_first_selected_edge(op->bevel_edges);
      if (op->bevel_type == NURB_BODY_BEVEL_CHAMFER) {
        op->chamfer_edges |= op->bevel_edges;
      }
      else {
        op->chamfer_edges &= ~op->bevel_edges;
      }
    }
  }
}

static void nurb_body_boolean_op_snapshot_from_object(NurbBodyBooleanOp &op,
                                                      const Object &target_ob,
                                                      const Object &operand_ob)
{
  const NurbBody *operand_body = id_cast<const NurbBody *>(operand_ob.data);
  op.primitive = operand_body->primitive;
  op.operand_radius = operand_body->radius;
  op.operand_depth = operand_body->depth;

  float target_inv[4][4];
  invert_m4_m4(target_inv, target_ob.object_to_world().ptr());
  mul_m4_m4m4(op.operand_to_target, target_inv, operand_ob.object_to_world().ptr());
}

static Object *object_nurb_body_add(bContext *C, wmOperator *op)
{
  ushort local_view_bits;
  float loc[3], rot[3];

  add_generic_get_opts(C, op, 'Z', loc, rot, nullptr, nullptr, &local_view_bits, nullptr);

  Object *ob = add_type(C, OB_NURB_BODY, "NURB Cylinder", loc, rot, false, local_view_bits);
  if (!ob || !ob->data) {
    return nullptr;
  }

  NurbBody *body = id_cast<NurbBody *>(ob->data);
  body->primitive = NURB_BODY_PRIMITIVE_CYLINDER;
  body->radius = RNA_float_get(op->ptr, "radius");
  body->depth = RNA_float_get(op->ptr, "depth");
  body->flag = NURB_BODY_MERGE_VERTICES | NURB_BODY_SMOOTH_SHADING |
               NURB_BODY_TRIANGULATE_MESH | NURB_BODY_AUTO_CREASE_SHARP_EDGES;
  body->boolean_operation = NURB_BODY_BOOLEAN_DIFFERENCE;
  body->select_mode = NURB_BODY_SELECT_MODE_EDGE;
  body->selected_edges = 0;
  body->selected_edge = -1;
  body->hovered_edge = -1;
  body->bevel_edges = 0;
  body->chamfer_edges = 0;
  body->bevel_edge = -1;
  body->bevel_type = NURB_BODY_BEVEL_FILLET;
  body->bevel_radius = 0.0f;

  DEG_id_tag_update(&body->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);

  return ob;
}

static wmOperatorStatus object_nurb_body_add_exec(bContext *C, wmOperator *op)
{
  return object_nurb_body_add(C, op) ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
}

void OBJECT_OT_nurb_body_add(wmOperatorType *ot)
{
  ot->name = "Add NURB Body";
  ot->description = "Add an OCCT-backed NURB cylinder body";
  ot->idname = "OBJECT_OT_nurb_body_add";
  ot->exec = object_nurb_body_add_exec;
  ot->poll = ED_operator_objectmode;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  add_generic_props(ot, false);

  RNA_def_float(ot->srna, "radius", 1.0f, 0.001f, 100000.0f, "Radius", "", 0.001f, 100.0f);
  RNA_def_float(ot->srna, "depth", 4.0f, 0.001f, 100000.0f, "Depth", "", 0.001f, 100.0f);
}

struct NurbBodyEdgeHit {
  NurbBodyBooleanOp *op = nullptr;
  int edge_index = -1;
  bool body_edge = false;

  bool is_valid() const
  {
    return (op != nullptr || body_edge) && edge_index >= 0;
  }
};

static uint64_t nurb_body_edge_mask_for_index(const int edge_index)
{
  return (edge_index >= 0 && edge_index < 64) ? (uint64_t(1) << uint(edge_index)) : uint64_t(0);
}

static int nurb_body_first_selected_edge(const uint64_t selected_edges)
{
  for (int i = 0; i < 64; i++) {
    if (selected_edges & nurb_body_edge_mask_for_index(i)) {
      return i;
    }
  }
  return -1;
}

static uint64_t nurb_body_effective_bevel_edges(const float bevel_radius,
                                                const uint64_t bevel_edges,
                                                const int bevel_edge,
                                                const float bevel_radii[64])
{
  if (bevel_edges != 0) {
    if (bevel_radius > 0.0f) {
      return bevel_edges;
    }
    uint64_t effective_edges = 0;
    for (int i = 0; i < 64; i++) {
      if ((bevel_edges & nurb_body_edge_mask_for_index(i)) != 0 && bevel_radii[i] > 0.0f) {
        effective_edges |= nurb_body_edge_mask_for_index(i);
      }
    }
    return effective_edges;
  }
  if (bevel_radius <= 0.0f) {
    return 0;
  }
  return nurb_body_edge_mask_for_index(bevel_edge);
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

static void nurb_body_set_edge_bevel_radii(const uint64_t bevel_edges,
                                           const float radius,
                                           float bevel_radii[64])
{
  for (int i = 0; i < 64; i++) {
    if ((bevel_edges & nurb_body_edge_mask_for_index(i)) != 0) {
      bevel_radii[i] = radius;
    }
  }
}

static float nurb_body_edge_bevel_radius(const float fallback_radius,
                                         const uint64_t bevel_edges,
                                         const int bevel_edge,
                                         const float bevel_radii[64],
                                         const int edge_index)
{
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(edge_index);
  if (edge_mask != 0 && (bevel_edges & edge_mask) != 0) {
    return bevel_radii[edge_index] > 0.0f ? bevel_radii[edge_index] : 0.0f;
  }
  if (bevel_edges == 0 && bevel_edge == edge_index) {
    return fallback_radius;
  }
  return 0.0f;
}

static int nurb_body_edge_bevel_type(const int fallback_type,
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

static void nurb_body_set_edge_chamfer_mask(const uint64_t edges,
                                            const bool use_chamfer,
                                            uint64_t &chamfer_edges)
{
  if (use_chamfer) {
    chamfer_edges |= edges;
  }
  else {
    chamfer_edges &= ~edges;
  }
}

static void nurb_body_sync_active_selected_edge(NurbBody &body)
{
  body.selected_edge = nurb_body_first_selected_edge(body.selected_edges);
}

static void nurb_body_sync_active_selected_edge(NurbBodyBooleanOp &op)
{
  op.selected_edge = nurb_body_first_selected_edge(op.selected_edges);
  if (op.selected_edges == 0) {
    op.flag &= ~NURB_BODY_BOOLEAN_OP_SELECTED;
  }
  else {
    op.flag |= NURB_BODY_BOOLEAN_OP_SELECTED;
  }
}

static void nurb_body_clear_edge_selection(NurbBody &body)
{
  body.selected_edges = 0;
  body.selected_edge = -1;
  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first); op;
       op = op->next)
  {
    op->selected_edges = 0;
    op->selected_edge = -1;
    op->flag &= ~NURB_BODY_BOOLEAN_OP_SELECTED;
  }
}

static bool nurb_body_has_edge_selection(const NurbBody &body)
{
  if (body.selected_edges != 0) {
    return true;
  }
  for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(body.boolean_ops.first);
       op;
       op = op->next)
  {
    if (op->selected_edges != 0) {
      return true;
    }
  }
  return false;
}

static void nurb_body_clear_edge_hover(NurbBody &body)
{
  body.hovered_edge = -1;
  nurb_body_boolean_ops_clear_flag(body, NURB_BODY_BOOLEAN_OP_HOVERED);
}

static bool nurb_body_has_edge_hover(const NurbBody &body)
{
  if (body.hovered_edge >= 0) {
    return true;
  }
  for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(body.boolean_ops.first);
       op;
       op = op->next)
  {
    if ((op->flag & NURB_BODY_BOOLEAN_OP_HOVERED) != 0 || op->hovered_edge >= 0) {
      return true;
    }
  }
  return false;
}

static void nurb_body_set_select_mode(NurbBody &body, const int mode)
{
  body.select_mode = mode;
  if (body.select_mode != NURB_BODY_SELECT_MODE_EDGE) {
    nurb_body_clear_edge_selection(body);
  }
  nurb_body_clear_edge_hover(body);
}

static bool nurb_body_is_edge_select_mode(const NurbBody &body)
{
  return body.select_mode == NURB_BODY_SELECT_MODE_EDGE;
}

static float3 nurb_body_local_point_to_world(const Object &ob, const float3 &local_point)
{
  float world[3];
  mul_v3_m4v3(world, ob.object_to_world().ptr(), local_point);
  return float3(world[0], world[1], world[2]);
}

static NurbBodyEdgeHit nurb_body_mouse_near_boolean_edge(Object &ob,
                                                         NurbBody &body,
                                                         const ARegion &region,
                                                         const int mval[2],
                                                         float r_anchor[2])
{
  UNUSED_VARS(body);
  const float mouse[2] = {float(mval[0]), float(mval[1])};
  const float threshold_sq = 18.0f * 18.0f;
  float best_anchor[2] = {};
  float best_dist_sq = threshold_sq;
  NurbBodyEdgeHit best_hit;
  Vector<NurbBodyEdgePolyline> polylines;

  BKE_nurb_body_boolean_edge_polylines(&ob, polylines, 64);
  for (const NurbBodyEdgePolyline &polyline : polylines) {
    const bool selectable_body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
    if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_SURFACE) == 0 || polyline.edge_index < 0 ||
        (!selectable_body_edge && polyline.op == nullptr) ||
        (polyline.flag & NURB_BODY_EDGE_POLYLINE_SELECTABLE) == 0 || polyline.points.size() < 2)
    {
      continue;
    }
    for (int i = 1; i < polyline.points.size(); i++) {
      float prev_screen[2];
      float screen[2];
      const float3 prev_world = nurb_body_local_point_to_world(ob, polyline.points[i - 1]);
      const float3 world = nurb_body_local_point_to_world(ob, polyline.points[i]);
      if (ED_view3d_project_float_global(
              &region, prev_world, prev_screen, V3D_PROJ_TEST_NOP) !=
              V3D_PROJ_RET_OK ||
          ED_view3d_project_float_global(&region, world, screen, V3D_PROJ_TEST_NOP) !=
              V3D_PROJ_RET_OK)
      {
        continue;
      }

      const float dist_sq = dist_squared_to_line_segment_v2(mouse, prev_screen, screen);
      if (dist_sq <= best_dist_sq) {
        best_dist_sq = dist_sq;
        closest_to_line_segment_v2(best_anchor, mouse, prev_screen, screen);
        best_hit.op = const_cast<NurbBodyBooleanOp *>(polyline.op);
        best_hit.edge_index = polyline.edge_index;
        best_hit.body_edge = selectable_body_edge;
      }
    }
  }

  if (best_hit.is_valid() && r_anchor != nullptr) {
    copy_v2_v2(r_anchor, best_anchor);
  }
  return best_hit;
}

static bool nurb_body_boolean_op_anchor_screen(const Object &ob,
                                               const NurbBody &body,
                                               const NurbBodyBooleanOp &op,
                                               const int edge_index,
                                               const ARegion &region,
                                               float r_anchor[2])
{
  UNUSED_VARS(body);
  float center[2] = {};
  int center_count = 0;
  Vector<NurbBodyEdgePolyline> polylines;

  BKE_nurb_body_boolean_edge_polylines(&ob, polylines, 64);
  for (const NurbBodyEdgePolyline &polyline : polylines) {
    if (polyline.op != &op || (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0 ||
        polyline.edge_index != edge_index ||
        (polyline.flag & NURB_BODY_EDGE_POLYLINE_SELECTABLE) == 0)
    {
      continue;
    }
    for (const float3 &local : polyline.points) {
      const float3 world = nurb_body_local_point_to_world(ob, local);
      float screen[2];
      if (ED_view3d_project_float_global(&region, world, screen, V3D_PROJ_TEST_NOP) !=
          V3D_PROJ_RET_OK)
      {
        continue;
      }

      add_v2_v2(center, screen);
      center_count++;
    }
  }

  if (center_count == 0) {
    return false;
  }

  mul_v2_fl(center, 1.0f / float(center_count));
  copy_v2_v2(r_anchor, center);
  return true;
}

static bool nurb_body_edge_anchor_screen(const Object &ob,
                                         const int edge_index,
                                         const ARegion &region,
                                         float r_anchor[2])
{
  float center[2] = {};
  int center_count = 0;
  Vector<NurbBodyEdgePolyline> polylines;

  BKE_nurb_body_boolean_edge_polylines(&ob, polylines, 64);
  for (const NurbBodyEdgePolyline &polyline : polylines) {
    if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) == 0 ||
        polyline.edge_index != edge_index ||
        (polyline.flag & NURB_BODY_EDGE_POLYLINE_SELECTABLE) == 0)
    {
      continue;
    }
    for (const float3 &local : polyline.points) {
      const float3 world = nurb_body_local_point_to_world(ob, local);
      float screen[2];
      if (ED_view3d_project_float_global(&region, world, screen, V3D_PROJ_TEST_NOP) !=
          V3D_PROJ_RET_OK)
      {
        continue;
      }

      add_v2_v2(center, screen);
      center_count++;
    }
  }

  if (center_count == 0) {
    return false;
  }

  mul_v2_fl(center, 1.0f / float(center_count));
  copy_v2_v2(r_anchor, center);
  return true;
}

static wmOperatorStatus object_nurb_body_hover_invoke(bContext *C,
                                                      wmOperator * /*op*/,
                                                      const wmEvent *event)
{
  Object *ob = active_nurb_body_object(C);
  ARegion *region = CTX_wm_region(C);
  if (!ob || !region) {
    return OPERATOR_PASS_THROUGH;
  }

  NurbBody *body = id_cast<NurbBody *>(ob->data);
  if (!nurb_body_is_edge_select_mode(*body)) {
    if (nurb_body_has_edge_hover(*body)) {
      nurb_body_clear_edge_hover(*body);
      DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
      WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
      ED_region_tag_redraw(region);
    }
    return OPERATOR_PASS_THROUGH;
  }

  float anchor[2];
  NurbBodyEdgeHit hovered_edge = nurb_body_mouse_near_boolean_edge(
      *ob, *body, *region, event->mval, anchor);

  bool changed = false;
  const int new_body_hovered_edge = hovered_edge.body_edge ? hovered_edge.edge_index : -1;
  if (body->hovered_edge != new_body_hovered_edge) {
    body->hovered_edge = new_body_hovered_edge;
    changed = true;
  }
  for (NurbBodyBooleanOp *op_iter = static_cast<NurbBodyBooleanOp *>(body->boolean_ops.first);
       op_iter;
       op_iter = op_iter->next)
  {
    const bool was_hovered = (op_iter->flag & NURB_BODY_BOOLEAN_OP_HOVERED) != 0;
    const bool is_hovered = !hovered_edge.body_edge && op_iter == hovered_edge.op;
    const int new_hovered_edge = is_hovered ? hovered_edge.edge_index : -1;
    if (was_hovered != is_hovered || op_iter->hovered_edge != new_hovered_edge) {
      op_iter->flag = is_hovered ? (op_iter->flag | NURB_BODY_BOOLEAN_OP_HOVERED) :
                                   (op_iter->flag & ~NURB_BODY_BOOLEAN_OP_HOVERED);
      op_iter->hovered_edge = new_hovered_edge;
      changed = true;
    }
  }

  if (changed) {
    DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
    WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
    ED_region_tag_redraw(region);
  }
  return OPERATOR_PASS_THROUGH;
}

void OBJECT_OT_nurb_body_hover(wmOperatorType *ot)
{
  ot->name = "Hover NURB Body Edge";
  ot->description = "Update the hovered generated NURB Body edge";
  ot->idname = "OBJECT_OT_nurb_body_hover";
  ot->invoke = object_nurb_body_hover_invoke;
  ot->poll = object_nurb_body_poll;
  ot->flag = OPTYPE_INTERNAL;
}

static wmOperatorStatus object_nurb_body_select_cut_edge_invoke(bContext *C,
                                                                wmOperator * /*op*/,
                                                                const wmEvent *event)
{
  Object *ob = active_nurb_body_object(C);
  ARegion *region = CTX_wm_region(C);
  if (!ob || !region) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  NurbBody *body = id_cast<NurbBody *>(ob->data);
  if (body->select_mode == NURB_BODY_SELECT_MODE_OBJECT) {
    return OPERATOR_PASS_THROUGH;
  }
  if (body->select_mode == NURB_BODY_SELECT_MODE_FACE) {
    if (nurb_body_has_edge_selection(*body) || nurb_body_has_edge_hover(*body)) {
      nurb_body_clear_edge_selection(*body);
      nurb_body_clear_edge_hover(*body);
      WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
      ED_region_tag_redraw(region);
      DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
    }
    return OPERATOR_CANCELLED;
  }

  float anchor[2];
  NurbBodyEdgeHit selected_edge = nurb_body_mouse_near_boolean_edge(
      *ob, *body, *region, event->mval, anchor);
  const bool extend = (event->modifier & KM_SHIFT) != 0;

  if (!selected_edge.is_valid()) {
    const bool had_selection = nurb_body_has_edge_selection(*body);
    const bool had_hover = nurb_body_has_edge_hover(*body);
    if (!extend && had_selection) {
      nurb_body_clear_edge_selection(*body);
    }
    if (had_hover) {
      nurb_body_clear_edge_hover(*body);
    }
    if ((!extend && had_selection) || had_hover) {
      WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
      ED_region_tag_redraw(region);
      if (!extend && had_selection) {
        DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
        return OPERATOR_FINISHED;
      }
    }
    return OPERATOR_CANCELLED;
  }

  if (!extend) {
    nurb_body_clear_edge_selection(*body);
  }

  const uint64_t edge_mask = nurb_body_edge_mask_for_index(selected_edge.edge_index);
  if (edge_mask == 0) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  nurb_body_clear_edge_hover(*body);
  body->hovered_edge = selected_edge.body_edge ? selected_edge.edge_index : -1;
  if (selected_edge.body_edge) {
    if (extend && (body->selected_edges & edge_mask)) {
      body->selected_edges &= ~edge_mask;
    }
    else {
      body->selected_edges |= edge_mask;
    }
    nurb_body_sync_active_selected_edge(*body);
  }
  else {
    selected_edge.op->flag |= NURB_BODY_BOOLEAN_OP_HOVERED;
    selected_edge.op->hovered_edge = selected_edge.edge_index;
    if (extend && (selected_edge.op->selected_edges & edge_mask)) {
      selected_edge.op->selected_edges &= ~edge_mask;
    }
    else {
      selected_edge.op->selected_edges |= edge_mask;
    }
    nurb_body_sync_active_selected_edge(*selected_edge.op);
  }
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
  ED_region_tag_redraw(region);
  DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
  return OPERATOR_FINISHED;
}

void OBJECT_OT_nurb_body_select_cut_edge(wmOperatorType *ot)
{
  ot->name = "Select NURB Body Edge";
  ot->description = "Select the generated boolean edge under the cursor";
  ot->idname = "OBJECT_OT_nurb_body_select_cut_edge";
  ot->invoke = object_nurb_body_select_cut_edge_invoke;
  ot->poll = object_nurb_body_poll;
  ot->flag = OPTYPE_UNDO;
}

static wmOperatorStatus object_nurb_body_select_mode_exec(bContext *C, wmOperator *op)
{
  Object *ob = active_nurb_body_object(C);
  if (ob == nullptr) {
    return OPERATOR_CANCELLED;
  }

  const int mode = RNA_enum_get(op->ptr, "mode");
  if (!ELEM(mode,
            NURB_BODY_SELECT_MODE_EDGE,
            NURB_BODY_SELECT_MODE_FACE,
            NURB_BODY_SELECT_MODE_OBJECT))
  {
    return OPERATOR_CANCELLED;
  }

  NurbBody *body = id_cast<NurbBody *>(ob->data);
  nurb_body_set_select_mode(*body, mode);

  DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
  WM_event_add_notifier(C, NC_OBJECT | ND_DATA, ob);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_VIEW3D, nullptr);
  if (ScrArea *area = CTX_wm_area(C)) {
    ED_area_tag_redraw(area);
  }
  if (ARegion *region = CTX_wm_region(C)) {
    ED_region_tag_redraw(region);
  }
  return OPERATOR_FINISHED;
}

void OBJECT_OT_nurb_body_select_mode(wmOperatorType *ot)
{
  ot->name = "Set NURB Body Selection Mode";
  ot->description = "Set whether Object Mode selects NURB Body edges, faces, or objects";
  ot->idname = "OBJECT_OT_nurb_body_select_mode";
  ot->exec = object_nurb_body_select_mode_exec;
  ot->poll = object_nurb_body_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_enum(ot->srna,
               "mode",
               nurb_body_select_mode_items,
               NURB_BODY_SELECT_MODE_EDGE,
               "Mode",
               "Object Mode NURB Body selection target");
}

static wmOperatorStatus object_nurb_body_boolean_apply_exec(bContext *C, wmOperator *op)
{
  Object *active_ob = active_nurb_body_object(C);
  if (!active_ob) {
    return OPERATOR_CANCELLED;
  }

  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  NurbBody *body = id_cast<NurbBody *>(active_ob->data);
  const int operation = RNA_enum_get(op->ptr, "operation");
  const bool display_operands_bounds = RNA_boolean_get(op->ptr, "display_operands_bounds");
  bool operands_hidden = false;
  Vector<Object *> operands;

  nurb_body_preserve_existing_bevel_edges(*body);
  nurb_body_boolean_ops_clear_flag(*body,
                                   NURB_BODY_BOOLEAN_OP_SELECTED | NURB_BODY_BOOLEAN_OP_HOVERED);

  CTX_DATA_BEGIN (C, Object *, selected_ob, selected_editable_objects) {
    if (selected_ob == active_ob || selected_ob->type != OB_NURB_BODY || selected_ob->data == nullptr)
    {
      continue;
    }
    operands.append(selected_ob);
  }
  CTX_DATA_END;

  if (operands.is_empty()) {
    BKE_report(op->reports, RPT_WARNING, "Select at least one other NURB Body operand");
    return OPERATOR_CANCELLED;
  }

  BKE_view_layer_synced_ensure(scene, view_layer);
  for (Object *selected_ob : operands) {

    NurbBodyBooleanOp *boolean_op = MEM_new_zeroed<NurbBodyBooleanOp>(__func__);
    boolean_op->selected_edges = 0;
    boolean_op->selected_edge = -1;
    boolean_op->hovered_edge = -1;
    boolean_op->bevel_edges = 0;
    boolean_op->chamfer_edges = 0;
    boolean_op->bevel_edge = -1;
    boolean_op->bevel_type = NURB_BODY_BEVEL_FILLET;
    nurb_body_boolean_op_snapshot_from_object(*boolean_op, *active_ob, *selected_ob);
    BLI_addtail(&body->boolean_ops, boolean_op);

    if (display_operands_bounds) {
      selected_ob->dt = OB_BOUNDBOX;
      WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, selected_ob);
    }
    else if (Base *base = BKE_view_layer_base_find(view_layer, selected_ob)) {
      base_select(base, BA_DESELECT);
      base->flag |= BASE_HIDDEN;
      operands_hidden = true;
    }

    boolean_op->operation = operation;
  }
  body->selected_edges = 0;
  body->selected_edge = -1;
  body->hovered_edge = -1;

  DEG_id_tag_update(&body->id, ID_RECALC_GEOMETRY);
  if (operands_hidden) {
    BKE_view_layer_need_resync_tag(view_layer);
    DEG_id_tag_update(&scene->id, ID_RECALC_BASE_FLAGS);
    WM_event_add_notifier(C, NC_SCENE | ND_OB_SELECT, scene);
    WM_event_add_notifier(C, NC_SCENE | ND_OB_VISIBLE, scene);
  }
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, active_ob);
  return OPERATOR_FINISHED;
}

void OBJECT_OT_nurb_body_boolean_apply(wmOperatorType *ot)
{
  ot->name = "Apply NURB Body Boolean";
  ot->description = "Apply selected NURB Body operands into the active NURB Body";
  ot->idname = "OBJECT_OT_nurb_body_boolean_apply";
  ot->exec = object_nurb_body_boolean_apply_exec;
  ot->poll = object_nurb_body_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_enum(ot->srna,
               "operation",
               nurb_body_boolean_operation_items,
               NURB_BODY_BOOLEAN_DIFFERENCE,
               "Operation",
               "Boolean operation to store on the active NURB Body");
  RNA_def_boolean(ot->srna,
                  "display_operands_bounds",
                  false,
                  "Display Operands as Bounds",
                  "Show selected operand bodies as bounding boxes instead of hiding them after "
                  "applying the one-time boolean");
}

struct NurbBodyBevelData {
  NurbBodyBooleanOp *op = nullptr;
  int edge_index = -1;
  bool body_edge = false;
  float start_radius = 0.0f;
  int start_bevel_type = NURB_BODY_BEVEL_FILLET;
  int start_bevel_edge = -1;
  uint64_t start_bevel_edges = 0;
  uint64_t start_chamfer_edges = 0;
  float start_bevel_radii[64] = {};
  int start_selected_edge = -1;
  uint64_t start_selected_edges = 0;
  uint64_t edge_mask = 0;
  uint64_t edit_edge_mask = 0;
  int start_mouse_x = 0;
  float radius_step = 0.001f;
  float preview_step = 0.003f;
  float last_preview_radius = -1.0f;
  float mcenter[2] = {};
  void *draw_handle_pixel = nullptr;
};

static NurbBodyEdgeHit nurb_body_active_edge(NurbBody &body)
{
  if (!nurb_body_is_edge_select_mode(body)) {
    return {};
  }

  if (body.selected_edges != 0) {
    return {nullptr, nurb_body_first_selected_edge(body.selected_edges), true};
  }

  NurbBodyEdgeHit selected_edge;
  NurbBodyBooleanOp *first_op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first);
  for (NurbBodyBooleanOp *op = first_op; op; op = op->next) {
    if (op->selected_edges != 0) {
      selected_edge = {op, nurb_body_first_selected_edge(op->selected_edges)};
    }
  }
  if (selected_edge.is_valid()) {
    return selected_edge;
  }
  return {};
}

static NurbBodyEdgeHit nurb_body_selected_or_hovered_edge(NurbBody &body)
{
  if (!nurb_body_is_edge_select_mode(body)) {
    return {};
  }

  if (body.hovered_edge >= 0) {
    return {nullptr, body.hovered_edge, true};
  }
  if (body.selected_edge >= 0) {
    return {nullptr, body.selected_edge, true};
  }

  NurbBodyEdgeHit selected_edge;
  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first); op;
       op = op->next)
  {
    if ((op->flag & NURB_BODY_BOOLEAN_OP_HOVERED) && op->hovered_edge >= 0) {
      return {op, op->hovered_edge};
    }
    if (op->selected_edges != 0) {
      selected_edge = {op, nurb_body_first_selected_edge(op->selected_edges)};
    }
  }
  return selected_edge;
}

static bool nurb_body_boolean_op_anchor_world(const Object &ob,
                                              const NurbBodyBooleanOp &op,
                                              const int edge_index,
                                              float r_anchor[3])
{
  zero_v3(r_anchor);
  int point_count = 0;
  Vector<NurbBodyEdgePolyline> polylines;

  BKE_nurb_body_boolean_edge_polylines(&ob, polylines, 64);
  for (const NurbBodyEdgePolyline &polyline : polylines) {
    if (polyline.op != &op || polyline.edge_index != edge_index ||
        (polyline.flag & NURB_BODY_EDGE_POLYLINE_SELECTABLE) == 0)
    {
      continue;
    }
    for (const float3 &local : polyline.points) {
      const float3 world = nurb_body_local_point_to_world(ob, local);
      add_v3_v3(r_anchor, world);
      point_count++;
    }
  }

  if (point_count == 0) {
    return false;
  }

  mul_v3_fl(r_anchor, 1.0f / float(point_count));
  return true;
}

static void nurb_body_tag_geometry_changed(bContext *C, Object &ob, NurbBody &body)
{
  DEG_id_tag_update(&body.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, &ob);
  if (ARegion *region = CTX_wm_region(C)) {
    ED_region_tag_redraw(region);
  }
}

struct NurbBodyEdgeTranslateData {
  NurbBodyBooleanOp *op = nullptr;
  int edge_index = -1;
  float start_operand_to_target[4][4] = {};
  int start_mouse[2] = {};
  float anchor_world[3] = {};
  float zfac = 1.0f;
  int axis = -1;
};

static void object_nurb_body_edge_translate_finish(wmOperator *op)
{
  NurbBodyEdgeTranslateData *data = static_cast<NurbBodyEdgeTranslateData *>(op->customdata);
  if (data == nullptr) {
    return;
  }
  MEM_delete(data);
  op->customdata = nullptr;
}

static void object_nurb_body_edge_translate_apply(bContext *C,
                                                  Object &ob,
                                                  NurbBody &body,
                                                  NurbBodyEdgeTranslateData &data,
                                                  const wmEvent &event)
{
  ARegion *region = CTX_wm_region(C);
  if (region == nullptr || data.op == nullptr) {
    return;
  }

  const float screen_delta[2] = {float(event.mval[0] - data.start_mouse[0]),
                                 float(event.mval[1] - data.start_mouse[1])};
  float world_delta[3];
  ED_view3d_win_to_delta(region, screen_delta, data.zfac, world_delta);

  if (data.axis != -1) {
    for (int i = 0; i < 3; i++) {
      if (i != data.axis) {
        world_delta[i] = 0.0f;
      }
    }
  }

  float target_inv[4][4];
  float local_delta[3];
  invert_m4_m4(target_inv, ob.object_to_world().ptr());
  copy_v3_v3(local_delta, world_delta);
  mul_mat3_m4_v3(target_inv, local_delta);

  copy_m4_m4(data.op->operand_to_target, data.start_operand_to_target);
  data.op->operand_to_target[3][0] += local_delta[0];
  data.op->operand_to_target[3][1] += local_delta[1];
  data.op->operand_to_target[3][2] += local_delta[2];
  data.op->flag |= NURB_BODY_BOOLEAN_OP_SELECTED | NURB_BODY_BOOLEAN_OP_HOVERED;
  data.op->selected_edges = nurb_body_edge_mask_for_index(data.edge_index);
  data.op->selected_edge = data.edge_index;
  data.op->hovered_edge = data.edge_index;
  body.selected_edges = 0;
  body.selected_edge = -1;
  body.hovered_edge = -1;

  nurb_body_tag_geometry_changed(C, ob, body);
}

static wmOperatorStatus object_nurb_body_edge_translate_modal(bContext *C,
                                                              wmOperator *op,
                                                              const wmEvent *event)
{
  Object *ob = active_nurb_body_object(C);
  NurbBodyEdgeTranslateData *data = static_cast<NurbBodyEdgeTranslateData *>(op->customdata);
  if (ob == nullptr || data == nullptr || data->op == nullptr) {
    object_nurb_body_edge_translate_finish(op);
    return OPERATOR_CANCELLED;
  }

  NurbBody *body = id_cast<NurbBody *>(ob->data);
  switch (event->type) {
    case MOUSEMOVE:
      object_nurb_body_edge_translate_apply(C, *ob, *body, *data, *event);
      return OPERATOR_RUNNING_MODAL;
    case EVT_XKEY:
      if (event->val == KM_PRESS) {
        data->axis = (data->axis == 0) ? -1 : 0;
        object_nurb_body_edge_translate_apply(C, *ob, *body, *data, *event);
      }
      return OPERATOR_RUNNING_MODAL;
    case EVT_YKEY:
      if (event->val == KM_PRESS) {
        data->axis = (data->axis == 1) ? -1 : 1;
        object_nurb_body_edge_translate_apply(C, *ob, *body, *data, *event);
      }
      return OPERATOR_RUNNING_MODAL;
    case EVT_ZKEY:
      if (event->val == KM_PRESS) {
        data->axis = (data->axis == 2) ? -1 : 2;
        object_nurb_body_edge_translate_apply(C, *ob, *body, *data, *event);
      }
      return OPERATOR_RUNNING_MODAL;
    case LEFTMOUSE:
    case EVT_RETKEY:
    case EVT_PADENTER:
      if (event->val == KM_PRESS) {
        object_nurb_body_edge_translate_finish(op);
        return OPERATOR_FINISHED;
      }
      break;
    case RIGHTMOUSE:
    case EVT_ESCKEY:
      if (event->val == KM_PRESS) {
        copy_m4_m4(data->op->operand_to_target, data->start_operand_to_target);
        nurb_body_tag_geometry_changed(C, *ob, *body);
        object_nurb_body_edge_translate_finish(op);
        return OPERATOR_CANCELLED;
      }
      break;
    default:
      break;
  }

  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus object_nurb_body_edge_translate_invoke(bContext *C,
                                                               wmOperator *op,
                                                               const wmEvent *event)
{
  Object *ob = active_nurb_body_object(C);
  ARegion *region = CTX_wm_region(C);
  RegionView3D *rv3d = CTX_wm_region_view3d(C);
  if (ob == nullptr || region == nullptr || rv3d == nullptr) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  NurbBody *body = id_cast<NurbBody *>(ob->data);
  NurbBodyEdgeHit active_edge = nurb_body_selected_or_hovered_edge(*body);
  if (!active_edge.is_valid() || active_edge.body_edge) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  nurb_body_boolean_ops_clear_flag(*body,
                                   NURB_BODY_BOOLEAN_OP_SELECTED | NURB_BODY_BOOLEAN_OP_HOVERED);
  active_edge.op->flag |= NURB_BODY_BOOLEAN_OP_SELECTED | NURB_BODY_BOOLEAN_OP_HOVERED;
  active_edge.op->selected_edges = nurb_body_edge_mask_for_index(active_edge.edge_index);
  active_edge.op->selected_edge = active_edge.edge_index;
  active_edge.op->hovered_edge = active_edge.edge_index;
  body->selected_edges = 0;
  body->selected_edge = -1;
  body->hovered_edge = -1;

  NurbBodyEdgeTranslateData *data = MEM_new<NurbBodyEdgeTranslateData>(__func__);
  data->op = active_edge.op;
  data->edge_index = active_edge.edge_index;
  copy_m4_m4(data->start_operand_to_target, active_edge.op->operand_to_target);
  data->start_mouse[0] = event->mval[0];
  data->start_mouse[1] = event->mval[1];
  if (!nurb_body_boolean_op_anchor_world(
          *ob, *active_edge.op, active_edge.edge_index, data->anchor_world))
  {
    copy_v3_v3(data->anchor_world, ob->object_to_world().location());
  }
  data->zfac = ED_view3d_calc_zfac(rv3d, data->anchor_world);
  op->customdata = data;

  WM_event_add_modal_handler(C, op);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
  ED_region_tag_redraw(region);
  return OPERATOR_RUNNING_MODAL;
}

void OBJECT_OT_nurb_body_edge_translate(wmOperatorType *ot)
{
  ot->name = "Move NURB Body Edge";
  ot->description = "Move the selected stored NURB Body boolean edge";
  ot->idname = "OBJECT_OT_nurb_body_edge_translate";
  ot->invoke = object_nurb_body_edge_translate_invoke;
  ot->modal = object_nurb_body_edge_translate_modal;
  ot->poll = object_nurb_body_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_GRAB_CURSOR_XY;
}

static void object_nurb_body_bevel_finish(bContext *C, wmOperator *op)
{
  NurbBodyBevelData *data = static_cast<NurbBodyBevelData *>(op->customdata);
  if (!data) {
    return;
  }
  if (data->draw_handle_pixel) {
    ARegion *region = CTX_wm_region(C);
    if (region) {
      ED_region_draw_cb_exit(region->runtime->type, data->draw_handle_pixel);
    }
  }
  MEM_delete(data);
  op->customdata = nullptr;
}

static wmOperatorStatus object_nurb_body_bevel_modal(bContext *C,
                                                     wmOperator *op,
                                                     const wmEvent *event)
{
  Object *ob = active_nurb_body_object(C);
  NurbBodyBevelData *data = static_cast<NurbBodyBevelData *>(op->customdata);
  if (!ob || !data) {
    return OPERATOR_CANCELLED;
  }

  NurbBody *body = id_cast<NurbBody *>(ob->data);
  if (!data->body_edge && data->op == nullptr) {
    object_nurb_body_bevel_finish(C, op);
    return OPERATOR_CANCELLED;
  }

  switch (event->type) {
    case MOUSEMOVE: {
      const float delta = float(event->mval[0] - data->start_mouse_x) * data->radius_step;
      const float radius = std::max(0.0f, data->start_radius + delta);
      if (data->body_edge) {
        body->bevel_radius = radius;
        body->bevel_edges = data->edge_mask;
        body->bevel_edge = data->edge_index;
        nurb_body_set_edge_bevel_radii(data->edit_edge_mask, radius, body->bevel_radii);
      }
      else {
        data->op->bevel_radius = radius;
        data->op->bevel_edges = data->edge_mask;
        data->op->bevel_edge = data->edge_index;
        nurb_body_set_edge_bevel_radii(data->edit_edge_mask, radius, data->op->bevel_radii);
      }
      if (std::abs(radius - data->last_preview_radius) >= data->preview_step) {
        data->last_preview_radius = radius;
        nurb_body_tag_geometry_changed(C, *ob, *body);
      }
      else if (ARegion *region = CTX_wm_region(C)) {
        ED_region_tag_redraw(region);
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case EVT_CKEY:
      if (event->val == KM_PRESS) {
        if (data->body_edge) {
          const bool use_chamfer = (body->chamfer_edges & data->edit_edge_mask) !=
                                   data->edit_edge_mask;
          nurb_body_set_edge_chamfer_mask(data->edit_edge_mask, use_chamfer, body->chamfer_edges);
          body->bevel_type = use_chamfer ? NURB_BODY_BEVEL_CHAMFER :
                                           NURB_BODY_BEVEL_FILLET;
        }
        else {
          const bool use_chamfer = (data->op->chamfer_edges & data->edit_edge_mask) !=
                                   data->edit_edge_mask;
          nurb_body_set_edge_chamfer_mask(
              data->edit_edge_mask, use_chamfer, data->op->chamfer_edges);
          data->op->bevel_type = use_chamfer ? NURB_BODY_BEVEL_CHAMFER :
                                               NURB_BODY_BEVEL_FILLET;
        }
        nurb_body_tag_geometry_changed(C, *ob, *body);
      }
      return OPERATOR_RUNNING_MODAL;
    case EVT_FKEY:
      if (event->val == KM_PRESS) {
        if (data->body_edge) {
          nurb_body_set_edge_chamfer_mask(data->edit_edge_mask, false, body->chamfer_edges);
          body->bevel_type = NURB_BODY_BEVEL_FILLET;
        }
        else {
          nurb_body_set_edge_chamfer_mask(data->edit_edge_mask, false, data->op->chamfer_edges);
          data->op->bevel_type = NURB_BODY_BEVEL_FILLET;
        }
        nurb_body_tag_geometry_changed(C, *ob, *body);
      }
      return OPERATOR_RUNNING_MODAL;
    case LEFTMOUSE:
    case EVT_RETKEY:
    case EVT_PADENTER:
      if (event->val == KM_PRESS) {
        if (data->body_edge) {
          body->selected_edges = 0;
          body->selected_edge = -1;
          body->hovered_edge = -1;
        }
        else {
          data->op->selected_edges = 0;
          data->op->selected_edge = -1;
          data->op->hovered_edge = -1;
          data->op->flag &= ~(NURB_BODY_BOOLEAN_OP_SELECTED | NURB_BODY_BOOLEAN_OP_HOVERED);
        }
        nurb_body_tag_geometry_changed(C, *ob, *body);
        object_nurb_body_bevel_finish(C, op);
        return OPERATOR_FINISHED;
      }
      break;
    case RIGHTMOUSE:
    case EVT_ESCKEY:
      if (event->val == KM_PRESS) {
        if (data->body_edge) {
          body->bevel_radius = data->start_radius;
          body->bevel_type = data->start_bevel_type;
          body->bevel_edge = data->start_bevel_edge;
          body->bevel_edges = data->start_bevel_edges;
          body->chamfer_edges = data->start_chamfer_edges;
          std::copy_n(data->start_bevel_radii, 64, body->bevel_radii);
          body->selected_edge = data->start_selected_edge;
          body->selected_edges = data->start_selected_edges;
        }
        else {
          data->op->bevel_radius = data->start_radius;
          data->op->bevel_type = data->start_bevel_type;
          data->op->bevel_edge = data->start_bevel_edge;
          data->op->bevel_edges = data->start_bevel_edges;
          data->op->chamfer_edges = data->start_chamfer_edges;
          std::copy_n(data->start_bevel_radii, 64, data->op->bevel_radii);
          data->op->selected_edge = data->start_selected_edge;
          data->op->selected_edges = data->start_selected_edges;
          nurb_body_sync_active_selected_edge(*data->op);
        }
        nurb_body_tag_geometry_changed(C, *ob, *body);
        object_nurb_body_bevel_finish(C, op);
        return OPERATOR_CANCELLED;
      }
      break;
    default:
      break;
  }
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus object_nurb_body_bevel_invoke(bContext *C,
                                                      wmOperator *op,
                                                      const wmEvent *event)
{
  Object *ob = active_nurb_body_object(C);
  if (!ob) {
    return OPERATOR_CANCELLED;
  }
  NurbBody *body = id_cast<NurbBody *>(ob->data);
  ARegion *region = CTX_wm_region(C);
  NurbBodyEdgeHit active_edge = nurb_body_active_edge(*body);
  if (!active_edge.is_valid()) {
    BKE_report(op->reports, RPT_WARNING, "Select one or more NURB Body edges before beveling");
    return OPERATOR_CANCELLED;
  }

  float anchor_screen[2] = {};
  bool found_anchor = false;
  if (region) {
    found_anchor =
        active_edge.body_edge ?
            nurb_body_edge_anchor_screen(*ob, active_edge.edge_index, *region, anchor_screen) :
            nurb_body_boolean_op_anchor_screen(
                *ob, *body, *active_edge.op, active_edge.edge_index, *region, anchor_screen);
    if (!found_anchor) {
      float hovered_anchor[2];
      if (NurbBodyEdgeHit hovered_edge = nurb_body_mouse_near_boolean_edge(
              *ob, *body, *region, event->mval, hovered_anchor);
          hovered_edge.is_valid())
      {
        copy_v2_v2(anchor_screen, hovered_anchor);
        found_anchor = true;
      }
    }
  }

  const float start_radius =
      active_edge.body_edge ?
          nurb_body_edge_bevel_radius(body->bevel_radius,
                                      body->bevel_edges,
                                      body->bevel_edge,
                                      body->bevel_radii,
                                      active_edge.edge_index) :
          nurb_body_edge_bevel_radius(active_edge.op->bevel_radius,
                                      active_edge.op->bevel_edges,
                                      active_edge.op->bevel_edge,
                                      active_edge.op->bevel_radii,
                                      active_edge.edge_index);
  const int start_bevel_type =
      active_edge.body_edge ?
          nurb_body_edge_bevel_type(body->bevel_type,
                                    body->bevel_edges,
                                    body->chamfer_edges,
                                    body->bevel_edge,
                                    active_edge.edge_index) :
          nurb_body_edge_bevel_type(active_edge.op->bevel_type,
                                    active_edge.op->bevel_edges,
                                    active_edge.op->chamfer_edges,
                                    active_edge.op->bevel_edge,
                                    active_edge.edge_index);
  const int start_bevel_edge = active_edge.body_edge ? body->bevel_edge :
                                                        active_edge.op->bevel_edge;
  const uint64_t start_bevel_edges = active_edge.body_edge ? body->bevel_edges :
                                                             active_edge.op->bevel_edges;
  const uint64_t start_chamfer_edges = active_edge.body_edge ? body->chamfer_edges :
                                                               active_edge.op->chamfer_edges;
  const int start_selected_edge = active_edge.body_edge ? body->selected_edge :
                                                          active_edge.op->selected_edge;
  const uint64_t start_selected_edges = active_edge.body_edge ? body->selected_edges :
                                                               active_edge.op->selected_edges;
  const uint64_t active_edges = active_edge.body_edge ? body->selected_edges :
                                                       active_edge.op->selected_edges;
  const bool had_explicit_bevel_edges = active_edge.body_edge ? body->bevel_edges != 0 :
                                                                active_edge.op->bevel_edges != 0;
  float start_bevel_radii[64];
  std::copy_n(active_edge.body_edge ? body->bevel_radii : active_edge.op->bevel_radii,
              64,
              start_bevel_radii);
  const uint64_t existing_bevel_edges =
      active_edge.body_edge ?
          nurb_body_effective_bevel_edges(
              body->bevel_radius, body->bevel_edges, body->bevel_edge, body->bevel_radii) :
          nurb_body_effective_bevel_edges(active_edge.op->bevel_radius,
                                          active_edge.op->bevel_edges,
                                          active_edge.op->bevel_edge,
                                          active_edge.op->bevel_radii);
  const uint64_t target_edges = existing_bevel_edges | active_edges;
  const int profile = RNA_enum_get(op->ptr, "profile");

  if (active_edge.body_edge) {
    nurb_body_materialize_edge_bevel_radii(
        existing_bevel_edges, body->bevel_radius, body->bevel_radii);
    if (!had_explicit_bevel_edges) {
      nurb_body_set_edge_chamfer_mask(
          existing_bevel_edges, body->bevel_type == NURB_BODY_BEVEL_CHAMFER, body->chamfer_edges);
    }
    body->bevel_edges = target_edges;
    body->bevel_edge = active_edge.edge_index;
    body->bevel_type = profile;
    nurb_body_set_edge_bevel_radii(active_edges, start_radius, body->bevel_radii);
    nurb_body_set_edge_chamfer_mask(
        active_edges, profile == NURB_BODY_BEVEL_CHAMFER, body->chamfer_edges);
  }
  else {
    nurb_body_materialize_edge_bevel_radii(existing_bevel_edges,
                                           active_edge.op->bevel_radius,
                                           active_edge.op->bevel_radii);
    if (!had_explicit_bevel_edges) {
      nurb_body_set_edge_chamfer_mask(existing_bevel_edges,
                                      active_edge.op->bevel_type == NURB_BODY_BEVEL_CHAMFER,
                                      active_edge.op->chamfer_edges);
    }
    active_edge.op->bevel_edges = target_edges;
    active_edge.op->bevel_edge = active_edge.edge_index;
    active_edge.op->bevel_type = profile;
    nurb_body_set_edge_bevel_radii(active_edges, start_radius, active_edge.op->bevel_radii);
    nurb_body_set_edge_chamfer_mask(
        active_edges, profile == NURB_BODY_BEVEL_CHAMFER, active_edge.op->chamfer_edges);
  }

  NurbBodyBevelData *data = MEM_new<NurbBodyBevelData>(__func__);
  data->op = active_edge.op;
  data->edge_index = active_edge.edge_index;
  data->body_edge = active_edge.body_edge;
  data->start_radius = start_radius;
  data->start_bevel_type = start_bevel_type;
  data->start_bevel_edge = start_bevel_edge;
  data->start_bevel_edges = start_bevel_edges;
  data->start_chamfer_edges = start_chamfer_edges;
  std::copy_n(start_bevel_radii, 64, data->start_bevel_radii);
  data->start_selected_edge = start_selected_edge;
  data->start_selected_edges = start_selected_edges;
  data->edge_mask = target_edges;
  data->edit_edge_mask = active_edges;
  data->start_mouse_x = event->mval[0];
  const float radius_limit = active_edge.body_edge ? body->radius : active_edge.op->operand_radius;
  const float radius_scale = std::max(radius_limit, 0.001f);
  data->radius_step = std::clamp(radius_scale * 0.001f, 0.0001f, 0.005f);
  data->preview_step = std::max(data->radius_step * 3.0f, 0.0001f);
  data->last_preview_radius = -1.0f;

  if (region) {
    if (found_anchor) {
      copy_v2_v2(data->mcenter, anchor_screen);
    }
    else {
      copy_v2_fl(data->mcenter, 0.0f);
    }

    data->draw_handle_pixel = ED_region_draw_cb_activate(region->runtime->type,
                                                         ED_region_draw_mouse_line_cb,
                                                         data->mcenter,
                                                         REGION_DRAW_POST_PIXEL);
    ED_region_tag_redraw(region);
  }
  op->customdata = data;

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus object_nurb_body_bevel_exec(bContext *C, wmOperator *op)
{
  Object *ob = active_nurb_body_object(C);
  if (!ob) {
    return OPERATOR_CANCELLED;
  }
  NurbBody *body = id_cast<NurbBody *>(ob->data);
  NurbBodyEdgeHit active_edge = nurb_body_active_edge(*body);
  if (!active_edge.is_valid()) {
    BKE_report(op->reports, RPT_WARNING, "Select one or more NURB Body edges before beveling");
    return OPERATOR_CANCELLED;
  }

  const uint64_t active_edges = active_edge.body_edge ? body->selected_edges :
                                                       active_edge.op->selected_edges;
  const bool had_explicit_bevel_edges = active_edge.body_edge ? body->bevel_edges != 0 :
                                                                active_edge.op->bevel_edges != 0;
  const uint64_t existing_bevel_edges =
      active_edge.body_edge ?
          nurb_body_effective_bevel_edges(
              body->bevel_radius, body->bevel_edges, body->bevel_edge, body->bevel_radii) :
          nurb_body_effective_bevel_edges(active_edge.op->bevel_radius,
                                          active_edge.op->bevel_edges,
                                          active_edge.op->bevel_edge,
                                          active_edge.op->bevel_radii);
  const uint64_t target_edges = existing_bevel_edges | active_edges;
  const float radius = std::max(0.0f, RNA_float_get(op->ptr, "radius"));
  const int profile = RNA_enum_get(op->ptr, "profile");
  if (active_edge.body_edge) {
    nurb_body_materialize_edge_bevel_radii(
        existing_bevel_edges, body->bevel_radius, body->bevel_radii);
    if (!had_explicit_bevel_edges) {
      nurb_body_set_edge_chamfer_mask(
          existing_bevel_edges, body->bevel_type == NURB_BODY_BEVEL_CHAMFER, body->chamfer_edges);
    }
    body->bevel_edges = target_edges;
    body->bevel_edge = active_edge.edge_index;
    body->bevel_type = profile;
    body->bevel_radius = radius;
    nurb_body_set_edge_bevel_radii(active_edges, radius, body->bevel_radii);
    nurb_body_set_edge_chamfer_mask(
        active_edges, profile == NURB_BODY_BEVEL_CHAMFER, body->chamfer_edges);
    body->selected_edges = 0;
    body->selected_edge = -1;
    body->hovered_edge = -1;
  }
  else {
    nurb_body_materialize_edge_bevel_radii(existing_bevel_edges,
                                           active_edge.op->bevel_radius,
                                           active_edge.op->bevel_radii);
    if (!had_explicit_bevel_edges) {
      nurb_body_set_edge_chamfer_mask(existing_bevel_edges,
                                      active_edge.op->bevel_type == NURB_BODY_BEVEL_CHAMFER,
                                      active_edge.op->chamfer_edges);
    }
    active_edge.op->bevel_edges = target_edges;
    active_edge.op->bevel_edge = active_edge.edge_index;
    active_edge.op->bevel_type = profile;
    active_edge.op->bevel_radius = radius;
    nurb_body_set_edge_bevel_radii(active_edges, radius, active_edge.op->bevel_radii);
    nurb_body_set_edge_chamfer_mask(
        active_edges, profile == NURB_BODY_BEVEL_CHAMFER, active_edge.op->chamfer_edges);
    active_edge.op->selected_edges = 0;
    active_edge.op->selected_edge = -1;
    active_edge.op->hovered_edge = -1;
    active_edge.op->flag &= ~(NURB_BODY_BOOLEAN_OP_SELECTED | NURB_BODY_BOOLEAN_OP_HOVERED);
  }
  DEG_id_tag_update(&body->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
  return OPERATOR_FINISHED;
}

void OBJECT_OT_nurb_body_bevel_selected(wmOperatorType *ot)
{
  ot->name = "Bevel NURB Body Edge";
  ot->description = "Adjust the fillet radius for the selected generated boolean edge";
  ot->idname = "OBJECT_OT_nurb_body_bevel_selected";
  ot->invoke = object_nurb_body_bevel_invoke;
  ot->modal = object_nurb_body_bevel_modal;
  ot->exec = object_nurb_body_bevel_exec;
  ot->poll = object_nurb_body_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_GRAB_CURSOR_XY;

  RNA_def_float(ot->srna, "radius", 0.08f, 0.0f, 100000.0f, "Radius", "", 0.0f, 10.0f);
  RNA_def_enum(ot->srna,
               "profile",
               nurb_body_bevel_type_items,
               NURB_BODY_BEVEL_FILLET,
               "Profile",
               "Selected edge bevel profile");
}

}  // namespace blender::ed::object
