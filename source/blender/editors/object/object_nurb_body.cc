/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edobj
 */

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <cstdio>

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
#include "BLI_math_vector.hh"
#include "BLI_time.h"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_nurb_body.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "DEG_depsgraph_build.hh"
#include "DEG_depsgraph.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_keymap.hh"
#include "WM_types.hh"

#include "ED_object.hh"
#include "ED_screen.hh"
#include "ED_select_utils.hh"
#include "ED_space_api.hh"
#include "ED_util.hh"
#include "ED_view3d.hh"

#include "UI_resources.hh"

#include "object_intern.hh"

namespace blender::ed::object {

static constexpr float NURB_BODY_BEVEL_PRECISION_FACTOR = 0.05f;

enum {
  NURB_BODY_BEV_MODAL_PRECISION_ON = 1,
  NURB_BODY_BEV_MODAL_PRECISION_OFF,
};

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

static bool object_nurb_body_select_cut_edge_poll(bContext *C)
{
  return ED_operator_objectmode(C) && ED_operator_region_view3d_active(C);
}

static bool nurb_body_select_mode_is_valid(const int mode)
{
  return ELEM(mode,
              NURB_BODY_SELECT_MODE_EDGE,
              NURB_BODY_SELECT_MODE_FACE,
              NURB_BODY_SELECT_MODE_OBJECT);
}

static int nurb_body_global_select_mode(bContext *C)
{
  const Scene *scene = CTX_data_scene(C);
  if (scene != nullptr && scene->toolsettings != nullptr &&
      nurb_body_select_mode_is_valid(scene->toolsettings->nurb_body_select_mode))
  {
    return scene->toolsettings->nurb_body_select_mode;
  }
  return NURB_BODY_SELECT_MODE_EDGE;
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

static const EnumPropertyItem nurb_body_primitive_items[] = {
    {NURB_BODY_PRIMITIVE_BOX, "BOX", ICON_NURB_BODY_BOX, "Box", "Add an OCCT box"},
    {NURB_BODY_PRIMITIVE_SPHERE,
     "SPHERE",
     ICON_NURB_BODY_SPHERE,
     "Sphere",
     "Add an OCCT sphere"},
    {NURB_BODY_PRIMITIVE_CYLINDER,
     "CYLINDER",
     ICON_NURB_BODY_CYLINDER,
     "Cylinder",
     "Add an OCCT cylinder"},
    {NURB_BODY_PRIMITIVE_CONE, "CONE", ICON_NURB_BODY_CONE, "Cone", "Add an OCCT cone"},
    {NURB_BODY_PRIMITIVE_TORUS,
     "TORUS",
     ICON_NURB_BODY_TORUS,
     "Torus",
     "Add an OCCT torus"},
    {NURB_BODY_PRIMITIVE_WEDGE,
     "WEDGE",
     ICON_NURB_BODY_WEDGE,
     "Wedge",
     "Add an OCCT wedge"},
    {0, nullptr, 0, nullptr, nullptr},
};

static uint64_t nurb_body_edge_mask_for_index(int edge_index);
static int nurb_body_first_selected_edge(uint64_t selected_edges);
static int nurb_body_surface_selection_slot_for_key(const NurbBody &body,
                                                    int preferred_edge_index,
                                                    uint64_t edge_key);
static void nurb_body_assign_edge_bevel_orders(uint64_t edges,
                                               int bevel_order[64],
                                               int &bevel_order_next);

static const char *nurb_body_primitive_name(const int primitive)
{
  switch (primitive) {
    case NURB_BODY_PRIMITIVE_BOX:
      return "NURB Box";
    case NURB_BODY_PRIMITIVE_SPHERE:
      return "NURB Sphere";
    case NURB_BODY_PRIMITIVE_CONE:
      return "NURB Cone";
    case NURB_BODY_PRIMITIVE_TORUS:
      return "NURB Torus";
    case NURB_BODY_PRIMITIVE_WEDGE:
      return "NURB Wedge";
    case NURB_BODY_PRIMITIVE_CYLINDER:
    default:
      return "NURB Cylinder";
  }
}

static bool nurb_body_edge_radii_has_positive(const float edge_radii[64],
                                              const uint64_t edges)
{
  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) != 0 && edge_radii[i] > 0.0f) {
      return true;
    }
  }
  return false;
}

static void nurb_body_materialize_pending_edge_radii(const uint64_t edges,
                                                     const float fallback_radius,
                                                     float edge_radii[64])
{
  if (fallback_radius <= 0.0f) {
    return;
  }
  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) != 0 && edge_radii[i] <= 0.0f) {
      edge_radii[i] = fallback_radius;
    }
  }
}

static float nurb_body_uniform_pending_edge_radius(const uint64_t edges,
                                                   const float fallback_radius,
                                                   const float edge_radii[64])
{
  if (fallback_radius > 0.0f) {
    return fallback_radius;
  }
  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) != 0 && edge_radii[i] > 0.0f) {
      return edge_radii[i];
    }
  }
  return 0.0f;
}

static void nurb_body_force_uniform_pending_edge_radii(const uint64_t edges,
                                                       const float uniform_radius,
                                                       float edge_radii[64])
{
  if (uniform_radius <= 0.0f) {
    return;
  }
  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) != 0) {
      edge_radii[i] = uniform_radius;
    }
  }
}

static void nurb_body_preserve_bevel_target(const uint64_t selected_edges,
                                            const int selected_edge,
                                            uint64_t &bevel_edges,
                                            int &bevel_edge,
                                            uint64_t &chamfer_edges,
                                            const int bevel_type,
                                            const float bevel_radius,
                                            float bevel_radii[64],
                                            int bevel_order[64],
                                            int &bevel_order_next)
{
  uint64_t edges = bevel_edges;
  if (edges == 0 && bevel_edge >= 0) {
    const uint64_t edge_mask = nurb_body_edge_mask_for_index(bevel_edge);
    if (bevel_radius > 0.0f || nurb_body_edge_radii_has_positive(bevel_radii, edge_mask)) {
      edges |= edge_mask;
    }
  }

  uint64_t pending_edges = 0;
  if (bevel_edges == 0) {
    pending_edges = selected_edges;
    if (pending_edges == 0 && selected_edge >= 0) {
      pending_edges = nurb_body_edge_mask_for_index(selected_edge);
    }
  }
  const bool has_pending_operation = pending_edges != 0 &&
                                     (bevel_radius > 0.0f ||
                                      nurb_body_edge_radii_has_positive(bevel_radii,
                                                                        pending_edges));
  if (has_pending_operation) {
    edges |= pending_edges;
  }

  if (edges == 0) {
    return;
  }

  if (has_pending_operation && (pending_edges & (pending_edges - 1)) != 0) {
    const float uniform_radius = nurb_body_uniform_pending_edge_radius(
        pending_edges, bevel_radius, bevel_radii);
    nurb_body_force_uniform_pending_edge_radii(pending_edges, uniform_radius, bevel_radii);
  }
  nurb_body_materialize_pending_edge_radii(edges, bevel_radius, bevel_radii);
  if (bevel_edges == 0 && bevel_edge >= 0) {
    const uint64_t legacy_edge = nurb_body_edge_mask_for_index(bevel_edge);
    if (bevel_type == NURB_BODY_BEVEL_CHAMFER) {
      chamfer_edges |= legacy_edge;
    }
    else {
      chamfer_edges &= ~legacy_edge;
    }
  }
  if (has_pending_operation) {
    if (bevel_type == NURB_BODY_BEVEL_CHAMFER) {
      chamfer_edges |= pending_edges;
    }
    else {
      chamfer_edges &= ~pending_edges;
    }
  }

  bevel_edges = edges;
  chamfer_edges &= bevel_edges;
  bevel_edge = nurb_body_first_selected_edge(bevel_edges);
  nurb_body_assign_edge_bevel_orders(bevel_edges, bevel_order, bevel_order_next);
}

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
  nurb_body_preserve_bevel_target(body.selected_edges,
                                  body.selected_edge,
                                  body.bevel_edges,
                                  body.bevel_edge,
                                  body.chamfer_edges,
                                  body.bevel_type,
                                  body.bevel_radius,
                                  body.bevel_radii,
                                  body.bevel_order,
                                  body.bevel_order_next);
  nurb_body_preserve_bevel_target(body.surface_selected_edges,
                                  body.surface_selected_edge,
                                  body.surface_bevel_edges,
                                  body.surface_bevel_edge,
                                  body.surface_chamfer_edges,
                                  body.surface_bevel_type,
                                  body.surface_bevel_radius,
                                  body.surface_bevel_radii,
                                  body.surface_bevel_order,
                                  body.surface_bevel_order_next);

  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first); op;
       op = op->next)
  {
    nurb_body_preserve_bevel_target(op->selected_edges,
                                    op->selected_edge,
                                    op->bevel_edges,
                                    op->bevel_edge,
                                    op->chamfer_edges,
                                    op->bevel_type,
                                    op->bevel_radius,
                                    op->bevel_radii,
                                    op->bevel_order,
                                    op->bevel_order_next);
  }
}

static bool nurb_body_surface_bevel_may_change_shape(const NurbBody &body)
{
  return body.surface_bevel_edges != 0 &&
         (body.surface_bevel_radius > 0.0f ||
          nurb_body_edge_radii_has_positive(body.surface_bevel_radii, body.surface_bevel_edges));
}

static bool nurb_body_commit_surface_bevel_stage(NurbBody &body)
{
  if (!nurb_body_surface_bevel_may_change_shape(body)) {
    return false;
  }

  NurbBodyBooleanOp *stage = MEM_new_zeroed<NurbBodyBooleanOp>(__func__);
  const uint64_t active_surface_edges = body.surface_selected_edges & body.surface_bevel_edges;
  stage->operation = NURB_BODY_BOOLEAN_SURFACE_BLEND_STAGE;
  stage->selected_edges = active_surface_edges;
  stage->selected_edge = active_surface_edges != 0 ?
                             nurb_body_first_selected_edge(active_surface_edges) :
                             -1;
  stage->hovered_edge = -1;
  stage->bevel_edges = body.surface_bevel_edges;
  stage->chamfer_edges = body.surface_chamfer_edges & body.surface_bevel_edges;
  stage->bevel_edge = body.surface_bevel_edge;
  stage->bevel_type = body.surface_bevel_type;
  stage->bevel_order_next = body.surface_bevel_order_next;
  stage->bevel_radius = body.surface_bevel_radius;
  std::copy_n(body.surface_bevel_radii, 64, stage->bevel_radii);
  std::copy_n(body.surface_bevel_order, 64, stage->bevel_order);
  std::copy_n(body.surface_edge_keys, 64, stage->operand_surface_edge_keys);
  BLI_addtail(&body.boolean_ops, stage);

  body.surface_selected_edges = 0;
  body.surface_bevel_edges = 0;
  body.surface_chamfer_edges = 0;
  std::fill_n(body.surface_bevel_radii, 64, 0.0f);
  std::fill_n(body.surface_bevel_order, 64, 0);
  std::fill_n(body.surface_edge_keys, 64, uint64_t(0));
  body.surface_selected_edge = -1;
  body.surface_hovered_edge = -1;
  body.surface_bevel_edge = -1;
  body.surface_bevel_type = NURB_BODY_BEVEL_FILLET;
  body.surface_bevel_order_next = 1;
  body.surface_bevel_radius = 0.0f;
  return true;
}

static void nurb_body_boolean_op_snapshot_from_object(NurbBodyBooleanOp &op,
                                                      const Object &target_ob,
                                                      const Object &operand_ob)
{
  const NurbBody *operand_body = id_cast<const NurbBody *>(operand_ob.data);
  op.primitive = operand_body->primitive;
  op.operand_radius = operand_body->radius;
  op.operand_depth = operand_body->depth;
  op.operand_minor_radius = operand_body->minor_radius;
  std::copy_n(operand_body->dimensions, 3, op.operand_dimensions);
  op.operand_selected_edges = operand_body->selected_edges;
  op.operand_bevel_edges = operand_body->bevel_edges;
  op.operand_chamfer_edges = operand_body->chamfer_edges;
  op.operand_surface_selected_edges = operand_body->surface_selected_edges;
  op.operand_surface_bevel_edges = operand_body->surface_bevel_edges;
  op.operand_surface_chamfer_edges = operand_body->surface_chamfer_edges;
  std::copy_n(operand_body->bevel_radii, 64, op.operand_bevel_radii);
  std::copy_n(operand_body->surface_bevel_radii, 64, op.operand_surface_bevel_radii);
  std::copy_n(operand_body->bevel_order, 64, op.operand_bevel_order);
  std::copy_n(operand_body->surface_bevel_order, 64, op.operand_surface_bevel_order);
  std::copy_n(operand_body->surface_edge_keys, 64, op.operand_surface_edge_keys);
  op.operand_selected_edge = operand_body->selected_edge;
  op.operand_bevel_edge = operand_body->bevel_edge;
  op.operand_bevel_type = operand_body->bevel_type;
  op.operand_bevel_order_next = operand_body->bevel_order_next;
  op.operand_surface_selected_edge = operand_body->surface_selected_edge;
  op.operand_surface_bevel_edge = operand_body->surface_bevel_edge;
  op.operand_surface_bevel_type = operand_body->surface_bevel_type;
  op.operand_surface_bevel_order_next = operand_body->surface_bevel_order_next;
  op.operand_bevel_radius = operand_body->bevel_radius;
  op.operand_surface_bevel_radius = operand_body->surface_bevel_radius;

  float target_inv[4][4];
  float operand_to_target[4][4];
  invert_m4_m4(target_inv, target_ob.object_to_world().ptr());
  mul_m4_m4m4(operand_to_target, target_inv, operand_ob.object_to_world().ptr());

  float loc[3];
  float rot[3][3];
  mat4_to_loc_rot_size(loc, rot, op.operand_scale, operand_to_target);

  const float unit_scale[3] = {1.0f, 1.0f, 1.0f};
  loc_rot_size_to_mat4(op.operand_to_target, loc, rot, unit_scale);
}

static float nurb_body_boolean_op_scaled_radius(const NurbBodyBooleanOp &op)
{
  const float x_scale = op.operand_scale[0] == 0.0f ? 1.0f : std::abs(op.operand_scale[0]);
  const float y_scale = op.operand_scale[1] == 0.0f ? 1.0f : std::abs(op.operand_scale[1]);
  return std::max(op.operand_radius * (x_scale + y_scale) * 0.5f, 0.001f);
}

static float nurb_body_primitive_bevel_radius_limit(const int primitive,
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

static float nurb_body_bevel_radius_limit(const NurbBody &body)
{
  return nurb_body_primitive_bevel_radius_limit(
      body.primitive, body.radius, body.depth, body.minor_radius, body.dimensions);
}

static float nurb_body_boolean_op_scaled_bevel_radius_limit(const NurbBodyBooleanOp &op)
{
  const float x_scale = op.operand_scale[0] == 0.0f ? 1.0f : std::abs(op.operand_scale[0]);
  const float y_scale = op.operand_scale[1] == 0.0f ? 1.0f : std::abs(op.operand_scale[1]);
  const float z_scale = op.operand_scale[2] == 0.0f ? 1.0f : std::abs(op.operand_scale[2]);
  const float radial_scale = (x_scale + y_scale) * 0.5f;
  const float dimensions[3] = {
      op.operand_dimensions[0] * x_scale,
      op.operand_dimensions[1] * y_scale,
      op.operand_dimensions[2] * z_scale,
  };
  return nurb_body_primitive_bevel_radius_limit(op.primitive,
                                                nurb_body_boolean_op_scaled_radius(op),
                                                op.operand_depth * z_scale,
                                                op.operand_minor_radius * radial_scale,
                                                dimensions);
}

static Object *object_nurb_body_add(bContext *C, wmOperator *op)
{
  ushort local_view_bits;
  float loc[3], rot[3];

  add_generic_get_opts(C, op, 'Z', loc, rot, nullptr, nullptr, &local_view_bits, nullptr);

  const int primitive = RNA_enum_get(op->ptr, "type");
  Object *ob = add_type(
      C, OB_NURB_BODY, nurb_body_primitive_name(primitive), loc, rot, false, local_view_bits);
  if (!ob || !ob->data) {
    return nullptr;
  }

  NurbBody *body = id_cast<NurbBody *>(ob->data);
  body->primitive = primitive;
  body->radius = RNA_float_get(op->ptr, "radius");
  body->depth = RNA_float_get(op->ptr, "depth");
  body->minor_radius = RNA_float_get(op->ptr, "minor_radius");
  const float size = RNA_float_get(op->ptr, "size");
  body->dimensions[0] = size;
  body->dimensions[1] = size;
  body->dimensions[2] = size;
  body->flag = NURB_BODY_SMOOTH_SHADING;
  body->tessellation_topology = NURB_BODY_TESSELLATION_NGONS;
  body->boolean_operation = NURB_BODY_BOOLEAN_DIFFERENCE;
  body->select_mode = nurb_body_global_select_mode(C);
  body->selected_edges = 0;
  body->selected_edge = -1;
  body->hovered_edge = -1;
  body->surface_selected_edges = 0;
  body->surface_selected_edge = -1;
  body->surface_hovered_edge = -1;
  body->bevel_edges = 0;
  body->chamfer_edges = 0;
  body->surface_bevel_edges = 0;
  body->surface_chamfer_edges = 0;
  std::fill_n(body->surface_edge_keys, 64, uint64_t(0));
  std::fill_n(body->bevel_order, 64, 0);
  std::fill_n(body->surface_bevel_order, 64, 0);
  body->bevel_order_next = 1;
  body->surface_bevel_order_next = 1;
  body->bevel_edge = -1;
  body->bevel_type = NURB_BODY_BEVEL_FILLET;
  body->surface_bevel_edge = -1;
  body->surface_bevel_type = NURB_BODY_BEVEL_FILLET;
  body->bevel_radius = 0.0f;
  body->surface_bevel_radius = 0.0f;

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
  ot->description = "Add an OCCT-backed NURB body primitive";
  ot->idname = "OBJECT_OT_nurb_body_add";
  ot->exec = object_nurb_body_add_exec;
  ot->poll = ED_operator_objectmode;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  add_generic_props(ot, false);

  RNA_def_enum(ot->srna,
               "type",
               nurb_body_primitive_items,
               NURB_BODY_PRIMITIVE_CYLINDER,
               "Type",
               "OCCT primitive type");
  RNA_def_float(ot->srna, "size", 2.0f, 0.001f, 100000.0f, "Size", "", 0.001f, 100.0f);
  RNA_def_float(ot->srna, "radius", 1.0f, 0.001f, 100000.0f, "Radius", "", 0.001f, 100.0f);
  RNA_def_float(ot->srna, "depth", 2.0f, 0.001f, 100000.0f, "Depth", "", 0.001f, 100.0f);
  RNA_def_float(
      ot->srna, "minor_radius", 0.25f, 0.001f, 100000.0f, "Minor Radius", "", 0.001f, 100.0f);
}

struct NurbBodyEdgeHit {
  NurbBodyBooleanOp *op = nullptr;
  int edge_index = -1;
  uint64_t edge_key = 0;
  bool body_edge = false;
  bool surface_edge = false;
  float distance_sq = FLT_MAX;

  bool is_valid() const
  {
    return (op != nullptr || body_edge || surface_edge) && edge_index >= 0;
  }
};

struct NurbBodyObjectEdgeHit {
  Object *ob = nullptr;
  NurbBody *body = nullptr;
  Base *base = nullptr;
  NurbBodyEdgeHit edge;
  float anchor[2] = {};

  bool is_valid() const
  {
    return ob != nullptr && body != nullptr && edge.is_valid();
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

static int nurb_body_edge_mask_count(const uint64_t edges)
{
  int count = 0;
  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) != 0) {
      count++;
    }
  }
  return count;
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
    if ((bevel_edges & nurb_body_edge_mask_for_index(i)) != 0 && bevel_radii[i] <= 0.0f) {
      bevel_radii[i] = fallback_radius;
    }
  }
}

static void nurb_body_assign_edge_bevel_orders(const uint64_t edges,
                                               int bevel_order[64],
                                               int &bevel_order_next)
{
  int next_order = std::max(bevel_order_next, 1);
  for (int i = 0; i < 64; i++) {
    if (bevel_order[i] > 0) {
      next_order = std::max(next_order, bevel_order[i] + 1);
    }
  }
  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) != 0 && bevel_order[i] <= 0) {
      bevel_order[i] = next_order++;
    }
  }
  bevel_order_next = next_order;
}

static void nurb_body_promote_edge_bevel_orders(const uint64_t edges,
                                                int bevel_order[64],
                                                int &bevel_order_next)
{
  if (edges == 0) {
    return;
  }

  int next_order = std::max(bevel_order_next, 1);
  for (int i = 0; i < 64; i++) {
    if (bevel_order[i] > 0) {
      next_order = std::max(next_order, bevel_order[i] + 1);
    }
  }
  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) != 0) {
      bevel_order[i] = next_order++;
    }
  }
  bevel_order_next = next_order;
}

static void nurb_body_clear_unused_edge_bevel_orders(const uint64_t edges, int bevel_order[64])
{
  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) == 0) {
      bevel_order[i] = 0;
    }
  }
}

static void nurb_body_set_uniform_edge_bevel_radii(const uint64_t bevel_edges,
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

static float nurb_body_uniform_edge_bevel_radius(const float fallback_radius,
                                                 const uint64_t bevel_edges,
                                                 const int bevel_edge,
                                                 const float bevel_radii[64],
                                                 const uint64_t edges,
                                                 const int preferred_edge)
{
  const uint64_t preferred_mask = nurb_body_edge_mask_for_index(preferred_edge);
  if ((edges & preferred_mask) != 0) {
    const float preferred_radius = nurb_body_edge_bevel_radius(
        fallback_radius, bevel_edges, bevel_edge, bevel_radii, preferred_edge);
    if (preferred_radius > 0.0f) {
      return preferred_radius;
    }
  }

  for (int i = 0; i < 64; i++) {
    if ((edges & nurb_body_edge_mask_for_index(i)) == 0) {
      continue;
    }
    const float radius = nurb_body_edge_bevel_radius(
        fallback_radius, bevel_edges, bevel_edge, bevel_radii, i);
    if (radius > 0.0f) {
      return radius;
    }
  }

  return fallback_radius > 0.0f ? fallback_radius : 0.0f;
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
  body.surface_selected_edge = nurb_body_first_selected_edge(body.surface_selected_edges);
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

static void nurb_body_clear_edge_selection(Object *ob, NurbBody &body)
{
  body.selected_edges = 0;
  body.selected_edge = -1;
  body.surface_selected_edges = 0;
  body.surface_selected_edge = -1;
  for (int i = 0; i < 64; i++) {
    if ((body.surface_bevel_edges & nurb_body_edge_mask_for_index(i)) == 0) {
      body.surface_edge_keys[i] = 0;
    }
  }
  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first); op;
       op = op->next)
  {
    op->selected_edges = 0;
    op->selected_edge = -1;
    op->flag &= ~NURB_BODY_BOOLEAN_OP_SELECTED;
  }
  BKE_nurb_body_selected_edge_key_clear(ob);
}

static bool nurb_body_has_edge_selection(const NurbBody &body)
{
  if (body.selected_edges != 0 || body.surface_selected_edges != 0) {
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

static bool nurb_body_base_can_pick_edges(const Base &base)
{
  if ((base.flag & BASE_ENABLED_AND_MAYBE_VISIBLE_IN_VIEWPORT) == 0) {
    return false;
  }

  Object *ob = base.object;
  if (ob == nullptr || ob->type != OB_NURB_BODY || ob->data == nullptr) {
    return false;
  }

  return true;
}

static bool nurb_body_deselect_objects_for_edge_selection(bContext *C)
{
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (scene == nullptr || view_layer == nullptr) {
    return false;
  }

  bool changed = false;
  BKE_view_layer_synced_ensure(scene, view_layer);
  for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
    Object *ob = base.object;
    if (ob == nullptr) {
      continue;
    }
    if ((base.flag & BASE_SELECTED) != 0 || (ob->base_flag & BASE_SELECTED) != 0) {
      base_select(&base, BA_DESELECT);
      ob->base_flag &= ~BASE_SELECTED;
      DEG_id_tag_update(&ob->id, ID_RECALC_SELECT);
      WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
      changed = true;
    }
  }

  if (changed) {
    BKE_view_layer_need_resync_tag(view_layer);
    WM_event_add_notifier(C, NC_SCENE | ND_OB_SELECT, scene);
  }
  return changed;
}

static void nurb_body_clear_edge_hover(Object *ob, NurbBody &body)
{
  body.hovered_edge = -1;
  body.surface_hovered_edge = -1;
  nurb_body_boolean_ops_clear_flag(body, NURB_BODY_BOOLEAN_OP_HOVERED);
  BKE_nurb_body_hovered_edge_key_clear(ob);
}

static bool nurb_body_has_edge_hover(const NurbBody &body)
{
  if (body.hovered_edge >= 0 || body.surface_hovered_edge >= 0) {
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

static bool nurb_body_polyline_is_selectable_edge(const NurbBodyEdgePolyline &polyline)
{
  const bool selectable_body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
  const bool selectable_surface_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0;
  return (polyline.flag & NURB_BODY_EDGE_POLYLINE_SURFACE) != 0 &&
         (polyline.flag & NURB_BODY_EDGE_POLYLINE_SELECTABLE) != 0 &&
         polyline.edge_index >= 0 && polyline.edge_index < 64 &&
         (selectable_body_edge || selectable_surface_edge || polyline.op != nullptr) &&
         polyline.points.size() >= 2;
}

static bool nurb_body_polyline_is_selected(const NurbBody &body,
                                           const NurbBodyEdgePolyline &polyline)
{
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(polyline.edge_index);
  if (edge_mask == 0) {
    return false;
  }
  if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0) {
    return (body.selected_edges & edge_mask) != 0;
  }
  if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0) {
    if (polyline.edge_key == 0) {
      return false;
    }
    for (int i = 0; i < 64; i++) {
      if ((body.surface_selected_edges & nurb_body_edge_mask_for_index(i)) != 0 &&
          body.surface_edge_keys[i] == polyline.edge_key)
      {
        return true;
      }
    }
    return false;
  }
  const NurbBodyBooleanOp *op = polyline.op;
  return op != nullptr && (op->selected_edges & edge_mask) != 0;
}

static bool nurb_body_apply_polyline_select_action(NurbBody &body,
                                                   const NurbBodyEdgePolyline &polyline,
                                                   const int action)
{
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(polyline.edge_index);
  if (edge_mask == 0) {
    return false;
  }

  if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0) {
    const bool was_selected = (body.selected_edges & edge_mask) != 0;
    switch (action) {
      case SEL_SELECT:
        if (was_selected) {
          return false;
        }
        body.selected_edges |= edge_mask;
        break;
      case SEL_DESELECT:
        if (!was_selected) {
          return false;
        }
        body.selected_edges &= ~edge_mask;
        break;
      case SEL_INVERT:
        body.selected_edges ^= edge_mask;
        break;
      default:
        return false;
    }
    nurb_body_sync_active_selected_edge(body);
    return true;
  }

  if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0) {
    const int edge_index = nurb_body_surface_selection_slot_for_key(
        body, polyline.edge_index, polyline.edge_key);
    const uint64_t surface_edge_mask = nurb_body_edge_mask_for_index(edge_index);
    if (surface_edge_mask == 0) {
      return false;
    }

    const bool was_selected = (body.surface_selected_edges & surface_edge_mask) != 0 &&
                              body.surface_edge_keys[edge_index] == polyline.edge_key;
    switch (action) {
      case SEL_SELECT:
        if (was_selected) {
          return false;
        }
        body.surface_selected_edges |= surface_edge_mask;
        body.surface_edge_keys[edge_index] = polyline.edge_key;
        break;
      case SEL_DESELECT:
        if (!was_selected) {
          return false;
        }
        body.surface_selected_edges &= ~surface_edge_mask;
        if ((body.surface_bevel_edges & surface_edge_mask) == 0) {
          body.surface_edge_keys[edge_index] = 0;
        }
        break;
      case SEL_INVERT:
        if (was_selected) {
          body.surface_selected_edges &= ~surface_edge_mask;
          if ((body.surface_bevel_edges & surface_edge_mask) == 0) {
            body.surface_edge_keys[edge_index] = 0;
          }
        }
        else {
          body.surface_selected_edges |= surface_edge_mask;
          body.surface_edge_keys[edge_index] = polyline.edge_key;
        }
        break;
      default:
        return false;
    }
    nurb_body_sync_active_selected_edge(body);
    return true;
  }

  NurbBodyBooleanOp *op = const_cast<NurbBodyBooleanOp *>(polyline.op);
  if (op == nullptr) {
    return false;
  }
  const bool was_selected = (op->selected_edges & edge_mask) != 0;
  switch (action) {
    case SEL_SELECT:
      if (was_selected) {
        return false;
      }
      op->selected_edges |= edge_mask;
      break;
    case SEL_DESELECT:
      if (!was_selected) {
        return false;
      }
      op->selected_edges &= ~edge_mask;
      break;
    case SEL_INVERT:
      op->selected_edges ^= edge_mask;
      break;
    default:
      return false;
  }
  nurb_body_sync_active_selected_edge(*op);
  return true;
}

bool nurb_body_select_all_edges_from_context(bContext *C, const int action, bool *r_handled)
{
  if (r_handled != nullptr) {
    *r_handled = false;
  }
  if (nurb_body_global_select_mode(C) != NURB_BODY_SELECT_MODE_EDGE) {
    return false;
  }

  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (scene == nullptr || view_layer == nullptr) {
    return false;
  }

  bool any_nurb_body = false;
  bool any_selectable_edge = false;
  bool all_selectable_edges_selected = true;
  BKE_view_layer_synced_ensure(scene, view_layer);
  for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
    if (!nurb_body_base_can_pick_edges(base)) {
      continue;
    }

    any_nurb_body = true;
    Object *ob = base.object;
    NurbBody *body = id_cast<NurbBody *>(ob->data);
    const Span<NurbBodyEdgePolyline> polylines =
        BKE_nurb_body_boolean_edge_polylines_cached(ob, 64);
    for (const NurbBodyEdgePolyline &polyline : polylines) {
      if (!nurb_body_polyline_is_selectable_edge(polyline)) {
        continue;
      }
      any_selectable_edge = true;
      if (!nurb_body_polyline_is_selected(*body, polyline)) {
        all_selectable_edges_selected = false;
      }
    }
  }

  if (!any_nurb_body) {
    return false;
  }
  if (r_handled != nullptr) {
    *r_handled = true;
  }

  int effective_action = action;
  if (effective_action == SEL_TOGGLE) {
    const bool deselect_all = any_selectable_edge && all_selectable_edges_selected;
    effective_action = deselect_all ? SEL_DESELECT : SEL_SELECT;
  }

  bool changed = false;
  for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
    if (!nurb_body_base_can_pick_edges(base)) {
      continue;
    }

    Object *ob = base.object;
    NurbBody *body = id_cast<NurbBody *>(ob->data);
    bool body_changed = false;
    const Span<NurbBodyEdgePolyline> polylines =
        BKE_nurb_body_boolean_edge_polylines_cached(ob, 64);
    for (const NurbBodyEdgePolyline &polyline : polylines) {
      if (!nurb_body_polyline_is_selectable_edge(polyline)) {
        continue;
      }
      body_changed |= nurb_body_apply_polyline_select_action(*body, polyline, effective_action);
    }
    if (!body_changed) {
      continue;
    }

    BKE_nurb_body_selected_edge_key_clear(ob);
    changed = true;
    DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
    WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
  }

  changed |= nurb_body_deselect_objects_for_edge_selection(C);
  if (changed) {
    if (ARegion *region = CTX_wm_region(C)) {
      ED_region_tag_redraw(region);
    }
  }
  return changed;
}

static void nurb_body_set_select_mode(Object *ob, NurbBody &body, const int mode)
{
  body.select_mode = mode;
  if (body.select_mode != NURB_BODY_SELECT_MODE_EDGE) {
    nurb_body_clear_edge_selection(ob, body);
  }
  nurb_body_clear_edge_hover(ob, body);
}

static void nurb_body_set_global_select_mode(bContext *C, const int mode)
{
  if (Scene *scene = CTX_data_scene(C)) {
    if (scene->toolsettings != nullptr) {
      scene->toolsettings->nurb_body_select_mode = mode;
    }
  }

  if (Main *bmain = CTX_data_main(C)) {
    for (Object &ob : bmain->objects) {
      if (ob.type != OB_NURB_BODY || ob.data == nullptr) {
        continue;
      }
      NurbBody *body = id_cast<NurbBody *>(ob.data);
      nurb_body_set_select_mode(&ob, *body, mode);
      DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
    }
  }
}

static float3 nurb_body_local_point_to_world(const Object &ob, const float3 &local_point)
{
  float world[3];
  mul_v3_m4v3(world, ob.object_to_world().ptr(), local_point);
  return float3(world[0], world[1], world[2]);
}

static float nurb_body_polyline_length_local(const Span<float3> points)
{
  float length = 0.0f;
  for (int i = 1; i < points.size(); i++) {
    length += math::distance(points[i - 1], points[i]);
  }
  return length;
}

struct NurbBodyProjectedPoint {
  float co[2] = {};
  bool ok = false;
};

static bool nurb_body_project_polyline_points(const Object &ob,
                                              const ARegion &region,
                                              const Span<float3> local_points,
                                              Vector<NurbBodyProjectedPoint> &r_points,
                                              float r_min[2],
                                              float r_max[2])
{
  r_points.clear();
  r_points.reserve(local_points.size());
  r_min[0] = r_min[1] = FLT_MAX;
  r_max[0] = r_max[1] = -FLT_MAX;

  bool any_projected = false;
  for (const float3 &local : local_points) {
    NurbBodyProjectedPoint projected;
    const float3 world = nurb_body_local_point_to_world(ob, local);
    projected.ok = ED_view3d_project_float_global(
                       &region, world, projected.co, V3D_PROJ_TEST_NOP) == V3D_PROJ_RET_OK;
    if (projected.ok) {
      minmax_v2v2_v2(r_min, r_max, projected.co);
      any_projected = true;
    }
    r_points.append(projected);
  }
  return any_projected;
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
  const Span<NurbBodyEdgePolyline> polylines =
      BKE_nurb_body_boolean_edge_polylines_cached(&ob, 64);
  Vector<NurbBodyProjectedPoint> projected_points;
  for (const NurbBodyEdgePolyline &polyline : polylines) {
    const bool selectable_body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
    const bool selectable_surface_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0;
    if (!nurb_body_polyline_is_selectable_edge(polyline)) {
      continue;
    }
    float bounds_min[2];
    float bounds_max[2];
    if (!nurb_body_project_polyline_points(
            ob, region, polyline.points.as_span(), projected_points, bounds_min, bounds_max))
    {
      continue;
    }
    const float bbox_slop = std::sqrt(best_dist_sq);
    if (mouse[0] < bounds_min[0] - bbox_slop || mouse[0] > bounds_max[0] + bbox_slop ||
        mouse[1] < bounds_min[1] - bbox_slop || mouse[1] > bounds_max[1] + bbox_slop)
    {
      continue;
    }
    for (int i = 1; i < projected_points.size(); i++) {
      const NurbBodyProjectedPoint &prev = projected_points[i - 1];
      const NurbBodyProjectedPoint &screen = projected_points[i];
      if (!prev.ok || !screen.ok) {
        continue;
      }

      const float dist_sq = dist_squared_to_line_segment_v2(mouse, prev.co, screen.co);
      if (dist_sq <= best_dist_sq) {
        best_dist_sq = dist_sq;
        closest_to_line_segment_v2(best_anchor, mouse, prev.co, screen.co);
        best_hit.op = const_cast<NurbBodyBooleanOp *>(polyline.op);
        best_hit.edge_index = polyline.edge_index;
        best_hit.edge_key = polyline.edge_key;
        best_hit.body_edge = selectable_body_edge;
        best_hit.surface_edge = selectable_surface_edge;
        best_hit.distance_sq = dist_sq;
      }
    }
  }

  if (best_hit.is_valid() && r_anchor != nullptr) {
    copy_v2_v2(r_anchor, best_anchor);
  }
  return best_hit;
}

static NurbBodyObjectEdgeHit nurb_body_mouse_near_any_boolean_edge(bContext *C,
                                                                   const ARegion &region,
                                                                   const int mval[2])
{
  NurbBodyObjectEdgeHit best_hit;
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (scene == nullptr || view_layer == nullptr) {
    return best_hit;
  }
  if (nurb_body_global_select_mode(C) != NURB_BODY_SELECT_MODE_EDGE) {
    return best_hit;
  }

  BKE_view_layer_synced_ensure(scene, view_layer);
  for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
    if (!nurb_body_base_can_pick_edges(base)) {
      continue;
    }

    Object *ob = base.object;
    NurbBody *body = id_cast<NurbBody *>(ob->data);
    float anchor[2];
    NurbBodyEdgeHit hit = nurb_body_mouse_near_boolean_edge(*ob, *body, region, mval, anchor);
    if (!hit.is_valid() || hit.distance_sq >= best_hit.edge.distance_sq) {
      continue;
    }

    best_hit.ob = ob;
    best_hit.body = body;
    best_hit.base = &base;
    best_hit.edge = hit;
    copy_v2_v2(best_hit.anchor, anchor);
  }

  return best_hit;
}

static bool nurb_body_polyline_matches_edge_hit_domain(const NurbBodyEdgePolyline &polyline,
                                                       const NurbBodyEdgeHit &edge)
{
  if (!nurb_body_polyline_is_selectable_edge(polyline)) {
    return false;
  }

  const bool polyline_body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
  const bool polyline_surface_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0;
  if (edge.body_edge) {
    return polyline_body_edge;
  }
  if (edge.surface_edge) {
    return polyline_surface_edge && polyline.edge_key != 0;
  }
  return !polyline_body_edge && !polyline_surface_edge &&
         polyline.op == static_cast<const NurbBodyBooleanOp *>(edge.op);
}

static bool nurb_body_polyline_matches_edge_hit(const NurbBodyEdgePolyline &polyline,
                                                const NurbBodyEdgeHit &edge)
{
  if (!nurb_body_polyline_matches_edge_hit_domain(polyline, edge)) {
    return false;
  }
  if (polyline.edge_index != edge.edge_index) {
    return false;
  }
  if (edge.surface_edge) {
    return polyline.edge_key != 0 && polyline.edge_key == edge.edge_key;
  }
  if (edge.edge_key != 0 && polyline.edge_key != 0) {
    return polyline.edge_key == edge.edge_key;
  }
  return true;
}

static bool nurb_body_edge_hit_is_selected(const NurbBody &body, const NurbBodyEdgeHit &edge)
{
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(edge.edge_index);
  if (edge_mask == 0) {
    return false;
  }
  if (edge.body_edge) {
    return (body.selected_edges & edge_mask) != 0;
  }
  if (edge.surface_edge) {
    if (edge.edge_key == 0) {
      return false;
    }
    for (int i = 0; i < 64; i++) {
      if ((body.surface_selected_edges & nurb_body_edge_mask_for_index(i)) != 0 &&
          body.surface_edge_keys[i] == edge.edge_key)
      {
        return true;
      }
    }
    return false;
  }
  return edge.op != nullptr && (edge.op->selected_edges & edge_mask) != 0;
}

static int nurb_body_surface_selection_slot_for_key(const NurbBody &body,
                                                    const int preferred_edge_index,
                                                    const uint64_t edge_key)
{
  if (edge_key == 0) {
    return -1;
  }

  for (int i = 0; i < 64; i++) {
    if (body.surface_edge_keys[i] == edge_key) {
      return i;
    }
  }

  if (preferred_edge_index >= 0 && preferred_edge_index < 64) {
    const uint64_t preferred_mask = nurb_body_edge_mask_for_index(preferred_edge_index);
    if (((body.surface_bevel_edges | body.surface_selected_edges) & preferred_mask) == 0) {
      return preferred_edge_index;
    }
  }

  for (int i = 0; i < 64; i++) {
    const uint64_t edge_mask = nurb_body_edge_mask_for_index(i);
    if (((body.surface_bevel_edges | body.surface_selected_edges) & edge_mask) == 0) {
      return i;
    }
  }

  return -1;
}

static bool nurb_body_apply_exact_surface_select_action(NurbBody &body,
                                                        const NurbBodyEdgeHit &edge,
                                                        const bool extend,
                                                        bool *r_selected)
{
  if (r_selected != nullptr) {
    *r_selected = false;
  }
  const int edge_index = nurb_body_surface_selection_slot_for_key(
      body, edge.edge_index, edge.edge_key);
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(edge_index);
  if (edge_mask == 0) {
    return false;
  }

  const bool was_selected = (body.surface_selected_edges & edge_mask) != 0 &&
                            body.surface_edge_keys[edge_index] == edge.edge_key;
  if (extend && was_selected) {
    body.surface_selected_edges &= ~edge_mask;
    if ((body.surface_bevel_edges & edge_mask) == 0) {
      body.surface_edge_keys[edge_index] = 0;
    }
    if (r_selected != nullptr) {
      *r_selected = false;
    }
  }
  else {
    body.surface_selected_edges |= edge_mask;
    body.surface_edge_keys[edge_index] = edge.edge_key;
    if (r_selected != nullptr) {
      *r_selected = true;
    }
  }
  nurb_body_sync_active_selected_edge(body);
  return true;
}

static bool nurb_body_apply_edge_hit_select_action(NurbBody &body,
                                                   const NurbBodyEdgeHit &edge,
                                                   const bool extend,
                                                   bool *r_selected)
{
  if (r_selected != nullptr) {
    *r_selected = false;
  }

  if (edge.surface_edge) {
    return nurb_body_apply_exact_surface_select_action(body, edge, extend, r_selected);
  }

  const uint64_t edge_mask = nurb_body_edge_mask_for_index(edge.edge_index);
  if (edge_mask == 0) {
    return false;
  }

  auto apply_mask = [&](uint64_t &selected_edges) {
    const bool was_selected = (selected_edges & edge_mask) != 0;
    if (extend && was_selected) {
      selected_edges &= ~edge_mask;
      if (r_selected != nullptr) {
        *r_selected = false;
      }
    }
    else {
      selected_edges |= edge_mask;
      if (r_selected != nullptr) {
        *r_selected = true;
      }
    }
  };

  if (edge.body_edge) {
    apply_mask(body.selected_edges);
    nurb_body_sync_active_selected_edge(body);
    return true;
  }

  if (edge.op != nullptr) {
    apply_mask(edge.op->selected_edges);
    nurb_body_sync_active_selected_edge(*edge.op);
    if (edge.op->selected_edges == 0) {
      edge.op->flag &= ~NURB_BODY_BOOLEAN_OP_SELECTED;
    }
    else {
      edge.op->flag |= NURB_BODY_BOOLEAN_OP_SELECTED;
    }
    return true;
  }

  return false;
}

static bool nurb_body_polyline_endpoint_tangent(const NurbBodyEdgePolyline &polyline,
                                                const bool start_endpoint,
                                                float3 &r_tangent)
{
  const Span<float3> points = polyline.points.as_span();
  const int points_num = int(points.size());
  if (points_num < 2) {
    return false;
  }

  const int endpoint_i = start_endpoint ? 0 : points_num - 1;
  const float3 endpoint = points[endpoint_i];
  for (int i = start_endpoint ? 1 : points_num - 2; i >= 0 && i < points_num;
       i += start_endpoint ? 1 : -1)
  {
    const float3 tangent = points[i] - endpoint;
    if (math::length_squared(tangent) > 1.0e-12f) {
      r_tangent = math::normalize(tangent);
      return true;
    }
  }
  return false;
}

static bool nurb_body_polyline_endpoint_connection(const NurbBodyEdgePolyline &a,
                                                   const bool a_start,
                                                   const NurbBodyEdgePolyline &b,
                                                   const float endpoint_epsilon_sq,
                                                   bool &r_b_start,
                                                   float &r_tangent_dot)
{
  const Span<float3> a_points = a.points.as_span();
  const Span<float3> b_points = b.points.as_span();
  if (a_points.size() < 2 || b_points.size() < 2) {
    return false;
  }

  float3 a_tangent;
  if (!nurb_body_polyline_endpoint_tangent(a, a_start, a_tangent)) {
    return false;
  }

  const float3 a_endpoint = a_start ? a_points[0] : a_points[a_points.size() - 1];
  bool found = false;
  float best_dot = 1.0f;
  bool best_b_start = false;
  for (int b_endpoint_i = 0; b_endpoint_i < 2; b_endpoint_i++) {
    const bool b_start = b_endpoint_i == 0;
    const float3 b_endpoint = b_start ? b_points[0] : b_points[b_points.size() - 1];
    if (math::distance_squared(a_endpoint, b_endpoint) > endpoint_epsilon_sq) {
      continue;
    }

    float3 b_tangent;
    if (!nurb_body_polyline_endpoint_tangent(b, b_start, b_tangent)) {
      continue;
    }

    const float tangent_dot = math::dot(a_tangent, b_tangent);
    if (!found || tangent_dot < best_dot) {
      best_dot = tangent_dot;
      best_b_start = b_start;
      found = true;
    }
  }
  if (!found) {
    return false;
  }

  r_b_start = best_b_start;
  r_tangent_dot = best_dot;
  return true;
}

static float nurb_body_bevel_chain_endpoint_epsilon(
    const NurbBody &body, const Span<const NurbBodyEdgePolyline *> candidates)
{
  float3 bounds_min(FLT_MAX);
  float3 bounds_max(-FLT_MAX);
  bool has_points = false;
  for (const NurbBodyEdgePolyline *polyline : candidates) {
    for (const float3 &point : polyline->points) {
      bounds_min.x = std::min(bounds_min.x, point.x);
      bounds_min.y = std::min(bounds_min.y, point.y);
      bounds_min.z = std::min(bounds_min.z, point.z);
      bounds_max.x = std::max(bounds_max.x, point.x);
      bounds_max.y = std::max(bounds_max.y, point.y);
      bounds_max.z = std::max(bounds_max.z, point.z);
      has_points = true;
    }
  }
  const float extent = has_points ?
                           std::max(std::max(bounds_max.x - bounds_min.x,
                                             bounds_max.y - bounds_min.y),
                                    bounds_max.z - bounds_min.z) :
                           1.0f;
  const float relative_epsilon = std::max(1.0e-5f, extent * 5.0e-5f);
  const float deflection_epsilon = std::max(1.0e-5f, body.tessellation_deflection * 0.1f);
  return std::max(1.0e-5f, std::min(relative_epsilon, deflection_epsilon));
}

enum eNurbBodyBevelChainResult {
  NURB_BODY_BEVEL_CHAIN_SELECTED,
  NURB_BODY_BEVEL_CHAIN_NOT_RESOLVED,
};

static bool nurb_body_bevel_chain_best_continuation(
    const Span<const NurbBodyEdgePolyline *> candidates,
    const float endpoint_epsilon_sq,
    const int current_i,
    const bool current_start,
    const Span<int> chain_indices,
    int &r_next_i,
    bool &r_next_start)
{
  /* Tangents are measured from the shared endpoint into each edge. End-to-end continuation is
   * close to -1.0, while corners and branches are closer to 0 or positive. */
  constexpr float tangent_dot_limit = -0.5f;
  constexpr float ambiguity_margin = 0.15f;

  int best_i = -1;
  bool best_start = false;
  float best_dot = 1.0f;
  float second_best_dot = 1.0f;

  for (const int other_i : candidates.index_range()) {
    if (other_i == current_i || chain_indices.contains(other_i)) {
      continue;
    }

    bool other_start = false;
    float tangent_dot = 1.0f;
    if (!nurb_body_polyline_endpoint_connection(*candidates[current_i],
                                                current_start,
                                                *candidates[other_i],
                                                endpoint_epsilon_sq,
                                                other_start,
                                                tangent_dot))
    {
      continue;
    }

    if (tangent_dot > tangent_dot_limit) {
      continue;
    }

    if (tangent_dot < best_dot) {
      second_best_dot = best_dot;
      best_dot = tangent_dot;
      best_i = other_i;
      best_start = other_start;
    }
    else {
      second_best_dot = std::min(second_best_dot, tangent_dot);
    }
  }

  if (best_i == -1) {
    return false;
  }

  if (second_best_dot <= tangent_dot_limit && second_best_dot - best_dot < ambiguity_margin) {
    return false;
  }

  r_next_i = best_i;
  r_next_start = best_start;
  return true;
}

static void nurb_body_grow_bevel_chain_from_endpoint(
    const Span<const NurbBodyEdgePolyline *> candidates,
    const float endpoint_epsilon_sq,
    const int seed_i,
    const bool seed_start,
    Vector<int> &chain_indices)
{
  int current_i = seed_i;
  bool current_start = seed_start;

  while (true) {
    int next_i = -1;
    bool next_start = false;
    if (!nurb_body_bevel_chain_best_continuation(candidates,
                                                 endpoint_epsilon_sq,
                                                 current_i,
                                                 current_start,
                                                 chain_indices.as_span(),
                                                 next_i,
                                                 next_start))
    {
      return;
    }

    chain_indices.append(next_i);
    current_i = next_i;
    current_start = !next_start;
  }
}

static eNurbBodyBevelChainResult nurb_body_select_bevel_chain(Object &ob,
                                                              NurbBody &body,
                                                              const NurbBodyEdgeHit &seed_edge,
                                                              const bool extend)
{
  const Span<NurbBodyEdgePolyline> polylines =
      BKE_nurb_body_boolean_edge_polylines_cached(&ob, 64);
  Vector<const NurbBodyEdgePolyline *> candidates;
  Vector<int> seed_indices;
  for (const NurbBodyEdgePolyline &polyline : polylines) {
    if (!nurb_body_polyline_matches_edge_hit_domain(polyline, seed_edge)) {
      continue;
    }
    const int candidate_i = int(candidates.size());
    candidates.append(&polyline);
    if (nurb_body_polyline_matches_edge_hit(polyline, seed_edge)) {
      seed_indices.append(candidate_i);
    }
  }

  if (seed_indices.is_empty()) {
    return NURB_BODY_BEVEL_CHAIN_NOT_RESOLVED;
  }

  Vector<int> chain_indices;
  for (const int seed_i : seed_indices) {
    if (!chain_indices.contains(seed_i)) {
      chain_indices.append(seed_i);
    }
  }

  const float endpoint_epsilon = nurb_body_bevel_chain_endpoint_epsilon(body,
                                                                        candidates.as_span());
  const float endpoint_epsilon_sq = endpoint_epsilon * endpoint_epsilon;
  const int seed_count = seed_indices.size();
  for (int seed_order = 0; seed_order < seed_count; seed_order++) {
    const int seed_i = seed_indices[seed_order];
    nurb_body_grow_bevel_chain_from_endpoint(
        candidates.as_span(), endpoint_epsilon_sq, seed_i, true, chain_indices);
    nurb_body_grow_bevel_chain_from_endpoint(
        candidates.as_span(), endpoint_epsilon_sq, seed_i, false, chain_indices);
  }

  if (chain_indices.is_empty()) {
    return NURB_BODY_BEVEL_CHAIN_NOT_RESOLVED;
  }

  const int action = extend && nurb_body_edge_hit_is_selected(body, seed_edge) ? SEL_DESELECT :
                                                                             SEL_SELECT;
  for (const int candidate_i : chain_indices) {
    nurb_body_apply_polyline_select_action(body, *candidates[candidate_i], action);
  }
  return NURB_BODY_BEVEL_CHAIN_SELECTED;
}

static int nurb_body_edge_hit_polyline_flag(const NurbBodyEdgeHit &edge)
{
  if (edge.body_edge) {
    return NURB_BODY_EDGE_POLYLINE_BODY;
  }
  if (edge.surface_edge) {
    return NURB_BODY_EDGE_POLYLINE_FINAL;
  }
  return 0;
}

static bool nurb_body_set_edge_hover(Object *ob,
                                     NurbBody &body,
                                     const NurbBodyEdgeHit *hovered_edge)
{
  bool changed = false;
  const bool key_changed =
      (hovered_edge != nullptr && hovered_edge->is_valid()) ?
          BKE_nurb_body_hovered_edge_key_set(ob,
                                             hovered_edge->op,
                                             hovered_edge->edge_index,
                                             nurb_body_edge_hit_polyline_flag(*hovered_edge),
                                             hovered_edge->edge_key) :
          BKE_nurb_body_hovered_edge_key_clear(ob);
  changed |= key_changed;

  const int new_body_hovered_edge = (hovered_edge != nullptr && hovered_edge->body_edge) ?
                                        hovered_edge->edge_index :
                                        -1;
  const int new_surface_hovered_edge = (hovered_edge != nullptr && hovered_edge->surface_edge) ?
                                           hovered_edge->edge_index :
                                           -1;
  if (body.hovered_edge != new_body_hovered_edge) {
    body.hovered_edge = new_body_hovered_edge;
    changed = true;
  }
  if (body.surface_hovered_edge != new_surface_hovered_edge) {
    body.surface_hovered_edge = new_surface_hovered_edge;
    changed = true;
  }
  for (NurbBodyBooleanOp *op_iter = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first);
       op_iter;
       op_iter = op_iter->next)
  {
    const bool was_hovered = (op_iter->flag & NURB_BODY_BOOLEAN_OP_HOVERED) != 0;
    const bool is_hovered = hovered_edge != nullptr && !hovered_edge->body_edge &&
                            !hovered_edge->surface_edge &&
                            op_iter == hovered_edge->op;
    const int new_hovered_edge = is_hovered ? hovered_edge->edge_index : -1;
    if (was_hovered != is_hovered || op_iter->hovered_edge != new_hovered_edge) {
      op_iter->flag = is_hovered ? (op_iter->flag | NURB_BODY_BOOLEAN_OP_HOVERED) :
                                   (op_iter->flag & ~NURB_BODY_BOOLEAN_OP_HOVERED);
      op_iter->hovered_edge = new_hovered_edge;
      changed = true;
    }
  }
  return changed;
}

static bool nurb_body_clear_visible_edge_state(bContext *C,
                                               ARegion *region,
                                               const bool clear_selection,
                                               const bool clear_hover)
{
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (scene == nullptr || view_layer == nullptr) {
    return false;
  }

  bool changed = false;
  BKE_view_layer_synced_ensure(scene, view_layer);
  for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
    Object *ob = base.object;
    if (ob == nullptr || ob->type != OB_NURB_BODY || ob->data == nullptr ||
        (base.flag & BASE_ENABLED_AND_MAYBE_VISIBLE_IN_VIEWPORT) == 0)
    {
      continue;
    }

    NurbBody *body = id_cast<NurbBody *>(ob->data);
    bool body_changed = false;
    if (clear_selection && nurb_body_has_edge_selection(*body)) {
      nurb_body_clear_edge_selection(ob, *body);
      body_changed = true;
    }
    if (clear_hover && nurb_body_has_edge_hover(*body)) {
      nurb_body_clear_edge_hover(ob, *body);
      body_changed = true;
    }
    if (!body_changed) {
      continue;
    }

    changed = true;
    DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
    WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
  }

  if (changed && region != nullptr) {
    ED_region_tag_redraw(region);
  }
  return changed;
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
  const Span<NurbBodyEdgePolyline> polylines =
      BKE_nurb_body_boolean_edge_polylines_cached(&ob, 64);
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
                                         const int polyline_flag,
                                         const ARegion &region,
                                         float r_anchor[2])
{
  float center[2] = {};
  int center_count = 0;
  const Span<NurbBodyEdgePolyline> polylines =
      BKE_nurb_body_boolean_edge_polylines_cached(&ob, 64);
  for (const NurbBodyEdgePolyline &polyline : polylines) {
    if ((polyline.flag & polyline_flag) == 0 ||
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
  ARegion *region = CTX_wm_region(C);
  if (!region) {
    return OPERATOR_PASS_THROUGH;
  }

  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (scene == nullptr || view_layer == nullptr) {
    return OPERATOR_PASS_THROUGH;
  }

  NurbBodyObjectEdgeHit hovered_edge = nurb_body_mouse_near_any_boolean_edge(
      C, *region, event->mval);
  bool changed = false;

  BKE_view_layer_synced_ensure(scene, view_layer);
  for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
    Object *ob = base.object;
    if (ob == nullptr || ob->type != OB_NURB_BODY || ob->data == nullptr ||
        (base.flag & BASE_ENABLED_AND_MAYBE_VISIBLE_IN_VIEWPORT) == 0)
    {
      continue;
    }

    NurbBody *body = id_cast<NurbBody *>(ob->data);
    const NurbBodyEdgeHit *body_hover = hovered_edge.ob == ob ?
                                            &hovered_edge.edge :
                                            nullptr;
    if (nurb_body_set_edge_hover(ob, *body, body_hover)) {
      DEG_id_tag_update(&body->id, ID_RECALC_SELECT);
      WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
      changed = true;
    }
  }

  if (changed) {
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
  ot->poll = object_nurb_body_select_cut_edge_poll;
  ot->flag = OPTYPE_INTERNAL;
}

static wmOperatorStatus object_nurb_body_select_cut_edge_invoke(bContext *C,
                                                                wmOperator *op,
                                                                const wmEvent *event)
{
  ARegion *region = CTX_wm_region(C);
  if (!region) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  const bool extend = (event->modifier & KM_SHIFT) != 0;
  const bool alt_active = (event->modifier & KM_ALT) != 0;
  const bool bevel_chain = RNA_boolean_get(op->ptr, "bevel_chain") && alt_active;
  if (nurb_body_global_select_mode(C) != NURB_BODY_SELECT_MODE_EDGE) {
    nurb_body_clear_visible_edge_state(C, region, !extend, true);
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  NurbBodyObjectEdgeHit selected = nurb_body_mouse_near_any_boolean_edge(C, *region, event->mval);

  if (!selected.is_valid()) {
    const bool changed = nurb_body_clear_visible_edge_state(C, region, !extend, true);
    if (changed) {
      return OPERATOR_FINISHED;
    }
    return OPERATOR_CANCELLED;
  }

  Object *ob = selected.ob;
  NurbBody *body = selected.body;
  NurbBodyEdgeHit selected_edge = selected.edge;
  const uint64_t edge_mask = nurb_body_edge_mask_for_index(selected_edge.edge_index);
  if (edge_mask == 0) {
    return OPERATOR_CANCELLED;
  }
  if (selected_edge.surface_edge && selected_edge.edge_key == 0) {
    return OPERATOR_CANCELLED;
  }

  if (!extend) {
    nurb_body_clear_visible_edge_state(C, region, true, true);
  }
  else {
    nurb_body_clear_visible_edge_state(C, region, false, true);
  }

  if (selected.base != nullptr) {
    base_activate(C, selected.base);
  }

  nurb_body_set_edge_hover(ob, *body, &selected_edge);
  eNurbBodyBevelChainResult bevel_chain_result = NURB_BODY_BEVEL_CHAIN_NOT_RESOLVED;
  if (bevel_chain) {
    bevel_chain_result = nurb_body_select_bevel_chain(*ob, *body, selected_edge, extend);
  }
  const bool bevel_chain_selected = bevel_chain_result == NURB_BODY_BEVEL_CHAIN_SELECTED;
  if (!bevel_chain_selected) {
    bool edge_selected_after_action = false;
    const bool edge_selected = nurb_body_apply_edge_hit_select_action(
        *body, selected_edge, extend, &edge_selected_after_action);
    if (edge_selected) {
      if (selected_edge.edge_key != 0 && edge_selected_after_action) {
        BKE_nurb_body_selected_edge_key_set(ob,
                                            selected_edge.op,
                                            selected_edge.edge_index,
                                            nurb_body_edge_hit_polyline_flag(selected_edge),
                                            selected_edge.edge_key);
      }
      else {
        BKE_nurb_body_selected_edge_key_clear(ob);
      }
    }
    else {
      BKE_report(op->reports,
                 RPT_WARNING,
                 selected_edge.surface_edge ?
                     "Could not allocate an exact NURB Body edge selection slot" :
                     "Could not resolve the NURB Body edge selection");
      BKE_nurb_body_selected_edge_key_clear(ob);
    }
  }
  else {
    BKE_nurb_body_selected_edge_key_clear(ob);
  }

  nurb_body_deselect_objects_for_edge_selection(C);
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
  ot->poll = object_nurb_body_select_cut_edge_poll;
  ot->flag = OPTYPE_UNDO;

  RNA_def_boolean(ot->srna,
                  "bevel_chain",
                  false,
                  "Bevel Chain",
                  "Select connected edge segments that should bevel together");
}

static wmOperatorStatus object_nurb_body_select_mode_exec(bContext *C, wmOperator *op)
{
  Object *ob = active_nurb_body_object(C);
  if (ob == nullptr) {
    return OPERATOR_CANCELLED;
  }

  const int mode = RNA_enum_get(op->ptr, "mode");
  if (!nurb_body_select_mode_is_valid(mode)) {
    return OPERATOR_CANCELLED;
  }

  nurb_body_set_global_select_mode(C, mode);

  WM_event_add_notifier(C, NC_OBJECT | ND_DATA, ob);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, ob);
  WM_event_add_notifier(C, NC_SCENE | ND_TOOLSETTINGS, nullptr);
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
  nurb_body_commit_surface_bevel_stage(*body);
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
    NurbBody *operand_body = id_cast<NurbBody *>(selected_ob->data);
    nurb_body_preserve_existing_bevel_edges(*operand_body);

    NurbBodyBooleanOp *boolean_op = MEM_new_zeroed<NurbBodyBooleanOp>(__func__);
    boolean_op->selected_edges = 0;
    boolean_op->selected_edge = -1;
    boolean_op->hovered_edge = -1;
    boolean_op->bevel_edges = 0;
    boolean_op->chamfer_edges = 0;
    boolean_op->bevel_order_next = 1;
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
    DEG_id_tag_update(&operand_body->id, ID_RECALC_GEOMETRY);
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
  bool surface_edge = false;
  float start_radius = 0.0f;
  int start_bevel_type = NURB_BODY_BEVEL_FILLET;
  int start_bevel_edge = -1;
  uint64_t start_bevel_edges = 0;
  uint64_t start_chamfer_edges = 0;
  float start_bevel_radii[64] = {};
  int start_bevel_order[64] = {};
  int start_bevel_order_next = 1;
  int start_selected_edge = -1;
  uint64_t start_selected_edges = 0;
  uint64_t edge_mask = 0;
  uint64_t edit_edge_mask = 0;
  int active_edge_count = 1;
  int start_mouse_x = 0;
  int start_mouse_y = 0;
  float drag_pixels = 0.0f;
  float radius_step = 0.001f;
  float preview_radius_step = 0.004f;
  float max_preview_radius = FLT_MAX;
  float max_chamfer_preview_radius = FLT_MAX;
  float last_preview_radius = -1.0f;
  bool use_precision = false;
  bool precision_key_down = false;
  bool precision_anchor_valid = false;
  float precision_anchor_radius = 0.0f;
  float precision_anchor_drag_pixels = 0.0f;
  uint64_t debug_tick = 0;
  float mcenter[2] = {};
  void *draw_handle_pixel = nullptr;
};

struct NurbBodyBevelTarget {
  uint64_t *selected_edges = nullptr;
  uint64_t *bevel_edges = nullptr;
  uint64_t *chamfer_edges = nullptr;
  float *bevel_radius = nullptr;
  float *bevel_radii = nullptr;
  int *bevel_order = nullptr;
  int *bevel_order_next = nullptr;
  int *selected_edge = nullptr;
  int *hovered_edge = nullptr;
  int *bevel_edge = nullptr;
  int *bevel_type = nullptr;
};

static NurbBodyBevelTarget nurb_body_bevel_target_for_edge(NurbBody &body,
                                                           NurbBodyEdgeHit &edge)
{
  if (edge.body_edge) {
    return {&body.selected_edges,
            &body.bevel_edges,
            &body.chamfer_edges,
            &body.bevel_radius,
            body.bevel_radii,
            body.bevel_order,
            &body.bevel_order_next,
            &body.selected_edge,
            &body.hovered_edge,
            &body.bevel_edge,
            &body.bevel_type};
  }
  if (edge.surface_edge) {
    return {&body.surface_selected_edges,
            &body.surface_bevel_edges,
            &body.surface_chamfer_edges,
            &body.surface_bevel_radius,
            body.surface_bevel_radii,
            body.surface_bevel_order,
            &body.surface_bevel_order_next,
            &body.surface_selected_edge,
            &body.surface_hovered_edge,
            &body.surface_bevel_edge,
            &body.surface_bevel_type};
  }
  return {&edge.op->selected_edges,
          &edge.op->bevel_edges,
          &edge.op->chamfer_edges,
          &edge.op->bevel_radius,
          edge.op->bevel_radii,
          edge.op->bevel_order,
          &edge.op->bevel_order_next,
          &edge.op->selected_edge,
          &edge.op->hovered_edge,
          &edge.op->bevel_edge,
          &edge.op->bevel_type};
}

static NurbBodyBevelTarget nurb_body_bevel_target_for_data(NurbBody &body,
                                                           NurbBodyBevelData &data)
{
  NurbBodyEdgeHit edge;
  edge.op = data.op;
  edge.edge_index = data.edge_index;
  edge.body_edge = data.body_edge;
  edge.surface_edge = data.surface_edge;
  return nurb_body_bevel_target_for_edge(body, edge);
}

static NurbBodyEdgeHit nurb_body_active_edge(bContext *C, NurbBody &body)
{
  if (nurb_body_global_select_mode(C) != NURB_BODY_SELECT_MODE_EDGE) {
    return {};
  }

  if (body.selected_edges != 0) {
    NurbBodyEdgeHit hit;
    hit.edge_index = nurb_body_first_selected_edge(body.selected_edges);
    hit.body_edge = true;
    return hit;
  }
  if (body.surface_selected_edges != 0) {
    NurbBodyEdgeHit hit;
    hit.edge_index = nurb_body_first_selected_edge(body.surface_selected_edges);
    hit.edge_key = hit.edge_index >= 0 ? body.surface_edge_keys[hit.edge_index] : 0;
    hit.surface_edge = true;
    return hit;
  }

  NurbBodyEdgeHit selected_edge;
  NurbBodyBooleanOp *first_op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first);
  for (NurbBodyBooleanOp *op = first_op; op; op = op->next) {
    if (op->selected_edges != 0) {
      selected_edge.op = op;
      selected_edge.edge_index = nurb_body_first_selected_edge(op->selected_edges);
    }
  }
  if (selected_edge.is_valid()) {
    return selected_edge;
  }
  return {};
}

static NurbBodyEdgeHit nurb_body_selected_or_hovered_edge(bContext *C, NurbBody &body)
{
  if (nurb_body_global_select_mode(C) != NURB_BODY_SELECT_MODE_EDGE) {
    return {};
  }

  if (body.hovered_edge >= 0) {
    NurbBodyEdgeHit hit;
    hit.edge_index = body.hovered_edge;
    hit.body_edge = true;
    return hit;
  }
  if (body.surface_hovered_edge >= 0) {
    NurbBodyEdgeHit hit;
    hit.edge_index = body.surface_hovered_edge;
    hit.edge_key = hit.edge_index >= 0 ? body.surface_edge_keys[hit.edge_index] : 0;
    hit.surface_edge = true;
    return hit;
  }
  if (body.selected_edge >= 0) {
    NurbBodyEdgeHit hit;
    hit.edge_index = body.selected_edge;
    hit.body_edge = true;
    return hit;
  }
  if (body.surface_selected_edge >= 0) {
    NurbBodyEdgeHit hit;
    hit.edge_index = body.surface_selected_edge;
    hit.edge_key = hit.edge_index >= 0 ? body.surface_edge_keys[hit.edge_index] : 0;
    hit.surface_edge = true;
    return hit;
  }

  NurbBodyEdgeHit selected_edge;
  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body.boolean_ops.first); op;
       op = op->next)
  {
    if ((op->flag & NURB_BODY_BOOLEAN_OP_HOVERED) && op->hovered_edge >= 0) {
      NurbBodyEdgeHit hit;
      hit.op = op;
      hit.edge_index = op->hovered_edge;
      return hit;
    }
    if (op->selected_edges != 0) {
      selected_edge.op = op;
      selected_edge.edge_index = nurb_body_first_selected_edge(op->selected_edges);
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
  const Span<NurbBodyEdgePolyline> polylines =
      BKE_nurb_body_boolean_edge_polylines_cached(&ob, 64);
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

static float nurb_body_modal_bevel_radius_limit(const Object & /*ob*/,
                                                const NurbBodyEdgeHit & /*edge*/,
                                                const float radius_limit)
{
  return std::max(radius_limit, 0.001f);
}

static float nurb_body_modal_bevel_edge_length(const Object &ob, const NurbBodyEdgeHit &edge)
{
  float length = 0.0f;
  const Span<NurbBodyEdgePolyline> polylines =
      BKE_nurb_body_boolean_edge_polylines_cached(&ob, 64);
  for (const NurbBodyEdgePolyline &polyline : polylines) {
    if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_SELECTABLE) == 0 ||
        polyline.edge_index != edge.edge_index || polyline.points.size() < 2)
    {
      continue;
    }
    if (edge.body_edge) {
      if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) == 0) {
        continue;
      }
    }
    else if (edge.surface_edge) {
      if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) == 0) {
        continue;
      }
      if (edge.edge_key != 0 && polyline.edge_key != edge.edge_key) {
        continue;
      }
    }
    else if (polyline.op != edge.op || (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0 ||
             (polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0)
    {
      continue;
    }
    length = std::max(length, nurb_body_polyline_length_local(polyline.points.as_span()));
  }
  return length;
}

static int nurb_body_debug_bevel_domain(const NurbBodyBevelData &data)
{
  if (data.body_edge) {
    return 1;
  }
  if (data.surface_edge) {
    return 2;
  }
  return 3;
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
  body.surface_selected_edges = 0;
  body.surface_selected_edge = -1;
  body.surface_hovered_edge = -1;

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
  NurbBodyEdgeHit active_edge = nurb_body_selected_or_hovered_edge(C, *body);
  if (!active_edge.is_valid() || active_edge.body_edge || active_edge.surface_edge ||
      active_edge.op == nullptr)
  {
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
  body->surface_selected_edges = 0;
  body->surface_selected_edge = -1;
  body->surface_hovered_edge = -1;

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
  if (!data->body_edge && !data->surface_edge && data->op == nullptr) {
    body->flag &= ~NURB_BODY_FAST_BEVEL_PREVIEW;
    nurb_body_tag_geometry_changed(C, *ob, *body);
    object_nurb_body_bevel_finish(C, op);
    return OPERATOR_CANCELLED;
  }
  NurbBodyBevelTarget target = nurb_body_bevel_target_for_data(*body, *data);

  int live_modifier = -1;
  if (const wmWindow *win = CTX_wm_window(C)) {
    if (win->runtime != nullptr && win->runtime->eventstate != nullptr) {
      live_modifier = int(win->runtime->eventstate->modifier);
    }
  }

  auto precision_active = [&]() {
    return data->use_precision;
  };

  auto precision_factor = [&]() {
    return precision_active() ? NURB_BODY_BEVEL_PRECISION_FACTOR : 1.0f;
  };

  auto active_chamfer_mode = [&]() {
    return data->edit_edge_mask != 0 &&
           ((*target.chamfer_edges & data->edit_edge_mask) == data->edit_edge_mask);
  };

  auto current_max_preview_radius = [&]() {
    return active_chamfer_mode() ? std::min(data->max_preview_radius,
                                            data->max_chamfer_preview_radius) :
                                   data->max_preview_radius;
  };

  auto radius_from_raw_drag = [&]() {
    const float delta = data->drag_pixels * data->radius_step;
    return std::clamp(data->start_radius + delta, 0.0f, current_max_preview_radius());
  };

  auto radius_from_mouse = [&]() {
    if (!data->use_precision || !data->precision_anchor_valid) {
      return radius_from_raw_drag();
    }
    const float drag_delta = data->drag_pixels - data->precision_anchor_drag_pixels;
    const float radius = data->precision_anchor_radius +
                         drag_delta * data->radius_step * NURB_BODY_BEVEL_PRECISION_FACTOR;
    return std::clamp(radius, 0.0f, current_max_preview_radius());
  };

  auto rebase_drag_pixels_to_radius = [&](const float radius) {
    if (data->radius_step > 0.0f && std::isfinite(radius)) {
      data->drag_pixels = (radius - data->start_radius) / data->radius_step;
    }
  };

  auto set_precision = [&](const bool enabled) {
    if (enabled) {
      if (!data->use_precision) {
        data->precision_anchor_radius = radius_from_mouse();
        data->precision_anchor_drag_pixels = data->drag_pixels;
        data->precision_anchor_valid = true;
      }
      data->use_precision = true;
      return;
    }

    if (data->use_precision) {
      rebase_drag_pixels_to_radius(radius_from_mouse());
    }
    data->use_precision = false;
    data->precision_anchor_valid = false;
  };

  auto modifier_shift_down = [&]() {
    return ((event->modifier & KM_SHIFT) != 0) ||
           (live_modifier != -1 && (live_modifier & KM_SHIFT) != 0);
  };

  auto sync_precision_from_mouse = [&]() {
    if (event->type == MOUSEMOVE) {
      set_precision(data->precision_key_down || modifier_shift_down());
    }
  };

  sync_precision_from_mouse();

  auto accumulate_mouse_delta = [&]() {
    const float dx = float(event->xy[0] - event->prev_xy[0]);
    const float dy = float(event->xy[1] - event->prev_xy[1]);
    const float pixels = std::sqrt(dx * dx + dy * dy);
    if (!std::isfinite(pixels) || pixels <= 0.0f) {
      return;
    }
    const float signed_pixels = (dx < 0.0f ? -1.0f : 1.0f) * std::min(pixels, 48.0f);
    data->drag_pixels += signed_pixels;
  };

  auto apply_radius = [&](const float radius) {
    *target.bevel_radius = radius;
    *target.bevel_edges = data->edge_mask;
    *target.bevel_edge = data->edge_index;
    nurb_body_set_uniform_edge_bevel_radii(data->edit_edge_mask, radius, target.bevel_radii);
  };

  auto current_preview_radius_step = [&]() {
    float step = data->preview_radius_step;
    if (data->active_edge_count > 1) {
      const float edge_factor = data->active_edge_count > 1 ?
                                    std::min(float(data->active_edge_count) * 2.0f, 16.0f) :
                                    4.0f;
      step = std::max(step, data->radius_step * edge_factor);
      step = std::max(step, current_max_preview_radius() / 160.0f);
    }
    else if (active_chamfer_mode()) {
      step = std::max(step, data->radius_step * 4.0f);
      step = std::max(step, current_max_preview_radius() / 120.0f);
    }
    return step * precision_factor();
  };

  auto begin_profile_tick = [&](const char *event_name, const float radius) {
    if (!BKE_nurb_body_debug_bevel_enabled()) {
      return;
    }
    data->debug_tick++;
    const double tick_time = BLI_time_now_seconds();
    const int domain = nurb_body_debug_bevel_domain(*data);
    const float preview_step = current_preview_radius_step();
    BKE_nurb_body_debug_bevel_set_drag_tick(data->debug_tick,
                                            radius,
                                            data->edge_index,
                                            domain,
                                            data->edit_edge_mask);
    std::fprintf(stderr,
                 "NURB_BODY_BEVEL_DRAG tick=%llu event=%s domain=%d edge=%d "
                 "mouse_x=%d mouse_y=%d radius=%.8f start_radius=%.8f "
                 "last_preview=%.8f raw_radius=%.8f drag_pixels=%.3f radius_step=%.8f "
                 "precision=%d precision_factor=%.2f event_modifier=%d live_modifier=%d "
                 "modifier_shift=%d precision_key_down=%d precision_anchor=%d "
                 "precision_anchor_radius=%.8f precision_anchor_drag=%.3f "
                 "preview_step=%.8f max=%.8f "
                 "chamfer_max=%.8f edit_mask=%llu edge_mask=%llu "
                 "chamfer_edges=%llu selected_edges=%llu profile=%d active_edges=%d "
                 "tag_geometry=1 fast_preview=%d time=%.6f\n",
                 static_cast<unsigned long long>(data->debug_tick),
                 event_name,
                 domain,
                 data->edge_index,
                 event->mval[0],
                 event->mval[1],
                 double(radius),
                 double(data->start_radius),
                 double(data->last_preview_radius),
                 double(radius_from_raw_drag()),
                 double(data->drag_pixels),
                 double(data->radius_step),
                 int(precision_active()),
                 double(precision_factor()),
                 int(event->modifier),
                 live_modifier,
                 int(modifier_shift_down()),
                 int(data->precision_key_down),
                 int(data->precision_anchor_valid),
                 double(data->precision_anchor_radius),
                 double(data->precision_anchor_drag_pixels),
                 double(preview_step),
                 double(current_max_preview_radius()),
                 double(data->max_chamfer_preview_radius),
                 static_cast<unsigned long long>(data->edit_edge_mask),
                 static_cast<unsigned long long>(data->edge_mask),
                 static_cast<unsigned long long>(*target.chamfer_edges),
                 static_cast<unsigned long long>(*target.selected_edges),
                 *target.bevel_type,
                 data->active_edge_count,
                 int((body->flag & NURB_BODY_FAST_BEVEL_PREVIEW) != 0),
                 tick_time);
    std::fflush(stderr);
  };

  if (event->type == EVT_MODAL_MAP) {
    switch (event->val) {
      case NURB_BODY_BEV_MODAL_PRECISION_ON: {
        data->precision_key_down = true;
        set_precision(true);
        return OPERATOR_RUNNING_MODAL;
      }
      case NURB_BODY_BEV_MODAL_PRECISION_OFF: {
        data->precision_key_down = false;
        set_precision(false);
        return OPERATOR_RUNNING_MODAL;
      }
      default:
        break;
    }
  }

  switch (event->type) {
    case MOUSEMOVE: {
      accumulate_mouse_delta();
      const float radius = radius_from_mouse();
      const bool first_preview = data->last_preview_radius < 0.0f;
      const float max_preview_radius = current_max_preview_radius();
      const bool force_endpoint_preview = radius == 0.0f || radius == max_preview_radius;
      const bool endpoint_changed = force_endpoint_preview && radius != data->last_preview_radius;
      const float preview_step = current_preview_radius_step();
      const bool radius_step_ready = std::abs(radius - data->last_preview_radius) >=
                                     preview_step;
      const bool will_tag_geometry = first_preview || endpoint_changed || radius_step_ready;
      if (BKE_nurb_body_debug_bevel_enabled()) {
        data->debug_tick++;
        const double tick_time = BLI_time_now_seconds();
        const int domain = nurb_body_debug_bevel_domain(*data);
        BKE_nurb_body_debug_bevel_set_drag_tick(data->debug_tick,
                                                radius,
                                                data->edge_index,
                                                domain,
                                                data->edit_edge_mask);
        std::fprintf(stderr,
                     "NURB_BODY_BEVEL_DRAG tick=%llu event=mousemove domain=%d edge=%d "
                     "mouse_x=%d mouse_y=%d start_mouse_x=%d start_mouse_y=%d "
                     "radius=%.8f start_radius=%.8f "
                     "last_preview=%.8f raw_radius=%.8f drag_pixels=%.3f radius_step=%.8f "
                     "precision=%d precision_factor=%.2f event_modifier=%d live_modifier=%d "
                     "modifier_shift=%d precision_key_down=%d precision_anchor=%d "
                     "precision_anchor_radius=%.8f precision_anchor_drag=%.3f "
                     "preview_step=%.8f max=%.8f "
                     "chamfer_max=%.8f edit_mask=%llu edge_mask=%llu "
                     "chamfer_edges=%llu selected_edges=%llu profile=%d active_edges=%d "
                     "first=%d endpoint=%d "
                     "step_ready=%d tag_geometry=%d fast_preview=%d time=%.6f\n",
                     static_cast<unsigned long long>(data->debug_tick),
                     domain,
                     data->edge_index,
                     event->mval[0],
                     event->mval[1],
                     data->start_mouse_x,
                     data->start_mouse_y,
                     double(radius),
                     double(data->start_radius),
                     double(data->last_preview_radius),
                     double(radius_from_raw_drag()),
                     double(data->drag_pixels),
                     double(data->radius_step),
                     int(precision_active()),
                     double(precision_factor()),
                     int(event->modifier),
                     live_modifier,
                     int(modifier_shift_down()),
                     int(data->precision_key_down),
                     int(data->precision_anchor_valid),
                     double(data->precision_anchor_radius),
                     double(data->precision_anchor_drag_pixels),
                     double(preview_step),
                     double(max_preview_radius),
                     double(data->max_chamfer_preview_radius),
                     static_cast<unsigned long long>(data->edit_edge_mask),
                     static_cast<unsigned long long>(data->edge_mask),
                     static_cast<unsigned long long>(*target.chamfer_edges),
                     static_cast<unsigned long long>(*target.selected_edges),
                     *target.bevel_type,
                     data->active_edge_count,
                     int(first_preview),
                     int(endpoint_changed),
                     int(radius_step_ready),
                     int(will_tag_geometry),
                     int((body->flag & NURB_BODY_FAST_BEVEL_PREVIEW) != 0),
                     tick_time);
        std::fflush(stderr);
      }
      if (will_tag_geometry) {
        apply_radius(radius);
        data->last_preview_radius = radius;
        nurb_body_tag_geometry_changed(C, *ob, *body);
      }
      else if (ARegion *region = CTX_wm_region(C)) {
        ED_region_tag_redraw(region);
        BKE_nurb_body_debug_bevel_end_drag_tick("redraw_only");
      }
      else {
        BKE_nurb_body_debug_bevel_end_drag_tick("redraw_only_no_region");
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case EVT_LEFTSHIFTKEY:
    case EVT_RIGHTSHIFTKEY:
      if (event->val == KM_PRESS) {
        data->precision_key_down = true;
        set_precision(true);
      }
      else if (event->val == KM_RELEASE) {
        data->precision_key_down = false;
        set_precision(false);
      }
      if (ARegion *region = CTX_wm_region(C)) {
        ED_region_tag_redraw(region);
      }
      return OPERATOR_RUNNING_MODAL;
    case EVT_CKEY:
      if (event->val == KM_PRESS) {
        const float radius = radius_from_mouse();
        const bool use_chamfer = (*target.chamfer_edges & data->edit_edge_mask) !=
                                 data->edit_edge_mask;
        apply_radius(radius);
        nurb_body_set_edge_chamfer_mask(data->edit_edge_mask, use_chamfer, *target.chamfer_edges);
        *target.bevel_type = use_chamfer ? NURB_BODY_BEVEL_CHAMFER : NURB_BODY_BEVEL_FILLET;
        begin_profile_tick("profile_toggle", radius);
        data->last_preview_radius = radius;
        nurb_body_tag_geometry_changed(C, *ob, *body);
      }
      return OPERATOR_RUNNING_MODAL;
    case EVT_FKEY:
      if (event->val == KM_PRESS) {
        const float radius = radius_from_mouse();
        apply_radius(radius);
        nurb_body_set_edge_chamfer_mask(data->edit_edge_mask, false, *target.chamfer_edges);
        *target.bevel_type = NURB_BODY_BEVEL_FILLET;
        begin_profile_tick("profile_fillet", radius);
        data->last_preview_radius = radius;
        nurb_body_tag_geometry_changed(C, *ob, *body);
      }
      return OPERATOR_RUNNING_MODAL;
    case LEFTMOUSE:
    case EVT_RETKEY:
    case EVT_PADENTER:
      if (event->val == KM_PRESS) {
        const float accepted_radius = data->last_preview_radius >= 0.0f ?
                                          data->last_preview_radius :
                                          radius_from_mouse();
        apply_radius(accepted_radius);
        body->flag &= ~NURB_BODY_FAST_BEVEL_PREVIEW;
        const bool committed_surface_stage = data->surface_edge &&
                                             nurb_body_commit_surface_bevel_stage(*body);
        if (!committed_surface_stage) {
          /* Keep the accepted edge selected so the next evaluated mesh uses the same active
           * blend path as the modal preview. Clearing it here can rebuild the confirmed topology
           * differently from the previewed fillet/chamfer. */
          *target.hovered_edge = -1;
        }
        if (!data->body_edge && !data->surface_edge) {
          data->op->hovered_edge = -1;
          data->op->flag &= ~NURB_BODY_BOOLEAN_OP_HOVERED;
        }
        nurb_body_tag_geometry_changed(C, *ob, *body);
        object_nurb_body_bevel_finish(C, op);
        return OPERATOR_FINISHED;
      }
      break;
    case RIGHTMOUSE:
    case EVT_ESCKEY:
      if (event->val == KM_PRESS) {
        body->flag &= ~NURB_BODY_FAST_BEVEL_PREVIEW;
        *target.bevel_radius = data->start_radius;
        *target.bevel_type = data->start_bevel_type;
        *target.bevel_edge = data->start_bevel_edge;
        *target.bevel_edges = data->start_bevel_edges;
        *target.chamfer_edges = data->start_chamfer_edges;
        std::copy_n(data->start_bevel_radii, 64, target.bevel_radii);
        std::copy_n(data->start_bevel_order, 64, target.bevel_order);
        *target.bevel_order_next = data->start_bevel_order_next;
        *target.selected_edge = data->start_selected_edge;
        *target.selected_edges = data->start_selected_edges;
        if (!data->body_edge && !data->surface_edge) {
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

wmKeyMap *nurb_body_bevel_modal_keymap(wmKeyConfig *keyconf)
{
  static const EnumPropertyItem modal_items[] = {
      {NURB_BODY_BEV_MODAL_PRECISION_ON, "PRECISION_ON", 0, "Precision", ""},
      {NURB_BODY_BEV_MODAL_PRECISION_OFF, "PRECISION_OFF", 0, "Precision (OFF)", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  wmKeyMap *keymap = WM_modalkeymap_ensure(keyconf, "NURB Body Bevel Modal Map", modal_items);

  auto add_shift_precision_item = [&](const int event_type,
                                      const int event_value,
                                      const int modal_value) {
    KeyMapItem_Params params{};
    params.type = event_type;
    params.value = event_value;
    params.modifier = KM_ANY;
    params.direction = KM_ANY;
    WM_modalkeymap_add_item(keymap, &params, modal_value);
  };

  if (WM_modalkeymap_find_propvalue(keymap, NURB_BODY_BEV_MODAL_PRECISION_ON) == nullptr) {
    add_shift_precision_item(
        EVT_LEFTSHIFTKEY, KM_PRESS, NURB_BODY_BEV_MODAL_PRECISION_ON);
    add_shift_precision_item(
        EVT_RIGHTSHIFTKEY, KM_PRESS, NURB_BODY_BEV_MODAL_PRECISION_ON);
  }
  if (WM_modalkeymap_find_propvalue(keymap, NURB_BODY_BEV_MODAL_PRECISION_OFF) == nullptr) {
    add_shift_precision_item(
        EVT_LEFTSHIFTKEY, KM_RELEASE, NURB_BODY_BEV_MODAL_PRECISION_OFF);
    add_shift_precision_item(
        EVT_RIGHTSHIFTKEY, KM_RELEASE, NURB_BODY_BEV_MODAL_PRECISION_OFF);
  }

  WM_modalkeymap_assign(keymap, "OBJECT_OT_nurb_body_bevel_selected");

  return keymap;
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
  NurbBodyEdgeHit active_edge = nurb_body_active_edge(C, *body);
  if (!active_edge.is_valid()) {
    BKE_report(op->reports, RPT_WARNING, "Select one or more NURB Body edges before beveling");
    return OPERATOR_CANCELLED;
  }
  if (active_edge.surface_edge && active_edge.edge_key == 0) {
    BKE_report(op->reports,
               RPT_ERROR,
               "Cannot resolve the selected NURB Body edge for bevel");
    return OPERATOR_CANCELLED;
  }

  float anchor_screen[2] = {};
  bool found_anchor = false;
  if (region) {
    found_anchor =
        active_edge.body_edge ?
            nurb_body_edge_anchor_screen(*ob,
                                         active_edge.edge_index,
                                         NURB_BODY_EDGE_POLYLINE_BODY,
                                         *region,
                                         anchor_screen) :
        active_edge.surface_edge ?
            nurb_body_edge_anchor_screen(*ob,
                                         active_edge.edge_index,
                                         NURB_BODY_EDGE_POLYLINE_FINAL,
                                         *region,
                                         anchor_screen) :
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

  NurbBodyBevelTarget target = nurb_body_bevel_target_for_edge(*body, active_edge);
  const int start_bevel_type = nurb_body_edge_bevel_type(*target.bevel_type,
                                                         *target.bevel_edges,
                                                         *target.chamfer_edges,
                                                         *target.bevel_edge,
                                                         active_edge.edge_index);
  const int start_bevel_edge = *target.bevel_edge;
  const uint64_t start_bevel_edges = *target.bevel_edges;
  const uint64_t start_chamfer_edges = *target.chamfer_edges;
  const int start_bevel_order_next = *target.bevel_order_next;
  const int start_selected_edge = *target.selected_edge;
  const uint64_t start_selected_edges = *target.selected_edges;
  uint64_t active_edges = *target.selected_edges;
  if (active_edges == 0) {
    active_edges = nurb_body_edge_mask_for_index(active_edge.edge_index);
  }
  const int active_edge_count = std::max(nurb_body_edge_mask_count(active_edges), 1);
  const float start_radius = active_edge_count > 1 ?
                                 nurb_body_uniform_edge_bevel_radius(*target.bevel_radius,
                                                                     *target.bevel_edges,
                                                                     *target.bevel_edge,
                                                                     target.bevel_radii,
                                                                     active_edges,
                                                                     active_edge.edge_index) :
                                 nurb_body_edge_bevel_radius(*target.bevel_radius,
                                                             *target.bevel_edges,
                                                             *target.bevel_edge,
                                                             target.bevel_radii,
                                                             active_edge.edge_index);
  const bool had_explicit_bevel_edges = *target.bevel_edges != 0;
  float start_bevel_radii[64];
  std::copy_n(target.bevel_radii, 64, start_bevel_radii);
  int start_bevel_order[64];
  std::copy_n(target.bevel_order, 64, start_bevel_order);
  const uint64_t existing_bevel_edges = nurb_body_effective_bevel_edges(*target.bevel_radius,
                                                                       *target.bevel_edges,
                                                                       *target.bevel_edge,
                                                                       target.bevel_radii);
  const uint64_t target_edges = existing_bevel_edges | active_edges;
  const int profile = start_bevel_type;
  const float radius_limit = (active_edge.body_edge || active_edge.surface_edge) ?
                                 nurb_body_bevel_radius_limit(*body) :
                                 std::max(nurb_body_bevel_radius_limit(*body),
                                          nurb_body_boolean_op_scaled_bevel_radius_limit(
                                              *active_edge.op));
  const float max_preview_radius = nurb_body_modal_bevel_radius_limit(*ob,
                                                                      active_edge,
                                                                      radius_limit);
  const float active_edge_length = nurb_body_modal_bevel_edge_length(*ob, active_edge);
  const float max_chamfer_preview_radius = max_preview_radius;

  nurb_body_materialize_edge_bevel_radii(
      existing_bevel_edges, *target.bevel_radius, target.bevel_radii);
  if (!had_explicit_bevel_edges) {
    nurb_body_set_edge_chamfer_mask(existing_bevel_edges,
                                    *target.bevel_type == NURB_BODY_BEVEL_CHAMFER,
                                    *target.chamfer_edges);
  }
  nurb_body_assign_edge_bevel_orders(
      existing_bevel_edges, target.bevel_order, *target.bevel_order_next);
  nurb_body_assign_edge_bevel_orders(active_edges, target.bevel_order, *target.bevel_order_next);
  nurb_body_promote_edge_bevel_orders(active_edges, target.bevel_order, *target.bevel_order_next);
  *target.bevel_edges = target_edges;
  nurb_body_clear_unused_edge_bevel_orders(*target.bevel_edges, target.bevel_order);
  *target.bevel_edge = active_edge.edge_index;
  *target.bevel_type = profile;
  nurb_body_set_uniform_edge_bevel_radii(active_edges, start_radius, target.bevel_radii);
  nurb_body_set_edge_chamfer_mask(
      active_edges, profile == NURB_BODY_BEVEL_CHAMFER, *target.chamfer_edges);

  NurbBodyBevelData *data = MEM_new<NurbBodyBevelData>(__func__);
  data->op = active_edge.op;
  data->edge_index = active_edge.edge_index;
  data->body_edge = active_edge.body_edge;
  data->surface_edge = active_edge.surface_edge;
  data->start_radius = start_radius;
  data->start_bevel_type = start_bevel_type;
  data->start_bevel_edge = start_bevel_edge;
  data->start_bevel_edges = start_bevel_edges;
  data->start_chamfer_edges = start_chamfer_edges;
  std::copy_n(start_bevel_radii, 64, data->start_bevel_radii);
  std::copy_n(start_bevel_order, 64, data->start_bevel_order);
  data->start_bevel_order_next = start_bevel_order_next;
  data->start_selected_edge = start_selected_edge;
  data->start_selected_edges = start_selected_edges;
  data->edge_mask = target_edges;
  data->edit_edge_mask = active_edges;
  data->active_edge_count = active_edge_count;
  data->start_mouse_x = event->mval[0];
  data->start_mouse_y = event->mval[1];
  const float drag_scale = std::clamp(active_edge_length > 0.0f ?
                                          active_edge_length * 0.85f :
                                          max_preview_radius * 0.90f,
                                      0.001f,
                                      max_preview_radius);
  data->radius_step = std::max(drag_scale / 85.0f, 0.00012f);
  data->preview_radius_step = std::max(data->radius_step * 2.0f, max_preview_radius / 900.0f);
  data->max_preview_radius = max_preview_radius;
  data->max_chamfer_preview_radius = max_chamfer_preview_radius;
  data->last_preview_radius = -1.0f;
  body->flag |= NURB_BODY_FAST_BEVEL_PREVIEW;

  if (region) {
    if (found_anchor) {
      copy_v2_v2(data->mcenter, anchor_screen);
    }
    else {
      data->mcenter[0] = float(event->mval[0]);
      data->mcenter[1] = float(event->mval[1]);
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
  NurbBodyEdgeHit active_edge = nurb_body_active_edge(C, *body);
  if (!active_edge.is_valid()) {
    BKE_report(op->reports, RPT_WARNING, "Select one or more NURB Body edges before beveling");
    return OPERATOR_CANCELLED;
  }
  if (active_edge.surface_edge && active_edge.edge_key == 0) {
    BKE_report(op->reports,
               RPT_ERROR,
               "Cannot resolve the selected NURB Body edge for bevel");
    return OPERATOR_CANCELLED;
  }

  NurbBodyBevelTarget target = nurb_body_bevel_target_for_edge(*body, active_edge);
  uint64_t active_edges = *target.selected_edges;
  if (active_edges == 0) {
    active_edges = nurb_body_edge_mask_for_index(active_edge.edge_index);
  }
  const bool had_explicit_bevel_edges = *target.bevel_edges != 0;
  const uint64_t existing_bevel_edges = nurb_body_effective_bevel_edges(*target.bevel_radius,
                                                                       *target.bevel_edges,
                                                                       *target.bevel_edge,
                                                                       target.bevel_radii);
  const uint64_t target_edges = existing_bevel_edges | active_edges;
  const float radius_limit = (active_edge.body_edge || active_edge.surface_edge) ?
                                 nurb_body_bevel_radius_limit(*body) :
                                 std::max(nurb_body_bevel_radius_limit(*body),
                                          nurb_body_boolean_op_scaled_bevel_radius_limit(
                                              *active_edge.op));
  const float radius = std::min(std::max(0.0f, RNA_float_get(op->ptr, "radius")),
                                nurb_body_modal_bevel_radius_limit(*ob,
                                                                   active_edge,
                                                                   radius_limit));
  const int profile = RNA_enum_get(op->ptr, "profile");

  nurb_body_materialize_edge_bevel_radii(
      existing_bevel_edges, *target.bevel_radius, target.bevel_radii);
  if (!had_explicit_bevel_edges) {
    nurb_body_set_edge_chamfer_mask(existing_bevel_edges,
                                    *target.bevel_type == NURB_BODY_BEVEL_CHAMFER,
                                    *target.chamfer_edges);
  }
  nurb_body_assign_edge_bevel_orders(
      existing_bevel_edges, target.bevel_order, *target.bevel_order_next);
  nurb_body_assign_edge_bevel_orders(active_edges, target.bevel_order, *target.bevel_order_next);
  nurb_body_promote_edge_bevel_orders(active_edges, target.bevel_order, *target.bevel_order_next);
  *target.bevel_edges = target_edges;
  nurb_body_clear_unused_edge_bevel_orders(*target.bevel_edges, target.bevel_order);
  *target.bevel_edge = active_edge.edge_index;
  *target.bevel_type = profile;
  *target.bevel_radius = radius;
  nurb_body_set_uniform_edge_bevel_radii(active_edges, radius, target.bevel_radii);
  nurb_body_set_edge_chamfer_mask(
      active_edges, profile == NURB_BODY_BEVEL_CHAMFER, *target.chamfer_edges);
  const bool committed_surface_stage = active_edge.surface_edge &&
                                       nurb_body_commit_surface_bevel_stage(*body);
  if (!committed_surface_stage) {
    *target.hovered_edge = -1;
  }
  if (!active_edge.body_edge && !active_edge.surface_edge) {
    active_edge.op->hovered_edge = -1;
    active_edge.op->flag &= ~NURB_BODY_BOOLEAN_OP_HOVERED;
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
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_GRAB_CURSOR_XY | OPTYPE_BLOCKING;

  RNA_def_float(ot->srna, "radius", 0.08f, 0.0f, 100000.0f, "Radius", "", 0.0f, 10.0f);
  RNA_def_enum(ot->srna,
               "profile",
               nurb_body_bevel_type_items,
               NURB_BODY_BEVEL_FILLET,
               "Profile",
               "Selected edge bevel profile");
}

}  // namespace blender::ed::object
