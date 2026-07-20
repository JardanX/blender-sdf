/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edobj
 */

#include "MEM_guardedalloc.h"

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"
#include "DNA_space_types.h"
#include "DNA_vfont_types.h"

#include "BLI_listbase.h"
#include "BLI_math_base.h"
#include "BLI_string.h"
#include "BLI_color.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include <cfloat>
#include <cstdio>
#include <string>

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "BKE_context.hh"
#include "BKE_customdata.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_report.hh"
#include "BKE_sdf.hh"
#include "BKE_sdf_text.hh"
#include "BKE_vfont.hh"

#include "BLT_translation.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "BKE_screen.hh"

#include "ED_object.hh"
#include "ED_outliner.hh"
#include "ED_screen.hh"
#include "ED_select_utils.hh"
#include "ED_space_api.hh"
#include "ED_view3d.hh"

#include "GPU_immediate.hh"
#include "GPU_matrix.hh"
#include "GPU_state.hh"

#include "UI_interface.hh"

#include "BKE_attribute.h"
#include "BKE_attribute.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"

#include "BLI_index_mask.hh"
#include "GEO_mesh_merge_by_distance.hh"

#include "object_intern.hh"

#include "sdf_meshing.hh"

#include "BLI_fileops.h"
#include "BLI_path_utils.hh"

#include "BKE_appdir.hh"

namespace blender::draw::sdf {
void sdf_profile_request();
bool sdf_profile_is_ready();
bool sdf_profile_is_pending();
std::string sdf_profile_format_text();
}  // namespace blender::draw::sdf

namespace blender::ed::object {

/* SDF Add */

static const EnumPropertyItem sdf_add_type_items[] = {
    {SDF_TYPE_BOX, "BOX", ICON_SDF_CUBE, "Cube", "Cube SDF primitive"},
    {SDF_TYPE_SPHERE, "SPHERE", ICON_SDF_SPHERE, "Sphere", "Sphere SDF primitive"},
    {SDF_TYPE_CYLINDER,
     "CYLINDER",
     ICON_SDF_CYLINDER,
     "Cylinder",
     "Cylinder SDF primitive"},
    {SDF_TYPE_CONE, "CONE", ICON_SDF_CONE, "Cone", "Cone SDF primitive"},
    {SDF_TYPE_CAPSULE, "CAPSULE", ICON_SDF_CAPSULE, "Capsule", "Capsule SDF primitive"},
    {SDF_TYPE_TORUS, "TORUS", ICON_SDF_TORUS, "Torus", "Torus SDF primitive"},
    {SDF_TYPE_NGON, "NGON", ICON_SDF_NGON, "N-Gon", "Regular polygon prism"},
    {SDF_TYPE_POLYGON, "POLYGON", ICON_SDF_POLYGON, "Polygon", "Arbitrary polygon prism"},
    {SDF_TYPE_GROUP, "GROUP", ICON_DOT, "Group", "SDF group container"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const char *sdf_type_name(int type)
{
  switch (type) {
    case SDF_TYPE_BOX:
      return "SDF Cube";
    case SDF_TYPE_SPHERE:
      return "SDF Sphere";
    case SDF_TYPE_CYLINDER:
      return "SDF Cylinder";
    case SDF_TYPE_CONE:
      return "SDF Cone";
    case SDF_TYPE_CAPSULE:
      return "SDF Capsule";
    case SDF_TYPE_TORUS:
      return "SDF Torus";
    case SDF_TYPE_NGON:
      return "SDF N-Gon";
    case SDF_TYPE_POLYGON:
      return "SDF Polygon";
    default:
      return "SDF";
  }
}

static Object *object_sdf_add(bContext *C, wmOperator *op, const char *name)
{
  ushort local_view_bits;
  float loc[3], rot[3];

  add_generic_get_opts(C, op, 'Z', loc, rot, nullptr, nullptr, &local_view_bits, nullptr);

  const int type = RNA_enum_get(op->ptr, "type");
  if (!name) {
    name = sdf_type_name(type);
  }

  Main *bmain = CTX_data_main(C);

  Object *ob = add_type(C, OB_SDF, name, loc, rot, false, local_view_bits);
  if (ob && ob->data) {
    SDF *sdf_data = id_cast<SDF *>(ob->data);
    sdf_data->sdf_type = type;
    sdf_data->sdf_index = BKE_sdf_next_index(bmain);

    switch (type) {
      case SDF_TYPE_CAPSULE:
        sdf_data->size[0] = 0.5f;
        sdf_data->size[1] = 1.0f;
        sdf_data->size[2] = 0.5f;
        break;
      case SDF_TYPE_CONE:
        /* size = {bottom radius, half-height, top radius}; 0 top = sharp apex. */
        sdf_data->size[0] = 1.0f;
        sdf_data->size[1] = 1.0f;
        sdf_data->size[2] = 0.0f;
        break;
      case SDF_TYPE_TORUS:
        sdf_data->size[0] = 0.8f;
        sdf_data->size[1] = 0.25f;
        sdf_data->size[2] = 0.8f;
        break;
      case SDF_TYPE_NGON:
        sdf_data->size[0] = 1.0f;
        sdf_data->size[1] = 1.0f;
        sdf_data->size[2] = 1.0f;
        break;
      case SDF_TYPE_POLYGON:
        sdf_data->size[0] = 1.0f;
        sdf_data->size[1] = 1.0f;
        sdf_data->size[2] = 1.0f;
        BKE_sdf_polygon_init_triangle(sdf_data);
        break;
      default:
        break;
    }

  }
  return ob;
}

static wmOperatorStatus object_sdf_add_exec(bContext *C, wmOperator *op)
{
  return (object_sdf_add(C, op, nullptr) != nullptr) ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
}

void OBJECT_OT_sdf_add(wmOperatorType *ot)
{
  ot->name = "Add SDF";
  ot->description = "Add an SDF object to the scene";
  ot->idname = "OBJECT_OT_sdf_add";
  ot->exec = object_sdf_add_exec;
  ot->poll = ED_operator_objectmode;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  add_generic_props(ot, false);

  RNA_def_enum(ot->srna,
               "type",
               sdf_add_type_items,
               SDF_TYPE_BOX,
               "Type",
               "SDF primitive type");
}

/* SDF Text Add (kept separate from the primitives: it is a text object with
 * SDF rendering, edited by typing like a regular text object). */

static wmOperatorStatus object_sdf_text_add_exec(bContext *C, wmOperator *op)
{
  ushort local_view_bits;
  float loc[3], rot[3];

  add_generic_get_opts(C, op, 'Z', loc, rot, nullptr, nullptr, &local_view_bits, nullptr);

  Main *bmain = CTX_data_main(C);
  Object *ob = add_type(C, OB_SDF, "SDF Text", loc, rot, false, local_view_bits);
  if (ob && ob->data) {
    SDF *sdf_data = id_cast<SDF *>(ob->data);
    sdf_data->sdf_type = SDF_TYPE_TEXT;
    sdf_data->sdf_index = BKE_sdf_next_index(bmain);
    /* size[2] is the extrusion half-depth. */
    sdf_data->size[0] = 1.0f;
    sdf_data->size[1] = 1.0f;
    sdf_data->size[2] = 0.1f;
    BKE_sdf_text_set(sdf_data, "Text");
    sdf_data->text_font = BKE_vfont_builtin_ensure();
    id_us_plus(&sdf_data->text_font->id);
  }
  return (ob != nullptr) ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
}

void OBJECT_OT_sdf_text_add(wmOperatorType *ot)
{
  ot->name = "Add SDF Text";
  ot->description = "Add an SDF text object to the scene";
  ot->idname = "OBJECT_OT_sdf_text_add";
  ot->exec = object_sdf_text_add_exec;
  ot->poll = ED_operator_objectmode;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  add_generic_props(ot, false);
}

/* SDF CSG / Blend Cycle Operators */

static bool object_has_sdf_settings(const Object *ob)
{
  return ob && ((ob->type == OB_SDF && ob->data) || BKE_sdf_object_is_enabled(*ob));
}

static ID *object_sdf_settings_id(Object *ob)
{
  return ob->type == OB_SDF ? static_cast<ID *>(ob->data) : &ob->id;
}

static int object_sdf_csg_operation_get(const Object *ob)
{
  return ob->type == OB_SDF ? id_cast<const SDF *>(ob->data)->csg_operation :
                              ob->sdf_csg_operation;
}

static void object_sdf_csg_operation_set(Object *ob, const int value)
{
  if (ob->type == OB_SDF) {
    id_cast<SDF *>(ob->data)->csg_operation = value;
  }
  else {
    ob->sdf_csg_operation = value;
  }
}

static void object_sdf_blend_type_set(Object *ob, const int value)
{
  if (ob->type == OB_SDF) {
    id_cast<SDF *>(ob->data)->blend_type = value;
  }
  else {
    ob->sdf_blend_type = value;
  }
}

static float object_sdf_blend_get(const Object *ob)
{
  return ob->type == OB_SDF ? id_cast<const SDF *>(ob->data)->blend : ob->sdf_blend;
}

static float object_sdf_color_blend_get(const Object *ob)
{
  return ob->type == OB_SDF ? id_cast<const SDF *>(ob->data)->color_blend : ob->sdf_color_blend;
}

static float object_sdf_shell_distance_get(const Object *ob)
{
  return ob->type == OB_SDF ? id_cast<const SDF *>(ob->data)->shell_distance :
                              ob->sdf_shell_distance;
}

static float object_sdf_shell_top_get(const Object *ob)
{
  return ob->type == OB_SDF ? id_cast<const SDF *>(ob->data)->shell_blend_top :
                              ob->sdf_shell_blend_top;
}

static float object_sdf_shell_bottom_get(const Object *ob)
{
  return ob->type == OB_SDF ? id_cast<const SDF *>(ob->data)->shell_blend_bottom :
                              ob->sdf_shell_blend_bottom;
}

static int object_sdf_shell_mode_get(const Object *ob)
{
  return ob->type == OB_SDF ? id_cast<const SDF *>(ob->data)->shell_mode : ob->sdf_shell_mode;
}

static void object_sdf_tag_update(bContext *C, Object *ob)
{
  DEG_id_tag_update(object_sdf_settings_id(ob),
                    ob->type == OB_SDF ? ID_RECALC_GEOMETRY : ID_RECALC_SYNC_TO_EVAL);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
}

static wmOperatorStatus object_sdf_set_csg_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  char ob_name[MAX_ID_NAME - 2];
  RNA_string_get(op->ptr, "object_name", ob_name);
  const int csg_op = RNA_int_get(op->ptr, "csg_operation");

  Object *ob = (Object *)BKE_libblock_find_name(bmain, ID_OB, ob_name);
  if (!object_has_sdf_settings(ob)) {
    return OPERATOR_CANCELLED;
  }
  object_sdf_csg_operation_set(ob, csg_op);

  object_sdf_tag_update(C, ob);
  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_set_csg(wmOperatorType *ot)
{
  ot->name = "Set SDF CSG Operation";
  ot->description = "Set the CSG operation for an SDF object";
  ot->idname = "OBJECT_OT_sdf_set_csg";
  ot->exec = object_sdf_set_csg_exec;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna, "object_name", nullptr, MAX_ID_NAME - 2, "Object Name", "");
  RNA_def_int(ot->srna, "csg_operation", 0, 0, 10, "CSG Operation", "", 0, 10);
}

static wmOperatorStatus object_sdf_set_blend_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  char ob_name[MAX_ID_NAME - 2];
  RNA_string_get(op->ptr, "object_name", ob_name);
  const int blend_type = RNA_int_get(op->ptr, "blend_type");

  Object *ob = (Object *)BKE_libblock_find_name(bmain, ID_OB, ob_name);
  if (!object_has_sdf_settings(ob)) {
    return OPERATOR_CANCELLED;
  }
  object_sdf_blend_type_set(ob, blend_type);

  object_sdf_tag_update(C, ob);
  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_set_blend(wmOperatorType *ot)
{
  ot->name = "Set SDF Blend Type";
  ot->description = "Set the blend type for an SDF object";
  ot->idname = "OBJECT_OT_sdf_set_blend";
  ot->exec = object_sdf_set_blend_exec;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna, "object_name", nullptr, MAX_ID_NAME - 2, "Object Name", "");
  RNA_def_int(ot->srna, "blend_type", 0, 0, 10, "Blend Type", "", 0, 10);
}

/* SDF Blend / Distance Adjust (Modal) */

struct SDFValueAdjustData {
  float init_mval_x;
  float init_mval_y;
  float init_value;
  float current_value;
  bool slow_mode;
  float slow_mval_x;
  float slow_value;
  void *draw_handle;
  ARegion *region;
  float mval_x;
  float mval_y;
  Object *ob;
  float line_r, line_g, line_b;
  const char *label;
  /* Direction detection for shell blend */
  bool locked;
  bool is_horizontal;
  float init_other;
  float init_other2;
};

static void sdf_value_adjust_draw(const bContext * /*C*/, ARegion *region, void *userdata)
{
  SDFValueAdjustData *data = static_cast<SDFValueAdjustData *>(userdata);
  if (!data->ob || !data->locked) {
    return;
  }

  float obj_screen[2];
  float3 obj_world = float3(data->ob->object_to_world()[3]);
  if (ED_view3d_project_float_global(region, obj_world, obj_screen, V3D_PROJ_TEST_NOP) !=
      V3D_PROJ_RET_OK)
  {
    return;
  }

  GPU_blend(GPU_BLEND_ALPHA);
  GPU_line_smooth(true);
  GPU_line_width(2.0f);

  uint pos = GPU_vertformat_attr_add_legacy(
      immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  immUniformColor4f(data->line_r, data->line_g, data->line_b, 0.85f);
  immBegin(GPU_PRIM_LINES, 2);
  immVertex2f(pos, obj_screen[0], obj_screen[1]);
  immVertex2f(pos, data->mval_x, data->mval_y);
  immEnd();

  GPU_point_size(7.0f);
  immBegin(GPU_PRIM_POINTS, 1);
  immVertex2f(pos, data->mval_x, data->mval_y);
  immEnd();

  GPU_point_size(5.0f);
  immUniformColor4f(1.0f, 1.0f, 1.0f, 0.9f);
  immBegin(GPU_PRIM_POINTS, 1);
  immVertex2f(pos, obj_screen[0], obj_screen[1]);
  immEnd();

  immUnbindProgram();
  GPU_line_smooth(false);
  GPU_blend(GPU_BLEND_NONE);
}

static void sdf_draw_cleanup(ARegion *region, void *draw_handle, bContext *C)
{
  ED_region_draw_cb_exit(region->runtime->type, draw_handle);
  ED_area_status_text(CTX_wm_area(C), nullptr);
  ED_workspace_status_text(C, nullptr);
  ED_region_tag_redraw(region);
}

/* Polls */

static bool sdf_blend_adjust_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  return object_has_sdf_settings(ob) && ob->mode == OB_MODE_OBJECT && CTX_wm_region_view3d(C);
}

static bool sdf_shell_distance_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  if (!object_has_sdf_settings(ob) || ob->mode != OB_MODE_OBJECT || !CTX_wm_region_view3d(C)) {
    return false;
  }
  return object_sdf_csg_operation_get(ob) == SDF_CSG_SHELL;
}

static void sdf_value_update_header(bContext *C, SDFValueAdjustData *data)
{
  WorkspaceStatus status(C);
  status.item(IFACE_("Confirm"), ICON_EVENT_RETURN, ICON_MOUSE_LMB);
  status.item(IFACE_("Cancel"), ICON_EVENT_ESC, ICON_MOUSE_RMB);
  status.item_bool(IFACE_("Precision"), data->slow_mode, ICON_EVENT_SHIFT);
  status.item(IFACE_("Adjust"), ICON_MOUSE_MOVE);

  char msg[128];
  SNPRINTF(msg, "%s: %.3f", data->label, data->current_value);
  ED_area_status_text(CTX_wm_area(C), msg);
}

/* Shared single-value modal (locked direction, X-axis controls value) */

static wmOperatorStatus sdf_value_modal_common(bContext *C,
                                                  wmOperator *op,
                                                  const wmEvent *event,
                                                  void (*write_fn)(Object *, float),
                                                  void (*restore_fn)(Object *, float))
{
  SDFValueAdjustData *data = static_cast<SDFValueAdjustData *>(op->customdata);
  Object *ob = CTX_data_active_object(C);
  if (!object_has_sdf_settings(ob)) {
    sdf_draw_cleanup(data->region, data->draw_handle, C);
    MEM_delete(data);
    op->customdata = nullptr;
    return OPERATOR_CANCELLED;
  }
  if ((event->type == EVT_ESCKEY && event->val == KM_PRESS) ||
      (event->type == RIGHTMOUSE && event->val == KM_PRESS))
  {
    restore_fn(ob, data->init_value);
    object_sdf_tag_update(C, ob);
    sdf_draw_cleanup(data->region, data->draw_handle, C);
    MEM_delete(data);
    return OPERATOR_CANCELLED;
  }

  if ((event->type == LEFTMOUSE && event->val == KM_RELEASE) ||
      (event->type == EVT_RETKEY && event->val == KM_PRESS) ||
      (event->type == EVT_PADENTER && event->val == KM_PRESS))
  {
    write_fn(ob, data->current_value);
    object_sdf_tag_update(C, ob);
    sdf_draw_cleanup(data->region, data->draw_handle, C);
    MEM_delete(data);
    return OPERATOR_FINISHED;
  }

  float mx = float(event->mval[0]);
  float sensitivity = 0.01f;

  if (event->type == EVT_LEFTSHIFTKEY && event->val == KM_PRESS) {
    data->slow_mode = true;
    data->slow_mval_x = mx;
    data->slow_value = data->current_value;
  }
  if (event->type == EVT_LEFTSHIFTKEY && event->val == KM_RELEASE) {
    data->slow_mode = false;
  }
  if (data->slow_mode) {
    float dx = (mx - data->slow_mval_x) * sensitivity * 0.1f;
    data->current_value = max_ff(data->slow_value + dx, 0.0f);
  }
  else {
    float dx = (mx - data->init_mval_x) * sensitivity;
    data->current_value = max_ff(data->init_value + dx, 0.0f);
  }
  data->mval_x = mx;
  data->mval_y = float(event->mval[1]);

  write_fn(ob, data->current_value);
  object_sdf_tag_update(C, ob);
  ED_region_tag_redraw(data->region);

  sdf_value_update_header(C, data);
  return OPERATOR_RUNNING_MODAL;
}

static SDFValueAdjustData *sdf_value_init(bContext *C,
                                            const wmEvent *event,
                                            Object *ob,
                                            float init_value,
                                            float r, float g, float b,
                                            const char *label)
{
  ARegion *region = CTX_wm_region(C);
  SDFValueAdjustData *data = MEM_new<SDFValueAdjustData>(__func__);
  data->init_mval_x = float(event->mval[0]);
  data->init_mval_y = float(event->mval[1]);
  data->mval_x = float(event->mval[0]);
  data->mval_y = float(event->mval[1]);
  data->init_value = init_value;
  data->current_value = init_value;
  data->slow_mode = false;
  data->locked = true;
  data->is_horizontal = true;
  data->init_other = 0.0f;
  data->init_other2 = 0.0f;
  data->ob = ob;
  data->region = region;
  data->line_r = r;
  data->line_g = g;
  data->line_b = b;
  data->label = label;
  data->draw_handle = ED_region_draw_cb_activate(
      region->runtime->type, sdf_value_adjust_draw, data, REGION_DRAW_POST_PIXEL);
  return data;
}

/* B key: blend, color blend for Paint, or shell edges. */

static void sdf_write_blend(Object *ob, float v)
{
  if (ob->type == OB_SDF) {
    id_cast<SDF *>(ob->data)->blend = v;
  }
  else {
    ob->sdf_blend = v;
  }
}

static void sdf_write_color_blend(Object *ob, float v)
{
  if (ob->type == OB_SDF) {
    id_cast<SDF *>(ob->data)->color_blend = v;
  }
  else {
    ob->sdf_color_blend = v;
  }
}

static void sdf_write_shell_top(Object *ob, float v)
{
  if (object_sdf_shell_mode_get(ob) == SDF_SHELL_NORMAL) {
    v = min_ff(v, object_sdf_shell_distance_get(ob));
  }
  if (ob->type == OB_SDF) {
    id_cast<SDF *>(ob->data)->shell_blend_top = v;
  }
  else {
    ob->sdf_shell_blend_top = v;
  }
}

static void sdf_write_shell_bottom(Object *ob, float v)
{
  if (object_sdf_shell_mode_get(ob) == SDF_SHELL_NORMAL) {
    v = min_ff(v, object_sdf_shell_distance_get(ob));
  }
  if (ob->type == OB_SDF) {
    id_cast<SDF *>(ob->data)->shell_blend_bottom = v;
  }
  else {
    ob->sdf_shell_blend_bottom = v;
  }
}

static wmOperatorStatus sdf_blend_adjust_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  SDFValueAdjustData *data = static_cast<SDFValueAdjustData *>(op->customdata);
  Object *ob = CTX_data_active_object(C);
  if (!object_has_sdf_settings(ob)) {
    sdf_draw_cleanup(data->region, data->draw_handle, C);
    MEM_delete(data);
    op->customdata = nullptr;
    return OPERATOR_CANCELLED;
  }
  bool is_shell = object_sdf_csg_operation_get(ob) == SDF_CSG_SHELL;
  bool is_paint = object_sdf_csg_operation_get(ob) == SDF_CSG_PAINT;

  if (is_paint) {
    return sdf_value_modal_common(C, op, event, sdf_write_color_blend, sdf_write_color_blend);
  }
  if (!is_shell) {
    return sdf_value_modal_common(C, op, event, sdf_write_blend, sdf_write_blend);
  }

  /* Shell: direction detection */
  if (!data->locked) {
    float dx = fabsf(float(event->mval[0]) - data->init_mval_x);
    float dy = fabsf(float(event->mval[1]) - data->init_mval_y);
    constexpr float threshold = 8.0f;

    if (dx >= threshold || dy >= threshold) {
      data->locked = true;
      if (dx >= dy) {
        data->is_horizontal = true;
        data->init_value = object_sdf_shell_top_get(ob);
        data->current_value = object_sdf_shell_top_get(ob);
        data->init_other = object_sdf_shell_bottom_get(ob);
        data->label = "Shell Top";
        data->line_r = 1.0f; data->line_g = 0.3f; data->line_b = 0.3f;
      }
      else {
        data->is_horizontal = false;
        data->init_value = object_sdf_shell_bottom_get(ob);
        data->current_value = object_sdf_shell_bottom_get(ob);
        data->init_other = object_sdf_shell_top_get(ob);
        data->label = "Shell Bottom";
        data->line_r = 0.3f; data->line_g = 0.6f; data->line_b = 1.0f;
      }
      data->init_mval_x = float(event->mval[0]);
      data->init_mval_y = float(event->mval[1]);
    }

    data->mval_x = float(event->mval[0]);
    data->mval_y = float(event->mval[1]);
    ED_region_tag_redraw(data->region);

    if ((event->type == EVT_ESCKEY && event->val == KM_PRESS) ||
        (event->type == RIGHTMOUSE && event->val == KM_PRESS))
    {
      sdf_draw_cleanup(data->region, data->draw_handle, C);
      MEM_delete(data);
      return OPERATOR_CANCELLED;
    }

    sdf_value_update_header(C, data);
    return OPERATOR_RUNNING_MODAL;
  }

  /* Locked: use shared logic */
  if (data->is_horizontal) {
    return sdf_value_modal_common(C, op, event, sdf_write_shell_top, sdf_write_shell_top);
  }
  return sdf_value_modal_common(C, op, event, sdf_write_shell_bottom, sdf_write_shell_bottom);
}

static wmOperatorStatus sdf_blend_adjust_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  Object *ob = CTX_data_active_object(C);
  bool is_shell = object_sdf_csg_operation_get(ob) == SDF_CSG_SHELL;
  bool is_paint = object_sdf_csg_operation_get(ob) == SDF_CSG_PAINT;

  SDFValueAdjustData *data;
  if (is_shell) {
    data = sdf_value_init(C, event, ob, 0.0f, 1.0f, 1.0f, 1.0f, "Move to select");
    data->locked = false;
  }
  else if (is_paint) {
    data = sdf_value_init(
        C, event, ob, object_sdf_color_blend_get(ob), 0.3f, 0.7f, 1.0f, "Color Blend");
  }
  else {
    data = sdf_value_init(
        C, event, ob, object_sdf_blend_get(ob), 1.0f, 0.65f, 0.0f, "Blend");
  }

  op->customdata = data;
  WM_event_add_modal_handler(C, op);
  sdf_value_update_header(C, data);
  return OPERATOR_RUNNING_MODAL;
}

static void sdf_blend_adjust_cancel(bContext *C, wmOperator *op)
{
  SDFValueAdjustData *data = static_cast<SDFValueAdjustData *>(op->customdata);
  Object *ob = CTX_data_active_object(C);
  if (object_has_sdf_settings(ob)) {
    if (data->locked) {
      if (object_sdf_csg_operation_get(ob) == SDF_CSG_SHELL) {
        if (data->is_horizontal) {
          sdf_write_shell_top(ob, data->init_value);
        }
        else {
          sdf_write_shell_bottom(ob, data->init_value);
        }
      }
      else if (object_sdf_csg_operation_get(ob) == SDF_CSG_PAINT) {
        sdf_write_color_blend(ob, data->init_value);
      }
      else {
        sdf_write_blend(ob, data->init_value);
      }
      object_sdf_tag_update(C, ob);
    }
  }
  sdf_draw_cleanup(data->region, data->draw_handle, C);
  MEM_delete(data);
}

void OBJECT_OT_sdf_blend_adjust(wmOperatorType *ot)
{
  ot->name = "Adjust SDF Blend";
  ot->description = "Adjust blend, Paint color blend, or shell edges";
  ot->idname = "OBJECT_OT_sdf_blend_adjust";

  ot->invoke = sdf_blend_adjust_invoke;
  ot->modal = sdf_blend_adjust_modal;
  ot->cancel = sdf_blend_adjust_cancel;
  ot->poll = sdf_blend_adjust_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;
}

/* D key: shell_distance (only for shell objects) */

static void sdf_write_distance(Object *ob, float v)
{
  if (ob->type == OB_SDF) {
    SDF *sdf = id_cast<SDF *>(ob->data);
    sdf->shell_distance = v;
    if (sdf->shell_mode == SDF_SHELL_NORMAL) {
      sdf->shell_blend_top = min_ff(sdf->shell_blend_top, v);
      sdf->shell_blend_bottom = min_ff(sdf->shell_blend_bottom, v);
    }
  }
  else {
    ob->sdf_shell_distance = v;
    if (ob->sdf_shell_mode == SDF_SHELL_NORMAL) {
      ob->sdf_shell_blend_top = min_ff(ob->sdf_shell_blend_top, v);
      ob->sdf_shell_blend_bottom = min_ff(ob->sdf_shell_blend_bottom, v);
    }
  }
}

static wmOperatorStatus sdf_shell_distance_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  if ((event->type == EVT_ESCKEY && event->val == KM_PRESS) ||
      (event->type == RIGHTMOUSE && event->val == KM_PRESS))
  {
    SDFValueAdjustData *data = static_cast<SDFValueAdjustData *>(op->customdata);
    Object *ob = CTX_data_active_object(C);
    if (object_has_sdf_settings(ob)) {
      sdf_write_distance(ob, data->init_value);
      sdf_write_shell_top(ob, data->init_other);
      sdf_write_shell_bottom(ob, data->init_other2);
      object_sdf_tag_update(C, ob);
    }
    sdf_draw_cleanup(data->region, data->draw_handle, C);
    MEM_delete(data);
    op->customdata = nullptr;
    return OPERATOR_CANCELLED;
  }
  return sdf_value_modal_common(C, op, event, sdf_write_distance, sdf_write_distance);
}

static wmOperatorStatus sdf_shell_distance_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  Object *ob = CTX_data_active_object(C);

  SDFValueAdjustData *data = sdf_value_init(C,
                                            event,
                                            ob,
                                            object_sdf_shell_distance_get(ob),
                                            0.3f,
                                            1.0f,
                                            0.3f,
                                            "Distance");
  data->init_other = object_sdf_shell_top_get(ob);
  data->init_other2 = object_sdf_shell_bottom_get(ob);
  op->customdata = data;

  WM_event_add_modal_handler(C, op);
  sdf_value_update_header(C, data);
  return OPERATOR_RUNNING_MODAL;
}

static void sdf_shell_distance_cancel(bContext *C, wmOperator *op)
{
  SDFValueAdjustData *data = static_cast<SDFValueAdjustData *>(op->customdata);
  Object *ob = CTX_data_active_object(C);
  if (object_has_sdf_settings(ob)) {
    sdf_write_distance(ob, data->init_value);
    sdf_write_shell_top(ob, data->init_other);
    sdf_write_shell_bottom(ob, data->init_other2);
    object_sdf_tag_update(C, ob);
  }
  sdf_draw_cleanup(data->region, data->draw_handle, C);
  MEM_delete(data);
}

void OBJECT_OT_sdf_shell_distance_adjust(wmOperatorType *ot)
{
  ot->name = "Adjust Shell Distance";
  ot->description = "Interactively adjust shell distance by moving the mouse";
  ot->idname = "OBJECT_OT_sdf_shell_distance_adjust";

  ot->invoke = sdf_shell_distance_invoke;
  ot->modal = sdf_shell_distance_modal;
  ot->cancel = sdf_shell_distance_cancel;
  ot->poll = sdf_shell_distance_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;
}

/* SDF to Mesh (Dual Contouring) */

static wmOperatorStatus object_sdf_to_mesh_exec(bContext *C, wmOperator *op)
{
  const int grid_res = RNA_int_get(op->ptr, "resolution");

  Vector<float3> positions;
  Vector<float3> normals;
  Vector<int3> tris;
  Vector<float4> colors;
  int vert_count = 0, tri_count = 0;

  std::string err = draw::sdf::sdf_dual_contour_to_mesh(
      grid_res, positions, normals, tris, colors, &vert_count, &tri_count);

  if (!err.empty()) {
    BKE_reportf(op->reports, RPT_ERROR, "SDF to Mesh: %s", err.c_str());
    return OPERATOR_CANCELLED;
  }

  const float cell_size = 1.0f / float(grid_res);

  Mesh *mesh_raw = BKE_mesh_new_nomain(vert_count, 0, tri_count, tri_count * 3);
  mesh_raw->vert_positions_for_write().copy_from(positions.as_span());

  MutableSpan<int> offsets = mesh_raw->face_offsets_for_write();
  for (int i = 0; i <= tri_count; i++) {
    offsets[i] = i * 3;
  }

  MutableSpan<int> cverts = mesh_raw->corner_verts_for_write();
  for (int i = 0; i < tri_count; i++) {
    cverts[i * 3 + 0] = tris[i].x;
    cverts[i * 3 + 1] = tris[i].z;
    cverts[i * 3 + 2] = tris[i].y;
  }

  /* Add vertex colors before merge so they get interpolated */
  if (colors.size() >= vert_count) {
    const char *color_name = "SDF Color";
    if (mesh_raw->attributes_for_write().add(
            color_name,
            bke::AttrDomain::Point,
            bke::AttrType::ColorFloat,
            bke::AttributeInitDefaultValue()))
    {
      bke::SpanAttributeWriter<ColorGeometry4f> col_attr =
          mesh_raw->attributes_for_write().lookup_or_add_for_write_span<ColorGeometry4f>(
              color_name, bke::AttrDomain::Point);
      for (int i = 0; i < vert_count; i++) {
        col_attr.span[i] = ColorGeometry4f(colors[i].x, colors[i].y, colors[i].z, colors[i].w);
      }
      col_attr.finish();
      BKE_id_attributes_active_color_set(&mesh_raw->id, color_name);
      BKE_id_attributes_default_color_set(&mesh_raw->id, color_name);
    }
  }

  bke::mesh_calc_edges(*mesh_raw, false, false);

  /* Weld duplicate vertices at chunk boundaries */
  Mesh *mesh_clean = mesh_raw;
  {
    const float merge_dist = cell_size * 0.1f;
    std::optional<Mesh *> merged = geometry::mesh_merge_by_distance_all(
        *mesh_clean, IndexMask(mesh_clean->verts_num), merge_dist);
    if (merged.has_value()) {
      BKE_id_free(nullptr, &mesh_clean->id);
      mesh_clean = *merged;
    }
  }

  /* Validate mesh (removes degenerate faces, fixes indices) */
  bke::mesh_validate(*mesh_clean);

  Object *ob = add_type(C, OB_MESH, "SDF Mesh", nullptr, nullptr, false, 0);
  Mesh *mesh_dst = id_cast<Mesh *>(ob->data);
  BKE_mesh_nomain_to_mesh(mesh_clean, mesh_dst, ob, false);

  mesh_dst->tag_positions_changed();

  DEG_id_tag_update(&ob->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  BKE_reportf(op->reports,
              RPT_INFO,
              "Generated mesh: %d verts, %d tris (raw: %d verts, %d tris)",
              mesh_dst->verts_num, mesh_dst->faces_num,
              vert_count, tri_count);
  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_to_mesh(wmOperatorType *ot)
{
  ot->name = "SDF to Mesh";
  ot->description = "Convert SDF scene to triangle mesh via GPU Dual Contouring";
  ot->idname = "OBJECT_OT_sdf_to_mesh";

  ot->exec = object_sdf_to_mesh_exec;
  ot->poll = ED_operator_objectmode;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna,
              "resolution",
              64,
              1,
              256,
              "Resolution",
              "Voxels per Blender unit",
              1,
              256);
}

/* -------------------------------------------------------------------- */
/* SDF Frame Profile Operator */

static void sdf_profile_tag_3d_redraw(bContext *C)
{
  /* Force the 3D viewport to redraw so the SDF draw engine runs */
  ARegion *region = CTX_wm_region(C);
  if (region) {
    ED_region_tag_redraw(region);
  }
  /* Also notify globally in case region isn't the 3D view */
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
}

static wmOperatorStatus sdf_profile_frame_invoke(bContext *C,
                                                  wmOperator *op,
                                                  const wmEvent * /*event*/)
{
  blender::draw::sdf::sdf_profile_request();

  sdf_profile_tag_3d_redraw(C);

  wmWindow *win = CTX_wm_window(C);
  wmWindowManager *wm = CTX_wm_manager(C);
  op->customdata = WM_event_timer_add(wm, win, TIMER, 0.016);

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus sdf_profile_frame_modal(bContext *C,
                                                 wmOperator *op,
                                                 const wmEvent *event)
{
  if (event->type != TIMER) {
    return OPERATOR_PASS_THROUGH;
  }

  if (blender::draw::sdf::sdf_profile_is_pending()) {
    sdf_profile_tag_3d_redraw(C);
    return OPERATOR_RUNNING_MODAL;
  }

  wmWindowManager *wm = CTX_wm_manager(C);
  wmWindow *win = CTX_wm_window(C);
  WM_event_timer_remove(wm, win, static_cast<wmTimer *>(op->customdata));
  op->customdata = nullptr;

  if (!blender::draw::sdf::sdf_profile_is_ready()) {
    BKE_report(op->reports, RPT_WARNING, "SDF profile: no data collected");
    return OPERATOR_CANCELLED;
  }

  std::string text = blender::draw::sdf::sdf_profile_format_text();

  char filepath[FILE_MAX];
  const char *basepath = BKE_main_blendfile_path(CTX_data_main(C));
  if (basepath[0] != '\0') {
    BLI_path_split_dir_part(basepath, filepath, sizeof(filepath));
  }
  else {
    BLI_path_split_dir_part(BKE_tempdir_session(), filepath, sizeof(filepath));
  }
  BLI_path_append(filepath, sizeof(filepath), "sdf_profile.txt");

  FILE *f = BLI_fopen(filepath, "w");
  if (f) {
    fputs(text.c_str(), f);
    fclose(f);
  }

  /* Print full profile to console */
  printf("\n%s\n", text.c_str());

  /* Show save path prominently */
  if (f) {
    printf(">>> Profile saved to: %s\n\n", filepath);
    BKE_reportf(op->reports, RPT_INFO, "SDF profile saved: %s", filepath);
  }
  else {
    BKE_reportf(op->reports, RPT_WARNING, "Failed to write: %s", filepath);
  }

  return OPERATOR_FINISHED;
}

static void sdf_profile_frame_cancel(bContext *C, wmOperator *op)
{
  if (op->customdata) {
    wmWindowManager *wm = CTX_wm_manager(C);
    wmWindow *win = CTX_wm_window(C);
    WM_event_timer_remove(wm, win, static_cast<wmTimer *>(op->customdata));
    op->customdata = nullptr;
  }
}

void OBJECT_OT_sdf_profile_frame(wmOperatorType *ot)
{
  ot->name = "SDF Profile Frame";
  ot->description = "Profile one SDF render frame with per-pass GPU timing and export to file";
  ot->idname = "OBJECT_OT_sdf_profile_frame";

  ot->invoke = sdf_profile_frame_invoke;
  ot->modal = sdf_profile_frame_modal;
  ot->cancel = sdf_profile_frame_cancel;
  ot->poll = ED_operator_objectmode;

  ot->flag = OPTYPE_REGISTER;
}

}  // namespace blender::ed::object
