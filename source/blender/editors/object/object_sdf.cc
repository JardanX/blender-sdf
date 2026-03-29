/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edobj
 */

#include "MEM_guardedalloc.h"

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"
#include "DNA_space_types.h"

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
#include "BKE_sdf_group.hh"

#include "BLT_translation.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_object.hh"
#include "ED_outliner.hh"
#include "ED_screen.hh"
#include "ED_select_utils.hh"

#include "UI_interface.hh"

#include "BKE_attribute.h"
#include "BKE_attribute.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"

#include "BLI_index_mask.hh"
#include "GEO_mesh_merge_by_distance.hh"

#include "object_intern.hh"

#include "sdf_meshing.hh"

namespace blender::ed::object {

static void remove_from_current_sdf_group(Object *ob)
{
  SDF *sdf = id_cast<SDF *>(ob->data);
  if (!sdf || !sdf->sdf_group) return;
  SDFGroup *old_group = sdf->sdf_group;
  SDFGroupMember *member_next;
  for (SDFGroupMember *member = static_cast<SDFGroupMember *>(old_group->members.first);
       member; member = member_next)
  {
    member_next = member->next;
    if (member->object == ob) {
      BKE_sdf_group_member_remove(old_group, member);
      break;
    }
  }
}

/* SDF Add */

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

  /* Capture active group before add_type changes the active object */
  Main *bmain = CTX_data_main(C);
  SDFGroup *active_group = nullptr;
  {
    Object *active = CTX_data_active_object(C);
    if (active && active->type == OB_SDF && active->data) {
      active_group = id_cast<SDF *>(active->data)->sdf_group;
    }
  }

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
        sdf_data->size[0] = 1.0f;
        sdf_data->size[1] = 1.0f;
        sdf_data->size[2] = 1.0f;
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

    SDFGroup *group = active_group;
    if (!group) {
      group = static_cast<SDFGroup *>(bmain->sdf_groups.last);
    }
    if (!group) {
      group = BKE_sdf_group_add(bmain, "SDF Group");
    }
    BKE_sdf_group_member_add(group, ob);
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
               rna_enum_sdf_type_items,
               SDF_TYPE_BOX,
               "Type",
               "SDF primitive type");
}

/* SDF Group Operators */

static wmOperatorStatus object_sdf_group_add_exec(bContext *C, wmOperator * /*op*/)
{
  Main *bmain = CTX_data_main(C);
  SDFGroup *group = BKE_sdf_group_add(bmain, "SDF Group");

  CTX_DATA_BEGIN (C, Object *, ob, selected_objects) {
    if (ob->type == OB_SDF && ob->data) {
      remove_from_current_sdf_group(ob);
      BKE_sdf_group_member_add(group, ob);
    }
  }
  CTX_DATA_END;

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);

  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_group_add(wmOperatorType *ot)
{
  ot->name = "Add SDF Group";
  ot->description = "Create a new SDF group, optionally adding selected SDF objects";
  ot->idname = "OBJECT_OT_sdf_group_add";

  ot->exec = object_sdf_group_add_exec;
  ot->poll = ED_operator_objectmode;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static wmOperatorStatus object_sdf_group_assign_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  const int group_index = RNA_int_get(op->ptr, "group_index");

  SDFGroup *target = static_cast<SDFGroup *>(BLI_findlink(&bmain->sdf_groups, group_index));
  if (!target) {
    return OPERATOR_CANCELLED;
  }

  CTX_DATA_BEGIN (C, Object *, ob, selected_objects) {
    if (ob->type == OB_SDF && ob->data) {
      SDF *sdf = id_cast<SDF *>(ob->data);
      if (sdf->sdf_group != target) {
        remove_from_current_sdf_group(ob);
        BKE_sdf_group_member_add(target, ob);
      }
    }
  }
  CTX_DATA_END;

  DEG_id_tag_update(&target->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);

  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_group_assign(wmOperatorType *ot)
{
  ot->name = "Assign to SDF Group";
  ot->description = "Assign selected SDF objects to an SDF group";
  ot->idname = "OBJECT_OT_sdf_group_assign";

  ot->exec = object_sdf_group_assign_exec;
  ot->poll = ED_operator_objectmode;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna, "group_index", 0, 0, INT_MAX, "Group Index", "Index of target group", 0, 100);
}

/* SDF CSG / Blend Cycle Operators */

static wmOperatorStatus object_sdf_set_csg_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  char ob_name[MAX_ID_NAME - 2];
  RNA_string_get(op->ptr, "object_name", ob_name);
  const int csg_op = RNA_int_get(op->ptr, "csg_operation");

  Object *ob = (Object *)BKE_libblock_find_name(bmain, ID_OB, ob_name);
  if (!ob || ob->type != OB_SDF || !ob->data) {
    return OPERATOR_CANCELLED;
  }
  SDF *sdf = id_cast<SDF *>(ob->data);
  sdf->csg_operation = csg_op;

  DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
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
  if (!ob || ob->type != OB_SDF || !ob->data) {
    return OPERATOR_CANCELLED;
  }
  SDF *sdf = id_cast<SDF *>(ob->data);
  sdf->blend_type = blend_type;

  DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
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

/* Resolve SDFGroup: operator prop -> pinned -> active object */
static SDFGroup *sdf_group_from_operator(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);

  char group_name[MAX_ID_NAME - 2];
  RNA_string_get(op->ptr, "group_name", group_name);
  if (group_name[0] != '\0') {
    ID *id = BKE_libblock_find_name(bmain, ID_SG, group_name);
    if (id) {
      return (SDFGroup *)id;
    }
  }

  SpaceProperties *sbuts = CTX_wm_space_properties(C);
  if (sbuts && sbuts->pinid && GS(sbuts->pinid->name) == ID_SG) {
    return (SDFGroup *)sbuts->pinid;
  }

  Object *ob = CTX_data_active_object(C);
  if (ob && ob->type == OB_SDF && ob->data) {
    return id_cast<SDF *>(ob->data)->sdf_group;
  }

  return nullptr;
}

static wmOperatorStatus object_sdf_group_remove_member_exec(bContext *C, wmOperator *op)
{
  const int member_index = RNA_int_get(op->ptr, "member_index");

  SDFGroup *group = sdf_group_from_operator(C, op);
  if (!group) {
    return OPERATOR_CANCELLED;
  }

  SDFGroupMember *member = static_cast<SDFGroupMember *>(
      BLI_findlink(&group->members, member_index));
  if (!member) {
    return OPERATOR_CANCELLED;
  }

  BKE_sdf_group_member_remove(group, member);

  Main *bmain = CTX_data_main(C);
  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, nullptr);

  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_group_remove_member(wmOperatorType *ot)
{
  ot->name = "Remove from SDF Group";
  ot->description = "Remove a member from its SDF group";
  ot->idname = "OBJECT_OT_sdf_group_remove_member";

  ot->exec = object_sdf_group_remove_member_exec;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna, "member_index", 0, 0, INT_MAX, "Member Index", "Index of member to remove", 0, 100);
  RNA_def_string(ot->srna, "group_name", nullptr, MAX_ID_NAME - 2, "Group Name", "Name of the SDF group");
}

static wmOperatorStatus object_sdf_group_reorder_exec(bContext *C, wmOperator *op)
{
  const int member_index = RNA_int_get(op->ptr, "member_index");
  const int direction = RNA_int_get(op->ptr, "direction");

  SDFGroup *group = sdf_group_from_operator(C, op);
  if (!group) {
    return OPERATOR_CANCELLED;
  }

  SDFGroupMember *member = static_cast<SDFGroupMember *>(
      BLI_findlink(&group->members, member_index));
  if (!member) {
    return OPERATOR_CANCELLED;
  }

  BKE_sdf_group_member_move(group, member, direction);

  Main *bmain = CTX_data_main(C);
  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, nullptr);

  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_group_reorder(wmOperatorType *ot)
{
  ot->name = "Reorder in SDF Group";
  ot->description = "Move member up or down within its SDF group";
  ot->idname = "OBJECT_OT_sdf_group_reorder";

  ot->exec = object_sdf_group_reorder_exec;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna, "member_index", 0, 0, INT_MAX, "Member Index", "Index of member to move", 0, 100);
  RNA_def_int(ot->srna, "direction", -1, -1, 1, "Direction", "Move direction (-1=up, 1=down)", -1, 1);
  RNA_def_string(ot->srna, "group_name", nullptr, MAX_ID_NAME - 2, "Group Name", "Name of the SDF group");
}

/* Move to SDF Group */

static wmOperatorStatus move_to_sdf_group_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  const bool is_new = RNA_boolean_get(op->ptr, "is_new");

  SDFGroup *target = nullptr;

  if (is_new) {
    char name[MAX_ID_NAME - 2];
    RNA_string_get(op->ptr, "new_group_name", name);
    target = BKE_sdf_group_add(bmain, name[0] ? name : "SDF Group");
  }
  else {
    char group_name[MAX_ID_NAME - 2];
    RNA_string_get(op->ptr, "group_name", group_name);
    if (group_name[0] == '\0') {
      BKE_report(op->reports, RPT_ERROR, "No SDF group selected");
      return OPERATOR_CANCELLED;
    }
    ID *id = BKE_libblock_find_name(bmain, ID_SG, group_name);
    if (!id) {
      BKE_report(op->reports, RPT_ERROR, "SDF group not found");
      return OPERATOR_CANCELLED;
    }
    target = (SDFGroup *)id;
  }

  int moved_count = 0;
  CTX_DATA_BEGIN (C, Object *, ob, selected_objects) {
    if (ob->type == OB_SDF && ob->data) {
      SDF *sdf = id_cast<SDF *>(ob->data);
      if (sdf->sdf_group != target) {
        remove_from_current_sdf_group(ob);
        BKE_sdf_group_member_add(target, ob);
        moved_count++;
      }
    }
  }
  CTX_DATA_END;

  if (moved_count == 0) {
    BKE_report(op->reports, RPT_WARNING, "No SDF objects were moved");
    return OPERATOR_CANCELLED;
  }

  DEG_id_tag_update(&target->id, ID_RECALC_GEOMETRY);
  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_LAYER, nullptr);

  BKE_reportf(
      op->reports, RPT_INFO, "Moved %d object(s) to %s", moved_count, target->id.name + 2);

  return OPERATOR_FINISHED;
}

static wmOperatorStatus move_to_sdf_group_invoke(bContext *C,
                                                  wmOperator *op,
                                                  const wmEvent * /*event*/)
{
  if (!RNA_boolean_get(op->ptr, "is_new")) {
    return move_to_sdf_group_exec(C, op);
  }

  PropertyRNA *prop = RNA_struct_find_property(op->ptr, "new_group_name");
  if (!RNA_property_is_set(op->ptr, prop)) {
    RNA_property_string_set(op->ptr, prop, "SDF Group");
    return WM_operator_props_dialog_popup(
        C, op, 200, IFACE_("Move to New SDF Group"), IFACE_("Create"));
  }

  return move_to_sdf_group_exec(C, op);
}

void OBJECT_OT_move_to_sdf_group(wmOperatorType *ot)
{
  PropertyRNA *prop;

  ot->name = "Move to SDF Group";
  ot->description = "Move selected SDF objects to an SDF group";
  ot->idname = "OBJECT_OT_move_to_sdf_group";

  ot->exec = move_to_sdf_group_exec;
  ot->invoke = move_to_sdf_group_invoke;
  ot->poll = ED_operator_objectmode;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  prop = RNA_def_string(ot->srna,
                        "group_name",
                        nullptr,
                        MAX_ID_NAME - 2,
                        "Group Name",
                        "Name of the target SDF group");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE | PROP_HIDDEN);

  prop = RNA_def_boolean(ot->srna, "is_new", false, "New", "Move objects to a new SDF group");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE | PROP_HIDDEN);

  prop = RNA_def_string(ot->srna,
                        "new_group_name",
                        nullptr,
                        MAX_ID_NAME - 2,
                        "Name",
                        "Name of the newly created SDF group");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  ot->prop = prop;
}

/* SDF Group Reorder */

static wmOperatorStatus object_sdf_group_reorder_group_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  char group_name[MAX_ID_NAME - 2];
  RNA_string_get(op->ptr, "group_name", group_name);
  const int direction = RNA_int_get(op->ptr, "direction");

  ID *id = BKE_libblock_find_name(bmain, ID_SG, group_name);
  if (!id) {
    return OPERATOR_CANCELLED;
  }
  SDFGroup *group = (SDFGroup *)id;

  if (direction == -1) {
    SDFGroup *prev = (SDFGroup *)group->id.prev;
    if (prev) {
      BLI_remlink(&bmain->sdf_groups, group);
      BLI_insertlinkbefore(&bmain->sdf_groups, prev, group);
    }
  }
  else if (direction == 1) {
    SDFGroup *next = (SDFGroup *)group->id.next;
    if (next) {
      BLI_remlink(&bmain->sdf_groups, group);
      BLI_insertlinkafter(&bmain->sdf_groups, next, group);
    }
  }

  int i = 0;
  for (SDFGroup *g = reinterpret_cast<SDFGroup *>(bmain->sdf_groups.first); g; g = reinterpret_cast<SDFGroup *>(g->id.next)) {
    g->group_order = i++;
  }

  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, nullptr);

  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_group_reorder_group(wmOperatorType *ot)
{
  ot->name = "Reorder SDF Group";
  ot->description = "Move an SDF group up or down relative to other groups";
  ot->idname = "OBJECT_OT_sdf_group_reorder_group";

  ot->exec = object_sdf_group_reorder_group_exec;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna, "group_name", nullptr, MAX_ID_NAME - 2, "Group Name", "Name of the SDF group to move");
  RNA_def_int(ot->srna, "direction", -1, -1, 1, "Direction", "Move direction (-1=up, 1=down)", -1, 1);
}

/* SDF Group Cycle */

static bool sdf_group_cycle_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  if (!ob || ob->type != OB_SDF || !ob->data) {
    return false;
  }
  SDF *sdf = id_cast<SDF *>(ob->data);
  return sdf->sdf_group != nullptr;
}

static wmOperatorStatus object_sdf_group_cycle_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  const int direction = RNA_int_get(op->ptr, "direction");

  Object *ob = CTX_data_active_object(C);
  if (!ob || ob->type != OB_SDF || !ob->data) {
    return OPERATOR_CANCELLED;
  }

  SDF *sdf = id_cast<SDF *>(ob->data);
  SDFGroup *group = sdf->sdf_group;
  if (!group) {
    return OPERATOR_CANCELLED;
  }

  SDFGroupMember *current = BKE_sdf_group_member_find_by_object(group, ob);
  if (!current) {
    return OPERATOR_CANCELLED;
  }

  SDFGroupMember *target = (direction > 0) ? current->next : current->prev;
  if (!target) {
    target = (direction > 0) ? static_cast<SDFGroupMember *>(group->members.first)
                             : static_cast<SDFGroupMember *>(group->members.last);
  }
  if (!target || !target->object || target == current) {
    return OPERATOR_CANCELLED;
  }

  BKE_view_layer_synced_ensure(scene, view_layer);
  Base *base_new = BKE_view_layer_base_find(view_layer, target->object);
  if (!base_new) {
    return OPERATOR_CANCELLED;
  }

  base_deselect_all(scene, view_layer, nullptr, SEL_DESELECT);
  base_select(base_new, BA_SELECT);
  base_activate(C, base_new);

  DEG_id_tag_update(&scene->id, ID_RECALC_SELECT);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_SELECT, scene);
  ED_outliner_select_sync_from_object_tag(C);

  return OPERATOR_FINISHED;
}

void OBJECT_OT_sdf_group_cycle(wmOperatorType *ot)
{
  ot->name = "Cycle SDF Group Member";
  ot->description = "Cycle through SDF group members";
  ot->idname = "OBJECT_OT_sdf_group_cycle";

  ot->exec = object_sdf_group_cycle_exec;
  ot->poll = sdf_group_cycle_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna, "direction", 1, -1, 1, "Direction", "Cycle direction (-1=previous, 1=next)", -1, 1);
}

/* SDF Blend Adjust (Modal) */

struct SDFBlendAdjustData {
  float init_mval_x;
  float init_blend;
  float current_blend;
  bool slow_mode;
  float slow_mval_x;
  float slow_blend;
};

static bool sdf_blend_adjust_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  return ob && ob->type == OB_SDF && ob->data && CTX_wm_region_view3d(C);
}

static void sdf_blend_adjust_update_header(bContext *C, SDFBlendAdjustData *data)
{
  WorkspaceStatus status(C);
  status.item(IFACE_("Confirm"), ICON_EVENT_RETURN, ICON_MOUSE_LMB);
  status.item(IFACE_("Cancel"), ICON_EVENT_ESC, ICON_MOUSE_RMB);
  status.item(IFACE_("Adjust Blend"), ICON_MOUSE_MOVE);
  status.item_bool(IFACE_("Precision"), data->slow_mode, ICON_EVENT_SHIFT);

  char msg[128];
  SNPRINTF(msg, "Blend: %.3f", data->current_blend);
  ED_area_status_text(CTX_wm_area(C), msg);
}

static wmOperatorStatus sdf_blend_adjust_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  SDFBlendAdjustData *data = static_cast<SDFBlendAdjustData *>(op->customdata);
  Object *ob = CTX_data_active_object(C);
  if (!ob || ob->type != OB_SDF || !ob->data) {
    MEM_delete(data);
    op->customdata = nullptr;
    ED_area_status_text(CTX_wm_area(C), nullptr);
    ED_workspace_status_text(C, nullptr);
    return OPERATOR_CANCELLED;
  }
  SDF *sdf = id_cast<SDF *>(ob->data);

  if ((event->type == EVT_ESCKEY && event->val == KM_PRESS) ||
      (event->type == RIGHTMOUSE && event->val == KM_PRESS))
  {
    sdf->blend = data->init_blend;
    DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
    ED_area_status_text(CTX_wm_area(C), nullptr);
    ED_workspace_status_text(C, nullptr);
    MEM_delete(data);
    return OPERATOR_CANCELLED;
  }

  if ((event->type == LEFTMOUSE && event->val == KM_RELEASE) ||
      (event->type == EVT_RETKEY && event->val == KM_PRESS) ||
      (event->type == EVT_PADENTER && event->val == KM_PRESS))
  {
    sdf->blend = data->current_blend;
    DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
    ED_area_status_text(CTX_wm_area(C), nullptr);
    ED_workspace_status_text(C, nullptr);
    MEM_delete(data);
    return OPERATOR_FINISHED;
  }

  float mx = float(event->mval[0]);
  float sensitivity = 0.01f;

  if (event->type == EVT_LEFTSHIFTKEY && event->val == KM_PRESS) {
    data->slow_mode = true;
    data->slow_mval_x = mx;
    data->slow_blend = data->current_blend;
  }
  if (event->type == EVT_LEFTSHIFTKEY && event->val == KM_RELEASE) {
    data->slow_mode = false;
  }

  if (data->slow_mode) {
    float d = (mx - data->slow_mval_x) * sensitivity * 0.1f;
    data->current_blend = max_ff(data->slow_blend + d, 0.0f);
  }
  else {
    float d = (mx - data->init_mval_x) * sensitivity;
    data->current_blend = max_ff(data->init_blend + d, 0.0f);
  }

  sdf->blend = data->current_blend;
  DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);

  sdf_blend_adjust_update_header(C, data);
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus sdf_blend_adjust_invoke(bContext *C,
                                                 wmOperator *op,
                                                 const wmEvent *event)
{
  Object *ob = CTX_data_active_object(C);
  SDF *sdf = id_cast<SDF *>(ob->data);

  SDFBlendAdjustData *data = MEM_new<SDFBlendAdjustData>(__func__);
  data->init_mval_x = float(event->mval[0]);
  data->init_blend = sdf->blend;
  data->current_blend = sdf->blend;
  data->slow_mode = false;
  op->customdata = data;

  WM_event_add_modal_handler(C, op);
  sdf_blend_adjust_update_header(C, data);

  return OPERATOR_RUNNING_MODAL;
}

static void sdf_blend_adjust_cancel(bContext *C, wmOperator *op)
{
  SDFBlendAdjustData *data = static_cast<SDFBlendAdjustData *>(op->customdata);
  Object *ob = CTX_data_active_object(C);
  if (ob && ob->type == OB_SDF && ob->data) {
    SDF *sdf = id_cast<SDF *>(ob->data);
    sdf->blend = data->init_blend;
    DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  }
  ED_area_status_text(CTX_wm_area(C), nullptr);
  ED_workspace_status_text(C, nullptr);
  MEM_delete(data);
}

void OBJECT_OT_sdf_blend_adjust(wmOperatorType *ot)
{
  ot->name = "Adjust SDF Blend";
  ot->description = "Interactively adjust blend amount by moving the mouse";
  ot->idname = "OBJECT_OT_sdf_blend_adjust";

  ot->invoke = sdf_blend_adjust_invoke;
  ot->modal = sdf_blend_adjust_modal;
  ot->cancel = sdf_blend_adjust_cancel;
  ot->poll = sdf_blend_adjust_poll;

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

}  // namespace blender::ed::object
