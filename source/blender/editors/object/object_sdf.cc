/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edobj
 */

#include "MEM_guardedalloc.h"

#include "DNA_object_types.h"
#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"
#include "DNA_space_types.h"

#include "BLI_listbase.h"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "BKE_context.hh"
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
#include "ED_screen.hh"

#include "object_intern.hh"

namespace blender::ed::object {

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

  Object *ob = add_type(C, OB_SDF, name, loc, rot, false, local_view_bits);
  if (ob && ob->data) {
    Main *bmain = CTX_data_main(C);
    SDF *sdf_data = static_cast<SDF *>(ob->data);
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

    SDFGroup *group = nullptr;
    Object *active = CTX_data_active_object(C);
    if (active && active->type == OB_SDF && active->data) {
      group = static_cast<SDF *>(active->data)->sdf_group;
    }
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
      SDF *sdf = static_cast<SDF *>(ob->data);
      if (sdf->sdf_group) {
        SDFGroup *old_group = sdf->sdf_group;
        LISTBASE_FOREACH_MUTABLE (SDFGroupMember *, member, &old_group->members) {
          if (member->object == ob) {
            BKE_sdf_group_member_remove(old_group, member);
            break;
          }
        }
      }
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
      SDF *sdf = static_cast<SDF *>(ob->data);
      if (sdf->sdf_group && sdf->sdf_group != target) {
        SDFGroup *old_group = sdf->sdf_group;
        LISTBASE_FOREACH_MUTABLE (SDFGroupMember *, member, &old_group->members) {
          if (member->object == ob) {
            BKE_sdf_group_member_remove(old_group, member);
            break;
          }
        }
      }
      if (sdf->sdf_group != target) {
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
  SDF *sdf = static_cast<SDF *>(ob->data);
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
  SDF *sdf = static_cast<SDF *>(ob->data);
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
    return static_cast<SDF *>(ob->data)->sdf_group;
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

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, nullptr);

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

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, nullptr);

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
      SDF *sdf = static_cast<SDF *>(ob->data);
      /* Remove from previous group first. */
      if (sdf->sdf_group && sdf->sdf_group != target) {
        SDFGroup *old_group = sdf->sdf_group;
        LISTBASE_FOREACH_MUTABLE (SDFGroupMember *, member, &old_group->members) {
          if (member->object == ob) {
            BKE_sdf_group_member_remove(old_group, member);
            break;
          }
        }
      }
      if (sdf->sdf_group != target) {
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
  LISTBASE_FOREACH (SDFGroup *, g, &bmain->sdf_groups) {
    g->group_order = i++;
  }

  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, nullptr);

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

}  // namespace blender::ed::object
