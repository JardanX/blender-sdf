/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "rna_internal.hh"

#include "DNA_object_types.h"
#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"

#include "BLI_math_base.h"

#ifdef RNA_RUNTIME

#  include "MEM_guardedalloc.h"

#  include "BLI_listbase.h"
#  include "BLI_string.h"

#  include "BKE_main.hh"
#  include "BKE_report.hh"
#  include "BKE_sdf_group.hh"

#  include "DEG_depsgraph.hh"

#  include "WM_api.hh"
#  include "WM_types.hh"

static void rna_SDFGroup_update(Main * /*bmain*/, Scene * /*scene*/, PointerRNA *ptr)
{
  DEG_id_tag_update(ptr->owner_id, ID_RECALC_GEOMETRY);

  SDFGroup *group = (SDFGroup *)ptr->owner_id;
  LISTBASE_FOREACH (SDFGroupMember *, member, &group->members) {
    if (member->object) {
      DEG_id_tag_update(&member->object->id, ID_RECALC_GEOMETRY);
    }
  }

  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static SDFGroupMember *rna_SDFGroup_member_add(SDFGroup *group,
                                                ReportList *reports,
                                                Object *ob)
{
  if (!ob || ob->type != OB_SDF) {
    BKE_report(reports, RPT_ERROR, "Only SDF objects can be added to SDF groups");
    return nullptr;
  }

  SDFGroupMember *member = BKE_sdf_group_member_add(group, ob);

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);

  return member;
}

static void rna_SDFGroup_member_remove(SDFGroup *group,
                                        ReportList *reports,
                                        PointerRNA *member_ptr)
{
  SDFGroupMember *member = (SDFGroupMember *)member_ptr->data;

  if (BLI_findindex(&group->members, member) == -1) {
    BKE_report(reports, RPT_ERROR, "Member not in group");
    return;
  }

  BKE_sdf_group_member_remove(group, member);
  member_ptr->data = nullptr;

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static void rna_SDFGroup_member_move(SDFGroup *group,
                                      ReportList *reports,
                                      int from,
                                      int to)
{
  SDFGroupMember *member = (SDFGroupMember *)BLI_findlink(&group->members, from);
  if (!member) {
    BKE_reportf(reports, RPT_ERROR, "Invalid member index %d", from);
    return;
  }

  int direction = (to > from) ? 1 : -1;
  int steps = abs(to - from);
  for (int i = 0; i < steps; i++) {
    BKE_sdf_group_member_move(group, member, direction);
  }

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static SDFModifier *rna_SDFGroup_modifier_new(SDFGroup *group, int type)
{
  SDFModifier *mod = BKE_sdf_group_modifier_add(group, type);

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  LISTBASE_FOREACH (SDFGroupMember *, member, &group->members) {
    if (member->object) {
      DEG_id_tag_update(&member->object->id, ID_RECALC_GEOMETRY);
    }
  }
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);

  return mod;
}

static void rna_SDFGroup_modifier_remove(SDFGroup *group,
                                          ReportList *reports,
                                          PointerRNA *mod_ptr)
{
  SDFModifier *mod = (SDFModifier *)mod_ptr->data;

  if (BLI_findindex(&group->modifiers, mod) == -1) {
    BKE_reportf(reports, RPT_ERROR, "Modifier '%s' not in group", mod->name);
    return;
  }

  BKE_sdf_group_modifier_remove(group, mod);
  mod_ptr->data = nullptr;

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  LISTBASE_FOREACH (SDFGroupMember *, member, &group->members) {
    if (member->object) {
      DEG_id_tag_update(&member->object->id, ID_RECALC_GEOMETRY);
    }
  }
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static void rna_SDFGroup_modifier_move(SDFGroup *group,
                                        ReportList *reports,
                                        int from,
                                        int to)
{
  SDFModifier *mod = (SDFModifier *)BLI_findlink(&group->modifiers, from);
  if (!mod) {
    BKE_reportf(reports, RPT_ERROR, "Invalid modifier index %d", from);
    return;
  }

  int direction = (to > from) ? 1 : -1;
  int steps = abs(to - from);
  for (int i = 0; i < steps; i++) {
    BKE_sdf_group_modifier_move(group, mod, direction);
  }

  DEG_id_tag_update(&group->id, ID_RECALC_GEOMETRY);
  LISTBASE_FOREACH (SDFGroupMember *, member, &group->members) {
    if (member->object) {
      DEG_id_tag_update(&member->object->id, ID_RECALC_GEOMETRY);
    }
  }
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static bool rna_SDFGroupMember_object_poll(PointerRNA * /*ptr*/, PointerRNA value)
{
  Object *ob = (Object *)value.owner_id;
  return (ob->type == OB_SDF);
}

#else

extern const EnumPropertyItem rna_enum_sdf_type_items[];

static const EnumPropertyItem rna_enum_sdf_group_blend_type_items[] = {
    {SDF_BLEND_LINEAR, "LINEAR", ICON_SDF_BLEND_LINEAR, "Linear", "Hard union/difference"},
    {SDF_BLEND_SMOOTH, "SMOOTH", ICON_SDF_BLEND_SMOOTH, "Smooth", "Smooth blend"},
    {SDF_BLEND_CHAMFER, "CHAMFER", ICON_SDF_BLEND_CHAMFER, "Chamfer", "Chamfer blend"},
    {SDF_BLEND_ROUND, "ROUND", ICON_SDF_BLEND_ROUND, "Round", "Spherical round blend"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_group_shell_mode_items[] = {
    {SDF_SHELL_NORMAL, "NORMAL", 0, "Normal", "Standard shell operation"},
    {SDF_SHELL_PUSH, "PUSH", 0, "Shell Push", "Shell combined with push"},
    {SDF_SHELL_AVOID, "AVOID", 0, "Shell Avoid", "Shell combined with avoid"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_group_csg_items[] = {
    {SDF_CSG_UNION, "UNION", ICON_SDF_CSG_UNION, "Union", "Boolean union"},
    {SDF_CSG_SUBTRACT, "SUBTRACT", ICON_SDF_CSG_SUBTRACT, "Subtract", "Boolean subtraction"},
    {SDF_CSG_INTERSECT, "INTERSECT", ICON_SDF_CSG_INTERSECT, "Intersect", "Boolean intersection"},
    {SDF_CSG_SHELL, "SHELL", ICON_SDF_CSG_EXTRUDE, "Shell", "Shell/extrusion operation"},
    {SDF_CSG_PUSH, "PUSH", ICON_SDF_CSG_PUSH, "Push", "Subtract but keep pushing object visible"},
    {SDF_CSG_AVOID, "AVOID", ICON_SDF_CSG_AVOID, "Avoid", "Object carved by all other objects"},
    {0, nullptr, 0, nullptr, nullptr},
};

extern const EnumPropertyItem rna_enum_sdf_modifier_type_items[];

static void rna_def_sdf_group_modifiers(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;
  FunctionRNA *func;
  PropertyRNA *parm;

  RNA_def_property_srna(cprop, "SDFGroupModifiers");
  srna = RNA_def_struct(brna, "SDFGroupModifiers", nullptr);
  RNA_def_struct_sdna(srna, "SDFGroup");
  RNA_def_struct_ui_text(srna, "SDF Group Modifiers", "Collection of SDF group modifiers");

  func = RNA_def_function(srna, "new", "rna_SDFGroup_modifier_new");
  RNA_def_function_ui_description(func, "Add a new SDF modifier to the group");
  parm = RNA_def_enum(
      func, "type", rna_enum_sdf_modifier_type_items, 0, "Type", "Modifier type");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_pointer(func, "modifier", "SDFModifier", "", "New modifier");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "remove", "rna_SDFGroup_modifier_remove");
  RNA_def_function_flag(func, FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Remove an SDF modifier from the group");
  parm = RNA_def_pointer(func, "modifier", "SDFModifier", "", "Modifier to remove");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED | PARM_RNAPTR);
  RNA_def_parameter_clear_flags(parm, PROP_THICK_WRAP, ParameterFlag(0));

  func = RNA_def_function(srna, "move", "rna_SDFGroup_modifier_move");
  RNA_def_function_flag(func, FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Move a modifier in the group stack");
  parm = RNA_def_int(func, "from_index", -1, 0, INT_MAX, "From Index", "", 0, 10000);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_int(func, "to_index", -1, 0, INT_MAX, "To Index", "", 0, 10000);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
}

static void rna_def_sdf_group_member(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "SDFGroupMember", nullptr);
  RNA_def_struct_ui_text(srna, "SDF Group Member", "Member entry in an SDF evaluation group");
  RNA_def_struct_sdna(srna, "SDFGroupMember");

  /* Object */
  prop = RNA_def_property(srna, "object", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "object");
  RNA_def_property_struct_type(prop, "Object");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Object", "SDF object in this group");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Order (read-only) */
  prop = RNA_def_property(srna, "order", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "order");
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Order", "Evaluation order within the group");
}

static void rna_def_sdf_group_members(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;
  FunctionRNA *func;
  PropertyRNA *parm;

  RNA_def_property_srna(cprop, "SDFGroupMembers");
  srna = RNA_def_struct(brna, "SDFGroupMembers", nullptr);
  RNA_def_struct_sdna(srna, "SDFGroup");
  RNA_def_struct_ui_text(srna, "SDF Group Members", "Collection of SDF group members");

  /* Add */
  func = RNA_def_function(srna, "add", "rna_SDFGroup_member_add");
  RNA_def_function_flag(func, FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Add an SDF object to the group");
  parm = RNA_def_pointer(func, "object", "Object", "", "SDF object to add");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED);
  parm = RNA_def_pointer(func, "member", "SDFGroupMember", "", "New member entry");
  RNA_def_function_return(func, parm);

  /* Remove */
  func = RNA_def_function(srna, "remove", "rna_SDFGroup_member_remove");
  RNA_def_function_flag(func, FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Remove a member from the group");
  parm = RNA_def_pointer(func, "member", "SDFGroupMember", "", "Member to remove");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED | PARM_RNAPTR);
  RNA_def_parameter_clear_flags(parm, PROP_THICK_WRAP, ParameterFlag(0));

  /* Move */
  func = RNA_def_function(srna, "move", "rna_SDFGroup_member_move");
  RNA_def_function_flag(func, FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Move a member within the group");
  parm = RNA_def_int(func, "from_index", -1, 0, INT_MAX, "From Index", "", 0, 10000);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_int(func, "to_index", -1, 0, INT_MAX, "To Index", "", 0, 10000);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
}

static void rna_def_sdf_group(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "SDFGroup", "ID");
  RNA_def_struct_ui_text(
      srna, "SDF Group", "Ordered container for SDF objects with group-level CSG");
  RNA_def_struct_ui_icon(srna, ICON_SDF_DATA);

  /* CSG Operation */
  prop = RNA_def_property(srna, "csg_operation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_group_csg_items);
  RNA_def_property_ui_text(prop, "CSG Operation", "How this group combines with previous groups");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Blend Type */
  prop = RNA_def_property(srna, "blend_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_group_blend_type_items);
  RNA_def_property_ui_text(prop, "Blend Type", "Blend function type for group CSG");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Blend */
  prop = RNA_def_property(srna, "blend", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "blend");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Blend", "Blend amount for group-level CSG operations");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Chamfer/Round Smoothness */
  prop = RNA_def_property(srna, "chamfer_k2", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "chamfer_k2");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 0.5f, 0.01f, 3);
  RNA_def_property_ui_text(prop, "Smoothness 1", "Smooth the chamfer/round edge on shape 1");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  prop = RNA_def_property(srna, "chamfer_k3", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "chamfer_k3");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 0.5f, 0.01f, 3);
  RNA_def_property_ui_text(prop, "Smoothness 2", "Smooth the chamfer/round edge on shape 2");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Shell Distance */
  prop = RNA_def_property(srna, "shell_distance", PROP_FLOAT, PROP_DISTANCE);
  RNA_def_property_float_sdna(prop, nullptr, "shell_distance");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 0.1f, 3);
  RNA_def_property_ui_text(
      prop, "Shell Distance", "Offset distance for shell/extrusion operation");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Shell Mode */
  prop = RNA_def_property(srna, "shell_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_group_shell_mode_items);
  RNA_def_property_ui_text(prop, "Shell Mode", "Shell sub-operation mode");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Shell Blend Top */
  prop = RNA_def_property(srna, "shell_blend_top", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "shell_blend_top");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Blend Top", "Blend amount for the outer shell edge");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Shell Blend Bottom */
  prop = RNA_def_property(srna, "shell_blend_bottom", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "shell_blend_bottom");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Blend Bottom", "Blend amount for the inner shell edge");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Color */
  prop = RNA_def_property(srna, "color", PROP_FLOAT, PROP_COLOR);
  RNA_def_property_float_sdna(prop, nullptr, "color");
  RNA_def_property_array(prop, 4);
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_text(prop, "Color", "Global tint multiplier applied to all group members");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Group Order */
  prop = RNA_def_property(srna, "group_order", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "group_order");
  RNA_def_property_range(prop, 0, 1000);
  RNA_def_property_ui_text(prop, "Group Order", "Evaluation order among groups");
  RNA_def_property_update(prop, 0, "rna_SDFGroup_update");

  /* Members */
  prop = RNA_def_property(srna, "members", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "members", nullptr);
  RNA_def_property_struct_type(prop, "SDFGroupMember");
  RNA_def_property_ui_text(prop, "Members", "Ordered list of SDF objects in this group");
  rna_def_sdf_group_members(brna, prop);

  /* Modifiers */
  prop = RNA_def_property(srna, "modifiers", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "modifiers", nullptr);
  RNA_def_property_struct_type(prop, "SDFModifier");
  RNA_def_property_ui_text(prop, "Modifiers", "Modifier stack applied to the group as a whole");
  rna_def_sdf_group_modifiers(brna, prop);

  /* Animation data */
  rna_def_animdata_common(srna);
}

void RNA_def_sdf_group(BlenderRNA *brna)
{
  rna_def_sdf_group_member(brna);
  rna_def_sdf_group(brna);
}

#endif
