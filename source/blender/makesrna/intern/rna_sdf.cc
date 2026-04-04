/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_types.hh"

#include "rna_internal.hh"

#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"

#include "BLI_math_base.h"

namespace blender {

const EnumPropertyItem rna_enum_sdf_type_items[] = {
    {SDF_TYPE_BOX, "BOX", ICON_SDF_CUBE, "Cube", "Cube SDF primitive"},
    {SDF_TYPE_SPHERE, "SPHERE", ICON_SDF_SPHERE, "Sphere", "Sphere SDF primitive"},
    {SDF_TYPE_CYLINDER, "CYLINDER", ICON_SDF_CYLINDER, "Cylinder", "Cylinder SDF primitive"},
    {SDF_TYPE_CONE, "CONE", ICON_SDF_CONE, "Cone", "Cone SDF primitive"},
    {SDF_TYPE_CAPSULE, "CAPSULE", ICON_SDF_CAPSULE, "Capsule", "Capsule SDF primitive"},
    {SDF_TYPE_TORUS, "TORUS", ICON_SDF_TORUS, "Torus", "Torus SDF primitive"},
    {SDF_TYPE_NGON, "NGON", ICON_SDF_NGON, "N-Gon", "Regular polygon prism SDF primitive"},
    {SDF_TYPE_POLYGON, "POLYGON", ICON_SDF_POLYGON, "Polygon", "Arbitrary polygon prism SDF primitive"},
    {0, nullptr, 0, nullptr, nullptr},
};

extern const EnumPropertyItem rna_enum_sdf_modifier_type_items[] = {
    {SDF_MOD_MIRROR, "MIRROR", ICON_MOD_MIRROR, "Mirror", "Mirror across axes"},
    {SDF_MOD_TWIST, "TWIST", ICON_MOD_SIMPLEDEFORM, "Twist", "Twist around Z axis"},
    {SDF_MOD_BEND, "BEND", ICON_MOD_SIMPLEDEFORM, "Bend", "Bend around an axis"},
    {SDF_MOD_ELONGATE, "ELONGATE", ICON_MOD_LENGTH, "Elongate", "Stretch along axes"},
    {SDF_MOD_SOLIDIFY, "SOLIDIFY", ICON_MOD_SOLIDIFY, "Solidify", "Add internal shell thickness"},
    {SDF_MOD_ROUND, "ROUND", ICON_MOD_SMOOTH, "Expand/Shrink", "Expand or shrink the SDF surface"},
    {SDF_MOD_ONION, "ONION", ICON_MOD_SOLIDIFY, "Onion", "Create concentric shells"},
    {SDF_MOD_BEVEL, "BEVEL", ICON_MOD_BEVEL, "Bevel", "Bevel/round edges"},
    {SDF_MOD_ARRAY, "ARRAY", ICON_MOD_ARRAY, "Array", "Duplicate geometry"},
    {0, nullptr, 0, nullptr, nullptr},
};

}  // namespace blender

#ifdef RNA_RUNTIME

#  include "MEM_guardedalloc.h"

#  include "BLI_listbase.h"
#  include "BLI_string.h"

#  include "BKE_global.hh"
#  include "BKE_main.hh"
#  include "BKE_report.hh"
#  include "BKE_sdf.hh"
#  include "BKE_sdf_group.hh"

#  include "DNA_object_types.h"
#  include "DNA_sdf_group_types.h"

#  include "DEG_depsgraph.hh"

#  include "WM_api.hh"
#  include "WM_types.hh"

using namespace blender;

static void rna_SDF_update(Main * /*bmain*/, Scene * /*scene*/, PointerRNA *ptr)
{
  SDF *sdf = (SDF *)ptr->owner_id;

  if (sdf->csg_operation == SDF_CSG_SHELL) {
    if (sdf->shell_mode == SDF_SHELL_AVOID) {
      sdf->shell_op = SDF_SHELL_OP_UNION;
    }
    if (sdf->shell_mode == SDF_SHELL_NORMAL && sdf->shell_distance > 0.0f) {
      if (sdf->shell_blend_top > sdf->shell_distance) {
        sdf->shell_blend_top = sdf->shell_distance;
      }
      if (sdf->shell_blend_bottom > sdf->shell_distance) {
        sdf->shell_blend_bottom = sdf->shell_distance;
      }
    }
  }

  DEG_id_tag_update(ptr->owner_id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static void rna_SDF_type_update(Main *bmain, Scene *scene, PointerRNA *ptr)
{
  SDF *sdf = (SDF *)ptr->owner_id;

  switch (sdf->sdf_type) {
    case SDF_TYPE_CAPSULE:
      sdf->size[0] = 0.5f;
      sdf->size[1] = 1.0f;
      sdf->size[2] = 0.5f;
      break;
    case SDF_TYPE_TORUS:
      sdf->size[0] = 0.8f;
      sdf->size[1] = 0.25f;
      sdf->size[2] = 0.8f;
      break;
    case SDF_TYPE_CONE:
      sdf->size[0] = 1.0f;
      sdf->size[1] = 1.0f;
      sdf->size[2] = 1.0f;
      break;
    case SDF_TYPE_NGON:
      sdf->size[0] = 1.0f;
      sdf->size[1] = 1.0f;
      sdf->size[2] = 1.0f;
      break;
    case SDF_TYPE_POLYGON:
      sdf->size[0] = 1.0f;
      sdf->size[1] = 1.0f;
      sdf->size[2] = 1.0f;
      if (BLI_listbase_is_empty(&sdf->polygon_points)) {
        BKE_sdf_polygon_init_triangle(sdf);
      }
      break;
    default:
      sdf->size[0] = 1.0f;
      sdf->size[1] = 1.0f;
      sdf->size[2] = 1.0f;
      break;
  }

  rna_SDF_update(bmain, scene, ptr);
}

static void rna_SDF_modifier_update(Main * /*bmain*/, Scene * /*scene*/, PointerRNA *ptr)
{
  ID *owner = ptr->owner_id;
  DEG_id_tag_update(owner, ID_RECALC_GEOMETRY);
  if (GS(owner->name) == ID_SG) {
    SDFGroup *group = (SDFGroup *)owner;
    for (SDFGroupMember *member = static_cast<SDFGroupMember *>(group->members.first); member;
         member = member->next)
    {
      if (member->object) {
        DEG_id_tag_update(&member->object->id, ID_RECALC_GEOMETRY);
      }
    }
  }
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

/* MATHOPS: Removed old SDF modifier RNA functions — modifiers now on Object */

static SDFPolygonPoint *rna_SDF_polygon_point_new(SDF *sdf, float x, float y)
{
  SDFPolygonPoint *pt = BKE_sdf_polygon_point_add(sdf, x, y, 0.0f);
  DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
  return pt;
}

static void rna_SDF_polygon_point_remove(SDF *sdf, ReportList *reports, PointerRNA *pt_ptr)
{
  SDFPolygonPoint *pt = (SDFPolygonPoint *)pt_ptr->data;
  if (BLI_findindex(&sdf->polygon_points, pt) == -1) {
    BKE_report(reports, RPT_ERROR, "Point not in polygon");
    return;
  }
  if (sdf->totpolygon <= 3) {
    BKE_report(reports, RPT_ERROR, "Polygon must have at least 3 points");
    return;
  }
  BKE_sdf_polygon_point_remove(sdf, pt);
  pt_ptr->data = nullptr;
  DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static void rna_SDF_polygon_point_move(SDF *sdf, ReportList *reports, int from, int to)
{
  SDFPolygonPoint *pt = (SDFPolygonPoint *)BLI_findlink(&sdf->polygon_points, from);
  if (!pt) {
    BKE_reportf(reports, RPT_ERROR, "Invalid point index %d", from);
    return;
  }
  int direction = (to > from) ? 1 : -1;
  int steps = abs(to - from);
  for (int i = 0; i < steps; i++) {
    if (direction == -1) {
      SDFPolygonPoint *prev = pt->prev;
      if (prev) {
        BLI_remlink(&sdf->polygon_points, pt);
        BLI_insertlinkbefore(&sdf->polygon_points, prev, pt);
      }
    }
    else {
      SDFPolygonPoint *next = pt->next;
      if (next) {
        BLI_remlink(&sdf->polygon_points, pt);
        BLI_insertlinkafter(&sdf->polygon_points, next, pt);
      }
    }
  }
  DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static void rna_SDF_sdf_group_set(PointerRNA *ptr,
                                   PointerRNA value,
                                   ReportList * /*reports*/)
{
  SDF *sdf = (SDF *)ptr->owner_id;
  SDFGroup *new_group = (SDFGroup *)value.data;
  SDFGroup *old_group = sdf->sdf_group;

  if (old_group == new_group) {
    return;
  }

  Main *bmain = G_MAIN;
  Object *ob = nullptr;

  /* Find the Object that owns this SDF. */
  if (old_group) {
    for (SDFGroupMember *member = static_cast<SDFGroupMember *>(old_group->members.first);
         member;
         member = member->next)
    {
      if (member->object && member->object->type == OB_SDF &&
          member->object->data == (void *)sdf)
      {
        ob = member->object;
        break;
      }
    }
  }
  if (!ob) {
    for (Object &obj : bmain->objects) {
      if (obj.type == OB_SDF && obj.data == (void *)sdf) {
        ob = &obj;
        break;
      }
    }
  }

  if (old_group && ob) {
    for (SDFGroupMember *member = static_cast<SDFGroupMember *>(old_group->members.first);
         member;
         member = member->next)
    {
      if (member->object == ob) {
        BKE_sdf_group_member_remove(old_group, member);
        break;
      }
    }
  }

  if (new_group && ob) {
    BKE_sdf_group_member_add(new_group, ob);
  }
  else {
    sdf->sdf_group = new_group;
    if (new_group) {
      id_us_plus(&new_group->id);
    }
  }
}

#else

namespace blender {

static const EnumPropertyItem rna_enum_sdf_blend_type_items[] = {
    {SDF_BLEND_LINEAR, "LINEAR", ICON_SDF_BLEND_LINEAR, "Linear", "Hard union/difference"},
    {SDF_BLEND_SMOOTH, "SMOOTH", ICON_SDF_BLEND_SMOOTH, "Smooth", "Smooth blend"},
    {SDF_BLEND_CHAMFER, "CHAMFER", ICON_SDF_BLEND_CHAMFER, "Chamfer", "Chamfer blend"},
    {SDF_BLEND_ROUND, "ROUND", ICON_SDF_BLEND_ROUND, "Round", "Spherical round blend"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_csg_items[] = {
    {SDF_CSG_UNION, "UNION", ICON_SDF_CSG_UNION, "Union", "Boolean union"},
    {SDF_CSG_SUBTRACT, "SUBTRACT", ICON_SDF_CSG_SUBTRACT, "Subtract", "Boolean subtraction"},
    {SDF_CSG_INTERSECT, "INTERSECT", ICON_SDF_CSG_INTERSECT, "Intersect", "Boolean intersection"},
    {SDF_CSG_SHELL, "SHELL", ICON_SDF_CSG_EXTRUDE, "Shell", "Shell/extrusion operation"},
    {SDF_CSG_PUSH,
     "PUSH",
     ICON_SDF_CSG_PUSH,
     "Push",
     "Subtract from base but keep the pushing object visible"},
    {SDF_CSG_AVOID,
     "AVOID",
     ICON_SDF_CSG_AVOID,
     "Avoid",
     "Object is carved by all other objects in the scene"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_shell_mode_items[] = {
    {SDF_SHELL_NORMAL, "NORMAL", 0, "Normal", "Standard shell operation"},
    {SDF_SHELL_AVOID, "AVOID", 0, "Avoid", "Shell avoids the base surface"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_shell_op_items[] = {
    {SDF_SHELL_OP_UNION, "UNION", 0, "Union", "Shell adds to the SDF field"},
    {SDF_SHELL_OP_SUBTRACTION, "SUBTRACTION", 0, "Subtraction", "Shell subtracts from the SDF field"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_box_mode_items[] = {
    {SDF_BOX_MODE_SMOOTH, "SMOOTH", 0, "Smooth", "Smooth rounding"},
    {SDF_BOX_MODE_CHAMFER, "CHAMFER", 0, "Chamfer", "Chamfer (45-degree cut)"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_bend_axis_items[] = {
    {0, "X", 0, "X", "Bend along X axis"},
    {1, "Y", 0, "Y", "Bend along Y axis"},
    {2, "Z", 0, "Z", "Bend along Z axis"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_array_type_items[] = {
    {SDF_MOD_ARRAY_LINEAR, "LINEAR", 0, "Linear", "Linear repeating array"},
    {SDF_MOD_ARRAY_RADIAL, "RADIAL", 0, "Radial", "Radial repeating array"},
    {0, nullptr, 0, nullptr, nullptr},
};

/* Old SDFModifier RNA struct kept for SDFGroup modifier system */
static void rna_def_sdf_modifier(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "SDFModifier", nullptr);
  RNA_def_struct_ui_text(srna, "SDF Modifier", "Modifier in the SDF modifier stack (legacy/group)");
  RNA_def_struct_sdna(srna, "SDFModifier");

  prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_modifier_type_items);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Type", "Modifier type");

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "name");
  RNA_def_property_ui_text(prop, "Name", "Modifier name");
  RNA_def_struct_name_property(srna, prop);

  prop = RNA_def_property(srna, "show_viewport", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "show_viewport", 1);
  RNA_def_property_ui_text(prop, "Viewport", "Enable modifier in viewport");

  prop = RNA_def_property(srna, "params", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "params");
  RNA_def_property_array(prop, 8);
  RNA_def_property_ui_text(prop, "Parameters", "Per-type modifier parameters");
  RNA_def_property_update(prop, 0, "rna_SDF_update");
}

static void rna_def_sdf_polygon_point(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "SDFPolygonPoint", nullptr);
  RNA_def_struct_ui_text(srna, "SDF Polygon Point", "Point in an SDF polygon");
  RNA_def_struct_sdna(srna, "SDFPolygonPoint");

  prop = RNA_def_property(srna, "co", PROP_FLOAT, PROP_XYZ);
  RNA_def_property_float_sdna(prop, nullptr, "co");
  RNA_def_property_array(prop, 2);
  RNA_def_property_range(prop, -FLT_MAX, FLT_MAX);
  RNA_def_property_ui_range(prop, -10.0f, 10.0f, 0.1f, 3);
  RNA_def_property_ui_text(prop, "Position", "2D position in local XY plane");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  prop = RNA_def_property(srna, "corner", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "corner");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 0.1f, 3);
  RNA_def_property_ui_text(prop, "Corner", "Corner smoothing radius");
  RNA_def_property_update(prop, 0, "rna_SDF_update");
}

static void rna_def_sdf_polygon_points(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;
  FunctionRNA *func;
  PropertyRNA *parm;

  RNA_def_property_srna(cprop, "SDFPolygonPoints");
  srna = RNA_def_struct(brna, "SDFPolygonPoints", nullptr);
  RNA_def_struct_sdna(srna, "SDF");
  RNA_def_struct_ui_text(srna, "SDF Polygon Points", "Collection of polygon points");

  func = RNA_def_function(srna, "new", "rna_SDF_polygon_point_new");
  RNA_def_function_ui_description(func, "Add a point to the polygon");
  RNA_def_float(func, "x", 0.0f, -FLT_MAX, FLT_MAX, "X", "", -10.0f, 10.0f);
  RNA_def_float(func, "y", 0.0f, -FLT_MAX, FLT_MAX, "Y", "", -10.0f, 10.0f);
  parm = RNA_def_pointer(func, "point", "SDFPolygonPoint", "", "New point");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "remove", "rna_SDF_polygon_point_remove");
  RNA_def_function_flag(func, FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Remove a point from the polygon");
  parm = RNA_def_pointer(func, "point", "SDFPolygonPoint", "", "Point to remove");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED | PARM_RNAPTR);
  RNA_def_parameter_clear_flags(parm, PROP_THICK_WRAP, ParameterFlag(0));

  func = RNA_def_function(srna, "move", "rna_SDF_polygon_point_move");
  RNA_def_function_flag(func, FUNC_USE_REPORTS);
  RNA_def_function_ui_description(func, "Move a point in the polygon order");
  parm = RNA_def_int(func, "from_index", -1, 0, INT_MAX, "From Index", "", 0, 10000);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_int(func, "to_index", -1, 0, INT_MAX, "To Index", "", 0, 10000);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
}

static void rna_def_sdf(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "SDF", "ID");
  RNA_def_struct_ui_text(srna, "SDF", "SDF data-block for signed distance field objects");
  RNA_def_struct_ui_icon(srna, ICON_SDF_DATA);

  /* SDF Type */
  prop = RNA_def_property(srna, "sdf_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_type_items);
  RNA_def_property_ui_text(prop, "Type", "SDF primitive type");
  RNA_def_property_update(prop, 0, "rna_SDF_type_update");

  /* Size */
  prop = RNA_def_property(srna, "size", PROP_FLOAT, PROP_XYZ);
  RNA_def_property_float_sdna(prop, nullptr, "size");
  RNA_def_property_array(prop, 3);
  RNA_def_property_range(prop, 0.001f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.001f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Size", "Size in each axis");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Color */
  prop = RNA_def_property(srna, "color", PROP_FLOAT, PROP_COLOR);
  RNA_def_property_float_sdna(prop, nullptr, "color");
  RNA_def_property_array(prop, 4);
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_text(prop, "Color", "Display color");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Blend */
  prop = RNA_def_property(srna, "blend", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "blend");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Blend", "Blend amount for CSG operations");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Blend Type */
  prop = RNA_def_property(srna, "blend_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_blend_type_items);
  RNA_def_property_ui_text(prop, "Blend Type", "Blend function type");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* CSG Operation */
  prop = RNA_def_property(srna, "csg_operation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_csg_items);
  RNA_def_property_ui_text(prop, "CSG Operation", "Boolean operation type");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Chamfer/Round Smoothness */
  prop = RNA_def_property(srna, "chamfer_k2", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "chamfer_k2");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 0.5f, 0.01f, 3);
  RNA_def_property_ui_text(prop, "Smoothness 1", "Smooth the chamfer/round edge on shape 1");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  prop = RNA_def_property(srna, "chamfer_k3", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "chamfer_k3");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 0.5f, 0.01f, 3);
  RNA_def_property_ui_text(prop, "Smoothness 2", "Smooth the chamfer/round edge on shape 2");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  prop = RNA_def_property(srna, "chamfer_k4", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "chamfer_k4");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 0.5f, 0.01f, 3);
  RNA_def_property_ui_text(prop, "End Smoothness 1", "Smooth the end edge on shape 1");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  prop = RNA_def_property(srna, "chamfer_k5", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "chamfer_k5");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 0.5f, 0.01f, 3);
  RNA_def_property_ui_text(prop, "End Smoothness 2", "Smooth the end edge on shape 2");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  prop = RNA_def_property(srna, "flip_blend", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flip_blend", 1);
  RNA_def_property_ui_text(prop, "Flip Blend", "Flip the blend direction for round operations");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  prop = RNA_def_property(srna, "flip_blend_end", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flip_blend_end", 1);
  RNA_def_property_ui_text(
      prop, "Flip End", "Flip the end blend direction for round shell operations");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Shell Distance */
  prop = RNA_def_property(srna, "shell_distance", PROP_FLOAT, PROP_DISTANCE);
  RNA_def_property_float_sdna(prop, nullptr, "shell_distance");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 0.1f, 3);
  RNA_def_property_ui_text(
      prop, "Shell Distance", "Offset distance for shell/extrusion operation");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Shell Mode */
  prop = RNA_def_property(srna, "shell_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_shell_mode_items);
  RNA_def_property_ui_text(prop, "Shell Mode", "Shell sub-operation mode");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Shell Op */
  prop = RNA_def_property(srna, "shell_op", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_shell_op_items);
  RNA_def_property_ui_text(prop, "Shell Op", "Whether the shell adds or subtracts from the SDF field");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Shell Blend Top */
  prop = RNA_def_property(srna, "shell_blend_top", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "shell_blend_top");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Blend Top", "Blend amount for the outer shell edge");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Shell Blend Bottom */
  prop = RNA_def_property(srna, "shell_blend_bottom", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "shell_blend_bottom");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Blend Bottom", "Blend amount for the inner shell edge");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Box Corner Bevels */
  prop = RNA_def_property(srna, "box_corners", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "box_corners");
  RNA_def_property_array(prop, 4);
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Corner Bevels", "Per-corner bevel radii (normalized)");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Box Edge Top */
  prop = RNA_def_property(srna, "box_edge_top", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "box_edge_top");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Edge Top", "Top edge chamfer radius");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Box Edge Bottom */
  prop = RNA_def_property(srna, "box_edge_bottom", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "box_edge_bottom");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Edge Bottom", "Bottom edge chamfer radius");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Box Taper */
  prop = RNA_def_property(srna, "box_taper", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "box_taper");
  RNA_def_property_range(prop, -1.0f, 1.0f);
  RNA_def_property_ui_range(prop, -1.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(
      prop, "Taper", "Taper factor (positive tapers top, negative tapers bottom)");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Box Corner Mode */
  prop = RNA_def_property(srna, "box_corner_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_box_mode_items);
  RNA_def_property_ui_text(prop, "Corners", "Corner blend mode");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Box Edge Mode */
  prop = RNA_def_property(srna, "box_edge_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_box_mode_items);
  RNA_def_property_ui_text(prop, "Edges", "Edge blend mode");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* N-Gon Sides */
  prop = RNA_def_property(srna, "ngon_sides", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "ngon_sides");
  RNA_def_property_range(prop, 3, 32);
  RNA_def_property_ui_range(prop, 3, 32, 1, 0);
  RNA_def_property_ui_text(prop, "Sides", "Number of polygon sides");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* N-Gon Corner Bevel */
  prop = RNA_def_property(srna, "ngon_corner", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "ngon_corner");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Corner Bevel", "Uniform corner bevel radius");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* N-Gon Edge Top */
  prop = RNA_def_property(srna, "ngon_edge_top", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "ngon_edge_top");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Edge Top", "Top edge chamfer radius");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* N-Gon Edge Bottom */
  prop = RNA_def_property(srna, "ngon_edge_bottom", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "ngon_edge_bottom");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Edge Bottom", "Bottom edge chamfer radius");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* N-Gon Taper */
  prop = RNA_def_property(srna, "ngon_taper", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "ngon_taper");
  RNA_def_property_range(prop, -1.0f, 1.0f);
  RNA_def_property_ui_range(prop, -1.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(
      prop, "Taper", "Taper factor (positive tapers top, negative tapers bottom)");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* N-Gon Edge Mode */
  prop = RNA_def_property(srna, "ngon_edge_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_box_mode_items);
  RNA_def_property_ui_text(prop, "Edges", "Edge blend mode");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* N-Gon Star */
  prop = RNA_def_property(srna, "ngon_star", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "ngon_star");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Star", "Star factor (0 = regular polygon, 1 = maximum star)");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Polygon Points */
  prop = RNA_def_property(srna, "polygon_points", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "polygon_points", nullptr);
  RNA_def_property_struct_type(prop, "SDFPolygonPoint");
  RNA_def_property_ui_text(prop, "Polygon Points", "Points defining the polygon shape");
  rna_def_sdf_polygon_points(brna, prop);

  /* Polygon Edge Top */
  prop = RNA_def_property(srna, "polygon_edge_top", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "polygon_edge_top");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Edge Top", "Top edge chamfer radius");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Polygon Edge Bottom */
  prop = RNA_def_property(srna, "polygon_edge_bottom", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "polygon_edge_bottom");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Edge Bottom", "Bottom edge chamfer radius");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Polygon Taper */
  prop = RNA_def_property(srna, "polygon_taper", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "polygon_taper");
  RNA_def_property_range(prop, -1.0f, 1.0f);
  RNA_def_property_ui_range(prop, -1.0f, 1.0f, 1.0f, 3);
  RNA_def_property_ui_text(
      prop, "Taper", "Taper factor (positive tapers top, negative tapers bottom)");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Polygon Edge Mode */
  prop = RNA_def_property(srna, "polygon_edge_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_sdf_box_mode_items);
  RNA_def_property_ui_text(prop, "Edges", "Edge blend mode");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Torus Angle */
  prop = RNA_def_property(srna, "torus_angle", PROP_FLOAT, PROP_ANGLE);
  RNA_def_property_float_sdna(prop, nullptr, "torus_angle");
  RNA_def_property_range(prop, 0.0f, (float)M_PI * 2.0f);
  RNA_def_property_ui_range(prop, 0.0f, (float)M_PI * 2.0f, 10.0f, 3);
  RNA_def_property_ui_text(prop, "Angle", "Aperture angle (360 = full torus, less = capped arc)");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Materials */
  prop = RNA_def_property(srna, "materials", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "mat", "totcol");
  RNA_def_property_struct_type(prop, "Material");
  RNA_def_property_ui_text(prop, "Materials", "");
  RNA_def_property_srna(prop, "IDMaterials");
  RNA_def_property_collection_funcs(prop,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    "rna_IDMaterials_assign_int");

  /* MATHOPS: Removed old modifiers collection — modifiers now on Object */

  /* SDF Group */
  prop = RNA_def_property(srna, "sdf_group", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "sdf_group");
  RNA_def_property_struct_type(prop, "SDFGroup");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_pointer_funcs(
      prop, nullptr, "rna_SDF_sdf_group_set", nullptr, nullptr);
  RNA_def_property_ui_text(prop, "SDF Group", "Group this SDF belongs to");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Group Order */
  prop = RNA_def_property(srna, "group_order", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "group_order");
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Group Order", "Evaluation order within group");

  /* SDF Index */
  prop = RNA_def_property(srna, "sdf_index", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "sdf_index");
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "SDF Index", "Global evaluation order index");

  /* Animation data */
  rna_def_animdata_common(srna);
}




void RNA_def_sdf(BlenderRNA *brna)
{
  rna_def_sdf_modifier(brna);
  rna_def_sdf_polygon_point(brna);
  rna_def_sdf(brna);
}

}  // namespace blender

#endif
