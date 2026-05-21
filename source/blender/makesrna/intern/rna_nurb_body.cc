/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include "RNA_define.hh"
#include "RNA_types.hh"

#include "rna_internal.hh"

#include "DNA_nurb_body_types.h"

namespace blender {

static const EnumPropertyItem rna_enum_nurb_body_primitive_items[] = {
    {NURB_BODY_PRIMITIVE_CYLINDER, "CYLINDER", ICON_MESH_CYLINDER, "Cylinder", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_nurb_body_select_mode_items[] = {
    {NURB_BODY_SELECT_MODE_EDGE, "EDGE", ICON_EDGESEL, "Edge", "Select generated edges"},
    {NURB_BODY_SELECT_MODE_FACE, "FACE", ICON_FACESEL, "Face", "Reserve face selection"},
    {NURB_BODY_SELECT_MODE_OBJECT, "OBJECT", ICON_OBJECT_DATAMODE, "Object", "Select objects"},
    {0, nullptr, 0, nullptr, nullptr},
};

}  // namespace blender

#ifdef RNA_RUNTIME

#  include "BKE_context.hh"

#  include "DEG_depsgraph.hh"

#  include "WM_api.hh"
#  include "WM_types.hh"

namespace blender {

static void rna_NurbBody_update(Main * /*bmain*/, Scene * /*scene*/, PointerRNA *ptr)
{
  DEG_id_tag_update(ptr->owner_id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

static void rna_NurbBody_select_update(Main * /*bmain*/, Scene * /*scene*/, PointerRNA *ptr)
{
  NurbBody *body = static_cast<NurbBody *>(ptr->data);
  if (body->select_mode != NURB_BODY_SELECT_MODE_EDGE) {
    body->selected_edges = 0;
    body->selected_edge = -1;
    body->surface_selected_edges = 0;
    body->surface_selected_edge = -1;
  }
  body->hovered_edge = -1;
  body->surface_hovered_edge = -1;
  for (NurbBodyBooleanOp *op = static_cast<NurbBodyBooleanOp *>(body->boolean_ops.first); op;
       op = op->next)
  {
    if (body->select_mode != NURB_BODY_SELECT_MODE_EDGE) {
      op->selected_edges = 0;
      op->selected_edge = -1;
      op->flag &= ~NURB_BODY_BOOLEAN_OP_SELECTED;
    }
    op->hovered_edge = -1;
    op->flag &= ~NURB_BODY_BOOLEAN_OP_HOVERED;
  }

  DEG_id_tag_update(ptr->owner_id, ID_RECALC_SELECT);
  WM_main_add_notifier(NC_OBJECT | ND_DATA, nullptr);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
  WM_main_add_notifier(NC_SPACE | ND_SPACE_VIEW3D, nullptr);
}

}  // namespace blender

#else

namespace blender {

static void rna_def_nurb_body_data(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "NurbBody", "ID");
  RNA_def_struct_sdna(srna, "NurbBody");
  RNA_def_struct_ui_text(srna, "NURB Body", "OCCT-backed NURB body data-block");
  RNA_def_struct_ui_icon(srna, ICON_MESH_CYLINDER);

  prop = RNA_def_property(srna, "primitive", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "primitive");
  RNA_def_property_enum_items(prop, rna_enum_nurb_body_primitive_items);
  RNA_def_property_ui_text(prop, "Primitive", "Base analytic primitive");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "select_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "select_mode");
  RNA_def_property_enum_items(prop, rna_enum_nurb_body_select_mode_items);
  RNA_def_property_ui_text(prop, "Selection Mode", "Object Mode NURB Body selection target");
  RNA_def_property_update(prop, 0, "rna_NurbBody_select_update");

  prop = RNA_def_property(srna, "radius", PROP_FLOAT, PROP_DISTANCE);
  RNA_def_property_float_sdna(prop, nullptr, "radius");
  RNA_def_property_range(prop, 0.001f, 100000.0f);
  RNA_def_property_ui_range(prop, 0.001f, 100.0f, 1.0f, 4);
  RNA_def_property_ui_text(prop, "Radius", "Main cylinder radius");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "depth", PROP_FLOAT, PROP_DISTANCE);
  RNA_def_property_float_sdna(prop, nullptr, "depth");
  RNA_def_property_range(prop, 0.001f, 100000.0f);
  RNA_def_property_ui_range(prop, 0.001f, 100.0f, 1.0f, 4);
  RNA_def_property_ui_text(prop, "Depth", "Main cylinder length");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "use_merge_vertices", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NURB_BODY_MERGE_VERTICES);
  RNA_def_property_ui_text(
      prop,
      "Merge Vertices",
      "Legacy option kept for compatibility; NURB body previews do not merge vertices");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "use_smooth_shading", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NURB_BODY_SMOOTH_SHADING);
  RNA_def_property_ui_text(prop, "Smooth Shading", "Use smooth shading on the viewport preview mesh");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "use_triangulate_mesh", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NURB_BODY_TRIANGULATE_MESH);
  RNA_def_property_ui_text(
      prop, "Triangulate Mesh", "Keep OCCT preview tessellation as triangles");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "use_auto_crease_sharp_edges", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NURB_BODY_AUTO_CREASE_SHARP_EDGES);
  RNA_def_property_ui_text(
      prop,
      "Auto Crease Sharp Edges",
      "Legacy option kept for compatibility; NURB body shading uses NURB-derived custom normals");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "tessellation_deflection", PROP_FLOAT, PROP_DISTANCE);
  RNA_def_property_float_sdna(prop, nullptr, "tessellation_deflection");
  RNA_def_property_range(prop, 0.0001f, 10.0f);
  RNA_def_property_ui_range(prop, 0.0001f, 1.0f, 0.1f, 4);
  RNA_def_property_ui_text(prop, "Deflection", "Preview tessellation linear deflection");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "tessellation_angle", PROP_FLOAT, PROP_ANGLE);
  RNA_def_property_float_sdna(prop, nullptr, "tessellation_angle");
  RNA_def_property_range(prop, 0.01f, 3.14159f);
  RNA_def_property_ui_range(prop, 0.01f, 1.0f, 1.0f, 4);
  RNA_def_property_ui_text(prop, "Angle", "Preview tessellation angular deflection");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

  prop = RNA_def_property(srna, "auto_crease_angle", PROP_FLOAT, PROP_ANGLE);
  RNA_def_property_float_sdna(prop, nullptr, "auto_crease_angle");
  RNA_def_property_range(prop, 0.0f, 3.14159f);
  RNA_def_property_ui_range(prop, 0.0f, 3.14159f, 1.0f, 4);
  RNA_def_property_ui_text(
      prop, "Auto Crease Angle", "Legacy angle value kept for compatibility; no longer used");
  RNA_def_property_update(prop, 0, "rna_NurbBody_update");

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

  rna_def_animdata_common(srna);
}

void RNA_def_nurb_body(BlenderRNA *brna)
{
  rna_def_nurb_body_data(brna);
}

}  // namespace blender

#endif
