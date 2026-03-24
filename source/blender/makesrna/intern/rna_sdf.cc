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

#include "DNA_sdf_types.h"

using namespace blender;

#include "BLI_math_base.h"

#ifdef RNA_RUNTIME

#  include "BKE_main.hh"

#  include "DEG_depsgraph.hh"

#  include "WM_api.hh"
#  include "WM_types.hh"

static void rna_SDF_update(Main * /*bmain*/, Scene * /*scene*/, PointerRNA *ptr)
{
  DEG_id_tag_update(ptr->owner_id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
}

#else

static const EnumPropertyItem rna_enum_sdf_type_items[] = {
    {SDF_TYPE_BOX, "BOX", 0, "Box", "Box primitive"},
    {SDF_TYPE_SPHERE, "SPHERE", 0, "Sphere", "Sphere primitive"},
    {SDF_TYPE_CAPSULE, "CAPSULE", 0, "Capsule", "Capsule primitive"},
    {SDF_TYPE_TORUS, "TORUS", 0, "Torus", "Torus primitive"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_blend_type_items[] = {
    {SDF_BLEND_LINEAR, "LINEAR", 0, "Linear", "Hard union/difference"},
    {SDF_BLEND_SMOOTH, "SMOOTH", 0, "Smooth", "Smooth blend"},
    {SDF_BLEND_CHAMFER, "CHAMFER", 0, "Chamfer", "Chamfer blend"},
    {SDF_BLEND_ROUND, "ROUND", 0, "Round", "Round blend"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_sdf_csg_items[] = {
    {SDF_CSG_UNION, "UNION", 0, "Union", "Boolean union"},
    {SDF_CSG_SUBTRACT, "SUBTRACT", 0, "Subtract", "Boolean subtraction"},
    {SDF_CSG_INTERSECT, "INTERSECT", 0, "Intersect", "Boolean intersection"},
    {0, nullptr, 0, nullptr, nullptr},
};

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
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Size */
  prop = RNA_def_property(srna, "size", PROP_FLOAT, PROP_XYZ);
  RNA_def_property_float_sdna(prop, nullptr, "size");
  RNA_def_property_array(prop, 3);
  RNA_def_property_range(prop, 0.001f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.001f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Size", "Size in each axis");
  RNA_def_property_update(prop, 0, "rna_SDF_update");

  /* Bevel */
  prop = RNA_def_property(srna, "bevel", PROP_FLOAT, PROP_DISTANCE);
  RNA_def_property_float_sdna(prop, nullptr, "bevel");
  RNA_def_property_range(prop, 0.0f, FLT_MAX);
  RNA_def_property_ui_range(prop, 0.0f, 5.0f, 1.0f, 3);
  RNA_def_property_ui_text(prop, "Bevel", "Bevel radius");
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

  /* Animation data */
  rna_def_animdata_common(srna);
}

namespace blender {

void RNA_def_sdf(BlenderRNA *brna)
{
  rna_def_sdf(brna);
}

}  // namespace blender

#endif
