/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_modifier_types.h"

#include "BKE_modifier.hh"

#include "BLO_read_write.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "BLT_translation.hh"

#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "MOD_ui_common.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *smd = reinterpret_cast<SDFDisplaceModifierData *>(md);
  INIT_DEFAULT_STRUCT_AFTER(smd, modifier);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  BKE_modifier_copydata_generic(md, target, flag);
}

static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  layout.use_property_split_set(true);
  layout.prop(ptr, "noise_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "strength", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "frequency", UI_ITEM_NONE, std::nullopt, ICON_NONE);

  int ntype = RNA_enum_get(ptr, "noise_type");
  bool has_fbm = (ntype == MOD_SDF_DISPLACE_NOISE || ntype == MOD_SDF_DISPLACE_VORONOI);
  ui::Layout &col = layout.column(false);
  col.active_set(has_fbm);
  col.prop(ptr, "octaves", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  col.prop(ptr, "lacunarity", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  col.prop(ptr, "roughness", UI_ITEM_NONE, std::nullopt, ICON_NONE);

  modifier_error_message_draw(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_SDFDisplace, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *smd = reinterpret_cast<const SDFDisplaceModifierData *>(md);
  writer->write_struct(smd);
}

static void blend_read(BlendDataReader * /*reader*/, ModifierData * /*md*/) {}

ModifierTypeInfo modifierType_SDFDisplace = {
    /*idname*/ "SDFDisplace",
    /*name*/ N_("SDF Displacement"),
    /*struct_name*/ "SDFDisplaceModifierData",
    /*struct_size*/ sizeof(SDFDisplaceModifierData),
    /*srna*/ &RNA_SDFDisplaceModifier,
    /*type*/ ModifierTypeType::NonGeometrical,
    /*flags*/ eModifierTypeFlag_AcceptsSDF,
    /*icon*/ ICON_MOD_DISPLACE,

    /*copy_data*/ copy_data,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ nullptr,

    /*init_data*/ init_data,
    /*required_data_mask*/ nullptr,
    /*free_data*/ nullptr,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ nullptr,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ nullptr,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ panel_register,
    /*blend_write*/ blend_write,
    /*blend_read*/ blend_read,
};

}  // namespace blender
