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
  auto *smd = reinterpret_cast<SDFArrayModifierData *>(md);
  INIT_DEFAULT_STRUCT_AFTER(smd, modifier);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  BKE_modifier_copydata_generic(md, target, flag);
}

/* Main panel */
static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  layout.use_property_split_set(true);

  layout.prop(ptr, "array_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "count", UI_ITEM_NONE, std::nullopt, ICON_NONE);

  int array_type = RNA_enum_get(ptr, "array_type");
  if (array_type == MOD_SDF_ARRAY_RADIAL) {
    layout.prop(ptr, "array_radius", UI_ITEM_NONE, std::nullopt, ICON_NONE);
    layout.prop(ptr, "rotation_offset", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  }

  modifier_error_message_draw(layout, ptr);
}

/* Relative Offset sub-panel */
static void relative_offset_header_draw(const bContext * /*C*/, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  layout.prop(ptr, "use_relative_offset", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

static void relative_offset_draw(const bContext * /*C*/, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  layout.use_property_split_set(true);

  ui::Layout &col = layout.column(false);
  col.active_set(RNA_boolean_get(ptr, "use_relative_offset"));
  col.prop(ptr, "relative_offset", UI_ITEM_NONE, IFACE_("Factor"), ICON_NONE);
}

/* Constant Offset sub-panel */
static void constant_offset_header_draw(const bContext * /*C*/, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  layout.prop(ptr, "use_constant_offset", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

static void constant_offset_draw(const bContext * /*C*/, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  layout.use_property_split_set(true);

  ui::Layout &col = layout.column(false);
  col.active_set(RNA_boolean_get(ptr, "use_constant_offset"));
  col.prop(ptr, "constant_offset", UI_ITEM_NONE, IFACE_("Distance"), ICON_NONE);
}

/* Blend sub-panel */
static void blend_panel_draw(const bContext * /*C*/, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  layout.use_property_split_set(true);

  layout.prop(ptr, "blend_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  ui::Layout &col = layout.column(false);
  col.active_set(RNA_enum_get(ptr, "blend_type") != MOD_SDF_BLEND_LINEAR);
  col.prop(ptr, "blend", UI_ITEM_NONE, IFACE_("Radius"), ICON_NONE);
}

static void panel_register(ARegionType *region_type)
{
  PanelType *panel_type = modifier_panel_register(
      region_type, eModifierType_SDFArray, panel_draw);
  modifier_subpanel_register(region_type,
                             "relative_offset",
                             "",
                             relative_offset_header_draw,
                             relative_offset_draw,
                             panel_type);
  modifier_subpanel_register(region_type,
                             "constant_offset",
                             "",
                             constant_offset_header_draw,
                             constant_offset_draw,
                             panel_type);
  modifier_subpanel_register(
      region_type, "blend", "Blend", nullptr, blend_panel_draw, panel_type);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *smd = reinterpret_cast<const SDFArrayModifierData *>(md);
  writer->write_struct(smd);
}

static void blend_read(BlendDataReader * /*reader*/, ModifierData * /*md*/) {}

ModifierTypeInfo modifierType_SDFArray = {
    /*idname*/ "SDFArray",
    /*name*/ N_("SDF Array"),
    /*struct_name*/ "SDFArrayModifierData",
    /*struct_size*/ sizeof(SDFArrayModifierData),
    /*srna*/ &RNA_SDFArrayModifier,
    /*type*/ ModifierTypeType::NonGeometrical,
    /*flags*/ eModifierTypeFlag_AcceptsSDF,
    /*icon*/ ICON_MOD_ARRAY,

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
