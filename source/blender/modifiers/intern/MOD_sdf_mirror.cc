/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

#include "BKE_lib_query.hh"
#include "BKE_modifier.hh"

#include "BLO_read_write.hh"

#include "DEG_depsgraph_build.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "BLT_translation.hh"

#include "WM_types.hh"

#include "RNA_prototypes.hh"

#include "MOD_ui_common.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *smd = reinterpret_cast<SDFMirrorModifierData *>(md);
  INIT_DEFAULT_STRUCT_AFTER(smd, modifier);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  BKE_modifier_copydata_generic(md, target, flag);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *smd = reinterpret_cast<SDFMirrorModifierData *>(md);
  walk(user_data, ob, reinterpret_cast<ID **>(&smd->mirror_object), IDWALK_CB_NOP);
}

static void update_depsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  auto *smd = reinterpret_cast<SDFMirrorModifierData *>(md);
  if (smd->mirror_object != nullptr) {
    DEG_add_object_relation(
        ctx->node, smd->mirror_object, DEG_OB_COMP_TRANSFORM, "SDF Mirror Modifier");
    DEG_add_depends_on_transform_relation(ctx->node, "SDF Mirror Modifier");
  }
}

static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  ui::Layout &layout = *panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);
  const ui::eUI_Item_Flag toggles_flag = ui::ITEM_R_TOGGLE | ui::ITEM_R_FORCE_BLANK_DECORATE;

  layout.use_property_split_set(true);

  ui::Layout &row = layout.row(true, IFACE_("Axis"));
  row.prop(ptr, "use_axis_x", toggles_flag, std::nullopt, ICON_NONE);
  row.prop(ptr, "use_axis_y", toggles_flag, std::nullopt, ICON_NONE);
  row.prop(ptr, "use_axis_z", toggles_flag, std::nullopt, ICON_NONE);

  layout.prop(ptr, "offset_distance", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "mirror_object", UI_ITEM_NONE, std::nullopt, ICON_NONE);

  layout.prop(ptr, "blend_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "blend", UI_ITEM_NONE, std::nullopt, ICON_NONE);

  modifier_error_message_draw(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_SDFMirror, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *smd = reinterpret_cast<const SDFMirrorModifierData *>(md);
  writer->write_struct(smd);
}

static void blend_read(BlendDataReader * /*reader*/, ModifierData * /*md*/) {}

ModifierTypeInfo modifierType_SDFMirror = {
    /*idname*/ "SDFMirror",
    /*name*/ N_("SDF Mirror"),
    /*struct_name*/ "SDFMirrorModifierData",
    /*struct_size*/ sizeof(SDFMirrorModifierData),
    /*srna*/ &RNA_SDFMirrorModifier,
    /*type*/ ModifierTypeType::NonGeometrical,
    /*flags*/ eModifierTypeFlag_AcceptsSDF,
    /*icon*/ ICON_MOD_MIRROR,

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
    /*update_depsgraph*/ update_depsgraph,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ panel_register,
    /*blend_write*/ blend_write,
    /*blend_read*/ blend_read,
};

}  // namespace blender
