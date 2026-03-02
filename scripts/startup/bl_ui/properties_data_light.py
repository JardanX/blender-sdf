# SPDX-FileCopyrightText: 2018-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.app.translations import contexts as i18n_contexts
from bpy.types import Panel
from rna_prop_ui import PropertyPanel
from bl_ui.space_properties import PropertiesAnimationMixin


class DataButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        engine = context.engine
        return context.light and (engine in cls.COMPAT_ENGINES)


class DATA_PT_context_light(DataButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    def draw(self, context):
        layout = self.layout

        ob = context.object
        light = context.light
        space = context.space_data

        if ob:
            layout.template_ID(ob, "data")
        elif light:
            layout.template_ID(space, "pin_id")


class DATA_PT_preview(DataButtonsPanel, Panel):
    bl_label = "Preview"
    bl_options = {'DEFAULT_CLOSED'}
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    def draw(self, context):
        self.layout.template_preview(context.light)


class DATA_PT_light(DataButtonsPanel, Panel):
    bl_label = "Light"
    COMPAT_ENGINES = {'BLENDER_RENDER', 'BLENDER_PROXIMITY'}

    def draw(self, context):
        layout = self.layout

        light = context.light

        # Compact layout for node editor.
        if self.bl_space_type == 'PROPERTIES':
            layout.row().prop(light, "type", expand=True)
            layout.use_property_split = True
        else:
            layout.use_property_split = True
            layout.row().prop(light, "type")


class DATA_PT_spot(DataButtonsPanel, Panel):
    bl_label = "Spot Shape"
    bl_parent_id = "DATA_PT_light"
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    @classmethod
    def poll(cls, context):
        light = context.light
        engine = context.engine
        return (light and light.type == 'SPOT') and (engine in cls.COMPAT_ENGINES)

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        light = context.light

        col = layout.column()

        col.prop(light, "spot_size", text="Angle")
        col.prop(light, "spot_blend", text="Blend", slider=True)

        col.prop(light, "show_cone")


class DATA_PT_light_animation(DataButtonsPanel, PropertiesAnimationMixin, PropertyPanel, Panel):
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        # DataButtonsPanel.poll ensures this is not None.
        light = context.light

        col = layout.column(align=True)
        col.label(text="Light")
        self.draw_action_and_slot_selector(context, col, light)

        if node_tree := light.node_tree:
            col = layout.column(align=True)
            col.label(text="Shader Node Tree")
            self.draw_action_and_slot_selector(context, col, node_tree)


class DATA_PT_custom_props_light(DataButtonsPanel, PropertyPanel, Panel):
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }
    _context_path = "object.data"
    _property_type = bpy.types.Light


classes = (
    DATA_PT_context_light,
    DATA_PT_preview,
    DATA_PT_light,
    DATA_PT_spot,
    DATA_PT_light_animation,
    DATA_PT_custom_props_light,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
