# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Panel
from rna_prop_ui import PropertyPanel
from bl_ui.space_properties import PropertiesAnimationMixin


class WorldButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "world"
    # COMPAT_ENGINES must be defined in each subclass, external engines can add themselves here

    @classmethod
    def poll(cls, context):
        return (context.world and context.engine in cls.COMPAT_ENGINES)


class WORLD_PT_context_world(WorldButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    @classmethod
    def poll(cls, context):
        return (context.engine in cls.COMPAT_ENGINES)

    def draw(self, context):
        layout = self.layout

        scene = context.scene
        world = context.world
        space = context.space_data

        if scene:
            layout.template_ID(scene, "world", new="world.new")
        elif world:
            layout.template_ID(space, "pin_id")
class WORLD_PT_animation(WorldButtonsPanel, PropertiesAnimationMixin, PropertyPanel, Panel):
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        # WorldButtonsPanel.poll ensures this is not None.
        world = context.world

        col = layout.column(align=True)
        col.label(text="World")
        self.draw_action_and_slot_selector(context, col, world)

        if node_tree := world.node_tree:
            col = layout.column(align=True)
            col.label(text="Shader Node Tree")
            self.draw_action_and_slot_selector(context, col, node_tree)


class WORLD_PT_viewport_display(WorldButtonsPanel, Panel):
    bl_label = "Viewport Display"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 10

    @classmethod
    def poll(cls, context):
        return context.world

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        world = context.world
        layout.prop(world, "color")


class WORLD_PT_custom_props(WorldButtonsPanel, PropertyPanel, Panel):
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }
    _context_path = "world"
    _property_type = bpy.types.World


classes = (
    WORLD_PT_context_world,
    WORLD_PT_viewport_display,
    WORLD_PT_animation,
    WORLD_PT_custom_props,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
