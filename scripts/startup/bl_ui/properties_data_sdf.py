# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

from bpy.types import Panel


class SDFButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        return context.sdf


class DATA_PT_context_sdf(SDFButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}

    def draw(self, context):
        layout = self.layout

        ob = context.object
        sdf = context.sdf
        space = context.space_data

        if ob:
            layout.template_ID(ob, "data")
        elif sdf:
            layout.template_ID(space, "pin_id")


class DATA_PT_sdf_shape(SDFButtonsPanel, Panel):
    bl_label = "Shape"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        sdf = context.sdf

        layout.prop(sdf, "sdf_type", text="Type")
        layout.prop(sdf, "size")
        layout.prop(sdf, "bevel")
        layout.prop(sdf, "color")


class DATA_PT_sdf_blending(SDFButtonsPanel, Panel):
    bl_label = "Blending"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        sdf = context.sdf

        layout.prop(sdf, "blend")
        layout.prop(sdf, "blend_type")
        layout.prop(sdf, "csg_operation")


classes = (
    DATA_PT_context_sdf,
    DATA_PT_sdf_shape,
    DATA_PT_sdf_blending,
)


def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)


def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)


if __name__ == "__main__":
    register()
