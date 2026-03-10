# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Panel, Menu, Operator


# -- Pie Menus ----------------------------------------------------------------

class SDF_MT_csg_pie(Menu):
    bl_idname = "SDF_MT_csg_pie"
    bl_label = "CSG Operation"

    def draw(self, context):
        pie = self.layout.menu_pie()
        sdf = context.object.data
        pie.prop_enum(sdf, "csg_operation", value='UNION')
        pie.prop_enum(sdf, "csg_operation", value='SUBTRACT')
        pie.prop_enum(sdf, "csg_operation", value='INTERSECT')
        pie.prop_enum(sdf, "csg_operation", value='SHELL')


class SDF_MT_blend_pie(Menu):
    bl_idname = "SDF_MT_blend_pie"
    bl_label = "Blend Type"

    def draw(self, context):
        pie = self.layout.menu_pie()
        sdf = context.object.data
        pie.prop_enum(sdf, "blend_type", value='LINEAR')
        pie.prop_enum(sdf, "blend_type", value='SMOOTH')
        pie.prop_enum(sdf, "blend_type", value='CHAMFER')
        pie.prop_enum(sdf, "blend_type", value='ROUND')


class SDF_OT_csg_pie_call(Operator):
    """Open the CSG operation pie menu (Tab)"""
    bl_idname = "sdf.csg_pie_call"
    bl_label = "SDF CSG Pie"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob is not None and ob.type == 'SDF'

    def execute(self, context):
        bpy.ops.wm.call_menu_pie(name="SDF_MT_csg_pie")
        return {'FINISHED'}


class SDF_OT_blend_pie_call(Operator):
    """Open the blend type pie menu (Shift+Tab)"""
    bl_idname = "sdf.blend_pie_call"
    bl_label = "SDF Blend Pie"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob is not None and ob.type == 'SDF'

    def execute(self, context):
        bpy.ops.wm.call_menu_pie(name="SDF_MT_blend_pie")
        return {'FINISHED'}


# -- Properties Panels --------------------------------------------------------

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
        sdf = context.sdf

        # Shape type — full-width icon-only buttons
        row = layout.row(align=True)
        row.scale_y = 1.6
        row.prop(sdf, "sdf_type", expand=True, icon_only=True)

        layout.separator()

        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.prop(sdf, "size")
        layout.prop(sdf, "bevel")
        layout.prop(sdf, "color")


class DATA_PT_sdf_operation(SDFButtonsPanel, Panel):
    bl_label = "Operation"

    def draw(self, context):
        layout = self.layout
        layout.use_property_decorate = False

        sdf = context.sdf

        # CSG operation — full-width icon-only buttons
        row = layout.row(align=True)
        row.scale_y = 1.6
        row.prop(sdf, "csg_operation", expand=True, icon_only=True)

        layout.separator()

        # Blend type — full-width icon-only buttons
        row = layout.row(align=True)
        row.scale_y = 1.6
        row.prop(sdf, "blend_type", expand=True, icon_only=True)

        layout.separator()

        # Shell distance — only visible when CSG is Shell
        if sdf.csg_operation == 'SHELL':
            col = layout.column(align=True)
            col.label(text="Shell Thickness")
            col.prop(sdf, "shell_distance", text="")

        # Blend amount — disabled for Linear (no blending needed)
        col = layout.column(align=True)
        col.label(text="Blend")
        sub = col.row()
        sub.enabled = (sdf.blend_type != 'LINEAR')
        sub.prop(sdf, "blend", text="")


# -- Registration -------------------------------------------------------------

classes = (
    SDF_MT_csg_pie,
    SDF_MT_blend_pie,
    SDF_OT_csg_pie_call,
    SDF_OT_blend_pie_call,
    DATA_PT_context_sdf,
    DATA_PT_sdf_shape,
    DATA_PT_sdf_operation,
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
