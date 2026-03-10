# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Panel, Menu, Operator
from bpy.props import EnumProperty, IntProperty


# -- Pie Menus ----------------------------------------------------------------

class SDF_MT_csg_pie(Menu):
    bl_idname = "SDF_MT_csg_pie"
    bl_label = "CSG Operation"

    def draw(self, context):
        pie = self.layout.menu_pie()
        sdf = context.object.data
        pie.prop_enum(sdf, "csg_operation", value='UNION')       # W
        pie.prop_enum(sdf, "csg_operation", value='SUBTRACT')    # E
        pie.separator()                                           # S (skip)
        pie.separator()                                           # N (skip)
        pie.prop_enum(sdf, "csg_operation", value='INTERSECT')   # NW
        pie.prop_enum(sdf, "csg_operation", value='SHELL')       # NE
        pie.prop_enum(sdf, "csg_operation", value='PUSH')        # SW
        pie.prop_enum(sdf, "csg_operation", value='AVOID')       # SE
        # Layout: 3 left (NW=Intersect, W=Union, SW=Push)
        #         3 right (NE=Shell, E=Subtract, SE=Avoid)


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
        layout.prop(sdf, "color")


class DATA_PT_sdf_operation(SDFButtonsPanel, Panel):
    bl_label = "Operation"

    def draw(self, context):
        layout = self.layout
        layout.use_property_decorate = False

        sdf = context.sdf

        # CSG operation — 2 rows of 3
        layout.label(text="Operation")
        col = layout.column(align=True)
        row = col.row(align=True)
        row.scale_y = 1.6
        row.prop_enum(sdf, "csg_operation", "UNION", text="")
        row.prop_enum(sdf, "csg_operation", "SUBTRACT", text="")
        row.prop_enum(sdf, "csg_operation", "INTERSECT", text="")
        row = col.row(align=True)
        row.scale_y = 1.6
        row.prop_enum(sdf, "csg_operation", "SHELL", text="")
        row.prop_enum(sdf, "csg_operation", "PUSH", text="")
        row.prop_enum(sdf, "csg_operation", "AVOID", text="")

        layout.separator()

        # Blend type — single row of 4
        layout.label(text="Blend Type")
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


# -- Shape Property Panel (box-specific) --------------------------------------

class DATA_PT_sdf_property(SDFButtonsPanel, Panel):
    bl_label = "Property"

    @classmethod
    def poll(cls, context):
        sdf = context.sdf
        return sdf and sdf.sdf_type == 'BOX'

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf

        # Corner Bevels
        layout.label(text="Corner Bevels")
        col = layout.column(align=True)
        row = col.row(align=True)
        row.prop(sdf, "box_corners", index=0, text="1")
        row.prop(sdf, "box_corners", index=1, text="2")
        row = col.row(align=True)
        row.prop(sdf, "box_corners", index=2, text="3")
        row.prop(sdf, "box_corners", index=3, text="4")

        layout.separator()

        # Edge Chamfer
        layout.label(text="Edge Chamfer")
        col = layout.column(align=True)
        row = col.row(align=True)
        row.prop(sdf, "box_edge_top", text="Top")
        row.prop(sdf, "box_edge_bottom", text="Bottom")

        layout.separator()

        # Taper
        layout.prop(sdf, "box_taper")

        # Mode dropdowns — only show when any shape property is active
        corners = sdf.box_corners
        has_shape = (corners[0] + corners[1] + corners[2] + corners[3]
                     + sdf.box_edge_top + sdf.box_edge_bottom
                     + abs(sdf.box_taper)) > 0.001
        if has_shape:
            layout.separator()
            row = layout.row(align=True)
            row.prop(sdf, "box_corner_mode", text="Corners")
            row.prop(sdf, "box_edge_mode", text="Edges")


# -- Modifier Operators -------------------------------------------------------

class SDF_OT_modifier_add(Operator):
    """Add an SDF modifier"""
    bl_idname = "sdf.modifier_add"
    bl_label = "Add SDF Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    type: EnumProperty(
        name="Type",
        items=[
            ('MIRROR', "Mirror", "Mirror across axes", 'MOD_MIRROR', 0),
            ('TWIST', "Twist", "Twist around Z axis", 'MOD_SIMPLEDEFORM', 1),
            ('BEND', "Bend", "Bend around an axis", 'MOD_SIMPLEDEFORM', 2),
            ('ELONGATE', "Elongate", "Stretch along axes", 'MOD_LENGTH', 3),
            ('HOLLOW', "Hollow", "Make hollow with wall thickness", 'MOD_SOLIDIFY', 4),
            ('ROUND', "Round", "Additional rounding", 'MOD_SMOOTH', 5),
            ('ONION', "Onion", "Concentric shells", 'MOD_SOLIDIFY', 6),
            ('BEVEL', "Bevel", "Bevel/round edges", 'MOD_BEVEL', 7),
        ],
    )

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf', None) is not None

    def execute(self, context):
        sdf = getattr(context, 'sdf', None)
        if sdf is None:
            return {'CANCELLED'}
        sdf.modifiers.new(type=self.type)
        return {'FINISHED'}


class SDF_OT_modifier_remove(Operator):
    """Remove an SDF modifier"""
    bl_idname = "sdf.modifier_remove"
    bl_label = "Remove SDF Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    index: IntProperty()

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf', None) is not None

    def execute(self, context):
        sdf = getattr(context, 'sdf', None)
        if sdf is None:
            return {'CANCELLED'}
        if self.index < len(sdf.modifiers):
            sdf.modifiers.remove(sdf.modifiers[self.index])
        return {'FINISHED'}


class SDF_OT_modifier_move(Operator):
    """Move an SDF modifier up or down"""
    bl_idname = "sdf.modifier_move"
    bl_label = "Move SDF Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    index: IntProperty()
    direction: IntProperty()  # -1 = up, 1 = down

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf', None) is not None

    def execute(self, context):
        sdf = getattr(context, 'sdf', None)
        if sdf is None:
            return {'CANCELLED'}
        new_index = self.index + self.direction
        if 0 <= new_index < len(sdf.modifiers):
            sdf.modifiers.move(self.index, new_index)
        return {'FINISHED'}


# -- Modifier Panel -----------------------------------------------------------

class DATA_PT_sdf_modifiers(SDFButtonsPanel, Panel):
    bl_label = "Modifiers"

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf

        # Add modifier dropdown
        row = layout.row()
        row.menu("SDF_MT_modifier_add", text="Add Modifier", icon='ADD')

        if not sdf.modifiers:
            return

        for idx, mod in enumerate(sdf.modifiers):
            box = layout.box()
            # Header row: icon, name, enabled, move up/down, remove
            row = box.row(align=True)
            row.prop(mod, "show_viewport", text="", icon='RESTRICT_VIEW_OFF' if mod.show_viewport else 'RESTRICT_VIEW_ON')
            row.prop(mod, "name", text="")

            sub = row.row(align=True)
            sub.scale_x = 0.8
            op = sub.operator("sdf.modifier_move", text="", icon='TRIA_UP')
            op.index = idx
            op.direction = -1
            op = sub.operator("sdf.modifier_move", text="", icon='TRIA_DOWN')
            op.index = idx
            op.direction = 1
            op = row.operator("sdf.modifier_remove", text="", icon='X')
            op.index = idx

            # Type-specific parameters
            if not mod.show_viewport:
                continue

            col = box.column(align=True)
            col.use_property_split = True
            col.use_property_decorate = False

            if mod.type == 'MIRROR':
                row = col.row(align=True)
                row.label(text="Axes:")
                row.prop(mod, "use_mirror_x", toggle=True)
                row.prop(mod, "use_mirror_y", toggle=True)
                row.prop(mod, "use_mirror_z", toggle=True)
            elif mod.type == 'TWIST':
                col.prop(mod, "strength", text="Strength")
            elif mod.type == 'BEND':
                col.prop(mod, "strength", text="Strength")
            elif mod.type == 'ELONGATE':
                col.prop(mod, "elongation")
            elif mod.type == 'HOLLOW':
                col.prop(mod, "thickness")
            elif mod.type == 'ROUND':
                col.prop(mod, "radius")
            elif mod.type == 'ONION':
                col.prop(mod, "thickness")
            elif mod.type == 'BEVEL':
                col.prop(mod, "radius")


class SDF_MT_modifier_add(Menu):
    bl_idname = "SDF_MT_modifier_add"
    bl_label = "Add SDF Modifier"

    def draw(self, _context):
        layout = self.layout
        layout.operator("sdf.modifier_add", text="Mirror", icon='MOD_MIRROR').type = 'MIRROR'
        layout.operator("sdf.modifier_add", text="Twist", icon='MOD_SIMPLEDEFORM').type = 'TWIST'
        layout.operator("sdf.modifier_add", text="Bend", icon='MOD_SIMPLEDEFORM').type = 'BEND'
        layout.operator("sdf.modifier_add", text="Elongate", icon='MOD_LENGTH').type = 'ELONGATE'
        layout.separator()
        layout.operator("sdf.modifier_add", text="Hollow", icon='MOD_SOLIDIFY').type = 'HOLLOW'
        layout.operator("sdf.modifier_add", text="Round", icon='MOD_SMOOTH').type = 'ROUND'
        layout.operator("sdf.modifier_add", text="Onion", icon='MOD_SOLIDIFY').type = 'ONION'
        layout.separator()
        layout.operator("sdf.modifier_add", text="Bevel", icon='MOD_BEVEL').type = 'BEVEL'


# -- Registration -------------------------------------------------------------

classes = (
    SDF_MT_csg_pie,
    SDF_MT_blend_pie,
    SDF_OT_csg_pie_call,
    SDF_OT_blend_pie_call,
    SDF_OT_modifier_add,
    SDF_OT_modifier_remove,
    SDF_OT_modifier_move,
    SDF_MT_modifier_add,
    DATA_PT_context_sdf,
    DATA_PT_sdf_shape,
    DATA_PT_sdf_operation,
    DATA_PT_sdf_property,
    DATA_PT_sdf_modifiers,
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
