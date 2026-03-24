# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Panel, Menu, Operator
from bpy.props import EnumProperty, IntProperty


class SDFGroupButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        if context.sdf is not None:
            return False
        return getattr(context, 'sdf_group', None) is not None


class DATA_PT_context_sdf_group(SDFGroupButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf
        grp = context.sdf_group

        if sdf and grp:
            layout.template_ID(sdf, "sdf_group")
        elif grp:
            space = context.space_data
            layout.template_ID(space, "pin_id")


class DATA_PT_sdf_group_operation(SDFGroupButtonsPanel, Panel):
    bl_label = "Group Operation"

    def draw(self, context):
        layout = self.layout
        grp = context.sdf_group

        layout.label(text="CSG Operation")
        row = layout.row(align=True)
        row.scale_y = 1.4
        row.prop(grp, "csg_operation", expand=True, icon_only=True)

        layout.separator()

        if grp.csg_operation == 'SHELL':
            layout.label(text="Shell Mode")
            layout.prop(grp, "shell_mode", text="")
            layout.label(text="Shell Op")
            layout.prop(grp, "shell_op", text="")

        layout.label(text="Blend Type")
        row = layout.row(align=True)
        row.scale_y = 1.4
        row.prop(grp, "blend_type", expand=True, icon_only=True)

        if grp.blend_type != 'LINEAR':
            if grp.csg_operation == 'SHELL':
                layout.label(text="Blend Top")
                layout.prop(grp, "shell_blend_top", text="")
                layout.label(text="Blend Bottom")
                layout.prop(grp, "shell_blend_bottom", text="")
            else:
                layout.label(text="Blend")
                layout.prop(grp, "blend", text="")

            if grp.blend_type in ('CHAMFER', 'ROUND'):
                layout.label(text="Edge Smoothness")
                row = layout.row(align=True)
                row.prop(grp, "chamfer_k2", text="K2")
                row.prop(grp, "chamfer_k3", text="K3")

        if grp.csg_operation == 'SHELL':
            layout.label(text="Distance")
            layout.prop(grp, "shell_distance", text="")


class DATA_PT_sdf_group_members(SDFGroupButtonsPanel, Panel):
    bl_label = "Members"

    def draw(self, context):
        layout = self.layout
        grp = context.sdf_group

        if not grp.members:
            layout.label(text="No members")
            return

        for idx, member in enumerate(grp.members):
            row = layout.row(align=True)
            row.label(text=f"{idx + 1}.")
            if member.object:
                row.label(text=member.object.name, icon='OUTLINER_OB_SDF')
            else:
                row.label(text="(empty)", icon='ERROR')

            sub = row.row(align=True)
            sub.scale_x = 0.8
            op = sub.operator("object.sdf_group_reorder", text="", icon='TRIA_UP')
            op.member_index = idx
            op.direction = -1
            op.group_name = grp.name
            op = sub.operator("object.sdf_group_reorder", text="", icon='TRIA_DOWN')
            op.member_index = idx
            op.direction = 1
            op.group_name = grp.name
            op = sub.operator("object.sdf_group_remove_member", text="", icon='X')
            op.member_index = idx
            op.group_name = grp.name


# Group Modifier Operators

class SDFGROUP_OT_modifier_add(Operator):
    """Add an SDF modifier to the group"""
    bl_idname = "sdf_group.modifier_add"
    bl_label = "Add Group Modifier"
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
            ('ARRAY', "Array", "Duplicate geometry", 'MOD_ARRAY', 8),
        ],
    )

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf_group', None) is not None

    def execute(self, context):
        grp = getattr(context, 'sdf_group', None)
        if grp is None:
            return {'CANCELLED'}
        grp.modifiers.new(type=self.type)
        return {'FINISHED'}


class SDFGROUP_OT_modifier_remove(Operator):
    """Remove an SDF modifier from the group"""
    bl_idname = "sdf_group.modifier_remove"
    bl_label = "Remove Group Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    index: IntProperty()

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf_group', None) is not None

    def execute(self, context):
        grp = getattr(context, 'sdf_group', None)
        if grp is None:
            return {'CANCELLED'}
        if self.index < len(grp.modifiers):
            mod = grp.modifiers[self.index]
            if mod.type == 'MIRROR' and mod.mirror_object:
                mirror_ob = mod.mirror_object
                mod.mirror_object = None
                try:
                    bpy.data.objects.remove(mirror_ob)
                except RuntimeError:
                    pass
            grp.modifiers.remove(mod)
        return {'FINISHED'}


class SDFGROUP_OT_modifier_move(Operator):
    """Move a group modifier up or down"""
    bl_idname = "sdf_group.modifier_move"
    bl_label = "Move Group Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    index: IntProperty()
    direction: IntProperty()

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf_group', None) is not None

    def execute(self, context):
        grp = getattr(context, 'sdf_group', None)
        if grp is None:
            return {'CANCELLED'}
        new_index = self.index + self.direction
        if 0 <= new_index < len(grp.modifiers):
            grp.modifiers.move(self.index, new_index)
        return {'FINISHED'}


class SDFGROUP_MT_modifier_add(Menu):
    bl_idname = "SDFGROUP_MT_modifier_add"
    bl_label = "Add Group Modifier"

    def draw(self, _context):
        layout = self.layout
        layout.operator("sdf_group.modifier_add", text="Mirror", icon='MOD_MIRROR').type = 'MIRROR'
        layout.operator("sdf_group.modifier_add", text="Twist", icon='MOD_SIMPLEDEFORM').type = 'TWIST'
        layout.operator("sdf_group.modifier_add", text="Bend", icon='MOD_SIMPLEDEFORM').type = 'BEND'
        layout.operator("sdf_group.modifier_add", text="Elongate", icon='MOD_LENGTH').type = 'ELONGATE'
        layout.separator()
        layout.operator("sdf_group.modifier_add", text="Hollow", icon='MOD_SOLIDIFY').type = 'HOLLOW'
        layout.operator("sdf_group.modifier_add", text="Round", icon='MOD_SMOOTH').type = 'ROUND'
        layout.operator("sdf_group.modifier_add", text="Onion", icon='MOD_SOLIDIFY').type = 'ONION'
        layout.separator()
        layout.operator("sdf_group.modifier_add", text="Bevel", icon='MOD_BEVEL').type = 'BEVEL'
        layout.operator("sdf_group.modifier_add", text="Array", icon='MOD_ARRAY').type = 'ARRAY'


class DATA_PT_sdf_group_modifiers(SDFGroupButtonsPanel, Panel):
    bl_label = "Modifiers"

    def draw(self, context):
        layout = self.layout
        grp = context.sdf_group

        row = layout.row()
        row.menu("SDFGROUP_MT_modifier_add", text="Add Modifier", icon='ADD')

        if not grp.modifiers:
            return

        for idx, mod in enumerate(grp.modifiers):
            box = layout.box()
            row = box.row(align=True)
            row.prop(mod, "show_viewport", text="",
                     icon='RESTRICT_VIEW_OFF' if mod.show_viewport else 'RESTRICT_VIEW_ON')
            row.prop(mod, "name", text="")

            sub = row.row(align=True)
            sub.scale_x = 0.8
            op = sub.operator("sdf_group.modifier_move", text="", icon='TRIA_UP')
            op.index = idx
            op.direction = -1
            op = sub.operator("sdf_group.modifier_move", text="", icon='TRIA_DOWN')
            op.index = idx
            op.direction = 1
            op = row.operator("sdf_group.modifier_remove", text="", icon='X')
            op.index = idx

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
                col.prop(mod, "offset_distance")
                col.prop(mod, "mirror_object")

                box_csg = box.box()
                box_csg.label(text="Mirror Blending:")
                row = box_csg.row(align=True)
                row.scale_y = 1.2
                row.prop(mod, "blend_type", expand=True, icon_only=True)
                sub = box_csg.row()
                sub.enabled = (mod.blend_type != 'LINEAR')
                sub.prop(mod, "mirror_blend", text="Radius")

            elif mod.type == 'TWIST':
                col.prop(mod, "strength", text="Strength")
            elif mod.type == 'BEND':
                col.prop(mod, "strength", text="Strength")
                col.prop(mod, "bend_axis", text="Axis")
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
            elif mod.type == 'ARRAY':
                col.prop(mod, "array_type")
                col.prop(mod, "count")
                if mod.array_type == 'LINEAR':
                    col.prop(mod, "offset")
                elif mod.array_type == 'RADIAL':
                    col.prop(mod, "array_radius")
                    col.prop(mod, "rotation_offset")

                box_csg = box.box()
                box_csg.label(text="Array Blending:")
                row = box_csg.row(align=True)
                row.scale_y = 1.2
                row.prop(mod, "blend_type", expand=True, icon_only=True)
                sub = box_csg.row()
                sub.enabled = (mod.blend_type != 'LINEAR')
                sub.prop(mod, "array_blend", text="Radius")


class DATA_PT_sdf_group_display(SDFGroupButtonsPanel, Panel):
    bl_label = "Global Tint"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        grp = context.sdf_group
        layout.prop(grp, "color")


classes = (
    SDFGROUP_OT_modifier_add,
    SDFGROUP_OT_modifier_remove,
    SDFGROUP_OT_modifier_move,
    SDFGROUP_MT_modifier_add,
    DATA_PT_context_sdf_group,
    DATA_PT_sdf_group_operation,
    DATA_PT_sdf_group_members,
    DATA_PT_sdf_group_modifiers,
    DATA_PT_sdf_group_display,
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
