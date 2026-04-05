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
        grid = layout.grid_flow(row_major=True, columns=4, even_columns=True, even_rows=True, align=True)
        grid.scale_y = 1.4
        for item in grp.bl_rna.properties["csg_operation"].enum_items:
            grid.prop_enum(grp, "csg_operation", item.identifier, text="")

        layout.separator()

        layout.label(text="Blend Type")
        grid = layout.grid_flow(row_major=True, columns=4, even_columns=True, even_rows=True, align=True)
        grid.scale_y = 1.4
        for item in grp.bl_rna.properties["blend_type"].enum_items:
            grid.prop_enum(grp, "blend_type", item.identifier, text="")

        is_shell = (grp.csg_operation == 'SHELL')
        bt = grp.blend_type

        if is_shell:
            row = layout.row(align=True)
            row.prop(grp, "shell_distance", text="Distance")
            sub = row.row(align=True)
            sub.scale_x = 0.9
            sub.enabled = (grp.shell_mode != 'AVOID')
            sub.prop_enum(grp, "shell_op", 'UNION', text="", icon='ADD')
            sub.prop_enum(grp, "shell_op", 'SUBTRACTION', text="", icon='REMOVE')
            row = layout.row(align=True)
            row.prop(grp, "shell_mode", expand=True)

        if bt != 'LINEAR':
            col = layout.column(align=True)
            col.separator()

            if is_shell:
                inward = (grp.shell_op == 'SUBTRACTION')
                start_flip_on = (not inward) or (bt == 'ROUND')
                end_flip_on = inward or (bt == 'ROUND')

                row = col.row(align=True)
                row.prop(grp, "shell_blend_top", text="Start")
                sub = row.row()
                sub.enabled = start_flip_on
                sub.prop(grp, "flip_blend", text="", icon='UV_SYNC_SELECT', toggle=True)

                row = col.row(align=True)
                row.prop(grp, "shell_blend_bottom", text="End")
                sub = row.row()
                sub.enabled = end_flip_on
                sub.prop(grp, "flip_blend_end", text="", icon='UV_SYNC_SELECT', toggle=True)

                if grp.shell_mode == 'AVOID':
                    col.prop(grp, "blend", text="Avoid Blend")
            else:
                col.prop(grp, "blend", text="Blend Strength")

            if bt in ('CHAMFER', 'ROUND'):
                col.separator()
                if is_shell:
                    col.label(text="Smooth Start")
                    row = col.row(align=True)
                    row.prop(grp, "chamfer_k2", text="K2")
                    row.prop(grp, "chamfer_k3", text="K3")
                    col.label(text="Smooth End")
                    row = col.row(align=True)
                    row.prop(grp, "chamfer_k4", text="K4")
                    row.prop(grp, "chamfer_k5", text="K5")
                else:
                    lbl = "Smooth Chamfer" if bt == 'CHAMFER' else "Smooth Round"
                    col.label(text=lbl)
                    row = col.row(align=True)
                    row.prop(grp, "chamfer_k2", text="K2")
                    row.prop(grp, "chamfer_k3", text="K3")


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
            ('SOLIDIFY', "Solidify", "Add internal shell thickness", 'MOD_SOLIDIFY', 4),
            ('ROUND', "Expand/Shrink", "Expand or shrink the SDF surface", 'MOD_SMOOTH', 5),
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
        layout.operator("sdf_group.modifier_add", text="Solidify", icon='MOD_SOLIDIFY').type = 'SOLIDIFY'
        layout.operator("sdf_group.modifier_add", text="Expand/Shrink", icon='MOD_SMOOTH').type = 'ROUND'
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
                grid = box_csg.grid_flow(row_major=True, columns=4, even_columns=True, even_rows=True, align=True)
                grid.scale_y = 1.2
                for item in mod.bl_rna.properties["blend_type"].enum_items:
                    grid.prop_enum(mod, "blend_type", item.identifier, text="")
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
            elif mod.type == 'SOLIDIFY':
                col.prop(mod, "thickness")
                col.prop(mod, "mode")
                if mod.mode == 'OPEN':
                    col.prop(mod, "axis")
            elif mod.type == 'ROUND':
                col.prop(mod, "offset")
            elif mod.type == 'ONION':
                col.prop(mod, "thickness")
                col.prop(mod, "layers")
                col.prop(mod, "gap")
            elif mod.type == 'BEVEL':
                col.prop(mod, "radius")
            elif mod.type == 'ARRAY':
                col.prop(mod, "array_type")
                col.prop(mod, "count")
                if mod.array_type == 'LINEAR':
                    col.prop(mod, "linear_offset")
                elif mod.array_type == 'RADIAL':
                    col.prop(mod, "array_radius")
                    col.prop(mod, "rotation_offset")

                box_csg = box.box()
                box_csg.label(text="Array Blending:")
                grid = box_csg.grid_flow(row_major=True, columns=4, even_columns=True, even_rows=True, align=True)
                grid.scale_y = 1.2
                for item in mod.bl_rna.properties["blend_type"].enum_items:
                    grid.prop_enum(mod, "blend_type", item.identifier, text="")
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
