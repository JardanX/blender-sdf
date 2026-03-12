# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Panel


class SDFGroupButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf_group', None) is not None


class DATA_PT_context_sdf_group(SDFGroupButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}

    def draw(self, context):
        layout = self.layout
        space = context.space_data
        grp = context.sdf_group

        if grp:
            layout.template_ID(space, "pin_id")


class DATA_PT_sdf_group_operation(SDFGroupButtonsPanel, Panel):
    bl_label = "Group Operation"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        grp = context.sdf_group

        layout.prop(grp, "csg_operation")
        layout.prop(grp, "blend_type")

        sub = layout.row()
        sub.enabled = (grp.blend_type != 'LINEAR')
        sub.prop(grp, "blend")

        if grp.csg_operation == 'SHELL':
            layout.prop(grp, "shell_distance")


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


class DATA_PT_sdf_group_display(SDFGroupButtonsPanel, Panel):
    bl_label = "Display"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        grp = context.sdf_group
        layout.prop(grp, "color")


# -- Registration -------------------------------------------------------------

classes = (
    DATA_PT_context_sdf_group,
    DATA_PT_sdf_group_operation,
    DATA_PT_sdf_group_members,
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
