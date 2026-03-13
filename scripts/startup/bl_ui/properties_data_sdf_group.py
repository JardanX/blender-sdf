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
        # Only show group panels when directly viewing a group (e.g. pinned),
        # not when an SDF merely references a group via back-pointer.
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

        # CSG operation — full-width icon-only buttons
        layout.label(text="CSG Operation")
        row = layout.row(align=True)
        row.scale_y = 1.4
        row.prop(grp, "csg_operation", expand=True, icon_only=True)

        layout.separator()

        # Blend type — full-width icon-only buttons
        layout.label(text="Blend Type")
        row = layout.row(align=True)
        row.scale_y = 1.4
        row.prop(grp, "blend_type", expand=True, icon_only=True)

        # Blend amount — only show when not linear
        if grp.blend_type != 'LINEAR':
            layout.label(text="Blend")
            layout.prop(grp, "blend", text="")

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


class DATA_PT_sdf_group_display(SDFGroupButtonsPanel, Panel):
    bl_label = "Global Tint"

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
