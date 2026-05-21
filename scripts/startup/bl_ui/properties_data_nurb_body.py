# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

from bpy.types import Panel
from bpy.app.translations import contexts as i18n_contexts


class DataButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob and ob.type == 'NURB_BODY'


class DATA_PT_context_nurb_body(DataButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}
    bl_translation_context = i18n_contexts.id_id

    def draw(self, context):
        layout = self.layout
        ob = context.object
        layout.template_ID(ob, "data")


class DATA_PT_nurb_body_shape(DataButtonsPanel, Panel):
    bl_label = "Shape"
    bl_translation_context = i18n_contexts.id_id

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        body = context.object.data
        layout.prop(body, "primitive")
        layout.prop(body, "select_mode", text="Selection", expand=True)
        layout.prop(body, "radius")
        layout.prop(body, "depth")


class DATA_PT_nurb_body_viewport(DataButtonsPanel, Panel):
    bl_label = "Viewport Tessellation"

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        body = context.object.data
        layout.prop(body, "tessellation_deflection", text="Deflection")
        layout.prop(body, "tessellation_angle", text="Angle")
        layout.prop(body, "use_triangulate_mesh")
        layout.prop(body, "use_smooth_shading")


classes = (
    DATA_PT_context_nurb_body,
    DATA_PT_nurb_body_shape,
    DATA_PT_nurb_body_viewport,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
