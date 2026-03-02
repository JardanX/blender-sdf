# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

from bpy.types import Panel
from bpy.app.translations import contexts as i18n_contexts
from bl_ui.space_view3d import (
    VIEW3D_PT_shading_lighting,
    VIEW3D_PT_shading_color,
    VIEW3D_PT_shading_options,
    VIEW3D_PT_shading_cavity,
)
from bl_ui.utils import PresetPanel


class RenderButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    # COMPAT_ENGINES must be defined in each subclass, external engines can add themselves here

    @classmethod
    def poll(cls, context):
        return (context.engine in cls.COMPAT_ENGINES)


class RENDER_PT_context(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_options = {'HIDE_HEADER'}
    bl_label = ""

    @classmethod
    def poll(cls, context):
        return context.scene

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        scene = context.scene
        rd = scene.render

        if rd.has_multiple_engines:
            layout.prop(rd, "engine", text="Render Engine")


# ---------------------------------------------------------------------------
# MATHOPS: Viewport shading helpers
# ---------------------------------------------------------------------------

def _any_viewport_solid(context):
    """True if any 3D viewport is in Solid, Wireframe, or Material Preview."""
    for area in context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D' and space.shading.type in ('SOLID', 'WIREFRAME', 'MATERIAL'):
                    return True
    return False


def _any_viewport_rendered(context):
    """True if any 3D viewport is in Rendered mode."""
    for area in context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D' and space.shading.type == 'RENDERED':
                    return True
    return False


# ---------------------------------------------------------------------------
# MATHOPS: Wrapper panels for organized Render Properties
# Order: Workbench (10) → Path Tracer (20) → SDF Ray Marcher (30)
#        → Simplify (80) → Color Management (90)
# ---------------------------------------------------------------------------

class RENDER_PT_proximity_workbench(Panel):
    """Workbench viewport settings (Solid / Material Preview)"""
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "Workbench"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 10

    @classmethod
    def poll(cls, context):
        return context.engine == 'BLENDER_PROXIMITY' and _any_viewport_solid(context)

    def draw(self, context):
        pass


class RENDER_PT_proximity_cycles(Panel):
    """Cycles path tracer settings (Rendered viewport / F12)"""
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "Path Tracer"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 20

    @classmethod
    def poll(cls, context):
        return context.engine == 'BLENDER_PROXIMITY' and _any_viewport_rendered(context)

    def draw(self, context):
        pass


class RENDER_PT_proximity_raymarcher(Panel):
    """SDF Ray Marcher settings (MathOPS addon)"""
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "SDF Ray Marcher"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 30

    @classmethod
    def poll(cls, context):
        return context.engine == 'BLENDER_PROXIMITY'

    def draw(self, context):
        layout = self.layout
        layout.label(text="MathOPS ray marcher settings will appear here.")


# ---------------------------------------------------------------------------
# Workbench settings — children of Workbench wrapper
# ---------------------------------------------------------------------------

class RENDER_PT_opengl_sampling(RenderButtonsPanel, Panel):
    bl_label = "Sampling"
    bl_parent_id = "RENDER_PT_proximity_workbench"
    COMPAT_ENGINES = {'BLENDER_PROXIMITY'}

    @classmethod
    def poll(cls, context):
        return (context.engine in cls.COMPAT_ENGINES)

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        scene = context.scene
        props = scene.display

        col = layout.column()
        col.prop(props, "render_aa", text="Render")
        col.prop(props, "viewport_aa", text="Viewport")


class RENDER_PT_opengl_film(RenderButtonsPanel, Panel):
    bl_label = "Film"
    bl_options = {'DEFAULT_CLOSED'}
    bl_parent_id = "RENDER_PT_proximity_workbench"
    COMPAT_ENGINES = {'BLENDER_PROXIMITY'}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        rd = context.scene.render
        layout.prop(rd, "film_transparent", text="Transparent")


class RENDER_PT_opengl_lighting(RenderButtonsPanel, Panel):
    bl_label = "Lighting"
    bl_parent_id = "RENDER_PT_proximity_workbench"
    COMPAT_ENGINES = {'BLENDER_PROXIMITY'}

    @classmethod
    def poll(cls, context):
        return (context.engine in cls.COMPAT_ENGINES)

    def draw(self, context):
        VIEW3D_PT_shading_lighting.draw(self, context)


class RENDER_PT_opengl_color(RenderButtonsPanel, Panel):
    bl_label = "Object Color"
    bl_parent_id = "RENDER_PT_proximity_workbench"
    COMPAT_ENGINES = {'BLENDER_PROXIMITY'}

    @classmethod
    def poll(cls, context):
        return (context.engine in cls.COMPAT_ENGINES)

    def draw(self, context):
        VIEW3D_PT_shading_color._draw_color_type(self, context)


class RENDER_PT_opengl_options(RenderButtonsPanel, Panel):
    bl_label = "Options"
    bl_parent_id = "RENDER_PT_proximity_workbench"
    COMPAT_ENGINES = {'BLENDER_PROXIMITY'}

    @classmethod
    def poll(cls, context):
        return (context.engine in cls.COMPAT_ENGINES)

    def draw(self, context):
        VIEW3D_PT_shading_options.draw(self, context)

        # Cavity properties.
        VIEW3D_PT_shading_cavity.draw_header(self, context)
        VIEW3D_PT_shading_cavity.draw(self, context)


# ---------------------------------------------------------------------------
# Simplify (standalone, before Color Management)
# ---------------------------------------------------------------------------

class RENDER_PT_simplify(RenderButtonsPanel, Panel):
    bl_label = "Simplify"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 80
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    def draw_header(self, context):
        rd = context.scene.render
        self.layout.prop(rd, "use_simplify", text="")

    def draw(self, context):
        pass


class RENDER_PT_simplify_viewport(RenderButtonsPanel, Panel):
    bl_label = "Viewport"
    bl_parent_id = "RENDER_PT_simplify"
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        rd = context.scene.render

        layout.active = rd.use_simplify

        flow = layout.grid_flow(row_major=True, columns=0, even_columns=False, even_rows=False, align=True)

        col = flow.column()
        col.prop(rd, "simplify_subdivision", text="Max Subdivision")

        col = flow.column()
        col.prop(rd, "simplify_child_particles", text="Max Child Particles")

        col = flow.column()
        col.prop(rd, "simplify_volumes", text="Volume Resolution")

        col = flow.column()
        col.prop(rd, "use_simplify_normals", text="Normals")


class RENDER_PT_simplify_render(RenderButtonsPanel, Panel):
    bl_label = "Render"
    bl_parent_id = "RENDER_PT_simplify"
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        rd = context.scene.render

        layout.active = rd.use_simplify

        flow = layout.grid_flow(row_major=True, columns=0, even_columns=False, even_rows=False, align=True)

        col = flow.column()
        col.prop(rd, "simplify_subdivision_render", text="Max Subdivision")

        col = flow.column()
        col.prop(rd, "simplify_child_particles_render", text="Max Child Particles")


# ---------------------------------------------------------------------------
# Color Management (always visible, standalone)
# ---------------------------------------------------------------------------

class RENDER_PT_color_management(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "Color Management"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 90

    @classmethod
    def poll(cls, context):
        return context.scene is not None

    def draw(self, context):

        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        scene = context.scene
        view = scene.view_settings

        flow = layout.grid_flow(row_major=True, columns=0, even_columns=False, even_rows=False, align=True)

        col = flow.column()
        col.prop(scene.display_settings, "display_device")

        col.separator()

        col.prop(view, "view_transform")
        col.prop(view, "look")

        if view.is_hdr and not context.window.support_hdr_color:
            row = col.split(factor=0.4)
            row.label()
            row.label(text="HDR display not supported", icon="INFO")

        col = flow.column()
        col.prop(view, "exposure")
        col.prop(view, "gamma")


class RENDER_PT_color_management_working_space(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "Working Space"
    bl_parent_id = "RENDER_PT_color_management"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return context.scene is not None

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        scene = context.scene
        blend_colorspace = context.blend_data.colorspace

        flow = layout.grid_flow(row_major=True, columns=0, even_columns=False, even_rows=False, align=True)

        col = flow.column()

        split = col.split(factor=0.4)
        row = split.row()
        row.label(text="File")
        row.alignment = 'RIGHT'
        split.operator_menu_enum(
            "wm.set_working_color_space",
            "working_space",
            text=blend_colorspace.working_space,
            text_ctxt=i18n_contexts.default,
        )

        col.prop(scene.sequencer_colorspace_settings, "name", text="Sequencer")


class RENDER_PT_color_management_advanced(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "Advanced"
    bl_parent_id = "RENDER_PT_color_management"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return context.scene is not None

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        scene = context.scene

        col = layout.column()
        col.active = scene.view_settings.support_emulation
        col.prop(scene.display_settings, "emulation")


class RENDER_PT_color_management_curves(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "Curves"
    bl_parent_id = "RENDER_PT_color_management"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return context.scene is not None

    def draw_header(self, context):

        scene = context.scene
        view = scene.view_settings

        self.layout.prop(view, "use_curve_mapping", text="")

    def draw(self, context):
        layout = self.layout

        scene = context.scene
        view = scene.view_settings

        layout.use_property_split = False
        layout.use_property_decorate = False  # No animation.

        layout.active = view.use_curve_mapping

        layout.template_curve_mapping(view, "curve_mapping", type='COLOR', levels=True)


class RENDER_PT_color_management_white_balance_presets(PresetPanel, Panel):
    bl_label = "White Balance Presets"
    preset_subdir = "color_management/white_balance"
    preset_operator = "script.execute_preset"
    preset_add_operator = "render.color_management_white_balance_preset_add"


class RENDER_PT_color_management_white_balance(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "White Balance"
    bl_parent_id = "RENDER_PT_color_management"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return context.scene is not None

    def draw_header(self, context):
        scene = context.scene
        view = scene.view_settings

        self.layout.prop(view, "use_white_balance", text="")

    def draw_header_preset(self, context):
        layout = self.layout

        RENDER_PT_color_management_white_balance_presets.draw_panel_header(layout)

        eye = layout.operator("ui.eyedropper_color", text="", icon='EYEDROPPER')
        eye.prop_data_path = "scene.view_settings.white_balance_whitepoint"

    def draw(self, context):
        layout = self.layout

        scene = context.scene
        view = scene.view_settings

        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        layout.active = view.use_white_balance

        col = layout.column()
        col.prop(view, "white_balance_temperature")
        col.prop(view, "white_balance_tint")


def draw_curves_settings(self, context):
    layout = self.layout
    scene = context.scene
    rd = scene.render

    layout.use_property_split = True
    layout.use_property_decorate = False  # No animation.

    layout.prop(rd, "hair_type", text="Shape", expand=True)
    layout.prop(rd, "hair_subdiv")


class CompositorPerformanceButtonsPanel:
    bl_label = "Compositor"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        rd = scene.render

        layout.use_property_split = True
        layout.use_property_decorate = False

        col = layout.column()
        row = col.row()
        row.prop(rd, "compositor_device", text="Device", expand=True)
        if rd.compositor_device == 'GPU':
            col.prop(rd, "compositor_precision", text="Precision")


class CompositorDenoisePerformanceButtonsPanel:
    bl_label = "Denoise Nodes"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        rd = scene.render

        layout.use_property_split = True
        layout.use_property_decorate = False

        col = layout.column()
        row = col.row()
        row.prop(rd, "compositor_denoise_device", text="Denoising Device", expand=True)
        col.prop(rd, "compositor_denoise_preview_quality", text="Preview Quality")
        col.prop(rd, "compositor_denoise_final_quality", text="Final Quality")


class RENDER_PT_hydra_debug(RenderButtonsPanel, Panel):
    bl_label = "Hydra Debug"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 200
    COMPAT_ENGINES = {'HYDRA_STORM'}

    @classmethod
    def poll(cls, context):
        prefs = context.preferences
        return (context.engine in cls.COMPAT_ENGINES) and prefs.view.show_developer_ui

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        hydra = context.scene.hydra
        layout.prop(hydra, "export_method")


classes = (
    RENDER_PT_context,
    RENDER_PT_proximity_workbench,
    RENDER_PT_proximity_cycles,
    RENDER_PT_proximity_raymarcher,
    RENDER_PT_opengl_sampling,
    RENDER_PT_opengl_lighting,
    RENDER_PT_opengl_color,
    RENDER_PT_opengl_options,
    RENDER_PT_opengl_film,
    RENDER_PT_simplify,
    RENDER_PT_simplify_viewport,
    RENDER_PT_simplify_render,
    RENDER_PT_hydra_debug,
    RENDER_PT_color_management,
    RENDER_PT_color_management_curves,
    RENDER_PT_color_management_white_balance_presets,
    RENDER_PT_color_management_white_balance,
    RENDER_PT_color_management_working_space,
    RENDER_PT_color_management_advanced,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
