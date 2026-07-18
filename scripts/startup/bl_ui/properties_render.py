# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Panel, Operator
from bpy.props import EnumProperty
from bpy.app.translations import contexts as i18n_contexts
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
# MATHOPS: Wrapper panels for organized Render Properties
# ---------------------------------------------------------------------------

class RENDER_PT_proximity_cycles(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "Path Tracer"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 10

    @classmethod
    def poll(cls, context):
        return context.engine == 'CYCLES'

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        scene = context.scene
        if hasattr(scene, 'cycles'):
            cscene = scene.cycles
            layout.prop(cscene, "device")

            try:
                from cycles import engine as cycles_engine
                if cycles_engine.with_osl():
                    use_cpu = (cscene.device == 'CPU')
                    prefs = context.preferences.addons.get('cycles')
                    dev_type = prefs.preferences.compute_device_type if prefs else 'NONE'
                    use_optix = (dev_type == 'OPTIX' and cscene.device == 'GPU')
                    osl_ok = use_cpu or (use_optix and (
                        cycles_engine.osl_version()[1] >= 13 or
                        cycles_engine.osl_version()[0] > 1))
                    if osl_ok:
                        layout.prop(cscene, "shading_system")
            except (ImportError, AttributeError):
                pass


_SDF_PRESETS = {
    'LOW': {
        'sdf_resolution_scale': 50.0,
        'sdf_adaptive_resolution': True,
        'sdf_frustum_cull': True,
        'sdf_max_steps': 100,
        'sdf_ray_epsilon': 0.001,
        'sdf_over_relaxation': 1.3,
        'sdf_use_cone_trace': True,
        'sdf_cone_aperture': 0.5,
        'sdf_cone_steps': 32,
    },
    'MEDIUM': {
        'sdf_resolution_scale': 100.0,
        'sdf_adaptive_resolution': True,
        'sdf_frustum_cull': True,
        'sdf_max_steps': 512,
        'sdf_ray_epsilon': 0.001,
        'sdf_over_relaxation': 1.3,
        'sdf_use_cone_trace': True,
        'sdf_cone_aperture': 0.5,
        'sdf_cone_steps': 32,
    },
    'HIGH': {
        'sdf_resolution_scale': 100.0,
        'sdf_adaptive_resolution': False,
        'sdf_frustum_cull': True,
        'sdf_max_steps': 1024,
        'sdf_ray_epsilon': 0.001,
        'sdf_over_relaxation': 1.3,
        'sdf_use_cone_trace': True,
        'sdf_cone_aperture': 0.5,
        'sdf_cone_steps': 32,
    },
}


_SDF_UNVERSIONED_DEFAULTS = {
    'sdf_resolution_scale': (0.0, 100.0),
    'sdf_max_steps': (0, 1024),
    'sdf_ray_epsilon': (0.0, 0.001),
    'sdf_over_relaxation': (0.0, 1.3),
    'sdf_cone_aperture': (0.0, 0.5),
    'sdf_cone_steps': (0, 32),
}


def _matches_preset(shading, preset_key):
    vals = _SDF_PRESETS[preset_key]
    for attr, expected in vals.items():
        actual = getattr(shading, attr)
        if attr in _SDF_UNVERSIONED_DEFAULTS:
            zero_val, fallback = _SDF_UNVERSIONED_DEFAULTS[attr]
            if isinstance(zero_val, float):
                if abs(actual - zero_val) < 1e-6:
                    actual = fallback
            elif actual == zero_val:
                actual = fallback
        if isinstance(expected, float):
            if abs(actual - expected) > 1e-4:
                return False
        elif actual != expected:
            return False
    return True


def _current_preset_label(shading):
    for key in ('LOW', 'MEDIUM', 'HIGH'):
        if _matches_preset(shading, key):
            return key.title()
    return "Custom"


class SDF_OT_raymarcher_preset(Operator):
    bl_idname = "sdf.raymarcher_preset"
    bl_label = "SDF Ray Marcher Preset"
    bl_options = {'INTERNAL'}

    preset: EnumProperty(
        items=[
            ('LOW', "Low", "Fast viewport: 50% resolution, adaptive, 100 steps"),
            ('MEDIUM', "Medium", "Balanced: full resolution, adaptive, 512 steps, 0.001 epsilon"),
            ('HIGH', "High", "Maximum quality: full resolution, 1024 steps, 0.001 epsilon"),
        ],
    )

    def execute(self, context):
        for area in context.screen.areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        vals = _SDF_PRESETS[self.preset]
                        for attr, val in vals.items():
                            setattr(space.shading, attr, val)
                        area.tag_redraw()
                        return {'FINISHED'}
        return {'CANCELLED'}


class RENDER_PT_proximity_raymarcher(Panel):
    """SDF Ray Marcher settings (MathOPS addon)"""
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "SDF Ray Marcher"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 20

    @classmethod
    def poll(cls, context):
        return context.engine == 'CYCLES'

    @staticmethod
    def _get_shading(context):
        for area in context.screen.areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        return space.shading
        return None

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        shading = self._get_shading(context)
        if shading is None:
            layout.label(text="No 3D Viewport found.")
            return

        row = layout.row(align=True)
        row.alignment = 'EXPAND'
        row.label(text="Preset:")
        row.operator_menu_enum(
            "sdf.raymarcher_preset",
            "preset",
            text=_current_preset_label(shading),
            icon="DOWNARROW_HLT",
        )

        col = layout.column()
        col.prop(shading, "sdf_resolution_scale", slider=True)
        col.prop(shading, "sdf_adaptive_resolution")
        col.prop(shading, "sdf_frustum_cull")
        col.prop(shading, "sdf_max_steps")
        col.prop(shading, "sdf_ray_epsilon")
        col.prop(shading, "sdf_over_relaxation")
        col.separator()
        col.prop(shading, "sdf_use_cone_trace")
        col.prop(shading, "sdf_cone_aperture")
        col.prop(shading, "sdf_cone_steps")
        col.prop(shading, "sdf_bvh_debug_view")
        col.prop(shading, "sdf_fd_normals")


# ---------------------------------------------------------------------------
# Simplify
# ---------------------------------------------------------------------------

class RENDER_PT_simplify(RenderButtonsPanel, Panel):
    bl_label = "Simplify"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 80
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'CYCLES',
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
        'CYCLES',
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
        'CYCLES',
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
# Color Management (always visible, at the very bottom with separator)
# ---------------------------------------------------------------------------

class RENDER_PT_color_management(Panel):
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "render"
    bl_label = "Color Management"
    bl_order = 200

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


# MATHOPS: Removed — EEVEE render panels
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
    bl_order = 300
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
    SDF_OT_raymarcher_preset,
    RENDER_PT_context,
    RENDER_PT_proximity_cycles,
    RENDER_PT_proximity_raymarcher,
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

def _sdf_apply_high_defaults():
    """Apply High preset to viewports that don't match any preset."""
    for wm in bpy.data.window_managers:
        for window in wm.windows:
            for area in window.screen.areas:
                if area.type == 'VIEW_3D':
                    for space in area.spaces:
                        if space.type == 'VIEW_3D':
                            shading = space.shading
                            if not shading.sdf_frustum_cull:
                                shading.sdf_frustum_cull = True
                            if not any(_matches_preset(shading, k) for k in _SDF_PRESETS):
                                for attr, val in _SDF_PRESETS['HIGH'].items():
                                    setattr(shading, attr, val)


@bpy.app.handlers.persistent
def _sdf_load_post_handler(_):
    _sdf_apply_high_defaults()


bpy.app.handlers.load_post.append(_sdf_load_post_handler)
bpy.app.timers.register(_sdf_apply_high_defaults, first_interval=0.1)


if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
