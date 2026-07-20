# SPDX-FileCopyrightText: 2016 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  PUBLIC .
  intern

  ../editors/include
  ../editors/space_view3d
  ../gpu/intern
  ../makesrna
  ../render/intern
  ../compositor
  ../compositor/algorithms
  ../compositor/cached_resources
  ../compositor/derived_resources

  ../../../intern/opensubdiv
)

set(INC_SYS
)

set(SRC
  intern/attribute_convert.cc
  intern/draw_cache.cc
  intern/draw_lut.cc
  intern/draw_cache_extract_mesh.cc
  intern/draw_cache_extract_mesh_render_data.cc
  intern/mesh_extractors/extract_mesh.cc
  intern/mesh_extractors/extract_mesh_ibo_edituv.cc
  intern/mesh_extractors/extract_mesh_ibo_fdots.cc
  intern/mesh_extractors/extract_mesh_ibo_lines.cc
  intern/mesh_extractors/extract_mesh_ibo_lines_adjacency.cc
  intern/mesh_extractors/extract_mesh_ibo_lines_paint_mask.cc
  intern/mesh_extractors/extract_mesh_ibo_points.cc
  intern/mesh_extractors/extract_mesh_ibo_tris.cc
  intern/mesh_extractors/extract_mesh_vbo_attributes.cc
  intern/mesh_extractors/extract_mesh_vbo_edge_fac.cc
  intern/mesh_extractors/extract_mesh_vbo_edit_data.cc
  intern/mesh_extractors/extract_mesh_vbo_edituv_data.cc
  intern/mesh_extractors/extract_mesh_vbo_edituv_stretch_angle.cc
  intern/mesh_extractors/extract_mesh_vbo_edituv_stretch_area.cc
  intern/mesh_extractors/extract_mesh_vbo_fdots_edituv_data.cc
  intern/mesh_extractors/extract_mesh_vbo_fdots_nor.cc
  intern/mesh_extractors/extract_mesh_vbo_fdots_pos.cc
  intern/mesh_extractors/extract_mesh_vbo_fdots_uv.cc
  intern/mesh_extractors/extract_mesh_vbo_lnor.cc
  intern/mesh_extractors/extract_mesh_vbo_mesh_analysis.cc
  intern/mesh_extractors/extract_mesh_vbo_orco.cc
  intern/mesh_extractors/extract_mesh_vbo_paint_overlay_flag.cc
  intern/mesh_extractors/extract_mesh_vbo_pos.cc
  intern/mesh_extractors/extract_mesh_vbo_sculpt_data.cc
  intern/mesh_extractors/extract_mesh_vbo_select_idx.cc
  intern/mesh_extractors/extract_mesh_vbo_skin_roots.cc
  intern/mesh_extractors/extract_mesh_vbo_tan.cc
  intern/mesh_extractors/extract_mesh_vbo_uv.cc
  intern/mesh_extractors/extract_mesh_vbo_vnor.cc
  intern/mesh_extractors/extract_mesh_vbo_weights.cc
  intern/draw_attributes.cc
  intern/draw_cache_impl_curve.cc
  intern/draw_cache_impl_curves.cc
  # MATHOPS: Kept — batch cache infrastructure for GP data (bf_editor_grease_pencil restored)
  intern/draw_cache_impl_grease_pencil.cc
  intern/draw_cache_impl_lattice.cc
  intern/draw_cache_impl_mesh.cc
  intern/draw_cache_impl_particles.cc
  intern/draw_cache_impl_pointcloud.cc
  intern/draw_cache_impl_subdivision.cc
  intern/draw_cache_impl_volume.cc
  intern/draw_color_management.cc
  intern/draw_command.cc
  intern/draw_context.cc
  intern/draw_curves.cc
  intern/draw_debug.cc
  intern/draw_fluid.cc
  intern/draw_gpu_context.cc
  intern/draw_hair.cc
  intern/draw_manager.cc
  intern/draw_manager_text.cc
  intern/draw_pbvh.cc
  intern/draw_pointcloud.cc
  intern/draw_resource.cc
  intern/draw_sculpt.cc
  intern/draw_select_buffer.cc
  intern/draw_shader.cc
  intern/draw_view.cc
  intern/draw_view_c.cc
  intern/draw_view_data.cc
  intern/draw_volume.cc
  engines/compositor/compositor_engine.cc
  engines/image/image_drawing_mode.cc
  engines/image/image_engine.cc
  engines/image/image_shader.cc
  engines/workbench/workbench_effect_antialiasing.cc
  engines/workbench/workbench_effect_cavity.cc
  engines/workbench/workbench_effect_dof.cc
  engines/workbench/workbench_effect_outline.cc
  engines/workbench/workbench_engine.cc
  engines/workbench/workbench_materials.cc
  engines/workbench/workbench_mesh_passes.cc
  engines/workbench/workbench_resources.cc
  engines/workbench/workbench_shader_cache.cc
  engines/workbench/workbench_shadow.cc
  engines/workbench/workbench_state.cc
  engines/workbench/workbench_volume.cc
  engines/sdf/sdf_engine.cc
  engines/sdf/sdf_lp_engine.cc
  engines/sdf/sdf_bvh.cc
  engines/external/external_engine.cc
  # gpencil draw engine sources removed (not needed for SDF)
  # but shader files kept for overlay system dependencies
  # engines/gpencil/gpencil_antialiasing.cc
  # engines/gpencil/gpencil_cache_utils.cc
  # engines/gpencil/gpencil_draw_data.cc
  # engines/gpencil/gpencil_engine_c.cc
  # engines/gpencil/gpencil_render.cc
  # engines/gpencil/gpencil_shader_fx.cc
  engines/select/select_engine.cc
  engines/select/select_instance.cc
  engines/overlay/overlay_armature.cc
  engines/overlay/overlay_engine.cc
  engines/overlay/overlay_instance.cc
  engines/overlay/overlay_mode_transfer.cc
  engines/overlay/overlay_shader.cc
  engines/overlay/overlay_shape.cc

  DRW_engine.hh
  DRW_pbvh.hh
  DRW_select_buffer.hh
  intern/DRW_gpu_wrapper.hh
  intern/DRW_render.hh
  intern/attribute_convert.hh
  intern/draw_attributes.hh
  intern/draw_cache.hh
  intern/draw_cache_extract.hh
  intern/draw_lut.hh
  intern/draw_cache_impl.hh
  intern/draw_cache_inline.hh
  intern/draw_color_management.hh
  intern/draw_command.hh
  intern/draw_common.hh
  intern/draw_common_c.hh
  intern/draw_context_private.hh
  intern/draw_curves_private.hh
  intern/draw_debug.hh
  intern/draw_hair_private.hh
  intern/draw_handle.hh
  intern/draw_manager.hh
  intern/draw_manager_text.hh
  intern/draw_pass.hh
  intern/draw_resource.hh
  intern/draw_sculpt.hh
  intern/draw_shader.hh
  intern/draw_shader_shared.hh
  intern/draw_state.hh
  intern/draw_subdiv_defines.hh
  intern/draw_subdiv_shader_shared.hh
  intern/draw_subdivision.hh
  intern/draw_view.hh
  intern/draw_view_c.hh
  intern/draw_view_data.hh
  intern/mesh_extractors/extract_mesh.hh
  engines/compositor/compositor_engine.h
  engines/external/external_engine.h
  # engines/gpencil/gpencil_engine.hh
  # engines/gpencil/gpencil_engine_private.hh
  # engines/gpencil/gpencil_shader.hh
  engines/image/image_batches.hh
  engines/image/image_buffer_cache.hh
  engines/image/image_drawing_mode.hh
  engines/image/image_engine.h
  engines/image/image_enums.hh
  engines/image/image_instance.hh
  engines/image/image_partial_updater.hh
  engines/image/image_private.hh
  engines/image/image_shader.hh
  engines/image/image_shader_params.hh
  engines/image/image_space.hh
  engines/image/image_space_image.hh
  engines/image/image_space_node.hh
  engines/image/image_state.hh
  engines/image/image_texture_info.hh
  engines/image/image_usage.hh
  engines/overlay/overlay_antialiasing.hh
  engines/overlay/overlay_armature.hh
  engines/overlay/overlay_attribute_text.hh
  engines/overlay/overlay_attribute_viewer.hh
  engines/overlay/overlay_axes.hh
  engines/overlay/overlay_background.hh
  engines/overlay/overlay_base.hh
  engines/overlay/overlay_bounds.hh
  engines/overlay/overlay_camera.hh
  engines/overlay/overlay_cursor.hh
  engines/overlay/overlay_curve.hh
  engines/overlay/overlay_empty.hh
  engines/overlay/overlay_engine.h
  engines/overlay/overlay_facing.hh
  engines/overlay/overlay_fade.hh
  engines/overlay/overlay_fluid.hh
  engines/overlay/overlay_force_field.hh
  # MATHOPS: Removed — Grease Pencil overlay
  # engines/overlay/overlay_grease_pencil.hh
  engines/overlay/overlay_grid.hh
  engines/overlay/overlay_image.hh
  engines/overlay/overlay_instance.hh
  engines/overlay/overlay_lattice.hh
  engines/overlay/overlay_light.hh
  engines/overlay/overlay_lightprobe.hh
  engines/overlay/overlay_mesh.hh
  engines/overlay/overlay_mode_transfer.hh
  engines/overlay/overlay_motion_path.hh
  engines/overlay/overlay_name.hh
  engines/overlay/overlay_origin.hh
  engines/overlay/overlay_outline.hh
  engines/overlay/overlay_paint.hh
  engines/overlay/overlay_particle.hh
  engines/overlay/overlay_pointcloud.hh
  engines/overlay/overlay_prepass.hh
  engines/overlay/overlay_private.hh
  engines/overlay/overlay_relation.hh
  engines/overlay/overlay_sculpt.hh
  engines/overlay/overlay_speaker.hh
  engines/overlay/overlay_text.hh
  engines/overlay/overlay_wireframe.hh
  engines/overlay/overlay_xray_fade.hh
  engines/select/select_defines.hh
  engines/select/select_engine.hh
  engines/select/select_instance.hh
  engines/sdf/sdf_engine.h
  engines/sdf/sdf_engine_api.hh
  engines/sdf/sdf_engine_internal.hh
  engines/sdf/sdf_bvh.hh
  engines/sdf/sdf_meshing.hh
  engines/sdf/sdf_private.hh
  engines/sdf/sdf_shader_shared.hh
  engines/workbench/workbench_defines.hh
  engines/workbench/workbench_engine.h
  engines/workbench/workbench_enums.hh
  engines/workbench/workbench_private.hh
  engines/workbench/workbench_shader_shared.hh
)

set(LIB
  PRIVATE bf::blenfont
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::blenloader
  PRIVATE bf::blentranslation
  PRIVATE bf::bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::functions
  PRIVATE bf::gpu
  PRIVATE bf::imbuf
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  bf_compositor
  PRIVATE bf::nodes
  PRIVATE bf::render
  PRIVATE bf::windowmanager
  PRIVATE bf::intern::atomic
  PRIVATE bf::dependencies::optional::opensubdiv
)

set(GLSL_SRC
  intern/shaders/draw_curves_infos.hh
  intern/shaders/draw_debug_infos.hh
  intern/shaders/draw_object_infos_infos.hh
  intern/shaders/draw_view_infos.hh
  intern/shaders/subdiv_infos.hh
  intern/shaders/subdiv_patch_eval_infos.hh

  # MATHOPS: Removed — EEVEE shader infos

  engines/image/shaders/infos/engine_image_infos.hh

  engines/gpencil/shaders/infos/gpencil_infos.hh
  engines/gpencil/shaders/infos/gpencil_vfx_infos.hh

  engines/overlay/shaders/infos/overlay_antialiasing_infos.hh
  engines/overlay/shaders/infos/overlay_armature_infos.hh
  engines/overlay/shaders/infos/overlay_background_infos.hh
  engines/overlay/shaders/infos/overlay_common_infos.hh
  engines/overlay/shaders/infos/overlay_edit_mode_infos.hh
  engines/overlay/shaders/infos/overlay_extra_infos.hh
  engines/overlay/shaders/infos/overlay_facing_infos.hh
  engines/overlay/shaders/infos/overlay_fullscreen_infos.hh
  engines/overlay/shaders/infos/overlay_grid_infos.hh
  engines/overlay/shaders/infos/overlay_outline_infos.hh
  engines/overlay/shaders/infos/overlay_paint_infos.hh
  engines/overlay/shaders/infos/overlay_sculpt_curves_infos.hh
  engines/overlay/shaders/infos/overlay_sculpt_infos.hh
  engines/overlay/shaders/infos/overlay_viewer_attribute_infos.hh
  engines/overlay/shaders/infos/overlay_volume_infos.hh

  engines/overlay/shaders/infos/overlay_wireframe_infos.hh

  engines/sdf/shaders/infos/sdf_shader_infos.hh

  engines/select/shaders/infos/select_id_infos.hh

  engines/workbench/shaders/infos/workbench_depth_infos.hh
  engines/workbench/shaders/infos/workbench_effect_antialiasing_infos.hh
  engines/workbench/shaders/infos/workbench_effect_dof_infos.hh
  engines/workbench/shaders/infos/workbench_effect_outline_infos.hh
  engines/workbench/shaders/infos/workbench_prepass_infos.hh
  engines/workbench/shaders/infos/workbench_shadow_infos.hh
  engines/workbench/shaders/infos/workbench_transparent_resolve_infos.hh

  # MATHOPS: Removed — EEVEE shaders


  engines/workbench/shaders/workbench_cavity.bsl.hh
  engines/workbench/shaders/workbench_common.bsl.hh
  engines/workbench/shaders/workbench_common_lib.glsl
  engines/workbench/shaders/workbench_composite.bsl.hh
  engines/workbench/shaders/workbench_curvature.bsl.hh
  engines/workbench/shaders/workbench_effect_dof_prepare_frag.glsl
  engines/workbench/shaders/workbench_effect_dof_downsample_frag.glsl
  engines/workbench/shaders/workbench_effect_dof_blur1_frag.glsl
  engines/workbench/shaders/workbench_effect_dof_blur2_frag.glsl
  engines/workbench/shaders/workbench_effect_dof_resolve_frag.glsl
  engines/workbench/shaders/workbench_effect_dof_lib.glsl
  engines/workbench/shaders/workbench_effect_outline_frag.glsl
  engines/workbench/shaders/workbench_effect_smaa_frag.glsl
  engines/workbench/shaders/workbench_effect_smaa_vert.glsl
  engines/workbench/shaders/workbench_effect_taa_frag.glsl
  engines/workbench/shaders/workbench_fullscreen_vert.glsl
  engines/workbench/shaders/workbench_image_lib.glsl
  engines/workbench/shaders/workbench_image.bsl.hh
  engines/workbench/shaders/workbench_matcap_lib.glsl
  engines/workbench/shaders/workbench_matcap.bsl.hh
  engines/workbench/shaders/workbench_material.bsl.hh
  engines/workbench/shaders/workbench_merge_depth_frag.glsl
  engines/workbench/shaders/workbench_overlay_depth_frag.glsl
  engines/workbench/shaders/workbench_prepass.bsl.hh
  engines/workbench/shaders/workbench_shadow_caps_vert.glsl
  engines/workbench/shaders/workbench_shadow_debug_frag.glsl
  engines/workbench/shaders/workbench_shadow_frag.glsl
  engines/workbench/shaders/workbench_shadow_lib.glsl
  engines/workbench/shaders/workbench_shadow_vert.glsl
  engines/workbench/shaders/workbench_shadow_visibility_comp.glsl
  engines/workbench/shaders/workbench_transparent_accum_frag.glsl
  engines/workbench/shaders/workbench_transparent_resolve_frag.glsl
  engines/workbench/shaders/workbench_volume.bsl.hh
  engines/workbench/shaders/workbench_world_light_lib.glsl
  engines/workbench/shaders/workbench_world_light.bsl.hh

  engines/workbench/workbench_defines.hh
  engines/workbench/workbench_shader_shared.hh

  engines/sdf/shaders/sdf_trace_comp.glsl
  engines/sdf/shaders/sdf_aabb_project_comp.glsl
  engines/sdf/shaders/sdf_tile_cull_comp.glsl
  engines/sdf/shaders/sdf_cone_march_comp.glsl
  engines/sdf/shaders/sdf_color_resolve_comp.glsl
  engines/sdf/shaders/sdf_normal_comp.glsl
  engines/sdf/shaders/sdf_shade_comp.glsl
  engines/sdf/shaders/sdf_lp_prune_comp.glsl
  engines/sdf/shaders/sdf_lp_march_comp.glsl
  engines/sdf/shaders/sdf_lp_resolve_comp.glsl
  engines/sdf/shaders/sdf_lp_common.glsl
  engines/sdf/shaders/sdf_lp_trace_lib.glsl
  engines/sdf/shaders/sdf_blit_frag.glsl
  engines/sdf/shaders/sdf_fxaa_frag.glsl
  engines/sdf/shaders/sdf_fxaa_lib.glsl
  engines/sdf/shaders/sdf_grid_eval_comp.glsl
  engines/sdf/shaders/sdf_dc_contour_comp.glsl
  engines/sdf/shaders/sdf_dc_triangulate_comp.glsl
  engines/sdf/shaders/sdf_dc_vertex_color_comp.glsl
  engines/sdf/shaders/sdf_outline_prepass_frag.glsl
  engines/sdf/shaders/sdf_analytical_pick_frag.glsl
  engines/sdf/shaders/sdf_edge_detect_frag.glsl

  engines/sdf/shaders/sdf_lib.glsl
  engines/sdf/shaders/sdf_mesh_lib.glsl
  engines/sdf/shaders/sdf_grad_lib.glsl
  engines/sdf/sdf_shader_shared.hh

  intern/shaders/osd_patch_basis.glsl
  intern/shaders/subdiv_custom_data_interp_comp.glsl
  intern/shaders/subdiv_ibo_lines_comp.glsl
  intern/shaders/subdiv_ibo_tris_comp.glsl
  intern/shaders/subdiv_lib.glsl
  intern/shaders/subdiv_normals_accumulate_comp.glsl
  intern/shaders/subdiv_patch_evaluation_comp.glsl
  intern/shaders/subdiv_vbo_edge_fac_comp.glsl
  intern/shaders/subdiv_vbo_edituv_strech_angle_comp.glsl
  intern/shaders/subdiv_vbo_edituv_strech_area_comp.glsl
  intern/shaders/subdiv_vbo_lnor_comp.glsl
  intern/shaders/subdiv_vbo_paint_overlay_flag_comp.glsl
  intern/shaders/subdiv_vbo_sculpt_data_comp.glsl
  intern/shaders/draw_aabb_lib.glsl
  intern/shaders/draw_colormanagement_lib.glsl
  intern/shaders/draw_debug_draw_lib.glsl
  intern/shaders/draw_debug_shape_lib.glsl
  intern/shaders/draw_fxaa_lib.glsl
  intern/shaders/draw_curves_interpolation_comp.glsl
  intern/shaders/draw_curves_length_intercept_comp.glsl
  intern/shaders/draw_curves_lib.glsl
  intern/shaders/draw_curves_topology_comp.glsl
  intern/shaders/draw_shape_lib.glsl
  intern/shaders/draw_command_generate_comp.glsl
  intern/shaders/draw_debug_draw_display_frag.glsl
  intern/shaders/draw_debug_draw_display_vert.glsl
  intern/shaders/draw_grease_pencil_lib.glsl
  intern/shaders/draw_intersect_lib.glsl
  intern/shaders/draw_math_geom_lib.glsl
  intern/shaders/draw_model_lib.glsl
  intern/shaders/draw_object_infos_lib.glsl
  intern/shaders/draw_pointcloud_lib.glsl
  intern/shaders/draw_resource_finalize_comp.glsl
  intern/shaders/draw_view_clipping_lib.glsl
  intern/shaders/draw_view_finalize_comp.glsl
  intern/shaders/draw_view_lib.glsl
  intern/shaders/draw_view_reconstruction_lib.glsl
  intern/shaders/draw_visibility_comp.glsl

  intern/draw_attribute_shader_shared.hh
  intern/draw_command_shared.hh
  intern/draw_curves_defines.hh
  intern/draw_defines.hh
  intern/draw_shader_shared.hh
  intern/draw_subdiv_defines.hh
  intern/draw_subdiv_shader_shared.hh

  # MATHOPS: Removed — Grease Pencil shaders
  engines/gpencil/shaders/gpencil_frag.glsl
  engines/gpencil/shaders/gpencil_vert.glsl
  engines/gpencil/shaders/gpencil_antialiasing_accumulation_frag.glsl
  engines/gpencil/shaders/gpencil_antialiasing_frag.glsl
  engines/gpencil/shaders/gpencil_antialiasing_vert.glsl
  engines/gpencil/shaders/gpencil_common_lib.glsl
  engines/gpencil/shaders/gpencil_fullscreen_vert.glsl
  engines/gpencil/shaders/gpencil_layer_blend_frag.glsl
  engines/gpencil/shaders/gpencil_mask_invert_frag.glsl
  engines/gpencil/shaders/gpencil_depth_merge_frag.glsl
  engines/gpencil/shaders/gpencil_depth_merge_vert.glsl
  engines/gpencil/shaders/gpencil_vfx_frag.glsl

  engines/gpencil/gpencil_defines.hh
  engines/gpencil/gpencil_shader_shared.hh

  engines/select/shaders/select_id_vert.glsl
  engines/select/shaders/select_id_frag.glsl
  engines/select/shaders/select_lib.glsl
  engines/select/shaders/select_debug_frag.glsl

  engines/select/select_defines.hh
  engines/select/select_shader_shared.hh

  engines/overlay/shaders/overlay_antialiasing_frag.glsl
  engines/overlay/shaders/overlay_armature_dof_solid_frag.glsl
  engines/overlay/shaders/overlay_armature_dof_vert.glsl
  engines/overlay/shaders/overlay_armature_envelope_outline_vert.glsl
  engines/overlay/shaders/overlay_armature_envelope_solid_frag.glsl
  engines/overlay/shaders/overlay_armature_envelope_solid_vert.glsl
  engines/overlay/shaders/overlay_armature_shape_outline_vert.glsl
  engines/overlay/shaders/overlay_armature_shape_solid_frag.glsl
  engines/overlay/shaders/overlay_armature_shape_solid_vert.glsl
  engines/overlay/shaders/overlay_armature_shape_wire_vert.glsl
  engines/overlay/shaders/overlay_armature_shape_wire_frag.glsl
  engines/overlay/shaders/overlay_armature_sphere_outline_vert.glsl
  engines/overlay/shaders/overlay_armature_sphere_solid_frag.glsl
  engines/overlay/shaders/overlay_armature_sphere_solid_vert.glsl
  engines/overlay/shaders/overlay_armature_stick_frag.glsl
  engines/overlay/shaders/overlay_armature_stick_vert.glsl
  engines/overlay/shaders/overlay_armature_wire_frag.glsl
  engines/overlay/shaders/overlay_armature_wire_vert.glsl
  engines/overlay/shaders/overlay_background_frag.glsl
  engines/overlay/shaders/overlay_clipbound_vert.glsl
  engines/overlay/shaders/overlay_common_lib.glsl
  engines/overlay/shaders/overlay_depth_only_curves_vert.glsl
  engines/overlay/shaders/overlay_depth_only_frag.glsl
  engines/overlay/shaders/overlay_depth_only_gpencil_frag.glsl
  engines/overlay/shaders/overlay_depth_only_gpencil_vert.glsl
  engines/overlay/shaders/overlay_depth_only_mesh_conservative_vert.glsl
  engines/overlay/shaders/overlay_depth_only_pointcloud_vert.glsl
  engines/overlay/shaders/overlay_depth_only_vert.glsl
  engines/overlay/shaders/overlay_edit_curve_handle_vert.glsl
  engines/overlay/shaders/overlay_edit_curve_normals_vert.glsl
  engines/overlay/shaders/overlay_edit_curve_point_vert.glsl
  engines/overlay/shaders/overlay_edit_curve_wire_vert.glsl
  engines/overlay/shaders/overlay_edit_curves_handle_vert.glsl
  engines/overlay/shaders/overlay_edit_gpencil_canvas_vert.glsl
  engines/overlay/shaders/overlay_edit_lattice_point_vert.glsl
  engines/overlay/shaders/overlay_edit_lattice_wire_vert.glsl
  engines/overlay/shaders/overlay_edit_mesh_analysis_frag.glsl
  engines/overlay/shaders/overlay_edit_mesh_analysis_vert.glsl
  engines/overlay/shaders/overlay_edit_mesh_common_lib.glsl
  engines/overlay/shaders/overlay_edit_mesh_depth_vert.glsl
  engines/overlay/shaders/overlay_edit_mesh_frag.glsl
  engines/overlay/shaders/overlay_edit_mesh_normal_vert.glsl
  engines/overlay/shaders/overlay_edit_mesh_skin_root_vert.glsl
  engines/overlay/shaders/overlay_edit_mesh_vert.glsl
  engines/overlay/shaders/overlay_edit_mesh_lib.glsl
  engines/overlay/shaders/overlay_edit_mesh_edge_vert.glsl
  engines/overlay/shaders/overlay_edit_mesh_facedot_vert.glsl
  engines/overlay/shaders/overlay_edit_particle_point_vert.glsl
  engines/overlay/shaders/overlay_edit_particle_strand_vert.glsl
  engines/overlay/shaders/overlay_edit_pointcloud_vert.glsl
  engines/overlay/shaders/overlay_edit_uv_edges_frag.glsl
  engines/overlay/shaders/overlay_edit_uv_edges_vert.glsl
  engines/overlay/shaders/overlay_edit_uv_face_dots_vert.glsl
  engines/overlay/shaders/overlay_edit_uv_faces_vert.glsl
  engines/overlay/shaders/overlay_edit_uv_image_mask_frag.glsl
  engines/overlay/shaders/overlay_edit_uv_image_vert.glsl
  engines/overlay/shaders/overlay_edit_uv_stretching_vert.glsl
  engines/overlay/shaders/overlay_edit_uv_tiled_image_borders_vert.glsl
  engines/overlay/shaders/overlay_edit_uv_verts_frag.glsl
  engines/overlay/shaders/overlay_edit_uv_verts_vert.glsl
  engines/overlay/shaders/overlay_extra_frag.glsl
  engines/overlay/shaders/overlay_extra_groundline_vert.glsl
  engines/overlay/shaders/overlay_extra_lightprobe_grid_vert.glsl
  engines/overlay/shaders/overlay_extra_loose_point_frag.glsl
  engines/overlay/shaders/overlay_extra_loose_point_vert.glsl
  engines/overlay/shaders/overlay_extra_point_vert.glsl
  engines/overlay/shaders/overlay_sdf_group_point_vert.glsl
  engines/overlay/shaders/overlay_extra_vert.glsl
  engines/overlay/shaders/overlay_extra_wire_frag.glsl
  engines/overlay/shaders/overlay_extra_wire_vert.glsl
  engines/overlay/shaders/overlay_facing_frag.glsl
  engines/overlay/shaders/overlay_facing_vert.glsl
  engines/overlay/shaders/overlay_fullscreen_vert.glsl
  engines/overlay/shaders/overlay_grid_frag.glsl
  engines/overlay/shaders/overlay_grid_vert.glsl
  engines/overlay/shaders/overlay_grid_common_lib.glsl
  engines/overlay/shaders/overlay_image_frag.glsl
  engines/overlay/shaders/overlay_image_vert.glsl
  engines/overlay/shaders/overlay_motion_path_line_frag.glsl
  engines/overlay/shaders/overlay_motion_path_line_vert.glsl
  engines/overlay/shaders/overlay_motion_path_point_vert.glsl
  engines/overlay/shaders/overlay_outline_detect_frag.glsl
  engines/overlay/shaders/overlay_outline_prepass_curves_vert.glsl
  engines/overlay/shaders/overlay_outline_prepass_frag.glsl
  engines/overlay/shaders/overlay_outline_prepass_gpencil_frag.glsl
  engines/overlay/shaders/overlay_outline_prepass_gpencil_vert.glsl
  engines/overlay/shaders/overlay_outline_prepass_pointcloud_vert.glsl
  engines/overlay/shaders/overlay_outline_prepass_vert.glsl
  engines/overlay/shaders/overlay_outline_prepass_wire_vert.glsl
  engines/overlay/shaders/overlay_paint_face_vert.glsl
  engines/overlay/shaders/overlay_paint_point_vert.glsl
  engines/overlay/shaders/overlay_paint_texture_frag.glsl
  engines/overlay/shaders/overlay_paint_texture_vert.glsl
  engines/overlay/shaders/overlay_paint_weight_frag.glsl
  engines/overlay/shaders/overlay_paint_weight_vert.glsl
  engines/overlay/shaders/overlay_paint_wire_vert.glsl
  engines/overlay/shaders/overlay_particle_frag.glsl
  engines/overlay/shaders/overlay_particle_vert.glsl
  engines/overlay/shaders/overlay_particle_hair_vert.glsl
  engines/overlay/shaders/overlay_particle_shape_vert.glsl
  engines/overlay/shaders/overlay_particle_shape_frag.glsl
  engines/overlay/shaders/overlay_point_varying_color_frag.glsl
  engines/overlay/shaders/overlay_point_varying_color_varying_outline_aa_frag.glsl
  engines/overlay/shaders/overlay_sculpt_curves_cage_vert.glsl
  engines/overlay/shaders/overlay_sculpt_curves_selection_frag.glsl
  engines/overlay/shaders/overlay_sculpt_curves_selection_vert.glsl
  engines/overlay/shaders/overlay_sculpt_mask_frag.glsl
  engines/overlay/shaders/overlay_sculpt_mask_vert.glsl

  engines/overlay/shaders/overlay_uniform_color_frag.glsl
  engines/overlay/shaders/overlay_varying_color.glsl
  engines/overlay/shaders/overlay_viewer_attribute_curve_vert.glsl
  engines/overlay/shaders/overlay_viewer_attribute_curves_vert.glsl
  engines/overlay/shaders/overlay_viewer_attribute_frag.glsl
  engines/overlay/shaders/overlay_viewer_attribute_mesh_vert.glsl
  engines/overlay/shaders/overlay_viewer_attribute_pointcloud_vert.glsl
  engines/overlay/shaders/overlay_volume_gridlines_vert.glsl
  engines/overlay/shaders/overlay_volume_velocity_vert.glsl
  engines/overlay/shaders/overlay_wireframe_frag.glsl
  engines/overlay/shaders/overlay_wireframe_vert.glsl
  engines/overlay/shaders/overlay_xray_fade_frag.glsl

  engines/overlay/overlay_shader_shared.hh

  engines/image/shaders/image_engine_color_frag.glsl
  engines/image/shaders/image_engine_color_vert.glsl
  engines/image/shaders/image_engine_depth_frag.glsl
  engines/image/shaders/image_engine_depth_vert.glsl

  # This test doesn't depend on gpu_shader_test, so it's always included.
  intern/shaders/draw_curves_test.glsl
)

# These tests depend on gpu_shader_test, so they're only included when WITH_GPU_BACKEND_TESTS is enabled.
# MATHOPS: Removed — EEVEE test shaders
set(GLSL_SRC_TEST
)

if(WITH_GTESTS)
  if(WITH_GPU_BACKEND_TESTS)
    list(APPEND GLSL_SRC ${GLSL_SRC_TEST})
  endif()
endif()

set(GLSL_INC
  ../gpu
  ../gpu/intern
  ../gpu/shaders
  ../gpu/shaders/infos
  ../gpu/shaders/common

  engines/gpencil
  engines/gpencil/shaders
  engines/gpencil/shaders/infos
  engines/workbench
  engines/workbench/shaders
  engines/workbench/shaders/infos
  # MATHOPS: Removed — EEVEE include dirs
  engines/overlay
  engines/overlay/shaders
  engines/overlay/shaders/infos
  engines/select
  engines/select/shaders
  engines/select/shaders/infos
  engines/image
  engines/image/shaders
  engines/image/shaders/infos
  engines/sdf
  engines/sdf/shaders
  engines/sdf/shaders/infos

  intern
  intern/shaders
)

set(GLSL_C "")
foreach(GLSL_FILE ${GLSL_SRC})
  glsl_to_c(${GLSL_FILE} GLSL_C GLSL_INC)
endforeach()

blender_add_lib(bf_draw_shaders "${GLSL_C}" "" "" "")
blender_set_target_unity_build(bf_draw_shaders 10)

list(APPEND LIB
  PRIVATE bf::animrig
  bf_draw_shaders
)

set(GLSL_SOURCE_CONTENT "")
set(GLSL_METADATA_CONTENT "")
set(GLSL_INFOS_CONTENT "")
foreach(GLSL_FILE ${GLSL_SRC})
  get_filename_component(GLSL_FILE_NAME ${GLSL_FILE} NAME)
  string(REPLACE "." "_" GLSL_FILE_NAME_UNDERSCORES ${GLSL_FILE_NAME})
  string(APPEND GLSL_SOURCE_CONTENT "SHADER_SOURCE\(${GLSL_FILE_NAME_UNDERSCORES}, \"${GLSL_FILE_NAME}\", \"${GLSL_FILE}\"\)\n")
  string(APPEND GLSL_METADATA_CONTENT "#include \"${GLSL_FILE}.hh\"\n")
  string(APPEND GLSL_INFOS_CONTENT "#include \"${GLSL_FILE}.info\"\n")
endforeach()

set(glsl_source_list_file "${CMAKE_CURRENT_BINARY_DIR}/glsl_draw_source_list.h")
file(GENERATE OUTPUT ${glsl_source_list_file} CONTENT "${GLSL_SOURCE_CONTENT}")
list(APPEND SRC ${glsl_source_list_file})
set(glsl_metadata_list_file "${CMAKE_CURRENT_BINARY_DIR}/glsl_draw_metadata_list.hh")
file(GENERATE OUTPUT ${glsl_metadata_list_file} CONTENT "${GLSL_METADATA_CONTENT}")
list(APPEND SRC ${glsl_metadata_list_file})
set(glsl_infos_list_file "${CMAKE_CURRENT_BINARY_DIR}/glsl_draw_infos_list.hh")
file(GENERATE OUTPUT ${glsl_infos_list_file} CONTENT "${GLSL_INFOS_CONTENT}")
list(APPEND SRC ${glsl_infos_list_file})
list(APPEND INC ${CMAKE_CURRENT_BINARY_DIR})

target_include_directories(bf_draw_shaders PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

if(WITH_DRAW_DEBUG)
  list(APPEND SRC
    engines/select/select_debug_engine.cc
  )
  add_definitions(-DWITH_DRAW_DEBUG)
endif()

if(WITH_MOD_FLUID)
  list(APPEND INC
    ../../../intern/mantaflow/extern
  )
  add_definitions(-DWITH_FLUID)
endif()

if(WITH_FREESTYLE)
  add_definitions(-DWITH_FREESTYLE)
endif()

if(WITH_XR_OPENXR)
  add_definitions(-DWITH_XR_OPENXR)
endif()

if(WITH_GTESTS)
  if(WITH_GPU_DRAW_TESTS)
    add_definitions(-DWITH_GPU_DRAW_TESTS)
    if(WITH_OPENGL_BACKEND)
      add_definitions(-DWITH_OPENGL_BACKEND)
    endif()
    if(WITH_METAL_BACKEND)
      add_definitions(-DWITH_METAL_BACKEND)
    endif()
    if(WITH_VULKAN_BACKEND)
      add_definitions(-DWITH_VULKAN_BACKEND)
      list(APPEND LIB
        PRIVATE bf::dependencies::optional::vulkan
      )
    endif()
  endif()
endif()

if(WITH_TBB)
  if(WIN32)
    # TBB includes Windows.h which will define min/max macros
    # that will collide with the STL versions.
    add_definitions(-DNOMINMAX)
  endif()
endif()

# SDF engine uses GL timestamp queries for performance overlay.
if(WITH_OPENGL_BACKEND)
  list(APPEND INC_SYS
    ${Epoxy_INCLUDE_DIRS}
  )
endif()

blender_add_lib(bf_draw "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_library(bf::draw ALIAS bf_draw)


if(WITH_GTESTS)
  if(WITH_GPU_DRAW_TESTS)
    set(TEST_SRC
      tests/draw_curves_test.cc
      tests/draw_pass_test.cc
      # MATHOPS: Removed — eevee_test.cc
      tests/draw_testing.cc

      tests/draw_testing.hh
    )
    set(TEST_INC
      ${INC}
      ../../../intern/ghost
      ../gpu/tests
    )
    set(TEST_LIB
      ${LIB}
    )
    set(TEST_COMMON_SRC
      tests/draw_testing.cc
    )
    if(NOT WITH_TESTS_SINGLE_BINARY)
      list(APPEND TEST_COMMON_SRC
        ../gpu/tests/gpu_testing.cc
        ../gpu/tests/gpu_testing.hh
      )
    else()
      list(APPEND TEST_LIB gpu_tests)
    endif()
    blender_add_test_suite_lib(draw "${TEST_SRC}" "${TEST_INC}" "${INC_SYS}" "${TEST_LIB}" "${TEST_COMMON_SRC}")
  endif()
endif()
