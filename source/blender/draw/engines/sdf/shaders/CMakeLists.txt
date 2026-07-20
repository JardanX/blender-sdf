# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC_GLSL
  .
  ..

  # For variadic macros
  ../../../../blenlib

  ../../../intern
  ../../../intern/shaders

  ../../../../gpu
  ../../../../gpu/intern
  ../../../../gpu/shaders
  ../../../../gpu/shaders/common
  ../../../../gpu/shaders/infos

  ../../select
  ../../select/shaders
)

set(SRC_GLSL_FRAG
  sdf_blit_frag.glsl
  sdf_fxaa_frag.glsl
  sdf_outline_prepass_frag.glsl
  sdf_analytical_pick_frag.glsl
  sdf_edge_detect_frag.glsl
)

set(SRC_GLSL_COMP
  sdf_trace_comp.glsl
  sdf_aabb_project_comp.glsl
  sdf_tile_cull_comp.glsl
  sdf_cone_march_comp.glsl
  sdf_color_resolve_comp.glsl
  sdf_normal_comp.glsl
  sdf_shade_comp.glsl
  sdf_lp_prune_comp.glsl
  sdf_lp_march_comp.glsl
  sdf_lp_debug_comp.glsl
  sdf_mesh_bake_comp.glsl
  sdf_grid_eval_comp.glsl
  sdf_dc_vertex_color_comp.glsl
)

if(WITH_GPU_SHADER_CPP_COMPILATION)
  with_shader_cpp_compilation_config()
  add_definitions(-DUSE_GPU_SHADER_CREATE_INFO)

  compile_sources_as_cpp(sdf_cpp_shaders_frag "${SRC_GLSL_FRAG}" "GPU_FRAGMENT_SHADER")
  compile_sources_as_cpp(sdf_cpp_shaders_comp "${SRC_GLSL_COMP}" "GPU_COMPUTE_SHADER")
endif()
