/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"
#  include "sdf_shader_shared.hh"
#  include "select_shader_shared.hh"
#endif

#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name SDF Trace Compute Shader (Pass 1: sphere tracing -> G-buffer)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_trace_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
DEFINE_VALUE("kTileSize", "8")
DEFINE_VALUE("kMaxTileObjects", "128")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(3, read, SdfAabbNodeGPU, aabb_nodes[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, out_color_img)
IMAGE(1, SFLOAT_32, write, image2D, out_depth_img)
IMAGE(2, SFLOAT_32_32_32_32, write, image2D, gbuf_pos_img)
IMAGE(3, SFLOAT_16_16_16_16, write, image2D, gbuf_color_img)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(int, use_bvh)
PUSH_CONSTANT(int, use_cone_trace)
PUSH_CONSTANT(int, bvh_root)
PUSH_CONSTANT(int, debug_bvh_views)
PUSH_CONSTANT(int, sdf_max_steps)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(float, sdf_over_relaxation)
PUSH_CONSTANT(float, sdf_step_factor)
PUSH_CONSTANT(float3, scene_aabb_min)
PUSH_CONSTANT(float3, scene_aabb_max)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(draw_view)
COMPUTE_SOURCE("sdf_trace_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF AABB Project Pass (pre-projects object AABBs to screen space)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_aabb_project_comp)
LOCAL_GROUP_SIZE(64)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(9, write, int4, screen_aabbs[])
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(draw_view)
COMPUTE_SOURCE("sdf_aabb_project_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Tile Cull Pass (builds per-tile primitive lists)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_tile_cull_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
DEFINE_VALUE("kTileSize", "8")
DEFINE_VALUE("kMaxTileObjects", "128")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(9, read, int4, screen_aabbs[])
STORAGE_BUF(5, write, int, tile_prim_counts[])
STORAGE_BUF(6, write, int, tile_prim_lists[])
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_tile_cull_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Cone March Pre-Pass
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_cone_march_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(4, write, float4, tile_hit_pos[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
STORAGE_BUF(5, read_write, int, tile_prim_counts[])
STORAGE_BUF(6, read, int, tile_prim_lists[])
STORAGE_BUF(7, write, float, tile_far_hint[])
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(float, sdf_cone_aperture)
PUSH_CONSTANT(int, sdf_cone_steps)
PUSH_CONSTANT(float3, scene_aabb_min)
PUSH_CONSTANT(float3, scene_aabb_max)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(draw_view)
COMPUTE_SOURCE("sdf_cone_march_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Tile-Culled Trace
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_trace_tile_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
DEFINE("USE_TILE_CULLING")
DEFINE_VALUE("kTileSize", "8")
DEFINE_VALUE("kMaxTileObjects", "128")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(4, read, float4, tile_hit_pos[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
STORAGE_BUF(5, read, int, tile_prim_counts[])
STORAGE_BUF(6, read, int, tile_prim_lists[])
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, out_color_img)
IMAGE(1, SFLOAT_32, write, image2D, out_depth_img)
IMAGE(2, SFLOAT_32_32_32_32, write, image2D, gbuf_pos_img)
IMAGE(3, SFLOAT_16_16_16_16, write, image2D, gbuf_color_img)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(int, use_bvh)
PUSH_CONSTANT(int, use_cone_trace)
PUSH_CONSTANT(int, bvh_root)
PUSH_CONSTANT(int, debug_bvh_views)
PUSH_CONSTANT(int, sdf_max_steps)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(float, sdf_over_relaxation)
PUSH_CONSTANT(float, sdf_step_factor)
PUSH_CONSTANT(float3, scene_aabb_min)
PUSH_CONSTANT(float3, scene_aabb_max)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(draw_view)
COMPUTE_SOURCE("sdf_trace_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Color Resolve (evaluates color at trace hit positions)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_color_resolve_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
DEFINE_VALUE("kTileSize", "8")
DEFINE_VALUE("kMaxTileObjects", "128")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(5, read, int, tile_prim_counts[])
STORAGE_BUF(6, read, int, tile_prim_lists[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
IMAGE(0, SFLOAT_32_32_32_32, read, image2D, gbuf_pos_img)
IMAGE(1, SFLOAT_16_16_16_16, read_write, image2D, gbuf_color_img)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_color_resolve_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Normal Compute Shader (analytical gradient)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_normal_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
DEFINE_VALUE("kTileSize", "8")
DEFINE_VALUE("kMaxTileObjects", "128")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(5, read, int, tile_prim_counts[])
STORAGE_BUF(6, read, int, tile_prim_lists[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
IMAGE(0, SFLOAT_32_32_32_32, read, image2D, gbuf_pos_img)
IMAGE(1, SFLOAT_16_16_16_16, write, image2D, gbuf_normal_img)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(int, debug_fd_normals)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_normal_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Shade Compute Shader (Pass 3: lighting from G-buffer)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_shade_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
SAMPLER(0, sampler2DArray, matcap_tx)
IMAGE(0, SFLOAT_32_32_32_32, read, image2D, gbuf_pos_img)
IMAGE(1, SFLOAT_16_16_16_16, read, image2D, gbuf_color_img)
IMAGE(2, SFLOAT_16_16_16_16, write, image2D, out_color_img)
IMAGE(3, SFLOAT_32, write, image2D, out_depth_img)
IMAGE(4, SFLOAT_16_16_16_16, read, image2D, gbuf_normal_img)
PUSH_CONSTANT(int, lighting_type)
PUSH_CONSTANT(int, use_specular)
PUSH_CONSTANT(int, use_matcap_flip)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
PUSH_CONSTANT(float4, studio_light0)
PUSH_CONSTANT(float4, studio_light1)
PUSH_CONSTANT(float4, studio_light2)
PUSH_CONSTANT(float4, studio_light3)
PUSH_CONSTANT(float4, studio_color0)
PUSH_CONSTANT(float4, studio_color1)
PUSH_CONSTANT(float4, studio_color2)
PUSH_CONSTANT(float4, studio_color3)
PUSH_CONSTANT(float4, studio_spec0)
PUSH_CONSTANT(float4, studio_spec1)
PUSH_CONSTANT(float4, studio_spec2)
PUSH_CONSTANT(float4, studio_spec3)
PUSH_CONSTANT(float3, studio_ambient)
ADDITIONAL_INFO(draw_view)
COMPUTE_SOURCE("sdf_shade_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Blit Fragment Shader (Blits compute output)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_blit)
DO_STATIC_COMPILATION()
SAMPLER(0, sampler2D, color_tx)
SAMPLER(1, sampler2D, depth_tx)
PUSH_CONSTANT(int, debug_bvh_views)
PUSH_CONSTANT(float3, bg_color)
PUSH_CONSTANT(float2, uv_scale)
FRAGMENT_OUT(0, float4, out_color)
DEPTH_WRITE(DepthWrite::ANY)
ADDITIONAL_INFO(gpu_fullscreen)
FRAGMENT_SOURCE("sdf_blit_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF FXAA Post-Process Fragment Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_fxaa)
DO_STATIC_COMPILATION()
SAMPLER(0, sampler2D, color_tx)
PUSH_CONSTANT(float2, rcpFrame)
FRAGMENT_OUT(0, float4, out_color)
ADDITIONAL_INFO(gpu_fullscreen)
FRAGMENT_SOURCE("sdf_fxaa_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Grid Evaluation (samples SDF at 3D grid vertices)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_grid_eval_comp)
LOCAL_GROUP_SIZE(4, 4, 4)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(3, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(4, write, float, grid_values[])
STORAGE_BUF(5, read, SdfAabbNodeGPU, aabb_nodes[])
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(int, grid_verts)
PUSH_CONSTANT(float3, grid_origin)
PUSH_CONSTANT(float, cell_size)
PUSH_CONSTANT(int, use_bvh)
PUSH_CONSTANT(int, bvh_root)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_grid_eval_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Dual Contouring (generates vertices from grid sign changes)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_dc_contour_comp)
LOCAL_GROUP_SIZE(4, 4, 4)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, float, grid_values[])
STORAGE_BUF(1, write, float4, dc_vertices[])
STORAGE_BUF(2, read_write, int, dc_counters[])
STORAGE_BUF(3, write, int, dc_cell_verts[])
PUSH_CONSTANT(int, grid_verts)
PUSH_CONSTANT(float3, grid_origin)
PUSH_CONSTANT(float, cell_size)
PUSH_CONSTANT(int, max_verts)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_dc_contour_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF DC Triangulation (pass 2: emit triangles from cell vertex map)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_dc_triangulate_comp)
LOCAL_GROUP_SIZE(4, 4, 4)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, float, grid_values[])
STORAGE_BUF(1, write, int4, dc_triangles[])
STORAGE_BUF(2, read_write, int, dc_counters[])
STORAGE_BUF(3, read, int, dc_cell_verts[])
PUSH_CONSTANT(int, grid_verts)
PUSH_CONSTANT(int, inner_start)
PUSH_CONSTANT(int, inner_end)
PUSH_CONSTANT(int, max_tris)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_dc_triangulate_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF DC Vertex Color (samples SDF color at each DC vertex position)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_dc_vertex_color_comp)
LOCAL_GROUP_SIZE(64)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(3, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(4, read, float4, dc_positions[])
STORAGE_BUF(5, write, float4, dc_colors[])
STORAGE_BUF(6, read, SdfAabbNodeGPU, aabb_nodes[])
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(int, vert_count)
PUSH_CONSTANT(int, use_bvh)
PUSH_CONSTANT(int, bvh_root)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_dc_vertex_color_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Outline Prepass (writes packed object IDs for outline detection)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_outline_prepass)
DO_STATIC_COMPILATION()
STORAGE_BUF(7, read, uint, outline_ids[])
SAMPLER(0, sampler2D, sdf_depth_tx)
SAMPLER(1, sampler2D, sdf_gbuf_color_tx)
SAMPLER(2, sampler2D, scene_depth_tx)
PUSH_CONSTANT(float2, uv_scale)
FRAGMENT_OUT(0, uint, out_object_id)
DEPTH_WRITE(DepthWrite::ANY)
ADDITIONAL_INFO(gpu_fullscreen)
FRAGMENT_SOURCE("sdf_outline_prepass_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Analytical Pick (sphere-trace for GPU selection)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_analytical_pick)
DO_STATIC_COMPILATION()
SAMPLER(0, sampler2D, sdf_depth_tx)
SAMPLER(1, sampler2D, sdf_gbuf_color_tx)
STORAGE_BUF(7, read, uint, select_id_map_buf[])
STORAGE_BUF(6, read_write, uint, out_select_buf[])
UNIFORM_BUF(4, SelectInfoData, select_info_buf)
PUSH_CONSTANT(float2, uv_scale)
DEPTH_WRITE(DepthWrite::ANY)
TYPEDEF_SOURCE("select_shader_shared.hh")
ADDITIONAL_INFO(gpu_fullscreen)
FRAGMENT_SOURCE("sdf_analytical_pick_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */
