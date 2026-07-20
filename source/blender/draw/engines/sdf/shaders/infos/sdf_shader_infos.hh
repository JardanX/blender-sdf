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
DEFINE_VALUE("kMaxTileObjects", "256")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(3, read, SdfAabbNodeGPU, aabb_nodes[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
STORAGE_BUF(11, read_write, uint, prof_eval_counts[])
STORAGE_BUF(12, read_write, uint, prof_trace_stats[])
STORAGE_BUF(9, read, uint4, mesh_data_buf[])
STORAGE_BUF(13, read, uint, bake_dist[])
STORAGE_BUF(14, read, uint, bake_nrm[])
STORAGE_BUF(15, read, uint, bake_col[])
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
PUSH_CONSTANT(int, prof_enabled)
PUSH_CONSTANT(int, skip_object)
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
DEFINE_VALUE("kMaxTileObjects", "256")
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
STORAGE_BUF(3, read, SdfAabbNodeGPU, aabb_nodes[])
STORAGE_BUF(4, write, float4, tile_hit_pos[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
STORAGE_BUF(5, read_write, int, tile_prim_counts[])
STORAGE_BUF(6, read, int, tile_prim_lists[])
STORAGE_BUF(7, write, float, tile_far_hint[])
STORAGE_BUF(9, read, uint4, mesh_data_buf[])
STORAGE_BUF(11, read, uint, bake_dist[])
STORAGE_BUF(12, read, uint, bake_nrm[])
STORAGE_BUF(13, read, uint, bake_col[])
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(float, sdf_cone_aperture)
PUSH_CONSTANT(int, sdf_cone_steps)
PUSH_CONSTANT(float3, scene_aabb_min)
PUSH_CONSTANT(float3, scene_aabb_max)
PUSH_CONSTANT(int, use_bvh)
PUSH_CONSTANT(int, bvh_root)
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
DEFINE_VALUE("kMaxTileObjects", "256")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(3, read, SdfAabbNodeGPU, aabb_nodes[])
STORAGE_BUF(4, read, float4, tile_hit_pos[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
STORAGE_BUF(11, read_write, uint, prof_eval_counts[])
STORAGE_BUF(12, read_write, uint, prof_trace_stats[])
STORAGE_BUF(5, read, int, tile_prim_counts[])
STORAGE_BUF(6, read, int, tile_prim_lists[])
STORAGE_BUF(9, read, uint4, mesh_data_buf[])
STORAGE_BUF(13, read, uint, bake_dist[])
STORAGE_BUF(14, read, uint, bake_nrm[])
STORAGE_BUF(15, read, uint, bake_col[])
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
PUSH_CONSTANT(int, prof_enabled)
PUSH_CONSTANT(int, skip_object)
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
DEFINE_VALUE("kMaxTileObjects", "256")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(3, read, SdfAabbNodeGPU, aabb_nodes[])
STORAGE_BUF(5, read, int, tile_prim_counts[])
STORAGE_BUF(6, read, int, tile_prim_lists[])
STORAGE_BUF(8, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(10, read, SDFObjectAABB, object_aabbs[])
STORAGE_BUF(9, read, uint4, mesh_data_buf[])
STORAGE_BUF(11, read, uint, bake_dist[])
STORAGE_BUF(12, read, uint, bake_nrm[])
STORAGE_BUF(13, read, uint, bake_col[])
IMAGE(0, SFLOAT_32_32_32_32, read, image2D, gbuf_pos_img)
IMAGE(1, SFLOAT_16_16_16_16, read_write, image2D, gbuf_color_img)
IMAGE(2, SFLOAT_16_16_16_16, write, image2D, gbuf_normal_img)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(int, use_bvh)
PUSH_CONSTANT(int, bvh_root)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_color_resolve_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Normal Compute Shader (screen-space reconstruction)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_normal_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
IMAGE(0, SFLOAT_32_32_32_32, read, image2D, gbuf_pos_img)
IMAGE(1, SFLOAT_16_16_16_16, read_write, image2D, gbuf_normal_img)
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
UNIFORM_BUF(1, SDFShadingDataGPU, shading_data)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
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
/** \name SDF Lipschitz Pruning Pass (builds per-cell pruned CSG trees)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_lp_prune_comp)
LOCAL_GROUP_SIZE(4, 4, 4)
/* Not statically compiled: the LP shaders are huge (sdf_lp_common.glsl) and
 * the NVIDIA driver spends minutes compiling them at startup. Compiling
 * lazily on first use lets the driver's disk shader cache take over on
 * subsequent runs. */
/* The prune pass only evaluates distances (its own forward pass): strip the
 * trace-side list evaluators (lp_list_eval/lp_list_eval_obj_id) from
 * sdf_lp_common.glsl to cut compile time. */
DEFINE_VALUE("SDF_LP_NO_LIST_EVAL", "1")
/* Active list node words are uint (index | sign); parent indices live in a
 * parallel uint array consumed only by this prune pass. Cell metadata packs
 * num_active in .x (SDF_LP_FALLBACK_LIST sentinel), the cell list offset in .y
 * and the float bits of the cell value in .z. */
STORAGE_BUF(0, read, SDFLpPrimitive, lp_prims[])
STORAGE_BUF(1, read, SDFLpNode, lp_nodes[])
STORAGE_BUF(2, read, uint4, lp_binary_ops[])
STORAGE_BUF(3, read, uint, lp_active_in[])
STORAGE_BUF(4, write, uint, lp_active_out[])
STORAGE_BUF(5, read, int4, lp_cell_meta_in[])
STORAGE_BUF(6, write, int4, lp_cell_meta_out[])
STORAGE_BUF(7, read_write, int, lp_counters[])
STORAGE_BUF(8, read_write, uint, lp_tmp[])
STORAGE_BUF(9, read_write, uint, lp_scratch[])
STORAGE_BUF(10, read, SDFObjectGPU, objects[])
STORAGE_BUF(11, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(12, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(13, read, uint4, mesh_data_buf[])
STORAGE_BUF(14, read, uint, lp_active_parents_in[])
STORAGE_BUF(15, write, uint, lp_active_parents_out[])
STORAGE_BUF(16, read, uint, bake_dist[])
PUSH_CONSTANT(float3, aabb_min)
PUSH_CONSTANT(float3, aabb_max)
PUSH_CONSTANT(int, total_num_nodes)
PUSH_CONSTANT(int, grid_size)
PUSH_CONSTANT(int, first_lvl)
PUSH_CONSTANT(int, active_capacity)
PUSH_CONSTANT(int, tmp_capacity)
PUSH_CONSTANT(int, counter_slot)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_lp_prune_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Lipschitz March (sphere tracing over per-cell pruned trees)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_lp_march_comp)
LOCAL_GROUP_SIZE(8, 8)
/* Not statically compiled: see sdf_lp_prune_comp. Compiled lazily on first
 * use; the driver's disk shader cache makes subsequent startups fast. */
/* Distance-only: the march pass evaluates the pruned tree for distances and
 * seeds gbuf_color.a with the dominant object id (light lp_list_eval_obj_id
 * fold) for picking; hit color and normals come from the shared classic
 * passes (sdf_color_resolve_comp / sdf_normal_comp). */
STORAGE_BUF(0, read, SDFLpPrimitive, lp_prims[])
STORAGE_BUF(1, read, SDFLpNode, lp_nodes[])
STORAGE_BUF(2, read, uint4, lp_binary_ops[])
STORAGE_BUF(3, read, uint, lp_active_in[])
STORAGE_BUF(4, read, int4, lp_cell_meta[])
STORAGE_BUF(5, read, SDFObjectGPU, objects[])
STORAGE_BUF(6, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(7, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(8, read, uint4, mesh_data_buf[])
STORAGE_BUF(9, read, uint, bake_dist[])
IMAGE(0, SFLOAT_32_32_32_32, write, image2D, gbuf_pos_img)
IMAGE(1, SFLOAT_16_16_16_16, write, image2D, gbuf_color_img)
PUSH_CONSTANT(float3, aabb_min)
PUSH_CONSTANT(float3, aabb_max)
PUSH_CONSTANT(int, grid_size)
PUSH_CONSTANT(int, total_num_nodes)
PUSH_CONSTANT(int, culling_enabled)
PUSH_CONSTANT(int, max_steps)
PUSH_CONSTANT(float, ray_epsilon)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(draw_view)
COMPUTE_SOURCE("sdf_lp_march_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Lipschitz Debug Colorize (heatmap / normals viz)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_lp_debug_comp)
LOCAL_GROUP_SIZE(8, 8)
/* Not statically compiled: see sdf_lp_prune_comp. */
/* Tiny shader (cell metadata lookup / normal viz only, no SDF tree
 * evaluation): debug shading modes stay cheap to compile. */
STORAGE_BUF(0, read, int4, lp_cell_meta[])
IMAGE(0, SFLOAT_32_32_32_32, read, image2D, gbuf_pos_img)
IMAGE(1, SFLOAT_16_16_16_16, read, image2D, gbuf_normal_img)
IMAGE(2, SFLOAT_16_16_16_16, read_write, image2D, gbuf_color_img)
PUSH_CONSTANT(float3, aabb_min)
PUSH_CONSTANT(float3, aabb_max)
PUSH_CONSTANT(int, grid_size)
PUSH_CONSTANT(int, total_num_nodes)
PUSH_CONSTANT(int, culling_enabled)
PUSH_CONSTANT(int, shading_mode)
PUSH_CONSTANT(float, viz_max)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_lp_debug_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Mesh Volume Bake (dense per-mesh SDF/normal/color voxel bake)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_mesh_bake_comp)
LOCAL_GROUP_SIZE(4, 4, 4)
/* Not statically compiled: see sdf_lp_prune_comp. Compiled lazily on first
 * use; the driver's disk shader cache makes subsequent startups fast. */
/* The bake only walks the mesh BVH (lp_mesh_nearest): strip the trace-side
 * list evaluators (lp_list_eval/lp_list_eval_obj_id) from sdf_lp_common.glsl
 * to cut compile time. The lp_* / modifier / polygon buffers below are still
 * declared because sdf_lp_common.glsl references them in other (dead-code)
 * functions — declarations must exist or the shader does not compile. */
DEFINE_VALUE("SDF_LP_NO_LIST_EVAL", "1")
STORAGE_BUF(0, read, SDFLpPrimitive, lp_prims[])
STORAGE_BUF(1, read, SDFLpNode, lp_nodes[])
STORAGE_BUF(2, read, uint4, lp_binary_ops[])
STORAGE_BUF(3, read, uint, lp_active_in[])
STORAGE_BUF(4, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(5, read, SDFPolygonPointGPU, polygon_points[])
STORAGE_BUF(6, read, uint4, mesh_data_buf[])
/* Per-triangle corner colors (xyz = 3 packed RGBA8, w = 0). */
STORAGE_BUF(7, read, uint4, mesh_color_buf[])
/* Shared append-only voxel pools, one uint per element; see
 * sdf_mesh_bake_comp.glsl for the layout. bake_dist is read_write because
 * sdf_lp_common.glsl (included below) also reads it in the baked-volume
 * distance sampler. */
STORAGE_BUF(8, read_write, uint, bake_dist[])
STORAGE_BUF(9, write, uint, bake_nrm[])
STORAGE_BUF(10, write, uint, bake_col[])
/* vertex start, triangle start, triangle count, BVH node start (uint4-record
 * offsets into mesh_data_buf, same layout as SDFLpPrimitive.mesh_data). */
PUSH_CONSTANT(int4, mesh_data)
PUSH_CONSTANT(int, mesh_node_count)
/* uint4-record offset into mesh_color_buf. */
PUSH_CONSTANT(int, color_start)
/* xyz = voxel grid resolution, w = first voxel index (same in all pools). */
PUSH_CONSTANT(int4, res_and_base)
/* Local UNSCALED mesh space min corner of the voxel grid. */
PUSH_CONSTANT(float3, origin)
PUSH_CONSTANT(float, voxel_size)
/* Narrow band half-width (4 * voxel_size); distances clamp to +/-band. */
PUSH_CONSTANT(float, band)
PUSH_CONSTANT(int, has_colors)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_mesh_bake_comp.glsl")
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
STORAGE_BUF(9, read, uint4, mesh_data_buf[])
STORAGE_BUF(6, read, uint, bake_dist[])
STORAGE_BUF(7, read, uint, bake_nrm[])
STORAGE_BUF(8, read, uint, bake_col[])
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
STORAGE_BUF(9, read, uint4, mesh_data_buf[])
STORAGE_BUF(7, read, uint, bake_dist[])
STORAGE_BUF(8, read, uint, bake_nrm[])
STORAGE_BUF(10, read, uint, bake_col[])
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
PUSH_CONSTANT(int, outline_id_count)
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

/* -------------------------------------------------------------------- */
/** \name SDF Edge Detect (outline rendering for SDF objects)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_edge_detect)
DO_STATIC_COMPILATION()
SAMPLER(0, sampler2D, sdf_depth_tx)
SAMPLER(1, sampler2D, sdf_gbuf_color_tx)
SAMPLER(2, sampler2D, scene_depth_tx)
PUSH_CONSTANT(float2, uv_scale)
PUSH_CONSTANT(float, line_opacity)
FRAGMENT_OUT(0, float4, out_color)
DEPTH_WRITE(DepthWrite::ANY)
ADDITIONAL_INFO(gpu_fullscreen)
FRAGMENT_SOURCE("sdf_edge_detect_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */
