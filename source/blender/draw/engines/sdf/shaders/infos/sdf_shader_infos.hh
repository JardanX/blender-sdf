/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"
#endif

#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name SDF Trace Compute Shader (Pass 1: sphere tracing → G-buffer)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_trace_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(3, read, SdfAabbNodeGPU, aabb_nodes[])
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, out_color_img)
IMAGE(1, SFLOAT_32, write, image2D, out_depth_img)
IMAGE(2, SFLOAT_32_32_32_32, write, image2D, gbuf_pos_img)
IMAGE(3, SFLOAT_16_16_16_16, write, image2D, gbuf_color_img)
IMAGE(4, SFLOAT_16_16_16_16, write, image2D, gbuf_normal_img)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(int, use_bvh)
PUSH_CONSTANT(int, bvh_root)
PUSH_CONSTANT(int, debug_bvh_views)
PUSH_CONSTANT(int, sdf_max_steps)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(float, sdf_over_relaxation)
PUSH_CONSTANT(float3, scene_aabb_min)
PUSH_CONSTANT(float3, scene_aabb_max)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(draw_view)
COMPUTE_SOURCE("sdf_trace_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Tile-Culled Trace Compute Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_trace_tile_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
DEFINE("USE_TILE_CULLING")
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
STORAGE_BUF(3, read, SdfAabbNodeGPU, aabb_nodes[])
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, out_color_img)
IMAGE(1, SFLOAT_32, write, image2D, out_depth_img)
IMAGE(2, SFLOAT_32_32_32_32, write, image2D, gbuf_pos_img)
IMAGE(3, SFLOAT_16_16_16_16, write, image2D, gbuf_color_img)
IMAGE(4, SFLOAT_16_16_16_16, write, image2D, gbuf_normal_img)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(int, use_bvh)
PUSH_CONSTANT(int, bvh_root)
PUSH_CONSTANT(int, debug_bvh_views)
PUSH_CONSTANT(int, sdf_max_steps)
PUSH_CONSTANT(float, sdf_ray_epsilon)
PUSH_CONSTANT(float, sdf_over_relaxation)
PUSH_CONSTANT(float3, scene_aabb_min)
PUSH_CONSTANT(float3, scene_aabb_max)
PUSH_CONSTANT(int2, screen_size)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(draw_view)
COMPUTE_SOURCE("sdf_trace_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Shade Compute Shader (Pass 2: lighting from G-buffer)
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_shade_comp)
LOCAL_GROUP_SIZE(8, 8)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(2, read, SDFGroupGPU, groups[])
SAMPLER(0, sampler2DArray, matcap_tx)
IMAGE(0, SFLOAT_32_32_32_32, read, image2D, gbuf_pos_img)
IMAGE(1, SFLOAT_16_16_16_16, read, image2D, gbuf_color_img)
IMAGE(2, SFLOAT_16_16_16_16, write, image2D, out_color_img)
IMAGE(3, SFLOAT_32, write, image2D, out_depth_img)
IMAGE(4, SFLOAT_16_16_16_16, read, image2D, gbuf_normal_img)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
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
/** \name SDF Selection March Fragment Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_select_march)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, uint, select_id_map_buf[])
STORAGE_BUF(1, read, SDFObjectGPU, sdf_objects[])
STORAGE_BUF(4, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(5, read, SDFGroupGPU, groups[])
/* Selection bindings: match select_id_patch slots without the vertex interface. */
UNIFORM_BUF(4, SelectInfoData, select_info_buf)
STORAGE_BUF(6, read_write, uint, out_select_buf[])
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
DEPTH_WRITE(DepthWrite::ANY)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
TYPEDEF_SOURCE("select_shader_shared.hh")
ADDITIONAL_INFO(gpu_fullscreen)
ADDITIONAL_INFO(draw_view)
FRAGMENT_SOURCE("sdf_select_march_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */

