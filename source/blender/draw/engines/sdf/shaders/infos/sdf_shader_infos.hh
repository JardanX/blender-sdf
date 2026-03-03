/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"
#endif

#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name SDF Classify Compute Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_classify)
DO_STATIC_COMPILATION()
LOCAL_GROUP_SIZE(4, 4, 4)
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read_write, BrickCounter, brick_counter)
IMAGE(0, SINT_32, write, iimage3D, indirection_tex)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(int3, grid_resolution)
PUSH_CONSTANT(float, brick_half_diag)
PUSH_CONSTANT(float, max_blend)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_classify_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Bake Compute Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_bake)
DO_STATIC_COMPILATION()
LOCAL_GROUP_SIZE(12, 12, 1)
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
SAMPLER(0, isampler3D, indirection_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image3D, compact_atlas)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(int3, grid_resolution)
PUSH_CONSTANT(int, bricks_per_axis)
PUSH_CONSTANT(float, max_blend)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_bake_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Grid Blend Compute Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_grid_blend)
DO_STATIC_COMPILATION()
LOCAL_GROUP_SIZE(12, 12, 1)
SAMPLER(0, isampler3D, indirection_tx)
SAMPLER(1, sampler3D, sdf_grid)
IMAGE(0, SFLOAT_16_16_16_16, read_write, image3D, compact_atlas)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(int3, grid_resolution)
PUSH_CONSTANT(int, bricks_per_axis)
PUSH_CONSTANT(float4x4, grid_world_to_texture)
PUSH_CONSTANT(float4, grid_color)
PUSH_CONSTANT(float, grid_blend)
COMPUTE_SOURCE("sdf_grid_blend_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Ray-March Fragment Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_march)
DO_STATIC_COMPILATION()
SAMPLER(0, sampler3D, compact_atlas)
SAMPLER(1, isampler3D, indirection_tx)
SAMPLER(2, sampler2DDepth, depth_tx)
SAMPLER(3, sampler2DArray, matcap_tx)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(float3, atlas_extent)
PUSH_CONSTANT(int3, grid_resolution)
PUSH_CONSTANT(int, bricks_per_axis)
PUSH_CONSTANT(int, lighting_type)
PUSH_CONSTANT(int, use_specular)
PUSH_CONSTANT(int, use_matcap_flip)
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
FRAGMENT_OUT(0, float4, out_color)
DEPTH_WRITE(DepthWrite::ANY)
ADDITIONAL_INFO(gpu_fullscreen)
ADDITIONAL_INFO(draw_view)
FRAGMENT_SOURCE("sdf_march_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */
