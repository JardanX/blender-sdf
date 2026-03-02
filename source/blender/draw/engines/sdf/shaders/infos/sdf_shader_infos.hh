/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"
#endif

#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name SDF Bake Compute Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_bake)
DO_STATIC_COMPILATION()
LOCAL_GROUP_SIZE(8, 8, 4)
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
IMAGE(0, SFLOAT_16_16_16_16, write, image3D, sdf_atlas)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(int3, atlas_resolution)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_bake_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Ray-March Fragment Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_march)
DO_STATIC_COMPILATION()
SAMPLER(0, sampler3D, sdf_atlas)
SAMPLER(1, sampler2DDepth, depth_tx)
SAMPLER(2, sampler2DArray, matcap_tx)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(float3, atlas_extent)
PUSH_CONSTANT(int3, atlas_resolution)
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
