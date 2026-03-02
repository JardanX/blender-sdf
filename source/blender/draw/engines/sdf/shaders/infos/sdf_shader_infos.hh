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
IMAGE(0, SFLOAT_16, write, image3D, sdf_atlas)
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
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(float3, atlas_extent)
PUSH_CONSTANT(int, object_count)
FRAGMENT_OUT(0, float4, out_color)
DEPTH_WRITE(DepthWrite::ANY)
ADDITIONAL_INFO(gpu_fullscreen)
ADDITIONAL_INFO(draw_view)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
FRAGMENT_SOURCE("sdf_march_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */
