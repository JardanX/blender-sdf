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
STORAGE_BUF(2, write, ActiveBrick, active_bricks[])
STORAGE_BUF(3, read, BVHNodeGPU, bvh_nodes[])
STORAGE_BUF(4, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(5, read, SDFGroupGPU, groups[])
STORAGE_BUF(6, read, int, dirty_flags[])
IMAGE(0, SINT_32, read_write, iimage3D, indirection_tex)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(int3, grid_resolution)
PUSH_CONSTANT(float, brick_half_diag)
PUSH_CONSTANT(int, bvh_node_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(int, incremental_mode)
PUSH_CONSTANT(int3, dirty_brick_min)
PUSH_CONSTANT(int3, dirty_brick_max)
PUSH_CONSTANT(int, has_dirty_flags)
PUSH_CONSTANT(int, max_active_bricks)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_classify_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Augment Grids Compute Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_augment_grids)
DO_STATIC_COMPILATION()
LOCAL_GROUP_SIZE(4, 4, 4)
STORAGE_BUF(1, read_write, BrickCounter, brick_counter)
STORAGE_BUF(2, write, ActiveBrick, active_bricks[])
IMAGE(0, SINT_32, read_write, iimage3D, indirection_tex)
PUSH_CONSTANT(int3, grid_brick_min)
PUSH_CONSTANT(int3, grid_brick_max)
PUSH_CONSTANT(int3, grid_resolution)
PUSH_CONSTANT(int, max_active_bricks)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
COMPUTE_SOURCE("sdf_augment_grids_comp.glsl")
GPU_SHADER_CREATE_END()

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Bake Compute Shader
 * \{ */

GPU_SHADER_CREATE_INFO(sdf_bake)
DO_STATIC_COMPILATION()
LOCAL_GROUP_SIZE(8, 8, 1)
STORAGE_BUF(0, read, SDFObjectGPU, objects[])
STORAGE_BUF(1, read, ActiveBrick, active_bricks[])
STORAGE_BUF(2, read, BVHNodeGPU, bvh_nodes[])
STORAGE_BUF(3, read, BrickCounter, brick_counter)
STORAGE_BUF(4, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(5, read, SDFGroupGPU, groups[])
STORAGE_BUF(6, read, int, dirty_flags[])
IMAGE(0, SFLOAT_16_16_16_16, write, image3D, compact_atlas)

PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(int, bricks_per_axis)
PUSH_CONSTANT(int, dispatch_width)
PUSH_CONSTANT(int, bvh_node_count)
PUSH_CONSTANT(int, group_count)
PUSH_CONSTANT(int, has_dirty_flags)
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
LOCAL_GROUP_SIZE(8, 8, 1)
STORAGE_BUF(0, read, ActiveBrick, active_bricks[])
STORAGE_BUF(4, read, SDFModifierGPU, sdf_modifiers[])
SAMPLER(0, sampler3D, sdf_grid)
IMAGE(0, SFLOAT_16_16_16_16, read_write, image3D, compact_atlas)
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(int, bricks_per_axis)
PUSH_CONSTANT(float4x4, grid_world_to_texture)
PUSH_CONSTANT(float4, grid_color)
PUSH_CONSTANT(float, grid_blend)
PUSH_CONSTANT(int, grid_blend_type)
PUSH_CONSTANT(int, grid_csg_operation)
PUSH_CONSTANT(float, grid_shell_distance)
PUSH_CONSTANT(int, grid_shell_mode)
PUSH_CONSTANT(int, active_brick_count)
PUSH_CONSTANT(int, dispatch_width)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
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
SAMPLER(2, sampler2DArray, matcap_tx)

STORAGE_BUF(0, read, SDFShapeGPU, shapes[])
STORAGE_BUF(1, read, SDFInstanceGPU, instances[])
STORAGE_BUF(2, read, BVHNodeGPU, bvh_nodes[])
STORAGE_BUF(3, read, int, shape_indir[])
STORAGE_BUF(4, read, SDFModifierGPU, sdf_modifiers[])
STORAGE_BUF(5, read, SDFObjectGPU, objects[])
STORAGE_BUF(6, read, SDFGroupGPU, groups[])
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float3, atlas_origin)
PUSH_CONSTANT(float3, atlas_extent)
PUSH_CONSTANT(int3, grid_resolution)
PUSH_CONSTANT(int, bricks_per_axis)
PUSH_CONSTANT(int, lighting_type)
PUSH_CONSTANT(int, use_specular)
PUSH_CONSTANT(int, use_matcap_flip)

PUSH_CONSTANT(int, use_instanced)
PUSH_CONSTANT(int, instance_count)
PUSH_CONSTANT(int, bvh_node_count)
PUSH_CONSTANT(int, object_count)
PUSH_CONSTANT(int, group_count)
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
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(gpu_fullscreen)
ADDITIONAL_INFO(draw_view)
FRAGMENT_SOURCE("sdf_march_frag.glsl")
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

/* -------------------------------------------------------------------- */
/** \name SDF Outline March — Instanced AABB Rasterization
 * \{ */

/**
 * Interface between vertex and fragment shaders for instanced SDF outline.
 * Each instance is one selected SDF object; the vertex shader passes the
 * object index so the fragment shader evaluates only that single object.
 */
GPU_SHADER_INTERFACE_INFO(sdf_outline_inst_iface)
FLAT(int, obj_index)
GPU_SHADER_INTERFACE_END()

GPU_SHADER_CREATE_INFO(sdf_outline_march)
DO_STATIC_COMPILATION()
STORAGE_BUF(0, read, uint, outline_id_map_buf[])
STORAGE_BUF(1, read, SDFObjectGPU, sdf_objects[])
STORAGE_BUF(2, read, int, selected_indices[])
STORAGE_BUF(4, read, SDFModifierGPU, sdf_modifiers[])
PUSH_CONSTANT(float, voxel_size)
PUSH_CONSTANT(float2, viewport_size_inv)
VERTEX_OUT(sdf_outline_inst_iface)
/* Using uint because 16bit uint can contain more ids than int. */
FRAGMENT_OUT(0, uint, out_object_id)
DEPTH_WRITE(DepthWrite::ANY)
TYPEDEF_SOURCE("sdf_shader_shared.hh")
ADDITIONAL_INFO(draw_view)
VERTEX_SOURCE("sdf_outline_march_vert.glsl")
FRAGMENT_SOURCE("sdf_outline_march_frag.glsl")
GPU_SHADER_CREATE_END()

/** \} */
