/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"

#  include "draw_object_infos_infos.hh"
#  include "draw_view_infos.hh"

#  include "overlay_shader_shared.hh"
#endif

#include "overlay_common_infos.hh"

/* -------------------------------------------------------------------- */
/** \name SDF Picking (solid ray-march)
 * \{ */

GPU_SHADER_INTERFACE_INFO(overlay_sdf_pick_iface)
SMOOTH(float3, world_pos)
GPU_SHADER_INTERFACE_END()

GPU_SHADER_CREATE_INFO(overlay_sdf_pick_base)
VERTEX_IN(0, float3, pos)
VERTEX_OUT(overlay_sdf_pick_iface)
PUSH_CONSTANT(int, sdf_type)
PUSH_CONSTANT(float3, sdf_size)
PUSH_CONSTANT(float, sdf_bevel)
PUSH_CONSTANT(float3, sdf_bound)
DEPTH_WRITE(DepthWrite::ANY)
VERTEX_SOURCE("overlay_sdf_pick_vert.glsl")
FRAGMENT_SOURCE("overlay_sdf_pick_frag.glsl")
ADDITIONAL_INFO(draw_view)
ADDITIONAL_INFO(draw_globals)
ADDITIONAL_INFO(draw_resource_id_varying)
GPU_SHADER_CREATE_END()

OVERLAY_INFO_VARIATIONS_MODELMAT(overlay_sdf_pick, overlay_sdf_pick_base)

/** \} */
