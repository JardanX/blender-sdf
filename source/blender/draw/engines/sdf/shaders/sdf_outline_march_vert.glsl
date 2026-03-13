/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF outline vertex shader — instanced AABB box rasterization.
 *
 * Each instance is one selected SDF object. The vertex shader transforms
 * a procedural unit cube [0,1]^3 to the object's world-space AABB, then
 * projects to clip space. The flat `obj_index` output tells the fragment
 * shader which single object to sphere-march.
 *
 * This replaces the old fullscreen approach (O(pixels * objects)) with
 * an instanced approach (O(covered_pixels)) — each fragment evaluates
 * exactly one object, and the GPU depth test resolves overlaps.
 */

#include "infos/sdf_shader_infos.hh"

VERTEX_SHADER_CREATE_INFO(sdf_outline_march)

#include "draw_view_lib.glsl"

/* 36 vertices forming 12 triangles of a unit cube [0,1]^3.
 * Outward-facing normals (CCW winding from outside). Used with
 * GPU_CULL_FRONT so back faces render — this ensures fragments are
 * generated both when the camera is outside and inside an AABB. */
const float3 cube_verts[36] = float3[36](
    /* -Z face */
    float3(0, 0, 0), float3(1, 1, 0), float3(1, 0, 0),
    float3(0, 0, 0), float3(0, 1, 0), float3(1, 1, 0),
    /* +Z face */
    float3(0, 0, 1), float3(1, 0, 1), float3(1, 1, 1),
    float3(0, 0, 1), float3(1, 1, 1), float3(0, 1, 1),
    /* -X face */
    float3(0, 0, 0), float3(0, 0, 1), float3(0, 1, 1),
    float3(0, 0, 0), float3(0, 1, 1), float3(0, 1, 0),
    /* +X face */
    float3(1, 0, 0), float3(1, 1, 0), float3(1, 1, 1),
    float3(1, 0, 0), float3(1, 1, 1), float3(1, 0, 1),
    /* -Y face */
    float3(0, 0, 0), float3(1, 0, 0), float3(1, 0, 1),
    float3(0, 0, 0), float3(1, 0, 1), float3(0, 0, 1),
    /* +Y face */
    float3(0, 1, 0), float3(0, 1, 1), float3(1, 1, 1),
    float3(0, 1, 0), float3(1, 1, 1), float3(1, 1, 0));

void main()
{
  int vert_id = gl_VertexID % 36;
  int instance_id = gl_InstanceID;

  /* Look up which SDF object this instance represents. */
  int idx = selected_indices[instance_id];
  SDFObjectGPU obj = sdf_objects[idx];

  /* Transform unit cube vertex to world-space AABB. */
  float3 unit_pos = cube_verts[vert_id];
  float3 world_pos = mix(obj.bbox_min.xyz, obj.bbox_max.xyz, unit_pos);

  /* Project to clip space. */
  ViewMatrices vm = drw_view();
  float4x4 persmat = vm.winmat * vm.viewmat;
  gl_Position = persmat * float4(world_pos, 1.0f);

  /* Pass the SDF object index to the fragment shader. */
  obj_index = idx;
}
