/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF bake compute shader.
 * Evaluates all SDF objects at each voxel and writes minimum distance to the 3D atlas.
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_bake)

#include "sdf_lib.glsl"

void main()
{
  int3 voxel = int3(gl_GlobalInvocationID);

  if (any(greaterThanEqual(voxel, atlas_resolution.xyz))) {
    return;
  }

  /* World-space position of this voxel center. */
  float3 world_pos = atlas_origin + (float3(voxel) + 0.5f) * voxel_size;

  float acc_dist = 1e10f;
  float3 acc_color = float3(0.0f);

  for (int i = 0; i < object_count; i++) {
    SDFObjectGPU obj = objects[i];

    /* Transform to local space (rotation-only inverse). */
    float3 local_pos = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;

    /* Evaluate sdBox with bevel (external Minkowski). */
    float3 size = obj.sdf_size.xyz - obj.bevel;
    size = max(size, float3(0.001f));
    float dist = sdBox(local_pos, size) - obj.bevel;

    /* Smooth union when blend > 0, hard union otherwise.
     * Color is blended with the same weight as the distance. */
    float k = obj.blend;
    if (k > 0.0f && acc_dist < 1e9f) {
      float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
      acc_color = mix(acc_color, obj.color.rgb, h);
      acc_dist = mix(acc_dist, dist, h) - k * h * (1.0f - h);
    }
    else {
      if (dist < acc_dist) {
        acc_color = obj.color.rgb;
        acc_dist = dist;
      }
    }
  }

  imageStore(sdf_atlas, voxel, float4(acc_dist, acc_color));
}
