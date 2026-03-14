/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Per-shape SDF bake: one workgroup per brick, 12x12 threads, loop Z.
 * Thread 0 culls objects per-brick (AABB test) so voxels only evaluate nearby objects. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_shape_bake)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12
#define MAX_BRICK_OBJS 32

shared int shared_brick_objs[MAX_BRICK_OBJS];
shared int shared_brick_obj_count;

void main()
{
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= active_brick_count) {
    return;
  }

  int4 brick_data = active_bricks[brick_idx + brick_offset].coord;
  int3 brick = brick_data.xyz;
  int slot = brick_data.w;

  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  /* Thread 0: compute brick world AABB and cull objects. */
  if (gl_LocalInvocationIndex == 0u) {
    float3 b_local_min = local_origin +
                         float3(brick * BRICK_SIZE - int3(2)) * local_voxel_size;
    float3 b_local_max = local_origin +
                         float3(brick * BRICK_SIZE + BRICK_SIZE + int3(2)) * local_voxel_size;

    float3 bw_min = float3(1e10f);
    float3 bw_max = float3(-1e10f);
    for (int c = 0; c < 8; c++) {
      float3 corner = float3((c & 1) != 0 ? b_local_max.x : b_local_min.x,
                             (c & 2) != 0 ? b_local_max.y : b_local_min.y,
                             (c & 4) != 0 ? b_local_max.z : b_local_min.z);
      float3 wc = (local_to_world * float4(corner, 1.0f)).xyz;
      bw_min = min(bw_min, wc);
      bw_max = max(bw_max, wc);
    }

    shared_brick_obj_count = 0;
    for (int i = 0; i < object_count; i++) {
      SDFObjectGPU obj = objects[i];
      float expand = obj.blend;
      if (all(lessThanEqual(bw_min, obj.bbox_max.xyz + float3(expand))) &&
          all(greaterThanEqual(bw_max, obj.bbox_min.xyz - float3(expand))))
      {
        if (shared_brick_obj_count < MAX_BRICK_OBJS) {
          shared_brick_objs[shared_brick_obj_count++] = i;
        }
      }
    }
  }
  barrier();

  int num_brick_objs = shared_brick_obj_count;
  int2 local_xy = int2(gl_LocalInvocationID.xy);

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    float3 local_pos = local_origin +
                       float3(brick * BRICK_SIZE + local_voxel - int3(2)) *
                           local_voxel_size;

    float3 world_pos = (local_to_world * float4(local_pos, 1.0f)).xyz;

    float acc_dist = 1e10f;
    float3 acc_color = float3(0.0f);

    /* Group-aware CSG evaluation (only brick-culled objects). */
    for (int g = 0; g < group_count; g++) {
      SDFGroupGPU grp = groups[g];
      float grp_dist = 1e10f;
      float3 grp_color = float3(0.0f);

      for (int bc = 0; bc < num_brick_objs; bc++) {
        SDFObjectGPU obj = objects[shared_brick_objs[bc]];
        if (obj.group_id != g) {
          continue;
        }

        float3 obj_local = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;

        SDFPrimitiveData prim;
        prim.sdf_type = obj.sdf_type;
        prim.size = obj.sdf_size.xyz;
        prim.bevel = obj.bevel;
        prim.box_corners = obj.box_corners;
        prim.box_edges = obj.box_edges;
        prim.box_modes = obj.box_modes;
        prim.modifier_start = obj.modifier_start;
        prim.modifier_count = obj.modifier_count;

        float dist = evalObjectSDF(prim, obj_local);

        if (obj.group_first == 1) {
          grp_dist = dist;
          grp_color = obj.color.rgb;
        }
        else {
          float h = csgColorWeight(
              grp_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
          grp_color = mix(grp_color, obj.color.rgb, h);
          grp_dist = combineCSG(
              grp_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
        }
      }

      if (grp_dist >= 1e10f) {
        continue;
      }

      grp_color *= grp.color.rgb;

      if (acc_dist >= 1e10f) {
        acc_dist = grp_dist;
        acc_color = grp_color;
      }
      else {
        float h = csgColorWeight(
            acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
        acc_color = mix(acc_color, grp_color, h);
        acc_dist = combineCSG(
            acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
      }
    }

    /* Ungrouped objects. */
    for (int bc = 0; bc < num_brick_objs; bc++) {
      SDFObjectGPU obj = objects[shared_brick_objs[bc]];
      if (obj.group_id != -1) {
        continue;
      }

      float3 obj_local = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;

      SDFPrimitiveData prim;
      prim.sdf_type = obj.sdf_type;
      prim.size = obj.sdf_size.xyz;
      prim.bevel = obj.bevel;
      prim.box_corners = obj.box_corners;
      prim.box_edges = obj.box_edges;
      prim.box_modes = obj.box_modes;
      prim.modifier_start = obj.modifier_start;
      prim.modifier_count = obj.modifier_count;

      float dist = evalObjectSDF(prim, obj_local);

      if (acc_dist >= 1e10f) {
        acc_dist = dist;
        acc_color = obj.color.rgb;
      }
      else {
        float h = csgColorWeight(
            acc_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
        acc_color = mix(acc_color, obj.color.rgb, h);
        acc_dist = combineCSG(
            acc_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
      }
    }

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
  }
}
