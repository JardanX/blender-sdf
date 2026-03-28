/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF bake compute shader (sparse brick version).
 * Each workgroup handles one brick. Local threads cover the 12x12 XY slice,
 * looping over Z to fill 12x12x12 voxels (8 inner + 2 overlap each side).
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_bake)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12
#define MAX_CANDIDATES 64

/* SDF primitive types (must match eSDFType in DNA_sdf_types.h). */
#define SDF_TYPE_BOX 0
#define SDF_TYPE_SPHERE 1
#define SDF_TYPE_CAPSULE 4
#define SDF_TYPE_TORUS 5

float sdSphere(float3 p, float r)
{
  return length(p) - r;
}

float sdEllipsoid(float3 p, float3 r)
{
  float k0 = length(p / r);
  if (k0 < 0.0001f) {
    return -min(min(r.x, r.y), r.z);
  }
  float k1 = length(p / (r * r));
  return k0 * (k0 - 1.0f) / k1;
}

float sdCapsule(float3 p, float3 size)
{
  float h = size.y;
  float r = size.x;
  p.y -= clamp(p.y, -h, h);
  return length(p) - r;
}

float sdTorus(float3 p, float2 t)
{
  float2 q = float2(length(p.xz) - t.x, p.y);
  return length(q) - t.y;
}

/* Compact per-object data cached in shared memory.
 * Only the fields needed for SDF evaluation — avoids re-reading the
 * full 160-byte SDFObjectGPU struct from SSBO for every voxel. */
struct SharedObj {
  float4x4 inverse_matrix;
  float4 position;
  float4 sdf_size;
  float4 color;
  float bevel;
  float blend;
  int sdf_type;
  int obj_index;
};

/** Evaluate the actual SDF primitive (reads from shared memory cache). */
float evalSDFPrimitiveSh(float3 local_pos, SharedObj obj)
{
  float3 size = obj.sdf_size.xyz;
  float bevel = obj.bevel;
  float dist;

  if (obj.sdf_type == SDF_TYPE_SPHERE) {
    float3 r = size - float3(bevel);
    r = max(r, float3(0.001f));
    if (abs(r.x - r.y) < 0.0001f && abs(r.x - r.z) < 0.0001f) {
      dist = sdSphere(local_pos, r.x);
    }
    else {
      dist = sdEllipsoid(local_pos, r);
    }
  }
  else if (obj.sdf_type == SDF_TYPE_CAPSULE) {
    float3 cap_size = size - float3(bevel);
    cap_size = max(cap_size, float3(0.001f));
    dist = sdCapsule(local_pos, cap_size);
  }
  else if (obj.sdf_type == SDF_TYPE_TORUS) {
    float major = size.x - bevel;
    float minor = size.y - bevel;
    major = max(major, 0.001f);
    minor = max(minor, 0.001f);
    dist = sdTorus(local_pos, float2(major, minor));
  }
  else {
    /* SDF_TYPE_BOX (default). */
    float3 box_size = size - float3(bevel);
    box_size = max(box_size, float3(0.001f));
    dist = sdBox(local_pos, box_size);
  }

  return dist - bevel;
}

/* Shared candidate list and object cache: BVH traversal done once per
 * workgroup by thread 0, object data loaded cooperatively by all threads.
 * Eliminates redundant SSBO reads across 144 threads x 12 Z iterations. */
shared int shared_candidates[MAX_CANDIDATES];
shared int shared_num_candidates;
/* MATHOPS: SharedObj cache removed for 5.1 shader preprocessor compat.
 * Object data is re-read from SSBO per invocation instead. */

void main()
{
  /* Active-brick-only dispatch: one workgroup per active brick.
   * 2D dispatch to avoid GL's 65535 workgroup limit per axis.
   * Read active count from SSBO (avoids CPU readback stall between classify and bake). */
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= int(brick_counter.count)) {
    return;
  }

  int4 brick_data = active_bricks[brick_idx].coord;
  int3 brick = brick_data.xyz;
  int packed_w = brick_data.w;
  int slot = packed_w >> 2;
  int lod = packed_w & 3;

  int scale_shift = 2 - lod;
  float lod_voxel_size = voxel_size * float(1 << scale_shift);

  /* Compute slot origin in compact atlas. */
  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  float3 brick_base = atlas_origin + float3(brick * BRICK_SIZE) * voxel_size;

  /* Per-brick AABB for object culling (includes 2-voxel overlap border).
   * Expand by max_blend so objects contributing to smooth union are evaluated.
   * Objects outside max_blend of the storage region get h=0 in the smooth
   * union (no contribution), so this is safe for cross-brick consistency. */
  float3 brick_min = brick_base - 2.0f * lod_voxel_size - float3(max_blend);
  float3 brick_max = brick_base + (float(BRICK_SIZE) + 2.0f) * lod_voxel_size + float3(max_blend);

  /* Elect thread 0 to collect candidates for the entire workgroup.
   * All threads in the brick share the same AABB, so the candidate list
   * is identical — no need for each thread to traverse independently. */
  if (gl_LocalInvocationIndex == 0u) {
    shared_num_candidates = 0;

    if (bvh_node_count > 0) {
      int stack[BVH_MAX_STACK];
      int sp = 0;
      stack[sp++] = 0;

      while (sp > 0) {
        int node_idx = stack[--sp];
        BVHNodeGPU node = bvh_nodes[node_idx];

        if (!aabb_overlap(
                brick_min, brick_max, node.min_and_left.xyz, node.max_and_right.xyz))
        {
          continue;
        }

        int left = bvh_decode_int(node.min_and_left.w);
        int right = bvh_decode_int(node.max_and_right.w);

        if (left == -1) {
          if (shared_num_candidates < MAX_CANDIDATES) {
            shared_candidates[shared_num_candidates++] = right;
          }
        }
        else {
          if (sp < BVH_MAX_STACK - 1) {
            stack[sp++] = left;
            stack[sp++] = right;
          }
        }
      }

      /* Sort candidates by object index (insertion sort, small N). */
      for (int i = 1; i < shared_num_candidates; i++) {
        int key = shared_candidates[i];
        int j = i - 1;
        while (j >= 0 && shared_candidates[j] > key) {
          shared_candidates[j + 1] = shared_candidates[j];
          j--;
        }
        shared_candidates[j + 1] = key;
      }
    }
    else {
      /* Fallback: linear scan collects all AABB-overlapping objects. */
      for (int i = 0; i < object_count; i++) {
        SDFObjectGPU obj = objects[i];

        if (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
            any(lessThan(brick_max, obj.bbox_min.xyz)))
        {
          continue;
        }

        if (shared_num_candidates < MAX_CANDIDATES) {
          shared_candidates[shared_num_candidates++] = i;
        }
      }
      /* Already in index order, no sort needed. */
    }
  }
  barrier();

  int num_candidates = shared_num_candidates;

  /* Local thread covers XY, loop over Z. */
  int2 local_xy = int2(gl_LocalInvocationID.xy);

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* World-space position */
    float3 world_pos = brick_base + (float3(local_voxel - int3(2)) + 0.5f) * lod_voxel_size;

    float acc_dist = 1e10f;
    float3 acc_color = float3(0.0f);
    int acc_obj_id = -1;
    float closest_raw_dist = 1e10f;

    for (int c = 0; c < num_candidates; c++) {
      int obj_idx = shared_candidates[c];
      SDFObjectGPU sobj = objects[obj_idx];

      float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;

      SharedObj sobj_sh;
      sobj_sh.inverse_matrix = sobj.inverse_matrix;
      sobj_sh.position = sobj.position;
      sobj_sh.sdf_size = sobj.sdf_size;
      sobj_sh.color = sobj.color;
      sobj_sh.bevel = sobj.bevel;
      sobj_sh.blend = sobj.blend;
      sobj_sh.sdf_type = sobj.sdf_type;
      sobj_sh.obj_index = obj_idx;
      float dist = evalSDFPrimitiveSh(local_pos, sobj_sh);

      if (dist < closest_raw_dist) {
        closest_raw_dist = dist;
        acc_obj_id = sobj_sh.obj_index;
      }

      float k = sobj_sh.blend;
      if (k > 0.0f && acc_dist < 1e9f) {
        float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
        acc_color = mix(acc_color, sobj_sh.color.rgb, h);
        acc_dist = mix(acc_dist, dist, h) - k * h * (1.0f - h);
      }
      else {
        if (dist < acc_dist) {
          acc_color = sobj_sh.color.rgb;
          acc_dist = dist;
        }
      }
    }

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
    imageStore(object_id_atlas, atlas_coord, int4(acc_obj_id, 0, 0, 0));
  }
}

