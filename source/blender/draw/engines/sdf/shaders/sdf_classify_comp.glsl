/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF classify compute shader (Virtual Octree).
 * Evaluates blocks of 4x4x4 fine bricks (LOD0).
 * Subdivides to 2x2x2 (LOD1) and 1x1x1 (LOD2) based on SDF linearity.
 * Outputs flat indirection texture with identical packed pointers for coarse blocks.
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_classify)

layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;

#include "sdf_lib.glsl"

/** Insert into hash table using relative coordinates (p - brick_offset). */
void hashInsert(int3 p, int value) {
  int3 rel = p - brick_offset;
  uint key = hashBrickKey(rel);
  uint start_idx = key % uint(SDF_HASH_TABLE_SIZE);
  for (uint i = 0u; i < 32u; i++) {
    uint idx = (start_idx + i) % uint(SDF_HASH_TABLE_SIZE);
    uint old_key = atomicCompSwap(hash_table[idx].key, 0xFFFFFFFFu, key);
    if (old_key == 0xFFFFFFFFu || old_key == key) {
      hash_table[idx].value = value;
      return;
    }
  }
}

#define BRICK_SIZE 8
#define MAX_CANDIDATES 64

#define LOD0_SCALE 4
#define LOD1_SCALE 2
#define LOD2_SCALE 1

/* SDF primitive types. */
#define SDF_TYPE_BOX 0
#define SDF_TYPE_SPHERE 1
#define SDF_TYPE_CAPSULE 4
#define SDF_TYPE_TORUS 5

float sdSphere(float3 p, float r) { return length(p) - r; }
float sdCapsule(float3 p, float3 size) {
  float h = size.y;
  float r = size.x;
  p.y -= clamp(p.y, -h, h);
  return length(p) - r;
}
float sdTorus(float3 p, float2 t) {
  float2 q = float2(length(p.xz) - t.x, p.y);
  return length(q) - t.y;
}

float evalSDFPrimitive(float3 local_pos, SDFObjectGPU obj) {
  float3 size = obj.sdf_size.xyz;
  float bevel = obj.bevel;
  if (obj.sdf_type == SDF_TYPE_SPHERE) {
    return sdSphere(local_pos, size.x - bevel) - bevel;
  }
  if (obj.sdf_type == SDF_TYPE_CAPSULE) {
    float3 cap_size = max(size - float3(bevel), float3(0.001f));
    return sdCapsule(local_pos, cap_size) - bevel;
  }
  if (obj.sdf_type == SDF_TYPE_TORUS) {
    float major = max(size.x - bevel, 0.001f);
    float minor = max(size.y - bevel, 0.001f);
    return sdTorus(local_pos, float2(major, minor)) - bevel;
  }
  float3 box_size = max(size - float3(bevel), float3(0.001f));
  return sdBox(local_pos, box_size) - bevel;
}

shared int shared_candidates[MAX_CANDIDATES];
shared int shared_num_candidates;

shared int lod0_status_status;
shared int lod0_status_slot;
shared int lod1_status_status[8];
shared int lod1_status_slot[8];

/** Evaluate full SDF for a set of candidates. */
float evalSDF(float3 pos, int num_candidates) {
  float acc_dist = 1e10f;
  for (int c = 0; c < num_candidates; c++) {
    SDFObjectGPU obj = objects[shared_candidates[c]];
    float3 local_pos = (obj.inverse_matrix * float4(pos - obj.position.xyz, 1.0f)).xyz;
    float dist = evalSDFPrimitive(local_pos, obj);
    float k = obj.blend;
    if (k > 0.0f && acc_dist < 1e9f) {
      float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
      acc_dist = mix(acc_dist, dist, h) - k * h * (1.0f - h);
    } else {
      acc_dist = min(acc_dist, dist);
    }
  }
  return acc_dist;
}

/** Evaluate block properties: distance at center and linearity error.
 * Uses MathOPS Engine 2's 7-point cross stencil for ultra-fast classification. */
void evaluateBlock(float3 center, float half_size, int num_candidates, float scaled_threshold, float surf_t, out float d_c, out float error, out bool is_empty) {
  /* Phase 1: Center eval */
  d_c = evalSDF(center, num_candidates);

  /* Fast rejection: if center is far enough, the entire block is empty (inside or outside).
   * Multiply by 2.0 for conservative bound to ensure no thin features are missed. */
  if (abs(d_c) > surf_t * 2.0f) {
    is_empty = true;
    error = 0.0f;
    return;
  }

  /* Sample at quarter-chunk spacing for second-order finite differences */
  float h = half_size * 0.5f;

  float dx_p = evalSDF(center + float3(h, 0.0f, 0.0f), num_candidates);
  float dx_m = evalSDF(center - float3(h, 0.0f, 0.0f), num_candidates);
  float dy_p = evalSDF(center + float3(0.0f, h, 0.0f), num_candidates);
  float dy_m = evalSDF(center - float3(0.0f, h, 0.0f), num_candidates);
  float dz_p = evalSDF(center + float3(0.0f, 0.0f, h), num_candidates);
  float dz_m = evalSDF(center - float3(0.0f, 0.0f, h), num_candidates);

  /* Thin feature detection: if all 7 points are far from the surface, it is empty. */
  int surf_count = 0;
  if (abs(d_c)  < surf_t) { surf_count++; }
  if (abs(dx_p) < surf_t) { surf_count++; }
  if (abs(dx_m) < surf_t) { surf_count++; }
  if (abs(dy_p) < surf_t) { surf_count++; }
  if (abs(dy_m) < surf_t) { surf_count++; }
  if (abs(dz_p) < surf_t) { surf_count++; }
  if (abs(dz_m) < surf_t) { surf_count++; }

  if (surf_count == 0) {
    is_empty = true;
    error = 0.0f;
    return;
  }
  is_empty = false;

  /* Second-order finite differences (Laplacian components) */
  float d2x = dx_p + dx_m - 2.0f * d_c;
  float d2y = dy_p + dy_m - 2.0f * d_c;
  float d2z = dz_p + dz_m - 2.0f * d_c;

  /* Dimensionless curvature error */
  error = (abs(d2x) + abs(d2y) + abs(d2z)) / h;

  /* Phase 3: Gradient-based edge detection for ambiguous cases */
  if (error > scaled_threshold * 0.3f && error < scaled_threshold * 3.0f) {
    float gx = (dx_p - dx_m) / (2.0f * h);
    float gy = (dy_p - dy_m) / (2.0f * h);
    float gz = (dz_p - dz_m) / (2.0f * h);
    float gradMag = sqrt(gx * gx + gy * gy + gz * gz);
    float gradDeviation = abs(gradMag - 1.0f);
    error = max(error, error + gradDeviation * scaled_threshold * 2.0f);
  }
}

void main() {
  int chunk_idx = int(gl_WorkGroupID.y * uint(dispatch_width) + gl_WorkGroupID.x);
  if (chunk_idx >= active_chunk_count) {
    return;
  }

  int3 coarse_brick = active_coarse_chunks[chunk_idx].xyz;
  int3 local_id = int3(gl_LocalInvocationID);
  uint tid = gl_LocalInvocationIndex;

  int3 fine_brick = coarse_brick * 4 + local_id;
  bool is_valid_fine = true;

  float lod0_half_size = float(LOD0_SCALE * BRICK_SIZE) * 0.5f * voxel_size;
  float3 lod0_center = atlas_origin + (float3(coarse_brick * 4) * float(BRICK_SIZE) + float(LOD0_SCALE * BRICK_SIZE) * 0.5f) * voxel_size;

  /* 1. Collect candidates for the LOD0 block (Thread 0) */
  if (tid == 0u) {
    shared_num_candidates = 0;
    float expand = lod0_half_size * 1.73205f + max_blend + (brick_half_diag * 4.0f);
    float3 brick_min = lod0_center - float3(expand);
    float3 brick_max = lod0_center + float3(expand);

    if (bvh_node_count > 0) {
      int stack[BVH_MAX_STACK];
      int sp = 0;
      stack[sp++] = 0;
      while (sp > 0) {
        int node_idx = stack[--sp];
        BVHNodeGPU node = bvh_nodes[node_idx];
        if (!aabb_overlap(brick_min, brick_max, node.min_and_left.xyz, node.max_and_right.xyz)) {
          continue;
        }
        int left = bvh_decode_int(node.min_and_left.w);
        int right = bvh_decode_int(node.max_and_right.w);
        if (left == -1) {
          if (shared_num_candidates < MAX_CANDIDATES) {
            shared_candidates[shared_num_candidates++] = right;
          }
        } else {
          if (sp < BVH_MAX_STACK - 1) { stack[sp++] = left; stack[sp++] = right; }
        }
      }
    } else {
      for (int i = 0; i < object_count; i++) {
        SDFObjectGPU obj = objects[i];
        if (any(greaterThan(brick_min, obj.bbox_max.xyz)) || any(lessThan(brick_max, obj.bbox_min.xyz))) {
          continue;
        }
        if (shared_num_candidates < MAX_CANDIDATES) {
          shared_candidates[shared_num_candidates++] = i;
        }
      }
    }
  }
  barrier();

  int num_cands = shared_num_candidates;
  if (num_cands == 0) {
    if (is_valid_fine) {
      hashInsert(fine_brick, -1);
    }
    return;
  }

  /* 2. Evaluate LOD0 (Thread 0) */
  float bound0 = lod0_half_size * 1.73205f + brick_half_diag;
  if (tid == 0u) {
    float d_c, error;
    bool is_empty;
    float threshold0 = 0.025f; // Constant dimensionless threshold
    evaluateBlock(lod0_center, lod0_half_size, num_cands, threshold0, bound0, d_c, error, is_empty);

    if (is_empty) {
      if (d_c > 0.0f) {
        lod0_status_status = 0;
      }
      else {
        lod0_status_status = 0;
      }
      lod0_status_slot = (d_c > 0.0f) ? -1 : -2;
    } else {
      if (error < threshold0) {
        lod0_status_status = 2; // flat: allocate
        lod0_status_slot = int(atomicAdd(brick_counter.count, 1u));
        active_bricks[lod0_status_slot].coord = int4(fine_brick, (lod0_status_slot << 2) | 0);
      } else {
        lod0_status_status = 1; // subdivide
      }
    }
  }
  barrier();

  if (lod0_status_status == 0) {
    if (is_valid_fine) {
      hashInsert(fine_brick, lod0_status_slot);
    }
    return;
  }
  if (lod0_status_status == 2) {
    if (is_valid_fine) {
      hashInsert(fine_brick, (lod0_status_slot << 2) | 0);
    }
    return;
  }

  /* 3. Evaluate LOD1 (Threads 0..7) */
  float lod1_half_size = lod0_half_size * 0.5f;
  float bound1 = lod1_half_size * 1.73205f + brick_half_diag;
  if (tid < 8u) {
    int3 l1_offset = int3(tid % 2, (tid / 2) % 2, tid / 4);
    float3 lod1_center = lod0_center + (float3(l1_offset) * 2.0f - 1.0f) * lod1_half_size;

    float d_c, error;
    bool is_empty;
    float threshold1 = 0.025f; // Constant dimensionless threshold
    evaluateBlock(lod1_center, lod1_half_size, num_cands, threshold1, bound1, d_c, error, is_empty);

    if (is_empty) {
      lod1_status_status[tid] = 0;
      lod1_status_slot[tid] = (d_c > 0.0f) ? -1 : -2;
    } else {
      if (error < threshold1) {
        lod1_status_status[tid] = 2;
        int slot = int(atomicAdd(brick_counter.count, 1u));
        lod1_status_slot[tid] = slot;
        int3 l1_fine_brick = coarse_brick * 4 + l1_offset * 2;
        active_bricks[slot].coord = int4(l1_fine_brick, (slot << 2) | 1);
      } else {
        lod1_status_status[tid] = 1;
      }
    }
  }
  barrier();

  int l1_idx = local_id.x / 2 + (local_id.y / 2) * 2 + (local_id.z / 2) * 4;
  if (lod1_status_status[l1_idx] == 0) {
    if (is_valid_fine) {
      hashInsert(fine_brick, lod1_status_slot[l1_idx]);
    }
    return;
  }
  if (lod1_status_status[l1_idx] == 2) {
    if (is_valid_fine) {
      hashInsert(fine_brick, (lod1_status_slot[l1_idx] << 2) | 1);
    }
    return;
  }

  /* 4. Evaluate LOD2 (All 64 threads) */
  if (is_valid_fine) {
    float lod2_half_size = lod1_half_size * 0.5f;
    float3 lod2_center = atlas_origin + (float3(fine_brick) * float(BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) * voxel_size;
    float d_c = evalSDF(lod2_center, num_cands);

    if (abs(d_c) < brick_half_diag) {
      int slot = int(atomicAdd(brick_counter.count, 1u));
      hashInsert(fine_brick, (slot << 2) | 2);
      active_bricks[slot].coord = int4(fine_brick, (slot << 2) | 2);
    } else if (d_c < 0.0f) {
      hashInsert(fine_brick, -2);
    } else {
      hashInsert(fine_brick, -1);
    }
  }
}

