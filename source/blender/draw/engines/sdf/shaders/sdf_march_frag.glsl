/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* SDF ray-march fragment shader.
 * Fixed world-space voxels with sparse chunk-hashed brick storage. */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12
#define CHUNK_BRICK_RES 16
#define CHUNK_BRICK_COUNT (CHUNK_BRICK_RES * CHUNK_BRICK_RES * CHUNK_BRICK_RES)

#define MAX_BRICK_STEPS 96
#define MAX_VOXEL_STEPS 32

/* Atlas lookup */

int3 gridToCompact(int3 brick, int3 local_voxel, int slot, int bpa)
{
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;
  return slot_origin + local_voxel + int3(2);
}

void fetchCornersCompact(int3 brick, int3 local_cell, int slot, int bpa, out float s[8])
{
  int3 base = gridToCompact(brick, local_cell, slot, bpa);
  s[0] = texelFetch(compact_atlas, base + int3(0, 0, 0), 0).r;
  s[1] = texelFetch(compact_atlas, base + int3(1, 0, 0), 0).r;
  s[2] = texelFetch(compact_atlas, base + int3(0, 1, 0), 0).r;
  s[3] = texelFetch(compact_atlas, base + int3(1, 1, 0), 0).r;
  s[4] = texelFetch(compact_atlas, base + int3(0, 0, 1), 0).r;
  s[5] = texelFetch(compact_atlas, base + int3(1, 0, 1), 0).r;
  s[6] = texelFetch(compact_atlas, base + int3(0, 1, 1), 0).r;
  s[7] = texelFetch(compact_atlas, base + int3(1, 1, 1), 0).r;
}

/* Normal computation */

float3 computeDualVoxelNormal(float3 grid_pos_in_brick, int3 brick, int slot, int bpa)
{
  float3 shifted = grid_pos_in_brick + float3(0.5f);
  int3 dc = int3(floor(shifted));
  float3 uvw = shifted - float3(dc);
  int3 cb = gridToCompact(brick, dc - int3(1), slot, bpa);

  float vals[27];
  for (int k = 0; k < 3; k++) {
    for (int j = 0; j < 3; j++) {
      vals[k * 9 + j * 3 + 0] = texelFetch(compact_atlas, cb + int3(0, j, k), 0).r;
      vals[k * 9 + j * 3 + 1] = texelFetch(compact_atlas, cb + int3(1, j, k), 0).r;
      vals[k * 9 + j * 3 + 2] = texelFetch(compact_atlas, cb + int3(2, j, k), 0).r;
    }
  }

  float3 blended = float3(0.0f);

  for (int kk = 0; kk < 2; kk++) {
    for (int jj = 0; jj < 2; jj++) {
      for (int ii = 0; ii < 2; ii++) {
        float s[8];
        s[0] = vals[(kk) * 9 + (jj) * 3 + (ii)];
        s[1] = vals[(kk) * 9 + (jj) * 3 + (ii + 1)];
        s[2] = vals[(kk) * 9 + (jj + 1) * 3 + (ii)];
        s[3] = vals[(kk) * 9 + (jj + 1) * 3 + (ii + 1)];
        s[4] = vals[(kk + 1) * 9 + (jj) * 3 + (ii)];
        s[5] = vals[(kk + 1) * 9 + (jj) * 3 + (ii + 1)];
        s[6] = vals[(kk + 1) * 9 + (jj + 1) * 3 + (ii)];
        s[7] = vals[(kk + 1) * 9 + (jj + 1) * 3 + (ii + 1)];

        float3 local_p = grid_pos_in_brick - float3(dc - int3(1) + int3(ii, jj, kk));
        float3 grad = trilinearGradient(s, local_p);
        float l = length(grad);
        float3 n = l > 1e-8f ? grad / l : float3(0.0f, 0.0f, 1.0f);

        float wu = ii == 0 ? (1.0f - uvw.x) : uvw.x;
        float wv = jj == 0 ? (1.0f - uvw.y) : uvw.y;
        float ww = kk == 0 ? (1.0f - uvw.z) : uvw.z;

        blended += n * (wu * wv * ww);
      }
    }
  }

  float l = length(blended);
  return l > 1e-8f ? blended / l : float3(0.0f, 0.0f, 1.0f);
}

/* Chunk lookup */

uint hashChunkCoord(int3 coord)
{
  uvec3 u = uvec3(coord);
  return u.x * 73856093u ^ u.y * 19349663u ^ u.z * 83492791u;
}

int findChunkIndex(int3 coord)
{
  if (chunk_hash_mask < 0) {
    return -1;
  }

  uint slot = hashChunkCoord(coord) & uint(chunk_hash_mask);
  int table_size = chunk_hash_mask + 1;

  for (int probe = 0; probe < table_size; probe++) {
    ChunkHashEntryGPU entry = chunk_hash[slot];
    if (entry.coord.w < 0) {
      return -1;
    }
    if (all(equal(entry.coord.xyz, coord))) {
      return entry.coord.w;
    }
    slot = (slot + 1u) & uint(chunk_hash_mask);
  }

  return -1;
}

int readChunkBrickSlot(int chunk_idx, int3 local_brick)
{
  int flat_index = local_brick.x + local_brick.y * CHUNK_BRICK_RES +
                   local_brick.z * CHUNK_BRICK_RES * CHUNK_BRICK_RES;
  return chunk_bricks[chunk_idx * CHUNK_BRICK_COUNT + flat_index];
}

/* Chunk traversal */

void march_chunk_local(float3 ray_origin,
                       float3 ray_dir,
                       float t_enter,
                       float t_exit,
                       int chunk_idx,
                       int3 chunk_coord,
                       out float out_hit_t,
                       out int3 out_hit_brick,
                       out int3 out_hit_cell,
                       out int out_hit_slot)
{
  out_hit_t = -1.0f;
  out_hit_slot = -1;

  float brick_world_size = float(BRICK_SIZE) * voxel_size;
  float inv_voxel = 1.0f / voxel_size;
  float3 chunk_origin = float3(chunk_coord * (CHUNK_BRICK_RES * BRICK_SIZE)) * voxel_size;
  float inv_brick = 1.0f / brick_world_size;
  float3 B = (ray_origin - chunk_origin) * inv_brick;
  float3 BD = ray_dir * inv_brick;
  float3 B_enter = B + t_enter * BD;

  int3 brick_cell = int3(floor(B_enter));
  brick_cell = clamp(brick_cell, int3(0), int3(CHUNK_BRICK_RES - 1));

  int3 brick_step;
  brick_step.x = BD.x > 0.0f ? 1 : (BD.x < 0.0f ? -1 : 0);
  brick_step.y = BD.y > 0.0f ? 1 : (BD.y < 0.0f ? -1 : 0);
  brick_step.z = BD.z > 0.0f ? 1 : (BD.z < 0.0f ? -1 : 0);

  float3 brick_tDelta = float3(BD.x != 0.0f ? abs(1.0f / BD.x) : 1e30f,
                                BD.y != 0.0f ? abs(1.0f / BD.y) : 1e30f,
                                BD.z != 0.0f ? abs(1.0f / BD.z) : 1e30f);

  float3 brick_boundary;
  brick_boundary.x = BD.x > 0.0f ? float(brick_cell.x + 1) : float(brick_cell.x);
  brick_boundary.y = BD.y > 0.0f ? float(brick_cell.y + 1) : float(brick_cell.y);
  brick_boundary.z = BD.z > 0.0f ? float(brick_cell.z + 1) : float(brick_cell.z);

  float3 brick_tMax;
  brick_tMax.x = BD.x != 0.0f ? (brick_boundary.x - B.x) / BD.x : 1e30f;
  brick_tMax.y = BD.y != 0.0f ? (brick_boundary.y - B.y) / BD.y : 1e30f;
  brick_tMax.z = BD.z != 0.0f ? (brick_boundary.z - B.z) / BD.z : 1e30f;

  float t_brick_current = t_enter;

  for (int bstep = 0; bstep < MAX_BRICK_STEPS; bstep++) {
    if (any(lessThan(brick_cell, int3(0))) ||
        any(greaterThan(brick_cell, int3(CHUNK_BRICK_RES - 1))))
    {
      break;
    }

    int slot = readChunkBrickSlot(chunk_idx, brick_cell);
    int3 world_brick = chunk_coord * CHUNK_BRICK_RES + brick_cell;

    if (slot == -2) {
      out_hit_t = t_brick_current;
      out_hit_brick = world_brick;
      out_hit_cell = int3(0);
      out_hit_slot = -2;
      return;
    }

    float t_brick_exit = min(min(brick_tMax.x, brick_tMax.y), brick_tMax.z);
    t_brick_exit = min(t_brick_exit, t_exit);

    if (slot >= 0) {
      float3 brick_origin = float3(world_brick * BRICK_SIZE) * voxel_size;
      float3 V = (ray_origin - brick_origin) * inv_voxel;
      float3 VD = ray_dir * inv_voxel;
      float3 V_enter = V + t_brick_current * VD;

      int3 vcell = int3(floor(V_enter));
      vcell = clamp(vcell, int3(0), int3(BRICK_SIZE - 1));

      int3 vstep;
      vstep.x = VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0);
      vstep.y = VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0);
      vstep.z = VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0);

      float3 vtDelta = float3(VD.x != 0.0f ? abs(1.0f / VD.x) : 1e30f,
                               VD.y != 0.0f ? abs(1.0f / VD.y) : 1e30f,
                               VD.z != 0.0f ? abs(1.0f / VD.z) : 1e30f);

      float3 vbound;
      vbound.x = VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x);
      vbound.y = VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y);
      vbound.z = VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z);

      float3 vtMax;
      vtMax.x = VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f;
      vtMax.y = VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f;
      vtMax.z = VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f;

      float vt_current = t_brick_current;
      bool voxel_hit = false;

      for (int vstep_i = 0; vstep_i < MAX_VOXEL_STEPS; vstep_i++) {
        if (any(lessThan(vcell, int3(0))) || any(greaterThan(vcell, int3(BRICK_SIZE - 1)))) {
          break;
        }

        float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
        vt_cell_exit = min(vt_cell_exit, t_brick_exit);

        float s[8];
        fetchCornersCompact(world_brick, vcell, slot, bricks_per_axis, s);

        float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                         min(min(s[4], s[5]), min(s[6], s[7])));
        float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                         max(max(s[4], s[5]), max(s[6], s[7])));

        if (smin <= 0.0f) {
          if (smax < 0.0f) {
            out_hit_t = vt_current;
            out_hit_brick = world_brick;
            out_hit_cell = vcell;
            out_hit_slot = slot;
            voxel_hit = true;
            break;
          }

          float T_max = vt_cell_exit - vt_current;
          if (T_max > 1e-8f) {
            float k[8];
            computeTrilinearCoeffs(s, k);

            float3 o_local = V + vt_current * VD - float3(vcell);
            o_local = clamp(o_local, float3(0.0f), float3(1.0f));
            float3 d_scaled = VD * T_max;

            float c[4];
            computeCubicCoeffs(k, o_local, d_scaled, c);

            if (c[0] <= 0.0f) {
              out_hit_t = vt_current;
              out_hit_brick = world_brick;
              out_hit_cell = vcell;
              out_hit_slot = slot;
              voxel_hit = true;
              break;
            }

            float u_hit = solveCubicMarmittNR(c, 1.0f);
            if (u_hit >= 0.0f) {
              out_hit_t = vt_current + u_hit * T_max;
              out_hit_brick = world_brick;
              out_hit_cell = vcell;
              out_hit_slot = slot;
              voxel_hit = true;
              break;
            }
          }
        }

        if (vtMax.x < vtMax.y) {
          if (vtMax.x < vtMax.z) {
            vt_current = vtMax.x;
            vcell.x += vstep.x;
            vtMax.x += vtDelta.x;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }
        else {
          if (vtMax.y < vtMax.z) {
            vt_current = vtMax.y;
            vcell.y += vstep.y;
            vtMax.y += vtDelta.y;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }

        if (vt_current >= t_brick_exit) {
          break;
        }
      }

      if (voxel_hit) {
        return;
      }
    }

    if (brick_tMax.x < brick_tMax.y) {
      if (brick_tMax.x < brick_tMax.z) {
        t_brick_current = brick_tMax.x;
        brick_cell.x += brick_step.x;
        brick_tMax.x += brick_tDelta.x;
      }
      else {
        t_brick_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }
    else {
      if (brick_tMax.y < brick_tMax.z) {
        t_brick_current = brick_tMax.y;
        brick_cell.y += brick_step.y;
        brick_tMax.y += brick_tDelta.y;
      }
      else {
        t_brick_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }

    if (t_brick_current >= t_exit) {
      break;
    }
  }
}

void march_chunked_hash(float3 ray_origin,
                        float3 ray_dir,
                        float t_enter,
                        float t_exit,
                        out float out_hit_t,
                        out int3 out_hit_brick,
                        out int3 out_hit_cell,
                        out int out_hit_slot)
{
  out_hit_t = -1.0f;
  out_hit_slot = -1;

  float chunk_world_size = float(CHUNK_BRICK_RES * BRICK_SIZE) * voxel_size;
  int3 grid_res = chunk_grid_resolution;
  float inv_chunk = 1.0f / chunk_world_size;

  float3 C = (ray_origin - atlas_origin) * inv_chunk;
  float3 CD = ray_dir * inv_chunk;
  float3 C_enter = C + t_enter * CD;

  int3 chunk_cell = int3(floor(C_enter));
  chunk_cell = clamp(chunk_cell, int3(0), grid_res - int3(1));

  int3 chunk_step;
  chunk_step.x = CD.x > 0.0f ? 1 : (CD.x < 0.0f ? -1 : 0);
  chunk_step.y = CD.y > 0.0f ? 1 : (CD.y < 0.0f ? -1 : 0);
  chunk_step.z = CD.z > 0.0f ? 1 : (CD.z < 0.0f ? -1 : 0);

  float3 chunk_tDelta = float3(CD.x != 0.0f ? abs(1.0f / CD.x) : 1e30f,
                                CD.y != 0.0f ? abs(1.0f / CD.y) : 1e30f,
                                CD.z != 0.0f ? abs(1.0f / CD.z) : 1e30f);

  float3 chunk_boundary;
  chunk_boundary.x = CD.x > 0.0f ? float(chunk_cell.x + 1) : float(chunk_cell.x);
  chunk_boundary.y = CD.y > 0.0f ? float(chunk_cell.y + 1) : float(chunk_cell.y);
  chunk_boundary.z = CD.z > 0.0f ? float(chunk_cell.z + 1) : float(chunk_cell.z);

  float3 chunk_tMax;
  chunk_tMax.x = CD.x != 0.0f ? (chunk_boundary.x - C.x) / CD.x : 1e30f;
  chunk_tMax.y = CD.y != 0.0f ? (chunk_boundary.y - C.y) / CD.y : 1e30f;
  chunk_tMax.z = CD.z != 0.0f ? (chunk_boundary.z - C.z) / CD.z : 1e30f;

  float t_current = t_enter;

  for (int cstep = 0; cstep < 512; cstep++) {
    if (any(lessThan(chunk_cell, int3(0))) || any(greaterThanEqual(chunk_cell, grid_res))) {
      break;
    }

    float t_chunk_exit = min(min(chunk_tMax.x, chunk_tMax.y), chunk_tMax.z);
    t_chunk_exit = min(t_chunk_exit, t_exit);

    int3 chunk_coord = atlas_chunk_min + chunk_cell;
    int chunk_idx = findChunkIndex(chunk_coord);
    if (chunk_idx >= 0) {
      float hit_t;
      int3 hit_brick;
      int3 hit_cell;
      int hit_slot;
      march_chunk_local(ray_origin,
                        ray_dir,
                        t_current,
                        t_chunk_exit,
                        chunk_idx,
                        chunk_coord,
                        hit_t,
                        hit_brick,
                        hit_cell,
                        hit_slot);

      if (hit_t >= 0.0f) {
        out_hit_t = hit_t;
        out_hit_brick = hit_brick;
        out_hit_cell = hit_cell;
        out_hit_slot = hit_slot;
        return;
      }
    }

    if (chunk_tMax.x < chunk_tMax.y) {
      if (chunk_tMax.x < chunk_tMax.z) {
        t_current = chunk_tMax.x;
        chunk_cell.x += chunk_step.x;
        chunk_tMax.x += chunk_tDelta.x;
      }
      else {
        t_current = chunk_tMax.z;
        chunk_cell.z += chunk_step.z;
        chunk_tMax.z += chunk_tDelta.z;
      }
    }
    else {
      if (chunk_tMax.y < chunk_tMax.z) {
        t_current = chunk_tMax.y;
        chunk_cell.y += chunk_step.y;
        chunk_tMax.y += chunk_tDelta.y;
      }
      else {
        t_current = chunk_tMax.z;
        chunk_cell.z += chunk_step.z;
        chunk_tMax.z += chunk_tDelta.z;
      }
    }

    if (t_current > t_exit) {
      break;
    }
  }
}

void march_chunk_bvh(float3 ray_origin,
                     float3 ray_dir,
                     float t_enter,
                     float t_exit,
                     out float out_hit_t,
                     out int3 out_hit_brick,
                     out int3 out_hit_cell,
                     out int out_hit_slot)
{
  out_hit_t = -1.0f;
  out_hit_slot = -1;

  if (chunk_bvh_node_count <= 0) {
    return;
  }

  float3 inv_dir = 1.0f / ray_dir;

  int node_stack[BVH_MAX_STACK];
  float node_tn_stack[BVH_MAX_STACK];
  float node_tf_stack[BVH_MAX_STACK];
  int sp = 0;

  BVHNodeGPU root = chunk_bvh_nodes[0];
  float root_tn, root_tf;
  if (!ray_aabb_intersect(
          ray_origin, inv_dir, root.min_and_left.xyz, root.max_and_right.xyz, root_tn, root_tf))
  {
    return;
  }

  root_tn = max(root_tn, t_enter);
  root_tf = min(root_tf, t_exit);
  if (root_tn > root_tf) {
    return;
  }

  node_stack[sp] = 0;
  node_tn_stack[sp] = root_tn;
  node_tf_stack[sp] = root_tf;
  sp++;

  for (; sp > 0;) {
    sp--;
    int node_idx = node_stack[sp];
    float node_tn = node_tn_stack[sp];
    float node_tf = node_tf_stack[sp];

    if (node_tn > node_tf) {
      continue;
    }
    if (out_hit_t >= 0.0f && node_tn > out_hit_t) {
      continue;
    }

    BVHNodeGPU node = chunk_bvh_nodes[node_idx];
    int left = bvh_decode_int(node.min_and_left.w);
    int right = bvh_decode_int(node.max_and_right.w);

    if (left == -1) {
      int chunk_idx = right;
      ChunkPageGPU chunk = chunk_pages[chunk_idx];

      float hit_t;
      int3 hit_brick;
      int3 hit_cell;
      int hit_slot;
      march_chunk_local(ray_origin,
                        ray_dir,
                        node_tn,
                        node_tf,
                        chunk_idx,
                        chunk.coord.xyz,
                        hit_t,
                        hit_brick,
                        hit_cell,
                        hit_slot);

      if (hit_t >= 0.0f && (out_hit_t < 0.0f || hit_t < out_hit_t)) {
        out_hit_t = hit_t;
        out_hit_brick = hit_brick;
        out_hit_cell = hit_cell;
        out_hit_slot = hit_slot;
      }
      continue;
    }

    float left_tn, left_tf, right_tn, right_tf;
    bool has_left = ray_aabb_intersect(ray_origin,
                                       inv_dir,
                                       chunk_bvh_nodes[left].min_and_left.xyz,
                                       chunk_bvh_nodes[left].max_and_right.xyz,
                                       left_tn,
                                       left_tf);
    bool has_right = ray_aabb_intersect(ray_origin,
                                        inv_dir,
                                        chunk_bvh_nodes[right].min_and_left.xyz,
                                        chunk_bvh_nodes[right].max_and_right.xyz,
                                        right_tn,
                                        right_tf);

    if (has_left) {
      left_tn = max(left_tn, t_enter);
      left_tf = min(left_tf, t_exit);
      has_left = (left_tn <= left_tf);
    }
    if (has_right) {
      right_tn = max(right_tn, t_enter);
      right_tf = min(right_tf, t_exit);
      has_right = (right_tn <= right_tf);
    }

    if (has_left && has_right) {
      bool left_first = left_tn <= right_tn;
      int near_idx = left_first ? left : right;
      int far_idx = left_first ? right : left;
      float near_tn = left_first ? left_tn : right_tn;
      float near_tf = left_first ? left_tf : right_tf;
      float far_tn = left_first ? right_tn : left_tn;
      float far_tf = left_first ? right_tf : left_tf;

      if (sp < BVH_MAX_STACK) {
        node_stack[sp] = far_idx;
        node_tn_stack[sp] = far_tn;
        node_tf_stack[sp] = far_tf;
        sp++;
      }
      if (sp < BVH_MAX_STACK) {
        node_stack[sp] = near_idx;
        node_tn_stack[sp] = near_tn;
        node_tf_stack[sp] = near_tf;
        sp++;
      }
    }
    else if (has_left) {
      if (sp < BVH_MAX_STACK) {
        node_stack[sp] = left;
        node_tn_stack[sp] = left_tn;
        node_tf_stack[sp] = left_tf;
        sp++;
      }
    }
    else if (has_right) {
      if (sp < BVH_MAX_STACK) {
        node_stack[sp] = right;
        node_tn_stack[sp] = right_tn;
        node_tf_stack[sp] = right_tf;
        sp++;
      }
    }
  }
}

void march_world_exact(float3 ray_origin,
                       float3 ray_dir,
                       float t_enter,
                       float t_exit,
                       int chunk_grid_span,
                       out float out_hit_t,
                       out int3 out_hit_brick,
                       out int3 out_hit_cell,
                       out int out_hit_slot)
{
  if (chunk_grid_span < 0) {
    out_hit_t = -1.0f;
    out_hit_slot = -1;
    return;
  }
  if (chunk_grid_span <= 96) {
    march_chunked_hash(ray_origin,
                       ray_dir,
                       t_enter,
                       t_exit,
                       out_hit_t,
                       out_hit_brick,
                       out_hit_cell,
                       out_hit_slot);
  }
  else if (chunk_bvh_node_count > 0) {
    march_chunk_bvh(ray_origin,
                    ray_dir,
                    t_enter,
                    t_exit,
                    out_hit_t,
                    out_hit_brick,
                    out_hit_cell,
                    out_hit_slot);
  }
  else {
    march_chunked_hash(ray_origin,
                       ray_dir,
                       t_enter,
                       t_exit,
                       out_hit_t,
                       out_hit_brick,
                       out_hit_cell,
                       out_hit_slot);
  }
}

/* Main */

void main()
{
  float2 uv = screen_uv;

  ViewMatrices vm = drw_view();
  float4x4 view_inv = vm.viewinv;
  float4x4 win_inv = vm.wininv;

  float4 ndc_near = float4(uv * 2.0f - 1.0f, -1.0f, 1.0f);
  float4 ndc_far = float4(uv * 2.0f - 1.0f, 1.0f, 1.0f);

  float4 world_near = view_inv * (win_inv * ndc_near);
  float4 world_far = view_inv * (win_inv * ndc_far);
  world_near.xyz /= world_near.w;
  world_far.xyz /= world_far.w;

  float3 ray_origin = world_near.xyz;
  float3 ray_dir = normalize(world_far.xyz - world_near.xyz);

  float3 world_min = atlas_origin;
  float3 world_max = atlas_origin + atlas_extent;

  float3 inv_dir = 1.0f / ray_dir;
  float3 t0 = (world_min - ray_origin) * inv_dir;
  float3 t1 = (world_max - ray_origin) * inv_dir;
  float3 t_lo = min(t0, t1);
  float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  float t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

  if (t_enter > t_exit || t_exit < 0.0f) {
    discard;
    return;
  }
  t_enter = max(t_enter, 0.0f);

  float hit_t = -1.0f;
  int3 hit_brick = int3(0);
  int3 hit_local_cell = int3(0);
  int hit_slot = -1;

  float chunk_world_size = float(CHUNK_BRICK_RES * BRICK_SIZE) * voxel_size;
  int3 chunk_grid_res = max(int3(round(atlas_extent / chunk_world_size)), int3(1));
  int chunk_grid_span = chunk_grid_res.x + chunk_grid_res.y + chunk_grid_res.z;
  march_world_exact(
      ray_origin, ray_dir, t_enter, t_exit, chunk_grid_span, hit_t, hit_brick, hit_local_cell, hit_slot);

  if (hit_t < 0.0f) {
    discard;
    return;
  }

  float3 hit_pos = ray_origin + ray_dir * hit_t;
  float3 hit_color = float3(0.5f);
  float3 hit_normal = float3(0.0f, 0.0f, 1.0f);

  if (hit_slot == -2) {
    hit_normal = -ray_dir;
  }
  else {
    float3 brick_origin = float3(hit_brick * BRICK_SIZE) * voxel_size;
    float3 local_pos = (hit_pos - brick_origin) / voxel_size;

    int bpa = bricks_per_axis;
    int3 slot_block = int3(hit_slot % bpa, (hit_slot / bpa) % bpa, hit_slot / (bpa * bpa));
    float3 atlas_pos = float3(slot_block * BRICK_STORAGE) + local_pos + float3(2.0f);
    int3 compact_size = int3(textureSize(compact_atlas, 0));
    float3 atlas_uv = atlas_pos / float3(compact_size);
    hit_color = textureLod(compact_atlas, atlas_uv, 0.0f).gba;
    hit_normal = computeDualVoxelNormal(local_pos, hit_brick, hit_slot, bricks_per_axis);
  }

  float3 normal = hit_normal;
  float3 obj_color = hit_color;
  float3 shaded_color;

  if (lighting_type == 0) {
    shaded_color = obj_color;
  }
  else if (lighting_type == 1) {
    float3 N = normalize((vm.viewmat * float4(normal, 0.0f)).xyz);
    float3 view_pos = (vm.viewmat * float4(hit_pos, 1.0f)).xyz;
    float3 I = drw_view_incident_vector(view_pos);

    float4 dirs[4] = float4[4](studio_light0, studio_light1, studio_light2, studio_light3);
    float4 cols[4] = float4[4](studio_color0, studio_color1, studio_color2, studio_color3);
    float4 specs[4] = float4[4](studio_spec0, studio_spec1, studio_spec2, studio_spec3);

    float3 diffuse_light = studio_ambient;
    float3 specular_light = studio_ambient;
    float roughness = 0.5f;

    for (int i = 0; i < 4; i++) {
      float NL = dot(dirs[i].xyz, N);
      float w = cols[i].w;
      float w1 = w + 1.0f;
      diffuse_light += cols[i].rgb * clamp((NL + w) / (w1 * w1), 0.0f, 1.0f);
    }

    float3 spec_col = float3(0.0f);
    if (use_specular != 0) {
      float3 R = -reflect(I, N);
      for (int i = 0; i < 4; i++) {
        float3 L = dirs[i].xyz;
        float w = cols[i].w;
        float3 H = normalize(L + I);
        float spec_angle = clamp(dot(H, N), 0.0f, 1.0f);
        float cNL = clamp(dot(L, N), 0.0f, 1.0f);

        float gloss = (1.0f - roughness) * (1.0f - w);
        float shininess = exp2(10.0f * gloss + 1.0f);
        float norm_factor = shininess * 0.125f + 1.0f;
        float spec = pow(spec_angle, shininess) * cNL * norm_factor;

        float wrap_NL = dot(L, R);
        float w_s = mix(w, 1.0f, roughness);
        float w_s1 = w_s + 1.0f;
        float spec_env = clamp((wrap_NL + w_s) / (w_s1 * w_s1), 0.0f, 1.0f);

        specular_light += specs[i].rgb * mix(spec, spec_env, w * w);
      }

      spec_col = float3(0.05f);
      float NV = clamp(dot(N, I), 0.0f, 1.0f);
      float fresnel = exp2(-8.35f * NV) * (1.0f - roughness);
      spec_col = mix(spec_col, float3(1.0f), fresnel);
    }

    specular_light *= spec_col;
    float spec_energy = dot(spec_col, float3(0.33333f));
    diffuse_light *= obj_color * (1.0f - spec_energy);
    shaded_color = diffuse_light + specular_light;
  }
  else {
    float3 view_normal = normalize((vm.viewmat * float4(normal, 0.0f)).xyz);
    float3 view_pos = (vm.viewmat * float4(hit_pos, 1.0f)).xyz;
    float3 I = drw_view_incident_vector(view_pos);

    float a = 1.0f / (1.0f + I.z);
    float b = -I.x * I.y * a;
    float3 b1 = float3(1.0f - I.x * I.x * a, b, -I.x);
    float3 b2 = float3(b, 1.0f - I.y * I.y * a, -I.y);
    float2 matcap_uv = float2(dot(b1, view_normal), dot(b2, view_normal));
    if (use_matcap_flip != 0) {
      matcap_uv.x = -matcap_uv.x;
    }
    matcap_uv = matcap_uv * 0.496f + 0.5f;

    float3 diffuse = textureLod(matcap_tx, float3(matcap_uv, 0.0f), 0.0f).rgb;
    float3 specular = textureLod(matcap_tx, float3(matcap_uv, 1.0f), 0.0f).rgb;

    shaded_color = diffuse * obj_color + specular * float(use_specular);
  }

  out_color = float4(shaded_color, 1.0f);
  gl_FragDepth = drw_point_world_to_screen(hit_pos).z;
}
