/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* SDF bake: one workgroup per brick.
 * Coarse-to-fine evaluation: 4^3 coarse samples first, refine only near surface.
 * Warp-aligned workgroup (8x8=64 threads), reduced shared memory for occupancy. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_bake)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12
#define WG_SIZE 64
#define TOTAL_VOXELS (BRICK_STORAGE * BRICK_STORAGE * BRICK_STORAGE)

/* BVH candidate index buffer. If exceeded, falls back to linear scan. */
#define CANDIDATE_BUF_SIZE 512

/* Coarse-to-fine constants. */
#define COARSE_RES 4
#define COARSE_SPACING 3
#define COARSE_OFFSET 1
#define COARSE_TOTAL (COARSE_RES * COARSE_RES * COARSE_RES)

/* Finalize a completed group into the scene accumulator. */
void finalizeGroup(int group_id,
                   float grp_dist,
                   float3 grp_color,
                   inout float acc_dist,
                   inout float3 acc_color)
{
  if (group_id < 0 || grp_dist >= 1e10f) {
    return;
  }
  SDFGroupGPU grp = groups[group_id];
  float3 tinted = grp_color * grp.color.rgb;
  float new_dist = combineCSG(
      acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode);
  float h = csgColorWeight(
      acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode);
  acc_color = mix(acc_color, tinted, h);
  acc_dist = new_dist;
}

/* BVH candidate indices + Lipschitz pruning cache. */
shared int shared_candidates[CANDIDATE_BUF_SIZE];
shared int shared_num_candidates;
shared int shared_any_dirty;
shared int shared_overflow;
shared float shared_center_dist[CANDIDATE_BUF_SIZE];

/* Coarse evaluation results. */
shared float shared_coarse_dist[COARSE_TOTAL];
shared float shared_coarse_r[COARSE_TOTAL];
shared float shared_coarse_g[COARSE_TOTAL];
shared float shared_coarse_b[COARSE_TOTAL];

/* Full candidate evaluation at a single world position. */
void evaluateScene(float3 world_pos,
                   int total_count,
                   out float out_dist,
                   out float3 out_color)
{
  float acc_dist = 1e10f;
  float3 acc_color = float3(0.0f);
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.0f);
  int prev_group = -2;

  for (int c = 0; c < total_count; c++) {
    int i = (shared_overflow == 1) ? c : shared_candidates[c];
    SDFObjectGPU sobj = objects[i];

    /* Group transition. */
    if (sobj.group_id != prev_group) {
      finalizeGroup(prev_group, grp_dist, grp_color, acc_dist, acc_color);
      prev_group = sobj.group_id;
      if (prev_group >= 0) {
        grp_dist = 1e10f;
        grp_color = float3(0.0f);
      }
    }

    /* Per-voxel AABB skip (ungrouped unions only). */
    if (sobj.group_id < 0 && sobj.csg_operation == SDF_CSG_OP_UNION) {
      float expand = max(sobj.blend, 0.0f) + max(sobj.shell_distance, 0.0f);
      if (any(greaterThan(world_pos, sobj.bbox_max.xyz + expand)) ||
          any(lessThan(world_pos, sobj.bbox_min.xyz - expand)))
      {
        continue;
      }
    }

    float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;

    SDFPrimitiveData pd;
    pd.sdf_type = sobj.sdf_type;
    pd.size = sobj.sdf_size.xyz;
    pd.bevel = sobj.bevel;
    pd.box_corners = sobj.box_corners;
    pd.box_edges = sobj.box_edges;
    pd.box_modes = sobj.box_modes;
    pd.modifier_start = sobj.modifier_start;
    pd.modifier_count = sobj.modifier_count;
    pd.inverse_matrix = sobj.inverse_matrix;
    float dist = evalObjectSDF(pd, local_pos);

    if (sobj.group_id >= 0) {
      if (sobj.group_first == 1) {
        grp_dist = dist;
        grp_color = sobj.color.rgb;
      }
      else {
        float new_dist = combineCSG(
            grp_dist, dist, sobj.csg_operation, sobj.blend_type, sobj.blend, sobj.shell_distance, sobj.shell_mode);
        float h = csgColorWeight(
            grp_dist, dist, sobj.csg_operation, sobj.blend_type, sobj.blend, sobj.shell_distance, sobj.shell_mode);
        grp_color = mix(grp_color, sobj.color.rgb, h);
        grp_dist = new_dist;
      }
    }
    else {
      float new_dist = combineCSG(
          acc_dist, dist, sobj.csg_operation, sobj.blend_type, sobj.blend, sobj.shell_distance, sobj.shell_mode);
      float h = csgColorWeight(
          acc_dist, dist, sobj.csg_operation, sobj.blend_type, sobj.blend, sobj.shell_distance, sobj.shell_mode);
      acc_color = mix(acc_color, sobj.color.rgb, h);
      acc_dist = new_dist;
    }
  }

  finalizeGroup(prev_group, grp_dist, grp_color, acc_dist, acc_color);
  out_dist = acc_dist;
  out_color = acc_color;
}

/* Trilinear interpolation from the 4^3 coarse grid. */
void interpCoarse(int3 local_voxel, out float out_dist, out float3 out_color)
{
  float3 coarse_coord = (float3(local_voxel) - float(COARSE_OFFSET)) / float(COARSE_SPACING);
  coarse_coord = clamp(coarse_coord, float3(0.0f), float3(float(COARSE_RES - 1)));

  int3 c0 = int3(floor(coarse_coord));
  int3 c1 = min(c0 + 1, int3(COARSE_RES - 1));
  float3 f = coarse_coord - float3(c0);

#define CIDX(x, y, z) ((z) * COARSE_RES * COARSE_RES + (y) * COARSE_RES + (x))
  float d000 = shared_coarse_dist[CIDX(c0.x, c0.y, c0.z)];
  float d100 = shared_coarse_dist[CIDX(c1.x, c0.y, c0.z)];
  float d010 = shared_coarse_dist[CIDX(c0.x, c1.y, c0.z)];
  float d110 = shared_coarse_dist[CIDX(c1.x, c1.y, c0.z)];
  float d001 = shared_coarse_dist[CIDX(c0.x, c0.y, c1.z)];
  float d101 = shared_coarse_dist[CIDX(c1.x, c0.y, c1.z)];
  float d011 = shared_coarse_dist[CIDX(c0.x, c1.y, c1.z)];
  float d111 = shared_coarse_dist[CIDX(c1.x, c1.y, c1.z)];

  out_dist = mix(mix(mix(d000, d100, f.x), mix(d010, d110, f.x), f.y),
                 mix(mix(d001, d101, f.x), mix(d011, d111, f.x), f.y),
                 f.z);

  float r000 = shared_coarse_r[CIDX(c0.x, c0.y, c0.z)];
  float r100 = shared_coarse_r[CIDX(c1.x, c0.y, c0.z)];
  float r010 = shared_coarse_r[CIDX(c0.x, c1.y, c0.z)];
  float r110 = shared_coarse_r[CIDX(c1.x, c1.y, c0.z)];
  float r001 = shared_coarse_r[CIDX(c0.x, c0.y, c1.z)];
  float r101 = shared_coarse_r[CIDX(c1.x, c0.y, c1.z)];
  float r011 = shared_coarse_r[CIDX(c0.x, c1.y, c1.z)];
  float r111 = shared_coarse_r[CIDX(c1.x, c1.y, c1.z)];

  float g000 = shared_coarse_g[CIDX(c0.x, c0.y, c0.z)];
  float g100 = shared_coarse_g[CIDX(c1.x, c0.y, c0.z)];
  float g010 = shared_coarse_g[CIDX(c0.x, c1.y, c0.z)];
  float g110 = shared_coarse_g[CIDX(c1.x, c1.y, c0.z)];
  float g001 = shared_coarse_g[CIDX(c0.x, c0.y, c1.z)];
  float g101 = shared_coarse_g[CIDX(c1.x, c0.y, c1.z)];
  float g011 = shared_coarse_g[CIDX(c0.x, c1.y, c1.z)];
  float g111 = shared_coarse_g[CIDX(c1.x, c1.y, c1.z)];

  float b000 = shared_coarse_b[CIDX(c0.x, c0.y, c0.z)];
  float b100 = shared_coarse_b[CIDX(c1.x, c0.y, c0.z)];
  float b010 = shared_coarse_b[CIDX(c0.x, c1.y, c0.z)];
  float b110 = shared_coarse_b[CIDX(c1.x, c1.y, c0.z)];
  float b001 = shared_coarse_b[CIDX(c0.x, c0.y, c1.z)];
  float b101 = shared_coarse_b[CIDX(c1.x, c0.y, c1.z)];
  float b011 = shared_coarse_b[CIDX(c0.x, c1.y, c1.z)];
  float b111 = shared_coarse_b[CIDX(c1.x, c1.y, c1.z)];

  out_color.r = mix(mix(mix(r000, r100, f.x), mix(r010, r110, f.x), f.y),
                    mix(mix(r001, r101, f.x), mix(r011, r111, f.x), f.y),
                    f.z);
  out_color.g = mix(mix(mix(g000, g100, f.x), mix(g010, g110, f.x), f.y),
                    mix(mix(g001, g101, f.x), mix(g011, g111, f.x), f.y),
                    f.z);
  out_color.b = mix(mix(mix(b000, b100, f.x), mix(b010, b110, f.x), f.y),
                    mix(mix(b001, b101, f.x), mix(b011, b111, f.x), f.y),
                    f.z);
#undef CIDX
}

void main()
{
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= int(brick_counter.count)) {
    return;
  }

  int4 brick_data = active_bricks[brick_idx].coord;
  int3 brick = brick_data.xyz;
  int slot = brick_data.w;

  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  /* Per-brick AABB for BVH culling. */
  float bhd = float(BRICK_SIZE) * voxel_size * 0.866025f;
  float candidate_expand = bhd;
  float3 brick_min = atlas_origin + (float3(brick * BRICK_SIZE) - 2.0f) * voxel_size -
                      float3(candidate_expand);
  float3 brick_max = atlas_origin + (float3(brick * BRICK_SIZE + BRICK_SIZE) + 2.0f) * voxel_size +
                      float3(candidate_expand);

  uint tid = gl_LocalInvocationIndex;

  /* Thread 0: collect candidates via BVH traversal. */
  if (tid == 0u) {
    shared_num_candidates = 0;
    shared_overflow = 0;

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
          if (shared_num_candidates < CANDIDATE_BUF_SIZE) {
            shared_candidates[shared_num_candidates++] = right;
          }
          else {
            shared_overflow = 1;
          }
        }
        else {
          if (sp < BVH_MAX_STACK - 1) {
            stack[sp++] = left;
            stack[sp++] = right;
          }
          else {
            shared_overflow = 1;
          }
        }
      }

      /* Sort candidates by object index for correct CSG evaluation order. */
      if (shared_overflow == 0) {
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
    }
    else {
      for (int i = 0; i < object_count; i++) {
        SDFObjectGPU obj = objects[i];

        if (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
            any(lessThan(brick_max, obj.bbox_min.xyz)))
        {
          continue;
        }

        if (shared_num_candidates < CANDIDATE_BUF_SIZE) {
          shared_candidates[shared_num_candidates++] = i;
        }
        else {
          shared_overflow = 1;
        }
      }
    }

    /* Per-brick dirty check. */
    shared_any_dirty = 0;
    if (shared_overflow == 1) {
      shared_any_dirty = 1;
    }
    else if (has_dirty_flags == 1) {
      for (int c = 0; c < shared_num_candidates; c++) {
        if (dirty_flags[shared_candidates[c]] != 0) {
          shared_any_dirty = 1;
          break;
        }
      }
    }
    else {
      shared_any_dirty = 1;
    }
  }
  barrier();

  if (shared_any_dirty == 0) {
    return;
  }

  /* Lipschitz pruning: cooperatively evaluate candidates at brick center,
   * then thread 0 prunes those provably outside blend influence. */
  if (shared_overflow == 0 && shared_num_candidates > 1) {
    float3 brick_center = atlas_origin +
                          (float3(brick * BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) * voxel_size;
    float two_R = 2.0f * float(BRICK_STORAGE) * voxel_size * 0.866025f;

    for (int c = int(tid); c < shared_num_candidates; c += WG_SIZE) {
      int i = shared_candidates[c];
      SDFObjectGPU obj = objects[i];
      float3 lp = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;
      SDFPrimitiveData pd;
      pd.sdf_type = obj.sdf_type;
      pd.size = obj.sdf_size.xyz;
      pd.bevel = obj.bevel;
      pd.box_corners = obj.box_corners;
      pd.box_edges = obj.box_edges;
      pd.box_modes = obj.box_modes;
      pd.modifier_start = obj.modifier_start;
      pd.modifier_count = obj.modifier_count;
      pd.inverse_matrix = obj.inverse_matrix;
      shared_center_dist[c] = evalObjectSDF(pd, lp);
    }
    barrier();

    if (tid == 0u) {
      float acc_center = 1e10f;
      for (int g = 0; g < group_count; g++) {
        float grp_center = 1e10f;
        for (int c = 0; c < shared_num_candidates; c++) {
          SDFObjectGPU obj = objects[shared_candidates[c]];
          if (obj.group_id != g) continue;
          float d = shared_center_dist[c];
          if (obj.group_first == 1) {
            grp_center = d;
            continue;
          }
          float threshold = (obj.blend_type != SDF_BLEND_TYPE_LINEAR ? obj.blend : 0.0f) + two_R;
          bool pruned = false;
          if (obj.csg_operation == SDF_CSG_OP_UNION) pruned = (d - grp_center > threshold);
          else if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) pruned = (grp_center + d > threshold);
          else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) pruned = (grp_center - d > threshold);

          if (pruned) shared_center_dist[c] = -1e20f;
          else grp_center = combineCSG(grp_center, d, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode);
        }
        if (grp_center < 1e10f) {
          SDFGroupGPU grp = groups[g];
          if (g == 0) acc_center = grp_center;
          else acc_center = combineCSG(acc_center, grp_center, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode);
        }
      }
      for (int c = 0; c < shared_num_candidates; c++) {
        SDFObjectGPU obj = objects[shared_candidates[c]];
        if (obj.group_id != -1) continue;
        float d = shared_center_dist[c];
        float threshold = obj.blend + two_R;
        bool pruned = false;
        if (acc_center < 1e9f) {
          if (obj.csg_operation == SDF_CSG_OP_UNION) pruned = (d - acc_center > threshold);
          else if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) pruned = (acc_center + d > threshold);
          else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) pruned = (acc_center - d > threshold);
        }
        if (pruned) shared_center_dist[c] = -1e20f;
        else acc_center = combineCSG(acc_center, d, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode);
      }

      int new_count = 0;
      for (int c = 0; c < shared_num_candidates; c++) {
        if (shared_center_dist[c] > -1e19f) {
          shared_candidates[new_count++] = shared_candidates[c];
        }
      }
      shared_num_candidates = new_count;
    }
    barrier();
  }

  int total_count = (shared_overflow == 1) ? object_count : shared_num_candidates;

  /* Refinement threshold: voxels within this distance of surface get full eval. */
  float refine_threshold = max(4.0f, max_blend / voxel_size + 2.0f) * voxel_size;

  /* Phase 1: Coarse evaluation at 4^3 sample points (64 threads, 1 each). */
  if (int(tid) < COARSE_TOTAL) {
    int cx = int(tid) % COARSE_RES;
    int cy = (int(tid) / COARSE_RES) % COARSE_RES;
    int cz = int(tid) / (COARSE_RES * COARSE_RES);
    int3 coarse_voxel = int3(COARSE_OFFSET + cx * COARSE_SPACING,
                              COARSE_OFFSET + cy * COARSE_SPACING,
                              COARSE_OFFSET + cz * COARSE_SPACING);
    float3 world_pos = atlas_origin +
                       float3(brick * BRICK_SIZE + coarse_voxel - int3(2)) * voxel_size;

    float dist;
    float3 color;
    evaluateScene(world_pos, total_count, dist, color);

    shared_coarse_dist[tid] = dist;
    shared_coarse_r[tid] = color.r;
    shared_coarse_g[tid] = color.g;
    shared_coarse_b[tid] = color.b;
  }
  barrier();

  /* Check if any coarse sample is near surface. If none are,
   * the entire brick can use interpolated values (fast path). */
  bool any_near_surface = false;
  for (int ci = int(tid); ci < COARSE_TOTAL; ci += WG_SIZE) {
    if (abs(shared_coarse_dist[ci]) < refine_threshold) {
      any_near_surface = true;
    }
  }

  /* Phase 2: Fine evaluation with coarse-to-fine refinement. */
  for (int v = int(tid); v < TOTAL_VOXELS; v += WG_SIZE) {
    int lx = v % BRICK_STORAGE;
    int ly = (v / BRICK_STORAGE) % BRICK_STORAGE;
    int lz = v / (BRICK_STORAGE * BRICK_STORAGE);
    int3 local_voxel = int3(lx, ly, lz);

    float interp_dist;
    float3 interp_color;
    interpCoarse(local_voxel, interp_dist, interp_color);

    float final_dist;
    float3 final_color;

    if (any_near_surface && abs(interp_dist) < refine_threshold) {
      float3 world_pos = atlas_origin +
                         float3(brick * BRICK_SIZE + local_voxel - int3(2)) * voxel_size;
      evaluateScene(world_pos, total_count, final_dist, final_color);
    }
    else {
      final_dist = interp_dist;
      final_color = interp_color;
    }

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(final_dist, final_color));
  }
}
