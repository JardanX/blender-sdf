/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF ray-march fragment shader (sparse brick version).
 *
 * Two modes controlled by `use_instanced`:
 * - World-space mode (use_instanced == 0): Two-level DDA through a single
 *   world-space atlas. Classic approach, supports smooth blending.
 * - Instanced mode (use_instanced != 0): BVH traversal over instance AABBs,
 *   per-shape local DDA. Enables O(unique_shapes) atlas memory for thousands
 *   of instances.
 *
 * Based on "Ray Tracing of Signed Distance Function Grids"
 * (Hansson-Soderlund, Evans, Akenine-Moller 2022).
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

/** Max brick-level DDA steps (must cover grid diagonal). */
#define MAX_BRICK_STEPS 256
/** Max voxel-level DDA steps within one brick. */
#define MAX_VOXEL_STEPS 24
/** Max BVH traversal stack depth for instance traversal. */
#define MAX_INST_STACK 32

/* ---- Atlas lookup helpers ---- */

/**
 * Convert a grid-space voxel coordinate to compact atlas coordinate.
 * \param brick: brick coordinate in the grid.
 * \param local_voxel: voxel offset within the brick [0..7].
 * \param slot: compact atlas slot index for this brick.
 * \param bpa: bricks per axis in the compact atlas.
 * \return texel coordinate in the compact atlas.
 */
int3 gridToCompact(int3 brick, int3 local_voxel, int slot, int bpa)
{
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;
  /* +2 for the 2-voxel overlap padding border. */
  return slot_origin + local_voxel + int3(2);
}

/**
 * Fetch 8 corner SDF values for a voxel cell within a compact atlas brick.
 */
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

/* ---- Normal computation ---- */

/**
 * Compute normal from trilinear gradient at the hit point (single voxel).
 */
float3 computeNormalCompact(float3 grid_pos_in_brick, int3 brick, int slot, int bpa)
{
  int3 cell = int3(floor(grid_pos_in_brick));
  cell = clamp(cell, int3(0), int3(BRICK_SIZE - 1));

  float3 frac_pos = grid_pos_in_brick - float3(cell);
  frac_pos = clamp(frac_pos, float3(0.0f), float3(1.0f));

  float s[8];
  fetchCornersCompact(brick, cell, slot, bpa, s);

  float3 grad = trilinearGradient(s, frac_pos);
  float len = length(grad);
  return len > 1e-8f ? grad / len : float3(0.0f, 0.0f, 1.0f);
}

/**
 * Compute smooth normal via dual voxel interpolation.
 * (Section 3.2, Hansson-Soderlund et al. JCGT 2022)
 *
 * The hit point falls inside a dual voxel (shifted by half a voxel width).
 * We evaluate the analytic trilinear gradient in each of the 2x2x2 overlapping
 * voxels at the hit point, normalize each, then trilinearly blend using the
 * hit point's position within the dual voxel.
 *
 * This yields C0-continuous normals across voxel boundaries.  27 texture
 * fetches (3x3x3 neighborhood), 8 gradient evaluations + 8 normalizations.
 *
 * The 12^3 brick storage provides the required 3x3x3 neighborhood: for any
 * cell [0..7], we need positions (cell-1)..(cell+1), which spans [-1..8] —
 * well within the [-2..9] padding range.
 */
float3 computeDualVoxelNormal(float3 grid_pos_in_brick, int3 brick, int slot, int bpa)
{
  /* Dual voxel center: nearest grid corner to the hit point. */
  float3 shifted = grid_pos_in_brick + float3(0.5f);
  int3 dc = int3(floor(shifted));

  /* Position within the dual voxel [0,1]^3. */
  float3 uvw = shifted - float3(dc);

  /* Atlas base for the 3x3x3 neighborhood.
   * Corner positions range from (dc-1) to (dc+1). */
  int3 cb = gridToCompact(brick, dc - int3(1), slot, bpa);

  /* Fetch 3x3x3 neighborhood values. */
  float vals[27];
  for (int k = 0; k < 3; k++) {
    for (int j = 0; j < 3; j++) {
      vals[k * 9 + j * 3 + 0] = texelFetch(compact_atlas, cb + int3(0, j, k), 0).r;
      vals[k * 9 + j * 3 + 1] = texelFetch(compact_atlas, cb + int3(1, j, k), 0).r;
      vals[k * 9 + j * 3 + 2] = texelFetch(compact_atlas, cb + int3(2, j, k), 0).r;
    }
  }

  /* Evaluate analytic trilinear gradient in each of 8 overlapping voxels,
   * normalize each, then trilinearly blend. */
  float3 blended = float3(0.0f);

  for (int kk = 0; kk < 2; kk++) {
    for (int jj = 0; jj < 2; jj++) {
      for (int ii = 0; ii < 2; ii++) {
        /* Extract 8 corners for voxel (ii,jj,kk) from the 3x3x3 grid. */
        float s[8];
        s[0] = vals[(kk) * 9 + (jj) * 3 + (ii)];
        s[1] = vals[(kk) * 9 + (jj) * 3 + (ii + 1)];
        s[2] = vals[(kk) * 9 + (jj + 1) * 3 + (ii)];
        s[3] = vals[(kk) * 9 + (jj + 1) * 3 + (ii + 1)];
        s[4] = vals[(kk + 1) * 9 + (jj) * 3 + (ii)];
        s[5] = vals[(kk + 1) * 9 + (jj) * 3 + (ii + 1)];
        s[6] = vals[(kk + 1) * 9 + (jj + 1) * 3 + (ii)];
        s[7] = vals[(kk + 1) * 9 + (jj + 1) * 3 + (ii + 1)];

        /* Hit point in this voxel's local space.
         * May be outside [0,1]^3 for 7 of 8 voxels — that's intended. */
        float3 local_p = grid_pos_in_brick - float3(dc - int3(1) + int3(ii, jj, kk));

        float3 grad = trilinearGradient(s, local_p);
        float l = length(grad);
        float3 n = l > 1e-8f ? grad / l : float3(0.0f, 0.0f, 1.0f);

        /* Trilinear blend weight. */
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

/* ---- Inner DDA march (shared by both modes) ---- */

/**
 * Two-level DDA march through a brick atlas.
 *
 * Parameters are mode-dependent:
 * - World-space mode: reads indirection from indirection_tx (3D texture sampler).
 * - Instanced mode: reads indirection from shape_indir[] (SSBO) via indir_offset.
 *
 * \param ray_origin: ray origin in the atlas's coordinate space.
 * \param ray_dir: normalized ray direction.
 * \param t_enter: entry t for the atlas AABB.
 * \param t_exit: exit t for the atlas AABB.
 * \param grid_res: brick grid resolution.
 * \param atlas_orig: atlas origin (world or local space).
 * \param vs: voxel size.
 * \param bpa: bricks_per_axis for compact atlas lookup.
 * \param indir_offset: offset into shape_indir SSBO (-1 = use indirection_tx texture).
 * \param out_hit_t: output hit t (negative = no hit).
 * \param out_hit_brick: output brick coordinate of hit.
 * \param out_hit_cell: output voxel cell of hit.
 * \param out_hit_slot: output compact atlas slot of hit.
 */
void dda_march(float3 ray_origin,
               float3 ray_dir,
               float t_enter,
               float t_exit,
               int3 grid_res,
               float3 atlas_orig,
               float vs,
               int bpa,
               int indir_offset,
               out float out_hit_t,
               out int3 out_hit_brick,
               out int3 out_hit_cell,
               out int out_hit_slot)
{
  out_hit_t = -1.0f;

  float inv_voxel = 1.0f / vs;
  float brick_world_size = float(BRICK_SIZE) * vs;
  float inv_brick = 1.0f / brick_world_size;

  float3 B = (ray_origin - atlas_orig) * inv_brick;
  float3 BD = ray_dir * inv_brick;

  float3 B_enter = B + t_enter * BD;
  int3 brick_cell = int3(floor(B_enter));
  brick_cell = clamp(brick_cell, int3(0), grid_res - int3(1));

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

  float t_current = t_enter;

  for (int bstep = 0; bstep < MAX_BRICK_STEPS; bstep++) {
    if (any(lessThan(brick_cell, int3(0))) || any(greaterThanEqual(brick_cell, grid_res))) {
      break;
    }

    float t_brick_exit = min(min(brick_tMax.x, brick_tMax.y), brick_tMax.z);
    t_brick_exit = min(t_brick_exit, t_exit);

    /* Read indirection: from texture (world-space) or SSBO (instanced). */
    int slot;
    if (indir_offset < 0) {
      slot = texelFetch(indirection_tx, brick_cell, 0).r;
    }
    else {
      int flat_idx = brick_cell.x + brick_cell.y * grid_res.x +
                     brick_cell.z * grid_res.x * grid_res.y;
      slot = shape_indir[indir_offset + flat_idx];
    }

    if (slot >= 0) {
      /* Active brick: voxel-level DDA. */
      float3 brick_origin = atlas_orig + float3(brick_cell * BRICK_SIZE) * vs;
      float3 V = (ray_origin - brick_origin) * inv_voxel;
      float3 VD = ray_dir * inv_voxel;

      float3 V_enter = V + t_current * VD;
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

      float vt_current = t_current;
      bool voxel_hit = false;

      for (int vstep_i = 0; vstep_i < MAX_VOXEL_STEPS; vstep_i++) {
        if (any(lessThan(vcell, int3(0))) || any(greaterThan(vcell, int3(BRICK_SIZE - 1)))) {
          break;
        }

        float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
        vt_cell_exit = min(vt_cell_exit, t_brick_exit);

        float s[8];
        fetchCornersCompact(brick_cell, vcell, slot, bpa, s);

        float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                         min(min(s[4], s[5]), min(s[6], s[7])));
        float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                         max(max(s[4], s[5]), max(s[6], s[7])));

        if (smin <= 0.0f) {
          if (smax < 0.0f) {
            out_hit_t = vt_current;
            out_hit_brick = brick_cell;
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
              out_hit_brick = brick_cell;
              out_hit_cell = vcell;
              out_hit_slot = slot;
              voxel_hit = true;
              break;
            }

            float u_hit = solveCubicMarmittNR(c, 1.0f);
            if (u_hit >= 0.0f) {
              out_hit_t = vt_current + u_hit * T_max;
              out_hit_brick = brick_cell;
              out_hit_cell = vcell;
              out_hit_slot = slot;
              voxel_hit = true;
              break;
            }
          }
        }

        /* Advance voxel DDA. */
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

    /* Advance brick-level DDA. */
    if (brick_tMax.x < brick_tMax.y) {
      if (brick_tMax.x < brick_tMax.z) {
        t_current = brick_tMax.x;
        brick_cell.x += brick_step.x;
        brick_tMax.x += brick_tDelta.x;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }
    else {
      if (brick_tMax.y < brick_tMax.z) {
        t_current = brick_tMax.y;
        brick_cell.y += brick_step.y;
        brick_tMax.y += brick_tDelta.y;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }

    if (t_current > t_exit) {
      break;
    }
  }
}

/* ---- Instanced march: BVH over instances + per-shape DDA ---- */

struct InstanceHit {
  float world_t;
  int instance_id;
  int3 hit_brick;
  int3 hit_cell;
  int hit_slot;
  float3 local_hit_pos;
  int shape_id;
};

/**
 * March ray against all instances using BVH traversal.
 * For each instance candidate, transforms ray to shape-local space and
 * runs DDA through the shape's per-shape atlas.
 *
 * Returns closest hit across all instances.
 */
InstanceHit march_instanced(float3 ray_origin, float3 ray_dir)
{
  InstanceHit best;
  best.world_t = -1.0f;
  best.instance_id = -1;

  float3 inv_dir = 1.0f / ray_dir;
  float best_world_t = 1e30f;

  int stack[MAX_INST_STACK];
  int sp = 0;
  stack[sp++] = 0;

  while (sp > 0) {
    int node_idx = stack[--sp];
    BVHNodeGPU node = bvh_nodes[node_idx];

    /* Ray-AABB test against node bounds. */
    float node_t_near, node_t_far;
    if (!ray_aabb_intersect(ray_origin, inv_dir,
                            node.min_and_left.xyz, node.max_and_right.xyz,
                            node_t_near, node_t_far))
    {
      continue;
    }

    /* Early cull: if node entry is already behind our best hit, skip. */
    if (node_t_near > best_world_t) {
      continue;
    }

    int left = bvh_decode_int(node.min_and_left.w);
    int right = bvh_decode_int(node.max_and_right.w);

    if (left == -1) {
      /* Leaf node: test this instance.
       * BVH leaf right = object index = instance index
       * (1:1 mapping between objects[] and instances[]). */
      int inst_idx = right;
      if (inst_idx < 0 || inst_idx >= instance_count) {
        continue;
      }

      SDFInstanceGPU inst = instances[inst_idx];
      SDFShapeGPU shape = shapes[inst.shape_id];

      /* Check if this shape has been baked (atlas_params.y = active_brick_count). */
      if (shape.atlas_params.y <= 0) {
        continue;
      }

      /* Transform ray to shape's local space. */
      float3 local_origin = (inst.world_to_local * float4(ray_origin, 1.0f)).xyz;
      float3 local_dir_raw = (inst.world_to_local * float4(ray_dir, 0.0f)).xyz;
      float local_dir_len = length(local_dir_raw);
      if (local_dir_len < 1e-8f) {
        continue;
      }
      float3 local_dir = local_dir_raw / local_dir_len;

      /* Clip ray to shape's local atlas AABB. */
      int3 shape_grid_res = shape.grid_params.xyz;
      float3 shape_origin = shape.local_params.xyz;
      float shape_vs = shape.local_params.w;
      int3 shape_total_res = shape_grid_res * BRICK_SIZE;
      float3 local_min = shape_origin;
      float3 local_max = shape_origin + float3(shape_total_res) * shape_vs;

      float3 local_inv_dir = 1.0f / local_dir;
      float3 lt0 = (local_min - local_origin) * local_inv_dir;
      float3 lt1 = (local_max - local_origin) * local_inv_dir;
      float3 lt_lo = min(lt0, lt1);
      float3 lt_hi = max(lt0, lt1);
      float lt_enter = max(max(lt_lo.x, lt_lo.y), lt_lo.z);
      float lt_exit = min(min(lt_hi.x, lt_hi.y), lt_hi.z);

      if (lt_enter > lt_exit || lt_exit < 0.0f) {
        continue;
      }
      lt_enter = max(lt_enter, 0.0f);

      /* DDA through shape's local atlas. */
      float local_hit_t;
      int3 local_hit_brick, local_hit_cell;
      int local_hit_slot;

      dda_march(local_origin, local_dir, lt_enter, lt_exit,
                shape_grid_res, shape_origin, shape_vs,
                bricks_per_axis, shape.grid_params.w,
                local_hit_t, local_hit_brick, local_hit_cell, local_hit_slot);

      if (local_hit_t >= 0.0f) {
        /* Convert local hit to world space. */
        float3 local_hit = local_origin + local_dir * local_hit_t;
        float3 world_hit = (inst.local_to_world * float4(local_hit, 1.0f)).xyz;
        float wt = length(world_hit - ray_origin);

        if (wt < best_world_t) {
          best_world_t = wt;
          best.world_t = wt;
          best.instance_id = inst_idx;
          best.hit_brick = local_hit_brick;
          best.hit_cell = local_hit_cell;
          best.hit_slot = local_hit_slot;
          best.local_hit_pos = local_hit;
          best.shape_id = inst.shape_id;
        }
      }
    }
    else {
      /* Interior node: push children.
       * Push farther child first so closer child is popped first. */
      float t_left_near, t_left_far, t_right_near, t_right_far;

      bool hit_left = ray_aabb_intersect(ray_origin, inv_dir,
          bvh_nodes[left].min_and_left.xyz, bvh_nodes[left].max_and_right.xyz,
          t_left_near, t_left_far);
      bool hit_right = ray_aabb_intersect(ray_origin, inv_dir,
          bvh_nodes[right].min_and_left.xyz, bvh_nodes[right].max_and_right.xyz,
          t_right_near, t_right_far);

      if (hit_left && hit_right) {
        if (t_left_near < t_right_near) {
          if (sp < MAX_INST_STACK) { stack[sp++] = right; }
          if (sp < MAX_INST_STACK) { stack[sp++] = left; }
        }
        else {
          if (sp < MAX_INST_STACK) { stack[sp++] = left; }
          if (sp < MAX_INST_STACK) { stack[sp++] = right; }
        }
      }
      else if (hit_left) {
        if (sp < MAX_INST_STACK) { stack[sp++] = left; }
      }
      else if (hit_right) {
        if (sp < MAX_INST_STACK) { stack[sp++] = right; }
      }
    }
  }

  return best;
}

/* ---- Main ---- */

void main()
{
  float2 uv = screen_uv;

  /* ---- 1. Reconstruct camera ray ---- */
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

  /* ---- 2. Mode dispatch ---- */
  float hit_t = -1.0f;
  int3 hit_brick = int3(0);
  int3 hit_local_cell = int3(0);
  int hit_slot = -1;
  float3 hit_color = float3(0.5f);
  float3 hit_normal = float3(0.0f, 0.0f, 1.0f);
  float3 hit_pos = float3(0.0f);
  int hit_instance_id = -1;

  if (use_instanced != 0 && instance_count > 0 && bvh_node_count > 0) {
    /* ---- Instanced mode: BVH over instances + per-shape DDA ---- */
    InstanceHit ihit = march_instanced(ray_origin, ray_dir);

    if (ihit.world_t < 0.0f) {
      discard;
      return;
    }

    hit_t = ihit.world_t;
    hit_pos = ray_origin + ray_dir * hit_t;
    hit_instance_id = ihit.instance_id;

    /* Get color from instance. */
    SDFInstanceGPU inst = instances[ihit.instance_id];
    hit_color = inst.color.rgb;

    /* Compute normal in local space, transform to world. */
    SDFShapeGPU shape = shapes[ihit.shape_id];
    float shape_vs = shape.local_params.w;
    float3 shape_origin = shape.local_params.xyz;
    float3 brick_origin_local = shape_origin +
                                 float3(ihit.hit_brick * BRICK_SIZE) * shape_vs;
    float3 hit_in_brick = (ihit.local_hit_pos - brick_origin_local) / shape_vs;

    float3 local_normal;
    if (normal_quality == 0) {
      local_normal = computeNormalCompact(hit_in_brick, ihit.hit_brick, ihit.hit_slot, bricks_per_axis);
    }
    else {
      local_normal = computeDualVoxelNormal(hit_in_brick, ihit.hit_brick, ihit.hit_slot, bricks_per_axis);
    }

    /* Transform normal: N_world = normalize(transpose(world_to_local) * N_local). */
    hit_normal = normalize(mat3(transpose(inst.world_to_local)) * local_normal);
  }
  else {
    /* ---- World-space mode: global two-level DDA ---- */
    int3 grid_res = grid_resolution;
    int3 total_res = grid_res * BRICK_SIZE;
    float3 grid_world_min = atlas_origin;
    float3 grid_world_max = atlas_origin + float3(total_res) * voxel_size;

    float3 inv_dir = 1.0f / ray_dir;
    float3 t0 = (grid_world_min - ray_origin) * inv_dir;
    float3 t1 = (grid_world_max - ray_origin) * inv_dir;
    float3 t_lo = min(t0, t1);
    float3 t_hi = max(t0, t1);
    float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    float t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

    if (t_enter > t_exit || t_exit < 0.0f) {
      discard;
      return;
    }
    t_enter = max(t_enter, 0.0f);

    /* DDA through world-space atlas (indir_offset = -1 → use texture sampler). */
    dda_march(ray_origin, ray_dir, t_enter, t_exit,
              grid_res, atlas_origin, voxel_size, bricks_per_axis, -1,
              hit_t, hit_brick, hit_local_cell, hit_slot);

    if (hit_t < 0.0f) {
      discard;
      return;
    }

    hit_pos = ray_origin + ray_dir * hit_t;

    /* Read blended color from atlas. */
    float inv_voxel = 1.0f / voxel_size;
    float3 brick_origin = atlas_origin + float3(hit_brick * BRICK_SIZE) * voxel_size;
    float3 local_pos = (hit_pos - brick_origin) * inv_voxel;

    int bpa = bricks_per_axis;
    int3 slot_block = int3(hit_slot % bpa, (hit_slot / bpa) % bpa, hit_slot / (bpa * bpa));
    float3 atlas_pos = float3(slot_block * BRICK_STORAGE) + local_pos + float3(2.0f);
    int3 compact_size = int3(textureSize(compact_atlas, 0));
    float3 atlas_uv = atlas_pos / float3(compact_size);
    hit_color = textureLod(compact_atlas, atlas_uv, 0.0f).gba;

    /* Compute normal. */
    float3 hit_in_brick = (hit_pos - brick_origin) * inv_voxel;
    if (normal_quality == 0) {
      hit_normal = computeNormalCompact(hit_in_brick, hit_brick, hit_slot, bricks_per_axis);
    }
    else {
      hit_normal = computeDualVoxelNormal(hit_in_brick, hit_brick, hit_slot, bricks_per_axis);
    }
  }


  /* ---- Shading ---- */
  float3 normal = hit_normal;
  float3 obj_color = hit_color;
  float3 shaded_color;

  if (lighting_type == 0) {
    shaded_color = obj_color;
  }
  else if (lighting_type == 1) {
    /* STUDIO lighting. */
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
    /* MATCAP lighting. */
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

  /* Write depth. */
  gl_FragDepth = drw_point_world_to_screen(hit_pos).z;
}
