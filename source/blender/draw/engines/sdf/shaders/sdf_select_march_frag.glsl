/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF selection march fragment shader.
 * Stripped-down version of sdf_march_frag.glsl — same DDA logic,
 * but outputs select_id instead of shading.
 *
 * After finding the first surface, marches through the interior sampling
 * the object_id atlas to discover all objects under the cursor. This handles
 * smooth unions where objects blend into a single surface.
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_select_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

/** Max brick-level DDA steps. */
#define MAX_BRICK_STEPS 128
/** Max voxel-level DDA steps within one brick. */
#define MAX_VOXEL_STEPS 24
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

/** Read object ID from the object ID atlas at a world-space position.
 * Computes brick cell and slot from the position. Returns -1 if out of bounds
 * or the brick is inactive. */
int readObjectIdAtPos(float3 world_pos)
{
  float3 grid_pos = (world_pos - atlas_origin) / (float(BRICK_SIZE) * voxel_size);
  int3 brick = int3(floor(grid_pos));
  if (any(lessThan(brick, int3(0))) || any(greaterThanEqual(brick, grid_resolution))) {
    return -1;
  }

  int slot = texelFetch(indirection_tx, brick, 0).r;
  if (slot < 0) {
    return -1;
  }

  float3 brick_origin = atlas_origin + float3(brick * BRICK_SIZE) * voxel_size;
  float3 local_pos = (world_pos - brick_origin) / voxel_size;
  int3 slot_block = int3(slot % bricks_per_axis,
                         (slot / bricks_per_axis) % bricks_per_axis,
                         slot / (bricks_per_axis * bricks_per_axis));
  int3 atlas_coord = slot_block * BRICK_STORAGE + int3(floor(local_pos)) + int3(2);
  return texelFetch(object_id_tx, atlas_coord, 0).r;
}

/** Write a single hit to the selection buffer (inlined select_id_output). */
void writeSelectHit(uint sel_id, float ndc_depth)
{
  if (sel_id == uint(-1)) {
    return;
  }
  if (select_info_buf.mode == SELECT_ALL) {
    atomicOr(out_select_buf[sel_id / 32u], 1u << (sel_id % 32u));
  }
  else if (select_info_buf.mode == SELECT_PICK_ALL) {
    atomicMin(out_select_buf[sel_id], floatBitsToUint(ndc_depth));
  }
  else if (select_info_buf.mode == SELECT_PICK_NEAREST) {
    int2 coord = abs(int2(gl_FragCoord.xy) - select_info_buf.cursor);
    uint dist = uint(max(coord.x, coord.y));
    uint depth = uint(ndc_depth * float(0x00FFFFFFu));
    if (dist < 0xFFu) {
      atomicMin(out_select_buf[sel_id], (depth << 8u) | dist);
    }
  }
}

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

  /* ---- 2. Clip ray to total voxel volume ---- */
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

  /* ---- 3. Brick-level DDA setup ---- */
  float inv_voxel = 1.0f / voxel_size;
  float brick_world_size = float(BRICK_SIZE) * voxel_size;
  float inv_brick = 1.0f / brick_world_size;

  float3 B = (ray_origin - atlas_origin) * inv_brick;
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

  /* ---- 4. Single-hit DDA traversal (find first surface) ---- */
  float hit_t = -1.0f;
  float3 hit_pos;

  for (int bstep = 0; bstep < MAX_BRICK_STEPS; bstep++) {
    if (any(lessThan(brick_cell, int3(0))) || any(greaterThanEqual(brick_cell, grid_res))) {
      break;
    }

    float t_brick_exit = min(min(brick_tMax.x, brick_tMax.y), brick_tMax.z);
    t_brick_exit = min(t_brick_exit, t_exit);

    int slot = texelFetch(indirection_tx, brick_cell, 0).r;

    if (slot == -2) {
      break;
    }

    if (slot >= 0) {
      float3 brick_origin = atlas_origin + float3(brick_cell * BRICK_SIZE) * voxel_size;
      float3 V = (ray_origin - brick_origin) * inv_voxel;
      float3 VD = ray_dir * inv_voxel;

      float3 V_enter = V + t_enter * VD;
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

      float vt_current = t_enter;

      for (int vstep_i = 0; vstep_i < MAX_VOXEL_STEPS; vstep_i++) {
        if (any(lessThan(vcell, int3(0))) || any(greaterThan(vcell, int3(BRICK_SIZE - 1)))) {
          break;
        }

        float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
        vt_cell_exit = min(vt_cell_exit, t_brick_exit);

        float s[8];
        fetchCornersCompact(brick_cell, vcell, slot, bricks_per_axis, s);

        float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                         min(min(s[4], s[5]), min(s[6], s[7])));

        if (smin <= 0.0f) {
          float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                           max(max(s[4], s[5]), max(s[6], s[7])));

          if (smax < 0.0f) {
            hit_t = vt_current;
          }
          else {
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
                hit_t = vt_current;
              }
              else {
                float u_hit = solveCubicMarmittNR(c, 1.0f);
                if (u_hit >= 0.0f) {
                  hit_t = vt_current + u_hit * T_max;
                }
              }
            }
          }

          if (hit_t >= 0.0f) {
            hit_pos = ray_origin + ray_dir * hit_t;
            break;
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

      if (hit_t >= 0.0f) {
        break;
      }
    }

    /* ---- Advance brick-level DDA ---- */
    if (brick_tMax.x < brick_tMax.y) {
      if (brick_tMax.x < brick_tMax.z) {
        t_enter = brick_tMax.x;
        brick_cell.x += brick_step.x;
        brick_tMax.x += brick_tDelta.x;
      }
      else {
        t_enter = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }
    else {
      if (brick_tMax.y < brick_tMax.z) {
        t_enter = brick_tMax.y;
        brick_cell.y += brick_step.y;
        brick_tMax.y += brick_tDelta.y;
      }
      else {
        t_enter = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }

    if (t_enter > t_exit) {
      break;
    }
  }

  /* ---- 5. No hit: discard fragment ---- */
  if (hit_t < 0.0f) {
    discard;
    return;
  }

  /* ---- 6. Test all objects' AABBs against the ray for cycling ---- */
  /* The object_id atlas only stores one object per voxel, so interior scanning
   * can't reliably find all objects under the cursor (e.g. side-by-side objects
   * that share a blended surface). Instead, test each object's world AABB against
   * the camera ray. Any object whose AABB the ray intersects is a candidate.
   * The first-hit object from the DDA gets the surface depth; others get their
   * AABB entry depth. This matches how mesh selection works in Blender. */

  float base_depth = drw_point_world_to_screen(hit_pos).z;
  gl_FragDepth = base_depth;

  /* First, write the surface hit object (from object_id atlas at hit point). */
  int first_id = readObjectIdAtPos(hit_pos);
  if (first_id >= 0 && first_id < object_count) {
    uint sel_id = select_id_map_buf[first_id];
    writeSelectHit(sel_id, base_depth);
  }

  /* Then test all other objects' AABBs against the ray. */
  for (int i = 0; i < object_count; i++) {
    if (i == first_id) {
      continue;
    }

    float3 bmin = sdf_objects[i].bbox_min.xyz;
    float3 bmax = sdf_objects[i].bbox_max.xyz;

    /* Ray-AABB intersection (slab method). */
    float3 t_min = (bmin - ray_origin) * inv_dir;
    float3 t_max = (bmax - ray_origin) * inv_dir;
    float3 t_near = min(t_min, t_max);
    float3 t_far = max(t_min, t_max);
    float t_in = max(max(t_near.x, t_near.y), t_near.z);
    float t_out = min(min(t_far.x, t_far.y), t_far.z);

    if (t_in <= t_out && t_out >= 0.0f) {
      /* Ray hits this object's AABB. Use the entry point depth. */
      float entry_t = max(t_in, 0.0f);
      float3 entry_pos = ray_origin + ray_dir * entry_t;
      float entry_depth = drw_point_world_to_screen(entry_pos).z;

      uint sel_id = select_id_map_buf[i];
      writeSelectHit(sel_id, entry_depth);
    }
  }
}
