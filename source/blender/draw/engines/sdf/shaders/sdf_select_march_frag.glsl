/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF selection fragment shader — analytical per-object sphere-march.
 *
 * Instead of marching the baked voxel atlas (which has quantization and
 * staleness issues), this shader evaluates each SDF object analytically.
 * For every pixel, it tests all objects' AABBs against the camera ray,
 * then sphere-marches each candidate to find the exact surface. The
 * closest hit across all objects is written to the selection buffer.
 *
 * This approach matches the outline shader (sdf_outline_march_frag.glsl)
 * but outputs select::ID instead of outline IDs.
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_select_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define MAX_MARCH_STEPS 48

/* ---- Evaluate a single SDF primitive with modifiers ---- */

float evalSelectPrimitive(float3 local_pos, SDFObjectGPU obj)
{
  SDFPrimitiveData prim_data;
  prim_data.sdf_type = obj.sdf_type;
  prim_data.size = obj.sdf_size.xyz;
  prim_data.bevel = obj.bevel;
  prim_data.box_corners = obj.box_corners;
  prim_data.box_edges = obj.box_edges;
  prim_data.box_modes = obj.box_modes;
  prim_data.modifier_start = obj.modifier_start;
  prim_data.modifier_count = obj.modifier_count;

  return evalObjectSDF(prim_data, local_pos);
}

/* ---- Write a single hit to the selection buffer ---- */

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
  float3 inv_dir = 1.0f / ray_dir;

  /* ---- 2. Group-aware per-object analytical sphere-march ----
   * For grouped objects, evaluate the combined group SDF at each march step
   * so CSG operations are respected (e.g., subtract carves away geometry).
   * For each pixel, the closest hit across all objects is tracked.
   * Selection cycling is supported: each individual object writes a hit. */
  float best_depth = 1.0f;
  bool any_hit = false;

  for (int i = 0; i < object_count; i++) {
    SDFObjectGPU obj = sdf_objects[i];

    /* Ray-AABB intersection test. */
    float3 bmin = obj.bbox_min.xyz;
    float3 bmax = obj.bbox_max.xyz;
    float3 t0 = (bmin - ray_origin) * inv_dir;
    float3 t1 = (bmax - ray_origin) * inv_dir;
    float3 t_lo = min(t0, t1);
    float3 t_hi = max(t0, t1);
    float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    float t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

    if (t_enter > t_exit || t_exit < 0.0f) {
      continue;
    }
    t_enter = max(t_enter, 0.0f);

    /* Adaptive threshold: sub-voxel precision, capped at 0.2% of object extent. */
    float3 obj_extent = bmax - bmin;
    float thr = 0.002f * max(obj_extent.x, max(obj_extent.y, obj_extent.z));
    thr = clamp(thr, 1e-5f, voxel_size * 0.25f);
    float min_step = thr * 0.5f;

    /* Sphere-march this object's analytical SDF. */
    float t = t_enter;
    bool hit = false;
    float3 hit_pos;

    for (int s = 0; s < MAX_MARCH_STEPS; s++) {
      float3 wp = ray_origin + ray_dir * t;
      float3 lp = (obj.inverse_matrix * float4(wp - obj.position.xyz, 1.0f)).xyz;
      float d = evalSelectPrimitive(lp, obj);

      /* For grouped objects, verify this hit against the combined group SDF.
       * Without this, the base shape registers false hits at its raw surface
       * even where subtract/intersect operations have carved it away, blocking
       * selection of the objects that perform those CSG operations. */
      if (d < thr && obj.group_id >= 0) {
        /* Evaluate group combined SDF at this world position.
         * Iterate all objects filtering by group_id (members may not be contiguous). */
        int gid = obj.group_id;
        float grp_dist = 1e10f;
        for (int m = 0; m < object_count; m++) {
          SDFObjectGPU mobj = sdf_objects[m];
          if (mobj.group_id != gid) {
            continue;
          }
          float3 mlp = (mobj.inverse_matrix * float4(wp - mobj.position.xyz, 1.0f)).xyz;
          float md = evalSelectPrimitive(mlp, mobj);
          if (grp_dist >= 1e9f) {
            /* First object in group is always the base shape — CSG op ignored. */
            grp_dist = md;
          }
          else {
            grp_dist = combineCSG(grp_dist, md, mobj.csg_operation, mobj.blend_type,
                                  mobj.blend, mobj.shell_distance);
          }
        }
        /* Only count as hit if the combined group surface is near. */
        if (grp_dist >= thr * 2.0f) {
          d = grp_dist; /* Not a real surface here, continue marching. */
        }
      }

      if (d < thr) {
        hit = true;
        hit_pos = wp;
        break;
      }

      t += max(d, min_step);
      if (t > t_exit) {
        break;
      }
    }

    if (hit) {
      /* Write this object's hit to the selection buffer immediately.
       * For SELECT_PICK_ALL: each object gets its own entry (enables cycling).
       * For SELECT_PICK_NEAREST: atomicMin picks the closest automatically.
       * For SELECT_ALL: bitmap OR records all objects. */
      float depth = drw_point_world_to_screen(hit_pos).z;
      uint sel_id = select_id_map_buf[obj.original_index];
      writeSelectHit(sel_id, depth);

      if (depth < best_depth) {
        best_depth = depth;
      }
      any_hit = true;
    }
  }

  /* ---- 3. No hit: discard fragment ---- */
  if (!any_hit) {
    discard;
    return;
  }

  /* ---- 4. Set depth to nearest hit for correct mesh/SDF interop ---- */
  gl_FragDepth = best_depth;
}
