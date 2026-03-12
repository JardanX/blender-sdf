/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF outline fragment shader — per-object analytical sphere-march.
 *
 * Each selected SDF object is sphere-marched independently against the
 * camera ray using its raw primitive SDF (no CSG). The closest hit
 * across all selected objects wins.
 *
 * CSG operations (subtract, intersect, etc.) do NOT affect outlines —
 * each object always shows its full primitive silhouette. Boundaries
 * between overlapping selected objects are found naturally by the edge
 * detector (each object has a unique packed outline ID).
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_outline_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl" /* evalObjectSDF */

#define MAX_MARCH_STEPS 48

/* ---- Evaluate a single SDF primitive with modifiers ---- */

/** Matches evalSDFPrimitive() in sdf_classify_comp.glsl. */
float evalOutlinePrimitive(float3 local_pos, SDFObjectGPU obj)
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

void main()
{
  float2 uv = screen_uv;

  /* Reconstruct camera ray. */
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

  /* Per-object sphere-march for outlines.
   * Only selected objects are marched. Each object uses its raw primitive
   * SDF — CSG operations are intentionally ignored so outlines always show
   * the full primitive silhouette. Closest hit wins. */
  float best_depth = 1.0f;
  uint best_outline_id = 0u;

  for (int i = 0; i < object_count; i++) {
    SDFObjectGPU obj = sdf_objects[i];
    /* Skip non-selected objects entirely. */
    uint oid = outline_id_map_buf[obj.original_index];
    if (oid == 0u) {
      continue;
    }

    /* Ray-AABB intersection test. */
    float3 bmin = sdf_objects[i].bbox_min.xyz;
    float3 bmax = sdf_objects[i].bbox_max.xyz;
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

    /* Threshold: sub-voxel precision, capped at 0.2% of object extent. */
    float3 obj_extent = bmax - bmin;
    float thr = 0.002f * max(obj_extent.x, max(obj_extent.y, obj_extent.z));
    thr = clamp(thr, 1e-5f, voxel_size * 0.25f);
    float min_step = thr * 0.5f;

    /* Sphere-march this object's individual SDF. */
    float t = t_enter;
    bool hit = false;
    float3 hit_pos;

    for (int s = 0; s < MAX_MARCH_STEPS; s++) {
      float3 wp = ray_origin + ray_dir * t;
      float3 lp = (obj.inverse_matrix * float4(wp - obj.position.xyz, 1.0f)).xyz;
      float d = evalOutlinePrimitive(lp, obj);

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
      float depth = drw_point_world_to_screen(hit_pos).z;
      if (depth < best_depth) {
        best_depth = depth;
        best_outline_id = oid;
      }
    }
  }

  if (best_outline_id == 0u) {
    discard;
    return;
  }

  out_object_id = best_outline_id;
  gl_FragDepth = best_depth - 1e-5f;
}
