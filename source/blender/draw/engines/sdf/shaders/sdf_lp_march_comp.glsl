/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning march pass: sphere tracing where each step evaluates the
 * pruned CSG tree of the grid cell containing the sample point. Far-field
 * cells (num_active == 0) return a stored constant lower bound, giving large
 * empty-space steps for free. Writes the position G-buffer (w=1 on hit) and
 * seeds gbuf_color.a with the dominant object id for picking (mirroring
 * sdf_trace_comp.glsl); the hit color/normal are produced by the classic
 * color resolve pass (sdf_color_resolve_comp) in the default shading mode,
 * or by sdf_lp_resolve_comp in the debug shading modes.
 *
 * Compiled with SDF_LP_NO_COLOR: the folded color/normal evaluators in
 * sdf_lp_common.glsl are stripped, which drastically cuts driver compile
 * time for this latency-critical shader.
 *
 * Port of simple.frag.glsl from the reference engine. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_lp_march_comp)

#include "draw_view_lib.glsl"
#include "sdf_lp_common.glsl"
#include "sdf_lp_trace_lib.glsl"

bool lp_bbox_intersect(float3 box_min, float3 box_max, float3 r_o, float3 r_d, out float t_inter)
{
  float3 safe_dir = float3(abs(r_d.x) < 1e-8f ? 1e-8f : r_d.x,
                           abs(r_d.y) < 1e-8f ? 1e-8f : r_d.y,
                           abs(r_d.z) < 1e-8f ? 1e-8f : r_d.z);
  float3 tbot = (box_min - r_o) / safe_dir;
  float3 ttop = (box_max - r_o) / safe_dir;
  float3 tmin = min(ttop, tbot);
  float3 tmax = max(ttop, tbot);
  float2 t = max(tmin.xx, tmin.yz);
  float t0 = max(t.x, t.y);
  t = min(tmax.xx, tmax.yz);
  float t1 = min(t.x, t.y);
  t_inter = max(t0, 0.0f);
  return t1 > max(t0, 0.0f);
}

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);
  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float2 uv = (float2(pixel) + 0.5f) / float2(screen_size);

  ViewMatrices vm = drw_view();
  float4 ndc_near = float4(uv * 2.0f - 1.0f, -1.0f, 1.0f);
  float4 ndc_far = float4(uv * 2.0f - 1.0f, 1.0f, 1.0f);
  float4 world_near = vm.viewinv * (vm.wininv * ndc_near);
  float4 world_far = vm.viewinv * (vm.wininv * ndc_far);
  world_near.xyz /= world_near.w;
  world_far.xyz /= world_far.w;

  float3 ray_origin = world_near.xyz;
  float3 ray_dir = normalize(world_far.xyz - world_near.xyz);

  bool aabb_valid = all(lessThan(aabb_min, aabb_max)) && total_num_nodes > 0;

  float t = 0.0f;
  if (!aabb_valid || !lp_bbox_intersect(aabb_min, aabb_max, ray_origin, ray_dir, t)) {
    imageStore(gbuf_pos_img, pixel, float4(0.0f));
    imageStore(gbuf_color_img, pixel, float4(0.0f));
    return;
  }
  t += 1e-4f;

  float3 cell_size = (aabb_max - aabb_min) / float(grid_size);

  /* Step scaling: the pruned-tree field is NOT a conservative distance
   * under-estimate when ROUND blends are present — along a fillet crest
   * between (near-)parallel surfaces the field's position-Lipschitz constant
   * exceeds 1 (up to sqrt(n) for n nested ROUND ops; measured numerically),
   * so |d| can overestimate the true distance and a |d| step can clear a
   * thin feature between two fillet crossings (the classic engine catches
   * those with its SOR radius-sum backtrack, sdf_trace_comp.glsl:805).
   * Dividing by the full tree's Lipschitz constant makes every step provably
   * <= the true distance (|f(p)| <= scene_l * dist(p, zero-set)). Far-cell
   * constants are already pre-divided by the prune pass. For LINEAR/SMOOTH/
   * CHAMFER scenes scene_l == 1 and nothing changes. */
  float scene_l = lp_nodes[total_num_nodes - 1].lipschitz;

  bool hit = false;
  float hit_t = t;
  float t_prev = t;
  float d_prev = 1e30f;
  for (int i = 0; i < max_steps; i++) {
    float3 p = ray_origin + t * ray_dir;

    if (any(lessThan(p, aabb_min)) || any(greaterThanEqual(p, aabb_max))) {
      break;
    }

    int3 cell = lp_cell_from_pos(p, aabb_min, cell_size, grid_size);
    int cell_idx = int(lp_cell_idx(cell));

    bool near_field = true;
    float d = lp_sdf(p, cell_idx, near_field);
    if (isnan(d) || isinf(d)) {
      break;
    }

    /* Sign change: a step crossed the surface between t_prev and t. Plain
     * |d| steps cannot overshoot (far-cell values are conservative bounds),
     * but this keeps the hit test robust when a bound step lands exactly on
     * or just past the surface. */
    bool sign_change = d_prev < 1e29f && d_prev > 0.0f && d < 0.0f;

    if (near_field && (abs(d) < min(ray_epsilon, ray_epsilon * t) || sign_change)) {
      hit = true;
      /* Secant refinement: interpolate the zero crossing between the previous
       * and current SDF samples (both already evaluated, 0 extra SDF evals).
       * Shrinks the hit error from ~ray_epsilon to near-exact, removing the
       * depth stair-steps along the ray. */
      hit_t = t;
      float denom = d_prev - d;
      if (d_prev < 1e29f && denom > 1e-8f) {
        float alpha = d_prev / denom;
        if (alpha > 0.0f && alpha < 1.0f) {
          hit_t = mix(t_prev, t, alpha);
        }
      }
      break;
    }
    t_prev = t;
    d_prev = d;
    /* Near-field value: scale to a distance bound (see above). Far-cell
     * constant: already a distance bound, step as-is. */
    t += near_field ? abs(d) / scene_l : abs(d);
  }

  if (!hit) {
    imageStore(gbuf_pos_img, pixel, float4(0.0f));
    imageStore(gbuf_color_img, pixel, float4(0.0f));
    return;
  }

  float3 hit_pos = ray_origin + hit_t * ray_dir;

  /* History-independent surface snap (port of sdf_trace_comp.glsl:841-861):
   * the secant hit_t can be off by far more than ray_epsilon at grazing
   * angles (the d_prev-d denominator degenerates), which makes the shading
   * point jitter across surface features as the camera moves — visible as
   * camera-dependent normal noise, especially on flat faces where landing
   * on the wrong side of an edge flips the face normal. Two fresh
   * evaluations (hint-warm, cheap) give a cosine-corrected projection that
   * pins the point to the surface regardless of ray history. */
  {
    int3 snap_cell = lp_cell_from_pos(hit_pos, aabb_min, cell_size, grid_size);
    int snap_cell_idx = int(lp_cell_idx(snap_cell));
    bool snap_nf = true;
    float d0 = lp_sdf(hit_pos, snap_cell_idx, snap_nf);
    float eps_snap = ray_epsilon * 0.5f;
    float3 p1 = hit_pos + ray_dir * eps_snap;
    bool p1_in = all(greaterThanEqual(p1, aabb_min)) && all(lessThan(p1, aabb_max));
    if (snap_nf && p1_in && !isnan(d0) && !isinf(d0)) {
      int3 p1_cell = lp_cell_from_pos(p1, aabb_min, cell_size, grid_size);
      bool nf1 = true;
      float d1 = lp_sdf(p1, int(lp_cell_idx(p1_cell)), nf1);
      if (!isnan(d1) && !isinf(d1)) {
        /* Sign-preserving cosine so exit-surface snap heads forward. */
        float cos_raw = (d0 - d1) / eps_snap;
        float cos_est = (cos_raw >= 0.0f ? 1.0f : -1.0f) * clamp(abs(cos_raw), 0.1f, 1.0f);
        hit_pos += ray_dir * d0 / cos_est;
      }
    }
  }

  /* Dominant object id at the snapped hit point: seeds gbuf_color.a, which
   * the color resolve pass carries through to the final G-buffer for object
   * picking (same convention as sdf_trace_comp.glsl). */
  int3 id_cell = lp_cell_from_pos(hit_pos, aabb_min, cell_size, grid_size);
  float obj_id = lp_sdf_obj_id(hit_pos, int(lp_cell_idx(id_cell)));

  imageStore(gbuf_pos_img, pixel, float4(hit_pos, 1.0f));
  imageStore(gbuf_color_img, pixel, float4(0.0f, 0.0f, 0.0f, obj_id));
}
