/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning trace pass: sphere tracing where each step evaluates the
 * pruned CSG tree of the grid cell containing the sample point. Far-field
 * cells (num_active == 0) return a stored constant lower bound, giving large
 * empty-space steps for free. Writes the same G-buffer as the classic trace
 * so the shared shade/blit passes finish the frame.
 *
 * Port of simple.frag.glsl from the reference engine.
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_lp_trace_comp)

#include "draw_view_lib.glsl"
#include "sdf_lp_common.glsl"

#define LP_SHADING_SHADED 0
#define LP_SHADING_HEATMAP 1
#define LP_SHADING_NORMALS 2

int lp_cell_num_active(int cell_idx)
{
  return lp_cell_meta[cell_idx].x;
}

float lp_sdf(float3 p, int cell_idx, out bool near_field)
{
  if (culling_enabled == 0) {
    near_field = true;
    return lp_list_eval(p, total_num_nodes, 0);
  }
  /* Single meta load: num_active (x), list offset (y), far value (z). */
  int4 meta = lp_cell_meta[cell_idx];
  int num_active = meta.x;
  if (num_active == SDF_LP_FALLBACK_LIST) {
    /* Cell overflowed the dynamic pools during pruning: full tree eval. */
    near_field = true;
    return lp_list_eval(p, total_num_nodes, 0);
  }
  if (num_active == 0) {
    near_field = false;
    return intBitsToFloat(meta.z);
  }
  near_field = true;
  return lp_list_eval(p, num_active, meta.y);
}

float3 lp_grad(float3 p, int cell_idx)
{
  float h = 5e-4f;
  const float2 k = float2(1.0f, -1.0f);
  bool nf = false;
  return normalize(k.xyy * lp_sdf(p + k.xyy * h, cell_idx, nf) +
                   k.yyx * lp_sdf(p + k.yyx * h, cell_idx, nf) +
                   k.yxy * lp_sdf(p + k.yxy * h, cell_idx, nf) +
                   k.xxx * lp_sdf(p + k.xxx * h, cell_idx, nf));
}

float4 lp_albedo(float3 p, int cell_idx, out float obj_id)
{
  if (culling_enabled == 0) {
    return lp_list_eval_color(p, total_num_nodes, 0, obj_id);
  }
  int4 meta = lp_cell_meta[cell_idx];
  int num_active = meta.x;
  if (num_active == SDF_LP_FALLBACK_LIST) {
    return lp_list_eval_color(p, total_num_nodes, 0, obj_id);
  }
  if (num_active == 0) {
    obj_id = -1.0f;
    return float4(0.0f);
  }
  return lp_list_eval_color(p, num_active, meta.y, obj_id);
}

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
    imageStore(gbuf_normal_img, pixel, float4(0.0f));
    return;
  }
  t += 1e-4f;

  float3 cell_size = (aabb_max - aabb_min) / float(grid_size);

  bool hit = false;
  float hit_t = t;
  float t_prev = t;
  float d_prev = 1e30f;
  /* Over-relaxed sphere tracing (same scheme as the classic engine,
   * sdf_trace_comp.glsl:728-875): take omega * |d| steps and fall back to
   * omega = 1 for the rest of the ray when an overshoot is detected
   * (the radius shrank instead of growing past the previous step). Cuts the
   * step count roughly in half at omega ~1.5-2 on smooth fields. */
  float prev_radius = 0.0f;
  float step_length = 0.0f;
  float omega = over_relaxation;
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

    float abs_d = abs(d);
    bool sor_fail = omega > 1.0f && (abs_d + prev_radius) < step_length * 1.01f;
    if (sor_fail) {
      step_length -= omega * step_length;
      omega = 1.0f;
    }
    else {
      step_length = abs_d * omega;
    }

    if (!sor_fail && near_field && abs(d) < min(ray_epsilon, ray_epsilon * t)) {
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
    prev_radius = abs_d;
    if (sor_fail) {
      t += step_length;
    }
    else {
      t += max(step_length, ray_epsilon * 0.5f);
    }
  }

  if (!hit) {
    imageStore(gbuf_pos_img, pixel, float4(0.0f));
    imageStore(gbuf_color_img, pixel, float4(0.0f));
    imageStore(gbuf_normal_img, pixel, float4(0.0f));
    return;
  }

  float3 hit_pos = ray_origin + hit_t * ray_dir;
  int3 hit_cell = lp_cell_from_pos(hit_pos, aabb_min, cell_size, grid_size);
  int hit_cell_idx = int(lp_cell_idx(hit_cell));

  float obj_id = -1.0f;
  float4 albedo = lp_albedo(hit_pos, hit_cell_idx, obj_id);

  /* Normals: only when the hit cell resolves to a single primitive (a lone
   * mesh leaf, no CSG ops or blends in play) do we derive the shading normal
   * from the winning triangle's corner normals — smooth like a regular mesh
   * and nearly free (one hint-warm mesh re-evaluation, no finite
   * differences). In every other case — analytic shapes, modifier-warped
   * meshes, and especially blend/CSG zones where the true normal is a
   * position-dependent mix of both fields — we keep the 4-tap gradient.
   * Using the mesh normal inside blend zones is both wrong (the surface is
   * not the mesh there) and unstable (the obj_id winner flips mid-blend, so
   * the normal source popped with camera position). */
  float3 normal = float3(0.0f);
  bool normal_valid = false;
  int cell_active = (culling_enabled != 0) ? lp_cell_num_active(hit_cell_idx) :
                                             total_num_nodes;
  if (cell_active == 1 && obj_id >= 0.0f) {
    SDFObjectGPU hit_obj = objects[int(obj_id + 0.5f)];
    if (hit_obj.sdf_type == SDF_GPU_TYPE_MESH && hit_obj.modifier_count == 0) {
      /* Re-evaluate the winning mesh so the last-hit record belongs to this
       * object even when several meshes share the scene. */
      float3 hit_lp = (hit_obj.inverse_matrix * float4(hit_pos - hit_obj.position.xyz, 1.0f)).xyz;
      lp_sd_triangle_mesh(hit_lp, hit_obj.mesh_data, hit_obj.mesh_settings.z, hit_obj.obj_scale);
      float3 geometric_normal;
      normal_valid = lp_mesh_last_world_normals(hit_obj, normal, geometric_normal);
      if (normal_valid) {
        normal = normalize(normal);
      }
    }
  }
  if (!normal_valid) {
    normal = lp_grad(hit_pos, hit_cell_idx);
  }
  if (any(isnan(normal)) || dot(normal, normal) < 0.5f) {
    normal = -ray_dir;
  }

  float3 out_color = albedo.rgb;
  if (shading_mode == LP_SHADING_HEATMAP) {
    float num_active = 0.0f;
    if (culling_enabled != 0) {
      /* Divide by 2 to approximate the number of primitives (binary tree).
       * Fallback cells show as the maximum (useful overflow diagnostic). */
      int cell_count = lp_cell_num_active(hit_cell_idx);
      num_active = float(cell_count >= 0 ? cell_count : total_num_nodes) + 1.0f;
      num_active *= 0.5f;
    }
    else {
      num_active = float(total_num_nodes + 1) * 0.5f;
    }
    out_color = lp_inferno(clamp(num_active / max(viz_max, 1.0f), 0.0f, 1.0f));
  }
  else if (shading_mode == LP_SHADING_NORMALS) {
    out_color = normal * 0.5f + 0.5f;
  }

  imageStore(gbuf_pos_img, pixel, float4(hit_pos, 1.0f));
  imageStore(gbuf_color_img, pixel, float4(out_color, obj_id));
  imageStore(gbuf_normal_img, pixel, float4(normal, 0.0f));
}
