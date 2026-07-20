/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning resolve pass: runs after the march pass
 * (sdf_lp_march_comp) and, for every hit pixel, evaluates the folded
 * color/normal tree and writes the color and normal G-buffers. Miss pixels
 * are cleared here (the march pass only writes the position buffer).
 *
 * This is the heavyweight shader of the LP pipeline (the folded
 * color+normal evaluator inlines the full primitive library plus per-leaf
 * FD normals); splitting it from the march pass lets the driver compile
 * both in parallel and keeps the march shader small. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_lp_resolve_comp)

#include "draw_view_lib.glsl"
#include "sdf_lp_common.glsl"
#include "sdf_lp_trace_lib.glsl"

#define LP_SHADING_SHADED 0
#define LP_SHADING_HEATMAP 1
#define LP_SHADING_NORMALS 2

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

float4 lp_albedo_nrm(float3 p,
                     int cell_idx,
                     out float obj_id,
                     out float3 nrm,
                     out float3 grad,
                     out float mixv,
                     out bool nrm_valid)
{
  if (culling_enabled == 0) {
    return lp_list_eval_color_nrm(p, total_num_nodes, 0, obj_id, nrm, grad, mixv, nrm_valid);
  }
  int4 meta = lp_cell_meta[cell_idx];
  int num_active = meta.x;
  if (num_active == SDF_LP_FALLBACK_LIST) {
    return lp_list_eval_color_nrm(p, total_num_nodes, 0, obj_id, nrm, grad, mixv, nrm_valid);
  }
  if (num_active == 0) {
    /* A secant-refined hit can land in a far-field cell even though the hit
     * sample was near-field. Returning black/invalid here would push the
     * normal path to the gradient of a constant bound (garbage, view
     * dependent) — evaluate the full tree instead. Rare; costs one eval. */
    return lp_list_eval_color_nrm(p, total_num_nodes, 0, obj_id, nrm, grad, mixv, nrm_valid);
  }
  return lp_list_eval_color_nrm(p, num_active, meta.y, obj_id, nrm, grad, mixv, nrm_valid);
}

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);
  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float4 pos = imageLoad(gbuf_pos_img, pixel);
  if (pos.w <= 0.0f) {
    imageStore(gbuf_color_img, pixel, float4(0.0f));
    imageStore(gbuf_normal_img, pixel, float4(0.0f));
    return;
  }
  float3 hit_pos = pos.xyz;

  /* Ray direction, for the degenerate-normal fallback only. */
  float2 uv = (float2(pixel) + 0.5f) / float2(screen_size);
  ViewMatrices vm = drw_view();
  float4 ndc_near = float4(uv * 2.0f - 1.0f, -1.0f, 1.0f);
  float4 ndc_far = float4(uv * 2.0f - 1.0f, 1.0f, 1.0f);
  float4 world_near = vm.viewinv * (vm.wininv * ndc_near);
  float4 world_far = vm.viewinv * (vm.wininv * ndc_far);
  world_near.xyz /= world_near.w;
  world_far.xyz /= world_far.w;
  float3 ray_dir = normalize(world_far.xyz - world_near.xyz);

  float3 cell_size = (aabb_max - aabb_min) / float(grid_size);
  int3 hit_cell = lp_cell_from_pos(hit_pos, aabb_min, cell_size, grid_size);
  int hit_cell_idx = int(lp_cell_idx(hit_cell));

  float obj_id = -1.0f;
  float3 blended_nrm;
  float3 blended_grad;
  float blend_mix;
  bool normal_valid = false;
  /* Tree-blended normals in one folded evaluation. Every leaf contributes a
   * shading normal (mesh: corner-interpolated — smooth where the base mesh
   * is shade-smooth, the analytic face normal where faces are sharp) and a
   * geometric gradient (continuous across mesh triangle flips). Ops blend
   * both with the field's own gradient weights. The final normal cross-fades
   * from the shading normal to the geometric gradient as the blend deepens:
   * pure regions keep their exact flat/smooth shading, blend zones use the
   * stable gradient (flat shading normals jump by the full dihedral angle
   * when the closest face changes, which speckles blend regions). */
  float4 albedo = lp_albedo_nrm(
      hit_pos, hit_cell_idx, obj_id, blended_nrm, blended_grad, blend_mix, normal_valid);

  float3 normal;
  if (normal_valid) {
    float3 s = normalize(blended_nrm);
    float3 g = normalize(blended_grad);
    /* blend_mix: 0 = single operand owns the surface, 1 = 50/50 blend. */
    float gm = clamp(blend_mix * 2.0f, 0.0f, 1.0f);
    normal = normalize(mix(s, g, gm));
  }
  else {
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

  imageStore(gbuf_color_img, pixel, float4(out_color, obj_id));
  /* w=1 marks an exact (mesh corner-interpolated) normal: the shared
   * screen-space normal pass (sdf_normal_comp) keeps it; w=0 lets that pass
   * reconstruct the normal from the position buffer like the classic engine,
   * which does not band along the ROUND fillet fold lines the way the
   * finite-difference gradient does. */
  imageStore(gbuf_normal_img, pixel, float4(normal, normal_valid ? 1.0f : 0.0f));
}
