/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Shade pass: multi-tap bilateral screen-space normals + lighting.
 * Sums Gaussian-weighted central differences at offsets 1-4 pixels,
 * with depth-aware bilateral gating to preserve silhouette edges.
 * Smooths over sphere march epsilon-step banding in the position buffer. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_shade_comp)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

void main()
{
  int2 pixel = ivec2(gl_GlobalInvocationID.xy);
  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float4 gbuf = imageLoad(gbuf_pos_img, pixel);
  if (gbuf.w == 0.0f) {
    imageStore(out_color_img, pixel, float4(0.0));
    imageStore(out_depth_img, pixel, float4(0.0));
    return;
  }

  float3 hit_pos = gbuf.xyz;
  float3 hit_color = imageLoad(gbuf_color_img, pixel).rgb;

  /* Multi-tap bilateral Gaussian derivative.
   * Spatial weights: Gaussian with sigma ~2.5 pixels.
   * Bilateral gate: reject neighbors with depth jump > 5% of center depth. */
  float3 cam_pos = drw_view().viewinv[3].xyz;
  float depth0 = length(hit_pos - cam_pos);
  float disc = depth0 * 0.05f;

  /* Gaussian spatial weights for offsets 1,2,3,4 (sigma=2.5) */
  float gs[4] = float[4](0.38f, 0.30f, 0.20f, 0.12f);

  float3 dx = float3(0.0f);
  float3 dy = float3(0.0f);
  float wx = 0.0f;
  float wy = 0.0f;

  for (int k = 1; k <= 4; k++) {
    float ws = gs[k - 1];
    float inv_span = 1.0f / float(2 * k);

    float4 pr = imageLoad(gbuf_pos_img, pixel + int2(k, 0));
    float4 pl = imageLoad(gbuf_pos_img, pixel + int2(-k, 0));
    float4 pu = imageLoad(gbuf_pos_img, pixel + int2(0, k));
    float4 pd = imageLoad(gbuf_pos_img, pixel + int2(0, -k));

    float dr = (pr.w > 0.0f) ? abs(length(pr.xyz - cam_pos) - depth0) : 1e10f;
    float dl = (pl.w > 0.0f) ? abs(length(pl.xyz - cam_pos) - depth0) : 1e10f;
    float du = (pu.w > 0.0f) ? abs(length(pu.xyz - cam_pos) - depth0) : 1e10f;
    float dd = (pd.w > 0.0f) ? abs(length(pd.xyz - cam_pos) - depth0) : 1e10f;

    if (dr < disc && dl < disc) {
      dx += ws * (pr.xyz - pl.xyz) * inv_span;
      wx += ws;
    }
    if (du < disc && dd < disc) {
      dy += ws * (pu.xyz - pd.xyz) * inv_span;
      wy += ws;
    }
  }

  float3 normal;
  if (wx > 0.001f && wy > 0.001f) {
    dx /= wx;
    dy /= wy;
    normal = normalize(cross(dx, dy));
  }
  else {
    normal = float3(0.0f, 0.0f, 1.0f);
  }

  if (any(isnan(normal)) || dot(normal, normal) < 0.5f) {
    normal = float3(0.0f, 0.0f, 1.0f);
  }

  /* Ensure normal faces camera */
  float3 view_dir = normalize(hit_pos - cam_pos);
  if (dot(normal, view_dir) > 0.0f) {
    normal = -normal;
  }

  /* Shading */
  ViewMatrices vm = drw_view();
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

  imageStore(out_color_img, pixel, float4(shaded_color, 1.0f));
  imageStore(out_depth_img, pixel, float4(drw_point_world_to_screen(hit_pos).z));
}
