/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Shade pass: best-fit triangle normals + lighting from G-buffer. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_shade_comp)

#include "draw_view_lib.glsl"

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

  /* Normal reconstruction from position G-buffer.
   * Central differences on smooth surfaces (no left/right ambiguity — fixes
   * crosshatch in orthographic view). Best-fit triangle at discontinuities
   * (concave edges, silhouettes) to avoid crease artifacts. */
  float3 p0 = hit_pos;
  float3 view_fwd = -normalize(drw_view().viewinv[2].xyz);
  float d0 = dot(p0, view_fwd);

  float4 raw_r = imageLoad(gbuf_pos_img, pixel + int2(1, 0));
  float4 raw_l = imageLoad(gbuf_pos_img, pixel + int2(-1, 0));
  float4 raw_u = imageLoad(gbuf_pos_img, pixel + int2(0, 1));
  float4 raw_d = imageLoad(gbuf_pos_img, pixel + int2(0, -1));

  bool r_valid = raw_r.w > 0.0f;
  bool l_valid = raw_l.w > 0.0f;
  bool u_valid = raw_u.w > 0.0f;
  bool d_valid = raw_d.w > 0.0f;

  float dr = r_valid ? dot(raw_r.xyz, view_fwd) : d0;
  float dl = l_valid ? dot(raw_l.xyz, view_fwd) : d0;
  float du = u_valid ? dot(raw_u.xyz, view_fwd) : d0;
  float dd = d_valid ? dot(raw_d.xyz, view_fwd) : d0;

  float3 normal;
  bool all_valid = r_valid && l_valid && u_valid && d_valid;

  if (all_valid) {
    float4 deltas = abs(float4(dr, dl, du, dd) - d0);
    float max_d = max(max(deltas.x, deltas.y), max(deltas.z, deltas.w));
    float min_d = min(min(deltas.x, deltas.y), min(deltas.z, deltas.w));
    bool is_edge = max_d > min_d * 4.0f + sdf_ray_epsilon;

    if (!is_edge) {
      /* Smooth: central differences */
      float3 hDeriv = (raw_r.xyz - raw_l.xyz) * 0.5f;
      float3 vDeriv = (raw_u.xyz - raw_d.xyz) * 0.5f;
      normal = normalize(cross(hDeriv, vDeriv));
    }
    else {
      /* Edge: best-fit triangle (pick most coplanar pair) */
      float4 tri = float4(
          max(deltas.x, deltas.z),
          max(deltas.x, deltas.w),
          max(deltas.y, deltas.z),
          max(deltas.y, deltas.w));
      float best = min(min(tri.x, tri.y), min(tri.z, tri.w));

      float3 e1, e2;
      if (tri.x == best) {
        e1 = raw_r.xyz - p0; e2 = raw_u.xyz - p0;
      }
      else if (tri.y == best) {
        e1 = raw_d.xyz - p0; e2 = raw_r.xyz - p0;
      }
      else if (tri.z == best) {
        e1 = raw_u.xyz - p0; e2 = raw_l.xyz - p0;
      }
      else {
        e1 = raw_l.xyz - p0; e2 = raw_d.xyz - p0;
      }
      normal = normalize(cross(e1, e2));
    }
  }
  else {
    /* Missing neighbors: use whichever valid side */
    float3 hDeriv = r_valid ? (raw_r.xyz - p0) : (l_valid ? (p0 - raw_l.xyz) : float3(1, 0, 0));
    float3 vDeriv = u_valid ? (raw_u.xyz - p0) : (d_valid ? (p0 - raw_d.xyz) : float3(0, 1, 0));
    normal = normalize(cross(hDeriv, vDeriv));
  }

  if (any(isnan(normal)) || dot(normal, normal) < 0.5f) {
    normal = float3(0.0f, 0.0f, 1.0f);
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

  float alpha = all_valid ? 1.0f : 0.0f;
  imageStore(out_color_img, pixel, float4(shaded_color, alpha));
  imageStore(out_depth_img, pixel, float4(drw_point_world_to_screen(hit_pos).z));
}
