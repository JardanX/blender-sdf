/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Shade pass: best-fit triangle normals + lighting from G-buffer. */

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

  /* Best-fit triangle normal: pick the closer neighbor in each axis to avoid
   * artifacts at depth discontinuities (silhouettes, CSG edges). */
  float3 p0 = hit_pos;
  float4 raw_r = imageLoad(gbuf_pos_img, pixel + int2(1, 0));
  float4 raw_l = imageLoad(gbuf_pos_img, pixel + int2(-1, 0));
  float4 raw_u = imageLoad(gbuf_pos_img, pixel + int2(0, 1));
  float4 raw_d = imageLoad(gbuf_pos_img, pixel + int2(0, -1));

  float3 cam_pos = drw_view().viewinv[3].xyz;
  float depth0 = length(p0 - cam_pos);
  float depth_r = (raw_r.w > 0.0f) ? length(raw_r.xyz - cam_pos) : 1e10f;
  float depth_l = (raw_l.w > 0.0f) ? length(raw_l.xyz - cam_pos) : 1e10f;
  float depth_u = (raw_u.w > 0.0f) ? length(raw_u.xyz - cam_pos) : 1e10f;
  float depth_d = (raw_d.w > 0.0f) ? length(raw_d.xyz - cam_pos) : 1e10f;

  bool use_right = abs(depth_r - depth0) < abs(depth_l - depth0);
  bool use_up = abs(depth_u - depth0) < abs(depth_d - depth0);

  float3 h = use_right ? raw_r.xyz : raw_l.xyz;
  float3 v = use_up ? raw_u.xyz : raw_d.xyz;

  float3 normal;
  if (use_right && use_up)
    normal = normalize(cross(h - p0, v - p0));
  else if (use_right && !use_up)
    normal = normalize(cross(v - p0, h - p0));
  else if (!use_right && use_up)
    normal = normalize(cross(v - p0, h - p0));
  else
    normal = normalize(cross(h - p0, v - p0));

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

  imageStore(out_color_img, pixel, float4(shaded_color, 1.0f));
  imageStore(out_depth_img, pixel, float4(drw_point_world_to_screen(hit_pos).z));
}
