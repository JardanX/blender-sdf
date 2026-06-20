/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_blit)

/* Catmull-Rom cubic weights for fractional offset f. */
float4 catmull_rom_weights(float f)
{
  float f2 = f * f;
  float f3 = f2 * f;
  return float4((-f3 + 2.0f * f2 - f) * 0.5f,
                (3.0f * f3 - 5.0f * f2 + 2.0f) * 0.5f,
                (-3.0f * f3 + 4.0f * f2 + f) * 0.5f,
                (f3 - f2) * 0.5f);
}

float3 sample_bicubic(float2 sp, int2 maxc)
{
  int2 base = int2(floor(sp - 0.5f));
  float2 f = sp - 0.5f - float2(base);
  float4 wx4 = catmull_rom_weights(f.x);
  float4 wy4 = catmull_rom_weights(f.y);
  float wx[4];
  float wy[4];
  wx[0] = wx4.x;
  wx[1] = wx4.y;
  wx[2] = wx4.z;
  wx[3] = wx4.w;
  wy[0] = wy4.x;
  wy[1] = wy4.y;
  wy[2] = wy4.z;
  wy[3] = wy4.w;

  float3 col = float3(0.0f);
  for (int j = 0; j < 4; j++) {
    int sy = clamp(base.y - 1 + j, 0, maxc.y);
    float3 row = float3(0.0f);
    for (int i = 0; i < 4; i++) {
      int sx = clamp(base.x - 1 + i, 0, maxc.x);
      row += texelFetch(color_tx, int2(sx, sy), 0).rgb * wx[i];
    }
    col += row * wy[j];
  }
  return col;
}

/* Joint bilateral upsample: weight low-res taps by spatial distance and by
 * similarity of geometry (plane distance, normal, object id) so color does not
 * bleed across silhouettes or material boundaries. */
float3 sample_edge_aware(float2 sp, int2 ctr, int2 maxc)
{
  float3 n_c = texelFetch(gbuf_normal_tx, ctr, 0).xyz;
  float3 p_c = texelFetch(gbuf_pos_tx, ctr, 0).xyz;
  float id_c = texelFetch(gbuf_color_tx, ctr, 0).w;
  bool use_normal = dot(n_c, n_c) > 0.25f;

  /* Local world-space size of one low-res texel, used to scale the plane test. */
  float local_scale = 0.0f;
  float4 p_r = texelFetch(gbuf_pos_tx, clamp(ctr + int2(1, 0), int2(0), maxc), 0);
  float4 p_u = texelFetch(gbuf_pos_tx, clamp(ctr + int2(0, 1), int2(0), maxc), 0);
  if (p_r.w > 0.5f) {
    local_scale = max(local_scale, length(p_r.xyz - p_c));
  }
  if (p_u.w > 0.5f) {
    local_scale = max(local_scale, length(p_u.xyz - p_c));
  }

  bool use_plane = use_normal && (local_scale > 0.0f);
  float sigma_plane = 2.0f * local_scale;

  /* Spatial term always contributes, so the filter degrades to a smooth blur
   * (it never collapses to nearest) if the geometry guides are unavailable. */
  const float sigma_s = 1.0f;
  const float normal_pow = 8.0f;

  float3 acc = float3(0.0f);
  float acc_w = 0.0f;
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      int2 t = clamp(ctr + int2(dx, dy), int2(0), maxc);
      float4 p = texelFetch(gbuf_pos_tx, t, 0);
      if (p.w < 0.5f) {
        continue;
      }
      float2 d = (float2(t) + 0.5f) - sp;
      float w = exp(-dot(d, d) / (2.0f * sigma_s * sigma_s));

      if (use_plane) {
        float dp = abs(dot(p.xyz - p_c, n_c));
        w *= exp(-(dp * dp) / (2.0f * sigma_plane * sigma_plane));
      }
      if (use_normal) {
        float3 n = texelFetch(gbuf_normal_tx, t, 0).xyz;
        w *= pow(max(dot(n, n_c), 0.0f), normal_pow);
      }
      float id = texelFetch(gbuf_color_tx, t, 0).w;
      w *= (abs(id - id_c) < 0.5f) ? 1.0f : 0.05f;

      acc += texelFetch(color_tx, t, 0).rgb * w;
      acc_w += w;
    }
  }
  return (acc_w > 1e-5f) ? acc / acc_w : texelFetch(color_tx, ctr, 0).rgb;
}

void main()
{
  float2 uv = screen_uv * uv_scale;
  int2 maxc = src_size - 1;
  float2 sp = uv * float2(tex_size);
  int2 ctr = clamp(int2(floor(sp)), int2(0), maxc);

  /* Background: decide hit/miss from the nearest valid sample. */
  float valid = texelFetch(gbuf_pos_tx, ctr, 0).w;
  float depth = texelFetch(depth_tx, ctr, 0).r;
  if (valid < 0.5f && debug_bvh_views == 0) {
    discard;
    return;
  }

  bool upscaling = (src_size.x != out_size.x) || (src_size.y != out_size.y);
  int mode = upscaling ? upscale_quality : 0;
  if (debug_bvh_views != 0) {
    mode = 0;
  }

  float3 col;
  if (mode == 0) {
    col = texelFetch(color_tx, ctr, 0).rgb;
  }
  else if (mode == 1) {
    col = texture(color_tx, uv).rgb;
  }
  else if (mode == 2) {
    col = sample_bicubic(sp, maxc);
  }
  else {
    col = sample_edge_aware(sp, ctr, maxc);
  }

  out_color = float4(col, 1.0f);
  gl_FragDepth = depth;
}
