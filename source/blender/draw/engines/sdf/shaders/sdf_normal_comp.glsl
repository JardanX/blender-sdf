/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Screen-space normal reconstruction from world-space position buffer.
 * Central differences on smooth surfaces, one-sided at discontinuities.
 * Discontinuity detection via position extrapolation. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_normal_comp)

float4 loadPos(int2 p)
{
  p = clamp(p, int2(0), screen_size - int2(1));
  return imageLoad(gbuf_pos_img, p);
}

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);
  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float4 c = loadPos(pixel);
  if (c.w < 0.5f) {
    imageStore(gbuf_normal_img, pixel, float4(0.0f));
    return;
  }

  float3 pc = c.xyz;

  float4 pl1 = loadPos(pixel + int2(-1, 0));
  float4 pr1 = loadPos(pixel + int2( 1, 0));
  float4 pd1 = loadPos(pixel + int2( 0,-1));
  float4 pu1 = loadPos(pixel + int2( 0, 1));

  float4 pl2 = loadPos(pixel + int2(-2, 0));
  float4 pr2 = loadPos(pixel + int2( 2, 0));
  float4 pd2 = loadPos(pixel + int2( 0,-2));
  float4 pu2 = loadPos(pixel + int2( 0, 2));

  float3 l = pc - pl1.xyz;
  float3 r = pr1.xyz - pc;
  float3 d = pc - pd1.xyz;
  float3 u = pu1.xyz - pc;

  bool vl = pl1.w > 0.5f && pl2.w > 0.5f;
  bool vr = pr1.w > 0.5f && pr2.w > 0.5f;
  bool vd = pd1.w > 0.5f && pd2.w > 0.5f;
  bool vu = pu1.w > 0.5f && pu2.w > 0.5f;

  float he_l = vl ? length(2.0f * pl1.xyz - pl2.xyz - pc) : 1e10f;
  float he_r = vr ? length(2.0f * pr1.xyz - pr2.xyz - pc) : 1e10f;
  float ve_d = vd ? length(2.0f * pd1.xyz - pd2.xyz - pc) : 1e10f;
  float ve_u = vu ? length(2.0f * pu1.xyz - pu2.xyz - pc) : 1e10f;

  float3 hDeriv, vDeriv;

  if (vl && vr) {
    float ratio = max(he_l, he_r) / max(min(he_l, he_r), 1e-20f);
    if (ratio > 4.0f) {
      hDeriv = he_l < he_r ? l : r;
    }
    else {
      hDeriv = (pr1.xyz - pl1.xyz) * 0.5f;
    }
  }
  else if (pr1.w > 0.5f) {
    hDeriv = r;
  }
  else if (pl1.w > 0.5f) {
    hDeriv = l;
  }
  else {
    hDeriv = float3(1.0f, 0.0f, 0.0f);
  }

  if (vd && vu) {
    float ratio = max(ve_d, ve_u) / max(min(ve_d, ve_u), 1e-20f);
    if (ratio > 4.0f) {
      vDeriv = ve_d < ve_u ? d : u;
    }
    else {
      vDeriv = (pu1.xyz - pd1.xyz) * 0.5f;
    }
  }
  else if (pu1.w > 0.5f) {
    vDeriv = u;
  }
  else if (pd1.w > 0.5f) {
    vDeriv = d;
  }
  else {
    vDeriv = float3(0.0f, 1.0f, 0.0f);
  }

  float3 n = normalize(cross(vDeriv, hDeriv));

  if (any(isnan(n))) {
    n = float3(0.0f, 0.0f, 1.0f);
  }

  imageStore(gbuf_normal_img, pixel, float4(n, 1.0f));
}
