/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Contrast-Adaptive Sharpening (AMD CAS / FSR RCAS style). Sharpens the upscaled
 * image while limiting overshoot in high-contrast regions to avoid halos. */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_sharpen)

float4 tap(int2 p, int2 maxc)
{
  return texelFetch(color_tx, clamp(p, int2(0), maxc), 0);
}

void main()
{
  int2 maxc = tex_size - 1;
  int2 p = clamp(int2(screen_uv * float2(tex_size)), int2(0), maxc);

  float4 ec = tap(p, maxc);

  /* 3x3 neighborhood. */
  float3 a = tap(p + int2(-1, -1), maxc).rgb;
  float3 b = tap(p + int2(0, -1), maxc).rgb;
  float3 c = tap(p + int2(1, -1), maxc).rgb;
  float3 d = tap(p + int2(-1, 0), maxc).rgb;
  float3 e = ec.rgb;
  float3 f = tap(p + int2(1, 0), maxc).rgb;
  float3 g = tap(p + int2(-1, 1), maxc).rgb;
  float3 h = tap(p + int2(0, 1), maxc).rgb;
  float3 i = tap(p + int2(1, 1), maxc).rgb;

  float3 mn = min(min(min(d, e), min(f, b)), h);
  mn += min(mn, min(min(a, c), min(g, i)));
  float3 mx = max(max(max(d, e), max(f, b)), h);
  mx += max(mx, max(max(a, c), max(g, i)));

  float3 rcp_mx = 1.0f / max(mx, float3(1e-4f));
  float3 amp = sqrt(clamp(min(mn, 2.0f - mx) * rcp_mx, 0.0f, 1.0f));

  /* sharpness 0..1 maps lobe from gentle (-1/8) to strong (-1/5). */
  float peak = -1.0f / mix(8.0f, 5.0f, clamp(sharpness, 0.0f, 1.0f));
  float3 w = amp * peak;
  float3 res = ((b + d + f + h) * w + e) / (1.0f + 4.0f * w);

  out_color = float4(res, ec.a);
}
