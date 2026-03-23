/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_blit)

void main()
{
  float2 uv = screen_uv;

  /* Debug: outline ID visualization */
  if (debug_bvh_views == 6) {
    float mask = texture(outline_mask_debug_tx, uv).r;
    out_color = float4(mask, mask * 0.8f, 0.0f, 1.0f);
    gl_FragDepth = 0.5f;
    return;
  }

  out_color = texture(color_tx, uv);
  float depth = texture(depth_tx, uv).r;
  if (depth == 0.0f && debug_bvh_views == 0) {
    discard;
  }

  gl_FragDepth = depth;
}
