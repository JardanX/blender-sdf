/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_blit)

void main()
{
  float2 uv = screen_uv;
  float4 col = texture(color_tx, uv);
  float depth = texture(depth_tx, uv).r;
  if ((depth == 0.0f || col.a < 0.5f) && debug_bvh_views == 0) {
    discard;
  }
  out_color = col;
  gl_FragDepth = depth;
}
