/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_edge_detect)

void main()
{
  float2 sdf_uv = screen_uv * uv_scale;
  float2 texel = 1.0f / float2(textureSize(sdf_depth_tx, 0));

  float center_depth = texture(sdf_depth_tx, sdf_uv).r;
  if (center_depth <= 0.0f || center_depth >= 1.0f) {
    discard;
    return;
  }

  float scene_d = texture(scene_depth_tx, screen_uv).r;
  float eps = max(0.0001f, scene_d * 0.005f);
  if (scene_d > 0.0f && scene_d < 1.0f && center_depth > scene_d + eps) {
    discard;
    return;
  }

  float center_id = texture(sdf_gbuf_color_tx, sdf_uv).a;

  /* Edge detection: check 4-connected neighbors for object boundary. */
  bool is_edge = false;
  float2 offsets[4] = float2[4](
      float2(-1.0f, 0.0f), float2(1.0f, 0.0f), float2(0.0f, -1.0f), float2(0.0f, 1.0f));

  for (int i = 0; i < 4; i++) {
    float2 nuv = sdf_uv + offsets[i] * texel;
    float nd = texture(sdf_depth_tx, nuv).r;
    float nid = texture(sdf_gbuf_color_tx, nuv).a;

    if (nd <= 0.0f || nd >= 1.0f || abs(nid - center_id) > 0.5f) {
      is_edge = true;
      break;
    }
  }

  if (!is_edge) {
    discard;
    return;
  }

  out_color = float4(0.0f, 0.0f, 0.0f, line_opacity);
  gl_FragDepth = center_depth;
}
