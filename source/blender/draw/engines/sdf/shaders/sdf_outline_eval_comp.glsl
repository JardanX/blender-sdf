/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Full-res outline from low-res object ID buffer.
 * Samples gbuf_color.w (object ID) at full viewport resolution using
 * nearest filtering, compares 4 cardinal neighbors. Produces pixel-perfect
 * outlines regardless of trace resolution. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_outline_eval_comp)

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);
  if (pixel.x >= viewport_size.x || pixel.y >= viewport_size.y) {
    return;
  }

  float2 uv = (float2(pixel) + 0.5f) / float2(viewport_size);
  float2 pixel_step = 1.0f / float2(viewport_size);

  float center_id = texture(gbuf_color_tx, uv).w;
  float center_valid = texture(gbuf_pos_tx, uv).w;

  /* Background: check if any neighbor has geometry */
  if (center_valid <= 0.0f) {
    float edge = 0.0f;
    float2 offsets[4] = float2[4](
        float2(pixel_step.x, 0.0f), float2(-pixel_step.x, 0.0f),
        float2(0.0f, pixel_step.y), float2(0.0f, -pixel_step.y));
    for (int i = 0; i < 4; i++) {
      if (texture(gbuf_pos_tx, uv + offsets[i]).w > 0.0f) {
        edge = 1.0f;
        break;
      }
    }
    imageStore(outline_img, pixel, float4(edge));
    return;
  }

  /* Geometry: check for ID or validity discontinuity */
  float2 offsets[4] = float2[4](
      float2(pixel_step.x, 0.0f), float2(-pixel_step.x, 0.0f),
      float2(0.0f, pixel_step.y), float2(0.0f, -pixel_step.y));

  for (int i = 0; i < 4; i++) {
    float2 n_uv = uv + offsets[i];
    float n_valid = texture(gbuf_pos_tx, n_uv).w;
    float n_id = texture(gbuf_color_tx, n_uv).w;

    if (n_valid <= 0.0f || n_id != center_id) {
      imageStore(outline_img, pixel, float4(1.0f));
      return;
    }
  }

  imageStore(outline_img, pixel, float4(0.0f));
}
