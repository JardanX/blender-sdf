/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Outline overlay: composited on top of everything with depth test off. */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_outline_overlay)

void main()
{
  float2 uv = screen_uv;
  float edge = texture(outline_tx, uv).r;

  if (edge <= 0.0f) {
    discard;
  }

  out_color = float4(0.0f, 0.0f, 0.0f, edge * outline_strength);
}
