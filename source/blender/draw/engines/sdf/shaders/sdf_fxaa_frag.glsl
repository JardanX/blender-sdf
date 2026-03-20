/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF FXAA post-processing fragment shader.
 * Quality preset 29 (12 search steps) with sRGB-aware luma.
 */

#include "infos/sdf_shader_infos.hh"

#include "sdf_fxaa_lib.glsl"

void main()
{
  out_color = FxaaPixelShader(
      screen_uv,
      color_tx,
      rcpFrame,
      0.75,
      0.063,
      0.0312);
}
