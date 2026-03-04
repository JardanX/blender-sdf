/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF FXAA post-processing fragment shader.
 * Applies NVIDIA FXAA 3.11 (max quality preset 39) to the ray-march output.
 */

#include "infos/sdf_shader_infos.hh"

#include "draw_fxaa_lib.glsl"

void main()
{
  out_color = FxaaPixelShader(
      screen_uv,    /* UV from gpu_fullscreen vertex shader */
      color_tx,     /* offscreen march result */
      rcpFrame,     /* 1.0 / viewport size */
      0.75f,        /* subpix: default filtering amount */
      0.063f,       /* edgeThreshold: overkill quality */
      0.0312f);     /* edgeThresholdMin: visible limit */
}
