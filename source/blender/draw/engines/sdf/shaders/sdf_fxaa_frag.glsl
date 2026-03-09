/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF FXAA post-processing fragment shader.
 * Quality preset 25 (8 search steps) with perceptual luma.
 */

#include "infos/sdf_shader_infos.hh"

#include "sdf_fxaa_lib.glsl"

void main()
{
  out_color = FxaaPixelShader(
      screen_uv,    /* UV from gpu_fullscreen vertex shader */
      color_tx,     /* offscreen march result */
      rcpFrame,     /* 1.0 / viewport size */
      0.75,         /* subpix: default filtering amount */
      0.125,        /* edgeThreshold: good default quality */
      0.0312);      /* edgeThresholdMin: visible limit */
}
