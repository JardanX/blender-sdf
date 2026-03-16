/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine internals.
 */

#pragma once

#include "DRW_render.hh"
#include "GPU_shader.hh"
#include "draw_pass.hh"

#include "sdf_shader_shared.hh"

namespace blender::draw::sdf {

static constexpr int SDF_BRICK_SIZE = 8;
static constexpr int SDF_BRICK_STORAGE = 12; /* 8 inner + 2 overlap each side */
static constexpr int SDF_MAX_BRICKS = 8192;
static constexpr int SDF_MAX_GRID_RES = 128;

}  // namespace blender::draw::sdf
