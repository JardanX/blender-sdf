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
static constexpr int SDF_CHUNK_BRICK_RES = 16;
static constexpr int SDF_CHUNK_BRICK_COUNT = SDF_CHUNK_BRICK_RES * SDF_CHUNK_BRICK_RES *
                                             SDF_CHUNK_BRICK_RES;

}  // namespace blender::draw::sdf
