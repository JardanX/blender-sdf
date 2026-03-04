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

/** Inner voxels per brick axis. */
static constexpr int SDF_BRICK_SIZE = 8;
/** Storage voxels per brick axis (8 inner + 2 overlap on each side for dual voxel normals). */
static constexpr int SDF_BRICK_STORAGE = 12;
/** Practical max active bricks (~25% of 32^3). */
static constexpr int SDF_MAX_BRICKS = 8192;
/** Max bricks per axis for the indirection grid. 128^3 = 2M entries (~8MB). */
static constexpr int SDF_MAX_GRID_RES = 128;
/** Max bricks per axis for per-shape local grids (instanced rendering). */
static constexpr int SDF_MAX_SHAPE_GRID_RES = 32;

}  // namespace blender::draw::sdf
