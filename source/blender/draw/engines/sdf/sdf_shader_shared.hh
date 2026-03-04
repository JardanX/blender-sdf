/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifndef GPU_SHADER
#  include "GPU_shader_shared_utils.hh"
#endif

/** GPU-side SDF object data. Packed for SSBO upload. */
struct SDFObjectGPU {
  /** Rotation-only inverse matrix (scale baked into sdf_size). */
  float4x4 inverse_matrix;
  /** World-space translation. */
  float4 position;
  /** SDF primitive size with object scale applied: (sx, sy, sz, 0). */
  float4 sdf_size;
  /** World AABB min. */
  float4 bbox_min;
  /** World AABB max. */
  float4 bbox_max;
  /** Bevel radius (world-space). */
  float bevel;
  /** Smooth blend radius. */
  float blend;
  float _pad1;
  float _pad2;
  /** Object color RGBA. */
  float4 color;
};
/* Total: 64 + 16 + 16 + 16 + 16 + 16 + 16 = 160 bytes, 16-byte aligned. */

/** Brick counter SSBO (used by classify pass). */
struct BrickCounter {
  uint count;
  uint _pad0;
  uint _pad1;
  uint _pad2;
};

/** Active brick coordinate entry (written by classify, read by bake/grid_blend). */
struct ActiveBrick {
  int4 coord; /* xyz = brick coordinate, w = compact atlas slot index */
};

/** BVH node for GPU-side object AABB tree traversal.
 * Interior node: left >= 0, right = right child index.
 * Leaf node:     left = -1, right = object index into objects[]. */
struct BVHNodeGPU {
  float4 min_and_left;  /* xyz = AABB min, w = intBitsToFloat(left_child or -1) */
  float4 max_and_right; /* xyz = AABB max, w = intBitsToFloat(right_child or obj_idx) */
};

/** GPU-side SDF shape descriptor (unique primitive geometry).
 * Each unique combination of (type, size, bevel) defines a shape.
 * Multiple instances can reference the same shape, sharing atlas data.
 * Used for instanced rendering (Step 3+). */
struct SDFShapeGPU {
  /** SDF primitive size (unscaled, normalized: max component = 1.0). */
  float4 size_normalized;
  /** Bevel radius (normalized relative to max size component). */
  float bevel_normalized;
  /** SDF primitive type (eSDFType). */
  int sdf_type;
  /** Index into per-shape atlas pool (-1 = not yet baked). */
  int atlas_index;
  /** World-space scale factor to map normalized shape to world. */
  float world_scale;
};
/* 16 + 16 = 32 bytes, 16-byte aligned. */

/** GPU-side SDF instance (one per scene object).
 * References a shape and adds per-instance transform + appearance. */
struct SDFInstanceGPU {
  /** World-to-local transform (maps world ray into shape's [-1,1]^3 space). */
  float4x4 world_to_local;
  /** Local-to-world transform (maps shape-space positions to world). */
  float4x4 local_to_world;
  /** Per-instance display color RGBA. */
  float4 color;
  /** Smooth blend radius (world-space). */
  float blend;
  /** Index into shapes[] array. */
  int shape_id;
  /** Original object index in objects[] (for selection/debug). */
  int object_id;
  float _pad0;
};
/* 64 + 64 + 16 + 16 = 160 bytes, 16-byte aligned. */

/** Push constants for the classify compute shader. */
struct SDFClassifyParams {
  float4 atlas_origin;
  int4 grid_resolution;
  float voxel_size;
  int object_count;
  float brick_half_diag;
  float _pad0;
};

/** Push constants for the bake compute shader. */
struct SDFBakeParams {
  float4 atlas_origin;
  int4 grid_resolution;
  float voxel_size;
  int object_count;
  int bricks_per_axis;
  float _pad0;
};

/** Push constants for the ray-march fragment shader. */
struct SDFMarchParams {
  float4 atlas_origin;
  float4 atlas_extent;
  int4 grid_resolution;
  float voxel_size;
  int object_count;
  int bricks_per_axis;
  int debug_grid;
};
