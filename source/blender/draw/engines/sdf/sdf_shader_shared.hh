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
  /** SDF primitive type (eSDFType: 0=box, 1=sphere, 2=cylinder, 3=cone, 4=capsule, 5=torus). */
  int sdf_type;
  /** Blend function type (eSDFBlendType: 0=linear, 1=smooth, 2=chamfer, 3=round). */
  int blend_type;
  /** CSG operation (eSDFCSGOperation: 0=union, 1=subtract, 2=intersect, 3=shell). */
  int csg_operation;
  /** Shell/extrusion thickness (world-space, from DNA shell_distance). */
  float shell_distance;
  /** Index of first modifier in the modifier SSBO. */
  int modifier_start;
  /** Number of modifiers for this object. */
  int modifier_count;
  /** Object color RGBA. */
  float4 color;
  /** Box per-corner bevel radii (normalized 0–1). */
  float4 box_corners;
  /** Box edge chamfer: x=top, y=bottom, z=tapTop, w=tapBot. */
  float4 box_edges;
  /** Box modes: x=corner_mode (0=smooth,1=chamfer), y=edge_mode, z=0, w=0. */
  int4 box_modes;
};
/* Total: 64 + 16*5 + 16 + 16*3 = 64+80+16+48 = 208 bytes -> needs recount */

/** GPU-side SDF modifier. Flat data for SSBO upload. 32 bytes, 16-byte aligned. */
struct SDFModifierGPU {
  /** x=type, y=flags, z=0, w=0 */
  int4 header;
  /** Modifier-specific parameters. */
  float4 params;
};

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
 * Used for instanced rendering. */
struct SDFShapeGPU {
  /** SDF primitive size (unscaled, normalized: max component = 1.0). */
  float4 size_normalized;
  /** Bevel radius (normalized relative to max size component). */
  float bevel_normalized;
  /** SDF primitive type (eSDFType). */
  int sdf_type;
  /** Slot offset in the shared compact atlas (first brick slot for this shape). */
  int slot_offset;
  /** World-space scale factor to map normalized shape to world. */
  float world_scale;
  /** Per-shape grid: xyz = grid_res per axis, w = offset into shape_indirection SSBO. */
  int4 grid_params;
  /** Local-space atlas: xyz = local origin, w = local_voxel_size. */
  float4 local_params;
  /** Atlas layout: x = global bricks_per_axis, y = active_brick_count, zw = 0. */
  int4 atlas_params;
};
/* 16 + 16 + 16 + 16 + 16 = 80 bytes, 16-byte aligned. */

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
  /** Blend function type (eSDFBlendType). */
  int blend_type;
  /** CSG operation (eSDFCSGOperation). */
  int csg_operation;
  float _pad0;
  float _pad1;
  float _pad2;
};
/* 64 + 64 + 16 + 16 + 16 = 176 bytes, 16-byte aligned. */

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
