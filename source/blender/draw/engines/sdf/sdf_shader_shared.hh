/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifndef GPU_SHADER
#  include "GPU_shader_shared_utils.hh"
#endif

/* GPU-side SDF object data */
struct SDFObjectGPU {
  float4x4 inverse_matrix; /* Rotation-only (scale baked into sdf_size) */
  float4 position;
  float4 sdf_size;
  float4 bbox_min;
  float4 bbox_max;
  float bevel;
  float blend;
  int sdf_type;
  int blend_type;
  int csg_operation;
  float shell_distance;
  int modifier_start;
  int modifier_count;
  int group_id;     /* -1 = ungrouped */
  int group_first;  /* 1 = base shape in group */
  int group_order;
  int original_index;
  float4 color;
  float4 box_corners;
  float4 box_edges;   /* x=top, y=bottom, z=tapTop, w=tapBot */
  int4 box_modes;     /* x=corner_mode, y=edge_mode, z=ngon_sides, w=0 */
};

/* GPU-side SDF group */
struct SDFGroupGPU {
  int csg_operation;
  int blend_type;
  float blend;
  float shell_distance;
  int first_object;
  int object_count;
  float _pad0;
  float _pad1;
  float4 color;
};

/* GPU-side SDF modifier */
struct SDFModifierGPU {
  int4 header; /* x=type, y=flags */
  float4 params;
  float4 params2;
};

struct BrickCounter {
  uint count;
  uint next_slot;
  uint overflow;
  uint _pad;
};

enum eActiveBrickFlags : uint32_t {
  ACTIVE_BRICK_FLAG_NONE = 0,
  ACTIVE_BRICK_FLAG_FULL_REBAKE = 1,
};

struct ActiveBrick {
  int4 coord; /* xyz = brick coord, w = atlas slot */
  int4 meta;  /* x = eActiveBrickFlags */
};

struct ChunkPageGPU {
  int4 coord; /* xyz = chunk coord, w = unused */
};

struct ChunkHashEntryGPU {
  int4 coord; /* xyz = chunk coord, w = chunk index (-1 = empty) */
};

/* BVH node: interior (left>=0) or leaf (left=-1, right=obj_idx) */
struct BVHNodeGPU {
  float4 min_and_left;  /* xyz=AABB min, w=left_child or -1 */
  float4 max_and_right; /* xyz=AABB max, w=right_child or obj_idx */
};

