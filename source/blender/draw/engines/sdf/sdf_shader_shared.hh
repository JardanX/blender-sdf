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
  int shell_mode;
  int modifier_start;
  int modifier_count;
  int group_id;     /* -1 = ungrouped */
  int group_first;  /* 1 = base shape in group */
  int group_order;
  int original_index;
  int _pad0;
  int _pad1;
  int _pad2;
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
  int shell_mode;
  int first_object;
  int object_count;
  float _pad0;
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
  uint _pad1;
  uint _pad2;
};

struct ActiveBrick {
  int4 coord; /* xyz = brick coord, w = atlas slot */
};

/* BVH node: interior (left>=0) or leaf (left=-1, right=obj_idx) */
struct BVHNodeGPU {
  float4 min_and_left;  /* xyz=AABB min, w=left_child or -1 */
  float4 max_and_right; /* xyz=AABB max, w=right_child or obj_idx */
};

/* Per-object shape descriptor for instanced mode */
struct SDFShapeGPU {
  float4 sdf_size;
  float bevel;
  int sdf_type;
  int slot_offset;
  float world_scale;
  int4 grid_params;   /* xyz=grid_res, w=indirection offset */
  float4 local_params; /* xyz=origin, w=voxel_size */
  int4 atlas_params;   /* x=bricks_per_axis, y=active_bricks, z=object_id */
};

/* Per-instance data (transform + appearance) */
struct SDFInstanceGPU {
  float4x4 world_to_local;
  float4x4 local_to_world;
  float4 color;
  float blend;
  int shape_id;
  int object_id;
  int blend_type;
  int csg_operation;
  float shell_distance;
  int shell_mode;
  int group_id;
  int group_order;
  int _inst_pad0;
  int _inst_pad1;
  int _inst_pad2;
};

struct SDFClassifyParams {
  float4 atlas_origin;
  int4 grid_resolution;
  float voxel_size;
  int object_count;
  float brick_half_diag;
  int group_count;
};

struct SDFBakeParams {
  float4 atlas_origin;
  int4 grid_resolution;
  float voxel_size;
  int object_count;
  int bricks_per_axis;
  int group_count;
};

struct SDFMarchParams {
  float4 atlas_origin;
  float4 atlas_extent;
  int4 grid_resolution;
  float voxel_size;
  int object_count;
  int bricks_per_axis;
  int group_count;
};
