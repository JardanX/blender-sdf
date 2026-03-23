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
  float shell_blend_top;
  float shell_blend_bottom;
  float chamfer_k2;
  float chamfer_k3;
  int modifier_start;
  int modifier_count;
  int group_id;     /* -1 = ungrouped */
  int group_first;  /* 1 = base shape in group */
  int group_order;
  int original_index;
  int polygon_point_start;
  int polygon_point_count;
  int _pad2; /* scratch: visibility flag */
  int _pad3; /* scratch: max group blend (intBitsToFloat) */
  int _pad4;
  int _pad5;
  int _pad6; /* Align float4 color to 16 bytes (24 scalars × 4 = 96; 128+96=224=14×16) */
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
  float shell_blend_top;
  float shell_blend_bottom;
  float chamfer_k2;
  float chamfer_k3;
  int first_object;
  int object_count;
  int _pad0;
  float4 color;
};

/* GPU-side SDF modifier */
struct SDFModifierGPU {
  int4 header; /* x=type, y=flags */
  float4 params;
  float4 params2;
};

/* Per-edge precomputed polygon data (CPU → GPU) */
struct SDFPolygonPointGPU {
  float4 vi_edge;    /* vi.x, vi.y, edge.x, edge.y  (edge = vj - vi) */
  float4 arc_data;   /* R_signed, C.x, C.y, t_trim_start */
  float4 arc_bounds; /* t_trim_end, ang_mid, ang_half_span, flags */
};

struct BVHNodeGPU {
  float4 min_and_left;  /* xyz=AABB min, w=left_child or -1 */
  float4 max_and_right; /* xyz=AABB max, w=right_child or obj_idx */
};

struct SdfAabbNodeGPU {
  float4 bounds_min;
  float4 bounds_max;
  int parent;
  int child_a;
  int child_b;
  int shape_index;
};

/* Dual Contouring output vertex */
struct DCVertexGPU {
  float4 position; /* xyz = position, w = unused */
  float4 normal;   /* xyz = normal, w = unused */
};

