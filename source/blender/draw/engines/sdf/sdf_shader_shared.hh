/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifndef GPU_SHADER
#  include "GPU_shader_shared_utils.hh"
#endif

struct [[host_shared]] SDFObjectGPU {
  float4x4 inverse_matrix;
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
  int shell_op;
  float shell_blend_top;
  float shell_blend_bottom;
  float chamfer_k2;
  float chamfer_k3;
  int modifier_start;
  int modifier_count;
  int group_id;
  int group_first;
  int group_order;
  int original_index;
  int polygon_point_start;
  int polygon_point_count;
  int _pad2;
  int _pad3;
  int _pad4;
  int _pad5;
  float4 color;
  float4 box_corners;
  float4 box_edges;
  int4 box_modes;
};

struct [[host_shared]] SDFObjectAABB {
  float4 bbox_min;
  float4 bbox_max;
  int group_id;
  float max_group_blend;
  int _pad0;
  int _pad1;
};

struct [[host_shared]] SDFGroupGPU {
  int csg_operation;
  int blend_type;
  float blend;
  float shell_distance;
  int shell_mode;
  int shell_op;
  float shell_blend_top;
  float shell_blend_bottom;
  float chamfer_k2;
  float chamfer_k3;
  int first_object;
  int object_count;
  float4 color;
};

struct [[host_shared]] SDFModifierGPU {
  int4 header;
  float4 params;
  float4 params2;
};

struct [[host_shared]] SDFPolygonPointGPU {
  float4 vi_edge;
  float4 arc_data;
  float4 arc_bounds;
};

struct [[host_shared]] BVHNodeGPU {
  float4 min_and_left;
  float4 max_and_right;
};

struct [[host_shared]] SdfAabbNodeGPU {
  float4 bounds_min;
  float4 bounds_max;
  int parent;
  int child_a;
  int child_b;
  int shape_index;
};

struct [[host_shared]] DCVertexGPU {
  float4 position;
  float4 normal;
};
