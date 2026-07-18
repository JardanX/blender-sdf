/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "GPU_shader_shared_utils.hh"

#define SDF_GPU_TYPE_BOX 0
#define SDF_GPU_TYPE_SPHERE 1
#define SDF_GPU_TYPE_CYLINDER 2
#define SDF_GPU_TYPE_CONE 3
#define SDF_GPU_TYPE_CAPSULE 4
#define SDF_GPU_TYPE_TORUS 5
#define SDF_GPU_TYPE_NGON 6
#define SDF_GPU_TYPE_POLYGON 7
#define SDF_GPU_TYPE_MESH 8
#define SDF_GPU_TYPE_GROUP 100

struct [[host_shared]] SDFObjectGPU {
  float4x4 inverse_matrix;
  float4 position;
  /* xyz = per-axis object scale (applied as coordinate transform), w = min(scale). */
  float4 obj_scale;
  float4 sdf_size;
  float4 bbox_min;
  float4 bbox_max;
  float bevel;
  float blend;
  float clearance;
  float color_blend;
  int sdf_type;
  int blend_type;
  int color_blend_type;
  int csg_operation;
  float shell_distance;
  int shell_mode;
  int shell_op;
  float shell_blend_top;
  float shell_blend_bottom;
  float chamfer_k2;
  float chamfer_k3;
  float chamfer_k4;
  float chamfer_k5;
  int flip_blend;
  int flip_blend_end;
  int modifier_start;
  int modifier_count;
  int group_id;
  int original_index;
  int polygon_point_start;
  int polygon_point_count;
  int _pad2;
  float max_group_blend;
  int _pad3;
  float4 color;
  float4 box_corners;
  float4 box_edges;
  int4 box_modes;
  float4 orig_bbox_min;
  float4 orig_bbox_max;
  float4 mesh_bounds_min;
  float4 mesh_bounds_max;
  /* vertex start, triangle start, triangle count, BVH node start. */
  int4 mesh_data;
  /* normal mode, flags, node count, data version. */
  int4 mesh_settings;
};
BLI_STATIC_ASSERT_ALIGN(SDFObjectGPU, 16)
BLI_STATIC_ASSERT(sizeof(SDFObjectGPU) == 416, "SDFObjectGPU size mismatch")

struct [[host_shared]] SDFObjectAABB {
  float4 bbox_min;
  float4 bbox_max;
  int group_id;
  float max_group_blend;
  int _pad0;
  int _pad1;
};
BLI_STATIC_ASSERT_ALIGN(SDFObjectAABB, 16)

struct [[host_shared]] SDFGroupGPU {
  int csg_operation;
  int blend_type;
  float blend;
  float clearance;
  float color_blend;
  int color_blend_type;
  float shell_distance;
  int shell_mode;
  int shell_op;
  float shell_blend_top;
  float shell_blend_bottom;
  float chamfer_k2;
  float chamfer_k3;
  float chamfer_k4;
  float chamfer_k5;
  int flip_blend;
  int flip_blend_end;
  int first_object;
  int object_count;
  int modifier_start;
  int modifier_count;
  int _pad0;
  int _pad1;
  int _pad2;
  float4 color;
};
BLI_STATIC_ASSERT_ALIGN(SDFGroupGPU, 16)

struct [[host_shared]] SDFModifierGPU {
  int4 header;
  float4 params;
  float4 params2;
};
BLI_STATIC_ASSERT_ALIGN(SDFModifierGPU, 16)

struct [[host_shared]] SDFPolygonPointGPU {
  float4 vi_edge;
  float4 arc_data;
  float4 arc_bounds;
};
BLI_STATIC_ASSERT_ALIGN(SDFPolygonPointGPU, 16)

struct [[host_shared]] BVHNodeGPU {
  float4 min_and_left;
  float4 max_and_right;
};
BLI_STATIC_ASSERT_ALIGN(BVHNodeGPU, 16)
BLI_STATIC_ASSERT(sizeof(BVHNodeGPU) == 32, "BVHNodeGPU size mismatch")

struct [[host_shared]] SDFMeshTriangleGPU {
  uint4 vertices_and_material;
  uint4 corner_normals;
  uint4 edge_normals;
};
BLI_STATIC_ASSERT_ALIGN(SDFMeshTriangleGPU, 16)
BLI_STATIC_ASSERT(sizeof(SDFMeshTriangleGPU) == 48, "SDFMeshTriangleGPU size mismatch")

struct [[host_shared]] SdfAabbNodeGPU {
  float4 bounds_min;
  float4 bounds_max;
  int parent;
  int child_a;
  int child_b;
  int shape_index;
};
BLI_STATIC_ASSERT_ALIGN(SdfAabbNodeGPU, 16)

struct [[host_shared]] DCVertexGPU {
  float4 position;
  float4 normal;
};
BLI_STATIC_ASSERT_ALIGN(DCVertexGPU, 16)

struct [[host_shared]] SDFShadingDataGPU {
  float4 studio_light[4];
  float4 studio_color[4];
  float4 studio_spec[4];
  float4 studio_ambient;
};
BLI_STATIC_ASSERT_ALIGN(SDFShadingDataGPU, 16)
