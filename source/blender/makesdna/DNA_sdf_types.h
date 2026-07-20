/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 * \brief SDF (Signed Distance Field) data types.
 */

#pragma once

#include "DNA_ID.h"
#include "DNA_defs.h"

#ifdef __cplusplus
namespace blender::bke {
struct SDFRuntime;
}
#endif

namespace blender {

struct AnimData;
struct Material;
#ifdef __cplusplus
using SDFRuntimeHandle = bke::SDFRuntime;
#else
typedef struct SDFRuntimeHandle SDFRuntimeHandle;
#endif

/* Primitive types */
typedef enum eSDFType {
  SDF_TYPE_BOX = 0,
  SDF_TYPE_SPHERE = 1,
  SDF_TYPE_CYLINDER = 2,
  SDF_TYPE_CONE = 3,
  SDF_TYPE_CAPSULE = 4,
  SDF_TYPE_TORUS = 5,
  SDF_TYPE_NGON = 6,
  SDF_TYPE_POLYGON = 7,
  SDF_TYPE_MESH = 8,
  SDF_TYPE_GROUP = 100,
} eSDFType;

typedef enum eSDFMeshNormalMode {
  SDF_MESH_NORMAL_SHARP = 0,
  SDF_MESH_NORMAL_SMOOTH = 1,
} eSDFMeshNormalMode;

enum {
  SDF_MESH_FLAG_CLOSED = (1 << 0),
  SDF_MESH_FLAG_ORIENTED = (1 << 1),
  SDF_MESH_FLAG_THREADED_BVH = (1 << 2),
  SDF_MESH_FLAG_CORNER_NORMALS = (1 << 3),
  /* The base mesh shades smooth somewhere (normals domain is not Face):
   * corner normals are continuous across triangles, so normal blending uses
   * them directly. Flat meshes keep the geometric gradient in blend zones. */
  SDF_MESH_FLAG_SMOOTH_NORMALS = (1 << 4),
};

/* Box corner/edge blend mode */
typedef enum eSDFBoxMode {
  SDF_BOX_MODE_SMOOTH = 0,
  SDF_BOX_MODE_CHAMFER = 1,
} eSDFBoxMode;

typedef enum eSDFBlendType {
  SDF_BLEND_LINEAR = 0,
  SDF_BLEND_SMOOTH = 1,
  SDF_BLEND_CHAMFER = 2,
  SDF_BLEND_ROUND = 3,
} eSDFBlendType;

typedef enum eSDFColorBlendType {
  SDF_COLOR_BLEND_RGB = 0,
  SDF_COLOR_BLEND_HUE = 1,
} eSDFColorBlendType;

typedef enum eSDFCSGOperation {
  SDF_CSG_UNION = 0,
  SDF_CSG_SUBTRACT = 1,
  SDF_CSG_INTERSECT = 2,
  SDF_CSG_SHELL = 3,
  SDF_CSG_PUSH = 4,
  SDF_CSG_AVOID = 5,
  SDF_CSG_PAINT = 6,
} eSDFCSGOperation;

typedef enum eSDFShellMode {
  SDF_SHELL_NORMAL = 0,
  SDF_SHELL_PUSH = 1,
  SDF_SHELL_AVOID = 2,
} eSDFShellMode;

typedef enum eSDFShellOp {
  SDF_SHELL_OP_UNION = 0,
  SDF_SHELL_OP_SUBTRACTION = 1,
} eSDFShellOp;

/* Modifier types */
typedef enum eSDFModifierType {
  SDF_MOD_MIRROR = 0,
  SDF_MOD_TWIST = 1,
  SDF_MOD_BEND = 2,
  SDF_MOD_ELONGATE = 3,
  SDF_MOD_SOLIDIFY = 4,
  SDF_MOD_ROUND = 5,
  SDF_MOD_ONION = 6,
  SDF_MOD_BEVEL = 7,
  SDF_MOD_ARRAY = 8,
  SDF_MOD_DISPLACE = 9,
} eSDFModifierType;

/* Mirror axis flags */
enum {
  SDF_MOD_MIRROR_X = (1 << 0),
  SDF_MOD_MIRROR_Y = (1 << 1),
  SDF_MOD_MIRROR_Z = (1 << 2),
};

/* Array modifier types */
enum {
  SDF_MOD_ARRAY_LINEAR = 0,
  SDF_MOD_ARRAY_RADIAL = 1,
};

/* Displacement noise types */
enum {
  SDF_MOD_DISPLACE_NOISE = 0,
  SDF_MOD_DISPLACE_VORONOI = 1,
  SDF_MOD_DISPLACE_TRIANGLE = 2,
  SDF_MOD_DISPLACE_POINTS = 3,
};

typedef struct SDFPolygonPoint {
  struct SDFPolygonPoint *next, *prev;
  float co[2];
  float corner;
  char _pad[4];
} SDFPolygonPoint;

typedef struct SDFModifier {
  struct SDFModifier *next, *prev;

  int type; /* eSDFModifierType */
  int flag;

  /* Per-type params layout:
   * Mirror:   [0]=offset, [4]=blend
   * Twist:    [0]=strength
   * Bend:     [0]=strength, [1]=axis, [2..4]=origin xyz
   * Elongate: [0..2]=per-axis stretch
   * Solidify: [0]=thickness
   * Expand/Shrink: [0]=offset
   * Onion:    [0]=thickness
   * Array:    [0]=count, [1..3]=offset/radius, [4]=blend, [5..7]=rotation offset */
  float params[8];

  char name[64];
  short show_viewport;
  char _pad[6];

  struct Object *mirror_ob;
} SDFModifier;

typedef struct SDFMeshVertex {
  float co[3];
  unsigned int pseudonormal;
} SDFMeshVertex;

typedef struct SDFMeshTriangle {
  unsigned int vertices[3];
  int material_index;
  unsigned int corner_normals[3];
  unsigned int _pad0;
  unsigned int edge_normals[3];
  unsigned int _pad1;
} SDFMeshTriangle;

typedef struct SDFMeshBVHNode {
  float bounds_min[3];
  int child_or_first;
  float bounds_max[3];
  int child_or_count;
} SDFMeshBVHNode;

typedef struct SDF {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(SDF)
  static constexpr ID_Type id_type = ID_SF;
#endif

  ID id;
  struct AnimData *adt = nullptr;

  int sdf_type = 0; /* eSDFType */
  char _pad0[4] = {};

  float size[3] = {1.0f, 1.0f, 1.0f};
  float bevel = 0.0f;
  float color[4] = {0.8f, 0.8f, 0.8f, 1.0f};

  float blend = 0.1f;
  int blend_type = 1;    /* eSDFBlendType: SDF_BLEND_SMOOTH */
  int csg_operation = 0; /* eSDFCSGOperation */
  float clearance = 0.0f;
  float color_blend = 0.1f;
  int color_blend_type = 0; /* eSDFColorBlendType */
  char _pad_color_blend[4] = {};
  float shell_distance = 0.2f;
  int shell_mode = 0; /* eSDFShellMode */
  int shell_op = 0;   /* eSDFShellOp */
  float shell_blend_top = 0.1f;
  float shell_blend_bottom = 0.1f;
  float chamfer_k2 = 0.001f;
  float chamfer_k3 = 0.001f;
  float chamfer_k4 = 0.01f;
  float chamfer_k5 = 0.01f;
  int flip_blend = 0;
  int flip_blend_end = 0;

  float box_corners[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float box_edge_top = 0.0f;
  float box_edge_bottom = 0.0f;
  float box_taper = 0.0f;
  int box_corner_mode = 0; /* eSDFBoxMode */
  int box_edge_mode = 0;   /* eSDFBoxMode */

  int ngon_sides = 6;
  float ngon_corner = 0.0f;
  float ngon_edge_top = 0.0f;
  float ngon_edge_bottom = 0.0f;
  float ngon_taper = 0.0f;
  int ngon_edge_mode = 0; /* eSDFBoxMode */
  float ngon_star = 0.0f;

  ListBase polygon_points = {}; /* SDFPolygonPoint */
  int totpolygon = 0;
  float polygon_edge_top = 0.0f;
  float polygon_edge_bottom = 0.0f;
  float polygon_taper = 0.0f;
  int polygon_edge_mode = 0; /* eSDFBoxMode */
  int polygon_is_line = 0;
  float polygon_line_thickness = 0.1f;
  char _pad5[4] = {};

  float torus_angle = 6.2831853f;
  char _pad4[4] = {};

  int sdf_index = 0;
  char _pad_sg[4] = {};

  ListBase modifiers = {}; /* SDFModifier */
  int totmodifier = 0;
  char _pad3[4] = {};

  SDFMeshVertex *mesh_vertices = nullptr;
  SDFMeshTriangle *mesh_triangles = nullptr;
  SDFMeshBVHNode *mesh_bvh_nodes = nullptr;
  /* Packed RGBA8 (scene-linear, alpha=255) per triangle corner, 3 * mesh_triangle_count
   * elements, reordered identically to mesh_triangles. Null when the mesh has no active
   * color attribute. */
  unsigned int *mesh_corner_colors = nullptr;
  int mesh_vertex_count = 0;
  int mesh_triangle_count = 0;
  int mesh_bvh_node_count = 0;
  int mesh_normal_mode = 0; /* eSDFMeshNormalMode */
  float mesh_bounds_min[3] = {};
  int mesh_flags = 0;
  float mesh_bounds_max[3] = {};
  int mesh_data_version = 0;

  struct Material **mat = nullptr;
  short totcol = 0;
  char _pad2[6] = {};

  SDFRuntimeHandle *runtime = nullptr;
} SDF;


}  // namespace blender
