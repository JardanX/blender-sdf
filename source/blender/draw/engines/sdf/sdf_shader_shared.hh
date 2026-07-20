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

/* Mirrors eSDFMeshFlags (DNA_sdf_types.h); prefixed to avoid colliding with
 * the DNA enum member of the same name in host code. */
#define SDF_LP_MESH_FLAG_CLOSED (1 << 0)
#define SDF_LP_MESH_FLAG_ORIENTED (1 << 1)
#define SDF_LP_MESH_FLAG_SMOOTH_NORMALS (1 << 4)

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

/* -------------------------------------------------------------------- */
/** \name Lipschitz Pruning (per-cell pruned CSG trees)
 * \{ */

#define SDF_LP_NODETYPE_BINARY 0
#define SDF_LP_NODETYPE_PRIMITIVE 1
#define SDF_LP_NODETYPE_OFFSET 2

/* CSG op ids (must match eSDFCSGOperation in DNA_sdf_types.h). SHELL/PUSH/AVOID
 * never appear in serialized binary ops: the tree builder desugars them into
 * UNION/SUBTRACT/INTERSECT + OFFSET nodes so the dominance-based culling
 * applies to them too. Only PAINT stays opaque (geometry = left operand). */
#define SDF_LP_CSG_UNION 0
#define SDF_LP_CSG_SUBTRACT 1
#define SDF_LP_CSG_INTERSECT 2
#define SDF_LP_CSG_SHELL 3
#define SDF_LP_CSG_PUSH 4
#define SDF_LP_CSG_AVOID 5
#define SDF_LP_CSG_PAINT 6

/* Blend type ids (must match eSDFBlendType in DNA_sdf_types.h). */
#define SDF_LP_BLEND_LINEAR 0
#define SDF_LP_BLEND_SMOOTH 1
#define SDF_LP_BLEND_CHAMFER 2
#define SDF_LP_BLEND_ROUND 3

/* Binary op packing: one uint4 per op in lp_binary_ops[].
 * x = op_word:
 *   bit 0     = sign s of the min() form (+1 union, -1 subtract/intersect),
 *   bits 3..1 = SDF_LP_CSG_*,
 *   bits 5..4 = SDF_LP_BLEND_*,
 *   bits 31..6 = float bits of the blend radius k (low 6 mantissa bits
 *   cleared). For LINEAR (or blend <= 0) k is packed as 0.
 * y = float bits of k2 (start-edge softness of the first operand),
 * z = float bits of k3 (start-edge softness of the second operand)
 *   — the classic chamfer_k2/k3 (or k4/k5 for the SHELL end-edge op);
 *   only meaningful for CHAMFER/ROUND with k > 0, packed as 0 otherwise.
 * w = flags (SDF_LP_OP_FLAG_*). */
#define SDF_LP_OP_KMASK 0xFFFFFFC0u

/* Op flag: select the classic "Inverted" function variants used by the
 * SHELL desugar (sdf_lib.glsl combineCSG shell branch):
 * - SUBTRACT + ROUND: opIntersectionRound(d1, -d2, k) (inward shell start
 *   edge, non-flipped — NOT the opRoundSubtraction duality form);
 * - INTERSECT + ROUND + k2/k3: opSmoothRoundIntersectionInverted (outward
 *   shell end edge, flip_blend_end). */
#define SDF_LP_OP_FLAG_INVERTED 1u

/* Invalid parent index marker for SDFLp parent arrays. */
#define SDF_LP_INVALID_INDEX 0xFFFFFFFFu
/* Sign bit of an SDFLp active node entry (index in the low 31 bits). */
#define SDF_LP_SIGN_BIT 0x80000000u
/* Cell num_active sentinel: the cell's list did not fit the dynamic pools
 * during pruning; the trace pass evaluates the full tree for these cells.
 * Always exact geometry — overflow degrades to slower tracing locally. */
#define SDF_LP_FALLBACK_LIST (-1)
/* lp_stats slots (indices into the combined lp_counters buffer: slots 0-15
 * are the per-level active-list counters indexed by `counter_slot` (grid
 * level, only 2/4/6/8 used), slots 16-31 the per-level tmp counters; 14 and
 * 15 are reserved for overflow statistics). The prune shader increments
 * these once per cell that overflows; the engine reads them back after each
 * rebuild (counters are cleared to zero before every prune dispatch). */
#define SDF_LP_STAT_ACTIVE_OVERFLOW 14
#define SDF_LP_STAT_TMP_OVERFLOW 15

/* Analytic primitive for the Lipschitz pruning engine. Mirrors the classic
 * engine transform chain: lp = (m * (p - position)) / scale.xyz,
 * d = eval(lp, size.xyz) - size.w, scaled by scale.w. Advanced variants
 * (box corner/edge/taper, capped torus, advanced ngon/polygon) carry their
 * parameters in box_corners/box_edges/box_modes (same layout as
 * SDFObjectGPU); modifiers are referenced via modifier_start/count. */
struct [[host_shared]] SDFLpPrimitive {
  /* Rows of the (rotation-only) world-to-local matrix, w components unused. */
  float4 m_row0;
  float4 m_row1;
  float4 m_row2;
  /* xyz = world position, w unused. */
  float4 position;
  /* xyz = base half-size (pre-subtracted bevel), w = effective bevel. */
  float4 size;
  /* xyz = per-axis coordinate scale, w = distance scale (min axis scale). */
  float4 scale;
  /* rgb = albedo, w = sorted object index (for overlays/picking). */
  float4 color;
  /* SDF_GPU_TYPE_* (all analytic shapes + MESH). */
  int type;
  /* NGON: side count. POLYGON: polygon_points start. */
  int aux0;
  /* POLYGON: polygon_points count. */
  int aux1;
  /* POLYGON: corner rounding radius (0 = sharp corners). */
  float auxf;
  /* MESH: vertex start, triangle start, triangle count, BVH node start
   * (mirrors SDFObjectGPU.mesh_data; unused otherwise). */
  int4 mesh_data;
  /* MESH: BVH node count (mirrors SDFObjectGPU.mesh_settings.z). */
  int mesh_node_count;
  /* MESH: eSDFMeshFlags (mirrors SDFObjectGPU.mesh_settings.y; unused
   * otherwise). SDF_LP_MESH_FLAG_SMOOTH_NORMALS selects corner-normal blending. */
  int mesh_flags;
  int _pad1;
  int _pad2;
  /* Advanced-variant payload (mirrors SDFObjectGPU.box_corners/box_edges/
   * box_modes; unused by basic shapes). */
  float4 box_corners;
  float4 box_edges;
  int4 box_modes;
  /* Range into the shared modifier stack (mirrors SDFObjectGPU). */
  int modifier_start;
  int modifier_count;
  /* Index into the sorted object list (source of this primitive). */
  int obj_index;
  int _pad3;
};
BLI_STATIC_ASSERT_ALIGN(SDFLpPrimitive, 16)
BLI_STATIC_ASSERT(sizeof(SDFLpPrimitive) == 224, "SDFLpPrimitive size mismatch")

/* Serialized binary op (kept for reference; the runtime format is the packed
 * uint4 produced by lp_pack_binary_op, stored in lp_binary_ops_). param0 is
 * only used by PAINT (color blend radius); all other parameters are baked
 * into the tree structure by the builder. (`packed` is a GLSL reserved word,
 * hence op_word.) */
struct [[host_shared]] SDFLpBinaryOp {
  uint op_word;
  float param0;
  float param1;
  float param2;
};
BLI_STATIC_ASSERT_ALIGN(SDFLpBinaryOp, 16)
BLI_STATIC_ASSERT(sizeof(SDFLpBinaryOp) == 16, "SDFLpBinaryOp size mismatch")

struct [[host_shared]] SDFLpNode {
  /* SDF_LP_NODETYPE_*. */
  int type;
  /* Index into lp_prims or lp_binary_ops depending on type. */
  int idx_in_type;
  /* Conservative Lipschitz constant of this subtree's field w.r.t. position,
   * computed by the CPU builder: 1 for primitives, child constant for OFFSET,
   * and for BINARY the child constants folded with the op factor
   * (sqrt(l^2+r^2) for ROUND — the fillet arc is sqrt(2)-Lipschitz when both
   * child gradients align — max(l,r) for LINEAR/SMOOTH/CHAMFER). The prune
   * pass scales its dominance and far-field bounds with it. */
  float lipschitz;
  int _pad1;
};
BLI_STATIC_ASSERT_ALIGN(SDFLpNode, 16)

/** \} */
