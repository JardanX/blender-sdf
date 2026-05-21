/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 * \brief OCCT-backed NURB body data types.
 */

#pragma once

#include "DNA_ID.h"
#include "DNA_defs.h"
#include "DNA_listBase.h"

namespace blender {

struct AnimData;
struct Material;

typedef enum eNurbBodyPrimitive {
  NURB_BODY_PRIMITIVE_CYLINDER = 0,
  NURB_BODY_PRIMITIVE_BOX = 1,
  NURB_BODY_PRIMITIVE_SPHERE = 2,
  NURB_BODY_PRIMITIVE_CONE = 3,
  NURB_BODY_PRIMITIVE_TORUS = 4,
  NURB_BODY_PRIMITIVE_WEDGE = 5,
} eNurbBodyPrimitive;

typedef enum eNurbBodyBooleanOperation {
  NURB_BODY_BOOLEAN_DIFFERENCE = 0,
  NURB_BODY_BOOLEAN_UNION = 1,
  NURB_BODY_BOOLEAN_INTERSECT = 2,
  NURB_BODY_BOOLEAN_SURFACE_BLEND_STAGE = 3,
  NURB_BODY_BOOLEAN_BODY_BLEND_STAGE = 4,
  NURB_BODY_BOOLEAN_OUTPUT_BLEND_STAGE = 5,
  NURB_BODY_BOOLEAN_FACE_EXTRUDE_STAGE = 6,
  NURB_BODY_BOOLEAN_FACE_INSET_STAGE = 7,
  NURB_BODY_BOOLEAN_FACE_REFILLET_STAGE = 8,
} eNurbBodyBooleanOperation;

typedef enum eNurbBodyBevelType {
  NURB_BODY_BEVEL_FILLET = 0,
  NURB_BODY_BEVEL_CHAMFER = 1,
} eNurbBodyBevelType;

typedef enum eNurbBodySelectMode {
  NURB_BODY_SELECT_MODE_EDGE = 0,
  NURB_BODY_SELECT_MODE_FACE = 1,
  NURB_BODY_SELECT_MODE_OBJECT = 2,
} eNurbBodySelectMode;

typedef enum eNurbBodyTessellationTopology {
  NURB_BODY_TESSELLATION_TRIS = 0,
  NURB_BODY_TESSELLATION_QUADS = 1,
  NURB_BODY_TESSELLATION_NGONS = 2,
} eNurbBodyTessellationTopology;

enum {
  NURB_BODY_MERGE_VERTICES = (1 << 0),
  NURB_BODY_SMOOTH_SHADING = (1 << 1),
  NURB_BODY_TRIANGULATE_MESH = (1 << 2),
  NURB_BODY_AUTO_CREASE_SHARP_EDGES = (1 << 3),
  NURB_BODY_FAST_BEVEL_PREVIEW = (1 << 4),
};

enum {
  NURB_BODY_BOOLEAN_OP_SELECTED = (1 << 0),
  NURB_BODY_BOOLEAN_OP_HOVERED = (1 << 1),
};

typedef struct NurbBodyBooleanOp {
  struct NurbBodyBooleanOp *next, *prev;

  uint64_t selected_edges;
  uint64_t bevel_edges;
  uint64_t chamfer_edges;
  float bevel_radii[64];
  int bevel_order[64];

  int operation; /* eNurbBodyBooleanOperation */
  int flag;
  int primitive; /* eNurbBodyPrimitive */
  int selected_edge;
  int hovered_edge;
  int bevel_edge;
  int bevel_type; /* eNurbBodyBevelType */
  int bevel_order_next;

  float bevel_radius;
  float operand_radius;
  float operand_depth;
  float operand_minor_radius;
  float operand_dimensions[3];
  float _pad0;

  float operand_to_target[4][4];

  uint64_t operand_selected_edges;
  uint64_t operand_bevel_edges;
  uint64_t operand_chamfer_edges;
  uint64_t operand_surface_selected_edges;
  uint64_t operand_surface_bevel_edges;
  uint64_t operand_surface_chamfer_edges;
  float operand_bevel_radii[64];
  float operand_surface_bevel_radii[64];
  int operand_bevel_order[64];
  int operand_surface_bevel_order[64];
  uint64_t operand_surface_edge_keys[64];

  int operand_selected_edge;
  int operand_bevel_edge;
  int operand_bevel_type; /* eNurbBodyBevelType */
  int operand_bevel_order_next;
  int operand_surface_selected_edge;
  int operand_surface_bevel_edge;
  int operand_surface_bevel_type; /* eNurbBodyBevelType */
  int operand_surface_bevel_order_next;

  float operand_bevel_radius;
  float operand_surface_bevel_radius;
  float operand_scale[3];
  float _pad1;

  uint64_t face_key;
  uint64_t generated_face_keys[64];
  float face_extrude_delta[3];
  float face_inset;
} NurbBodyBooleanOp;

typedef struct NurbBody {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(NurbBody)
  static constexpr ID_Type id_type = ID_NB;
#endif

  ID id;
  struct AnimData *adt = nullptr;

  int primitive = NURB_BODY_PRIMITIVE_CYLINDER;
  int flag = NURB_BODY_SMOOTH_SHADING;

  float radius = 1.0f;
  float depth = 2.0f;
  float dimensions[3] = {2.0f, 2.0f, 2.0f};
  float minor_radius = 0.25f;

  float cutter_radius = 0.35f;
  float cutter_depth = 3.0f;
  float cutter_location[3] = {0.0f, 0.0f, 0.0f};
  float cutter_rotation[3] = {0.0f, 0.0f, 0.0f};

  uint64_t selected_edges = 0;
  uint64_t bevel_edges = 0;
  uint64_t chamfer_edges = 0;
  uint64_t surface_selected_edges = 0;
  uint64_t surface_bevel_edges = 0;
  uint64_t surface_chamfer_edges = 0;
  uint64_t selected_faces = 0;
  float bevel_radii[64] = {};
  float surface_bevel_radii[64] = {};
  int bevel_order[64] = {};
  int surface_bevel_order[64] = {};
  uint64_t surface_edge_keys[64] = {};
  uint64_t face_keys[64] = {};
  uint64_t hovered_face_key = 0;

  int boolean_operation = NURB_BODY_BOOLEAN_DIFFERENCE;
  int selected_edge = -1;
  int hovered_edge = -1;
  int bevel_edge = -1;
  int bevel_type = NURB_BODY_BEVEL_FILLET;
  int bevel_order_next = 1;
  int surface_selected_edge = -1;
  int surface_hovered_edge = -1;
  int surface_bevel_edge = -1;
  int surface_bevel_type = NURB_BODY_BEVEL_FILLET;
  int surface_bevel_order_next = 1;
  int selected_face = -1;
  int hovered_face = -1;
  float bevel_radius = 0.0f;
  float surface_bevel_radius = 0.0f;
  float tessellation_deflection = 1.0f;
  float tessellation_angle = 0.558505f;
  float tessellation_face_deflection = 3.0f;
  float tessellation_face_angle = 1.11701f;
  float tessellation_density = 1.0f;
  float tessellation_min_width = 0.0f;
  float tessellation_plane_angle = 0.523599f;
  float auto_crease_angle = 0.523599f;
  int select_mode = NURB_BODY_SELECT_MODE_EDGE;
  int tessellation_topology = NURB_BODY_TESSELLATION_TRIS;
  int _pad1 = 0;

  ListBase boolean_ops = {}; /* NurbBodyBooleanOp */

  struct Material **mat = nullptr;
  short totcol = 0;
  char _pad[6] = {};
  void *_pad2 = nullptr;
} NurbBody;

}  // namespace blender
