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
} eNurbBodyPrimitive;

typedef enum eNurbBodyBooleanOperation {
  NURB_BODY_BOOLEAN_DIFFERENCE = 0,
  NURB_BODY_BOOLEAN_UNION = 1,
  NURB_BODY_BOOLEAN_INTERSECT = 2,
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

enum {
  NURB_BODY_MERGE_VERTICES = (1 << 0),
  NURB_BODY_SMOOTH_SHADING = (1 << 1),
  NURB_BODY_TRIANGULATE_MESH = (1 << 2),
  NURB_BODY_AUTO_CREASE_SHARP_EDGES = (1 << 3),
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

  int operation; /* eNurbBodyBooleanOperation */
  int flag;
  int primitive; /* eNurbBodyPrimitive */
  int selected_edge;
  int hovered_edge;
  int bevel_edge;
  int bevel_type; /* eNurbBodyBevelType */

  float bevel_radius;
  float operand_radius;
  float operand_depth;
  float _pad0;
  float _pad1;

  float operand_to_target[4][4];
} NurbBodyBooleanOp;

typedef struct NurbBody {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(NurbBody)
  static constexpr ID_Type id_type = ID_NB;
#endif

  ID id;
  struct AnimData *adt = nullptr;

  int primitive = NURB_BODY_PRIMITIVE_CYLINDER;
  int flag = NURB_BODY_MERGE_VERTICES | NURB_BODY_SMOOTH_SHADING | NURB_BODY_TRIANGULATE_MESH |
             NURB_BODY_AUTO_CREASE_SHARP_EDGES;

  float radius = 1.0f;
  float depth = 4.0f;

  float cutter_radius = 0.35f;
  float cutter_depth = 3.0f;
  float cutter_location[3] = {0.0f, 0.0f, 0.0f};
  float cutter_rotation[3] = {0.0f, 0.0f, 0.0f};

  uint64_t selected_edges = 0;
  uint64_t bevel_edges = 0;
  uint64_t chamfer_edges = 0;
  float bevel_radii[64] = {};

  int boolean_operation = NURB_BODY_BOOLEAN_DIFFERENCE;
  int selected_edge = -1;
  int hovered_edge = -1;
  int bevel_edge = -1;
  int bevel_type = NURB_BODY_BEVEL_FILLET;
  float bevel_radius = 0.0f;
  float tessellation_deflection = 0.006f;
  float tessellation_angle = 0.2f;
  float auto_crease_angle = 0.523599f;
  int select_mode = NURB_BODY_SELECT_MODE_EDGE;
  void *_pad1 = nullptr;

  ListBase boolean_ops = {}; /* NurbBodyBooleanOp */

  struct Material **mat = nullptr;
  short totcol = 0;
  char _pad[6] = {};
} NurbBody;

}  // namespace blender
