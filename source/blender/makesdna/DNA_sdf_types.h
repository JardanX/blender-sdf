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

struct AnimData;
struct Material;
struct SDFGroup;

#ifdef __cplusplus
namespace blender::bke {
struct SDFRuntime;
}
using SDFRuntimeHandle = blender::bke::SDFRuntime;
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
} eSDFType;

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

typedef enum eSDFCSGOperation {
  SDF_CSG_UNION = 0,
  SDF_CSG_SUBTRACT = 1,
  SDF_CSG_INTERSECT = 2,
  SDF_CSG_SHELL = 3,
  SDF_CSG_PUSH = 4,
  SDF_CSG_AVOID = 5,
} eSDFCSGOperation;

typedef enum eSDFShellMode {
  SDF_SHELL_NORMAL = 0,
  SDF_SHELL_PUSH = 1,
  SDF_SHELL_AVOID = 2,
} eSDFShellMode;

/* Modifier types */
typedef enum eSDFModifierType {
  SDF_MOD_MIRROR = 0,
  SDF_MOD_TWIST = 1,
  SDF_MOD_BEND = 2,
  SDF_MOD_ELONGATE = 3,
  SDF_MOD_HOLLOW = 4,
  SDF_MOD_ROUND = 5,
  SDF_MOD_ONION = 6,
  SDF_MOD_BEVEL = 7,
  SDF_MOD_ARRAY = 8,
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

typedef struct SDFModifier {
  struct SDFModifier *next, *prev;

  int type; /* eSDFModifierType */
  int flag;

  /* Per-type params layout:
   * Mirror:   [0]=offset, [4]=blend
   * Twist:    [0]=strength
   * Bend:     [0]=strength, [1]=axis
   * Elongate: [0..2]=per-axis stretch
   * Hollow:   [0]=thickness
   * Round:    [0]=radius
   * Onion:    [0]=thickness
   * Array:    [0]=count, [1..3]=offset/radius, [4]=blend */
  float params[8];

  char name[64];
  short show_viewport;
  char _pad[6];

  struct Object *mirror_ob;
} SDFModifier;

typedef struct SDF {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(SDF)
  static constexpr ID_Type id_type = ID_SF;
#endif

  ID id;
  struct AnimData *adt; /* Must be immediately after id. */

  int sdf_type; /* eSDFType */
  char _pad0[4];

  float size[3];
  float bevel;
  float color[4];

  /* CSG */
  float blend;
  int blend_type;    /* eSDFBlendType */
  int csg_operation; /* eSDFCSGOperation */
  float shell_distance;
  int shell_mode; /* eSDFShellMode */
  char _pad6[4];

  /* Box properties */
  float box_corners[4]; /* Per-corner bevel (normalized 0–1) */
  float box_edge_top;
  float box_edge_bottom;
  float box_taper;
  int box_corner_mode; /* eSDFBoxMode */
  int box_edge_mode;   /* eSDFBoxMode */

  /* N-Gon properties */
  int ngon_sides;
  float ngon_corner;
  float ngon_edge_top;
  float ngon_edge_bottom;
  float ngon_taper;
  int ngon_edge_mode; /* eSDFBoxMode */
  float ngon_star;

  /* Torus */
  float torus_angle; /* Radians, 2*PI = full torus */
  char _pad4[4];

  /* Group */
  struct SDFGroup *sdf_group;
  int group_order;
  int sdf_index;

  /* Modifiers */
  ListBase modifiers; /* SDFModifier */
  int totmodifier;
  char _pad3[4];

  /* Materials */
  struct Material **mat;
  short totcol;
  char _pad2[6];

  SDFRuntimeHandle *runtime; /* Keep last. */
} SDF;

