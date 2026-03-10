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

#ifdef __cplusplus
namespace blender::bke {
struct SDFRuntime;
}
using SDFRuntimeHandle = blender::bke::SDFRuntime;
#else
typedef struct SDFRuntimeHandle SDFRuntimeHandle;
#endif

typedef enum eSDFType {
  SDF_TYPE_BOX = 0,
  SDF_TYPE_SPHERE = 1,
  SDF_TYPE_CYLINDER = 2,
  SDF_TYPE_CONE = 3,
  SDF_TYPE_CAPSULE = 4,
  SDF_TYPE_TORUS = 5,
  SDF_TYPE_NGON = 6,
} eSDFType;

/** Box corner/edge blend mode. */
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

/** SDF modifier types. */
typedef enum eSDFModifierType {
  SDF_MOD_MIRROR = 0,
  SDF_MOD_TWIST = 1,
  SDF_MOD_BEND = 2,
  SDF_MOD_ELONGATE = 3,
  SDF_MOD_HOLLOW = 4,
  SDF_MOD_ROUND = 5,
  SDF_MOD_ONION = 6,
  SDF_MOD_BEVEL = 7,
} eSDFModifierType;

/** Mirror axis flags. */
enum {
  SDF_MOD_MIRROR_X = (1 << 0),
  SDF_MOD_MIRROR_Y = (1 << 1),
  SDF_MOD_MIRROR_Z = (1 << 2),
};

/** Single SDF modifier in the stack. */
typedef struct SDFModifier {
  struct SDFModifier *next, *prev;

  /** Modifier type (eSDFModifierType). */
  int type;
  /** Modifier-specific flags (e.g., mirror axes). */
  int flag;

  /** Generic parameters — meaning depends on type:
   *  Mirror:   (unused, axes in .flag)
   *  Twist:    params[0] = strength (radians/unit)
   *  Bend:     params[0] = strength, params[1] = axis (0=X,1=Y,2=Z)
   *  Elongate: params[0..2] = elongation per axis
   *  Hollow:   params[0] = wall thickness
   *  Round:    params[0] = radius
   *  Onion:    params[0] = layer thickness
   */
  float params[4];

  /** Display name. */
  char name[64];

  /** Whether this modifier is enabled. */
  short show_viewport;
  char _pad[6];
} SDFModifier;

typedef struct SDF {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(SDF)
  /** See #ID_Type comment for why this is here. */
  static constexpr ID_Type id_type = ID_SF;
#endif

  ID id;
  /** Animation data (must be immediately after id for utilities to use it). */
  struct AnimData *adt;

  /** SDF primitive type. */
  int sdf_type;
  char _pad0[4];

  /** Size in each axis. */
  float size[3];
  /** Bevel radius. */
  float bevel;

  /** Display color (RGBA). */
  float color[4];

  /** Blend amount for CSG operations. */
  float blend;
  /** Blend type (eSDFBlendType). */
  int blend_type;
  /** CSG operation (eSDFCSGOperation). */
  int csg_operation;
  /** Shell/extrusion offset distance (used when csg_operation == SDF_CSG_SHELL). */
  float shell_distance;

  /** Box-specific shape properties. */
  /** Per-corner bevel radii (normalized 0–1, scaled by min(size.xy)). */
  float box_corners[4];
  /** Top/bottom edge chamfer (normalized 0–1). */
  float box_edge_top;
  float box_edge_bottom;
  /** Taper factor (-1 to 1). */
  float box_taper;
  /** Corner blend mode (eSDFBoxMode: 0=smooth, 1=chamfer). */
  int box_corner_mode;
  /** Edge blend mode (eSDFBoxMode: 0=smooth, 1=chamfer). */
  int box_edge_mode;

  /** N-Gon-specific shape properties. */
  /** Number of polygon sides (3–32). */
  int ngon_sides;
  /** Uniform corner bevel (normalized 0–1, Minkowski round). */
  float ngon_corner;
  /** Top/bottom edge chamfer (normalized 0–1). */
  float ngon_edge_top;
  float ngon_edge_bottom;
  /** Taper factor (-1 to 1). */
  float ngon_taper;
  /** Edge blend mode (eSDFBoxMode: 0=smooth, 1=chamfer). */
  int ngon_edge_mode;
  /** N-Gon star factor (0 = regular polygon, 1 = maximum star). */
  float ngon_star;

  /** Torus angle aperture (degrees, 0–360; 360 = full torus). */
  float torus_angle;
  char _pad4[4];

  /** Ordered modifier stack. */
  ListBase modifiers; /* SDFModifier */
  /** Number of modifiers. */
  int totmodifier;
  char _pad3[4];

  /** Material slots. */
  struct Material **mat;
  /** Number of material slots. */
  short totcol;
  char _pad2[6];

  /** Runtime data (keep last). */
  SDFRuntimeHandle *runtime;
} SDF;

