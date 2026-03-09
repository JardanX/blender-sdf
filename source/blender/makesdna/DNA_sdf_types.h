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
  /* 2, 3 reserved (removed: Cylinder, Cone). */
  SDF_TYPE_CAPSULE = 4,
  SDF_TYPE_TORUS = 5,
} eSDFType;

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
} eSDFCSGOperation;

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

  /** Material slots. */
  struct Material **mat;
  /** Number of material slots. */
  short totcol;
  char _pad2[6];

  /** Runtime data (keep last). */
  SDFRuntimeHandle *runtime;
} SDF;

