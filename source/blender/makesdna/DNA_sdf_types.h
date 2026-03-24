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

enum eSDFType {
  SDF_TYPE_BOX = 0,
  SDF_TYPE_SPHERE = 1,
  SDF_TYPE_CAPSULE = 4,
  SDF_TYPE_TORUS = 5,
};

enum eSDFBlendType {
  SDF_BLEND_LINEAR = 0,
  SDF_BLEND_SMOOTH = 1,
  SDF_BLEND_CHAMFER = 2,
  SDF_BLEND_ROUND = 3,
};

enum eSDFCSGOperation {
  SDF_CSG_UNION = 0,
  SDF_CSG_SUBTRACT = 1,
  SDF_CSG_INTERSECT = 2,
};

struct SDF {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(SDF)
  static constexpr ID_Type id_type = ID_SF;
#endif

  ID id;
  struct AnimData *adt;

  int sdf_type;
  char _pad0[4];

  float size[3];
  float bevel;

  float color[4];

  float blend;
  int blend_type;
  int csg_operation;
  char _pad1[4];

  struct Material **mat;
  short totcol;
  char _pad2[6];

  SDFRuntimeHandle *runtime;
};

/** #SDF.flag */
enum {
  SDF_DS_EXPAND = (1 << 0),
};

}  // namespace blender
