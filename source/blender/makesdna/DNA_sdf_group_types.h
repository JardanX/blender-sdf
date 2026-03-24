/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 * \brief SDF Group data types for ordered SDF evaluation.
 */

#pragma once

#include "DNA_ID.h"
#include "DNA_defs.h"
#include "DNA_listBase.h"

struct AnimData;
struct Object;

#ifdef __cplusplus
namespace blender::bke {
struct SDFGroupRuntime;
}
using SDFGroupRuntimeHandle = blender::bke::SDFGroupRuntime;
#else
typedef struct SDFGroupRuntimeHandle SDFGroupRuntimeHandle;
#endif

typedef struct SDFGroupMember {
  struct SDFGroupMember *next, *prev;
  struct Object *object;
  int order;
  char _pad[4];
} SDFGroupMember;

typedef struct SDFGroup {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(SDFGroup)
  static constexpr ID_Type id_type = ID_SG;
#endif

  ID id;
  struct AnimData *adt; /* Must be immediately after id. */

  /* Group-level CSG */
  int csg_operation; /* eSDFCSGOperation */
  int blend_type;    /* eSDFBlendType */
  float blend;
  float shell_distance;
  int shell_mode; /* eSDFShellMode */
  int shell_op;   /* eSDFShellOp */
  float shell_blend_top;
  float shell_blend_bottom;
  float chamfer_k2;
  float chamfer_k3;

  float color[4];

  /* Members */
  ListBase members; /* SDFGroupMember */
  int totmember;
  int group_order;

  /* Modifier stack */
  ListBase modifiers; /* SDFModifier */
  int totmodifier;
  char _pad1[4];

  SDFGroupRuntimeHandle *runtime; /* Keep last. */
} SDFGroup;
