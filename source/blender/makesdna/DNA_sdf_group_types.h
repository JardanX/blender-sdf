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

/** A member entry in an SDFGroup's ordered evaluation list. */
typedef struct SDFGroupMember {
  struct SDFGroupMember *next, *prev;
  /** OB_SDF object. */
  struct Object *object;
  /** Evaluation order within the group (0-based, auto-assigned). */
  int order;
  char _pad[4];
} SDFGroupMember;

typedef struct SDFGroup {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(SDFGroup)
  /** See #ID_Type comment for why this is here. */
  static constexpr ID_Type id_type = ID_SG;
#endif

  ID id;
  /** Animation data (must be immediately after id for utilities to use it). */
  struct AnimData *adt;

  /** Group-level CSG operation (eSDFCSGOperation: how this group combines with previous groups). */
  int csg_operation;
  /** Blend type (eSDFBlendType: linear/smooth/chamfer/round). */
  int blend_type;
  /** Blend amount for group-level CSG. */
  float blend;
  /** Shell/extrusion distance (when csg_operation == SDF_CSG_SHELL). */
  float shell_distance;

  /** Display color (RGBA) for outliner tag. */
  float color[4];

  /** Ordered member list. */
  ListBase members; /* SDFGroupMember */
  /** Number of members. */
  int totmember;
  /** Evaluation order among groups (0-based). */
  int group_order;

  /** Runtime data (keep last). */
  SDFGroupRuntimeHandle *runtime;
} SDFGroup;
