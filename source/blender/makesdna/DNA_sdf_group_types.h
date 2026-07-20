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

#ifdef __cplusplus
namespace blender::bke {
struct SDFGroupRuntime;
}
#endif

namespace blender {

struct AnimData;
struct Object;

#ifdef __cplusplus
using SDFGroupRuntimeHandle = bke::SDFGroupRuntime;
#else
typedef struct SDFGroupRuntimeHandle SDFGroupRuntimeHandle;
#endif

typedef struct SDFGroupMember {
  struct SDFGroupMember *next = nullptr, *prev = nullptr;
  struct Object *object = nullptr;
  int order = 0;
  char _pad[4] = {};
} SDFGroupMember;

typedef struct SDFGroup {
#ifdef __cplusplus
  DNA_DEFINE_CXX_METHODS(SDFGroup)
  static constexpr ID_Type id_type = ID_SG;
#endif

  ID id;
  struct AnimData *adt = nullptr;

  int csg_operation = 0;
  int blend_type = 1;
  float blend = 0.1f;
  float shell_distance = 0.2f;
  int shell_mode = 0;
  int shell_op = 0;
  float shell_blend_top = 0.1f;
  float shell_blend_bottom = 0.1f;
  float chamfer_k2 = 0.001f;
  float chamfer_k3 = 0.001f;
  float chamfer_k4 = 0.01f;
  float chamfer_k5 = 0.01f;
  int flip_blend = 0;
  int flip_blend_end = 0;

  float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  ListBase members = {};
  int totmember = 0;
  int group_order = 0;

  ListBase modifiers = {};
  int totmodifier = 0;
  char _pad1[4] = {};

  SDFGroupRuntimeHandle *runtime = nullptr;
} SDFGroup;

}  // namespace blender
