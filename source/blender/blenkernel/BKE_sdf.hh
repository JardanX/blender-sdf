/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief SDF (Signed Distance Field) data-block.
 */

struct Depsgraph;
struct Main;
struct Object;
struct Scene;
struct SDF;
struct SDFModifier;

namespace blender::bke {

struct SDFRuntime {
  /* Placeholder for future runtime data. */
};

}  // namespace blender::bke

SDF *BKE_sdf_add(Main *bmain, const char *name);

void BKE_sdf_data_update(Depsgraph *depsgraph, Scene *scene, Object *ob);

/** Add a new modifier of the given type to the end of the stack. */
SDFModifier *BKE_sdf_modifier_add(SDF *sdf, int type);
/** Remove and free a modifier from the stack. */
void BKE_sdf_modifier_remove(SDF *sdf, SDFModifier *mod);
/** Move a modifier up (-1) or down (+1) in the stack. */
void BKE_sdf_modifier_move(SDF *sdf, SDFModifier *mod, int direction);
