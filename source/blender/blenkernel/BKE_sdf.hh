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

struct SDFRuntime {};

}  // namespace blender::bke

SDF *BKE_sdf_add(Main *bmain, const char *name);
void BKE_sdf_data_update(Depsgraph *depsgraph, Scene *scene, Object *ob);

int BKE_sdf_next_index(Main *bmain);
void BKE_sdf_reindex_all(Main *bmain);
void BKE_sdf_shift_indices_from(Main *bmain, int from_index, const SDF *skip);

SDFModifier *BKE_sdf_modifier_add(SDF *sdf, int type);
void BKE_sdf_modifier_remove(SDF *sdf, SDFModifier *mod);
void BKE_sdf_modifier_move(SDF *sdf, SDFModifier *mod, int direction);
