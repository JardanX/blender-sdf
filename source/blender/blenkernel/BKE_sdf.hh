/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief SDF (Signed Distance Field) data-block.
 */

namespace blender {
struct Depsgraph;
struct Main;
struct Object;
struct Scene;
struct SDF;

namespace bke {
struct SDFRuntime {
  /* Placeholder for future runtime data. */
};
}  // namespace bke

SDF *BKE_sdf_add(Main *bmain, const char *name);

void BKE_sdf_data_update(Depsgraph *depsgraph, Scene *scene, Object *ob);

}  // namespace blender
