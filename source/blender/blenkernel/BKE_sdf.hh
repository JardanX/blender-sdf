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
struct BMEditMesh;
struct SDF;
struct SDFModifier;
struct SDFPolygonPoint;

namespace bke {

struct SDFRuntime {
  BMEditMesh *edit_mesh = nullptr;
  void *proxy_batch = nullptr;
  uint64_t proxy_hash = 0;
};

}  // namespace bke

SDF *BKE_sdf_add(Main *bmain, const char *name);
void BKE_sdf_data_update(Depsgraph *depsgraph, Scene *scene, Object *ob);

int BKE_sdf_next_index(Main *bmain);
void BKE_sdf_reindex_all(Main *bmain);
void BKE_sdf_shift_indices_from(Main *bmain, int from_index, const SDF *skip);

SDFModifier *BKE_sdf_modifier_add(SDF *sdf, int type);
void BKE_sdf_modifier_remove(SDF *sdf, SDFModifier *mod);
void BKE_sdf_modifier_move(SDF *sdf, SDFModifier *mod, int direction);

SDFPolygonPoint *BKE_sdf_polygon_point_add(SDF *sdf, float x, float y, float corner);
void BKE_sdf_polygon_point_remove(SDF *sdf, SDFPolygonPoint *point);
void BKE_sdf_polygon_init_triangle(SDF *sdf);

void BKE_sdf_editmode_enter(Object *ob);
void BKE_sdf_editmode_load(Object *ob);
void BKE_sdf_editmode_exit(Object *ob);

}  // namespace blender
