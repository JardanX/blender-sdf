/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <cstdint>
#include <memory>

/** \file
 * \ingroup bke
 * \brief SDF (Signed Distance Field) data-block.
 */

namespace blender {

struct Depsgraph;
struct Main;
struct Mesh;
struct ModifierData;
struct Object;
struct Scene;
struct BMEditMesh;
struct SDF;
struct SDFMeshBVHNode;
struct SDFMeshTriangle;
struct SDFMeshVertex;
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

enum class SDFMeshBuildResult {
  Success,
  NoTriangles,
  InvalidTopology,
  DegenerateTriangles,
};

struct SDFMeshPayload {
  SDFMeshPayload() = default;
  SDFMeshPayload(const SDFMeshPayload &) = delete;
  SDFMeshPayload &operator=(const SDFMeshPayload &) = delete;

  SDFMeshVertex *vertices = nullptr;
  SDFMeshTriangle *triangles = nullptr;
  SDFMeshBVHNode *bvh_nodes = nullptr;
  int vertex_count = 0;
  int triangle_count = 0;
  int bvh_node_count = 0;
  int flags = 0;
  float bounds_min[3] = {};
  float bounds_max[3] = {};
  uint64_t revision = 0;

  ~SDFMeshPayload();
};

struct SDFMeshRuntimeSnapshot {
  std::shared_ptr<const SDFMeshPayload> payload;
  SDFMeshBuildResult result = SDFMeshBuildResult::NoTriangles;
  int degenerate_triangles = 0;
};

SDFMeshBuildResult BKE_sdf_mesh_build(SDF *sdf,
                                      const Mesh *mesh,
                                      int *r_degenerate_triangles);
void BKE_sdf_mesh_clear(SDF *sdf);
bool BKE_sdf_mesh_bvh_make_stackless(SDF *sdf);
void BKE_sdf_mesh_runtime_update(Object &object, const Mesh &mesh);
void BKE_sdf_mesh_runtime_clear(Object &object);
bool BKE_sdf_mesh_runtime_snapshot(const Object &object, SDFMeshRuntimeSnapshot &r_snapshot);
bool BKE_sdf_object_is_enabled(const Object &object);
void BKE_sdf_object_settings_ensure(Object &object);

int BKE_sdf_next_index(Main *bmain);
void BKE_sdf_reindex_all(Main *bmain);
void BKE_sdf_shift_indices_from(Main *bmain, int from_index, const SDF *skip);

/* MATHOPS: Removed — SDF modifiers moved to native Object modifier stack */

SDFPolygonPoint *BKE_sdf_polygon_point_add(SDF *sdf, float x, float y, float corner);
void BKE_sdf_polygon_point_remove(SDF *sdf, SDFPolygonPoint *point);
void BKE_sdf_polygon_init_triangle(SDF *sdf);

void BKE_sdf_editmode_enter(Object *ob);
void BKE_sdf_editmode_load(Object *ob);
void BKE_sdf_editmode_exit(Object *ob);

}  // namespace blender
