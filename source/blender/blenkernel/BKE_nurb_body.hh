/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include <cstdint>

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

namespace blender {

struct Depsgraph;
struct Main;
struct Mesh;
struct NurbBody;
struct NurbBodyBooleanOp;
struct Object;
struct Scene;

struct NurbBodyEdgePolyline {
  const NurbBodyBooleanOp *op = nullptr;
  int edge_index = -1;
  int flag = 0;
  uint64_t edge_key = 0;
  /* Points are in the NurbBody object's local space. */
  Vector<float3> points;
};

struct NurbBodyFaceSurface {
  int face_index = -1;
  uint64_t face_key = 0;
  float3 center = float3(0.0f);
  float3 normal = float3(0.0f, 0.0f, 1.0f);
  /* Triangle points are in the NurbBody object's local space, stored as triplets. */
  Vector<float3> triangles;
};

enum NurbBodyEdgePolylineFlag {
  NURB_BODY_EDGE_POLYLINE_SELECTABLE = (1 << 0),
  NURB_BODY_EDGE_POLYLINE_SURFACE = (1 << 1),
  NURB_BODY_EDGE_POLYLINE_BODY = (1 << 2),
  NURB_BODY_EDGE_POLYLINE_FINAL = (1 << 3),
};

NurbBody *BKE_nurb_body_add(Main *bmain, const char *name);
Mesh *BKE_nurb_body_to_mesh(const NurbBody *body, const Object *object);
void BKE_nurb_body_boolean_edge_polylines(const Object *object,
                                          Vector<NurbBodyEdgePolyline> &r_polylines,
                                          int samples_per_edge = 96);
Span<NurbBodyEdgePolyline> BKE_nurb_body_boolean_edge_polylines_cached(const Object *object,
                                                                       int samples_per_edge = 96);
uint64_t BKE_nurb_body_boolean_edge_polylines_cache_key(const Object *object,
                                                        int samples_per_edge = 96);
void BKE_nurb_body_face_surfaces(const Object *object, Vector<NurbBodyFaceSurface> &r_faces);
Span<NurbBodyFaceSurface> BKE_nurb_body_face_surfaces_cached(const Object *object);
uint64_t BKE_nurb_body_face_surfaces_cache_key(const Object *object);
void BKE_nurb_body_runtime_cache_clear(const Object *object);
void BKE_nurb_body_debug_bevel_set_drag_tick(uint64_t tick_id,
                                             float radius,
                                             int edge_index,
                                             int domain,
                                             uint64_t active_mask);
void BKE_nurb_body_debug_bevel_end_drag_tick(const char *reason);
bool BKE_nurb_body_debug_bevel_enabled();
void BKE_nurb_body_bevel_preview_radius_begin(const Object *object,
                                              uint64_t active_mask,
                                              float requested_radius);
bool BKE_nurb_body_bevel_preview_radius_get(const Object *object,
                                            uint64_t active_mask,
                                            float requested_radius,
                                            float *r_radius);
bool BKE_nurb_body_bevel_preview_radius_clamp(const Object *object,
                                              uint64_t active_mask,
                                              float requested_radius,
                                              float *r_radius);
enum NurbBodyBevelPreviewFailure {
  NURB_BODY_BEV_PREVIEW_FAILURE_NONE = 0,
  NURB_BODY_BEV_PREVIEW_FAILURE_SOLVER = 1,
  NURB_BODY_BEV_PREVIEW_FAILURE_TIMEOUT = 2,
};
bool BKE_nurb_body_bevel_preview_failure_get(const Object *object,
                                             uint64_t active_mask,
                                             int *r_reason,
                                             float *r_failed_radius);
void BKE_nurb_body_bevel_preview_failure_clear(const Object *object, uint64_t active_mask);
void BKE_nurb_body_bevel_preview_radius_clear(const Object *object);
bool BKE_nurb_body_hovered_edge_key_set(const Object *object,
                                        const NurbBodyBooleanOp *op,
                                        int edge_index,
                                        int flag,
                                        uint64_t edge_key);
bool BKE_nurb_body_hovered_edge_key_clear(const Object *object);
uint64_t BKE_nurb_body_hovered_edge_key_get(const Object *object);
bool BKE_nurb_body_hovered_edge_key_matches(const Object *object,
                                            const NurbBodyEdgePolyline &polyline);
bool BKE_nurb_body_selected_edge_key_set(const Object *object,
                                         const NurbBodyBooleanOp *op,
                                         int edge_index,
                                         int flag,
                                         uint64_t edge_key);
bool BKE_nurb_body_selected_edge_key_clear(const Object *object);
uint64_t BKE_nurb_body_selected_edge_key_get(const Object *object);
bool BKE_nurb_body_selected_edge_key_matches(const Object *object,
                                             const NurbBodyEdgePolyline &polyline);
void BKE_nurb_body_data_update(Depsgraph *depsgraph, Scene *scene, Object *ob);

}  // namespace blender
