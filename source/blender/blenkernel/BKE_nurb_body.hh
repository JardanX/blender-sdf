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
bool BKE_nurb_body_hovered_edge_key_set(const Object *object,
                                        const NurbBodyBooleanOp *op,
                                        int edge_index,
                                        int flag,
                                        uint64_t edge_key);
bool BKE_nurb_body_hovered_edge_key_clear(const Object *object);
uint64_t BKE_nurb_body_hovered_edge_key_get(const Object *object);
bool BKE_nurb_body_hovered_edge_key_matches(const Object *object,
                                            const NurbBodyEdgePolyline &polyline);
void BKE_nurb_body_data_update(Depsgraph *depsgraph, Scene *scene, Object *ob);

}  // namespace blender
