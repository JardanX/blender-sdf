/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include "BLI_math_vector_types.hh"
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
  /* Points are in the NurbBody object's local space. */
  Vector<float3> points;
};

enum NurbBodyEdgePolylineFlag {
  NURB_BODY_EDGE_POLYLINE_SELECTABLE = (1 << 0),
  NURB_BODY_EDGE_POLYLINE_SURFACE = (1 << 1),
  NURB_BODY_EDGE_POLYLINE_BODY = (1 << 2),
};

NurbBody *BKE_nurb_body_add(Main *bmain, const char *name);
Mesh *BKE_nurb_body_to_mesh(const NurbBody *body, const Object *object);
void BKE_nurb_body_boolean_edge_polylines(const Object *object,
                                          Vector<NurbBodyEdgePolyline> &r_polylines,
                                          int samples_per_edge = 96);
void BKE_nurb_body_data_update(Depsgraph *depsgraph, Scene *scene, Object *ob);

}  // namespace blender
