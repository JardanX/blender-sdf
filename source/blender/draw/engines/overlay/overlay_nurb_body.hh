/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>

#include "DNA_mesh_types.h"
#include "DNA_nurb_body_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_userdef_types.h"

#include "BLI_array.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"
#include "BKE_nurb_body.hh"

#include "DEG_depsgraph_query.hh"

#include "DRW_render.hh"

#include "GPU_batch.hh"
#include "GPU_matrix.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"

#include "overlay_base.hh"

namespace blender::draw::overlay {

class NurbBodies : Overlay {
 private:
  PassMain depth_ps_ = {"NurbBodyDepth"};
  PassMain::Sub *depth_mesh_ps_ = nullptr;

  struct Entry {
    Object *object;
    const Object *original_object;
  };

  struct DrawCache {
    const Object *object = nullptr;
    uint64_t geometry_key = 0;
    uint64_t selection_key = 0;
    uint64_t hover_key = 0;
    uint64_t face_geometry_key = 0;
    uint64_t face_selection_key = 0;
    uint64_t face_hover_key = 0;
    gpu::Batch *normal_batch = nullptr;
    gpu::Batch *hovered_batch = nullptr;
    gpu::Batch *selected_batch = nullptr;
    gpu::Batch *face_hovered_batch = nullptr;
    gpu::Batch *face_selected_batch = nullptr;
  };

  Vector<Entry> entries_;
  Vector<DrawCache> draw_caches_;
  float line_width_ = 2.0f;
  View::OffsetData offset_data_;
  int select_mode_ = NURB_BODY_SELECT_MODE_EDGE;
  bool edge_overlay_enabled_ = false;
  bool xray_enabled_ = false;
  bool needs_depth_prepass_ = false;

  static bool edge_index_in_mask(const uint64_t mask, const int edge_index)
  {
    return edge_index >= 0 && edge_index < 64 && (mask & (uint64_t(1) << uint(edge_index)));
  }

  static void hash_bytes(uint64_t &hash, const void *data, const size_t size)
  {
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < size; i++) {
      hash ^= uint64_t(bytes[i]);
      hash *= 1099511628211ull;
    }
  }

  template<typename T> static void hash_value(uint64_t &hash, const T &value)
  {
    hash_bytes(hash, &value, sizeof(T));
  }

  static bool select_mode_is_valid(const int select_mode)
  {
    return select_mode == NURB_BODY_SELECT_MODE_EDGE ||
           select_mode == NURB_BODY_SELECT_MODE_FACE ||
           select_mode == NURB_BODY_SELECT_MODE_OBJECT;
  }

  static uint64_t line_selection_key(const Object *object,
                                     const NurbBody &body,
                                     const int select_mode,
                                     const bool object_selected)
  {
    uint64_t hash = 1469598103934665603ull;
    hash_value(hash, select_mode);
    hash_value(hash, object_selected);
    hash_value(hash, body.selected_edges);
    hash_value(hash, body.selected_edge);
    hash_value(hash, body.surface_selected_edges);
    hash_value(hash, body.surface_selected_edge);
    hash_bytes(hash, body.surface_edge_keys, sizeof(body.surface_edge_keys));
    hash_value(hash, BKE_nurb_body_selected_edge_key_get(object));

    int op_count = 0;
    for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(
             body.boolean_ops.first);
         op;
         op = op->next)
    {
      op_count++;
      hash_value(hash, op->flag & NURB_BODY_BOOLEAN_OP_SELECTED);
      hash_value(hash, op->selected_edges);
      hash_value(hash, op->selected_edge);
    }
    hash_value(hash, op_count);
    return hash;
  }

  static uint64_t line_hover_key(const Object *object,
                                 const NurbBody &body,
                                 const int select_mode,
                                 const bool object_selected)
  {
    uint64_t hash = 1469598103934665603ull;
    hash_value(hash, select_mode);
    hash_value(hash, object_selected);
    hash_value(hash, BKE_nurb_body_hovered_edge_key_get(object));
    hash_value(hash, body.hovered_edge);
    hash_value(hash, body.surface_hovered_edge);

    int op_count = 0;
    for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(
             body.boolean_ops.first);
         op;
         op = op->next)
    {
      op_count++;
      hash_value(hash, op->flag & NURB_BODY_BOOLEAN_OP_HOVERED);
      hash_value(hash, op->hovered_edge);
    }
    hash_value(hash, op_count);
    return hash;
  }

  static uint64_t face_selection_key(const NurbBody &body,
                                     const int select_mode,
                                     const bool object_selected)
  {
    uint64_t hash = 1469598103934665603ull;
    hash_value(hash, select_mode);
    hash_value(hash, object_selected);
    hash_value(hash, body.selected_faces);
    hash_value(hash, body.selected_face);
    hash_bytes(hash, body.face_keys, sizeof(body.face_keys));
    return hash;
  }

  static uint64_t face_hover_key(const NurbBody &body,
                                 const int select_mode,
                                 const bool object_selected)
  {
    uint64_t hash = 1469598103934665603ull;
    hash_value(hash, select_mode);
    hash_value(hash, object_selected);
    hash_value(hash, body.hovered_face);
    hash_value(hash, body.hovered_face_key);
    return hash;
  }

  static float line_thickness_for_overlay(const View3DOverlay &overlay)
  {
    const float pixel_size = std::max(U.pixelsize, 1.0f);
    if (!std::isfinite(overlay.nurb_body_line_thickness) ||
        overlay.nurb_body_line_thickness <= 0.0f)
    {
      return pixel_size;
    }
    return std::clamp(overlay.nurb_body_line_thickness * 0.5f, 1.0f, 2.0f) *
           pixel_size;
  }

  static bool edge_selection_mode_enabled(const int select_mode, const bool /*object_selected*/)
  {
    return select_mode == NURB_BODY_SELECT_MODE_EDGE;
  }

  static bool face_selection_mode_enabled(const int select_mode, const bool /*object_selected*/)
  {
    return select_mode == NURB_BODY_SELECT_MODE_FACE;
  }

  static bool polyline_is_surface(const NurbBodyEdgePolyline &polyline)
  {
    return (polyline.flag & NURB_BODY_EDGE_POLYLINE_SURFACE) != 0 && polyline.points.size() >= 2;
  }

  static bool surface_edge_key_is_selected(const NurbBody &body, const uint64_t edge_key)
  {
    if (edge_key == 0) {
      return false;
    }
    for (int i = 0; i < 64; i++) {
      if (edge_index_in_mask(body.surface_selected_edges, i) &&
          body.surface_edge_keys[i] == edge_key)
      {
        return true;
      }
    }
    return false;
  }

  static bool face_key_is_selected(const NurbBody &body, const uint64_t face_key)
  {
    if (face_key == 0) {
      return false;
    }
    for (int i = 0; i < 64; i++) {
      if (edge_index_in_mask(body.selected_faces, i) && body.face_keys[i] == face_key) {
        return true;
      }
    }
    return false;
  }

  static bool face_is_selected(const NurbBody &body,
                               const NurbBodyFaceSurface &face,
                               const int select_mode,
                               const bool object_selected)
  {
    if (!face_selection_mode_enabled(select_mode, object_selected)) {
      return false;
    }
    return face_key_is_selected(body, face.face_key);
  }

  static bool face_is_hovered(const NurbBody &body,
                              const NurbBodyFaceSurface &face,
                              const int select_mode,
                              const bool object_selected)
  {
    if (!face_selection_mode_enabled(select_mode, object_selected)) {
      return false;
    }
    return face.face_key != 0 && body.hovered_face_key == face.face_key;
  }

  static bool polyline_is_selected(const Object *object,
                                   const NurbBody &body,
                                   const NurbBodyEdgePolyline &polyline,
                                   const int select_mode,
                                   const bool object_selected)
  {
    if (!edge_selection_mode_enabled(select_mode, object_selected)) {
      return false;
    }

    const NurbBodyBooleanOp *op = polyline.op;
    const bool body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
    const bool surface_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0;
    const uint64_t selected_edge_key = BKE_nurb_body_selected_edge_key_get(object);
    if (selected_edge_key != 0 && BKE_nurb_body_selected_edge_key_matches(object, polyline)) {
      return true;
    }
    if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_SELECTABLE) != 0 &&
        surface_edge_key_is_selected(body, polyline.edge_key))
    {
      return true;
    }
    if (body_edge) {
      return edge_index_in_mask(body.selected_edges, polyline.edge_index);
    }
    if (surface_edge) {
      return edge_index_in_mask(body.surface_selected_edges, polyline.edge_index) &&
             (body.surface_edge_keys[polyline.edge_index] == 0 ||
              body.surface_edge_keys[polyline.edge_index] == polyline.edge_key);
    }
    return op != nullptr && polyline.edge_index >= 0 &&
           (op->flag & NURB_BODY_BOOLEAN_OP_SELECTED) != 0 &&
           edge_index_in_mask(op->selected_edges, polyline.edge_index);
  }

  static bool polyline_is_hovered(const Object *object,
                                  const NurbBody &body,
                                  const NurbBodyEdgePolyline &polyline,
                                  const int select_mode,
                                  const bool object_selected)
  {
    if (!edge_selection_mode_enabled(select_mode, object_selected)) {
      return false;
    }

    const NurbBodyBooleanOp *op = polyline.op;
    const bool body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
    const bool surface_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0;
    if (body_edge) {
      return body.hovered_edge == polyline.edge_index &&
             BKE_nurb_body_hovered_edge_key_matches(object, polyline);
    }
    if (surface_edge) {
      return body.surface_hovered_edge == polyline.edge_index &&
             BKE_nurb_body_hovered_edge_key_matches(object, polyline);
    }
    return op != nullptr && polyline.edge_index >= 0 &&
           (op->flag & NURB_BODY_BOOLEAN_OP_HOVERED) != 0 &&
           op->hovered_edge == polyline.edge_index &&
           BKE_nurb_body_hovered_edge_key_matches(object, polyline);
  }

  static void discard_batch(gpu::Batch *&batch)
  {
    if (batch != nullptr) {
      GPU_batch_discard(batch);
      batch = nullptr;
    }
  }

  static void clear_draw_cache(DrawCache &cache)
  {
    discard_batch(cache.normal_batch);
    discard_batch(cache.hovered_batch);
    discard_batch(cache.selected_batch);
    discard_batch(cache.face_hovered_batch);
    discard_batch(cache.face_selected_batch);
    cache.geometry_key = 0;
    cache.selection_key = 0;
    cache.hover_key = 0;
    cache.face_geometry_key = 0;
    cache.face_selection_key = 0;
    cache.face_hover_key = 0;
  }

  static gpu::Batch *create_line_batch_from_segments(const Span<float3> verts)
  {
    if (verts.is_empty()) {
      return nullptr;
    }

    GPUVertFormat format = {0};
    GPU_vertformat_attr_add(&format, "pos", gpu::VertAttrType::SFLOAT_32_32_32);
    gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
    GPU_vertbuf_data_alloc(*vbo, verts.size());
    vbo->data<float3>().copy_from(verts);
    return GPU_batch_create_ex(GPU_PRIM_LINES, vbo, nullptr, GPU_BATCH_OWNS_VBO);
  }

  struct PolylineProjection {
    float dist_sq = FLT_MAX;
    float3 tangent = float3(0.0f);
    float length = 0.0f;
  };

  struct PolylineVertexCandidate {
    int vert_index = -1;
    float length = 0.0f;
    float dist_sq = FLT_MAX;
  };

  static PolylineProjection closest_polyline_projection(const float3 &point,
                                                        const Span<float3> polyline)
  {
    PolylineProjection best;
    if (polyline.size() < 2) {
      return best;
    }

    float length_before = 0.0f;
    for (int i = 1; i < polyline.size(); i++) {
      const float3 a = polyline[i - 1];
      const float3 b = polyline[i];
      const float3 ab = b - a;
      const float ab_len_sq = math::length_squared(ab);
      if (ab_len_sq <= 1.0e-20f) {
        continue;
      }

      const float segment_length = std::sqrt(ab_len_sq);
      const float factor = std::clamp(math::dot(point - a, ab) / ab_len_sq, 0.0f, 1.0f);
      const float3 closest = a + ab * factor;
      const float dist_sq = math::distance_squared(point, closest);
      if (dist_sq < best.dist_sq) {
        best.dist_sq = dist_sq;
        best.tangent = ab / segment_length;
        best.length = length_before + segment_length * factor;
      }
      length_before += segment_length;
    }
    return best;
  }

  static float polyline_length(const Span<float3> polyline)
  {
    float length = 0.0f;
    for (int i = 1; i < polyline.size(); i++) {
      length += math::distance(polyline[i - 1], polyline[i]);
    }
    return length;
  }

  static bool polyline_is_closed(const Span<float3> polyline, const float threshold)
  {
    return polyline.size() > 2 &&
           math::distance_squared(polyline.first(), polyline.last()) <= threshold * threshold;
  }

  static float mesh_topology_match_threshold(const Span<float3> polyline)
  {
    if (polyline.size() < 2) {
      return 0.0f;
    }

    const float length = polyline_length(polyline);
    const float average_segment_length = length / float(polyline.size() - 1);
    return std::max(std::max(average_segment_length * 0.18f, length * 0.0005f), 1.0e-5f);
  }

  static void add_edge_source_face(Array<int2> &edge_source_faces,
                                   const int edge_i,
                                   const int source_face)
  {
    if (source_face < 0 || edge_i < 0 || edge_i >= edge_source_faces.size()) {
      return;
    }

    int2 &source_faces = edge_source_faces[edge_i];
    if (source_faces[0] == -1) {
      source_faces[0] = source_face;
      return;
    }
    if (source_faces[0] == source_face) {
      if (source_faces[1] == -1) {
        source_faces[1] = source_face;
      }
      return;
    }
    if (source_faces[1] == -1 || source_faces[1] == source_faces[0]) {
      source_faces[1] = source_face;
    }
  }

  static Array<int2> mesh_edge_source_face_pairs(const Mesh &mesh)
  {
    const bke::AttributeAccessor attributes = mesh.attributes();
    const bke::AttributeReader<int> source_face_reader = attributes.lookup<int>(
        ".nurb_body_source_face_index", bke::AttrDomain::Face);
    if (!source_face_reader) {
      return {};
    }

    const VArraySpan<int> source_faces_by_face = *source_face_reader;
    if (source_faces_by_face.size() != mesh.faces_num) {
      return {};
    }

    Array<int2> edge_source_faces(mesh.edges().size(), int2(-1, -1));
    const OffsetIndices<int> faces = mesh.faces();
    const Span<int> corner_edges = mesh.corner_edges();
    for (const int face_i : faces.index_range()) {
      const int source_face = source_faces_by_face[face_i];
      for (const int corner : faces[face_i]) {
        add_edge_source_face(edge_source_faces, corner_edges[corner], source_face);
      }
    }
    return edge_source_faces;
  }

  static bool mesh_edge_is_internal_source_face(const Span<int2> edge_source_faces,
                                                const int edge_i)
  {
    if (edge_i < 0 || edge_i >= edge_source_faces.size()) {
      return false;
    }
    const int2 source_faces = edge_source_faces[edge_i];
    return source_faces[0] != -1 && source_faces[1] == source_faces[0];
  }

  static void append_snapped_polyline_segments(const Span<float3> positions,
                                               const Span<int2> edges,
                                               const Span<int2> edge_source_faces,
                                               const Span<float3> polyline,
                                               Vector<float3> &r_verts)
  {
    const float threshold = mesh_topology_match_threshold(polyline);
    const float threshold_sq = threshold * threshold;
    Vector<PolylineVertexCandidate> candidates;
    candidates.reserve(positions.size());
    for (const int vert_i : positions.index_range()) {
      const PolylineProjection projection = closest_polyline_projection(positions[vert_i],
                                                                        polyline);
      if (projection.dist_sq > threshold_sq) {
        continue;
      }

      PolylineVertexCandidate candidate;
      candidate.vert_index = vert_i;
      candidate.length = projection.length;
      candidate.dist_sq = projection.dist_sq;
      candidates.append(candidate);
    }

    if (candidates.size() < 2) {
      return;
    }

    std::sort(candidates.begin(),
              candidates.end(),
              [](const PolylineVertexCandidate &a, const PolylineVertexCandidate &b) {
                return a.length < b.length;
              });

    Vector<PolylineVertexCandidate> snapped_vertices;
    snapped_vertices.reserve(candidates.size());
    const float merge_threshold = std::max(threshold * 1.5f, 1.0e-5f);
    for (const PolylineVertexCandidate &candidate : candidates) {
      if (snapped_vertices.is_empty() ||
          std::abs(candidate.length - snapped_vertices.last().length) > merge_threshold)
      {
        snapped_vertices.append(candidate);
        continue;
      }

      if (candidate.dist_sq < snapped_vertices.last().dist_sq) {
        snapped_vertices.last() = candidate;
      }
    }

    if (snapped_vertices.size() < 2) {
      return;
    }

    const int segment_count = std::max(int(polyline.size()) - 1, 1);
    const float edge_length_limit = std::max(threshold * 8.0f,
                                            polyline_length(polyline) /
                                                float(segment_count) * 3.0f);
    const bool closed = polyline_is_closed(polyline, threshold);

    Array<int> snapped_by_vert(positions.size(), -1);
    for (const int i : snapped_vertices.index_range()) {
      snapped_by_vert[snapped_vertices[i].vert_index] = i;
    }

    const float total_length = polyline_length(polyline);
    for (const int edge_i : edges.index_range()) {
      if (mesh_edge_is_internal_source_face(edge_source_faces, edge_i)) {
        continue;
      }

      const int2 &edge = edges[edge_i];
      if (edge[0] < 0 || edge[1] < 0 || edge[0] >= positions.size() ||
          edge[1] >= positions.size())
      {
        continue;
      }

      const int snapped_a = snapped_by_vert[edge[0]];
      const int snapped_b = snapped_by_vert[edge[1]];
      if (snapped_a == -1 || snapped_b == -1 || snapped_a == snapped_b) {
        continue;
      }

      float length_delta = std::abs(snapped_vertices[snapped_a].length -
                                    snapped_vertices[snapped_b].length);
      if (closed) {
        length_delta = std::min(length_delta, std::max(total_length - length_delta, 0.0f));
      }
      if (length_delta > edge_length_limit) {
        continue;
      }

      const float3 &a = positions[edge[0]];
      const float3 &b = positions[edge[1]];
      const float3 edge_vec = b - a;
      const float edge_len_sq = math::length_squared(edge_vec);
      if (edge_len_sq <= 1.0e-20f) {
        continue;
      }

      const PolylineProjection midpoint_projection = closest_polyline_projection((a + b) * 0.5f,
                                                                                 polyline);
      if (midpoint_projection.dist_sq > threshold_sq * 4.0f) {
        continue;
      }

      const float3 edge_dir = edge_vec / std::sqrt(edge_len_sq);
      if (std::abs(math::dot(edge_dir, midpoint_projection.tangent)) < 0.35f) {
        continue;
      }

      r_verts.append(a);
      r_verts.append(b);
    }
  }

  static void append_analytic_polyline_segments(const Span<float3> polyline, Vector<float3> &r_verts)
  {
    if (polyline.size() < 2) {
      return;
    }
    for (int i = 1; i < polyline.size(); i++) {
      if (math::distance_squared(polyline[i - 1], polyline[i]) <= 1.0e-12f) {
        continue;
      }
      r_verts.append(polyline[i - 1]);
      r_verts.append(polyline[i]);
    }
  }

  static gpu::Batch *create_tri_batch_from_verts(const Span<float3> verts)
  {
    if (verts.is_empty()) {
      return nullptr;
    }

    GPUVertFormat format = {0};
    GPU_vertformat_attr_add(&format, "pos", gpu::VertAttrType::SFLOAT_32_32_32);
    gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
    GPU_vertbuf_data_alloc(*vbo, verts.size());
    vbo->data<float3>().copy_from(verts);
    return GPU_batch_create_ex(GPU_PRIM_TRIS, vbo, nullptr, GPU_BATCH_OWNS_VBO);
  }

  template<typename Predicate>
  static gpu::Batch *create_line_batch(const Mesh &mesh,
                                       const Span<NurbBodyEdgePolyline> polylines,
                                       Predicate &&predicate)
  {
    const Span<float3> positions = mesh.vert_positions();
    const Span<int2> edges = mesh.edges();
    if (positions.is_empty()) {
      return nullptr;
    }

    const Array<int2> edge_source_faces = mesh_edge_source_face_pairs(mesh);
    Vector<float3> verts;
    for (const NurbBodyEdgePolyline &polyline : polylines) {
      if (!polyline_is_surface(polyline) || !predicate(polyline)) {
        continue;
      }

      const int verts_before = verts.size();
      append_snapped_polyline_segments(
          positions, edges, edge_source_faces.as_span(), polyline.points.as_span(), verts);
      if (verts.size() == verts_before) {
        append_analytic_polyline_segments(polyline.points.as_span(), verts);
      }
    }

    return create_line_batch_from_segments(verts.as_span());
  }

  template<typename Predicate>
  static gpu::Batch *create_face_batch(const Span<NurbBodyFaceSurface> faces,
                                       Predicate &&predicate)
  {
    Vector<float3> verts;
    for (const NurbBodyFaceSurface &face : faces) {
      if (face.face_key == 0 || face.triangles.size() < 3 || !predicate(face)) {
        continue;
      }
      verts.extend(face.triangles.as_span());
    }

    return create_tri_batch_from_verts(verts.as_span());
  }

  static void draw_face_batch(gpu::Batch *batch, const float4 &color)
  {
    if (batch == nullptr) {
      return;
    }

    GPU_batch_program_set_builtin(batch, GPU_SHADER_3D_UNIFORM_COLOR);
    GPU_batch_uniform_4fv(batch, "color", &color.x);
    GPU_batch_draw(batch);
  }

  static void draw_batch(gpu::Batch *batch, const float4 &color, const float width)
  {
    if (batch == nullptr) {
      return;
    }

    float viewport[4];
    GPU_viewport_size_get_f(viewport);

    GPU_batch_program_set_builtin(batch, GPU_SHADER_3D_POLYLINE_UNIFORM_COLOR);
    GPU_batch_uniform_2f(batch, "viewportSize", viewport[2], viewport[3]);
    GPU_batch_uniform_1f(batch, "lineWidth", width);
    GPU_batch_uniform_1b(batch, "lineSmooth", true);
    GPU_batch_uniform_4fv(batch, "color", &color.x);
    GPU_batch_draw(batch);
  }

  DrawCache &cache_for_object(const Object *object)
  {
    for (DrawCache &cache : draw_caches_) {
      if (cache.object == object) {
        return cache;
      }
    }

    constexpr int max_draw_caches = 64;
    if (draw_caches_.size() >= max_draw_caches) {
      clear_draw_cache(draw_caches_.first());
      draw_caches_.remove_and_reorder(0);
    }

    DrawCache cache;
    cache.object = object;
    draw_caches_.append(cache);
    return draw_caches_.last();
  }

  static void rebuild_line_batches(DrawCache &cache,
                                   const Object *evaluated_object,
                                   const Object *object,
                                   const NurbBody &body,
                                   const Span<NurbBodyEdgePolyline> polylines,
                                   const int select_mode,
                                   const bool object_selected)
  {
    discard_batch(cache.normal_batch);
    discard_batch(cache.hovered_batch);
    discard_batch(cache.selected_batch);
    const Mesh &mesh = DRW_object_get_data_for_drawing<Mesh>(*evaluated_object);
    cache.normal_batch = create_line_batch(
        mesh, polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return !polyline_is_selected(object, body, polyline, select_mode, object_selected) &&
                 !polyline_is_hovered(object, body, polyline, select_mode, object_selected);
        });
    cache.hovered_batch = create_line_batch(
        mesh, polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return polyline_is_hovered(object, body, polyline, select_mode, object_selected) &&
                 !polyline_is_selected(object, body, polyline, select_mode, object_selected);
        });
    cache.selected_batch = create_line_batch(
        mesh, polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return polyline_is_selected(object, body, polyline, select_mode, object_selected);
        });
  }

  static void rebuild_face_batches(DrawCache &cache,
                                   const NurbBody &body,
                                   const Span<NurbBodyFaceSurface> faces,
                                   const int select_mode,
                                   const bool object_selected)
  {
    discard_batch(cache.face_hovered_batch);
    discard_batch(cache.face_selected_batch);
    cache.face_hovered_batch = create_face_batch(
        faces, [&](const NurbBodyFaceSurface &face) {
          return face_is_hovered(body, face, select_mode, object_selected) &&
                 !face_is_selected(body, face, select_mode, object_selected);
        });
    cache.face_selected_batch = create_face_batch(
        faces, [&](const NurbBodyFaceSurface &face) {
          return face_is_selected(body, face, select_mode, object_selected);
        });
  }

  DrawCache *ensure_cache_for_entry(const Entry &entry)
  {
    if (entry.original_object->data == nullptr || GS(entry.original_object->data->name) != ID_NB)
    {
      return nullptr;
    }

    const NurbBody *body = reinterpret_cast<const NurbBody *>(entry.original_object->data);
    const uint64_t geometry_key = BKE_nurb_body_boolean_edge_polylines_cache_key(
        entry.original_object, 64);
    const uint64_t face_geometry_key = BKE_nurb_body_face_surfaces_cache_key(
        entry.original_object);
    if (geometry_key == 0 && face_geometry_key == 0) {
      return nullptr;
    }

    DrawCache &cache = cache_for_object(entry.original_object);
    const bool geometry_changed = cache.geometry_key != geometry_key ||
                                  cache.face_geometry_key != face_geometry_key;
    if (geometry_changed) {
      clear_draw_cache(cache);
      cache.geometry_key = geometry_key;
      cache.face_geometry_key = face_geometry_key;
    }

    const bool object_selected = ((entry.original_object->base_flag | entry.object->base_flag) &
                                  BASE_SELECTED) != 0;

    const uint64_t selection_key = line_selection_key(
        entry.original_object, *body, select_mode_, object_selected);
    const uint64_t hover_key = line_hover_key(
        entry.original_object, *body, select_mode_, object_selected);
    if (geometry_key != 0 &&
        (geometry_changed || cache.selection_key != selection_key || cache.hover_key != hover_key))
    {
      const Span<NurbBodyEdgePolyline> polylines =
          BKE_nurb_body_boolean_edge_polylines_cached(entry.original_object, 64);
      rebuild_line_batches(
          cache,
          entry.object,
          entry.original_object,
          *body,
          polylines,
          select_mode_,
          object_selected);
      cache.selection_key = selection_key;
      cache.hover_key = hover_key;
    }

    const uint64_t face_select_key = face_selection_key(*body, select_mode_, object_selected);
    const uint64_t face_hover_state_key = face_hover_key(*body, select_mode_, object_selected);
    if (face_geometry_key != 0 &&
        (geometry_changed || cache.face_selection_key != face_select_key ||
         cache.face_hover_key != face_hover_state_key))
    {
      const Span<NurbBodyFaceSurface> faces = BKE_nurb_body_face_surfaces_cached(
          entry.original_object);
      rebuild_face_batches(cache, *body, faces, select_mode_, object_selected);
      cache.face_selection_key = face_select_key;
      cache.face_hover_key = face_hover_state_key;
    }

    return &cache;
  }

 public:
  ~NurbBodies()
  {
    for (DrawCache &cache : draw_caches_) {
      clear_draw_cache(cache);
    }
  }

  void begin_sync(Resources &res, const State &state) final
  {
    enabled_ = state.is_space_v3d() && !state.hide_overlays;
    entries_.clear();
    line_width_ = line_thickness_for_overlay(state.overlay);
    offset_data_ = state.offset_data_get();
    select_mode_ = NURB_BODY_SELECT_MODE_EDGE;
    if (state.scene != nullptr && state.scene->toolsettings != nullptr &&
        select_mode_is_valid(state.scene->toolsettings->nurb_body_select_mode))
    {
      select_mode_ = state.scene->toolsettings->nurb_body_select_mode;
    }
    edge_overlay_enabled_ = enabled_;
    xray_enabled_ = state.xray_enabled;
    needs_depth_prepass_ = edge_overlay_enabled_ && state.xray_enabled;
    depth_ps_.init();
    depth_mesh_ps_ = nullptr;

    if (needs_depth_prepass_) {
      depth_ps_.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
      depth_ps_.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
      depth_ps_.state_set(DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL,
                          state.clipping_plane_count);

      auto &sub = depth_ps_.sub("Mesh");
      sub.shader_set(res.shaders->depth_mesh.get());
      depth_mesh_ps_ = &sub;
    }
  }

  void object_sync(Manager &manager,
                   const ObjectRef &ob_ref,
                   Resources & /*res*/,
                   const State & /*state*/) final
  {
    if (!edge_overlay_enabled_ || ob_ref.object->type != OB_NURB_BODY ||
        ob_ref.object->data == nullptr)
    {
      return;
    }

    const Object *original_object = DEG_get_original(ob_ref.object);
    if (original_object == nullptr || original_object->type != OB_NURB_BODY ||
        original_object->data == nullptr || GS(original_object->data->name) != ID_NB)
    {
      return;
    }

    entries_.append({ob_ref.object, original_object});

    if (depth_mesh_ps_ != nullptr) {
      if (gpu::Batch *geom = DRW_cache_mesh_surface_get(ob_ref.object)) {
        depth_mesh_ps_->draw(geom, manager.unique_handle(ob_ref));
      }
    }
  }

  void draw_depth_prepass(Framebuffer &framebuffer, Manager &manager, View &view)
  {
    if (!enabled_ || !needs_depth_prepass_ || entries_.is_empty()) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    manager.submit(depth_ps_, view);
  }

  void draw_on_render(gpu::FrameBuffer *framebuffer, Manager & /*manager*/, View & /*view*/) final
  {
    if (!edge_overlay_enabled_ || entries_.is_empty() ||
        select_mode_ != NURB_BODY_SELECT_MODE_FACE || framebuffer == nullptr)
    {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    GPU_depth_mask(false);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_polygon_offset(offset_data_.dist, 8.0f);

    for (const Entry &entry : entries_) {
      DrawCache *cache = ensure_cache_for_entry(entry);
      if (cache == nullptr) {
        continue;
      }

      GPU_matrix_push();
      GPU_matrix_mul(entry.object->object_to_world().ptr());

      /* Tint the already-rendered shaded surface instead of drawing a second shaded surface. */
      GPU_blend(GPU_BLEND_MULTIPLY);
      draw_face_batch(cache->face_selected_batch, float4(1.0f, 0.90f, 0.66f, 1.0f));

      GPU_blend(GPU_BLEND_ALPHA);
      draw_face_batch(cache->face_hovered_batch, float4(1.0f, 1.0f, 1.0f, 0.08f));
      draw_face_batch(cache->face_selected_batch, float4(1.0f, 0.74f, 0.24f, 0.14f));

      GPU_matrix_pop();
    }

    GPU_polygon_offset(0.0f, 0.0f);
    GPU_blend(GPU_BLEND_NONE);
    GPU_depth_mask(true);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
  }

  void draw_line(Framebuffer &framebuffer, Manager & /*manager*/, View & /*view*/) final
  {
    if (!edge_overlay_enabled_ || entries_.is_empty()) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_polygon_offset(offset_data_.dist, 1.0f);

    const float xray_occluded_alpha = 0.25f;
    const float normal_alpha = 1.0f;

    for (const Entry &entry : entries_) {
      DrawCache *cache = ensure_cache_for_entry(entry);
      if (cache == nullptr) {
        continue;
      }

      GPU_matrix_push();
      GPU_matrix_mul(entry.object->object_to_world().ptr());
      if (xray_enabled_) {
        GPU_depth_test(GPU_DEPTH_ALWAYS);
        draw_batch(
            cache->normal_batch, float4(0.0f, 0.0f, 0.0f, xray_occluded_alpha), line_width_);
        draw_batch(
            cache->hovered_batch, float4(1.0f, 0.78f, 0.18f, xray_occluded_alpha), line_width_);
        draw_batch(
            cache->selected_batch, float4(1.0f, 0.58f, 0.08f, xray_occluded_alpha), line_width_);
      }

      GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
      draw_batch(cache->normal_batch, float4(0.0f, 0.0f, 0.0f, normal_alpha), line_width_);
      draw_batch(cache->hovered_batch, float4(1.0f, 0.78f, 0.18f, normal_alpha), line_width_);
      draw_batch(cache->selected_batch, float4(1.0f, 0.58f, 0.08f, normal_alpha), line_width_);
      GPU_matrix_pop();
    }

    GPU_polygon_offset(0.0f, 0.0f);
    GPU_blend(GPU_BLEND_NONE);
    GPU_depth_mask(true);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
  }

};

}  // namespace blender::draw::overlay
