/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "DNA_nurb_body_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BLI_array.hh"
#include "BLI_math_matrix.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_mesh.hh"
#include "BKE_nurb_body.hh"

#include "DEG_depsgraph_query.hh"

#include "DRW_render.hh"

#include "GPU_batch.hh"
#include "GPU_matrix.hh"
#include "GPU_state.hh"

#include "overlay_base.hh"

namespace blender::draw::overlay {

class NurbBodies : Overlay {
 private:
  PassMain depth_ps_ = {"NurbBodyDepth"};
  PassMain::Sub *depth_mesh_ps_ = nullptr;

  struct Entry {
    const Object *object;
    const Object *original_object;
  };

  struct DrawCache {
    const Object *object = nullptr;
    uint64_t geometry_key = 0;
    uint64_t selection_key = 0;
    uint64_t hover_key = 0;
    uint64_t silhouette_key = 0;
    gpu::Batch *silhouette_batch = nullptr;
    gpu::Batch *normal_batch = nullptr;
    gpu::Batch *hovered_batch = nullptr;
    gpu::Batch *selected_batch = nullptr;
  };

  Vector<Entry> entries_;
  Vector<DrawCache> draw_caches_;
  float line_width_ = 2.0f;
  bool xray_flag_enabled_ = false;
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

  static uint64_t line_selection_key(const NurbBody &body)
  {
    uint64_t hash = 1469598103934665603ull;
    hash_value(hash, body.selected_edges);
    hash_value(hash, body.selected_edge);
    hash_value(hash, body.surface_selected_edges);
    hash_value(hash, body.surface_selected_edge);
    hash_bytes(hash, body.surface_edge_keys, sizeof(body.surface_edge_keys));

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

  static uint64_t line_hover_key(const Object *object, const NurbBody &body)
  {
    uint64_t hash = 1469598103934665603ull;
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

  static float line_thickness_for_overlay(const View3DOverlay &overlay)
  {
    if (!std::isfinite(overlay.nurb_body_line_thickness) ||
        overlay.nurb_body_line_thickness <= 0.0f)
    {
      return 2.0f;
    }
    return std::clamp(overlay.nurb_body_line_thickness, 0.5f, 8.0f);
  }

  static bool body_has_edge_selection(const NurbBody &body)
  {
    if (body.selected_edges != 0 || body.surface_selected_edges != 0) {
      return true;
    }
    for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(
             body.boolean_ops.first);
         op;
         op = op->next)
    {
      if (op->selected_edges != 0) {
        return true;
      }
    }
    return false;
  }

  static bool draw_silhouette_selected(const Object &object, const NurbBody &body)
  {
    return !body_has_edge_selection(body) && (object.base_flag & BASE_SELECTED) != 0;
  }

  static bool polyline_is_surface(const NurbBodyEdgePolyline &polyline)
  {
    return (polyline.flag & NURB_BODY_EDGE_POLYLINE_SURFACE) != 0 &&
           (polyline.flag & NURB_BODY_EDGE_POLYLINE_SELECTABLE) != 0 &&
           polyline.points.size() >= 2;
  }

  static bool polyline_is_selected(const NurbBody &body, const NurbBodyEdgePolyline &polyline)
  {
    const NurbBodyBooleanOp *op = polyline.op;
    const bool body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
    const bool surface_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0;
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
                                  const NurbBodyEdgePolyline &polyline)
  {
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
    discard_batch(cache.silhouette_batch);
    discard_batch(cache.normal_batch);
    discard_batch(cache.hovered_batch);
    discard_batch(cache.selected_batch);
    cache.geometry_key = 0;
    cache.selection_key = 0;
    cache.hover_key = 0;
    cache.silhouette_key = 0;
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

  template<typename Predicate>
  static gpu::Batch *create_line_batch(const Span<NurbBodyEdgePolyline> polylines,
                                       Predicate &&predicate)
  {
    Vector<float3> verts;
    for (const NurbBodyEdgePolyline &polyline : polylines) {
      if (!polyline_is_surface(polyline) || !predicate(polyline)) {
        continue;
      }
      for (int i = 1; i < polyline.points.size(); i++) {
        verts.append(polyline.points[i - 1]);
        verts.append(polyline.points[i]);
      }
    }

    return create_line_batch_from_segments(verts.as_span());
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
                                   const Object *object,
                                   const NurbBody &body,
                                   const Span<NurbBodyEdgePolyline> polylines)
  {
    discard_batch(cache.normal_batch);
    discard_batch(cache.hovered_batch);
    discard_batch(cache.selected_batch);
    cache.normal_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return !polyline_is_selected(body, polyline) &&
                 !polyline_is_hovered(object, body, polyline);
        });
    cache.hovered_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return polyline_is_hovered(object, body, polyline) &&
                 !polyline_is_selected(body, polyline);
        });
    cache.selected_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return polyline_is_selected(body, polyline);
        });
  }

  static void hash_quantized_float(uint64_t &hash, const float value, const float scale)
  {
    const int64_t quantized = int64_t(std::llround(double(value) * double(scale)));
    hash_value(hash, quantized);
  }

  static void hash_quantized_float3(uint64_t &hash, const float3 &value, const float scale)
  {
    hash_quantized_float(hash, value.x, scale);
    hash_quantized_float(hash, value.y, scale);
    hash_quantized_float(hash, value.z, scale);
  }

  static float3 view_direction_for_edge(const Object &object,
                                        const View &view,
                                        const float3 &edge_midpoint)
  {
    const float4x4 world_to_object = math::invert(object.object_to_world());
    if (view.is_persp()) {
      const float3 camera_local = math::transform_point(world_to_object, view.location());
      return math::normalize(camera_local - edge_midpoint);
    }
    return math::normalize(math::transform_direction(world_to_object, view.forward()));
  }

  static uint64_t silhouette_view_key(const Object &object,
                                      const View &view,
                                      const uint64_t geometry_key)
  {
    uint64_t hash = geometry_key;
    hash_value(hash, view.is_persp());
    const float4x4 world_to_object = math::invert(object.object_to_world());
    if (view.is_persp()) {
      hash_quantized_float3(hash,
                            math::transform_point(world_to_object, view.location()),
                            4096.0f);
    }
    else {
      hash_quantized_float3(hash,
                            math::normalize(math::transform_direction(world_to_object,
                                                                      view.forward())),
                            65536.0f);
    }
    return hash;
  }

  static gpu::Batch *create_mesh_silhouette_batch(const Object &object, const View &view)
  {
    const Mesh &mesh = DRW_object_get_data_for_drawing<Mesh>(object);
    const Span<float3> positions = mesh.vert_positions();
    const Span<int2> edges = mesh.edges();
    const OffsetIndices<int> faces = mesh.faces();
    const Span<int> corner_edges = mesh.corner_edges();
    const Span<float3> face_normals = mesh.face_normals();
    if (positions.is_empty() || edges.is_empty() || faces.is_empty() || corner_edges.is_empty()) {
      return nullptr;
    }

    Array<int2> edge_faces(edges.size(), int2(-1, -1));
    for (const int face_index : faces.index_range()) {
      for (const int corner : faces[face_index]) {
        const int edge_index = corner_edges[corner];
        if (edge_index < 0 || edge_index >= edge_faces.size()) {
          continue;
        }
        int2 &linked_faces = edge_faces[edge_index];
        if (linked_faces[0] == -1) {
          linked_faces[0] = face_index;
        }
        else if (linked_faces[1] == -1 && linked_faces[0] != face_index) {
          linked_faces[1] = face_index;
        }
      }
    }

    Vector<float3> verts;
    for (const int edge_index : edges.index_range()) {
      const int2 linked_faces = edge_faces[edge_index];
      if (linked_faces[0] == -1 || linked_faces[1] == -1) {
        continue;
      }

      const int2 edge = edges[edge_index];
      if (edge[0] < 0 || edge[0] >= positions.size() || edge[1] < 0 ||
          edge[1] >= positions.size())
      {
        continue;
      }

      const float adjacent_face_dot = math::dot(face_normals[linked_faces[0]],
                                               face_normals[linked_faces[1]]);
      if (adjacent_face_dot <= 0.75f) {
        continue;
      }

      const float3 midpoint = (positions[edge[0]] + positions[edge[1]]) * 0.5f;
      const float3 view_direction = view_direction_for_edge(object, view, midpoint);
      const float facing_a = math::dot(face_normals[linked_faces[0]], view_direction);
      const float facing_b = math::dot(face_normals[linked_faces[1]], view_direction);
      if ((facing_a <= 0.0f && facing_b > 0.0f) || (facing_a > 0.0f && facing_b <= 0.0f)) {
        verts.append(positions[edge[0]]);
        verts.append(positions[edge[1]]);
      }
    }
    return create_line_batch_from_segments(verts.as_span());
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
    xray_flag_enabled_ = state.xray_flag_enabled;
    needs_depth_prepass_ = enabled_ && state.xray_enabled;
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
    if (!enabled_ || ob_ref.object->type != OB_NURB_BODY || ob_ref.object->data == nullptr) {
      return;
    }

    const Object *original_object = DEG_get_original(ob_ref.object);
    if (original_object == nullptr || original_object->type != OB_NURB_BODY ||
        original_object->data == nullptr)
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

  void draw_line(Framebuffer &framebuffer, Manager & /*manager*/, View &view) final
  {
    if (!enabled_ || entries_.is_empty()) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    GPU_depth_test(xray_flag_enabled_ ? GPU_DEPTH_NONE : GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_line_smooth(true);

    for (const Entry &entry : entries_) {
      const NurbBody *body = reinterpret_cast<const NurbBody *>(entry.original_object->data);
      const uint64_t geometry_key = BKE_nurb_body_boolean_edge_polylines_cache_key(
          entry.original_object, 64);
      if (geometry_key == 0) {
        continue;
      }

      DrawCache &cache = cache_for_object(entry.original_object);
      const bool geometry_changed = cache.geometry_key != geometry_key;
      if (geometry_changed) {
        clear_draw_cache(cache);
        cache.geometry_key = geometry_key;
      }

      const uint64_t selection_key = line_selection_key(*body);
      const uint64_t hover_key = line_hover_key(entry.original_object, *body);
      if (geometry_changed || cache.selection_key != selection_key ||
          cache.hover_key != hover_key)
      {
        const Span<NurbBodyEdgePolyline> polylines =
            BKE_nurb_body_boolean_edge_polylines_cached(entry.original_object, 64);
        rebuild_line_batches(cache, entry.original_object, *body, polylines);
        cache.selection_key = selection_key;
        cache.hover_key = hover_key;
      }

      const uint64_t silhouette_key = silhouette_view_key(
          *entry.object, view, cache.geometry_key);
      if (geometry_changed || cache.silhouette_key != silhouette_key) {
        discard_batch(cache.silhouette_batch);
        cache.silhouette_batch = create_mesh_silhouette_batch(*entry.object, view);
        cache.silhouette_key = silhouette_key;
      }

      GPU_matrix_push();
      GPU_matrix_mul(entry.object->object_to_world().ptr());
      GPU_polygon_offset(1.0f, 2.0f);
      draw_batch(cache.normal_batch, float4(0.0f, 0.0f, 0.0f, 0.9f), line_width_);
      const bool selected_silhouette = draw_silhouette_selected(*entry.original_object, *body);
      draw_batch(cache.silhouette_batch,
                 selected_silhouette ? float4(1.0f, 0.62f, 0.0f, 1.0f) :
                                       float4(0.0f, 0.0f, 0.0f, 0.9f),
                 line_width_);
      draw_batch(cache.hovered_batch, float4(1.0f, 1.0f, 1.0f, 1.0f), line_width_);
      draw_batch(cache.selected_batch, float4(1.0f, 0.62f, 0.0f, 1.0f), line_width_);
      GPU_polygon_offset(0.0f, 0.0f);
      GPU_matrix_pop();
    }

    GPU_polygon_offset(0.0f, 0.0f);
    GPU_line_smooth(false);
    GPU_blend(GPU_BLEND_NONE);
    GPU_depth_mask(true);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
  }
};

}  // namespace blender::draw::overlay
