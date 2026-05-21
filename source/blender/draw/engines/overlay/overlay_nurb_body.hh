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
#include "DNA_object_types.h"

#include "BLI_math_matrix.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_nurb_body.hh"

#include "GPU_batch.hh"
#include "GPU_matrix.hh"
#include "GPU_state.hh"

#include "overlay_base.hh"

namespace blender::draw::overlay {

class NurbBodies : Overlay {
 private:
  struct Entry {
    const Object *object;
  };

  struct DrawCache {
    const Object *object = nullptr;
    uint64_t geometry_key = 0;
    uint64_t state_key = 0;
    gpu::Batch *normal_batch = nullptr;
    gpu::Batch *hovered_batch = nullptr;
    gpu::Batch *selected_batch = nullptr;
  };

  Vector<Entry> entries_;
  Vector<DrawCache> draw_caches_;
  float line_width_ = 2.0f;

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

  static uint64_t line_state_key(const NurbBody &body)
  {
    uint64_t hash = 1469598103934665603ull;
    hash_value(hash, body.selected_edges);
    hash_value(hash, body.selected_edge);
    hash_value(hash, body.hovered_edge);
    hash_value(hash, body.surface_selected_edges);
    hash_value(hash, body.surface_selected_edge);
    hash_value(hash, body.surface_hovered_edge);

    int op_count = 0;
    for (const NurbBodyBooleanOp *op = static_cast<const NurbBodyBooleanOp *>(
             body.boolean_ops.first);
         op;
         op = op->next)
    {
      op_count++;
      hash_value(hash, op->flag);
      hash_value(hash, op->selected_edges);
      hash_value(hash, op->selected_edge);
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

  static bool polyline_is_surface(const NurbBodyEdgePolyline &polyline)
  {
    return (polyline.flag & NURB_BODY_EDGE_POLYLINE_SURFACE) != 0 &&
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
      return edge_index_in_mask(body.surface_selected_edges, polyline.edge_index);
    }
    return op != nullptr && polyline.edge_index >= 0 &&
           (op->flag & NURB_BODY_BOOLEAN_OP_SELECTED) != 0 &&
           edge_index_in_mask(op->selected_edges, polyline.edge_index);
  }

  static bool polyline_is_hovered(const NurbBody &body, const NurbBodyEdgePolyline &polyline)
  {
    const NurbBodyBooleanOp *op = polyline.op;
    const bool body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
    const bool surface_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_FINAL) != 0;
    if (body_edge) {
      return body.hovered_edge == polyline.edge_index;
    }
    if (surface_edge) {
      return body.surface_hovered_edge == polyline.edge_index;
    }
    return op != nullptr && polyline.edge_index >= 0 &&
           (op->flag & NURB_BODY_BOOLEAN_OP_HOVERED) != 0 &&
           op->hovered_edge == polyline.edge_index;
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
    cache.geometry_key = 0;
    cache.state_key = 0;
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

    if (verts.is_empty()) {
      return nullptr;
    }

    GPUVertFormat format = {0};
    GPU_vertformat_attr_add(&format, "pos", gpu::VertAttrType::SFLOAT_32_32_32);
    gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
    GPU_vertbuf_data_alloc(*vbo, verts.size());
    vbo->data<float3>().copy_from(verts.as_span());
    return GPU_batch_create_ex(GPU_PRIM_LINES, vbo, nullptr, GPU_BATCH_OWNS_VBO);
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

  static void rebuild_geometry_batch(DrawCache &cache,
                                     const Span<NurbBodyEdgePolyline> /*polylines*/)
  {
    discard_batch(cache.normal_batch);
    discard_batch(cache.hovered_batch);
    discard_batch(cache.selected_batch);
    cache.state_key = 0;
  }

  static void rebuild_state_batches(DrawCache &cache,
                                    const NurbBody &body,
                                    const Span<NurbBodyEdgePolyline> polylines)
  {
    discard_batch(cache.normal_batch);
    discard_batch(cache.hovered_batch);
    discard_batch(cache.selected_batch);
    cache.normal_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return !polyline_is_hovered(body, polyline) && !polyline_is_selected(body, polyline);
        });
    cache.hovered_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return polyline_is_hovered(body, polyline) && !polyline_is_selected(body, polyline);
        });
    cache.selected_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return polyline_is_selected(body, polyline);
        });
  }

 public:
  ~NurbBodies()
  {
    for (DrawCache &cache : draw_caches_) {
      clear_draw_cache(cache);
    }
  }

  void begin_sync(Resources & /*res*/, const State &state) final
  {
    enabled_ = state.is_space_v3d() && !state.hide_overlays;
    entries_.clear();
    line_width_ = line_thickness_for_overlay(state.overlay);
  }

  void object_sync(Manager & /*manager*/,
                   const ObjectRef &ob_ref,
                   Resources & /*res*/,
                   const State & /*state*/) final
  {
    if (!enabled_ || ob_ref.object->type != OB_NURB_BODY || ob_ref.object->data == nullptr) {
      return;
    }

    entries_.append({ob_ref.object});
  }

  void draw_line(Framebuffer &framebuffer, Manager & /*manager*/, View & /*view*/) final
  {
    if (!enabled_ || entries_.is_empty()) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_line_smooth(true);
    GPU_matrix_push_projection();
    GPU_polygon_offset(1.0f, 2.0f);

    for (const Entry &entry : entries_) {
      const NurbBody *body = reinterpret_cast<const NurbBody *>(entry.object->data);
      const uint64_t geometry_key = BKE_nurb_body_boolean_edge_polylines_cache_key(entry.object,
                                                                                   64);
      if (geometry_key == 0) {
        continue;
      }

      DrawCache &cache = cache_for_object(entry.object);
      if (cache.geometry_key != geometry_key) {
        const Span<NurbBodyEdgePolyline> polylines =
            BKE_nurb_body_boolean_edge_polylines_cached(entry.object, 64);
        rebuild_geometry_batch(cache, polylines);
        cache.geometry_key = geometry_key;
      }

      const uint64_t state_key = line_state_key(*body);
      if (cache.state_key != state_key) {
        const Span<NurbBodyEdgePolyline> polylines =
            BKE_nurb_body_boolean_edge_polylines_cached(entry.object, 64);
        rebuild_state_batches(cache, *body, polylines);
        cache.state_key = state_key;
      }

      GPU_matrix_push();
      GPU_matrix_mul(entry.object->object_to_world().ptr());
      draw_batch(cache.normal_batch, float4(0.0f, 0.0f, 0.0f, 0.9f), line_width_);
      GPU_depth_test(GPU_DEPTH_NONE);
      draw_batch(cache.hovered_batch, float4(1.0f, 1.0f, 1.0f, 1.0f), line_width_);
      draw_batch(cache.selected_batch, float4(1.0f, 0.62f, 0.0f, 1.0f), line_width_);
      GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
      GPU_matrix_pop();
    }

    GPU_matrix_pop_projection();
    GPU_line_smooth(false);
    GPU_blend(GPU_BLEND_NONE);
    GPU_depth_mask(true);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
  }
};

}  // namespace blender::draw::overlay
