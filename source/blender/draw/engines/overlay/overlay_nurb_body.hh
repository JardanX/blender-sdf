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
#include "DNA_scene_types.h"

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
    gpu::Batch *normal_batch = nullptr;
    gpu::Batch *hovered_batch = nullptr;
    gpu::Batch *selected_batch = nullptr;
  };

  Vector<Entry> entries_;
  Vector<DrawCache> draw_caches_;
  float line_width_ = 2.0f;
  int select_mode_ = NURB_BODY_SELECT_MODE_EDGE;
  bool edge_overlay_enabled_ = false;
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

  static bool select_mode_is_valid(const int select_mode)
  {
    return select_mode == NURB_BODY_SELECT_MODE_EDGE ||
           select_mode == NURB_BODY_SELECT_MODE_FACE ||
           select_mode == NURB_BODY_SELECT_MODE_OBJECT;
  }

  static uint64_t line_selection_key(const NurbBody &body,
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

  static float line_thickness_for_overlay(const View3DOverlay &overlay)
  {
    if (!std::isfinite(overlay.nurb_body_line_thickness) ||
        overlay.nurb_body_line_thickness <= 0.0f)
    {
      return 2.0f;
    }
    return std::clamp(overlay.nurb_body_line_thickness, 0.5f, 8.0f);
  }

  static bool edge_selection_mode_enabled(const int select_mode, const bool object_selected)
  {
    return select_mode == NURB_BODY_SELECT_MODE_EDGE && !object_selected;
  }

  static bool polyline_is_surface(const NurbBodyEdgePolyline &polyline)
  {
    return (polyline.flag & NURB_BODY_EDGE_POLYLINE_SURFACE) != 0 && polyline.points.size() >= 2;
  }

  static bool polyline_is_selected(const NurbBody &body,
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
    cache.geometry_key = 0;
    cache.selection_key = 0;
    cache.hover_key = 0;
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
                                   const Span<NurbBodyEdgePolyline> polylines,
                                   const int select_mode,
                                   const bool object_selected)
  {
    discard_batch(cache.normal_batch);
    discard_batch(cache.hovered_batch);
    discard_batch(cache.selected_batch);
    cache.normal_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return !polyline_is_selected(body, polyline, select_mode, object_selected) &&
                 !polyline_is_hovered(object, body, polyline, select_mode, object_selected);
        });
    cache.hovered_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return polyline_is_hovered(object, body, polyline, select_mode, object_selected) &&
                 !polyline_is_selected(body, polyline, select_mode, object_selected);
        });
    cache.selected_batch = create_line_batch(
        polylines, [&](const NurbBodyEdgePolyline &polyline) {
          return polyline_is_selected(body, polyline, select_mode, object_selected);
        });
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
    select_mode_ = NURB_BODY_SELECT_MODE_EDGE;
    if (state.scene != nullptr && state.scene->toolsettings != nullptr &&
        select_mode_is_valid(state.scene->toolsettings->nurb_body_select_mode))
    {
      select_mode_ = state.scene->toolsettings->nurb_body_select_mode;
    }
    edge_overlay_enabled_ = enabled_;
    xray_flag_enabled_ = state.xray_flag_enabled;
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
    if (!edge_overlay_enabled_ || entries_.is_empty()) {
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

      const bool object_selected = ((entry.original_object->base_flag | entry.object->base_flag) &
                                    BASE_SELECTED) != 0;

      const uint64_t selection_key = line_selection_key(*body, select_mode_, object_selected);
      const uint64_t hover_key = line_hover_key(
          entry.original_object, *body, select_mode_, object_selected);
      if (geometry_changed || cache.selection_key != selection_key ||
          cache.hover_key != hover_key)
      {
        const Span<NurbBodyEdgePolyline> polylines =
            BKE_nurb_body_boolean_edge_polylines_cached(entry.original_object, 64);
        rebuild_line_batches(
            cache, entry.original_object, *body, polylines, select_mode_, object_selected);
        cache.selection_key = selection_key;
        cache.hover_key = hover_key;
      }

      GPU_matrix_push();
      GPU_matrix_mul(entry.object->object_to_world().ptr());
      GPU_polygon_offset(1.0f, 2.0f);
      draw_batch(cache.normal_batch, float4(0.0f, 0.0f, 0.0f, 0.9f), line_width_);
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
