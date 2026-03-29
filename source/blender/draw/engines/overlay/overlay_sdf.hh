/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 * SDF overlay: outline via G-buffer readback + GPU picking via sphere-trace.
 */

#pragma once

#include "BLI_assert.h"
#include "BLI_vector.hh"

#include "MEM_guardedalloc.h"

#include "DNA_object_types.h"
#include "DNA_sdf_types.h"

#include "GPU_batch.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_uniform_buffer.hh"

#include "draw_view.hh"

#include "overlay_base.hh"

#include "BLI_rect.h"

#include "engines/sdf/sdf_engine.h"
#include "engines/select/select_defines.hh"
#include "gpu_select_private.hh"

namespace blender::draw::overlay {

class Sdfs : Overlay {
 private:
  const SelectionType selection_type_;

  /* Per-SDF object data collected during object_sync. */
  struct SdfEntry {
    int sdf_index;
    uint32_t outline_packed_id;
    uint32_t select_id;
    float4x4 object_to_world;
    float3 local_extent;
  };
  Vector<SdfEntry> entries_;
  int max_sdf_index_ = 0;
  bool has_selected_ = false;

  Vector<uint32_t> select_table_;
  int select_buf_size_ = 0;

  gpu::Shader *outline_sh_ = nullptr;
  gpu::Batch *fullscreen_batch_ = nullptr;

  /* SSBO of sorted GPU indices of selected objects only (for outline trace). */
  gpu::StorageBuf *selected_indices_ssbo_ = nullptr;

  gpu::StorageBuf *outline_ids_ssbo_ = nullptr;
  gpu::StorageBuf **select_output_buf_ = nullptr;

 public:
  Sdfs(const SelectionType selection_type) : selection_type_(selection_type) {};

  ~Sdfs()
  {
    if (outline_sh_) {
      GPU_shader_free(outline_sh_);
    }
    if (outline_ids_ssbo_) {
      GPU_storagebuf_free(outline_ids_ssbo_);
    }
    if (selected_indices_ssbo_) {
      GPU_storagebuf_free(selected_indices_ssbo_);
    }
    if (fullscreen_batch_) {
      GPU_batch_discard(fullscreen_batch_);
    }
  }

  void begin_sync(Resources & /*res*/, const State &state) final
  {
    enabled_ = state.is_space_v3d();
    if (!enabled_) {
      return;
    }
    entries_.clear();
    max_sdf_index_ = 0;
    has_selected_ = false;
  }

  void object_sync(Manager & /*manager*/,
                   const ObjectRef &ob_ref,
                   Resources &res,
                   const State &state) final
  {
    if (!enabled_) {
      return;
    }
    if (ob_ref.object->type != OB_SDF) {
      return;
    }

    const SDF *sdf_data = reinterpret_cast<const SDF *>(ob_ref.object->data);
    if (!sdf_data) {
      return;
    }

    const bool is_select = (ob_ref.object->base_flag & BASE_SELECTED) != 0;
    const bool is_active = (ob_ref.object == state.object_active);

    uint outline_color_id = 0;
    if (is_active) {
      outline_color_id = 3u;
    }
    else if (is_select) {
      outline_color_id = 1u;
    }

    uint resource_id = uint(entries_.size()) + 1u;
    uint packed_id = is_select ? ((outline_color_id << 14u) | (resource_id & 0x3FFFu)) : 0u;

    const select::ID sel_id = res.select_id(ob_ref);

    int idx = sdf_data->sdf_index;
    if (idx > max_sdf_index_) {
      max_sdf_index_ = idx;
    }
    if (packed_id != 0u) {
      has_selected_ = true;
    }

    float4x4 obmat = float4x4(ob_ref.object->object_to_world());
    float3 ext = float3(sdf_data->size[0], sdf_data->size[1], sdf_data->size[2]);
    entries_.append({idx, packed_id, sel_id.get(), obmat, ext});
  }

  void end_sync(Resources &res, const State & /*state*/) final
  {
    if (!enabled_) {
      return;
    }
    if (entries_.is_empty()) {
      return;
    }

    if (!outline_sh_) {
      outline_sh_ = GPU_shader_create_from_info_name("sdf_outline_prepass");
      if (!outline_sh_) {
        printf("SDF: outline shader FAILED to compile!\n");
      }
    }

    select_output_buf_ = &res.select_output_buf;
    select_buf_size_ = math::max(int(res.select_id_map.size()), 4);
    select_buf_size_ = (select_buf_size_ + 3) & ~3;

    /* Build tables indexed by sdf_index (= original_index in GPU). */
    int table_size = max_sdf_index_ + 1;
    Vector<uint32_t> outline_table(table_size, 0u);
    select_table_.reinitialize(table_size);
    select_table_.fill(uint32_t(-1));
    for (const SdfEntry &e : entries_) {
      if (e.sdf_index >= 0 && e.sdf_index < table_size) {
        outline_table[e.sdf_index] = e.outline_packed_id;
        select_table_[e.sdf_index] = e.select_id;
      }
    }

    /* Build list of GPU array indices for selected objects. */
    int obj_count = sdf::sdf_object_count_get();
    int map_count = 0;
    const int *dg_to_sorted = sdf::sdf_depsgraph_to_sorted_get(&map_count);
    Vector<int32_t> sel_indices;
    if (dg_to_sorted && map_count == int(entries_.size())) {
      for (int i = 0; i < map_count; i++) {
        if (entries_[i].outline_packed_id != 0u) {
          int si = dg_to_sorted[i];
          if (si >= 0 && si < obj_count) {
            sel_indices.append(si);
          }
        }
      }
    }
    if (selected_indices_ssbo_) {
      GPU_storagebuf_free(selected_indices_ssbo_);
      selected_indices_ssbo_ = nullptr;
    }
    if (!sel_indices.is_empty()) {
      selected_indices_ssbo_ = GPU_storagebuf_create_ex(
          sel_indices.size() * sizeof(int32_t),
          sel_indices.data(),
          GPU_USAGE_STATIC,
          "sdf_selected_indices");
    }

    if (outline_ids_ssbo_) {
      GPU_storagebuf_free(outline_ids_ssbo_);
    }
    outline_ids_ssbo_ = GPU_storagebuf_create_ex(
        outline_table.size() * sizeof(uint32_t),
        outline_table.data(),
        GPU_USAGE_STATIC,
        "sdf_outline_ids");

  }

  void draw_outline(Framebuffer &framebuffer, View & /*view*/, gpu::Texture * /*scene_depth*/)
  {
    if (!enabled_ || !has_selected_) {
      return;
    }
    if (!outline_sh_ || !outline_ids_ssbo_) {
      return;
    }

    gpu::Texture *depth_tx = sdf::sdf_depth_texture_get();
    gpu::Texture *gbuf_tx = sdf::sdf_gbuf_color_texture_get();
    if (!depth_tx || !gbuf_tx) {
      return;
    }

    if (!fullscreen_batch_) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }

    GPU_framebuffer_bind(framebuffer);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);

    GPU_shader_bind(outline_sh_);

    GPU_storagebuf_bind(
        outline_ids_ssbo_, GPU_shader_get_ssbo_binding(outline_sh_, "outline_ids"));

    GPU_texture_filter_mode(depth_tx, true);
    GPU_texture_bind(depth_tx, GPU_shader_get_sampler_binding(outline_sh_, "sdf_depth_tx"));

    GPU_texture_filter_mode(gbuf_tx, false);
    GPU_texture_bind(gbuf_tx, GPU_shader_get_sampler_binding(outline_sh_, "sdf_gbuf_color_tx"));

    float2 uv_sc = sdf::sdf_uv_scale_get();
    GPU_shader_uniform_2fv(outline_sh_, "uv_scale", &uv_sc.x);

    GPU_batch_set_shader(fullscreen_batch_, outline_sh_);
    GPU_batch_draw(fullscreen_batch_);

    GPU_texture_unbind(depth_tx);
    GPU_texture_unbind(gbuf_tx);
    GPU_shader_unbind();
  }

  void draw(Framebuffer & /*framebuffer*/, Manager & /*manager*/, View & /*view*/) final
  {
    if (selection_type_ == SelectionType::DISABLED) {
      return;
    }
    if (!enabled_ || entries_.is_empty()) {
      return;
    }
    if (select_table_.is_empty() || !select_output_buf_ || !*select_output_buf_) {
      return;
    }

    gpu::Texture *gbuf_color_tx = sdf::sdf_gbuf_color_texture_get();
    gpu::Texture *depth_tx = sdf::sdf_depth_texture_get();
    if (!gbuf_color_tx || !depth_tx) {
      return;
    }

    /* Get the pick area in viewport coordinates. */
    rcti pick_rect = gpu_select_next_get_pick_area_rect();
    int vp_center_x = (pick_rect.xmin + pick_rect.xmax) / 2;
    int vp_center_y = (pick_rect.ymin + pick_rect.ymax) / 2;

    /* Map viewport coords to G-buffer texel via uv_scale. */
    float2 uv_sc = sdf::sdf_uv_scale_get();
    int gx = int(float(vp_center_x) * uv_sc.x);
    int gy = int(float(vp_center_y) * uv_sc.y);
    int gbuf_w = GPU_texture_width(gbuf_color_tx);
    int gbuf_h = GPU_texture_height(gbuf_color_tx);
    gx = math::clamp(gx, 0, gbuf_w - 1);
    gy = math::clamp(gy, 0, gbuf_h - 1);

    /* Read G-buffer at cursor pixel. */
    float *gbuf_data = static_cast<float *>(
        GPU_texture_read(gbuf_color_tx, GPU_DATA_FLOAT, 0));
    float *depth_data = static_cast<float *>(
        GPU_texture_read(depth_tx, GPU_DATA_FLOAT, 0));

    if (!gbuf_data || !depth_data) {
      if (gbuf_data) { MEM_delete_void(static_cast<void *>(gbuf_data)); }
      if (depth_data) { MEM_delete_void(static_cast<void *>(depth_data)); }
      return;
    }

    float obj_id_f = gbuf_data[(gy * gbuf_w + gx) * 4 + 3];
    float sdf_depth = depth_data[gy * gbuf_w + gx];

    MEM_delete_void(static_cast<void *>(gbuf_data));
    MEM_delete_void(static_cast<void *>(depth_data));

    if (sdf_depth <= 0.0f || sdf_depth >= 1.0f) {
      return;
    }

    int obj_id = int(obj_id_f + 0.5f);
    if (obj_id < 0 || obj_id >= int(select_table_.size())) {
      return;
    }

    uint sel_id = select_table_[obj_id];
    if (sel_id == uint32_t(-1)) {
      return;
    }

    if (int(sel_id) >= select_buf_size_) {
      return;
    }
    Vector<uint32_t> buf(select_buf_size_);
    GPU_storagebuf_read(*select_output_buf_, buf.data());
    uint32_t depth_bits;
    memcpy(&depth_bits, &sdf_depth, sizeof(uint32_t));
    buf[sel_id] = depth_bits;
    GPU_storagebuf_update(*select_output_buf_, buf.data());
  }
};

}  // namespace blender::draw::overlay
