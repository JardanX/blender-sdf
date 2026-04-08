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

#include "DEG_depsgraph_query.hh"

#include "MEM_guardedalloc.h"

#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"

#include "GPU_batch.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_immediate.hh"
#include "GPU_uniform_buffer.hh"

#include "draw_view.hh"

#include "overlay_base.hh"

#include "BLI_rect.h"

#include "engines/sdf/sdf_engine.h"
#include "engines/select/select_defines.hh"
#include "gpu_select_private.hh"

namespace blender::draw::overlay {

static inline void sdf_local_bb(const SDF *sdf, float3 &out_min, float3 &out_max)
{
  if (sdf->sdf_type == SDF_TYPE_GROUP) {
    out_min = float3(-0.5f);
    out_max = float3(0.5f);
    return;
  }
  float3 sz(sdf->size[0], sdf->size[1], sdf->size[2]);
  float3 ext = sz;
  switch (sdf->sdf_type) {
    case SDF_TYPE_CONE:
      ext = float3(sz.x, sz.x, sz.y);
      break;
    case SDF_TYPE_CAPSULE:
      ext = float3(sz.x, sz.x, sz.y + sz.x);
      break;
    case SDF_TYPE_TORUS:
      ext = float3(sz.x + sz.y, sz.x + sz.y, sz.y);
      break;
    case SDF_TYPE_NGON:
      ext = float3(sz.x, sz.x, sz.z);
      break;
    case SDF_TYPE_POLYGON: {
      float3 mn(1e30f), mx(-1e30f);
      for (const SDFPolygonPoint *pt =
               static_cast<const SDFPolygonPoint *>(sdf->polygon_points.first);
           pt; pt = pt->next)
      {
        mn.x = math::min(mn.x, pt->co[0]);
        mn.y = math::min(mn.y, pt->co[1]);
        mx.x = math::max(mx.x, pt->co[0]);
        mx.y = math::max(mx.y, pt->co[1]);
      }
      float line_pad = sdf->polygon_is_line ? sdf->polygon_line_thickness * 0.5f : 0.0f;
      out_min = float3(mn.x - line_pad, mn.y - line_pad, -sz.z);
      out_max = float3(mx.x + line_pad, mx.y + line_pad, sz.z);
      return;
    }
    default:
      break;
  }
  out_min = -ext;
  out_max = ext;
}

class Sdfs : Overlay {
 private:
  const SelectionType selection_type_;
  bool show_bbox_ = true;
  bool show_ngon_ = true;
  bool show_bbox_grid_ = false;

  /* Per-SDF object data collected during object_sync. */
  struct SdfEntry {
    int sdf_index;
    int sorted_index;
    uint32_t outline_packed_id;
    uint32_t select_id;
    float4x4 object_to_world;
    float3 bb_min;
    float3 bb_max;
    const SDF *sdf_data;
    const Object *object;
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
  int selected_count_ = 0;

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
    show_bbox_ = state.v3d && !state.hide_overlays &&
                 !(state.v3d->overlay.flag & V3D_OVERLAY_HIDE_SDF_BBOX);
    show_ngon_ = state.v3d && !state.hide_overlays &&
                 !(state.v3d->overlay.flag & V3D_OVERLAY_HIDE_SDF_NGON);
    show_bbox_grid_ = state.v3d && (state.v3d->shading.sdf_bvh_debug_view == 6);
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

    /* Group empties have no geometry — skip bbox/outline */
    if (sdf_data->sdf_type == SDF_TYPE_GROUP) {
      return;
    }

    const bool is_select = (ob_ref.object->base_flag & BASE_SELECTED) != 0;
    const bool is_active = (ob_ref.object == state.object_active);

    uint outline_color_id = 2u;
    if (is_select && is_active) {
      outline_color_id = 3u;
    }
    else if (is_select) {
      outline_color_id = 1u;
    }

    uint resource_id = uint(entries_.size()) + 1u;
    uint packed_id = (outline_color_id << 14u) | (resource_id & 0x3FFFu);

    const select::ID sel_id = res.select_id(ob_ref);

    int idx = sdf_data->sdf_index;
    if (idx > max_sdf_index_) {
      max_sdf_index_ = idx;
    }
    if (packed_id != 0u) {
      has_selected_ = true;
    }

    float4x4 obmat = float4x4(ob_ref.object->object_to_world());

    /* Store polygon point bounds for BB (other types computed from engine data). */
    float3 poly_min(0), poly_max(0);
    if (sdf_data->sdf_type == SDF_TYPE_POLYGON && sdf_data->totpolygon >= 3) {
      poly_min = float3(1e30f);
      poly_max = float3(-1e30f);
      for (const SDFPolygonPoint *pt =
               static_cast<const SDFPolygonPoint *>(sdf_data->polygon_points.first);
           pt; pt = pt->next)
      {
        poly_min.x = math::min(poly_min.x, pt->co[0]);
        poly_min.y = math::min(poly_min.y, pt->co[1]);
        poly_max.x = math::max(poly_max.x, pt->co[0]);
        poly_max.y = math::max(poly_max.y, pt->co[1]);
      }
      poly_min.z = -sdf_data->size[2];
      poly_max.z = sdf_data->size[2];
    }
    entries_.append({idx, -1, packed_id, sel_id.get(), obmat, poly_min, poly_max, sdf_data, ob_ref.object});
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

    /* Build tables indexed by sorted GPU position.
     * Use direct Object* → sorted index lookup for robustness. */
    int obj_count = sdf::sdf_object_count_get();
    int table_size = math::max(obj_count, 1);
    Vector<uint32_t> outline_table(table_size, 0u);
    select_table_.reinitialize(table_size);
    select_table_.fill(uint32_t(-1));

    /* Build Object* → entry index map */
    Map<const Object *, int> obj_to_entry;
    for (int i = 0; i < int(entries_.size()); i++) {
      if (entries_[i].object) {
        const Object *orig = DEG_get_original(entries_[i].object);
        if (!obj_to_entry.contains(orig)) {
          obj_to_entry.add(orig, i);
        }
      }
    }

    /* Populate tables for ALL sorted positions (including Array copies).
     * Use sorted_to_object map from engine to find each position's entry. */
    int sorted_obj_count = 0;
    const Object *const *sorted_ptrs = sdf::sdf_sorted_object_ptrs_get(&sorted_obj_count);
    for (int si = 0; si < math::min(sorted_obj_count, table_size); si++) {
      if (!sorted_ptrs || !sorted_ptrs[si]) { continue; }
      const Object *orig = DEG_get_original(const_cast<Object *>(sorted_ptrs[si]));
      const int *ei_ptr = obj_to_entry.lookup_ptr(orig);
      if (!ei_ptr) { continue; }
      int ei = *ei_ptr;
      if (entries_[ei].sorted_index < 0) {
        entries_[ei].sorted_index = si;
      }
      uint32_t color_id = entries_[ei].outline_packed_id >> 14u;
      uint32_t new_packed = (color_id << 14u) | (uint32_t(si + 1) & 0x3FFFu);
      outline_table[si] = new_packed;
      select_table_[si] = entries_[ei].select_id;
    }

    Vector<int32_t> sel_indices;
    for (int si = 0; si < table_size; si++) {
      if (outline_table[si] != 0u) { sel_indices.append(si); }
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

  void draw_outline(Framebuffer &framebuffer, View & /*view*/, gpu::Texture *scene_depth)
  {
    if (!enabled_ || entries_.is_empty()) {
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

    GPU_texture_filter_mode(depth_tx, false);
    GPU_texture_bind(depth_tx, GPU_shader_get_sampler_binding(outline_sh_, "sdf_depth_tx"));

    GPU_texture_filter_mode(gbuf_tx, false);
    GPU_texture_bind(gbuf_tx, GPU_shader_get_sampler_binding(outline_sh_, "sdf_gbuf_color_tx"));

    if (scene_depth) {
      GPU_texture_filter_mode(scene_depth, false);
      GPU_texture_bind(scene_depth,
                       GPU_shader_get_sampler_binding(outline_sh_, "scene_depth_tx"));
    }

    float2 uv_sc = sdf::sdf_uv_scale_get();
    GPU_shader_uniform_2fv(outline_sh_, "uv_scale", &uv_sc.x);

    GPU_batch_set_shader(fullscreen_batch_, outline_sh_);
    GPU_batch_draw(fullscreen_batch_);

    GPU_texture_unbind(depth_tx);
    GPU_texture_unbind(gbuf_tx);
    if (scene_depth) {
      GPU_texture_unbind(scene_depth);
    }
    GPU_shader_unbind();

  }

  void draw_line(Framebuffer &framebuffer, Manager & /*manager*/, View &view) final
  {
    if (!enabled_ || (!show_bbox_ && !show_bbox_grid_)) {
      return;
    }

    /* Find the single selected SDF entry for bbox drawing. */
    const SdfEntry *sel = nullptr;
    int sel_count = 0;
    for (const SdfEntry &e : entries_) {
      uint color_id = e.outline_packed_id >> 14u;
      if (color_id == 1u || color_id == 3u) {
        sel = &e;
        sel_count++;
      }
    }
    if (sel_count != 1 || !sel) {
      return;
    }

    float3 lo, hi;
    float4x4 rot;
    float3 obj_pos;

    /* Always try CPU SDF eval for tight bbox + debug points. */
    float3 hint_pos = float3(sel->object_to_world[3]);
    if (!sdf::sdf_object_bbox_get(sel->sorted_index, hint_pos, lo, hi, rot, obj_pos)) {
      if (sel->bb_min.x < sel->bb_max.x) {
        lo = sel->bb_min;
        hi = sel->bb_max;
        float3 sx = math::normalize(float3(sel->object_to_world[0]));
        float3 sy = math::normalize(float3(sel->object_to_world[1]));
        float3 sz = math::normalize(float3(sel->object_to_world[2]));
        rot = float4x4(float4(sx, 0), float4(sy, 0), float4(sz, 0), float4(0, 0, 0, 1));
        obj_pos = float3(sel->object_to_world[3]);
        float3 scl(math::length(float3(sel->object_to_world[0])),
                   math::length(float3(sel->object_to_world[1])),
                   math::length(float3(sel->object_to_world[2])));
        lo *= scl;
        hi *= scl;
      }
      else {
        return;
      }
    }

    GPU_framebuffer_bind(framebuffer);
    GPU_depth_test(GPU_DEPTH_ALWAYS);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_line_smooth(true);
    GPU_line_width(1.0f);

    /* Collect transforms (rotation + position) for base + modifier copies. */
    struct CopyXform {
      float4x4 r;
      float3 p;
    };
    Vector<CopyXform> copies;
    copies.append({rot, obj_pos});

    if (sel->object) {
      float4x4 inv_rot = math::transpose(rot);
      for (const ModifierData &md : sel->object->modifiers) {
        if (!(md.mode & eModifierMode_Realtime)) {
          continue;
        }
        if (md.type == eModifierType_SDFMirror) {
          const auto &m = reinterpret_cast<const SDFMirrorModifierData &>(md);
          float offset = m.offset_distance;
          float3 world_org(0);
          if (m.mirror_object) {
            world_org = float3(m.mirror_object->object_to_world()[3]) - obj_pos;
          }
          float3 local_org = float3(inv_rot * float4(world_org, 0.0f));

          auto do_mirror = [&](int axis_idx) {
            float3 mirror_pos = m.mirror_object ?
                                    float3(m.mirror_object->object_to_world()[3]) :
                                    float3(0.0f);
            float3 N(0);
            if (m.mirror_object) {
              N = math::normalize(
                  float3(m.mirror_object->object_to_world()[axis_idx]));
            }
            else {
              N[axis_idx] = 1.0f;
            }
            int cur = int(copies.size());
            for (int ci = 0; ci < cur; ci++) {
              float3 p = copies[ci].p;
              float d = math::dot(mirror_pos - p, N);
              float3 reflected = p + 2.0f * d * N;
              float4x4 mirror_r = copies[ci].r;
              for (int c = 0; c < 3; c++) {
                float3 col = float3(mirror_r[c]);
                col -= 2.0f * math::dot(col, N) * N;
                mirror_r[c] = float4(col, 0.0f);
              }
              copies[ci].p = p + offset * N;
              copies.append({mirror_r, reflected - offset * N});
            }
          };
          if (m.flag & MOD_SDF_MIRROR_AXIS_X) { do_mirror(0); }
          if (m.flag & MOD_SDF_MIRROR_AXIS_Y) { do_mirror(1); }
          if (m.flag & MOD_SDF_MIRROR_AXIS_Z) { do_mirror(2); }
        }
        else if (md.type == eModifierType_SDFArray) {
          const auto &m = reinterpret_cast<const SDFArrayModifierData &>(md);
          int count = m.count;
          if (count < 2) {
            continue;
          }
          float4x4 obmat = sel->object_to_world;
          float3 scl(math::length(float3(obmat[0])),
                     math::length(float3(obmat[1])),
                     math::length(float3(obmat[2])));

          /* Object offset delta in local space */
          float4x4 obj_delta = float4x4::identity();
          bool has_obj_off = (m.use_object_offset && m.offset_object != nullptr);
          if (has_obj_off) {
            obj_delta = math::invert(obmat) * m.offset_object->object_to_world();
          }

          if (m.array_type == MOD_SDF_ARRAY_LINEAR) {
            float3 dimensions(sel->sdf_data->size[0] * scl.x * 2.0f,
                              sel->sdf_data->size[1] * scl.y * 2.0f,
                              sel->sdf_data->size[2] * scl.z * 2.0f);
            float3 local_off(0.0f);
            if (m.use_relative_offset) {
              local_off += float3(m.relative_offset[0], m.relative_offset[1],
                                  m.relative_offset[2]) * dimensions;
            }
            if (m.use_constant_offset) {
              local_off += float3(m.constant_offset[0], m.constant_offset[1],
                                  m.constant_offset[2]);
            }
            float4x4 rot_only = obmat;
            if (scl.x > 0) rot_only[0] = float4(float3(obmat[0]) / scl.x, 0);
            if (scl.y > 0) rot_only[1] = float4(float3(obmat[1]) / scl.y, 0);
            if (scl.z > 0) rot_only[2] = float4(float3(obmat[2]) / scl.z, 0);
            rot_only[3] = float4(0, 0, 0, 1);
            float3 world_off = float3(rot_only * float4(local_off, 0.0f));
            int cur = int(copies.size());
            for (int ci = 0; ci < cur; ci++) {
              float4x4 local_acc = float4x4::identity();
              if (has_obj_off) { local_acc = obj_delta; }
              for (int ai = 1; ai < count; ai++) {
                float4x4 copy_mat;
                if (has_obj_off) {
                  copy_mat = obmat * local_acc;
                  local_acc = local_acc * obj_delta;
                }
                else {
                  copy_mat = obmat;
                }
                copy_mat[3] += float4(world_off * float(ai), 0.0f);
                float4x4 copy_r = copy_mat;
                float3 cs(math::length(float3(copy_mat[0])),
                          math::length(float3(copy_mat[1])),
                          math::length(float3(copy_mat[2])));
                if (cs.x > 0) copy_r[0] = float4(float3(copy_mat[0]) / cs.x, 0);
                if (cs.y > 0) copy_r[1] = float4(float3(copy_mat[1]) / cs.y, 0);
                if (cs.z > 0) copy_r[2] = float4(float3(copy_mat[2]) / cs.z, 0);
                copy_r[3] = float4(0, 0, 0, 1);
                copies.append({copy_r, float3(copy_mat[3])});
              }
            }
          }
          else if (m.array_type == MOD_SDF_ARRAY_RADIAL) {
            float radius = m.array_radius;
            float rx = m.rotation_offset[0], ry = m.rotation_offset[1],
                  rz = m.rotation_offset[2];
            float4x4 fwd = math::from_rotation<float4x4>(
                math::EulerXYZ(rx, ry, rz));
            float4x4 rot_std = math::invert(fwd);
            float4x4 fwd_mir = math::from_rotation<float4x4>(
                math::EulerXYZ(-rx, ry, -rz));
            float4x4 rot_mir = math::invert(fwd_mir);
            int cur = int(copies.size());
            for (int ci = 0; ci < cur; ci++) {
              float4x4 orig_rot = copies[ci].r;
              float3 orig_pos = copies[ci].p;
              float3 base_off = float3(orig_rot * float4(radius, 0, 0, 0));
              copies[ci].p = orig_pos + base_off;
              copies[ci].r = orig_rot * rot_std;

              for (int ai = 1; ai < count; ai++) {
                bool mirrored = (ai % 2 == 1);
                float angle = 2.0f * float(M_PI) * float(ai) / float(count);
                float ca = cosf(angle), sa = sinf(angle);
                float3 local_pos(radius * ca, radius * sa, 0);
                float3 world_pos = float3(orig_rot * float4(local_pos, 0.0f))
                                   + orig_pos;
                float4x4 arr_rot = float4x4(
                    float4(ca, sa, 0, 0),
                    float4(-sa, ca, 0, 0),
                    float4(0, 0, 1, 0),
                    float4(0, 0, 0, 1));
                copies.append({orig_rot * arr_rot * (mirrored ? rot_mir : rot_std),
                               world_pos});
              }
            }
          }
        }
      }
    }

    /* Draw BB at each copy. */
    float3 lc[8] = {
        {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z},
        {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
        {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z},
        {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z},
    };
    int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7},
    };

    uint pos_attr = GPU_vertformat_attr_add_legacy(
        immVertexFormat(), "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
    immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
    immUniformColor4f(1.0f, 0.65f, 0.0f, 0.2f);

    immBegin(GPU_PRIM_LINES, 24 * int(copies.size()));
    for (const CopyXform &cx : copies) {
      float3 wc[8];
      for (int i = 0; i < 8; i++) {
        float4 rp = cx.r * float4(lc[i], 1.0f);
        wc[i] = float3(rp.x, rp.y, rp.z) + cx.p;
      }
      for (int i = 0; i < 12; i++) {
        immVertex3fv(pos_attr, wc[edges[i][0]]);
        immVertex3fv(pos_attr, wc[edges[i][1]]);
      }
    }
    immEnd();

    immUnbindProgram();

    if (show_bbox_grid_) {
      const float3 *dbg_pts;
      int dbg_count;
      float4x4 dbg_rot;
      float3 dbg_pos;
      sdf::sdf_bbox_debug_points_get(&dbg_pts, &dbg_count, &dbg_rot, &dbg_pos);
      if (dbg_count > 0) {
        GPU_point_size(4.0f);
        uint dbg_attr = GPU_vertformat_attr_add_legacy(
            immVertexFormat(), "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
        immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
        immUniformColor4f(0.0f, 1.0f, 1.0f, 0.9f);
        immBegin(GPU_PRIM_POINTS, dbg_count);
        for (int di = 0; di < dbg_count; di++) {
          float4 rp = dbg_rot * float4(dbg_pts[di], 1.0f);
          float3 wp = float3(rp.x, rp.y, rp.z) + dbg_pos;
          immVertex3fv(dbg_attr, wp);
        }
        immEnd();
        immUnbindProgram();
      }
    }

    GPU_line_smooth(false);
    GPU_blend(GPU_BLEND_NONE);
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
    if (sel_id == uint32_t(-1) || sel_id >= uint32_t(select_buf_size_)) {
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
