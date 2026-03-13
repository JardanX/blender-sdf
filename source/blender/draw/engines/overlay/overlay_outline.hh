/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BLI_assert.h"

#include "overlay_base.hh"
/* MATHOPS: Removed — Grease Pencil overlay */
// #include "overlay_grease_pencil.hh"

#include "draw_common.hh"
#include "draw_view.hh"

#include "DNA_object_types.h"
#include "DNA_userdef_types.h"

#include "GPU_batch.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_uniform_buffer.hh"

#include "engines/sdf/sdf_engine.h"

namespace blender::draw::overlay {

/**
 * Display selected object outline.
 * The option can be found under (Viewport Overlays > Objects > Outline Selected).
 *
 * SDF objects are handled via a fullscreen ray-march pass that writes packed
 * outline IDs into the same framebuffer as the regular geometry prepass.
 */
class Outline : Overlay {
 private:
  /* Simple render pass that renders an object ID pass. */
  PassMain outline_prepass_ps_ = {"Prepass"};
  PassMain::Sub *prepass_curves_ps_ = nullptr;
  PassMain::Sub *prepass_pointcloud_ps_ = nullptr;
  /* MATHOPS: Removed — Grease Pencil */
  // PassMain::Sub *prepass_gpencil_ps_ = nullptr;
  PassMain::Sub *prepass_mesh_ps_ = nullptr;
  PassMain::Sub *prepass_volume_ps_ = nullptr;
  PassMain::Sub *prepass_wire_ps_ = nullptr;
  /* Detect edges inside the ID pass and output color for each of them. */
  PassSimple outline_resolve_ps_ = {"Resolve"};

  TextureFromPool object_id_tx_ = {"outline_ob_id_tx"};
  TextureFromPool tmp_depth_tx_ = {"outline_depth_tx"};

  Framebuffer prepass_fb_ = {"outline.prepass_fb"};

  Vector<FlatObjectRef> flat_objects_;

  PassMain outline_prepass_flat_ps_ = {"PrepassFlat"};

  /* ---- SDF outline state ---- */

  /**
   * Maps SDF object index (bake order) -> packed 16-bit outline ID.
   * 0 means not selected (no outline).
   * Format: (color_id << 14) | (unique_id & 0x3FFF)
   *   color_id: 0=transform, 1=selected, 3=active
   */
  Vector<uint32_t> sdf_outline_ids_;
  /** Compact list of SDF object indices that are selected (for instancing). */
  Vector<int32_t> sdf_selected_indices_;
  int sdf_selected_count_ = 0;

  gpu::Shader *sdf_outline_sh_ = nullptr;
  gpu::Batch *sdf_box_batch_ = nullptr;
  gpu::StorageBuf *sdf_outline_ssbo_ = nullptr;
  gpu::StorageBuf *sdf_selected_ssbo_ = nullptr;
  int sdf_outline_ssbo_count_ = 0;
  int sdf_selected_ssbo_count_ = 0;

  /* Cached binding locations for the SDF outline shader. */
  int sdf_map_slot_ = -1;
  int sdf_objects_slot_ = -1;
  int sdf_selected_slot_ = -1;
  int sdf_modifiers_slot_ = -1;
  int sdf_voxel_size_loc_ = -1;
  int sdf_viewport_inv_loc_ = -1;

 public:
  ~Outline()
  {
    if (sdf_outline_ssbo_) {
      GPU_storagebuf_free(sdf_outline_ssbo_);
    }
    if (sdf_selected_ssbo_) {
      GPU_storagebuf_free(sdf_selected_ssbo_);
    }
    if (sdf_box_batch_) {
      GPU_batch_discard(sdf_box_batch_);
    }
  }

  void begin_sync(Resources &res, const State &state) final
  {
    enabled_ = !res.is_selection();
    enabled_ &= state.v3d && (state.v3d_flag & V3D_SELECT_OUTLINE);

    flat_objects_.clear();
    sdf_outline_ids_.clear();
    sdf_selected_indices_.clear();
    sdf_selected_count_ = 0;

    if (!enabled_) {
      return;
    }

    const float outline_width = UI_GetThemeValuef(TH_OUTLINE_WIDTH);
    const bool do_smooth_lines = (U.gpu_flag & USER_GPU_FLAG_OVERLAY_SMOOTH_WIRE) != 0;
    const bool do_expand = (U.pixelsize > 1.0) || (outline_width > 2.0f);
    const bool is_transform = (G.moving & G_TRANSFORM_OBJ) != 0;

    {
      auto &pass = outline_prepass_ps_;
      pass.init();
      pass.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
      pass.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
      pass.framebuffer_set(&prepass_fb_);
      pass.clear_color_depth_stencil(float4(0.0f), 1.0f, 0x0);
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL,
                     state.clipping_plane_count);
      {
        auto &sub = pass.sub("Curves");
        sub.shader_set(res.shaders->outline_prepass_curves.get());
        sub.push_constant("is_transform", is_transform);
        prepass_curves_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("PointCloud");
        sub.shader_set(res.shaders->outline_prepass_pointcloud.get());
        sub.push_constant("is_transform", is_transform);
        prepass_pointcloud_ps_ = &sub;
      }
      /* MATHOPS: Removed — Grease Pencil overlay */
      // {
      //   auto &sub = pass.sub("GreasePencil");
      //   sub.shader_set(res.shaders->outline_prepass_gpencil.get());
      //   sub.push_constant("is_transform", is_transform);
      //   prepass_gpencil_ps_ = &sub;
      // }
      {
        auto &sub = pass.sub("Mesh");
        sub.shader_set(res.shaders->outline_prepass_mesh.get());
        sub.push_constant("is_transform", is_transform);
        prepass_mesh_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("Volume");
        sub.shader_set(res.shaders->outline_prepass_mesh.get());
        sub.push_constant("is_transform", is_transform);
        prepass_volume_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("Wire");
        sub.shader_set(res.shaders->outline_prepass_wire.get());
        sub.push_constant("is_transform", is_transform);
        prepass_wire_ps_ = &sub;
      }
    }
    {
      auto &pass = outline_resolve_ps_;
      pass.init();
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA_PREMUL);
      pass.shader_set(res.shaders->outline_detect.get());
      /* Don't occlude the outline if in xray mode as it causes too much flickering. */
      pass.push_constant("alpha_occlu", state.xray_enabled ? 1.0f : 0.35f);
      pass.push_constant("do_thick_outlines", do_expand);
      pass.push_constant("do_anti_aliasing", do_smooth_lines);
      pass.push_constant("is_xray_wires", state.xray_enabled_and_not_wire);
      pass.bind_texture("outline_id_tx", &object_id_tx_);
      pass.bind_texture("scene_depth_tx", &res.depth_tx);
      pass.bind_texture("outline_depth_tx", &tmp_depth_tx_);
      pass.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
      pass.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
      pass.draw_procedural(GPU_PRIM_TRIS, 1, 3);
    }
  }

  void object_sync(Manager &manager,
                   const ObjectRef &ob_ref,
                   Resources &res,
                   const State &state) final
  {
    if (!enabled_) {
      return;
    }

    /* Outlines of bounding boxes are not drawn. */
    if (ob_ref.object->dt == OB_BOUNDBOX) {
      return;
    }

    gpu::Batch *geom;
    switch (ob_ref.object->type) {
      case OB_CURVES: {
        const char *error = nullptr;
        /* The error string will always have been printed by the engine already.
         * No need to display it twice. */
        geom = curves_sub_pass_setup(*prepass_curves_ps_, state.scene, ob_ref.object, error);
        prepass_curves_ps_->draw(geom, manager.unique_handle(ob_ref));
        break;
      }
      /* MATHOPS: Removed — Grease Pencil overlay */
      // case OB_GREASE_PENCIL:
      //   GreasePencil::draw_grease_pencil(...);
      //   break;
      case OB_MESH:
        if (state.xray_enabled_and_not_wire) {
          geom = DRW_cache_mesh_edge_detection_get(ob_ref.object, nullptr);
          prepass_wire_ps_->draw_expand(geom, GPU_PRIM_LINES, 1, 1, manager.unique_handle(ob_ref));
        }
        else {
          geom = DRW_cache_mesh_surface_get(ob_ref.object);
          prepass_mesh_ps_->draw(geom, manager.unique_handle(ob_ref));

          /* Display flat object as a line when view is orthogonal to them.
           * This fixes only the biggest case which is a plane in ortho view. */
          int flat_axis = FlatObjectRef::flat_axis_index_get(ob_ref.object);
          if (flat_axis != -1) {
            geom = DRW_cache_mesh_edge_detection_get(ob_ref.object, nullptr);
            flat_objects_.append({geom, manager.unique_handle(ob_ref), flat_axis});
          }
        }
        break;
      case OB_POINTCLOUD:
        /* Looks bad in wireframe mode. Could be relaxed if we draw a wireframe of some sort in
         * the future. */
        if (!state.is_wireframe_mode) {
          geom = pointcloud_sub_pass_setup(*prepass_pointcloud_ps_, ob_ref.object);
          prepass_pointcloud_ps_->draw(geom, manager.unique_handle(ob_ref));
        }
        break;
      case OB_VOLUME:
        geom = DRW_cache_volume_selection_surface_get(ob_ref.object);
        /* TODO(fclem): Get rid of these check and enforce correct API on the batch cache. */
        if (geom) {
          prepass_volume_ps_->draw(geom, manager.unique_handle(ob_ref));
        }
        break;
      default:
        break;
    }
  }

  /**
   * Track an SDF object for outline rendering.
   * Must be called for ALL SDF objects in bake order (matching the SDF engine's iteration).
   * Selected objects get a packed outline ID; non-selected objects map to 0 (no outline).
   */
  void sdf_object_sync(const ObjectRef &ob_ref, const State &state)
  {
    if (!enabled_) {
      return;
    }
    if (ob_ref.object->type != OB_SDF) {
      return;
    }

    const bool is_selected = (ob_ref.object->base_flag & BASE_SELECTED) != 0;
    if (!is_selected) {
      sdf_outline_ids_.append(0);
      return;
    }

    const bool is_active = (ob_ref.object == state.object_active);
    const bool is_transform = (G.moving & G_TRANSFORM_OBJ) != 0;

    uint color_id;
    if (is_transform) {
      color_id = 0u; /* theme.colors.transform */
    }
    else if (is_active) {
      color_id = 3u; /* theme.colors.active_object */
    }
    else {
      color_id = 1u; /* theme.colors.object_select */
    }

    sdf_selected_count_++;

    /* Use IDs offset by 0x2000 to avoid collision with mesh resource IDs.
     * The bottom 14 bits hold the unique object ID, top 2 bits hold the color. */
    uint unique_id = uint(sdf_selected_count_) + 0x2000u;
    uint packed = (color_id << 14u) | (unique_id & 0x3FFFu);
    sdf_outline_ids_.append(packed);

    /* Record this object's index for instanced AABB draw. */
    sdf_selected_indices_.append(int32_t(sdf_outline_ids_.size() - 1));
  }

  /* Flat objects outline workaround need to generate passes for each redraw. */
  void flat_objects_pass_sync(Manager &manager, View &view, Resources &res, const State &state)
  {
    outline_prepass_flat_ps_.init();

    if (!enabled_) {
      return;
    }

    if (!view.is_persp()) {
      const bool is_transform = (G.moving & G_TRANSFORM_OBJ) != 0;
      /* Note: We need a dedicated pass since we have to populated it for each redraw. */
      auto &pass = outline_prepass_flat_ps_;
      pass.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
      pass.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
      pass.framebuffer_set(&prepass_fb_);
      pass.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL,
                     state.clipping_plane_count);
      pass.shader_set(res.shaders->outline_prepass_wire.get());
      pass.push_constant("is_transform", is_transform);

      for (FlatObjectRef flag_ob_ref : flat_objects_) {
        flag_ob_ref.if_flat_axis_orthogonal_to_view(
            manager, view, [&](gpu::Batch *geom, ResourceIndex resource_index) {
              pass.draw_expand(geom, GPU_PRIM_LINES, 1, 1, resource_index);
            });
      }
    }
  }

  void pre_draw(Manager &manager, View &view) final
  {
    if (!enabled_) {
      return;
    }

    manager.generate_commands(outline_prepass_ps_, view);
    manager.generate_commands(outline_prepass_flat_ps_, view);
  }

  /* TODO(fclem): Remove dependency on Resources. */
  void draw_line_only_ex(Framebuffer &framebuffer, Resources &res, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }

    GPU_debug_group_begin("Outline");

    int2 render_size = int2(res.depth_tx.size());

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT;
    tmp_depth_tx_.acquire(render_size, gpu::TextureFormat::SFLOAT_32_DEPTH_UINT_8, usage);
    object_id_tx_.acquire(render_size, gpu::TextureFormat::UINT_16, usage);

    prepass_fb_.ensure(GPU_ATTACHMENT_TEXTURE(tmp_depth_tx_),
                       GPU_ATTACHMENT_TEXTURE(object_id_tx_));

    manager.submit_only(outline_prepass_ps_, view);
    manager.submit_only(outline_prepass_flat_ps_, view);

    /* SDF outline: analytical per-object sphere-march into the same framebuffer. */
    draw_sdf_outline_(res, view);

    GPU_framebuffer_bind(framebuffer);
    manager.submit(outline_resolve_ps_, view);

    tmp_depth_tx_.release();
    object_id_tx_.release();

    GPU_debug_group_end();
  }

 private:
  /**
   * Draw SDF outline pass — instanced AABB rasterization.
   *
   * Instead of a fullscreen pass that loops over all objects per pixel (O(N)),
   * this draws one instanced box per selected object. Each fragment evaluates
   * exactly one SDF, and the GPU depth test resolves overlaps. Scales to
   * thousands of selected objects with near-zero overhead.
   */
  void draw_sdf_outline_(Resources &res, View &view)
  {
    if (sdf_selected_count_ == 0) {
      return;
    }
    gpu::StorageBuf *objects_ssbo = sdf::sdf_objects_ssbo_get();
    if (objects_ssbo == nullptr) {
      return;
    }

    /* Lazy-create the outline shader. */
    if (!sdf_outline_sh_) {
      sdf_outline_sh_ = GPU_shader_create_from_info_name("sdf_outline_march");
      if (sdf_outline_sh_) {
        sdf_map_slot_ = GPU_shader_get_ssbo_binding(sdf_outline_sh_, "outline_id_map_buf");
        sdf_objects_slot_ = GPU_shader_get_ssbo_binding(sdf_outline_sh_, "sdf_objects");
        sdf_selected_slot_ = GPU_shader_get_ssbo_binding(sdf_outline_sh_, "selected_indices");
        sdf_modifiers_slot_ = GPU_shader_get_ssbo_binding(sdf_outline_sh_, "sdf_modifiers");
        sdf_voxel_size_loc_ = GPU_shader_get_uniform(sdf_outline_sh_, "voxel_size");
        sdf_viewport_inv_loc_ = GPU_shader_get_uniform(sdf_outline_sh_, "viewport_size_inv");
      }
    }
    if (!sdf_outline_sh_) {
      return;
    }

    /* Use safe count: minimum of overlay-tracked and engine-tracked objects. */
    const int id_count = min(int(sdf_outline_ids_.size()), sdf::sdf_object_count_get());
    if (id_count == 0) {
      return;
    }

    /* Upload outline ID map SSBO. */
    if (sdf_outline_ssbo_ != nullptr && sdf_outline_ssbo_count_ != id_count) {
      GPU_storagebuf_free(sdf_outline_ssbo_);
      sdf_outline_ssbo_ = nullptr;
    }
    if (sdf_outline_ssbo_ == nullptr) {
      sdf_outline_ssbo_ = GPU_storagebuf_create_ex(id_count * sizeof(uint32_t),
                                                    sdf_outline_ids_.data(),
                                                    GPU_USAGE_DYNAMIC,
                                                    "sdf_outline_id_map");
      sdf_outline_ssbo_count_ = id_count;
    }
    else {
      GPU_storagebuf_update(sdf_outline_ssbo_, sdf_outline_ids_.data());
    }

    /* Upload selected indices SSBO (compact list for instancing). */
    const int sel_count = int(sdf_selected_indices_.size());
    if (sel_count == 0) {
      return;
    }
    if (sdf_selected_ssbo_ != nullptr && sdf_selected_ssbo_count_ != sel_count) {
      GPU_storagebuf_free(sdf_selected_ssbo_);
      sdf_selected_ssbo_ = nullptr;
    }
    if (sdf_selected_ssbo_ == nullptr) {
      sdf_selected_ssbo_ = GPU_storagebuf_create_ex(sel_count * sizeof(int32_t),
                                                     sdf_selected_indices_.data(),
                                                     GPU_USAGE_DYNAMIC,
                                                     "sdf_selected_indices");
      sdf_selected_ssbo_count_ = sel_count;
    }
    else {
      GPU_storagebuf_update(sdf_selected_ssbo_, sdf_selected_indices_.data());
    }

    /* Bind the prepass framebuffer (object_id_tx_ + tmp_depth_tx_). */
    GPU_framebuffer_bind(prepass_fb_);

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    /* Cull front faces: render back faces so fragments are generated even when
     * the camera is inside an AABB. The fragment shader writes gl_FragDepth to
     * the actual SDF surface, so the rasterized face depth is irrelevant. */
    GPU_face_culling(GPU_CULL_FRONT);

    GPU_shader_bind(sdf_outline_sh_);

    /* Bind SSBOs. */
    GPU_storagebuf_bind(sdf_outline_ssbo_, sdf_map_slot_);
    GPU_storagebuf_bind(objects_ssbo, sdf_objects_slot_);
    GPU_storagebuf_bind(sdf_selected_ssbo_, sdf_selected_slot_);
    gpu::StorageBuf *mod_ssbo = sdf::sdf_modifiers_ssbo_get();
    if (mod_ssbo) {
      GPU_storagebuf_bind(mod_ssbo, sdf_modifiers_slot_);
    }

    /* Push uniforms. */
    float vs;
    float3 origin, extent;
    int3 grid_res;
    int bpa;
    sdf::sdf_atlas_params_get(&vs, &origin, &extent, &grid_res, &bpa);
    GPU_shader_uniform_float_ex(sdf_outline_sh_, sdf_voxel_size_loc_, 1, 1, &vs);

    int2 render_size = int2(res.depth_tx.size());
    float viewport_inv[2] = {1.0f / float(render_size.x), 1.0f / float(render_size.y)};
    GPU_shader_uniform_float_ex(sdf_outline_sh_, sdf_viewport_inv_loc_, 2, 1, viewport_inv);

    /* Bind the view UBO for camera matrices. */
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    /* Draw instanced AABB boxes: 36 vertices (12 triangles) per instance,
     * one instance per selected SDF object. */
    if (!sdf_box_batch_) {
      sdf_box_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 36);
    }
    GPU_batch_set_shader(sdf_box_batch_, sdf_outline_sh_);
    GPU_batch_draw_advanced(sdf_box_batch_, 0, 36, 0, sel_count);

    /* Restore default state (no face culling). */
    GPU_face_culling(GPU_CULL_NONE);
    GPU_shader_unbind();
  }
};

}  // namespace blender::draw::overlay
