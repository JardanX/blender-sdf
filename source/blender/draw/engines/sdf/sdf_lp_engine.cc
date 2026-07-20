/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF Lipschitz pruning engine: hierarchical Lipschitz pruning of the CSG tree
 * (Barbier et al., EG 2025). Shares scene sync/upload and the color resolve /
 * normal / shade / blit passes with the classic engine through
 * #SdfInstanceBase.
 */

#include "sdf_engine_internal.hh"

namespace blender::draw::sdf {

/* LP binary ops pack DNA CSG ids verbatim (lp_pack_binary_op). */
static_assert(SDF_LP_CSG_UNION == SDF_CSG_UNION);
static_assert(SDF_LP_CSG_SUBTRACT == SDF_CSG_SUBTRACT);
static_assert(SDF_LP_CSG_INTERSECT == SDF_CSG_INTERSECT);
static_assert(SDF_LP_CSG_PAINT == SDF_CSG_PAINT);
static_assert(SDF_LP_BLEND_LINEAR == SDF_BLEND_LINEAR);
static_assert(SDF_LP_BLEND_SMOOTH == SDF_BLEND_SMOOTH);
static_assert(SDF_LP_BLEND_CHAMFER == SDF_BLEND_CHAMFER);
static_assert(SDF_LP_BLEND_ROUND == SDF_BLEND_ROUND);

static constexpr int kLpShaders[] = {
    SH_NORMAL_COMP,
    SH_SHADE_COMP,
    SH_BLIT,
    SH_FXAA,
    SH_LP_PRUNE_COMP,
    SH_LP_MARCH_COMP,
    SH_AABB_PROJECT_COMP,
    SH_TILE_CULL_COMP,
    SH_COLOR_RESOLVE_COMP,
    SH_LP_DEBUG_COMP,
    SH_MESH_BAKE_COMP,
};

class LpInstance : public SdfInstanceBase {
 protected:
  gpu::Shader *lp_prune_sh() { return sdf_shader_get(SH_LP_PRUNE_COMP); }
  gpu::Shader *lp_march_sh() { return sdf_shader_get(SH_LP_MARCH_COMP); }
  gpu::Shader *lp_debug_sh() { return sdf_shader_get(SH_LP_DEBUG_COMP); }

  /* ---- Lipschitz pruning engine state ---- */
  bool lp_enable_pruning_ = true;
  int lp_shading_mode_ = 0;
  int lp_grid_level_ = 6;
  int lp_colormap_max_ = 25;
  bool lp_aabb_auto_ = true;
  float3 lp_aabb_min_user_ = float3(-1.0f);
  float3 lp_aabb_max_user_ = float3(1.0f);

  /* CPU CSG tree (post-order serialized, rebuilt in end_sync). */
  Vector<SDFLpNode> lp_nodes_;
  Vector<SDFLpPrimitive> lp_prims_;
  Vector<uint4> lp_binary_ops_;
  /* Initial active list: node words (index | sign) and parent indices in
   * parallel arrays (parents are only needed by the prune pass). */
  Vector<uint32_t> lp_active_init_nodes_;
  Vector<uint32_t> lp_active_init_parents_;

  gpu::StorageBuf *lp_nodes_ssbo_ = nullptr;
  gpu::StorageBuf *lp_prims_ssbo_ = nullptr;
  gpu::StorageBuf *lp_binary_ops_ssbo_ = nullptr;
  gpu::StorageBuf *lp_active_init_nodes_ssbo_ = nullptr;
  gpu::StorageBuf *lp_active_init_parents_ssbo_ = nullptr;

  /* Grid buffers (ping-pong between hierarchy levels). Active list node words
   * are uint (index | sign; the only data the trace passes read); parent
   * indices are a parallel uint array consumed only by the prune pass. Cell
   * metadata is int4 (num_active, cell_offset, float bits of the cell value,
   * unused). */
  gpu::StorageBuf *lp_active_nodes_ssbo_[2] = {nullptr, nullptr};
  gpu::StorageBuf *lp_active_parents_ssbo_[2] = {nullptr, nullptr};
  gpu::StorageBuf *lp_cell_meta_ssbo_[2] = {nullptr, nullptr};
  gpu::StorageBuf *lp_active_count_ssbo_ = nullptr; /* 32 ints: active + tmp counters. */
  gpu::StorageBuf *lp_tmp_ssbo_ = nullptr;
  gpu::StorageBuf *lp_scratch_ssbo_ = nullptr;

  int lp_grid_size_ = 0; /* Allocated grid resolution per axis (0 = unallocated). */
  int64_t lp_active_capacity_ = 0;
  int64_t lp_tmp_capacity_ = 0;
  bool lp_grid_valid_ = false; /* Pruning results match the current tree/grid/AABB. */
  bool lp_grid_dirty_ = true;
  int lp_grid_level_built_ = 0; /* Grid level the current results were built with. */
  int lp_final_idx_ = 0; /* Ping-pong slot holding the final level results. */
  float3 lp_aabb_min_ = float3(0.0f);
  float3 lp_aabb_max_ = float3(0.0f);
  /* Overflow stats read back after prune rebuilds (cells that fell back
   * to full-tree tracing because a pool was exceeded). */
  int lp_stat_active_overflow_ = 0;
  int lp_stat_tmp_overflow_ = 0;
  /* Throttling for the always-on overflow warning (the stats readback is a
   * synchronous stall, so it is only sampled every Nth prune). */
  int lp_prune_count_ = 0;
  bool lp_overflow_warned_ = false;

  /* Mesh volume bake state (bake_records_ / bake_*_ssbo_ pools) lives in
   * SdfInstanceBase, shared with the classic engine. */

 public:
  LpInstance() {}

  blender::StringRefNull name_get() override
  {
    return "SDF LP";
  }

 protected:
  Span<const int> engine_shader_list() const override
  {
    return Span<const int>(kLpShaders, ARRAY_SIZE(kLpShaders));
  }

  void sync_extra() override
  {
    lp_build_tree();
  }

  void sync_engine_settings(const View3DShading &s) override
  {
    lp_enable_pruning_ = s.sdf_lp_enable_pruning != 0;
    lp_shading_mode_ = s.sdf_lp_shading_mode;
    lp_grid_level_ = s.sdf_lp_grid_level >= 2 ? s.sdf_lp_grid_level : 6;
    lp_colormap_max_ = s.sdf_lp_colormap_max > 0 ? s.sdf_lp_colormap_max : 25;
    lp_aabb_auto_ = s.sdf_lp_aabb_auto != 0;
    lp_aabb_min_user_ = float3(s.sdf_lp_aabb_min);
    lp_aabb_max_user_ = float3(s.sdf_lp_aabb_max);
  }

  void upload_extra() override
  {
    lp_upload_tree();
  }

  void pre_trace_hook() override
  {
    int lvl = std::clamp(lp_grid_level_, 2, 8);
    lvl += lvl & 1;
    lp_grid_level_ = lvl;

    float3 want_min, want_max;
    if (lp_aabb_auto_) {
      want_min = scene_min_;
      want_max = scene_max_;
      if (!(want_max.x > want_min.x) || !(want_max.y > want_min.y) ||
          !(want_max.z > want_min.z))
      {
        want_min = float3(-1.0f);
        want_max = float3(1.0f);
      }
      else {
        float3 pad = (want_max - want_min) * 0.001f + float3(1e-4f);
        want_min -= pad;
        want_max += pad;
      }
    }
    else {
      want_min = lp_aabb_min_user_;
      want_max = lp_aabb_max_user_;
      /* Guard against degenerate user input. */
      want_max = math::max(want_max, want_min + float3(1e-4f));
    }

    if (lvl != lp_grid_level_built_ || want_min != lp_aabb_min_ || want_max != lp_aabb_max_ ||
        scene_changed_ || mesh_changed_ || mesh_data_changed_)
    {
      if (!lp_grid_dirty_) {
        CLOG_INFO(&LOG,
                  "SDF LP grid invalidated: level %d->%d, aabb_changed=%d, scene_changed=%d, "
                  "mesh_changed=%d, mesh_data_changed=%d",
                  lp_grid_level_built_, lvl,
                  int(want_min != lp_aabb_min_ || want_max != lp_aabb_max_),
                  int(scene_changed_), int(mesh_changed_), int(mesh_data_changed_));
      }
      lp_grid_dirty_ = true;
      lp_aabb_min_ = want_min;
      lp_aabb_max_ = want_max;
    }
  }

  void hash_shading_extra(uint64_t &sh) override
  {
    auto hash_shading = [&](const void *data, size_t size) {
      const uint8_t *p = static_cast<const uint8_t *>(data);
      for (size_t i = 0; i < size; i++) {
        sh ^= p[i];
        sh *= 0x100000001b3ULL;
      }
    };
    hash_shading(&lp_enable_pruning_, sizeof(lp_enable_pruning_));
    hash_shading(&lp_shading_mode_, sizeof(lp_shading_mode_));
    hash_shading(&lp_grid_level_, sizeof(lp_grid_level_));
    hash_shading(&lp_colormap_max_, sizeof(lp_colormap_max_));
    hash_shading(&lp_aabb_auto_, sizeof(lp_aabb_auto_));
    hash_shading(lp_aabb_min_user_, sizeof(lp_aabb_min_user_));
    hash_shading(lp_aabb_max_user_, sizeof(lp_aabb_max_user_));
  }

  int effective_lighting(int lighting) override
  {
    return (lp_shading_mode_ != 0) ? 0 : lighting;
  }

  void draw_trace_pipeline(bool profiling) override
  {
#define PROF_START(name) if (profiling) { s_profiler.mark_start(name); }
#define PROF_END()       if (profiling) { s_profiler.mark_end(); }

    /* Dense per-mesh volume bakes: (re)bake any new/changed mesh payloads
     * into the shared voxel pools before pruning/marching. Self-contained
     * and idempotent; runtime sampling is the SDF_LP_MESH_FLAG_BAKED fast
     * path. When the update re-resolved baked object fields (records flipped
     * to ready or pool ranges reassigned), rebuild the tree: make_leaf
     * captures those fields into lp_prims_. */
    PROF_START("Mesh Bake");
    GPU_debug_group_begin("SDF LP Mesh Bake");
    const bool bake_changed = update_mesh_bakes();
    GPU_debug_group_end();
    PROF_END();
    if (bake_changed) {
      lp_build_tree();
      lp_upload_tree();
      lp_grid_dirty_ = true;
    }

    /* Lipschitz pruning path: build/refresh the pruning grid when the scene,
     * grid level or AABB changed (smart recompute; see pre_trace_hook), then
     * march against the per-cell active node lists. Shading reuses the
     * classic shade pass (lighting forced off for debug modes in draw_shade). */
    if (lp_enable_pruning_ && !lp_nodes_.is_empty() && (lp_grid_dirty_ || !lp_grid_valid_))
    {
      CLOG_INFO(&LOG,
                "SDF LP prune running: grid_dirty=%d, grid_valid=%d",
                int(lp_grid_dirty_), int(lp_grid_valid_));
      PROF_START("LP Prune");
      GPU_debug_group_begin("SDF LP Prune");
      lp_ensure_grid_buffers(lp_grid_level_);
      draw_lp_prune();
      GPU_debug_group_end();
      GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
      PROF_END();
      lp_grid_dirty_ = false;
    }

    PROF_START("LP March");
    GPU_debug_group_begin("SDF LP March");
    draw_lp_march();
    GPU_debug_group_end();
    GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
    PROF_END();

    /* All shading modes reuse the classic color+normal evaluation (per-tile
     * candidate lists folded over the shared scene SSBOs): the LP folded
     * color/normal resolve is gone — besides skipping its minutes-long
     * driver compile, the classic resolve also applies group distance
     * modifiers, which the folded LP evaluator did not support. */
    PROF_START("AABB Project");
    GPU_debug_group_begin("SDF AABB Project");
    draw_aabb_project();
    GPU_debug_group_end();
    GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
    PROF_END();

    PROF_START("Tile Cull");
    GPU_debug_group_begin("SDF Tile Cull");
    draw_tile_cull();
    GPU_debug_group_end();
    GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
    PROF_END();

    PROF_START("Color Resolve");
    GPU_debug_group_begin("SDF Color Resolve");
    draw_color_resolve();
    GPU_debug_group_end();
    GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
    PROF_END();

    /* Same screen-space normal reconstruction as the classic path. The
     * finite-difference gradient the LP trace computes bands along the
     * ROUND fillet fold lines (mirror swap at a==b, min(ad,corn)
     * selection); reconstructing from the position buffer matches the
     * classic engine's normals exactly. LP trace marks exact mesh
     * normals with w=1 so they survive. */
    PROF_START("Normal");
    GPU_debug_group_begin("SDF Normal");
    draw_normal();
    GPU_debug_group_end();
    GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
    PROF_END();

    /* Debug shading modes (heatmap/normals): cheap recolor from the cell
     * metadata / the just-computed normals — no SDF tree evaluation. */
    if (lp_shading_mode_ != 0) {
      PROF_START("LP Debug");
      GPU_debug_group_begin("SDF LP Debug");
      draw_lp_debug();
      GPU_debug_group_end();
      GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
      PROF_END();
    }

    PROF_START("Shade");
    GPU_debug_group_begin("SDF Shade");
    draw_shade();
    GPU_debug_group_end();
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    PROF_END();

    compute_valid_ = true;

#undef PROF_START
#undef PROF_END
  }

  /* -------------------------------------------------------------------- */
  /** \name Lipschitz pruning: CSG tree build, buffers, passes
   * \{ */

  static uint4 lp_pack_binary_op(float k,
                                 int op,
                                 int blend_type,
                                 float k2,
                                 float k3,
                                 uint32_t flags)
  {
    uint32_t sign = (op == SDF_LP_CSG_UNION) ? 1u : 0u;
    uint32_t k_bits;
    memcpy(&k_bits, &k, sizeof(float));
    k_bits &= SDF_LP_OP_KMASK; /* low 6 bits reused for blend type + op + sign */
    uint32_t k2_bits, k3_bits;
    memcpy(&k2_bits, &k2, sizeof(float));
    memcpy(&k3_bits, &k3, sizeof(float));
    return uint4(k_bits | (uint32_t(blend_type & 3) << 4) | (uint32_t(op & 7) << 1) | sign,
                 k2_bits,
                 k3_bits,
                 flags);
  }

  static bool lp_object_supported(const SDFObjectGPU &obj)
  {
    switch (obj.sdf_type) {
      case SDF_GPU_TYPE_BOX:
      case SDF_GPU_TYPE_SPHERE:
      case SDF_GPU_TYPE_CYLINDER:
      case SDF_GPU_TYPE_CONE:
      case SDF_GPU_TYPE_CAPSULE:
      case SDF_GPU_TYPE_TORUS:
      case SDF_GPU_TYPE_NGON:
      case SDF_GPU_TYPE_POLYGON:
      case SDF_GPU_TYPE_MESH:
        return true;
      default:
        return false;
    }
  }

  /* Build the post-order CSG tree (flat fold of the sorted object list, groups
   * folded as subtrees) from objects_/groups_gpu_. All analytic primitive
   * types (including advanced variants and objects with modifiers)
   * participate; only unknown type ids are skipped. SHELL/PUSH/AVOID are
   * desugared into UNION/SUBTRACT/INTERSECT + OFFSET nodes (see combineCSG in
   * sdf_lib.glsl); PAINT stays an opaque binary op. */
  void lp_build_tree()
  {
    lp_nodes_.clear();
    lp_prims_.clear();
    lp_binary_ops_.clear();
    lp_active_init_nodes_.clear();
    lp_active_init_parents_.clear();

    const int n = int(objects_.size());
    if (n == 0) {
      return;
    }

    struct BuildNode {
      int type;
      int prim_idx;
      int op;
      float blend;
      int blend_type;
      int left, right;
      /* OFFSET nodes: single child + float offset (left/right unused). */
      int child;
      float offset;
      /* Start/end edge softness radii (classic chamfer_k2/k3 or k4/k5;
       * CHAMFER/ROUND with blend > 0 only) and SDF_LP_OP_FLAG_*. */
      float k2, k3;
      uint32_t flags;
    };
    Vector<BuildNode> build;
    build.reserve(n * 2);

    auto make_leaf = [&](int i) -> int {
      const SDFObjectGPU &obj = objects_[i];
      if (!lp_object_supported(obj)) {
        return -1;
      }
      SDFLpPrimitive prim = {};
      const float4x4 &im = obj.inverse_matrix;
      prim.m_row0 = float4(im[0][0], im[1][0], im[2][0], im[3][0]);
      prim.m_row1 = float4(im[0][1], im[1][1], im[2][1], im[3][1]);
      prim.m_row2 = float4(im[0][2], im[1][2], im[2][2], im[3][2]);
      prim.position = obj.position;
      prim.size = obj.sdf_size;
      prim.scale = obj.obj_scale;
      prim.color = float4(obj.color.x, obj.color.y, obj.color.z, float(i));
      prim.type = obj.sdf_type;
      if (obj.sdf_type == SDF_GPU_TYPE_NGON) {
        prim.aux0 = obj.box_modes.z;
      }
      else if (obj.sdf_type == SDF_GPU_TYPE_POLYGON) {
        prim.aux0 = obj.polygon_point_start;
        prim.aux1 = obj.polygon_point_count;
        prim.auxf = obj.box_corners.x;
      }
      if (obj.sdf_type == SDF_GPU_TYPE_MESH) {
        prim.mesh_data = obj.mesh_data;
        prim.mesh_node_count = obj.mesh_settings.z;
        prim.mesh_flags = obj.mesh_settings.y;
        if ((obj.mesh_settings.y & SDF_LP_MESH_FLAG_BAKED) != 0) {
          /* Baked volume: reuse the unused-for-mesh box_* fields for the
           * grid (see SDFLpPrimitive in sdf_shader_shared.hh). The record
           * lookup already happened in object_sync — the object's bake_*
           * fields are the single source of truth. */
          prim.box_corners = obj.bake_origin;
          prim.box_edges = obj.bake_params;
          prim.box_modes = obj.bake_grid;
          prim.coarse_origin = obj.bake_coarse_origin;
          prim.coarse_grid = obj.bake_coarse_grid;
        }
      }
      if (obj.sdf_type != SDF_GPU_TYPE_MESH ||
          (obj.mesh_settings.y & SDF_LP_MESH_FLAG_BAKED) == 0)
      {
        prim.box_corners = obj.box_corners;
        prim.box_edges = obj.box_edges;
        prim.box_modes = obj.box_modes;
      }
      if (obj.sdf_type == SDF_GPU_TYPE_POLYGON) {
        /* box_corners.zw are unused by polygons: carry the blend/clearance
         * reach for the coarse-grid slack (lp_sd_polygon_2d_bvh). */
        float blend_reach = (obj.blend_type != 0) ? obj.blend : 0.0f;
        prim.box_corners.z = math::max(blend_reach, obj.max_group_blend);
        prim.box_corners.w = obj.clearance;
      }
      prim.modifier_start = obj.modifier_start;
      prim.modifier_count = obj.modifier_count;
      prim.obj_index = i;
      lp_prims_.append(prim);
      build.append({SDF_LP_NODETYPE_PRIMITIVE,
                    int(lp_prims_.size()) - 1,
                    0,
                    0.0f,
                    0,
                    -1,
                    -1,
                    -1,
                    0.0f,
                    0.0f,
                    0.0f,
                    0u});
      return int(build.size()) - 1;
    };

    auto make_op = [&](int op,
                       float blend,
                       int blend_type,
                       int left,
                       int right,
                       float k2,
                       float k3,
                       uint32_t flags) -> int {
      build.append(
          {SDF_LP_NODETYPE_BINARY, -1, op, blend, blend_type, left, right, -1, 0.0f, k2, k3, flags});
      return int(build.size()) - 1;
    };

    auto make_offset = [&](int child, float offset) -> int {
      build.append(
          {SDF_LP_NODETYPE_OFFSET, -1, 0, 0.0f, 0, -1, -1, child, offset, 0.0f, 0.0f, 0u});
      return int(build.size()) - 1;
    };

    /* Number of BuildNodes in the subtree rooted at bidx. */
    auto subtree_size = [&](int bidx) -> int {
      int count = 0;
      Vector<int> stack;
      stack.append(bidx);
      while (!stack.is_empty()) {
        const int cur = stack.last();
        stack.remove_last();
        count++;
        if (build[cur].type == SDF_LP_NODETYPE_BINARY) {
          stack.append(build[cur].left);
          stack.append(build[cur].right);
        }
        else if (build[cur].type == SDF_LP_NODETYPE_OFFSET) {
          stack.append(build[cur].child);
        }
      }
      return count;
    };

    /* Deep-copy the subtree rooted at bidx, appending the copies to `build`.
     * Leaf copies reference the SAME lp_prims_ entry (primitives are not
     * duplicated). Returns the root of the copy. */
    auto dup_subtree = [&](int bidx) -> int {
      const int old_size = int(build.size());
      Vector<int> copy_of(old_size, -1);

      /* Post-order traversal of the source subtree. */
      Vector<int> stack;
      Vector<int> order;
      stack.append(bidx);
      while (!stack.is_empty()) {
        const int cur = stack.last();
        stack.remove_last();
        order.append(cur);
        if (build[cur].type == SDF_LP_NODETYPE_BINARY) {
          stack.append(build[cur].left);
          stack.append(build[cur].right);
        }
        else if (build[cur].type == SDF_LP_NODETYPE_OFFSET) {
          stack.append(build[cur].child);
        }
      }

      for (int j = int(order.size()) - 1; j >= 0; j--) {
        const int cur = order[j];
        BuildNode bn = build[cur];
        if (bn.type == SDF_LP_NODETYPE_BINARY) {
          bn.left = copy_of[bn.left];
          bn.right = copy_of[bn.right];
        }
        else if (bn.type == SDF_LP_NODETYPE_OFFSET) {
          bn.child = copy_of[bn.child];
        }
        copy_of[cur] = int(build.size());
        build.append(bn);
      }
      return copy_of[bidx];
    };

    /* The pruning shader packs tmp state into 16 bits, so the flattened tree
     * must stay within 65535 nodes. Desugared ops duplicate subtrees; if a
     * duplication would blow the budget, fall back to a plain UNION for that
     * op (correct pruning, approximate geometry for that op). */
    bool dup_overflow_warned = false;
    auto can_dup = [&](int bidx) -> bool {
      if (int(build.size()) + subtree_size(bidx) + 8 <= 65535) {
        return true;
      }
      if (!dup_overflow_warned) {
        dup_overflow_warned = true;
        CLOG_WARN(&LOG,
                  "SDF LP tree build: node budget exceeded by CSG desugar, "
                  "falling back to plain UNION for the affected op(s)");
      }
      return false;
    };

    /* Effective blend params: k rides only on non-LINEAR blend types. */
    auto blend_k = [](float blend, int blend_type, float &k, int &bt) {
      bt = blend_type & 3;
      k = (bt != SDF_BLEND_LINEAR) ? blend : 0.0f;
    };

    /* Combine accumulated left subtree `acc` (d1) with right operand subtree
     * `operand` (d2) using the right operand's CSG parameters. Mirrors
     * combineCSG in sdf_lib.glsl; SHELL/PUSH/AVOID are desugared here.
     * k2/k3 are the start-edge softness radii (classic chamfer_k2/k3), k4/k5
     * the end-edge radii (shell limit-plane op only); flip_blend/flip_blend_end
     * follow the classic shell branch (:1585-1735). */
    auto combine = [&](int acc,
                       int operand,
                       int csg_operation,
                       int blend_type,
                       float blend,
                       float clearance,
                       float shell_distance,
                       int shell_mode,
                       int shell_op,
                       float shell_blend_top,
                       float shell_blend_bottom,
                       float color_blend,
                       float chamfer_k2,
                       float chamfer_k3,
                       float chamfer_k4,
                       float chamfer_k5,
                       int flip_blend,
                       int flip_blend_end) -> int
    {
      float k;
      int bt;
      /* Softness radii only ride on CHAMFER/ROUND ops with k > 0 (classic
       * combineCSG only reads them in those branches). */
      auto smooth_k = [](int bt_, float k_, float ka, float kb, float &s2, float &s3) {
        if (k_ > 0.0f && (bt_ == SDF_BLEND_CHAMFER || bt_ == SDF_BLEND_ROUND)) {
          s2 = ka;
          s3 = kb;
        }
        else {
          s2 = 0.0f;
          s3 = 0.0f;
        }
      };
      float s2, s3;
      switch (csg_operation) {
        case SDF_CSG_SUBTRACT:
        case SDF_CSG_INTERSECT:
        case SDF_CSG_UNION:
          blend_k(blend, blend_type, k, bt);
          smooth_k(bt, k, chamfer_k2, chamfer_k3, s2, s3);
          return make_op(csg_operation, k, bt, acc, operand, s2, s3, 0u);

        case SDF_CSG_PAINT:
          /* Geometry = left operand; color blend radius (NOT blend), LINEAR. */
          return make_op(SDF_LP_CSG_PAINT, color_blend, SDF_LP_BLEND_LINEAR, acc, operand,
                         0.0f, 0.0f, 0u);

        case SDF_CSG_PUSH: {
          /* result = min(max(d1, -(d2 - c)), d2) (sdf_lib.glsl:1539-1562). */
          if (!can_dup(operand)) {
            return make_op(SDF_LP_CSG_UNION, 0.0f, SDF_LP_BLEND_LINEAR, acc, operand,
                           0.0f, 0.0f, 0u);
          }
          blend_k(blend, blend_type, k, bt);
          smooth_k(bt, k, chamfer_k2, chamfer_k3, s2, s3);
          int sub = make_op(
              SDF_LP_CSG_SUBTRACT, k, bt, acc, make_offset(operand, -clearance), s2, s3, 0u);
          return make_op(
              SDF_LP_CSG_UNION, 0.0f, SDF_LP_BLEND_LINEAR, sub, dup_subtree(operand),
              0.0f, 0.0f, 0u);
        }

        case SDF_CSG_AVOID: {
          /* result = min(d1, max(d2, -(d1 - c))) (sdf_lib.glsl:1563-1584). */
          if (!can_dup(acc)) {
            return make_op(SDF_LP_CSG_UNION, 0.0f, SDF_LP_BLEND_LINEAR, acc, operand,
                           0.0f, 0.0f, 0u);
          }
          blend_k(blend, blend_type, k, bt);
          smooth_k(bt, k, chamfer_k2, chamfer_k3, s2, s3);
          int carved = make_op(SDF_LP_CSG_SUBTRACT,
                               k,
                               bt,
                               operand,
                               make_offset(dup_subtree(acc), -clearance),
                               s2,
                               s3,
                               0u);
          return make_op(SDF_LP_CSG_UNION, 0.0f, SDF_LP_BLEND_LINEAR, acc, carved,
                         0.0f, 0.0f, 0u);
        }

        case SDF_CSG_SHELL: {
          /* Shell branch of combineCSG (sdf_lib.glsl:1580-1735), including
           * flip_blend (start edge) and flip_blend_end (end edge).
           * TODO: SHELL_MODE_PUSH/AVOID extra carve. */
          (void)shell_mode;
          if (!can_dup(acc)) {
            return make_op(SDF_LP_CSG_UNION, 0.0f, SDF_LP_BLEND_LINEAR, acc, operand,
                           0.0f, 0.0f, 0u);
          }
          const float sd = (shell_op == SDF_SHELL_OP_SUBTRACTION) ? -shell_distance :
                                                                    shell_distance;
          const float h = fabsf(sd);
          float k_top = (blend_type != SDF_BLEND_LINEAR && shell_blend_top > 0.0f) ?
                            shell_blend_top :
                            0.0f;
          float k_bot = (blend_type != SDF_BLEND_LINEAR && shell_blend_bottom > 0.0f) ?
                            shell_blend_bottom :
                            0.0f;
          int bt_top = (k_top > 0.0f) ? (blend_type & 3) : SDF_LP_BLEND_LINEAR;
          int bt_bot = (k_bot > 0.0f) ? (blend_type & 3) : SDF_LP_BLEND_LINEAR;
          const bool fb = (flip_blend != 0);
          const bool fbe = (flip_blend_end != 0);
          /* Start-edge softness on the top op, end-edge on the bottom op. */
          float t2, t3, e2, e3;
          smooth_k(bt_top, k_top, chamfer_k2, chamfer_k3, t2, t3);
          smooth_k(bt_bot, k_bot, chamfer_k4, chamfer_k5, e2, e3);
          if (sd < 0.0f) {
            /* Inward: start = subtraction (bottom edge), end = union with the
             * d1 + h limit plane (top edge). */
            int t;
            if (bt_top == SDF_LP_BLEND_ROUND && !fb) {
              /* Classic uses opIntersectionRound(d1, -d2, k) here (:1616),
               * not the opRoundSubtraction duality form, and has no smooth
               * k2/k3 variant on this path. */
              t = make_op(SDF_LP_CSG_SUBTRACT, k_top, bt_top, acc, operand,
                          0.0f, 0.0f, SDF_LP_OP_FLAG_INVERTED);
            }
            else {
              /* SMOOTH/CHAMFER: flip and non-flip are identical in classic.
               * ROUND flip: opRoundSubtraction(d2, d1, k[, k2, k3]) (:1606),
               * which is exactly the LP SUBTRACT dispatch ((d2,d1) order). */
              t = make_op(SDF_LP_CSG_SUBTRACT, k_top, bt_top, acc, operand, t2, t3, 0u);
            }
            int lim = make_offset(dup_subtree(acc), h);
            /* The classic end-edge flip branch only runs when shell_k_bot > 0
             * (sdf_lib.glsl:1615); with k_bot == 0 (or a LINEAR blend) the
             * classic result is the plain min(d_sub, lim). Taking the flip
             * decomposition anyway would produce min(max(d_sub, -lim), lim)
             * instead — a different field with a different zero set. */
            if (!fbe || bt_bot == SDF_LP_BLEND_LINEAR) {
              return make_op(SDF_LP_CSG_UNION, k_bot, bt_bot, t, lim, e2, e3, 0u);
            }
            /* Flip end: min(subtract-variant(lim, d_sub), lim) for every
             * blend type (:1622-1644; the ROUND opRoundUnionInverted /
             * opSmoothRoundUnionInverted forms decompose into exactly this:
             * min(opRoundSubtraction(d_sub, lim), lim)). Needs a second copy
             * of the limit plane. */
            if (!can_dup(acc)) {
              return make_op(SDF_LP_CSG_UNION, k_bot, bt_bot, t, lim, e2, e3, 0u);
            }
            /* Plain ROUND needs the operands swapped: the classic
             * opRoundUnionInverted(d_sub, lim) is min(-iround(d_sub, -lim),
             * lim), the LP SUBTRACT evaluates -iround(-L, R), and iround is
             * sign-asymmetric (max(x,0) terms), so -iround(-L,R) with L=lim,
             * R=d_sub gives -iround(-lim, d_sub) == -iround(d_sub, -lim) by
             * the mirror symmetry. The smooth k4/k5 form and the other blend
             * types match with the regular (t, lim) order (verified
             * numerically). */
            const bool round_plain = (bt_bot == SDF_LP_BLEND_ROUND && e2 == 0.0f && e3 == 0.0f);
            int e = round_plain ?
                        make_op(SDF_LP_CSG_SUBTRACT, k_bot, bt_bot, lim, t, 0.0f, 0.0f, 0u) :
                        make_op(SDF_LP_CSG_SUBTRACT, k_bot, bt_bot, t, lim, e2, e3, 0u);
            int lim2 = make_offset(dup_subtree(acc), h);
            return make_op(SDF_LP_CSG_UNION, 0.0f, SDF_LP_BLEND_LINEAR, e, lim2,
                           0.0f, 0.0f, 0u);
          }
          /* Outward: start = union (top edge), end = intersection with the
           * d1 - h limit plane (bottom edge). */
          int t;
          if (fb && bt_top != SDF_LP_BLEND_LINEAR) {
            /* Flip start, push-like: min(subtract-variant(d2, d1), d2)
             * (:1656-1670). Needs a copy of the operand. */
            if (!can_dup(operand)) {
              t = make_op(SDF_LP_CSG_UNION, k_top, bt_top, acc, operand, t2, t3, 0u);
            }
            else {
              int sub = make_op(SDF_LP_CSG_SUBTRACT, k_top, bt_top, acc, operand, t2, t3, 0u);
              t = make_op(SDF_LP_CSG_UNION, 0.0f, SDF_LP_BLEND_LINEAR, sub,
                          dup_subtree(operand), 0.0f, 0.0f, 0u);
            }
          }
          else {
            t = make_op(SDF_LP_CSG_UNION, k_top, bt_top, acc, operand, t2, t3, 0u);
          }
          int lim = make_offset(dup_subtree(acc), -h);
          /* Same k_bot guard as the inward branch: the classic flip-end
           * branch is skipped when shell_k_bot == 0 or the blend is LINEAR
           * (sdf_lib.glsl:1681 -> else :1710), where the result is the plain
           * max(d_union, lim). Without the guard the decomposition below
           * evaluates max(max(d_union, -lim), lim) — a different zero set. */
          if (!fbe || bt_bot == SDF_LP_BLEND_LINEAR) {
            int bt_b = bt_bot;
            float be2 = e2, be3 = e3;
            if (bt_bot == SDF_LP_BLEND_ROUND) {
              /* Classic non-flip outward end edge with ROUND uses the
               * quadratic opSmoothIntersection (:1723), not a round kernel,
               * and ignores k4/k5. */
              bt_b = SDF_LP_BLEND_SMOOTH;
              be2 = 0.0f;
              be3 = 0.0f;
            }
            return make_op(SDF_LP_CSG_INTERSECT, k_bot, bt_b, t, lim, be2, be3, 0u);
          }
          if (bt_bot == SDF_LP_BLEND_ROUND) {
            /* Flip end ROUND: opRoundIntersection / the smooth
             * opSmoothRoundIntersectionInverted (:1694-1701) — a single op,
             * selected by the INVERTED flag. */
            return make_op(SDF_LP_CSG_INTERSECT, k_bot, bt_bot, t, lim, e2, e3,
                           SDF_LP_OP_FLAG_INVERTED);
          }
          /* Flip end SMOOTH/CHAMFER: max(subtract-variant(lim, d_union), lim)
           * (:1677-1692). Needs a second copy of the limit plane. */
          if (!can_dup(acc)) {
            return make_op(SDF_LP_CSG_INTERSECT, k_bot, bt_bot, t, lim, e2, e3, 0u);
          }
          int e = make_op(SDF_LP_CSG_SUBTRACT, k_bot, bt_bot, t, lim, e2, e3, 0u);
          int lim2 = make_offset(dup_subtree(acc), -h);
          return make_op(SDF_LP_CSG_INTERSECT, 0.0f, SDF_LP_BLEND_LINEAR, e, lim2,
                         0.0f, 0.0f, 0u);
        }

        default:
          return make_op(SDF_LP_CSG_UNION, 0.0f, SDF_LP_BLEND_LINEAR, acc, operand,
                         0.0f, 0.0f, 0u);
      }
    };

    /* Fold a group's member range into a subtree. */
    auto fold_leaves = [&](int begin, int end) -> int {
      int acc = -1;
      for (int i = begin; i < end; i++) {
        int operand = make_leaf(i);
        if (operand < 0) {
          continue;
        }
        if (acc < 0) {
          acc = operand;
          continue;
        }
        const SDFObjectGPU &obj = objects_[i];
        acc = combine(acc,
                      operand,
                      obj.csg_operation,
                      obj.blend_type,
                      obj.blend,
                      obj.clearance,
                      obj.shell_distance,
                      obj.shell_mode,
                      obj.shell_op,
                      obj.shell_blend_top,
                      obj.shell_blend_bottom,
                      obj.color_blend,
                      obj.chamfer_k2,
                      obj.chamfer_k3,
                      obj.chamfer_k4,
                      obj.chamfer_k5,
                      obj.flip_blend,
                      obj.flip_blend_end);
      }
      return acc;
    };

    /* Scene fold: groups as subtrees, ungrouped objects as leaves. */
    int root = -1;
    int i = 0;
    while (i < n) {
      const int gi = objects_[i].group_id;
      int operand;
      int consumed;
      if (gi >= 0 && gi < int(groups_gpu_.size()) && i == groups_gpu_[gi].first_object &&
          groups_gpu_[gi].object_count > 0)
      {
        const SDFGroupGPU &grp = groups_gpu_[gi];
        operand = fold_leaves(i, i + grp.object_count);
        if (operand >= 0 && root >= 0) {
          root = combine(root,
                         operand,
                         grp.csg_operation,
                         grp.blend_type,
                         grp.blend,
                         grp.clearance,
                         grp.shell_distance,
                         grp.shell_mode,
                         grp.shell_op,
                         grp.shell_blend_top,
                         grp.shell_blend_bottom,
                         grp.color_blend,
                         grp.chamfer_k2,
                         grp.chamfer_k3,
                         grp.chamfer_k4,
                         grp.chamfer_k5,
                         grp.flip_blend,
                         grp.flip_blend_end);
        }
        else if (operand >= 0) {
          root = operand;
        }
        consumed = grp.object_count;
      }
      else {
        operand = make_leaf(i);
        if (operand >= 0 && root >= 0) {
          const SDFObjectGPU &obj = objects_[i];
          root = combine(root,
                         operand,
                         obj.csg_operation,
                         obj.blend_type,
                         obj.blend,
                         obj.clearance,
                         obj.shell_distance,
                         obj.shell_mode,
                         obj.shell_op,
                         obj.shell_blend_top,
                         obj.shell_blend_bottom,
                         obj.color_blend,
                         obj.chamfer_k2,
                         obj.chamfer_k3,
                         obj.chamfer_k4,
                         obj.chamfer_k5,
                         obj.flip_blend,
                         obj.flip_blend_end);
        }
        else if (operand >= 0) {
          root = operand;
        }
        consumed = 1;
      }
      i += consumed;
    }

    if (root < 0) {
      return;
    }

    /* Serialize post-order (left subtree, right subtree, node). Signs are
     * absolute, not propagated (reference scene.cpp): every node is positive
     * except the immediate right child of a SUB op, whose output is negated.
     * The op word's sign bit (+1 union, -1 sub/inter) combined with the
     * negated right operand yields sub/inter from the common min() form. */
    Vector<int> gpu_of_build(build.size(), -1);
    lp_active_init_nodes_.resize(build.size());
    lp_active_init_parents_.resize(build.size());
    lp_nodes_.reserve(build.size());

    Vector<int> stack;
    Vector<bool> stack_sign;
    stack.append(root);
    stack_sign.append(false);
    Vector<int> order;
    Vector<bool> order_sign;
    order.reserve(build.size());
    order_sign.reserve(build.size());
    while (!stack.is_empty()) {
      const int bidx = stack.last();
      const bool sign = stack_sign.last();
      stack.remove_last();
      stack_sign.remove_last();
      order.append(bidx);
      order_sign.append(sign);
      if (build[bidx].type == SDF_LP_NODETYPE_BINARY) {
        stack.append(build[bidx].left);
        stack_sign.append(false);
        stack.append(build[bidx].right);
        stack_sign.append(build[bidx].op == SDF_LP_CSG_SUBTRACT);
      }
      else if (build[bidx].type == SDF_LP_NODETYPE_OFFSET) {
        /* Unary node: the sign stays on this node (it negates child+offset as
         * a whole); the child edge itself is always positive. */
        stack.append(build[bidx].child);
        stack_sign.append(false);
      }
    }

    for (int j = int(order.size()) - 1; j >= 0; j--) {
      const int bidx = order[j];
      const bool sign = order_sign[j];
      const BuildNode &bn = build[bidx];
      const int self_idx = int(lp_nodes_.size());

      SDFLpNode gpu_node = {};
      gpu_node.type = bn.type;
      if (bn.type == SDF_LP_NODETYPE_BINARY) {
        gpu_node.idx_in_type = int(lp_binary_ops_.size());
        lp_binary_ops_.append(
            lp_pack_binary_op(bn.blend, bn.op, bn.blend_type, bn.k2, bn.k3, bn.flags));
        lp_active_init_parents_[gpu_of_build[bn.left]] = uint32_t(self_idx);
        lp_active_init_parents_[gpu_of_build[bn.right]] = uint32_t(self_idx);
        /* Per-subtree Lipschitz constant w.r.t. position. The ROUND fillet
         * (lp_op_iround, and the lp_op_intersection_round / smooth-round
         * variants, whose corner arc gradients also sweep the full unit
         * circle) is 1-Lipschitz in the Euclidean (a,b)-metric, which
         * composes with two child fields of constants Ll/Lr into
         * sqrt(Ll^2+Lr^2) (tight: sqrt(2) along the fillet crest of two
         * parallel surfaces, where both child gradients align). All other
         * blend types — including the smooth k2/k3 CHAMFER variants, whose
         * (a,b)-gradients are convex combinations (weights sum to 1) — are
         * 1-Lipschitz in the max metric, so they keep max(Ll,Lr). Measured
         * numerically on plane fields (dense secant probes, orthogonal and
         * parallel normals). Negation (SUB) and PAINT do not change the
         * constant. */
        const float ll = lp_nodes_[gpu_of_build[bn.left]].lipschitz;
        const float lr = lp_nodes_[gpu_of_build[bn.right]].lipschitz;
        gpu_node.lipschitz = (bn.blend_type == SDF_BLEND_ROUND && bn.blend > 0.0f) ?
                                 sqrtf(ll * ll + lr * lr) :
                                 math::max(ll, lr);
      }
      else if (bn.type == SDF_LP_NODETYPE_OFFSET) {
        /* Unary node: idx_in_type carries the float bits of the offset; the
         * child's parent is this node (same as a binary child), and this
         * node's own parent is assigned by its parent op as usual. */
        int offset_bits;
        memcpy(&offset_bits, &bn.offset, sizeof(float));
        gpu_node.idx_in_type = offset_bits;
        lp_active_init_parents_[gpu_of_build[bn.child]] = uint32_t(self_idx);
        /* Adding a constant preserves the child's Lipschitz constant. */
        gpu_node.lipschitz = lp_nodes_[gpu_of_build[bn.child]].lipschitz;
      }
      else {
        gpu_node.idx_in_type = bn.prim_idx;
        /* Primitives are (conservatively) 1-Lipschitz: exact SDFs, with the
         * min-axis scale and taper/displacement corrections already applied
         * inside the primitive evaluation. Baked mesh volumes get 1.5: the
         * trilinear/clamped baked field and its Voronoi sign-flip guard are
         * not strictly 1-Lipschitz (bounded jumps up to ~band between
         * cells); the margin keeps the prune far-cell bounds conservative. */
        const SDFLpPrimitive &leaf_prim = lp_prims_[bn.prim_idx];
        gpu_node.lipschitz =
            (leaf_prim.type == SDF_GPU_TYPE_MESH &&
             (leaf_prim.mesh_flags & SDF_LP_MESH_FLAG_BAKED) != 0) ?
                1.5f :
                1.0f;
      }
      lp_nodes_.append(gpu_node);
      lp_active_init_nodes_[self_idx] = uint32_t(self_idx) | (sign ? SDF_LP_SIGN_BIT : 0u);
      lp_active_init_parents_[self_idx] = SDF_LP_INVALID_INDEX;
      gpu_of_build[bidx] = self_idx;
    }

    /* The pruning shader packs parent indices into 16 bits of scratch state. */
    if (lp_nodes_.size() > 65535) {
      lp_nodes_.clear();
      lp_prims_.clear();
      lp_binary_ops_.clear();
      lp_active_init_nodes_.clear();
      lp_active_init_parents_.clear();
    }
  }

  void lp_upload_tree()
  {
    SDFLpNode dummy_node = {};
    SDFLpPrimitive dummy_prim = {};
    uint4 dummy_u4 = uint4(0u);
    uint32_t dummy_u = 0u;

    const int64_t num_nodes = math::max(int64_t(lp_nodes_.size()), int64_t(1));
    const int64_t num_prims = math::max(int64_t(lp_prims_.size()), int64_t(1));
    const int64_t num_ops = math::max(int64_t(lp_binary_ops_.size()), int64_t(1));

    if (lp_nodes_ssbo_) GPU_storagebuf_free(lp_nodes_ssbo_);
    lp_nodes_ssbo_ = GPU_storagebuf_create_ex(num_nodes * sizeof(SDFLpNode),
                                              lp_nodes_.is_empty() ? &dummy_node : lp_nodes_.data(),
                                              GPU_USAGE_DYNAMIC, "sdf_lp_nodes");
    if (lp_prims_ssbo_) GPU_storagebuf_free(lp_prims_ssbo_);
    lp_prims_ssbo_ = GPU_storagebuf_create_ex(num_prims * sizeof(SDFLpPrimitive),
                                              lp_prims_.is_empty() ? &dummy_prim : lp_prims_.data(),
                                              GPU_USAGE_DYNAMIC, "sdf_lp_prims");
    if (lp_binary_ops_ssbo_) GPU_storagebuf_free(lp_binary_ops_ssbo_);
    lp_binary_ops_ssbo_ = GPU_storagebuf_create_ex(
        num_ops * sizeof(uint4),
        lp_binary_ops_.is_empty() ? &dummy_u4 : lp_binary_ops_.data(),
        GPU_USAGE_DYNAMIC, "sdf_lp_binary_ops");
    if (lp_active_init_nodes_ssbo_) GPU_storagebuf_free(lp_active_init_nodes_ssbo_);
    lp_active_init_nodes_ssbo_ = GPU_storagebuf_create_ex(
        num_nodes * sizeof(uint32_t),
        lp_active_init_nodes_.is_empty() ? &dummy_u : lp_active_init_nodes_.data(),
        GPU_USAGE_DYNAMIC, "sdf_lp_active_init_nodes");
    if (lp_active_init_parents_ssbo_) GPU_storagebuf_free(lp_active_init_parents_ssbo_);
    lp_active_init_parents_ssbo_ = GPU_storagebuf_create_ex(
        num_nodes * sizeof(uint32_t),
        lp_active_init_parents_.is_empty() ? &dummy_u : lp_active_init_parents_.data(),
        GPU_USAGE_DYNAMIC, "sdf_lp_active_init_parents");
  }

  /* Ensure grid buffers are allocated for (at least) the given grid level.
   * Grow-only; invalidates pruning results when anything is reallocated. */
  void lp_ensure_grid_buffers(int grid_level)
  {
    const int gs = 1 << grid_level;
    const int64_t num_cells = int64_t(gs) * gs * gs;
    const int64_t num_nodes = math::max(int64_t(lp_nodes_.size()), int64_t(1));

    /* Active list streams: far-field cells store nothing, so a small multiple
     * of the cell count is plenty for typical scenes; the shader clamps writes
     * to the capacity as a safety net. ROUND-blend-heavy scenes need much more
     * headroom: the sqrt-composed per-node Lipschitz constants (root ~ sqrt(n)
     * for n nested ROUND ops) widen every dominance band, so near cells keep
     * most of the tree — a 20-object all-ROUND scene at 64^3 emits ~2.4M
     * entries, overflowing the old 2M pool and forcing most near cells onto
     * the full-tree fallback path (measured in tools/sdf_lp/scene_sim.py). */
    int64_t active_entries = math::max(num_cells * 8, int64_t(1) << 22);
    active_entries = math::min(active_entries, int64_t(1) << 26);

    /* Scratch is allocated per workgroup and only for non-trivial cells; the
     * worst case is cells_total * num_nodes, capped at a reasonable bound. */
    int64_t cells_total = 0;
    for (int lvl = 2; lvl <= grid_level; lvl += 2) {
      const int64_t g = int64_t(1) << lvl;
      cells_total += g * g * g;
    }
    int64_t tmp_entries = cells_total * num_nodes;
    tmp_entries = math::min(tmp_entries, int64_t(1) << 26);
    tmp_entries = math::max(tmp_entries, int64_t(1) << 16);

    if (lp_active_nodes_ssbo_[0] != nullptr && gs <= lp_grid_size_ &&
        active_entries <= lp_active_capacity_ && tmp_entries <= lp_tmp_capacity_)
    {
      return;
    }

    const int new_gs = math::max(gs, lp_grid_size_);
    const int64_t new_cells = int64_t(new_gs) * new_gs * new_gs;
    active_entries = math::max(active_entries, lp_active_capacity_);
    tmp_entries = math::max(tmp_entries, lp_tmp_capacity_);

    for (int p = 0; p < 2; p++) {
      if (lp_active_nodes_ssbo_[p]) GPU_storagebuf_free(lp_active_nodes_ssbo_[p]);
      if (lp_active_parents_ssbo_[p]) GPU_storagebuf_free(lp_active_parents_ssbo_[p]);
      if (lp_cell_meta_ssbo_[p]) GPU_storagebuf_free(lp_cell_meta_ssbo_[p]);
      lp_active_nodes_ssbo_[p] = GPU_storagebuf_create_ex(
          active_entries * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_active_nodes");
      lp_active_parents_ssbo_[p] = GPU_storagebuf_create_ex(
          active_entries * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_active_parents");
      lp_cell_meta_ssbo_[p] = GPU_storagebuf_create_ex(
          new_cells * sizeof(int4), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_cell_meta");
    }
    if (lp_tmp_ssbo_) GPU_storagebuf_free(lp_tmp_ssbo_);
    lp_tmp_ssbo_ = GPU_storagebuf_create_ex(
        tmp_entries * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_tmp");
    if (lp_scratch_ssbo_) GPU_storagebuf_free(lp_scratch_ssbo_);
    lp_scratch_ssbo_ = GPU_storagebuf_create_ex(
        tmp_entries * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_scratch");

    if (lp_active_count_ssbo_ == nullptr) {
      /* [0..15] = active-list counters per level, [16..31] = tmp counters. */
      lp_active_count_ssbo_ = GPU_storagebuf_create_ex(
          32 * sizeof(int32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_counters");
    }

    lp_grid_size_ = new_gs;
    lp_active_capacity_ = active_entries;
    lp_tmp_capacity_ = tmp_entries;
    lp_grid_valid_ = false;
  }

  void lp_bind_ssbo(gpu::Shader *sh, const char *name, gpu::StorageBuf *buf)
  {
    if (buf == nullptr) {
      return;
    }
    const int slot = GPU_shader_get_ssbo_binding(sh, name);
    if (slot >= 0) {
      GPU_storagebuf_bind(buf, slot);
    }
  }

  void draw_lp_prune()
  {
    gpu::Shader *sh = lp_prune_sh();
    if (sh == nullptr || lp_nodes_.is_empty()) {
      return;
    }

    GPU_storagebuf_clear_to_zero(lp_active_count_ssbo_);

    int in_idx = 0;
    int out_idx = 1;

    GPU_shader_bind(sh);
    for (int lvl = 2; lvl <= lp_grid_level_; lvl += 2) {
      const int cur_gs = 1 << lvl;
      const bool first = (lvl == 2);

      lp_bind_ssbo(sh, "lp_prims", lp_prims_ssbo_);
      lp_bind_ssbo(sh, "lp_nodes", lp_nodes_ssbo_);
      lp_bind_ssbo(sh, "lp_binary_ops", lp_binary_ops_ssbo_);
      lp_bind_ssbo(sh, "lp_active_in",
                   first ? lp_active_init_nodes_ssbo_ : lp_active_nodes_ssbo_[in_idx]);
      lp_bind_ssbo(sh, "lp_active_out", lp_active_nodes_ssbo_[out_idx]);
      lp_bind_ssbo(sh, "lp_active_parents_in",
                   first ? lp_active_init_parents_ssbo_ : lp_active_parents_ssbo_[in_idx]);
      lp_bind_ssbo(sh, "lp_active_parents_out", lp_active_parents_ssbo_[out_idx]);
      lp_bind_ssbo(sh, "lp_cell_meta_in", lp_cell_meta_ssbo_[in_idx]);
      lp_bind_ssbo(sh, "lp_cell_meta_out", lp_cell_meta_ssbo_[out_idx]);
      lp_bind_ssbo(sh, "lp_counters", lp_active_count_ssbo_);
      lp_bind_ssbo(sh, "lp_tmp", lp_tmp_ssbo_);
      lp_bind_ssbo(sh, "lp_scratch", lp_scratch_ssbo_);
      lp_bind_ssbo(sh, "objects", object_ssbo_);
      lp_bind_ssbo(sh, "sdf_modifiers", modifier_ssbo_);
      lp_bind_ssbo(sh, "polygon_points", polygon_ssbo_);
      lp_bind_ssbo(sh, "mesh_data_buf", mesh_data_ssbo_);
      lp_bind_ssbo(sh, "bake_dist", bake_dist_ssbo_);

      GPU_shader_uniform_3fv(sh, "aabb_min", lp_aabb_min_);
      GPU_shader_uniform_3fv(sh, "aabb_max", lp_aabb_max_);
      GPU_shader_uniform_1i(sh, "total_num_nodes", int(lp_nodes_.size()));
      GPU_shader_uniform_1i(sh, "grid_size", cur_gs);
      GPU_shader_uniform_1i(sh, "first_lvl", first ? 1 : 0);
      GPU_shader_uniform_1i(sh, "active_capacity", int(lp_active_capacity_));
      GPU_shader_uniform_1i(sh, "tmp_capacity", int(lp_tmp_capacity_));
      GPU_shader_uniform_1i(sh, "counter_slot", lvl);

      GPU_compute_dispatch(sh, cur_gs / 4, cur_gs / 4, cur_gs / 4);
      GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

      std::swap(in_idx, out_idx);
    }
    GPU_shader_unbind();
    lp_final_idx_ = in_idx;
    lp_grid_valid_ = true;
    lp_grid_level_built_ = lp_grid_level_;

    /* Read back overflow stats (cleared to zero at the top of this pass).
     * GPU_storagebuf_read is a synchronous stall, so only pay it when the
     * numbers are actually consumed: profiled frames, verbose logging, or
     * the periodic sample feeding the always-visible overflow warning. */
    lp_stat_active_overflow_ = 0;
    lp_stat_tmp_overflow_ = 0;
    const bool want_stats = s_profile_pending || CLOG_CHECK(&LOG, CLG_LEVEL_INFO) ||
                            ((++lp_prune_count_ & 31) == 0);
    if (want_stats) {
      int32_t counters[32];
      GPU_memory_barrier(GPU_BARRIER_BUFFER_UPDATE);
      GPU_storagebuf_read(lp_active_count_ssbo_, counters);
      lp_stat_active_overflow_ = counters[SDF_LP_STAT_ACTIVE_OVERFLOW];
      lp_stat_tmp_overflow_ = counters[SDF_LP_STAT_TMP_OVERFLOW];
    }
    if (s_profile_pending) {
      s_profile_result.lp_pruned = true;
      s_profile_result.lp_active_capacity = lp_active_capacity_;
      s_profile_result.lp_tmp_capacity = lp_tmp_capacity_;
      s_profile_result.lp_active_overflow = lp_stat_active_overflow_;
      s_profile_result.lp_tmp_overflow = lp_stat_tmp_overflow_;
    }
    if (lp_stat_active_overflow_ > 0 || lp_stat_tmp_overflow_ > 0) {
      if (!lp_overflow_warned_) {
        lp_overflow_warned_ = true;
        CLOG_WARN(&LOG,
                  "SDF LP prune overflow: %d cell(s) exceeded the active-list pool, "
                  "%d cell(s) exceeded the tmp pool; affected cells fall back to "
                  "full-tree tracing (exact, much slower). Enlarge the LP pool "
                  "capacities or lower the grid level",
                  lp_stat_active_overflow_,
                  lp_stat_tmp_overflow_);
      }
      else {
        CLOG_INFO(&LOG,
                  "SDF LP prune overflow: %d cell(s) exceeded the active-list pool, "
                  "%d cell(s) exceeded the tmp pool",
                  lp_stat_active_overflow_,
                  lp_stat_tmp_overflow_);
      }
    }
  }

  void lp_bind_common(gpu::Shader *sh, bool culling)
  {
    lp_bind_ssbo(sh, "lp_prims", lp_prims_ssbo_);
    lp_bind_ssbo(sh, "lp_nodes", lp_nodes_ssbo_);
    lp_bind_ssbo(sh, "lp_binary_ops", lp_binary_ops_ssbo_);
    lp_bind_ssbo(sh, "lp_active_in", culling ? lp_active_nodes_ssbo_[lp_final_idx_] :
                                              lp_active_init_nodes_ssbo_);
    lp_bind_ssbo(sh, "lp_active_init", lp_active_init_nodes_ssbo_);
    lp_bind_ssbo(sh, "lp_cell_meta", lp_cell_meta_ssbo_[lp_final_idx_]);
    lp_bind_ssbo(sh, "objects", object_ssbo_);
    lp_bind_ssbo(sh, "sdf_modifiers", modifier_ssbo_);
    lp_bind_ssbo(sh, "polygon_points", polygon_ssbo_);
    lp_bind_ssbo(sh, "mesh_data_buf", mesh_data_ssbo_);
    lp_bind_ssbo(sh, "bake_dist", bake_dist_ssbo_);

    GPU_shader_uniform_3fv(sh, "aabb_min", lp_aabb_min_);
    GPU_shader_uniform_3fv(sh, "aabb_max", lp_aabb_max_);
    GPU_shader_uniform_1i(sh, "grid_size", culling ? (1 << lp_grid_level_) : 1);
    GPU_shader_uniform_1i(sh, "total_num_nodes", int(lp_nodes_.size()));
    GPU_shader_uniform_1i(sh, "culling_enabled", culling ? 1 : 0);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);
  }

  void draw_lp_march()
  {
    gpu::Shader *sh = lp_march_sh();
    if (sh == nullptr) {
      return;
    }

    lp_ensure_grid_buffers(lp_enable_pruning_ ? lp_grid_level_ : 2);
    const bool culling = lp_enable_pruning_ && lp_grid_valid_ && !lp_nodes_.is_empty();

    GPU_shader_bind(sh);

    lp_bind_common(sh, culling);
    GPU_shader_uniform_1i(sh, "max_steps", sdf_max_steps_);
    GPU_shader_uniform_1f(sh, "ray_epsilon", sdf_ray_epsilon_);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));

    const int dispatch_x = (render_size_.x + 7) / 8;
    const int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_shader_unbind();
  }

  void draw_lp_debug()
  {
    gpu::Shader *sh = lp_debug_sh();
    if (sh == nullptr) {
      return;
    }

    const bool culling = lp_enable_pruning_ && lp_grid_valid_ && !lp_nodes_.is_empty();

    GPU_shader_bind(sh);

    lp_bind_ssbo(sh, "lp_cell_meta", lp_cell_meta_ssbo_[lp_final_idx_]);

    GPU_shader_uniform_3fv(sh, "aabb_min", lp_aabb_min_);
    GPU_shader_uniform_3fv(sh, "aabb_max", lp_aabb_max_);
    GPU_shader_uniform_1i(sh, "grid_size", culling ? (1 << lp_grid_level_) : 1);
    GPU_shader_uniform_1i(sh, "total_num_nodes", int(lp_nodes_.size()));
    GPU_shader_uniform_1i(sh, "culling_enabled", culling ? 1 : 0);
    GPU_shader_uniform_1i(sh, "shading_mode", lp_shading_mode_);
    GPU_shader_uniform_1f(sh, "viz_max", float(lp_colormap_max_));
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));

    const int dispatch_x = (render_size_.x + 7) / 8;
    const int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_normal_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_shader_unbind();
  }

  /** \} */

  /* -------------------------------------------------------------------- */
  /** \name Mesh volume bake manager
   *
   * The manager itself (records, pools, update) lives in SdfInstanceBase,
   * shared with the classic engine; this hook binds the lp_* scene buffers
   * the bake shader declares (dead code there — the bake only walks the mesh
   * BVH).
   * \{ */

  void bind_bake_dead_ssbos(gpu::Shader *sh) override
  {
    lp_bind_ssbo(sh, "lp_prims", lp_prims_ssbo_);
    lp_bind_ssbo(sh, "lp_nodes", lp_nodes_ssbo_);
    lp_bind_ssbo(sh, "lp_binary_ops", lp_binary_ops_ssbo_);
    lp_bind_ssbo(sh, "lp_active_in", lp_active_init_nodes_ssbo_);
  }

  /** \} */

 public:
  ~LpInstance() override
  {
    /* GPU context may already be torn down during shutdown */
    if (!GPU_context_active_get()) {
      return;
    }

    /* Lipschitz pruning buffers */
    if (lp_nodes_ssbo_) {
      GPU_storagebuf_free(lp_nodes_ssbo_);
    }
    if (lp_prims_ssbo_) {
      GPU_storagebuf_free(lp_prims_ssbo_);
    }
    if (lp_binary_ops_ssbo_) {
      GPU_storagebuf_free(lp_binary_ops_ssbo_);
    }
    if (lp_active_init_nodes_ssbo_) {
      GPU_storagebuf_free(lp_active_init_nodes_ssbo_);
    }
    if (lp_active_init_parents_ssbo_) {
      GPU_storagebuf_free(lp_active_init_parents_ssbo_);
    }
    for (int i = 0; i < 2; i++) {
      if (lp_active_nodes_ssbo_[i]) {
        GPU_storagebuf_free(lp_active_nodes_ssbo_[i]);
      }
      if (lp_active_parents_ssbo_[i]) {
        GPU_storagebuf_free(lp_active_parents_ssbo_[i]);
      }
      if (lp_cell_meta_ssbo_[i]) {
        GPU_storagebuf_free(lp_cell_meta_ssbo_[i]);
      }
    }
    if (lp_active_count_ssbo_) {
      GPU_storagebuf_free(lp_active_count_ssbo_);
    }
    if (lp_tmp_ssbo_) {
      GPU_storagebuf_free(lp_tmp_ssbo_);
    }
    if (lp_scratch_ssbo_) {
      GPU_storagebuf_free(lp_scratch_ssbo_);
    }

    /* The bake pools are owned (and freed) by SdfInstanceBase. */
  }
};

DrawEngine *EngineLp::create_instance()
{
  return new LpInstance();
}

}  // namespace blender::draw::sdf
