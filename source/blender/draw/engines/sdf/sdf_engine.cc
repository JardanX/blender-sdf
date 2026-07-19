/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine: analytical sphere tracer — evaluates SDF primitives directly per-pixel.
 */

#include <algorithm>
#include <bit>

#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.h"
#include "BLI_math_solvers.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

#include "BKE_global.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"
#include "BKE_object_types.hh"
#include "BKE_sdf.hh"
#include "BKE_studiolight.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"
#include "DNA_userdef_types.h"
#include "DNA_view3d_enums.h"
#include "DNA_view3d_types.h"

#include "ED_view3d.hh"

#include "IMB_imbuf_types.hh"

#include "MEM_guardedalloc.h"

#include "GPU_batch.hh"
#include "GPU_capabilities.hh"
#include "GPU_framebuffer.hh"
#include "GPU_shader.hh"
#include "GPU_shader_builtin.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_compute.hh"
#include "GPU_uniform_buffer.hh"
#include "GPU_context.hh"

#include "draw_defines.hh"
#include "draw_manager.hh"
#include "draw_view.hh"
#include "draw_view_data.hh"

#include "sdf_cpu_eval.hh"
#include "sdf_private.hh"

#include "sdf_engine.h"
#include "sdf_meshing.hh"

#include "BLI_string.h"

#include <epoxy/gl.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace blender::draw::sdf {

static_assert(SDF_GPU_TYPE_BOX == SDF_TYPE_BOX);
static_assert(SDF_GPU_TYPE_SPHERE == SDF_TYPE_SPHERE);
static_assert(SDF_GPU_TYPE_CYLINDER == SDF_TYPE_CYLINDER);
static_assert(SDF_GPU_TYPE_CONE == SDF_TYPE_CONE);
static_assert(SDF_GPU_TYPE_CAPSULE == SDF_TYPE_CAPSULE);
static_assert(SDF_GPU_TYPE_TORUS == SDF_TYPE_TORUS);
static_assert(SDF_GPU_TYPE_NGON == SDF_TYPE_NGON);
static_assert(SDF_GPU_TYPE_POLYGON == SDF_TYPE_POLYGON);
static_assert(SDF_GPU_TYPE_MESH == SDF_TYPE_MESH);
static_assert(SDF_GPU_TYPE_GROUP == SDF_TYPE_GROUP);

using namespace draw;

/* Static state shared with overlay/selection code */
static int s_object_count = 0;
static gpu::StorageBuf *s_object_ssbo = nullptr;
static gpu::StorageBuf *s_modifier_ssbo = nullptr;
static gpu::StorageBuf *s_polygon_ssbo = nullptr;
static gpu::StorageBuf *s_group_ssbo = nullptr;
static gpu::StorageBuf *s_bvh_ssbo = nullptr;
static gpu::StorageBuf *s_mesh_data_ssbo = nullptr;
static int s_bvh_root = -1;
static int s_group_count = 0;
static Vector<int> s_depsgraph_to_sorted;
static Map<const Object *, int> s_object_to_sorted;
static Vector<const Object *> s_sorted_object_ptrs;
static gpu::Texture *s_depth_tx = nullptr;
static gpu::Texture *s_gbuf_color_tx = nullptr;
static int2 s_render_size = {0, 0};
static int2 s_texture_size = {0, 0};
static const void *s_viewport_key = nullptr;
static const SDFObjectGPU *s_objects_cpu = nullptr;
static int s_objects_cpu_count = 0;
static const SDFPolygonPointGPU *s_polygon_pts_cpu = nullptr;
static int s_polygon_pts_count = 0;
static const SDFModifierGPU *s_modifiers_cpu = nullptr;
static int s_modifier_count = 0;

static void clear_exported_state()
{
  s_object_count = 0;
  s_object_ssbo = nullptr;
  s_modifier_ssbo = nullptr;
  s_polygon_ssbo = nullptr;
  s_group_ssbo = nullptr;
  s_bvh_ssbo = nullptr;
  s_mesh_data_ssbo = nullptr;
  s_bvh_root = -1;
  s_group_count = 0;
  s_depsgraph_to_sorted.clear();
  s_object_to_sorted.clear();
  s_sorted_object_ptrs.clear();
  s_depth_tx = nullptr;
  s_gbuf_color_tx = nullptr;
  s_render_size = {0, 0};
  s_texture_size = {0, 0};
  s_viewport_key = nullptr;
  s_objects_cpu = nullptr;
  s_objects_cpu_count = 0;
  s_polygon_pts_cpu = nullptr;
  s_polygon_pts_count = 0;
  s_modifiers_cpu = nullptr;
  s_modifier_count = 0;
}

/* Frame profiling */

static constexpr int SDF_PROFILE_MAX_PASSES = 16;
static constexpr int SDF_PROFILE_MAX_OBJECTS = 1024;
static constexpr int SDF_PROFILE_MAX_MODS_PER_OBJ = 32;

struct SdfProfileModInfo {
  int type;
  int flags;
};

struct SdfProfileObjectInfo {
  char name[128];
  int sdf_type;
  int blend_type;
  int csg_operation;
  float blend;
  float bevel;
  float size[3];
  int modifier_count;
  SdfProfileModInfo modifiers[SDF_PROFILE_MAX_MODS_PER_OBJ];
};

struct SdfProfilePassTiming {
  char name[64];
  double time_ns;
};

/* Global trace diagnostics — layout matches GPU SSBO (16 uints) */
struct SdfTraceStats {
  uint32_t total_rays;
  uint32_t total_hits;
  uint32_t total_steps;
  uint32_t total_empty_steps;
  uint32_t total_evals;
  uint32_t total_aabb_skips;
  uint32_t total_sor_failures;
  uint32_t max_steps_any_ray;
  uint32_t _pad[8];
};

struct SdfProfileResult {
  bool valid;
  int render_width;
  int render_height;
  float resolution_scale;
  int object_count;
  int group_count;
  int modifier_total;
  int bvh_node_count;
  int pass_count;
  SdfProfilePassTiming passes[SDF_PROFILE_MAX_PASSES];
  double total_ns;
  int profiled_object_count;
  SdfProfileObjectInfo objects[SDF_PROFILE_MAX_OBJECTS];
  uint32_t eval_counts[SDF_PROFILE_MAX_OBJECTS];
  SdfTraceStats trace_stats;
};

static bool s_profile_pending = false;
static SdfProfileResult s_profile_result = {};

/* GL timestamp query profiler — nanosecond GPU-clock precision */
static constexpr int PROF_MAX_SLOTS = 16;

struct SdfFrameProfiler {
  bool active = false;
  int slot_count = 0;
  char names[PROF_MAX_SLOTS][64];
  GLuint queries[PROF_MAX_SLOTS * 2];

  void begin()
  {
    slot_count = 0;
    active = true;
    glGenQueries(PROF_MAX_SLOTS * 2, queries);
  }

  void mark_start(const char *name)
  {
    if (!active || slot_count >= PROF_MAX_SLOTS) return;
    BLI_strncpy(names[slot_count], name, sizeof(names[slot_count]));
    glQueryCounter(queries[slot_count * 2], GL_TIMESTAMP);
  }

  void mark_end()
  {
    if (!active || slot_count >= PROF_MAX_SLOTS) return;
    glQueryCounter(queries[slot_count * 2 + 1], GL_TIMESTAMP);
    slot_count++;
  }

  void finish(SdfProfileResult &result)
  {
    if (!active) return;
    active = false;

    if (slot_count > 0) {
      GLint ready = 0;
      while (!ready) {
        glGetQueryObjectiv(
            queries[(slot_count - 1) * 2 + 1], GL_QUERY_RESULT_AVAILABLE, &ready);
      }
    }

    int pass_count = 0;
    double total_ns = 0.0;
    for (int i = 0; i < slot_count; i++) {
      GLuint64 t0 = 0, t1 = 0;
      glGetQueryObjectui64v(queries[i * 2], GL_QUERY_RESULT, &t0);
      glGetQueryObjectui64v(queries[i * 2 + 1], GL_QUERY_RESULT, &t1);
      double ns = double(t1 - t0);
      auto &p = result.passes[pass_count++];
      BLI_strncpy(p.name, names[i], sizeof(p.name));
      p.time_ns = ns;
      total_ns += ns;
    }
    result.pass_count = pass_count;
    result.total_ns = total_ns;

    glDeleteQueries(PROF_MAX_SLOTS * 2, queries);
  }
};

static SdfFrameProfiler s_profiler = {};

/* Static shader cache — survives engine instance destruction (mode switches). */
enum ShaderIndex {
  SH_TRACE_COMP = 0,
  SH_TRACE_TILE_COMP,
  SH_AABB_PROJECT_COMP,
  SH_TILE_CULL_COMP,
  SH_CONE_MARCH_COMP,
  SH_COLOR_RESOLVE_COMP,
  SH_NORMAL_COMP,
  SH_SHADE_COMP,
  SH_BLIT,
  SH_FXAA,
  SH_LP_PRUNE_COMP,
  SH_LP_TRACE_COMP,
  SH_COUNT,
};

static constexpr const char *s_shader_info_names[SH_COUNT] = {
    "sdf_trace_comp",
    "sdf_trace_tile_comp",
    "sdf_aabb_project_comp",
    "sdf_tile_cull_comp",
    "sdf_cone_march_comp",
    "sdf_color_resolve_comp",
    "sdf_normal_comp",
    "sdf_shade_comp",
    "sdf_blit",
    "sdf_fxaa",
    "sdf_lp_prune_comp",
    "sdf_lp_trace_comp",
};

static gpu::StaticShader s_shaders[SH_COUNT];
static bool s_shaders_initialized = false;

static void sdf_shaders_ensure()
{
  if (!s_shaders_initialized) {
    for (int i = 0; i < SH_COUNT; i++) {
      s_shaders[i] = gpu::StaticShader(s_shader_info_names[i]);
    }
    s_shaders_initialized = true;
  }
  for (int i = 0; i < SH_COUNT; i++) {
    s_shaders[i].ensure_compile_async();
  }
}

static gpu::Shader *sdf_shader_get(int index)
{
  return s_shaders[index].get();
}

void sdf_shaders_free()
{
  for (int i = 0; i < SH_COUNT; i++) {
    s_shaders[i] = {};
  }
  s_shaders_initialized = false;
}

class Instance : public DrawEngine {
 private:
  struct MeshOffsets {
    int vertex_start;
    int triangle_start;
    int triangle_count;
    int bvh_start;
    int vertex_count;
    int bvh_count;
  };

  Vector<SDFObjectGPU> objects_;
  Vector<Object *> object_ptrs_;
  Vector<Object *> group_empties_;

  float3 scene_min_ = float3(1e30f);
  float3 scene_max_ = float3(-1e30f);

  bool fxaa_enabled_ = true;
  float max_blend_ = 0.0f;
  float max_shell_distance_ = 0.0f;
  float step_factor_ = 0.85f;
  bool needs_upload_ = true;

  gpu::Shader *trace_comp_sh() { return sdf_shader_get(SH_TRACE_COMP); }
  gpu::Shader *trace_tile_sh() { return sdf_shader_get(SH_TRACE_TILE_COMP); }
  gpu::Shader *aabb_project_sh() { return sdf_shader_get(SH_AABB_PROJECT_COMP); }
  gpu::Shader *tile_cull_sh() { return sdf_shader_get(SH_TILE_CULL_COMP); }
  gpu::Shader *cone_march_sh() { return sdf_shader_get(SH_CONE_MARCH_COMP); }
  gpu::Shader *color_resolve_sh() { return sdf_shader_get(SH_COLOR_RESOLVE_COMP); }
  gpu::Shader *normal_comp_sh() { return sdf_shader_get(SH_NORMAL_COMP); }
  gpu::Shader *shade_comp_sh() { return sdf_shader_get(SH_SHADE_COMP); }
  gpu::Shader *blit_sh() { return sdf_shader_get(SH_BLIT); }
  gpu::Shader *fxaa_sh() { return sdf_shader_get(SH_FXAA); }
  gpu::Shader *lp_prune_sh() { return sdf_shader_get(SH_LP_PRUNE_COMP); }
  gpu::Shader *lp_trace_sh() { return sdf_shader_get(SH_LP_TRACE_COMP); }

  gpu::Texture *comp_color_tx_ = nullptr;
  gpu::Texture *comp_depth_tx_ = nullptr;
  gpu::Texture *gbuf_pos_tx_ = nullptr;
  gpu::Texture *gbuf_color_tx_ = nullptr;
  gpu::Texture *gbuf_normal_tx_ = nullptr;
  gpu::Texture *march_color_tx_ = nullptr;
  gpu::FrameBuffer *march_fb_ = nullptr;
  int2 render_size_ = int2(0);
  int2 prev_render_size_ = int2(0);
  int2 texture_size_ = int2(0);
  int2 viewport_size_ = int2(0);
  int2 fxaa_size_ = int2(0);
  float resolution_scale_ = 1.0f;
  bool adaptive_resolution_ = false;
  /* When true, marcher is relaxed during adaptive low-res. When false, UI
   * marcher values are used at all times even during low-res navigation. */
  bool adaptive_precision_ = true;
  bool scene_changed_ = false;
  bool view_changed_ = false;
  bool mesh_changed_ = false;
  bool mesh_data_changed_ = false;
  int scroll_cooldown_ = 0;
  int idle_frames_ = 0;
  bool compute_valid_ = false;
  uint64_t prev_data_hash_ = 0;
  uint64_t prev_mesh_hash_ = 0;
  uint64_t prev_mesh_data_hash_ = 0;
  uint64_t prev_shading_hash_ = 0;
  Vector<float4x4> mesh_transforms_;
  float4x4 prev_viewmat_ = float4x4::identity();
  float4x4 prev_winmat_ = float4x4::identity();

  gpu::StorageBuf *object_ssbo_ = nullptr;
  int object_ssbo_count_ = 0;

  Vector<SDFObjectAABB> object_aabbs_;
  gpu::StorageBuf *object_aabb_ssbo_ = nullptr;
  int object_aabb_ssbo_count_ = 0;

  Vector<SDFModifierGPU> modifiers_;
  gpu::StorageBuf *modifier_ssbo_ = nullptr;
  int modifier_ssbo_count_ = 0;

  Vector<SDFPolygonPointGPU> polygon_points_;
  gpu::StorageBuf *polygon_ssbo_ = nullptr;
  int polygon_ssbo_count_ = 0;

  Vector<uint4> mesh_gpu_data_;
  Map<const void *, MeshOffsets> mesh_offsets_;
  Vector<std::shared_ptr<const SDFMeshPayload>> live_mesh_payloads_;
  gpu::StorageBuf *mesh_data_ssbo_ = nullptr;
  int mesh_data_ssbo_count_ = 0;

  Vector<SDFGroupGPU> groups_gpu_;
  gpu::StorageBuf *group_ssbo_ = nullptr;
  int group_ssbo_count_ = 0;

  SdfAabbTree bvh_tree_;
  Vector<const Object *> bvh_object_ptrs_;
  Vector<int> bvh_proxies_;
  gpu::StorageBuf *bvh_nodes_ssbo_ = nullptr;
  int bvh_nodes_ssbo_count_ = 0;

  gpu::StorageBuf *cone_hit_ssbo_ = nullptr;
  gpu::StorageBuf *tile_far_hint_ssbo_ = nullptr;
  gpu::StorageBuf *screen_aabbs_ssbo_ = nullptr;
  int screen_aabbs_ssbo_count_ = 0;

  gpu::StorageBuf *tile_prim_counts_ssbo_ = nullptr;
  int tile_prim_counts_ssbo_tiles_ = 0;
  gpu::StorageBuf *tile_prim_lists_ssbo_ = nullptr;

  gpu::StorageBuf *prof_eval_ssbo_ = nullptr;
  gpu::StorageBuf *prof_stats_ssbo_ = nullptr;
  int prof_eval_ssbo_count_ = 0;

  gpu::Batch *fullscreen_batch_ = nullptr;

  const DRWContext *draw_ctx_ = nullptr;

  gpu::Texture *matcap_tx_ = nullptr;
  std::string current_matcap_;
  int lighting_type_ = V3D_LIGHTING_STUDIO;
  int use_specular_ = 0;
  int use_matcap_flip_ = 0;
  int use_bvh_ = 1;
  int debug_bvh_views_ = 0;
  int debug_fd_normals_ = 0;
  int use_cone_trace_ = 0;
  int sdf_max_steps_ = 128;
  float sdf_ray_epsilon_ = 0.005f;
  float sdf_over_relaxation_ = 1.3f;
  float sdf_cone_aperture_ = 0.5f;
  int sdf_cone_steps_ = 32;

  /* UI-cached marcher values; the dispatched sdf_max_steps_/sdf_ray_epsilon_
   * are dynamically relaxed while adaptive resolution is rendering at
   * quarter scale (accuracy doesn't matter at that pixel size). */
  int ui_sdf_max_steps_ = 128;
  float ui_sdf_ray_epsilon_ = 0.005f;
  bool adaptive_lowres_active_ = false;

  float4 frustum_planes_[6];
  bool frustum_valid_ = false;
  bool use_frustum_cull_ = true;

  float4 studio_light_dir_[4] = {};
  float4 studio_light_col_[4] = {};
  float4 studio_light_spec_[4] = {};
  float3 studio_ambient_ = float3(0.0f);
  gpu::UniformBuf *shading_ubo_ = nullptr;

  /* ---- Lipschitz pruning engine state ---- */
  int engine_mode_ = 0; /* 0 = classic, 1 = Lipschitz pruning */
  bool lp_enable_pruning_ = true;
  bool lp_recompute_pruning_ = true;
  int lp_shading_mode_ = 0;
  int lp_grid_level_ = 6;
  int lp_colormap_max_ = 25;
  bool lp_aabb_auto_ = true;
  float3 lp_aabb_min_user_ = float3(-1.0f);
  float3 lp_aabb_max_user_ = float3(1.0f);

  /* CPU CSG tree (post-order serialized, rebuilt in end_sync). */
  Vector<SDFLpNode> lp_nodes_;
  Vector<SDFLpPrimitive> lp_prims_;
  Vector<uint32_t> lp_binary_ops_;
  Vector<uint32_t> lp_parents_init_;
  Vector<uint32_t> lp_active_init_;

  gpu::StorageBuf *lp_nodes_ssbo_ = nullptr;
  gpu::StorageBuf *lp_prims_ssbo_ = nullptr;
  gpu::StorageBuf *lp_binary_ops_ssbo_ = nullptr;
  gpu::StorageBuf *lp_parents_init_ssbo_ = nullptr;
  gpu::StorageBuf *lp_active_init_ssbo_ = nullptr;

  /* Grid buffers (ping-pong between hierarchy levels). */
  gpu::StorageBuf *lp_active_ssbo_[2] = {nullptr, nullptr};
  gpu::StorageBuf *lp_parents_ssbo_[2] = {nullptr, nullptr};
  gpu::StorageBuf *lp_num_active_ssbo_[2] = {nullptr, nullptr};
  gpu::StorageBuf *lp_cell_offset_ssbo_[2] = {nullptr, nullptr};
  gpu::StorageBuf *lp_cell_value_ssbo_[2] = {nullptr, nullptr};
  gpu::StorageBuf *lp_active_count_ssbo_ = nullptr;
  gpu::StorageBuf *lp_tmp_ssbo_ = nullptr;
  gpu::StorageBuf *lp_scratch_ssbo_ = nullptr;
  gpu::StorageBuf *lp_tmp_count_ssbo_ = nullptr;

  int lp_grid_size_ = 0; /* Allocated grid resolution per axis (0 = unallocated). */
  int64_t lp_active_capacity_ = 0;
  int64_t lp_tmp_capacity_ = 0;
  bool lp_grid_valid_ = false; /* Pruning results match the current tree/grid/AABB. */
  bool lp_grid_dirty_ = true;
  int lp_grid_level_built_ = 0; /* Grid level the current results were built with. */
  int lp_final_idx_ = 0; /* Ping-pong slot holding the final level results. */
  float3 lp_aabb_min_ = float3(0.0f);
  float3 lp_aabb_max_ = float3(0.0f);

 public:
  Instance() {}

  blender::StringRefNull name_get() final
  {
    return "SDF";
  }

  void init() final
  {
    draw_ctx_ = DRW_context_get();

    sync_sdf_settings();

    sdf_shaders_ensure();
  }

  void begin_sync() final
  {
    objects_.clear();
    object_ptrs_.clear();
    group_empties_.clear();
    modifiers_.clear();
    polygon_points_.clear();
    mesh_gpu_data_.clear();
    mesh_offsets_.clear();
    live_mesh_payloads_.clear();
    groups_gpu_.clear();
    s_depsgraph_to_sorted.clear();
    s_object_to_sorted.clear();
    s_sorted_object_ptrs.clear();
    mesh_transforms_.clear();
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
    max_blend_ = 0.0f;
    max_shell_distance_ = 0.0f;
    step_factor_ = 0.85f;
    frustum_valid_ = false;

  }

  void object_sync(ObjectRef &ob_ref, Manager & /*manager*/) final
  {
    Object *ob = ob_ref.object;

    SDF live_sdf = {};
    const SDF *sdf_data = nullptr;
    const void *mesh_payload_key = nullptr;
    if (ob->type == OB_MESH) {
      SDFMeshRuntimeSnapshot snapshot;
      if (!BKE_sdf_mesh_runtime_snapshot(*ob, snapshot)) {
        return;
      }
      mesh_transforms_.append(ob->object_to_world());
      live_mesh_payloads_.append(snapshot.payload);
      const SDFMeshPayload &payload = *snapshot.payload;
      live_sdf.sdf_type = SDF_TYPE_MESH;
      live_sdf.blend = ob->sdf_blend;
      live_sdf.blend_type = ob->sdf_blend_type;
      live_sdf.csg_operation = ob->sdf_csg_operation;
      live_sdf.clearance = ob->sdf_clearance;
      live_sdf.color_blend = ob->sdf_color_blend;
      live_sdf.color_blend_type = ob->sdf_color_blend_type;
      live_sdf.shell_distance = ob->sdf_shell_distance;
      live_sdf.shell_mode = ob->sdf_shell_mode;
      live_sdf.shell_op = ob->sdf_shell_op;
      live_sdf.shell_blend_top = ob->sdf_shell_blend_top;
      live_sdf.shell_blend_bottom = ob->sdf_shell_blend_bottom;
      live_sdf.chamfer_k2 = ob->sdf_chamfer_k2;
      live_sdf.chamfer_k3 = ob->sdf_chamfer_k3;
      live_sdf.chamfer_k4 = ob->sdf_chamfer_k4;
      live_sdf.chamfer_k5 = ob->sdf_chamfer_k5;
      live_sdf.flip_blend = ob->sdf_flip_blend;
      live_sdf.flip_blend_end = ob->sdf_flip_blend_end;
      live_sdf.mesh_vertices = payload.vertices;
      live_sdf.mesh_triangles = payload.triangles;
      live_sdf.mesh_bvh_nodes = payload.bvh_nodes;
      live_sdf.mesh_vertex_count = payload.vertex_count;
      live_sdf.mesh_triangle_count = payload.triangle_count;
      live_sdf.mesh_bvh_node_count = payload.bvh_node_count;
      live_sdf.mesh_flags = payload.flags | SDF_MESH_FLAG_CORNER_NORMALS;
      live_sdf.mesh_normal_mode = SDF_MESH_NORMAL_SMOOTH;
      live_sdf.mesh_data_version = int(payload.revision);
      copy_v3_v3(live_sdf.mesh_bounds_min, payload.bounds_min);
      copy_v3_v3(live_sdf.mesh_bounds_max, payload.bounds_max);
      copy_v4_v4(live_sdf.color, ob->color);
      live_sdf.sdf_index = ob->sdf_index;
      sdf_data = &live_sdf;
      mesh_payload_key = snapshot.payload.get();
    }
    else if (ob->type == OB_SDF) {
      sdf_data = id_cast<const SDF *>(ob->data);
      mesh_payload_key = sdf_data;
    }
    else {
      return;
    }

    if (sdf_data == nullptr) {
      return;
    }

    if (sdf_data->sdf_type == SDF_TYPE_MESH &&
        (sdf_data->mesh_vertex_count <= 0 || sdf_data->mesh_triangle_count <= 0 ||
         sdf_data->mesh_bvh_node_count <= 0 || sdf_data->mesh_vertices == nullptr ||
         sdf_data->mesh_triangles == nullptr || sdf_data->mesh_bvh_nodes == nullptr))
    {
      return;
    }

    /* Group empties are containers, not renderable primitives */
    if (sdf_data->sdf_type == SDF_TYPE_GROUP) {
      group_empties_.append(ob);
      return;
    }

    if (!frustum_valid_) {
      const View &view = View::default_get();
      float4x4 vp = view.winmat() * view.viewmat();
      for (int i = 0; i < 4; i++) {
        frustum_planes_[0][i] = vp[i][3] + vp[i][0];
        frustum_planes_[1][i] = vp[i][3] - vp[i][0];
        frustum_planes_[2][i] = vp[i][3] + vp[i][1];
        frustum_planes_[3][i] = vp[i][3] - vp[i][1];
        frustum_planes_[4][i] = vp[i][3] + vp[i][2];
        frustum_planes_[5][i] = vp[i][3] - vp[i][2];
      }
      for (int p = 0; p < 6; p++) {
        frustum_planes_[p] /= math::length(float3(frustum_planes_[p]));
      }
      frustum_valid_ = true;
    }

    const float4x4 &mat = ob->object_to_world();

    float3 scale;
    scale.x = math::length(float3(mat[0]));
    scale.y = math::length(float3(mat[1]));
    scale.z = math::length(float3(mat[2]));

    float4x4 rot_mat = mat;
    if (scale.x > 0.0f) {
      rot_mat[0] = float4(float3(mat[0]) / scale.x, 0.0f);
    }
    if (scale.y > 0.0f) {
      rot_mat[1] = float4(float3(mat[1]) / scale.y, 0.0f);
    }
    if (scale.z > 0.0f) {
      rot_mat[2] = float4(float3(mat[2]) / scale.z, 0.0f);
    }
    rot_mat[3] = float4(0.0f, 0.0f, 0.0f, 1.0f);

    float4x4 inv_rot = math::invert(rot_mat);
    float mesh_distance_scale = 1.0f;
    if (sdf_data->sdf_type == SDF_TYPE_MESH) {
      float rot_m3[3][3];
      float singular_values[3];
      copy_m3_m4(rot_m3, rot_mat.ptr());
      BLI_svd_m3(rot_m3, nullptr, singular_values, nullptr);
      mesh_distance_scale = std::max(
          std::min(singular_values[0], std::min(singular_values[1], singular_values[2])),
          1e-6f);
    }

    SDFObjectGPU gpu_obj = {};
    gpu_obj.inverse_matrix = inv_rot;
    gpu_obj.position = float4(mat[3].x, mat[3].y, mat[3].z, 0.0f);

    if (sdf_data->sdf_type == SDF_TYPE_MESH) {
      MeshOffsets offsets;
      if (const MeshOffsets *existing = mesh_offsets_.lookup_ptr(mesh_payload_key)) {
        offsets = *existing;
      }
      else {
        const size_t max_ssbo_size = GPU_max_storage_buffer_size();
        const size_t new_record_count = mesh_gpu_data_.size() + sdf_data->mesh_vertex_count +
                                        size_t(sdf_data->mesh_triangle_count) * 3 +
                                        size_t(sdf_data->mesh_bvh_node_count) * 2;
        if (new_record_count * sizeof(uint4) > max_ssbo_size)
        {
          return;
        }

        offsets.vertex_start = int(mesh_gpu_data_.size());
        offsets.triangle_count = sdf_data->mesh_triangle_count;
        offsets.vertex_count = sdf_data->mesh_vertex_count;
        offsets.bvh_count = sdf_data->mesh_bvh_node_count;

        mesh_gpu_data_.reserve(new_record_count);
        for (int i = 0; i < sdf_data->mesh_vertex_count; i++) {
          const SDFMeshVertex &vertex = sdf_data->mesh_vertices[i];
          mesh_gpu_data_.append(uint4(std::bit_cast<uint32_t>(vertex.co[0]),
                                      std::bit_cast<uint32_t>(vertex.co[1]),
                                      std::bit_cast<uint32_t>(vertex.co[2]),
                                      vertex.pseudonormal));
        }
        offsets.triangle_start = int(mesh_gpu_data_.size());
        for (int i = 0; i < sdf_data->mesh_triangle_count; i++) {
          const SDFMeshTriangle &triangle = sdf_data->mesh_triangles[i];
          mesh_gpu_data_.append(uint4(triangle.vertices[0],
                                      triangle.vertices[1],
                                      triangle.vertices[2],
                                      uint32_t(triangle.material_index)));
          mesh_gpu_data_.append(uint4(triangle.corner_normals[0],
                                      triangle.corner_normals[1],
                                      triangle.corner_normals[2],
                                      0));
          mesh_gpu_data_.append(uint4(triangle.edge_normals[0],
                                      triangle.edge_normals[1],
                                      triangle.edge_normals[2],
                                      0));
        }
        offsets.bvh_start = int(mesh_gpu_data_.size());
        for (int i = 0; i < sdf_data->mesh_bvh_node_count; i++) {
          const SDFMeshBVHNode &node = sdf_data->mesh_bvh_nodes[i];
          mesh_gpu_data_.append(uint4(std::bit_cast<uint32_t>(node.bounds_min[0]),
                                      std::bit_cast<uint32_t>(node.bounds_min[1]),
                                      std::bit_cast<uint32_t>(node.bounds_min[2]),
                                      uint32_t(node.child_or_first)));
          mesh_gpu_data_.append(uint4(std::bit_cast<uint32_t>(node.bounds_max[0]),
                                      std::bit_cast<uint32_t>(node.bounds_max[1]),
                                      std::bit_cast<uint32_t>(node.bounds_max[2]),
                                      uint32_t(node.child_or_count)));
        }
        mesh_offsets_.add(mesh_payload_key, offsets);
      }

      gpu_obj.mesh_data = int4(offsets.vertex_start,
                               offsets.triangle_start,
                               offsets.triangle_count,
                               offsets.bvh_start);
      gpu_obj.mesh_settings = int4(sdf_data->mesh_normal_mode,
                                     sdf_data->mesh_flags,
                                     offsets.bvh_count,
                                    sdf_data->mesh_data_version);
      gpu_obj.mesh_bounds_min = float4(float3(sdf_data->mesh_bounds_min), 0.0f);
      gpu_obj.mesh_bounds_max = float4(float3(sdf_data->mesh_bounds_max), 0.0f);
    }

    /* Detect group parent early — needed for bevel accumulation and modifier propagation */
    Object *group_parent_ob = nullptr;
    {
      Object *orig_ob = DEG_get_original(ob);
      if (orig_ob->parent && orig_ob->parent->type == OB_SDF) {
        const SDF *parent_sdf = id_cast<const SDF *>(orig_ob->parent->data);
        if (parent_sdf && parent_sdf->sdf_type == SDF_TYPE_GROUP) {
          group_parent_ob = orig_ob->parent;
        }
      }
    }

    float bevel = 0.0f;
    for (const ModifierData *bmd = static_cast<const ModifierData *>(ob->modifiers.first); bmd; bmd = bmd->next) {
      if ((bmd->mode & eModifierMode_Realtime) && bmd->type == eModifierType_SDFBevel) {
        bevel += reinterpret_cast<const SDFBevelModifierData *>(bmd)->radius;
      }
    }
    /* Also accumulate bevel from group parent */
    if (group_parent_ob) {
      for (const ModifierData *bmd = static_cast<const ModifierData *>(
               group_parent_ob->modifiers.first); bmd; bmd = bmd->next) {
        if ((bmd->mode & eModifierMode_Realtime) && bmd->type == eModifierType_SDFBevel) {
          bevel += reinterpret_cast<const SDFBevelModifierData *>(bmd)->radius;
        }
      }
    }
    /* Object scale is applied as a coordinate transform at eval time (so it
     * stretches/squishes the geometry rather than editing shape params). sdf_size
     * stores BASE (unscaled) dimensions; the AABB below applies per-axis scale. */
    float min_scale = std::max(math::reduce_min(scale), 1e-6f);
    float3 base_size(sdf_data->size[0], sdf_data->size[1], sdf_data->size[2]);
    float3 base_dimensions = base_size * 2.0f;
    if (sdf_data->sdf_type == SDF_TYPE_MESH) {
      base_dimensions = float3(sdf_data->mesh_bounds_max) -
                        float3(sdf_data->mesh_bounds_min);
    }
    /* Bevel radii are authored in base units; eval converts to world via min_scale. */
    float min_dim = math::reduce_min(base_size);
    /* Cone top radius (size.z) may be 0 for a sharp apex — exclude it from the
     * auto-bevel min dimension so a sharp cone keeps its small rounding. */
    if (sdf_data->sdf_type == SDF_TYPE_CONE) {
      min_dim = std::min(base_size.x, base_size.y);
    }
    if (sdf_data->sdf_type == SDF_TYPE_MESH) {
      bevel = 0.0f;
    }
    float eff_bevel = (sdf_data->sdf_type == SDF_TYPE_MESH) ?
                          0.0f :
                          std::max(bevel, std::min(0.005f, min_dim * 0.5f));
    /* Store pre-bevel-subtracted BASE size in xyz, effective bevel in w */
    float3 net_size = math::max(base_size - float3(eff_bevel), float3(0.001f));
    /* Cone top radius may shrink to 0 (sharp apex), not clamped to 0.001. */
    if (sdf_data->sdf_type == SDF_TYPE_CONE) {
      net_size.z = std::max(base_size.z - eff_bevel, 0.0f);
    }
    gpu_obj.sdf_size = float4(net_size.x, net_size.y, net_size.z, eff_bevel);
    gpu_obj.obj_scale = float4(std::max(scale.x, 1e-6f),
                               std::max(scale.y, 1e-6f),
                               std::max(scale.z, 1e-6f),
                               sdf_data->sdf_type == SDF_TYPE_MESH ? mesh_distance_scale :
                                                                    min_scale);
    gpu_obj.bevel = bevel;

    gpu_obj.blend = sdf_data->blend;
    gpu_obj.clearance = sdf_data->clearance;
    gpu_obj.color_blend = sdf_data->color_blend;
    gpu_obj.sdf_type = sdf_data->sdf_type;
    gpu_obj.blend_type = sdf_data->blend_type;
    gpu_obj.color_blend_type = sdf_data->color_blend_type;
    gpu_obj.csg_operation = sdf_data->csg_operation;
    gpu_obj.shell_distance = sdf_data->shell_distance;
    gpu_obj.shell_mode = sdf_data->shell_mode;
    gpu_obj.shell_op = sdf_data->shell_op;
    gpu_obj.shell_blend_top = sdf_data->shell_blend_top;
    gpu_obj.shell_blend_bottom = sdf_data->shell_blend_bottom;
    gpu_obj.chamfer_k2 = sdf_data->chamfer_k2;
    gpu_obj.chamfer_k3 = sdf_data->chamfer_k3;
    gpu_obj.chamfer_k4 = sdf_data->chamfer_k4;
    gpu_obj.chamfer_k5 = sdf_data->chamfer_k5;
    gpu_obj.flip_blend = sdf_data->flip_blend;
    gpu_obj.flip_blend_end = sdf_data->flip_blend_end;

    /* Early AABB + frustum cull (before expensive polygon/modifier work) */
    {
      float shell_expand = (sdf_data->csg_operation == SDF_CSG_SHELL) ?
                               fabsf(sdf_data->shell_distance) :
                               0.0f;
      float blend_pad = (sdf_data->blend_type != 0) ? sdf_data->blend : 0.0f;
      float aabb_pad = std::max(blend_pad, sdf_data->color_blend) + shell_expand;
      float3 local_extent;
      /* Base (unscaled) extent with correct geometric-axis assignment; per-axis
       * object scale is applied afterward so it stretches the geometry. */
      float3 sz = base_size;
      switch (sdf_data->sdf_type) {
        case SDF_TYPE_MESH:
          local_extent = math::max(math::abs(float3(sdf_data->mesh_bounds_min)),
                                   math::abs(float3(sdf_data->mesh_bounds_max)));
          break;
        case SDF_TYPE_SPHERE:
          local_extent = sz;
          break;
        case SDF_TYPE_CAPSULE: {
          float r = sz.x;
          float h = sz.y;
          local_extent = float3(r, r, h + r);
          break;
        }
        case SDF_TYPE_CYLINDER:
          local_extent = sz;
          break;
        case SDF_TYPE_CONE: {
          float r = std::max(sz.x, sz.z);
          float h = sz.y;
          local_extent = float3(r, r, h);
          break;
        }
        case SDF_TYPE_TORUS: {
          float outer = sz.x + sz.y;
          local_extent = float3(outer, outer, sz.y);
          break;
        }
        case SDF_TYPE_NGON: {
          float r = sz.x;
          local_extent = float3(r, r, sz.z);
          break;
        }
        case SDF_TYPE_POLYGON: {
          float max_x = 0.0f, max_y = 0.0f;
          for (const SDFPolygonPoint *pt = static_cast<const SDFPolygonPoint *>(sdf_data->polygon_points.first); pt; pt = pt->next) {
            max_x = math::max(max_x, fabsf(pt->co[0]));
            max_y = math::max(max_y, fabsf(pt->co[1]));
          }
          float line_pad = sdf_data->polygon_is_line ?
                               sdf_data->polygon_line_thickness * 0.5f :
                               0.0f;
          local_extent = float3(max_x + line_pad, max_y + line_pad, sz.z);
          break;
        }
        default:
          local_extent = sz;
          break;
      }
      local_extent = local_extent * scale + float3(aabb_pad);

      for (int mi = gpu_obj.modifier_start;
           mi < gpu_obj.modifier_start + gpu_obj.modifier_count; mi++)
      {
        const SDFModifierGPU &mod = modifiers_[mi];
        int mtype = mod.header.x;
        int mflags = mod.header.y;
        switch (mtype) {
          case SDF_MOD_MIRROR: {
            float offset = fabsf(mod.params.x);
            float3 local_org = float3(mod.params.y, mod.params.z, mod.params.w);
            if ((mflags & SDF_MOD_MIRROR_X) != 0) {
              float3 N = float3(inv_rot[0]);
              float extent_along = math::dot(local_extent, math::abs(N));
              float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, extent_along);
              local_extent += math::abs(N) * disp;
            }
            if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
              float3 N = float3(inv_rot[1]);
              float extent_along = math::dot(local_extent, math::abs(N));
              float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, extent_along);
              local_extent += math::abs(N) * disp;
            }
            if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
              float3 N = float3(inv_rot[2]);
              float extent_along = math::dot(local_extent, math::abs(N));
              float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, extent_along);
              local_extent += math::abs(N) * disp;
            }
            break;
          }
          case SDF_MOD_ELONGATE:
            local_extent += float3(mod.params.x, mod.params.y, mod.params.z);
            break;
          case SDF_MOD_SOLIDIFY:
          case SDF_MOD_ONION:
            break;
          case SDF_MOD_DISPLACE:
            local_extent += float3(math::abs(mod.params.x));
            break;
          case SDF_MOD_ROUND:
          case SDF_MOD_BEVEL:
            local_extent = math::max(local_extent + float3(mod.params.x), float3(0.0f));
            break;
          case SDF_MOD_TWIST: {
            int axis = int(mod.params.y);
            if (axis == 1) {
              float xz = math::sqrt(local_extent.x * local_extent.x +
                                    local_extent.z * local_extent.z);
              local_extent.x = xz;
              local_extent.z = xz;
            }
            else if (axis == 2) {
              float yz = math::sqrt(local_extent.y * local_extent.y +
                                    local_extent.z * local_extent.z);
              local_extent.y = yz;
              local_extent.z = yz;
            }
            else {
              float xy = math::sqrt(local_extent.x * local_extent.x +
                                    local_extent.y * local_extent.y);
              local_extent.x = xy;
              local_extent.y = xy;
            }
            break;
          }
          case SDF_MOD_BEND: {
            float k = mod.params.x;
            int axis = int(mod.params.y);
            float3 origin = float3(mod.params.z, mod.params.w, mod.params2.x);
            if (fabsf(k) > 0.0001f) {
              float3 new_ext = float3(0.0f);
              for (int c = 0; c < 8; c++) {
                float3 pt = float3((c & 1) ? local_extent.x : -local_extent.x,
                                   (c & 2) ? local_extent.y : -local_extent.y,
                                   (c & 4) ? local_extent.z : -local_extent.z);
                pt -= origin;
                float drive, curve, free_val;
                if (axis == 1) { drive = pt.y; curve = pt.z; free_val = pt.x; }
                else if (axis == 2) { drive = pt.z; curve = pt.x; free_val = pt.y; }
                else { drive = pt.x; curve = pt.y; free_val = pt.z; }
                float a = k * drive;
                float nd = cosf(a) * drive - sinf(a) * curve;
                float nc = sinf(a) * drive + cosf(a) * curve;
                float3 bent;
                if (axis == 1) { bent = float3(free_val, nd, nc); }
                else if (axis == 2) { bent = float3(nc, free_val, nd); }
                else { bent = float3(nd, nc, free_val); }
                bent += origin;
                new_ext = math::max(new_ext, math::abs(bent));
              }
              local_extent = new_ext;
            }
            break;
          }
          case SDF_MOD_ARRAY: {
            float count = mod.params.x;
            if (count > 0.5f) {
              if (mflags == SDF_MOD_ARRAY_LINEAR) {
                float3 arr_offset = float3(mod.params.y, mod.params.z, mod.params.w);
                local_extent += math::abs(arr_offset) * (count - 1.0f);
              }
              else if (mflags == SDF_MOD_ARRAY_RADIAL) {
                float radius = mod.params.y;
                local_extent.x += radius;
                local_extent.y += radius;
                float3 rot_off(mod.params2.y, mod.params2.z, mod.params2.w);
                if (math::length(rot_off) > 0.0001f) {
                  float r = math::length(local_extent);
                  local_extent = float3(r);
                }
              }
            }
            break;
          }
          default:
            break;
        }
      }

      /* Expand early AABB for array instances (scan modifier stack directly) */
      for (const ModifierData *amd = static_cast<const ModifierData *>(ob->modifiers.first);
           amd; amd = amd->next)
      {
        if (amd->type == eModifierType_SDFArray && (amd->mode & eModifierMode_Realtime)) {
          const auto &am = *reinterpret_cast<const SDFArrayModifierData *>(amd);
          if (am.count > 1) {
            if (am.array_type == MOD_SDF_ARRAY_LINEAR) {
              float3 arr_off(0.0f);
              if (am.use_relative_offset) {
                arr_off += float3(am.relative_offset[0], am.relative_offset[1],
                                  am.relative_offset[2]) * base_dimensions;
              }
              if (am.use_constant_offset) {
                arr_off += float3(am.constant_offset[0], am.constant_offset[1],
                                  am.constant_offset[2]);
              }
              local_extent += math::abs(arr_off) * float(am.count - 1);
            }
            else if (am.array_type == MOD_SDF_ARRAY_RADIAL) {
              local_extent.x += am.array_radius;
              local_extent.y += am.array_radius;
              float r_ext = math::length(local_extent);
              local_extent = float3(r_ext);
            }
            if (am.use_object_offset && am.offset_object != nullptr) {
              float r_ext = math::length(local_extent) * 1.5f;
              local_extent = float3(r_ext);
            }
          }
          break;
        }
      }

      float3 world_min = float3(1e30f);
      float3 world_max = float3(-1e30f);
      for (int corner = 0; corner < 8; corner++) {
        float3 lc = float3((corner & 1) ? local_extent.x : -local_extent.x,
                           (corner & 2) ? local_extent.y : -local_extent.y,
                           (corner & 4) ? local_extent.z : -local_extent.z);
        float3 wc = float3(rot_mat * float4(lc, 0.0f)) +
                     float3(mat[3].x, mat[3].y, mat[3].z);
        world_min = math::min(world_min, wc);
        world_max = math::max(world_max, wc);
      }

      gpu_obj.bbox_min = float4(world_min, 0.0f);
      gpu_obj.bbox_max = float4(world_max, 0.0f);
      gpu_obj.orig_bbox_min = gpu_obj.bbox_min;
      gpu_obj.orig_bbox_max = gpu_obj.bbox_max;

      float sphere_radius = math::length(local_extent);
      float3 obj_center = float3(mat[3].x, mat[3].y, mat[3].z);
      scene_min_ = math::min(scene_min_, obj_center - float3(sphere_radius));
      scene_max_ = math::max(scene_max_, obj_center + float3(sphere_radius));
    }

    gpu_obj.group_id = -1;
    gpu_obj.original_index = sdf_data->sdf_index;

    gpu_obj.color = float4(
        sdf_data->color[0], sdf_data->color[1], sdf_data->color[2], sdf_data->color[3]);

    /* Shape-specific data */
    if (sdf_data->sdf_type == SDF_TYPE_NGON) {
      float ngon_taper = sdf_data->ngon_taper;
      gpu_obj.box_corners = float4(sdf_data->ngon_corner, sdf_data->ngon_star, 0.0f, 0.0f);
      gpu_obj.box_edges = float4(sdf_data->ngon_edge_top,
                                 sdf_data->ngon_edge_bottom,
                                 math::max(ngon_taper, 0.0f),
                                 math::max(-ngon_taper, 0.0f));
      gpu_obj.box_modes = int4(0, sdf_data->ngon_edge_mode, sdf_data->ngon_sides, 0);
    }
    else if (sdf_data->sdf_type == SDF_TYPE_POLYGON) {
      float poly_taper = sdf_data->polygon_taper;
      gpu_obj.polygon_point_start = int(polygon_points_.size());
      gpu_obj.polygon_point_count = 0;

      /* Per-axis XY scale so S+X / S+Y stretch the polygon */
      float corner_scale = math::min(scale.x, scale.y);
      Vector<float2> pts;
      Vector<float> crn;
      for (const SDFPolygonPoint *pt = static_cast<const SDFPolygonPoint *>(sdf_data->polygon_points.first); pt; pt = pt->next) {
        pts.append(float2(pt->co[0] * scale.x, pt->co[1] * scale.y));
        crn.append(pt->corner * corner_scale);
      }
      int pc = int(pts.size());

      /* Line mode: expand polyline into thick polygon */
      if (sdf_data->polygon_is_line && pc >= 2) {
        float half_t = sdf_data->polygon_line_thickness * 0.5f *
                       math::min(scale.x, scale.y);

        Vector<float2> left_side(pc);
        Vector<float2> right_side(pc);
        Vector<float> left_crn(pc);
        Vector<float> right_crn(pc);

        for (int i = 0; i < pc; i++) {
          float2 n_prev(0.0f), n_next(0.0f);
          if (i > 0) {
            float2 d = math::normalize(pts[i] - pts[i - 1]);
            n_prev = float2(-d.y, d.x);
          }
          if (i < pc - 1) {
            float2 d = math::normalize(pts[i + 1] - pts[i]);
            n_next = float2(-d.y, d.x);
          }

          if (i == 0) {
            left_side[i] = pts[i] + n_next * half_t;
            right_side[i] = pts[i] - n_next * half_t;
            left_crn[i] = 0.0f;
            right_crn[i] = 0.0f;
          }
          else if (i == pc - 1) {
            left_side[i] = pts[i] + n_prev * half_t;
            right_side[i] = pts[i] - n_prev * half_t;
            left_crn[i] = 0.0f;
            right_crn[i] = 0.0f;
          }
          else {
            /* Miter at interior vertex */
            float2 miter = math::normalize(n_prev + n_next);
            float dot_val = math::dot(miter, n_prev);
            if (fabsf(dot_val) < 0.1f) {
              dot_val = (dot_val >= 0.0f) ? 0.1f : -0.1f;
            }
            float miter_len = half_t / dot_val;
            float max_miter = half_t * 3.0f;
            miter_len = math::clamp(miter_len, -max_miter, max_miter);

            left_side[i] = pts[i] + miter * miter_len;
            right_side[i] = pts[i] - miter * miter_len;

            /* Determine which side is outer/inner based on turn direction.
               Left normal = (-d.y, d.x). For a left turn (cross > 0),
               the outside of the curve is the RIGHT side (pts - normal). */
            float2 d_prev = math::normalize(pts[i] - pts[i - 1]);
            float2 d_next = math::normalize(pts[i + 1] - pts[i]);
            float cross_val = d_prev.x * d_next.y - d_prev.y * d_next.x;

            if (cross_val > 0.0f) {
              left_crn[i] = math::max(crn[i] - half_t, 0.0f);
              right_crn[i] = crn[i] + half_t;
            }
            else {
              left_crn[i] = crn[i] + half_t;
              right_crn[i] = math::max(crn[i] - half_t, 0.0f);
            }
          }
        }

        /* Build expanded polygon: left forward, right backward */
        Vector<float2> expanded_pts;
        Vector<float> expanded_crn;
        expanded_pts.reserve(2 * pc);
        expanded_crn.reserve(2 * pc);

        for (int i = 0; i < pc; i++) {
          expanded_pts.append(left_side[i]);
          expanded_crn.append(left_crn[i]);
        }
        for (int i = pc - 1; i >= 0; i--) {
          expanded_pts.append(right_side[i]);
          expanded_crn.append(right_crn[i]);
        }

        pts = std::move(expanded_pts);
        crn = std::move(expanded_crn);
        pc = int(pts.size());
      }

      float max_corner = 0.0f;
      for (int i = 0; i < pc; i++) {
        max_corner = math::max(max_corner, crn[i]);
      }

      /* Radius clamping pre-pass */
      Vector<float> half_angles(pc);
      Vector<float> tan_halfs(pc);
      for (int i = 0; i < pc; i++) {
        int ip = (i - 1 + pc) % pc;
        int j = (i + 1) % pc;
        float2 to_prev = math::normalize(pts[ip] - pts[i]);
        float2 to_next = math::normalize(pts[j] - pts[i]);
        float cos_theta = math::clamp(math::dot(to_prev, to_next), -0.999f, 0.999f);
        half_angles[i] = acosf(cos_theta) * 0.5f;
        tan_halfs[i] = tanf(half_angles[i]);
      }

      for (int i = 0; i < pc; i++) {
        int j = (i + 1) % pc;
        float edge_len = math::length(pts[j] - pts[i]);
        if (edge_len < 1e-6f) {
          continue;
        }
        float t_i = (tan_halfs[i] > 1e-6f) ? crn[i] / tan_halfs[i] : 0.0f;
        float t_j = (tan_halfs[j] > 1e-6f) ? crn[j] / tan_halfs[j] : 0.0f;
        if (t_i + t_j > edge_len) {
          float scale = edge_len / (t_i + t_j);
          crn[i] *= scale;
          crn[j] *= scale;
        }
      }

      /* Precompute per-edge data */
      for (int i = 0; i < pc; i++) {
        int j = (i + 1) % pc;
        int ip = (i - 1 + pc) % pc;

        float2 edge = pts[j] - pts[i];
        float edge_len = math::length(edge);

        SDFPolygonPointGPU gpu_pt = {};
        gpu_pt.vi_edge = float4(pts[i].x, pts[i].y, edge.x, edge.y);

        float R_signed = 0.0f;
        float2 C = pts[i];
        float t_trim_start = 0.0f;
        float t_trim_end = 1.0f;
        float ang_mid = 0.0f;
        float ang_half = 0.0f;

        /* Recompute half-angle/tan for clamped radii */
        float2 to_prev = math::normalize(pts[ip] - pts[i]);
        float2 to_next = math::normalize(pts[j] - pts[i]);
        float cos_theta = math::clamp(math::dot(to_prev, to_next), -0.999f, 0.999f);
        float half = acosf(cos_theta) * 0.5f;
        float sin_half = sinf(half);
        float tan_half = tanf(half);

        if (crn[i] > 0.001f && pc >= 3 && sin_half > 0.01f && tan_half > 1e-6f) {
          float2 bisector = math::normalize(to_prev + to_next);
          float cross_val = to_prev.x * to_next.y - to_prev.y * to_next.x;

          float inset_dist = crn[i] / sin_half;
          C = pts[i] + bisector * inset_dist;

          R_signed = (cross_val > 0.0f) ? -crn[i] : crn[i];

          /* Trim start on this edge */
          if (edge_len > 1e-6f) {
            t_trim_start = crn[i] / (tan_half * edge_len);
          }

          /* Arc angular bounds */
          float2 tangent_start = pts[i] + math::normalize(edge) * (crn[i] / tan_half);
          float2 dir_start = math::normalize(tangent_start - C);
          float2 tangent_prev_end = pts[i] + to_prev * (crn[i] / tan_half);
          float2 dir_prev_end = math::normalize(tangent_prev_end - C);

          float a1 = atan2f(dir_prev_end.y, dir_prev_end.x);
          float a2 = atan2f(dir_start.y, dir_start.x);

          float diff = a2 - a1;
          diff -= 6.2831853f * floorf((diff + 3.1415927f) / 6.2831853f);
          ang_mid = a1 + diff * 0.5f;
          ang_half = fabsf(diff) * 0.5f;
        }

        /* Trim end on this edge (from next vertex's rounding) */
        if (crn[j] > 0.001f && pc >= 3) {
          float2 to_prev_j = math::normalize(pts[i] - pts[j]);
          float2 to_next_j = math::normalize(pts[(j + 1) % pc] - pts[j]);
          float cos_theta_j = math::clamp(math::dot(to_prev_j, to_next_j), -0.999f, 0.999f);
          float tan_half_j = tanf(acosf(cos_theta_j) * 0.5f);
          if (edge_len > 1e-6f && tan_half_j > 1e-6f) {
            t_trim_end = 1.0f - crn[j] / (tan_half_j * edge_len);
          }
        }

        gpu_pt.arc_data = float4(R_signed, C.x, C.y, t_trim_start);
        gpu_pt.arc_bounds = float4(t_trim_end, ang_mid, ang_half, 0.0f);

        polygon_points_.append(gpu_pt);
        gpu_obj.polygon_point_count++;
      }
      gpu_obj.box_corners = float4(max_corner, 0.0f, 0.0f, 0.0f);
      gpu_obj.box_edges = float4(sdf_data->polygon_edge_top,
                                 sdf_data->polygon_edge_bottom,
                                 math::max(poly_taper, 0.0f),
                                 math::max(-poly_taper, 0.0f));
      gpu_obj.box_modes = int4(0, sdf_data->polygon_edge_mode, 0, 0);
    }
    else if (sdf_data->sdf_type == SDF_TYPE_TORUS) {
      float angle_rad = sdf_data->torus_angle;
      float half_rad = angle_rad * 0.5f;
      gpu_obj.box_corners = float4(sinf(half_rad), cosf(half_rad), 0.0f, 0.0f);
      gpu_obj.box_edges = float4(0.0f);
      gpu_obj.box_modes = int4(0, 0, 0, (angle_rad < (float(M_PI) * 2.0f) - 0.001f) ? 1 : 0);
    }
    else {
      gpu_obj.box_corners = float4(sdf_data->box_corners[0],
                                   sdf_data->box_corners[1],
                                   sdf_data->box_corners[2],
                                   sdf_data->box_corners[3]);
      float taper = sdf_data->box_taper;
      gpu_obj.box_edges = float4(sdf_data->box_edge_top,
                                 sdf_data->box_edge_bottom,
                                 math::max(taper, 0.0f),
                                 math::max(-taper, 0.0f));
      gpu_obj.box_modes = int4(sdf_data->box_corner_mode, sdf_data->box_edge_mode, 0, 0);
    }

    /* Pack modifiers. Array is packed as a GPU domain-repetition (space-folding)
     * modifier rather than instanced into N separate objects. */
    gpu_obj.modifier_start = int(modifiers_.size());
    gpu_obj.modifier_count = 0;
    const SDFArrayModifierData *array_mod = nullptr;

    auto pack_array_gpu_mod = [&](const SDFArrayModifierData *am) -> SDFModifierGPU {
      SDFModifierGPU gm = {};
      gm.header = int4(SDF_MOD_ARRAY, am->array_type, 0, 0);
      float countf = float(math::max(am->count, 1));
      if (am->array_type == MOD_SDF_ARRAY_RADIAL) {
        /* Radius is absolute world units — object scale grows the unit, not the ring. */
        gm.params = float4(countf, am->array_radius, 0.0f, 0.0f);
      }
      else {
        /* Linear offset in object-local (rotated, world-scale) space. Relative tracks
         * the scaled unit; constant offset stays absolute. */
        float3 dims = base_dimensions * scale;
        float3 off(0.0f);
        if (am->use_relative_offset) {
          off += float3(am->relative_offset[0], am->relative_offset[1], am->relative_offset[2]) *
                 dims;
        }
        if (am->use_constant_offset) {
          off += float3(am->constant_offset[0], am->constant_offset[1], am->constant_offset[2]);
        }
        gm.params = float4(countf, off.x, off.y, off.z);
      }
      gm.params2 = float4(am->blend, 0.0f, 0.0f, 0.0f);
      return gm;
    };

    for (const ModifierData *md = static_cast<const ModifierData *>(ob->modifiers.first);
         md; md = md->next)
    {
      if (!(md->mode & eModifierMode_Realtime)) {
        continue;
      }
      if (md->type == eModifierType_SDFArray) {
        array_mod = reinterpret_cast<const SDFArrayModifierData *>(md);
        if (array_mod->count > 1) {
          modifiers_.append(pack_array_gpu_mod(array_mod));
          gpu_obj.modifier_count++;
        }
        continue;
      }
      SDFModifierGPU gpu_mod = {};
      bool valid = false;
      switch (md->type) {
        case eModifierType_SDFMirror: {
          const auto &m = *reinterpret_cast<const SDFMirrorModifierData *>(md);
          float3 obj_pos = float3(mat[3]);
          float3 mirror_pos(0.0f);
          if (m.mirror_object != nullptr) {
            mirror_pos = float3(m.mirror_object->object_to_world()[3]);
          }
          float3 world_origin = mirror_pos - obj_pos;
          float3 local_origin = float3(inv_rot * float4(world_origin, 0.0f));
          int sides = ((obj_pos.x >= mirror_pos.x) ? 1 : 0) |
                      ((obj_pos.y >= mirror_pos.y) ? 2 : 0) |
                      ((obj_pos.z >= mirror_pos.z) ? 4 : 0);
          gpu_mod.header = int4(SDF_MOD_MIRROR, m.flag, m.blend_type, sides);
          gpu_mod.params = float4(
              m.offset_distance, local_origin.x, local_origin.y, local_origin.z);
          gpu_mod.params2 = float4(m.blend, 0.0f, float(m.blend_type), 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFTwist: {
          const auto &m = *reinterpret_cast<const SDFTwistModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_TWIST, 0, 0, 0);
          gpu_mod.params = float4(m.strength, float(m.axis), 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFBend: {
          const auto &m = *reinterpret_cast<const SDFBendModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_BEND, 0, 0, 0);
          gpu_mod.params = float4(m.strength, float(m.axis), m.origin[0], m.origin[1]);
          gpu_mod.params2 = float4(m.origin[2], 0.0f, 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFElongate: {
          const auto &m = *reinterpret_cast<const SDFElongateModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_ELONGATE, 0, 0, 0);
          gpu_mod.params = float4(m.elongation[0], m.elongation[1], m.elongation[2], 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFSolidify: {
          const auto &m = *reinterpret_cast<const SDFSolidifyModifierData *>(md);
          float inner_scale = 1.0f;
          if (m.mode == SDF_SOLIDIFY_OPEN) {
            int ax = math::clamp(m.axis, 0, 2);
            float ssz = math::max(base_dimensions[ax] * scale[ax] * 0.5f, 0.001f);
            inner_scale = math::max(1.0f - 2.0f * m.thickness / ssz, 0.1f);
          }
          gpu_mod.header = int4(SDF_MOD_SOLIDIFY, m.mode, m.axis, 0);
          gpu_mod.params = float4(m.thickness, inner_scale, m.bevel, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFRound: {
          const auto &m = *reinterpret_cast<const SDFRoundModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_ROUND, 0, 0, 0);
          gpu_mod.params = float4(m.offset, 0.0f, 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFOnion: {
          const auto &m = *reinterpret_cast<const SDFOnionModifierData *>(md);
          int layers = math::max(m.layers, 1);
          float3 osz = (sdf_data->sdf_type == SDF_TYPE_MESH) ?
                           base_dimensions * scale * 0.5f :
                           float3(gpu_obj.sdf_size);
          float min_ext = math::min(osz.x, math::min(osz.y, osz.z));
          gpu_mod.header = int4(SDF_MOD_ONION, layers, 0, 0);
          gpu_mod.params = float4(m.gap, min_ext, 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFBevel: {
          const auto &m = *reinterpret_cast<const SDFBevelModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_BEVEL, 0, 0, 0);
          gpu_mod.params = float4(m.radius, 0.0f, 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFDisplace: {
          const auto &m = *reinterpret_cast<const SDFDisplaceModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_DISPLACE, m.noise_type, m.octaves, 0);
          gpu_mod.params = float4(m.strength, m.frequency, m.lacunarity, m.roughness);
          valid = true;
          break;
        }
        default:
          break;
      }
      if (valid) {
        modifiers_.append(gpu_mod);
        gpu_obj.modifier_count++;
      }
    }

    /* Compute base local extent for AABB (without array) */
    float shell_expand = (sdf_data->csg_operation == SDF_CSG_SHELL) ?
                             fabsf(sdf_data->shell_distance) :
                             0.0f;
    float blend_pad = (sdf_data->blend_type != 0) ? sdf_data->blend : 0.0f;
    float pad = std::max(blend_pad, sdf_data->color_blend) + shell_expand;
    float3 base_local_extent;
    {
      /* Base (unscaled) extent; per-axis object scale applied afterward. */
      float3 sz = base_size;
      switch (sdf_data->sdf_type) {
        case SDF_TYPE_MESH:
          base_local_extent = math::max(math::abs(float3(sdf_data->mesh_bounds_min)),
                                        math::abs(float3(sdf_data->mesh_bounds_max)));
          break;
        case SDF_TYPE_CAPSULE: {
          float r = sz.x;
          float h = sz.y;
          base_local_extent = float3(r, r, h + r);
          break;
        }
        case SDF_TYPE_CYLINDER:
          base_local_extent = sz;
          break;
        case SDF_TYPE_CONE: {
          float r = std::max(sz.x, sz.z);
          float h = sz.y;
          base_local_extent = float3(r, r, h);
          break;
        }
        case SDF_TYPE_TORUS: {
          float outer = sz.x + sz.y;
          base_local_extent = float3(outer, outer, sz.y);
          break;
        }
        case SDF_TYPE_NGON: {
          float r = sz.x;
          base_local_extent = float3(r, r, sz.z);
          break;
        }
        case SDF_TYPE_POLYGON: {
          float max_x = 0.0f, max_y = 0.0f;
          for (const SDFPolygonPoint *pt = static_cast<const SDFPolygonPoint *>(
                   sdf_data->polygon_points.first);
               pt; pt = pt->next)
          {
            max_x = math::max(max_x, fabsf(pt->co[0]));
            max_y = math::max(max_y, fabsf(pt->co[1]));
          }
          float lp = sdf_data->polygon_is_line ?
                         sdf_data->polygon_line_thickness * 0.5f :
                         0.0f;
          base_local_extent = float3(max_x + lp, max_y + lp, sz.z);
          break;
        }
        default:
          base_local_extent = sz;
          break;
      }
      base_local_extent = base_local_extent * scale + float3(pad);
    }

    auto expand_modifier_bound = [&](const SDFModifierGPU &mod) {
      int mtype = mod.header.x;
      int mflags = mod.header.y;
      switch (mtype) {
        case SDF_MOD_MIRROR: {
          float offset = fabsf(mod.params.x);
          float3 local_org = float3(mod.params.y, mod.params.z, mod.params.w);
          float4x4 mir_inv = gpu_obj.inverse_matrix;
          if (mflags & SDF_MOD_MIRROR_X) {
            float3 N = float3(mir_inv[0][0], mir_inv[0][1], mir_inv[0][2]);
            float ea = math::dot(base_local_extent, math::abs(N));
            float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, ea);
            base_local_extent += math::abs(N) * disp;
          }
          if (mflags & SDF_MOD_MIRROR_Y) {
            float3 N = float3(mir_inv[1][0], mir_inv[1][1], mir_inv[1][2]);
            float ea = math::dot(base_local_extent, math::abs(N));
            float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, ea);
            base_local_extent += math::abs(N) * disp;
          }
          if (mflags & SDF_MOD_MIRROR_Z) {
            float3 N = float3(mir_inv[2][0], mir_inv[2][1], mir_inv[2][2]);
            float ea = math::dot(base_local_extent, math::abs(N));
            float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, ea);
            base_local_extent += math::abs(N) * disp;
          }
          break;
        }
        case SDF_MOD_ELONGATE:
          base_local_extent += float3(mod.params.x, mod.params.y, mod.params.z);
          break;
        case SDF_MOD_SOLIDIFY:
        case SDF_MOD_ONION:
          base_local_extent += float3(mod.params.x);
          break;
        case SDF_MOD_DISPLACE:
          base_local_extent += float3(math::abs(mod.params.x));
          break;
        case SDF_MOD_ROUND:
        case SDF_MOD_BEVEL:
          base_local_extent = math::max(base_local_extent + float3(mod.params.x), float3(0.0f));
          break;
        case SDF_MOD_TWIST: {
          int axis = int(mod.params.y);
          if (axis == 1) {
            float xz = math::sqrt(base_local_extent.x * base_local_extent.x +
                                  base_local_extent.z * base_local_extent.z);
            base_local_extent.x = xz;
            base_local_extent.z = xz;
          }
          else if (axis == 2) {
            float yz = math::sqrt(base_local_extent.y * base_local_extent.y +
                                  base_local_extent.z * base_local_extent.z);
            base_local_extent.y = yz;
            base_local_extent.z = yz;
          }
          else {
            float xy = math::sqrt(base_local_extent.x * base_local_extent.x +
                                  base_local_extent.y * base_local_extent.y);
            base_local_extent.x = xy;
            base_local_extent.y = xy;
          }
          break;
        }
        case SDF_MOD_BEND: {
          float k = mod.params.x;
          int axis = int(mod.params.y);
          float3 origin = float3(mod.params.z, mod.params.w, mod.params2.x);
          if (fabsf(k) > 0.0001f) {
            const float3 relative_extent = base_local_extent + math::abs(origin);
            if (axis == 1) {
              const float radius = math::sqrt(math::square(relative_extent.y) +
                                              math::square(relative_extent.z));
              base_local_extent.y = radius + math::abs(origin.y);
              base_local_extent.z = radius + math::abs(origin.z);
            }
            else if (axis == 2) {
              const float radius = math::sqrt(math::square(relative_extent.z) +
                                              math::square(relative_extent.x));
              base_local_extent.z = radius + math::abs(origin.z);
              base_local_extent.x = radius + math::abs(origin.x);
            }
            else {
              const float radius = math::sqrt(math::square(relative_extent.x) +
                                              math::square(relative_extent.y));
              base_local_extent.x = radius + math::abs(origin.x);
              base_local_extent.y = radius + math::abs(origin.y);
            }
          }
          break;
        }
        default:
          break;
      }
    };
    for (int mi = gpu_obj.modifier_start;
         mi < gpu_obj.modifier_start + gpu_obj.modifier_count;
         mi++)
    {
      expand_modifier_bound(modifiers_[mi]);
    }

    /* Propagate group parent's modifiers to this child.
     * Each child gets the group's modifiers applied individually:
     * Array → instancing, others → appended as GPU modifiers. */
    const int group_modifier_start = int(modifiers_.size());
    if (group_parent_ob) {
      for (const ModifierData *gmd = static_cast<const ModifierData *>(
               group_parent_ob->modifiers.first);
           gmd; gmd = gmd->next)
      {
        if (!(gmd->mode & eModifierMode_Realtime)) {
          continue;
        }
        if (gmd->type == eModifierType_SDFArray) {
          if (!array_mod) {
            array_mod = reinterpret_cast<const SDFArrayModifierData *>(gmd);
            if (array_mod->count > 1) {
              modifiers_.append(pack_array_gpu_mod(array_mod));
              gpu_obj.modifier_count++;
            }
          }
          continue;
        }
        /* Analytic primitive bevel. */
        if (gmd->type == eModifierType_SDFBevel && sdf_data->sdf_type != SDF_TYPE_MESH) {
          continue;
        }
        SDFModifierGPU gpu_mod = {};
        bool valid = false;
        switch (gmd->type) {
          case eModifierType_SDFMirror: {
            const auto &m = *reinterpret_cast<const SDFMirrorModifierData *>(gmd);
            float3 obj_pos = float3(mat[3]);
            float3 mirror_pos(0.0f);
            if (m.mirror_object != nullptr) {
              mirror_pos = float3(m.mirror_object->object_to_world()[3]);
            }
            float3 world_origin = mirror_pos - obj_pos;
            float3 local_origin = float3(inv_rot * float4(world_origin, 0.0f));
            int sides = ((obj_pos.x >= mirror_pos.x) ? 1 : 0) |
                        ((obj_pos.y >= mirror_pos.y) ? 2 : 0) |
                        ((obj_pos.z >= mirror_pos.z) ? 4 : 0);
            gpu_mod.header = int4(SDF_MOD_MIRROR, m.flag, m.blend_type, sides);
            gpu_mod.params = float4(
                m.offset_distance, local_origin.x, local_origin.y, local_origin.z);
            gpu_mod.params2 = float4(m.blend, 0.0f, float(m.blend_type), 0.0f);
            valid = true;
            break;
          }
          case eModifierType_SDFTwist: {
            const auto &m = *reinterpret_cast<const SDFTwistModifierData *>(gmd);
            gpu_mod.header = int4(SDF_MOD_TWIST, 0, 0, 0);
            gpu_mod.params = float4(m.strength, float(m.axis), 0.0f, 0.0f);
            valid = true;
            break;
          }
          case eModifierType_SDFBend: {
            const auto &m = *reinterpret_cast<const SDFBendModifierData *>(gmd);
            gpu_mod.header = int4(SDF_MOD_BEND, 0, 0, 0);
            gpu_mod.params = float4(m.strength, float(m.axis), m.origin[0], m.origin[1]);
            gpu_mod.params2 = float4(m.origin[2], 0.0f, 0.0f, 0.0f);
            valid = true;
            break;
          }
          case eModifierType_SDFElongate: {
            const auto &m = *reinterpret_cast<const SDFElongateModifierData *>(gmd);
            gpu_mod.header = int4(SDF_MOD_ELONGATE, 0, 0, 0);
            gpu_mod.params = float4(m.elongation[0], m.elongation[1], m.elongation[2], 0.0f);
            valid = true;
            break;
          }
          case eModifierType_SDFSolidify: {
            const auto &m = *reinterpret_cast<const SDFSolidifyModifierData *>(gmd);
            float inner_scale = 1.0f;
            if (m.mode == SDF_SOLIDIFY_OPEN) {
              int ax = math::clamp(m.axis, 0, 2);
              float ssz = math::max(base_dimensions[ax] * scale[ax] * 0.5f, 0.001f);
              inner_scale = math::max(1.0f - 2.0f * m.thickness / ssz, 0.1f);
            }
            gpu_mod.header = int4(SDF_MOD_SOLIDIFY, m.mode, m.axis, 0);
            gpu_mod.params = float4(m.thickness, inner_scale, m.bevel, 0.0f);
            valid = true;
            break;
          }
          case eModifierType_SDFRound: {
            const auto &m = *reinterpret_cast<const SDFRoundModifierData *>(gmd);
            gpu_mod.header = int4(SDF_MOD_ROUND, 0, 0, 0);
            gpu_mod.params = float4(m.offset, 0.0f, 0.0f, 0.0f);
            valid = true;
            break;
          }
          case eModifierType_SDFOnion: {
            const auto &m = *reinterpret_cast<const SDFOnionModifierData *>(gmd);
            int layers = math::max(m.layers, 1);
            float3 osz = (sdf_data->sdf_type == SDF_TYPE_MESH) ?
                             base_dimensions * scale * 0.5f :
                             float3(gpu_obj.sdf_size);
            float min_ext = math::min(osz.x, math::min(osz.y, osz.z));
            gpu_mod.header = int4(SDF_MOD_ONION, layers, 0, 0);
            gpu_mod.params = float4(m.gap, min_ext, 0.0f, 0.0f);
            valid = true;
            break;
          }
          case eModifierType_SDFBevel: {
            const auto &m = *reinterpret_cast<const SDFBevelModifierData *>(gmd);
            gpu_mod.header = int4(SDF_MOD_BEVEL, 0, 0, 0);
            gpu_mod.params = float4(m.radius, 0.0f, 0.0f, 0.0f);
            valid = true;
            break;
          }
          case eModifierType_SDFDisplace: {
            const auto &m = *reinterpret_cast<const SDFDisplaceModifierData *>(gmd);
            gpu_mod.header = int4(SDF_MOD_DISPLACE, m.noise_type, m.octaves, 0);
            gpu_mod.params = float4(m.strength, m.frequency, m.lacunarity, m.roughness);
            valid = true;
            break;
          }
          default:
            break;
        }
        if (valid) {
          modifiers_.append(gpu_mod);
          gpu_obj.modifier_count++;
        }
      }
    }
    for (int mi = group_modifier_start; mi < int(modifiers_.size()); mi++) {
      expand_modifier_bound(modifiers_[mi]);
    }

    /* Expand the AABB for the array domain repetition (object or group array). */
    if (array_mod != nullptr && array_mod->count > 1) {
      if (array_mod->array_type == MOD_SDF_ARRAY_RADIAL) {
        float rr = math::max(base_local_extent.x, base_local_extent.y) + array_mod->array_radius;
        base_local_extent.x = rr;
        base_local_extent.y = rr;
      }
      else {
        float3 dims = base_dimensions * scale;
        float3 off(0.0f);
        if (array_mod->use_relative_offset) {
          off += float3(array_mod->relative_offset[0], array_mod->relative_offset[1],
                        array_mod->relative_offset[2]) * dims;
        }
        if (array_mod->use_constant_offset) {
          off += float3(array_mod->constant_offset[0], array_mod->constant_offset[1],
                        array_mod->constant_offset[2]);
        }
        base_local_extent += math::abs(off) * float(array_mod->count - 1);
      }
    }

    /* Array is a domain-repetition modifier now — emit a single object (no instancing). */
    int copy_count = 1;

    /* Precompute object offset delta (local space) */
    float4x4 obj_delta = float4x4::identity();
    bool has_obj_offset = (array_mod != nullptr && array_mod->use_object_offset &&
                           array_mod->offset_object != nullptr);
    if (has_obj_offset) {
      float4x4 inv_base = math::invert(mat);
      obj_delta = inv_base * array_mod->offset_object->object_to_world();
    }

    /* Precompute linear offset in world space from object dimensions */
    float3 world_linear_offset(0.0f);
    if (array_mod != nullptr && array_mod->array_type == MOD_SDF_ARRAY_LINEAR) {
      float3 dimensions = base_dimensions * scale;
      float3 local_off(0.0f);
      if (array_mod->use_relative_offset) {
        local_off += float3(array_mod->relative_offset[0],
                            array_mod->relative_offset[1],
                            array_mod->relative_offset[2]) * dimensions;
      }
      if (array_mod->use_constant_offset) {
        local_off += float3(array_mod->constant_offset[0],
                            array_mod->constant_offset[1],
                            array_mod->constant_offset[2]);
      }
      world_linear_offset = float3(rot_mat * float4(local_off, 0.0f));
    }

    float group_pad = 0.0f;

    float4x4 accum_delta = float4x4::identity();
    for (int ci = 0; ci < copy_count; ci++) {
      /* Compute per-copy world matrix */
      float4x4 copy_mat;
      if (ci == 0) {
        copy_mat = mat;
      }
      else if (array_mod->array_type == MOD_SDF_ARRAY_RADIAL) {
        float angle = 2.0f * float(M_PI) * float(ci) / float(copy_count);
        float ca = cosf(angle), sa = sinf(angle);
        float4x4 radial_rot = float4x4::identity();
        radial_rot[0][0] = ca;  radial_rot[0][1] = sa;
        radial_rot[1][0] = -sa; radial_rot[1][1] = ca;
        float4x4 radial_xlate = float4x4::identity();
        radial_xlate[3][0] = array_mod->array_radius;
        /* Per-copy rotation offset (accumulated) */
        float3 rot = float3(array_mod->rotation_offset[0],
                             array_mod->rotation_offset[1],
                             array_mod->rotation_offset[2]) * float(ci);
        float4x4 per_rot = float4x4::identity();
        if (fabsf(rot.x) > 0.0001f) {
          float cx = cosf(rot.x), sx = sinf(rot.x);
          float4x4 rx = float4x4::identity();
          rx[1][1] = cx; rx[1][2] = sx;
          rx[2][1] = -sx; rx[2][2] = cx;
          per_rot = per_rot * rx;
        }
        if (fabsf(rot.y) > 0.0001f) {
          float cy = cosf(rot.y), sy = sinf(rot.y);
          float4x4 ry = float4x4::identity();
          ry[0][0] = cy; ry[0][2] = -sy;
          ry[2][0] = sy; ry[2][2] = cy;
          per_rot = per_rot * ry;
        }
        if (fabsf(rot.z) > 0.0001f) {
          float cz = cosf(rot.z), sz = sinf(rot.z);
          float4x4 rz = float4x4::identity();
          rz[0][0] = cz; rz[0][1] = sz;
          rz[1][0] = -sz; rz[1][1] = cz;
          per_rot = per_rot * rz;
        }
        float4x4 local_xform = radial_rot * radial_xlate * per_rot;
        if (has_obj_offset) {
          local_xform = local_xform * accum_delta;
        }
        copy_mat = mat * local_xform;
      }
      else {
        /* Linear: base transform + object delta + world linear offset */
        copy_mat = has_obj_offset ? (mat * accum_delta) : mat;
        copy_mat[3] += float4(world_linear_offset * float(ci), 0.0f);
      }

      /* Accumulate object offset delta */
      if (has_obj_offset && ci == 0) {
        accum_delta = obj_delta;
      }
      else if (has_obj_offset && ci > 0) {
        accum_delta = accum_delta * obj_delta;
      }

      /* Extract scale / rotation from copy matrix */
      float3 copy_scale;
      copy_scale.x = math::length(float3(copy_mat[0]));
      copy_scale.y = math::length(float3(copy_mat[1]));
      copy_scale.z = math::length(float3(copy_mat[2]));

      float4x4 copy_rot = copy_mat;
      if (copy_scale.x > 0.0f) {
        copy_rot[0] = float4(float3(copy_mat[0]) / copy_scale.x, 0.0f);
      }
      if (copy_scale.y > 0.0f) {
        copy_rot[1] = float4(float3(copy_mat[1]) / copy_scale.y, 0.0f);
      }
      if (copy_scale.z > 0.0f) {
        copy_rot[2] = float4(float3(copy_mat[2]) / copy_scale.z, 0.0f);
      }
      copy_rot[3] = float4(0.0f, 0.0f, 0.0f, 1.0f);
      float4x4 copy_inv_rot = math::invert(copy_rot);

      SDFObjectGPU copy_obj = gpu_obj;
      copy_obj.inverse_matrix = copy_inv_rot;
      copy_obj.position = float4(copy_mat[3].x, copy_mat[3].y, copy_mat[3].z, 0.0f);

      /* Adjust sdf_size for per-copy scale changes */
      if (ci > 0 && has_obj_offset) {
        float3 crs = float3(sdf_data->size[0] * copy_scale.x,
                             sdf_data->size[1] * copy_scale.y,
                             sdf_data->size[2] * copy_scale.z);
        float cmd = math::reduce_min(crs);
        float cb = std::max(copy_obj.bevel, std::min(0.005f, cmd * 0.5f));
        float3 cns = math::max(crs - float3(cb), float3(0.001f));
        copy_obj.sdf_size = float4(cns.x, cns.y, cns.z, cb);
      }

      /* All copies are exact clones of the original SDF (same blend, CSG, etc.) */

      /* Per-copy AABB: recompute from SDF size with this copy's scale */
      float3 copy_local_extent;
      if (ci > 0 && has_obj_offset) {
        float3 copy_sz = float3(sdf_data->size[0] * copy_scale.x,
                                sdf_data->size[1] * copy_scale.y,
                                sdf_data->size[2] * copy_scale.z);
        float copy_pad = pad;
        switch (sdf_data->sdf_type) {
          case SDF_TYPE_CAPSULE: {
            float r = copy_sz.x;
            float h = copy_sz.y;
            copy_local_extent = float3(r + copy_pad, r + copy_pad, h + r + copy_pad);
            break;
          }
          case SDF_TYPE_TORUS: {
            float outer = copy_sz.x + copy_sz.y;
            copy_local_extent = float3(outer + copy_pad, outer + copy_pad, copy_sz.y + copy_pad);
            break;
          }
          case SDF_TYPE_CONE: {
            float r = std::max(copy_sz.x, copy_sz.z), h = copy_sz.y;
            copy_local_extent = float3(r + copy_pad, r + copy_pad, h + copy_pad);
            break;
          }
          default:
            copy_local_extent = copy_sz + float3(copy_pad);
            break;
        }
        /* Expand for shared domain modifiers */
        for (int mi = copy_obj.modifier_start;
             mi < copy_obj.modifier_start + copy_obj.modifier_count; mi++)
        {
          const SDFModifierGPU &mod = modifiers_[mi];
          int mtype = mod.header.x;
          switch (mtype) {
            case SDF_MOD_ELONGATE:
              copy_local_extent += float3(mod.params.x, mod.params.y, mod.params.z);
              break;
            case SDF_MOD_SOLIDIFY:
            case SDF_MOD_ONION:
              copy_local_extent += float3(mod.params.x);
              break;
            case SDF_MOD_DISPLACE:
              copy_local_extent += float3(math::abs(mod.params.x));
              break;
            case SDF_MOD_ROUND:
            case SDF_MOD_BEVEL:
              copy_local_extent = math::max(copy_local_extent + float3(mod.params.x), float3(0.0f));
              break;
            default:
              break;
          }
        }
      }
      else {
        copy_local_extent = base_local_extent;
      }

      float3 world_min = float3(1e30f);
      float3 world_max = float3(-1e30f);
      for (int corner = 0; corner < 8; corner++) {
        float3 lc;
        if (sdf_data->sdf_type == SDF_TYPE_MESH && copy_obj.modifier_count == 0) {
          float3 mesh_min = float3(sdf_data->mesh_bounds_min) * float3(copy_obj.obj_scale) -
                            float3(pad);
          float3 mesh_max = float3(sdf_data->mesh_bounds_max) * float3(copy_obj.obj_scale) +
                            float3(pad);
          lc = float3((corner & 1) ? mesh_max.x : mesh_min.x,
                      (corner & 2) ? mesh_max.y : mesh_min.y,
                      (corner & 4) ? mesh_max.z : mesh_min.z);
        }
        else {
          lc = float3((corner & 1) ? copy_local_extent.x : -copy_local_extent.x,
                      (corner & 2) ? copy_local_extent.y : -copy_local_extent.y,
                      (corner & 4) ? copy_local_extent.z : -copy_local_extent.z);
        }
        float3 wc = float3(copy_rot * float4(lc, 0.0f)) +
                     float3(copy_mat[3].x, copy_mat[3].y, copy_mat[3].z);
        world_min = math::min(world_min, wc);
        world_max = math::max(world_max, wc);
      }

      copy_obj.bbox_min = float4(world_min, 0.0f);
      copy_obj.bbox_max = float4(world_max, 0.0f);
      copy_obj.orig_bbox_min = copy_obj.bbox_min;
      copy_obj.orig_bbox_max = copy_obj.bbox_max;

      /* Per-copy frustum cull */
      if (use_frustum_cull_) {
        float3 pad_min = world_min - float3(group_pad);
        float3 pad_max = world_max + float3(group_pad);
        bool outside = false;
        for (int p = 0; p < 6; p++) {
          float3 n(frustum_planes_[p]);
          float d = frustum_planes_[p].w;
          float3 pos_vertex(n.x > 0 ? pad_max.x : pad_min.x,
                            n.y > 0 ? pad_max.y : pad_min.y,
                            n.z > 0 ? pad_max.z : pad_min.z);
          if (math::dot(n, pos_vertex) + d < 0.0f) {
            outside = true;
            break;
          }
        }
        if (outside) {
          continue;
        }
      }

      float sphere_radius = math::length(copy_local_extent);
      float3 obj_center = float3(copy_mat[3].x, copy_mat[3].y, copy_mat[3].z);
      scene_min_ = math::min(scene_min_, obj_center - float3(sphere_radius));
      scene_max_ = math::max(scene_max_, obj_center + float3(sphere_radius));

      max_blend_ = math::max(max_blend_, std::max(copy_obj.blend, copy_obj.color_blend));
      {
        float sf = 0.85f;
        if (copy_obj.blend > 0.001f && copy_obj.blend_type > 0) {
          sf = (copy_obj.blend_type == SDF_BLEND_SMOOTH) ? 0.75f : 0.65f;
          if (copy_obj.blend > 0.5f) { sf = min_ff(sf, 0.5f); }
        }
        step_factor_ = min_ff(step_factor_, sf);
        copy_obj.orig_bbox_min.w = sf;
      }
      if (copy_obj.csg_operation == SDF_CSG_SHELL) {
        max_shell_distance_ = math::max(max_shell_distance_,
                                        fabsf(copy_obj.shell_distance));
      }

      object_ptrs_.append(ob);
      objects_.append(copy_obj);
    }
  }

  void end_sync() final
  {
    if (objects_.is_empty()) {
      clear_exported_state();
      return;
    }

    /* Build group GPU data */
    {
      groups_gpu_.clear();

      struct GroupMembership {
        int group_id;
        int group_order;
      };
      Map<Object *, GroupMembership> object_membership_map;

      int g_idx = 0;

      /* Parent-child group path: SDF_TYPE_GROUP empties as parents */
      for (Object *group_ob : group_empties_) {
        Object *orig_group = DEG_get_original(group_ob);
        const SDF *grp_sdf = id_cast<const SDF *>(group_ob->data);
        if (!grp_sdf) {
          continue;
        }

        SDFGroupGPU gpu_grp = {};
        gpu_grp.csg_operation = grp_sdf->csg_operation;
        gpu_grp.blend_type = grp_sdf->blend_type;
        gpu_grp.blend = grp_sdf->blend;
        gpu_grp.clearance = grp_sdf->clearance;
        gpu_grp.color_blend = grp_sdf->color_blend;
        gpu_grp.color_blend_type = grp_sdf->color_blend_type;
        gpu_grp.shell_distance = grp_sdf->shell_distance;
        gpu_grp.shell_mode = grp_sdf->shell_mode;
        gpu_grp.shell_op = grp_sdf->shell_op;
        gpu_grp.shell_blend_top = grp_sdf->shell_blend_top;
        gpu_grp.shell_blend_bottom = grp_sdf->shell_blend_bottom;
        gpu_grp.chamfer_k2 = grp_sdf->chamfer_k2;
        gpu_grp.chamfer_k3 = grp_sdf->chamfer_k3;
        gpu_grp.chamfer_k4 = grp_sdf->chamfer_k4;
        gpu_grp.chamfer_k5 = grp_sdf->chamfer_k5;
        gpu_grp.flip_blend = grp_sdf->flip_blend;
        gpu_grp.flip_blend_end = grp_sdf->flip_blend_end;
        gpu_grp.color = float4(grp_sdf->color[0], grp_sdf->color[1],
                               grp_sdf->color[2], grp_sdf->color[3]);
        gpu_grp.first_object = 0;
        gpu_grp.object_count = 0;

        /* Group modifiers are now propagated to each child during per-object sync.
         * No group-level modifier packing needed. */
        gpu_grp.modifier_start = 0;
        gpu_grp.modifier_count = 0;

        float grp_eff_blend = (gpu_grp.csg_operation == SDF_CSG_SHELL)
                                  ? std::max(gpu_grp.shell_blend_top, gpu_grp.shell_blend_bottom)
                                  : gpu_grp.blend;
        if (grp_eff_blend > 0.001f && gpu_grp.blend_type > 0) {
          float sf = (gpu_grp.blend_type == SDF_BLEND_SMOOTH) ? 0.75f : 0.65f;
          if (grp_eff_blend > 0.5f) {
            sf = min_ff(sf, 0.5f);
          }
          step_factor_ = min_ff(step_factor_, sf);
        }

        groups_gpu_.append(gpu_grp);

        /* Map children to this group, ordered by sdf_index */
        struct ChildEntry {
          int obj_idx;
          int sdf_index;
        };
        Vector<ChildEntry> child_entries;
        for (int i = 0; i < int(object_ptrs_.size()); i++) {
          Object *eval_ob = object_ptrs_[i];
          Object *orig_ob = DEG_get_original(eval_ob);
          if (orig_ob->parent == orig_group) {
            const int sidx = objects_[i].original_index;
            child_entries.append({i, sidx});
          }
        }
        std::sort(child_entries.begin(), child_entries.end(),
                  [](const ChildEntry &a, const ChildEntry &b) {
                    return a.sdf_index < b.sdf_index;
                  });
        int child_order = 0;
        for (const auto &entry : child_entries) {
          Object *orig_ob = DEG_get_original(object_ptrs_[entry.obj_idx]);
          object_membership_map.add_overwrite(orig_ob, {g_idx, child_order});
          child_order++;
        }

        g_idx++;
      }

      BLI_assert(int(object_ptrs_.size()) == int(objects_.size()));
      Vector<int> group_orders(int(objects_.size()), 0);
      for (int i = 0; i < int(objects_.size()); i++) {
        Object *eval_ob = object_ptrs_[i];
        Object *orig_ob = DEG_get_original(eval_ob);
        const GroupMembership *membership = object_membership_map.lookup_ptr(orig_ob);
        if (membership) {
          objects_[i].group_id = membership->group_id;
          group_orders[i] = membership->group_order;
        }
      }

      /* (diagnostic moved after fix-up) */

      /* Build map of group_id -> group empty's sdf_index for sorting */
      Map<int, int> group_sort_key;
      {
        int gi = 0;
        for (Object *group_ob : group_empties_) {
          const SDF *gs = id_cast<const SDF *>(group_ob->data);
          group_sort_key.add(gi, gs ? gs->sdf_index : 0);
          gi++;
        }
      }

      /* Sort objects: groups and ungrouped interleaved by sdf_index */
      {
        const int n = int(objects_.size());
        if (n > 0) {
          Vector<std::pair<int64_t, int>> sort_pairs(n);
          for (int i = 0; i < n; i++) {
            if (objects_[i].group_id >= 0) {
              int gkey = 0;
              const int *gk = group_sort_key.lookup_ptr(objects_[i].group_id);
              if (gk) { gkey = *gk; }
              sort_pairs[i] = {int64_t(gkey) * 1000000LL + group_orders[i], i};
            }
            else {
              sort_pairs[i] = {int64_t(objects_[i].original_index) * 1000000LL, i};
            }
          }
          std::stable_sort(sort_pairs.begin(), sort_pairs.end());

          Vector<SDFObjectGPU> sorted(n);
          Vector<int> old_to_new(n);
          for (int new_idx = 0; new_idx < n; new_idx++) {
            int old_idx = sort_pairs[new_idx].second;
            sorted[new_idx] = objects_[old_idx];
            old_to_new[old_idx] = new_idx;
          }
          objects_ = std::move(sorted);

          /* Build sorted_index → Object* array for overlay */
          s_sorted_object_ptrs.reinitialize(n);
          for (int new_idx = 0; new_idx < n; new_idx++) {
            int old_idx = sort_pairs[new_idx].second;
            s_sorted_object_ptrs[new_idx] = object_ptrs_[old_idx];
          }

          /* Build depsgraph-to-sorted mapping for the overlay.
           * The overlay has one entry per unique depsgraph object (no Array copies).
           * Map each unique object to its first copy's sorted position. */
          s_depsgraph_to_sorted.clear();
          s_object_to_sorted.clear();
          Object *prev_ptr = nullptr;
          for (int old_idx = 0; old_idx < n; old_idx++) {
            Object *ptr = object_ptrs_[old_idx];
            if (ptr != prev_ptr) {
              s_depsgraph_to_sorted.append(old_to_new[old_idx]);
              /* Direct Object* → sorted index lookup (robust, order-independent) */
              Object *orig = DEG_get_original(ptr);
              if (!s_object_to_sorted.contains(orig)) {
                s_object_to_sorted.add(orig, old_to_new[old_idx]);
              }
              prev_ptr = ptr;
            }
          }
        }
      }

      /* Fix first_object/object_count per group, force first member to Union */
      for (int gi = 0; gi < int(groups_gpu_.size()); gi++) {
        int first = -1;
        int count = 0;
        for (int i = 0; i < int(objects_.size()); i++) {
          if (objects_[i].group_id == gi) {
            if (first == -1) {
              first = i;
            }
            count++;
          }
        }
        groups_gpu_[gi].first_object = (first >= 0) ? first : 0;
        groups_gpu_[gi].object_count = count;
        if (first >= 0) {
          objects_[first].csg_operation = SDF_CSG_UNION;
        }
      }

      /* Force first stack entry to Union. */
      if (!objects_.is_empty()) {
        const int first_group = objects_.first().group_id;
        if (first_group >= 0) {
          groups_gpu_[first_group].csg_operation = SDF_CSG_UNION;
        }
        else {
          objects_.first().csg_operation = SDF_CSG_UNION;
        }
      }

      /* Expand group members' AABBs so they appear in all required tiles.
       * Per-object AABBs already account for per-object modifiers (including
       * group-propagated ones like Array). Just unify within each group. */
      for (int gi = 0; gi < int(groups_gpu_.size()); gi++) {
        int start = groups_gpu_[gi].first_object;
        int cnt = groups_gpu_[gi].object_count;
        if (cnt < 1) {
          continue;
        }
        int op = groups_gpu_[gi].csg_operation;

        float3 combined_min(1e30f), combined_max(-1e30f);
        for (int m = start; m < start + cnt; m++) {
          combined_min = math::min(combined_min, float3(objects_[m].bbox_min));
          combined_max = math::max(combined_max, float3(objects_[m].bbox_max));
        }

        /* For intersect/subtract: also include all ungrouped objects */
        if (op == SDF_CSG_INTERSECT || op == SDF_CSG_SUBTRACT || op == SDF_CSG_SHELL) {
          for (int i = 0; i < int(objects_.size()); i++) {
            if (objects_[i].group_id < 0) {
              combined_min = math::min(combined_min, float3(objects_[i].bbox_min));
              combined_max = math::max(combined_max, float3(objects_[i].bbox_max));
            }
          }
        }

        /* Expand by blend padding so members appear in blend-zone tiles */
        float grp_blend = (groups_gpu_[gi].blend_type > 0) ? groups_gpu_[gi].blend : 0.0f;
        if (op == SDF_CSG_SHELL) {
          grp_blend = std::max(groups_gpu_[gi].shell_blend_top,
                               groups_gpu_[gi].shell_blend_bottom);
        }
        if (grp_blend > 0.001f) {
          float bpad = grp_blend * 2.0f;
          combined_min -= float3(bpad);
          combined_max += float3(bpad);
        }

        for (int m = start; m < start + cnt; m++) {
          objects_[m].bbox_min = float4(combined_min, 0.0f);
          objects_[m].bbox_max = float4(combined_max, 0.0f);
        }
      }

      /* Max blend per group */
      for (int gi = 0; gi < int(groups_gpu_.size()); gi++) {
        float grp_blend = (groups_gpu_[gi].csg_operation == SDF_CSG_SHELL)
                              ? std::max(groups_gpu_[gi].shell_blend_top,
                                         groups_gpu_[gi].shell_blend_bottom)
                              : fabsf(groups_gpu_[gi].blend);
        float max_blend = grp_blend;
        int start = groups_gpu_[gi].first_object;
        int cnt = groups_gpu_[gi].object_count;
        for (int m = start; m < start + cnt; m++) {
          float b = (objects_[m].blend_type == 0) ? 0.0f : objects_[m].blend;
          max_blend = std::max(max_blend, b + fabsf(objects_[m].shell_distance));
        }
        for (int m = start; m < start + cnt; m++) {
          objects_[m].max_group_blend = max_blend;
        }
      }
      /* Per-object spatial threshold for ungrouped objects:
       * each object's threshold = max blend of any spatially overlapping neighbor.
       * Zero-blend objects far from blended neighbors get threshold≈0 (tight culling). */
      {
        const int N = int(objects_.size());
        for (int i = 0; i < N; i++) {
          if (objects_[i].group_id >= 0) { continue; }
          float own_b = (objects_[i].blend_type == 0) ? 0.0f : objects_[i].blend;
          own_b += fabsf(objects_[i].shell_distance);
          float thresh = own_b;

          for (int j = 0; j < N; j++) {
            if (j == i || objects_[j].group_id >= 0) { continue; }
            float jb = (objects_[j].blend_type == 0) ? 0.0f : objects_[j].blend;
            jb += fabsf(objects_[j].shell_distance);
            if (jb <= thresh) { continue; }

            float3 a_min = float3(objects_[i].orig_bbox_min) - float3(jb);
            float3 a_max = float3(objects_[i].orig_bbox_max) + float3(jb);
            float3 b_min = float3(objects_[j].orig_bbox_min);
            float3 b_max = float3(objects_[j].orig_bbox_max);
            if (a_min.x <= b_max.x && a_max.x >= b_min.x &&
                a_min.y <= b_max.y && a_max.y >= b_min.y &&
                a_min.z <= b_max.z && a_max.z >= b_min.z)
            {
              thresh = jb;
            }
          }
          objects_[i].max_group_blend = thresh;
        }
      }

      /* Expand ungrouped objects' AABBs and skip thresholds to cover group
       * extent + blend padding. Required for any group with blend so that
       * tiles/BVH include both the group and ungrouped objects in the blend zone. */
      for (int gi = 0; gi < int(groups_gpu_.size()); gi++) {
        int start = groups_gpu_[gi].first_object;
        int cnt = groups_gpu_[gi].object_count;

        /* Compute group's combined AABB */
        float3 grp_min(1e30f), grp_max(-1e30f);
        for (int m = start; m < start + cnt; m++) {
          grp_min = math::min(grp_min, float3(objects_[m].bbox_min));
          grp_max = math::max(grp_max, float3(objects_[m].bbox_max));
        }
        float grp_blend = (groups_gpu_[gi].csg_operation == SDF_CSG_SHELL)
                              ? std::max(groups_gpu_[gi].shell_blend_top,
                                         groups_gpu_[gi].shell_blend_bottom)
                              : groups_gpu_[gi].blend;
        float pad = grp_blend * 2.0f + 0.5f;
        grp_min -= float3(pad);
        grp_max += float3(pad);

        for (int i = 0; i < int(objects_.size()); i++) {
          if (objects_[i].group_id >= 0) {
            continue;
          }
          /* Expand ungrouped object's AABB to cover the group */
          objects_[i].bbox_min = float4(math::min(float3(objects_[i].bbox_min), grp_min), 0.0f);
          objects_[i].bbox_max = float4(math::max(float3(objects_[i].bbox_max), grp_max), 0.0f);

          /* Also expand skip threshold */
          float3 u_min = float3(objects_[i].orig_bbox_min);
          float3 u_max = float3(objects_[i].orig_bbox_max);
          float max_dist = 0.0f;
          for (int m = start; m < start + cnt; m++) {
            float3 g_min = float3(objects_[m].orig_bbox_min);
            float3 g_max = float3(objects_[m].orig_bbox_max);
            for (int c = 0; c < 8; c++) {
              float3 pt(c & 1 ? g_max.x : g_min.x,
                        c & 2 ? g_max.y : g_min.y,
                        c & 4 ? g_max.z : g_min.z);
              float3 closest = math::clamp(pt, u_min, u_max);
              max_dist = std::max(max_dist, math::length(pt - closest));
            }
          }
          max_dist += pad;
          objects_[i].max_group_blend = std::max(objects_[i].max_group_blend, max_dist);
        }
      }
    }

    /* Early frustum cull on pre-expansion AABBs with conservative padding */
    {
      float max_expansion = max_blend_ + max_shell_distance_;
      for (int g = 0; g < int(groups_gpu_.size()); g++) {
        float gb = (groups_gpu_[g].csg_operation == SDF_CSG_SHELL)
                       ? std::max(groups_gpu_[g].shell_blend_top,
                                  groups_gpu_[g].shell_blend_bottom)
                       : groups_gpu_[g].blend;
        max_expansion = std::max(max_expansion, gb + fabsf(groups_gpu_[g].shell_distance));
      }

      const View &view = View::default_get();
      float4x4 vp = view.winmat() * view.viewmat();
      float4 planes[6];
      for (int i = 0; i < 4; i++) {
        planes[0][i] = vp[i][3] + vp[i][0];
        planes[1][i] = vp[i][3] - vp[i][0];
        planes[2][i] = vp[i][3] + vp[i][1];
        planes[3][i] = vp[i][3] - vp[i][1];
        planes[4][i] = vp[i][3] + vp[i][2];
        planes[5][i] = vp[i][3] - vp[i][2];
      }
      for (int p = 0; p < 6; p++) {
        planes[p] /= math::length(float3(planes[p]));
      }

      float min_pad = std::max(max_expansion, 2.0f);
      for (int i = 0; i < int(objects_.size()); i++) {
        float3 bmin = float3(objects_[i].bbox_min) - float3(min_pad);
        float3 bmax = float3(objects_[i].bbox_max) + float3(min_pad);
        bool visible = true;
        for (int p = 0; p < 6; p++) {
          float3 n(planes[p]);
          float d = planes[p].w;
          float3 pos_vertex(n.x > 0 ? bmax.x : bmin.x,
                            n.y > 0 ? bmax.y : bmin.y,
                            n.z > 0 ? bmax.z : bmin.z);
          if (math::dot(n, pos_vertex) + d < 0.0f) {
            visible = false;
            break;
          }
        }
        objects_[i]._pad2 = visible ? 1 : 0;
      }

      /* Group propagation: if any member visible, keep entire group */
      for (int g = 0; g < int(groups_gpu_.size()); g++) {
        int start = groups_gpu_[g].first_object;
        int count = groups_gpu_[g].object_count;
        bool any_visible = false;
        for (int i = start; i < start + count; i++) {
          if (objects_[i]._pad2 != 0) {
            any_visible = true;
            break;
          }
        }
        if (any_visible) {
          for (int i = start; i < start + count; i++) {
            objects_[i]._pad2 = 1;
          }
        }
      }

      /* Compact visible objects */
      int visible_count = 0;
      for (int i = 0; i < int(objects_.size()); i++) {
        if (objects_[i]._pad2 != 0) {
          visible_count++;
        }
      }

      if (visible_count < int(objects_.size())) {
        Vector<SDFObjectGPU> compact_objects;
        Vector<SDFModifierGPU> compact_modifiers;
        Vector<SDFPolygonPointGPU> compact_polygon_points;
        Vector<int> obj_remap(objects_.size(), -1);
        compact_objects.reserve(visible_count);

        for (int i = 0; i < int(objects_.size()); i++) {
          if (objects_[i]._pad2 == 0) {
            continue;
          }
          obj_remap[i] = int(compact_objects.size());
          SDFObjectGPU obj = objects_[i];

          int new_mod_start = int(compact_modifiers.size());
          for (int m = 0; m < obj.modifier_count; m++) {
            compact_modifiers.append(modifiers_[obj.modifier_start + m]);
          }
          obj.modifier_start = new_mod_start;

          int new_poly_start = int(compact_polygon_points.size());
          for (int p = 0; p < obj.polygon_point_count; p++) {
            compact_polygon_points.append(polygon_points_[obj.polygon_point_start + p]);
          }
          obj.polygon_point_start = new_poly_start;

          compact_objects.append(obj);
        }

        Vector<SDFGroupGPU> compact_groups;
        Vector<int> group_remap(groups_gpu_.size(), -1);
        for (int g = 0; g < int(groups_gpu_.size()); g++) {
          int old_start = groups_gpu_[g].first_object;
          int old_count = groups_gpu_[g].object_count;
          int new_first = -1;
          int new_count = 0;
          for (int i = old_start; i < old_start + old_count; i++) {
            if (obj_remap[i] >= 0) {
              if (new_first < 0) {
                new_first = obj_remap[i];
              }
              new_count++;
            }
          }
          if (new_count > 0) {
            group_remap[g] = int(compact_groups.size());
            SDFGroupGPU grp = groups_gpu_[g];
            grp.first_object = new_first;
            grp.object_count = new_count;
            compact_groups.append(grp);
          }
        }

        for (auto &obj : compact_objects) {
          if (obj.group_id >= 0) {
            obj.group_id = group_remap[obj.group_id];
          }
        }

        /* Remap both index mappings through compaction */
        for (int &idx : s_depsgraph_to_sorted) {
          idx = (idx >= 0 && idx < int(obj_remap.size())) ? obj_remap[idx] : -1;
        }
        for (auto item : s_object_to_sorted.items()) {
          int old_si = item.value;
          s_object_to_sorted.lookup(item.key) =
              (old_si >= 0 && old_si < int(obj_remap.size())) ? obj_remap[old_si] : -1;
        }

        /* Compact sorted_object_ptrs */
        Vector<const Object *> compact_sorted_ptrs;
        compact_sorted_ptrs.reserve(visible_count);
        for (int i = 0; i < int(s_sorted_object_ptrs.size()); i++) {
          if (obj_remap[i] >= 0) {
            compact_sorted_ptrs.append(s_sorted_object_ptrs[i]);
          }
        }
        s_sorted_object_ptrs = std::move(compact_sorted_ptrs);

        objects_ = std::move(compact_objects);
        modifiers_ = std::move(compact_modifiers);
        polygon_points_ = std::move(compact_polygon_points);
        groups_gpu_ = std::move(compact_groups);
      }
    }

    if (objects_.is_empty()) {
      clear_exported_state();
      return;
    }

    /* Reassign original_index to sorted position (globally unique for gbuffer) */
    for (int i = 0; i < int(objects_.size()); i++) {
      objects_[i].original_index = i;
    }

    /* Expand AABBs for blend-aware CSG (now runs on visible set only) */
    Vector<float3> new_mins(objects_.size());
    Vector<float3> new_maxs(objects_.size());
    for (int i = 0; i < int(objects_.size()); i++) {
      float blend_radius = (objects_[i].blend_type == 0) ? 0.0f : objects_[i].blend;
      float exp = blend_radius + fabsf(objects_[i].shell_distance);
      new_mins[i] = float3(objects_[i].bbox_min) - float3(exp);
      new_maxs[i] = float3(objects_[i].bbox_max) + float3(exp);
    }

    int ungrouped_start = int(objects_.size());
    for (int i = 0; i < int(objects_.size()); i++) {
      if (objects_[i].group_id == -1) {
        ungrouped_start = i;
        break;
      }
    }

    for (int i = int(objects_.size()) - 1; i >= ungrouped_start; i--) {
      SDFObjectGPU &obj = objects_[i];
      for (int j = i + 1; j < int(objects_.size()); j++) {
        float sub_blend_radius = (objects_[j].blend_type == 0) ? 0.0f : objects_[j].blend;
        float sub_blend = sub_blend_radius + fabsf(objects_[j].shell_distance);
        if (sub_blend > 0.0f) {
          float3 obj_exp_min = float3(obj.bbox_min) - float3(sub_blend);
          float3 obj_exp_max = float3(obj.bbox_max) + float3(sub_blend);
          float3 sub_exp_min = new_mins[j];
          float3 sub_exp_max = new_maxs[j];
          float3 ix_min = math::max(obj_exp_min, sub_exp_min);
          float3 ix_max = math::min(obj_exp_max, sub_exp_max);
          if (ix_min.x <= ix_max.x && ix_min.y <= ix_max.y && ix_min.z <= ix_max.z) {
            new_mins[i] = math::min(new_mins[i], ix_min);
            new_maxs[i] = math::max(new_maxs[i], ix_max);
          }
        }
      }
    }

    Vector<float3> group_mins(groups_gpu_.size(), float3(1e10f));
    Vector<float3> group_maxs(groups_gpu_.size(), float3(-1e10f));

    for (int g = int(groups_gpu_.size()) - 1; g >= 0; g--) {
      SDFGroupGPU &grp = groups_gpu_[g];
      int start_idx = grp.first_object;
      int end_idx = grp.first_object + grp.object_count - 1;

      if (grp.object_count > 0 && start_idx >= 0 && end_idx < int(objects_.size())) {
        for (int i = end_idx; i >= start_idx; i--) {
          SDFObjectGPU &obj = objects_[i];

          for (int j = i + 1; j <= end_idx; j++) {
            float sub_blend_radius = (objects_[j].blend_type == 0) ? 0.0f : objects_[j].blend;
            float sub_blend = sub_blend_radius + fabsf(objects_[j].shell_distance);
            if (sub_blend > 0.0f) {
              float3 obj_exp_min = float3(obj.bbox_min) - float3(sub_blend);
              float3 obj_exp_max = float3(obj.bbox_max) + float3(sub_blend);
              float3 sub_exp_min = new_mins[j];
              float3 sub_exp_max = new_maxs[j];
              float3 ix_min = math::max(obj_exp_min, sub_exp_min);
              float3 ix_max = math::min(obj_exp_max, sub_exp_max);
              if (ix_min.x <= ix_max.x && ix_min.y <= ix_max.y && ix_min.z <= ix_max.z) {
                new_mins[i] = math::min(new_mins[i], ix_min);
                new_maxs[i] = math::max(new_maxs[i], ix_max);
              }
            }
          }

          for (int sub_g = g + 1; sub_g < int(groups_gpu_.size()); sub_g++) {
            float sg_blend = (groups_gpu_[sub_g].csg_operation == SDF_CSG_SHELL)
                                 ? std::max(groups_gpu_[sub_g].shell_blend_top,
                                            groups_gpu_[sub_g].shell_blend_bottom)
                                 : groups_gpu_[sub_g].blend;
            float sub_blend = sg_blend + fabsf(groups_gpu_[sub_g].shell_distance);
            if (sub_blend > 0.0f) {
              float3 obj_exp_min = float3(obj.bbox_min) - float3(sub_blend);
              float3 obj_exp_max = float3(obj.bbox_max) + float3(sub_blend);
              float3 sub_exp_min = group_mins[sub_g];
              float3 sub_exp_max = group_maxs[sub_g];
              float3 ix_min = math::max(obj_exp_min, sub_exp_min);
              float3 ix_max = math::min(obj_exp_max, sub_exp_max);
              if (ix_min.x <= ix_max.x && ix_min.y <= ix_max.y && ix_min.z <= ix_max.z) {
                new_mins[i] = math::min(new_mins[i], ix_min);
                new_maxs[i] = math::max(new_maxs[i], ix_max);
              }
            }
          }

          for (int j = ungrouped_start; j < int(objects_.size()); j++) {
            float sub_blend_radius = (objects_[j].blend_type == 0) ? 0.0f : objects_[j].blend;
            float sub_blend = sub_blend_radius + fabsf(objects_[j].shell_distance);
            if (sub_blend > 0.0f) {
              float3 obj_exp_min = float3(obj.bbox_min) - float3(sub_blend);
              float3 obj_exp_max = float3(obj.bbox_max) + float3(sub_blend);
              float3 sub_exp_min = new_mins[j];
              float3 sub_exp_max = new_maxs[j];
              float3 ix_min = math::max(obj_exp_min, sub_exp_min);
              float3 ix_max = math::min(obj_exp_max, sub_exp_max);
              if (ix_min.x <= ix_max.x && ix_min.y <= ix_max.y && ix_min.z <= ix_max.z) {
                new_mins[i] = math::min(new_mins[i], ix_min);
                new_maxs[i] = math::max(new_maxs[i], ix_max);
              }
            }
          }

          group_mins[g] = math::min(group_mins[g], new_mins[i]);
          group_maxs[g] = math::max(group_maxs[g], new_maxs[i]);
        }
      }
    }

    for (int i = 0; i < int(objects_.size()); i++) {
      objects_[i].bbox_min = float4(new_mins[i], 0.0f);
      objects_[i].bbox_max = float4(new_maxs[i], 0.0f);
    }

    /* Fine frustum cull on expanded AABBs (before intersection expansion) */
    {
      const View &view = View::default_get();
      float4x4 vp = view.winmat() * view.viewmat();
      float4 planes[6];
      for (int i = 0; i < 4; i++) {
        planes[0][i] = vp[i][3] + vp[i][0];
        planes[1][i] = vp[i][3] - vp[i][0];
        planes[2][i] = vp[i][3] + vp[i][1];
        planes[3][i] = vp[i][3] - vp[i][1];
        planes[4][i] = vp[i][3] + vp[i][2];
        planes[5][i] = vp[i][3] - vp[i][2];
      }
      for (int p = 0; p < 6; p++) {
        planes[p] /= math::length(float3(planes[p]));
      }
      for (int i = 0; i < int(objects_.size()); i++) {
        float3 bmin = float3(objects_[i].bbox_min);
        float3 bmax = float3(objects_[i].bbox_max);
        bool visible = true;
        for (int p = 0; p < 6; p++) {
          float3 n(planes[p]);
          float d = planes[p].w;
          float3 pos_vertex(n.x > 0 ? bmax.x : bmin.x,
                            n.y > 0 ? bmax.y : bmin.y,
                            n.z > 0 ? bmax.z : bmin.z);
          if (math::dot(n, pos_vertex) + d < 0.0f) {
            visible = false;
            break;
          }
        }
        objects_[i]._pad2 = visible ? 1 : 0;
      }
    }

    /* Intersection/Push: expand to visible scene bounds.
     * Runs after frustum cull so only visible objects contribute. */
    {
      float3 smin = float3(1e30f), smax = float3(-1e30f);
      for (int i = 0; i < int(objects_.size()); i++) {
        if (objects_[i]._pad2 == 0) { continue; }
        smin = math::min(smin, float3(objects_[i].bbox_min));
        smax = math::max(smax, float3(objects_[i].bbox_max));
      }
      for (int i = 0; i < int(objects_.size()); i++) {
        if (objects_[i].csg_operation == SDF_CSG_INTERSECT ||
            objects_[i].csg_operation == SDF_CSG_PUSH)
        {
          objects_[i].bbox_min = float4(smin, 0.0f);
          objects_[i].bbox_max = float4(smax, 0.0f);
          objects_[i]._pad2 = 1;
        }
      }
      for (int g = 0; g < int(groups_gpu_.size()); g++) {
        int grp_op = groups_gpu_[g].csg_operation;
        int start = groups_gpu_[g].first_object;
        int count = groups_gpu_[g].object_count;
        bool has_push = false;
        for (int i = start; i < start + count; i++) {
          has_push |= objects_[i].csg_operation == SDF_CSG_PUSH;
        }
        if (grp_op == SDF_CSG_INTERSECT || grp_op == SDF_CSG_PUSH || has_push) {
          for (int i = start; i < start + count; i++) {
            objects_[i].bbox_min = float4(smin, 0.0f);
            objects_[i].bbox_max = float4(smax, 0.0f);
            objects_[i]._pad2 = 1;
          }
        }
      }

      /* Preserve CSG field dependencies. */
      for (int i = 0; i < int(objects_.size()); i++) {
        int op = objects_[i].csg_operation;
        if (op != SDF_CSG_PUSH && op != SDF_CSG_AVOID && op != SDF_CSG_SHELL) {
          continue;
        }

        float3 target_min = float3(objects_[i].bbox_min);
        float3 target_max = float3(objects_[i].bbox_max);
        for (int j = 0; j < i; j++) {
          float3 source_min = float3(objects_[j].orig_bbox_min);
          float3 source_max = float3(objects_[j].orig_bbox_max);
          float max_dist = 0.0f;
          for (int c = 0; c < 8; c++) {
            float3 point(c & 1 ? target_max.x : target_min.x,
                         c & 2 ? target_max.y : target_min.y,
                         c & 4 ? target_max.z : target_min.z);
            float3 closest = math::clamp(point, source_min, source_max);
            max_dist = std::max(max_dist, math::length(point - closest));
          }
          objects_[j].bbox_min = float4(math::min(float3(objects_[j].bbox_min), target_min),
                                        0.0f);
          objects_[j].bbox_max = float4(math::max(float3(objects_[j].bbox_max), target_max),
                                        0.0f);
          objects_[j].max_group_blend = std::max(objects_[j].max_group_blend, max_dist);
        }
      }
    }

    /* Recompute scene AABB and update the persistent BVH. */
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
    const int object_count = int(objects_.size());
    bool rebuild_bvh = bvh_object_ptrs_.size() != object_count;
    if (!rebuild_bvh) {
      for (int i = 0; i < object_count; i++) {
        if (bvh_object_ptrs_[i] != s_sorted_object_ptrs[i]) {
          rebuild_bvh = true;
          break;
        }
      }
    }
    if (rebuild_bvh) {
      bvh_tree_.clear();
      bvh_object_ptrs_.clear();
      bvh_proxies_.clear();
      bvh_object_ptrs_.reserve(object_count);
      bvh_proxies_.reserve(object_count);
    }
    for (int i = 0; i < object_count; i++) {
      scene_min_ = math::min(scene_min_, float3(objects_[i].bbox_min));
      scene_max_ = math::max(scene_max_, float3(objects_[i].bbox_max));

      SdfAabb bounds;
      bounds.min = float3(objects_[i].bbox_min);
      bounds.max = float3(objects_[i].bbox_max);
      if (rebuild_bvh) {
        bvh_object_ptrs_.append(s_sorted_object_ptrs[i]);
        bvh_proxies_.append(bvh_tree_.create_proxy(bounds, i));
      }
      else {
        bvh_tree_.update_proxy(bvh_proxies_[i], bounds);
      }
    }

    /* Hot AABB buffer (must be after compaction + AABB expansion) */
    {
      const int n = int(objects_.size());
      object_aabbs_.resize(n);
      for (int i = 0; i < n; i++) {
        SDFObjectAABB &a = object_aabbs_[i];
        a.bbox_min = objects_[i].bbox_min;
        a.bbox_max = objects_[i].bbox_max;
        a.group_id = objects_[i].group_id;
        a.max_group_blend = objects_[i].max_group_blend;
        a._pad0 = 0;
        a._pad1 = 0;
      }
    }

    if (engine_mode_ == 1) {
      lp_build_tree();
    }

    needs_upload_ = true;

  }

  /* -------------------------------------------------------------------- */
  /** \name Lipschitz pruning: CSG tree build, buffers, passes
   * \{ */

  static uint32_t lp_pack_binary_op(float k, int op)
  {
    uint32_t sign = (op == SDF_LP_OP_UNION) ? 1u : 0u;
    uint32_t k_bits;
    memcpy(&k_bits, &k, sizeof(float));
    k_bits &= ~7u; /* low 3 bits reused for op + sign */
    return k_bits | (uint32_t(op & 3) << 1) | sign;
  }

  static int lp_map_csg_op(int csg_operation)
  {
    switch (csg_operation) {
      case SDF_CSG_SUBTRACT:
        return SDF_LP_OP_SUB;
      case SDF_CSG_INTERSECT:
        return SDF_LP_OP_INTER;
      default:
        return SDF_LP_OP_UNION;
    }
  }

  static bool lp_object_supported(const SDFObjectGPU &obj)
  {
    if (obj.modifier_count > 0) {
      return false;
    }
    switch (obj.sdf_type) {
      case SDF_GPU_TYPE_BOX:
      case SDF_GPU_TYPE_SPHERE:
      case SDF_GPU_TYPE_CYLINDER:
      case SDF_GPU_TYPE_CONE:
        return true;
      default:
        return false;
    }
  }

  /* Build the post-order CSG tree (flat fold of the sorted object list, groups
   * folded as subtrees) from objects_/groups_gpu_. Only basic analytic
   * primitives participate; unsupported objects are skipped. */
  void lp_build_tree()
  {
    lp_nodes_.clear();
    lp_prims_.clear();
    lp_binary_ops_.clear();
    lp_parents_init_.clear();
    lp_active_init_.clear();

    const int n = int(objects_.size());
    if (n == 0) {
      return;
    }

    struct BuildNode {
      int type;
      int prim_idx;
      int op;
      float blend;
      int left, right;
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
      lp_prims_.append(prim);
      build.append({SDF_LP_NODETYPE_PRIMITIVE, int(lp_prims_.size()) - 1, 0, 0.0f, -1, -1});
      return int(build.size()) - 1;
    };

    auto make_op = [&](int op, float blend, int left, int right) -> int {
      build.append({SDF_LP_NODETYPE_BINARY, -1, op, blend, left, right});
      return int(build.size()) - 1;
    };

    auto obj_smooth_k = [&](int i) -> float {
      return (objects_[i].blend_type == SDF_BLEND_SMOOTH) ? objects_[i].blend : 0.0f;
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
        acc = make_op(lp_map_csg_op(objects_[i].csg_operation), obj_smooth_k(i), acc, operand);
      }
      return acc;
    };

    /* Scene fold: groups as subtrees, ungrouped objects as leaves. */
    int root = -1;
    int i = 0;
    while (i < n) {
      const int gi = objects_[i].group_id;
      int operand;
      int op;
      float k;
      int consumed;
      if (gi >= 0 && gi < int(groups_gpu_.size()) && i == groups_gpu_[gi].first_object &&
          groups_gpu_[gi].object_count > 0)
      {
        const SDFGroupGPU &grp = groups_gpu_[gi];
        operand = fold_leaves(i, i + grp.object_count);
        op = lp_map_csg_op(grp.csg_operation);
        k = (grp.blend_type == SDF_BLEND_SMOOTH) ? grp.blend : 0.0f;
        consumed = grp.object_count;
      }
      else {
        operand = make_leaf(i);
        op = lp_map_csg_op(objects_[i].csg_operation);
        k = obj_smooth_k(i);
        consumed = 1;
      }

      if (operand >= 0) {
        if (root < 0) {
          root = operand;
        }
        else {
          root = make_op(op, k, root, operand);
        }
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
    lp_parents_init_.resize(build.size());
    lp_nodes_.reserve(build.size());
    lp_active_init_.reserve(build.size());

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
        stack_sign.append(build[bidx].op == SDF_LP_OP_SUB);
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
        lp_binary_ops_.append(lp_pack_binary_op(bn.blend, bn.op));
        lp_parents_init_[gpu_of_build[bn.left]] = uint32_t(self_idx);
        lp_parents_init_[gpu_of_build[bn.right]] = uint32_t(self_idx);
      }
      else {
        gpu_node.idx_in_type = bn.prim_idx;
      }
      lp_nodes_.append(gpu_node);
      lp_parents_init_[self_idx] = SDF_LP_INVALID_INDEX;
      lp_active_init_.append(uint32_t(self_idx) | (sign ? SDF_LP_SIGN_BIT : 0u));
      gpu_of_build[bidx] = self_idx;
    }

    /* The pruning shader packs parent indices into 16 bits of scratch state. */
    if (lp_nodes_.size() > 65535) {
      lp_nodes_.clear();
      lp_prims_.clear();
      lp_binary_ops_.clear();
      lp_parents_init_.clear();
      lp_active_init_.clear();
    }
  }

  void lp_upload_tree()
  {
    SDFLpNode dummy_node = {};
    SDFLpPrimitive dummy_prim = {};
    uint32_t dummy_u = 0;

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
        num_ops * sizeof(uint32_t),
        lp_binary_ops_.is_empty() ? &dummy_u : lp_binary_ops_.data(),
        GPU_USAGE_DYNAMIC, "sdf_lp_binary_ops");
    if (lp_parents_init_ssbo_) GPU_storagebuf_free(lp_parents_init_ssbo_);
    lp_parents_init_ssbo_ = GPU_storagebuf_create_ex(
        num_nodes * sizeof(uint32_t),
        lp_parents_init_.is_empty() ? &dummy_u : lp_parents_init_.data(),
        GPU_USAGE_DYNAMIC, "sdf_lp_parents_init");
    if (lp_active_init_ssbo_) GPU_storagebuf_free(lp_active_init_ssbo_);
    lp_active_init_ssbo_ = GPU_storagebuf_create_ex(
        num_nodes * sizeof(uint32_t),
        lp_active_init_.is_empty() ? &dummy_u : lp_active_init_.data(),
        GPU_USAGE_DYNAMIC, "sdf_lp_active_init");
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
     * to the capacity as a safety net. */
    int64_t active_entries = math::max(num_cells * 2, int64_t(1) << 21);
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

    if (lp_active_ssbo_[0] != nullptr && gs <= lp_grid_size_ &&
        active_entries <= lp_active_capacity_ && tmp_entries <= lp_tmp_capacity_)
    {
      return;
    }

    const int new_gs = math::max(gs, lp_grid_size_);
    const int64_t new_cells = int64_t(new_gs) * new_gs * new_gs;
    active_entries = math::max(active_entries, lp_active_capacity_);
    tmp_entries = math::max(tmp_entries, lp_tmp_capacity_);

    for (int p = 0; p < 2; p++) {
      if (lp_active_ssbo_[p]) GPU_storagebuf_free(lp_active_ssbo_[p]);
      if (lp_parents_ssbo_[p]) GPU_storagebuf_free(lp_parents_ssbo_[p]);
      if (lp_num_active_ssbo_[p]) GPU_storagebuf_free(lp_num_active_ssbo_[p]);
      if (lp_cell_offset_ssbo_[p]) GPU_storagebuf_free(lp_cell_offset_ssbo_[p]);
      if (lp_cell_value_ssbo_[p]) GPU_storagebuf_free(lp_cell_value_ssbo_[p]);
      lp_active_ssbo_[p] = GPU_storagebuf_create_ex(
          active_entries * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_active");
      lp_parents_ssbo_[p] = GPU_storagebuf_create_ex(
          active_entries * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_parents");
      lp_num_active_ssbo_[p] = GPU_storagebuf_create_ex(
          new_cells * sizeof(int32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_num_active");
      lp_cell_offset_ssbo_[p] = GPU_storagebuf_create_ex(
          new_cells * sizeof(int32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_cell_offset");
      lp_cell_value_ssbo_[p] = GPU_storagebuf_create_ex(
          new_cells * sizeof(float), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_cell_value");
    }
    if (lp_tmp_ssbo_) GPU_storagebuf_free(lp_tmp_ssbo_);
    lp_tmp_ssbo_ = GPU_storagebuf_create_ex(
        tmp_entries * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_tmp");
    if (lp_scratch_ssbo_) GPU_storagebuf_free(lp_scratch_ssbo_);
    lp_scratch_ssbo_ = GPU_storagebuf_create_ex(
        tmp_entries * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_scratch");

    if (lp_active_count_ssbo_ == nullptr) {
      lp_active_count_ssbo_ = GPU_storagebuf_create_ex(
          16 * sizeof(int32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_active_count");
    }
    if (lp_tmp_count_ssbo_ == nullptr) {
      lp_tmp_count_ssbo_ = GPU_storagebuf_create_ex(
          16 * sizeof(int32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_lp_tmp_count");
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
    GPU_storagebuf_clear_to_zero(lp_tmp_count_ssbo_);

    int in_idx = 0;
    int out_idx = 1;

    GPU_shader_bind(sh);
    for (int lvl = 2; lvl <= lp_grid_level_; lvl += 2) {
      const int cur_gs = 1 << lvl;
      const bool first = (lvl == 2);

      lp_bind_ssbo(sh, "lp_prims", lp_prims_ssbo_);
      lp_bind_ssbo(sh, "lp_nodes", lp_nodes_ssbo_);
      lp_bind_ssbo(sh, "lp_binary_ops", lp_binary_ops_ssbo_);
      lp_bind_ssbo(sh, "lp_parents_in", first ? lp_parents_init_ssbo_ : lp_parents_ssbo_[in_idx]);
      lp_bind_ssbo(sh, "lp_parents_out", lp_parents_ssbo_[out_idx]);
      lp_bind_ssbo(sh, "lp_active_nodes", first ? lp_active_init_ssbo_ : lp_active_ssbo_[in_idx]);
      lp_bind_ssbo(sh, "lp_active_out", lp_active_ssbo_[out_idx]);
      lp_bind_ssbo(sh, "lp_parent_cells_offset", lp_cell_offset_ssbo_[in_idx]);
      lp_bind_ssbo(sh, "lp_child_cells_offset", lp_cell_offset_ssbo_[out_idx]);
      lp_bind_ssbo(sh, "lp_parent_num_active", lp_num_active_ssbo_[in_idx]);
      lp_bind_ssbo(sh, "lp_num_active_out", lp_num_active_ssbo_[out_idx]);
      lp_bind_ssbo(sh, "lp_active_count", lp_active_count_ssbo_);
      lp_bind_ssbo(sh, "lp_cell_value_in", lp_cell_value_ssbo_[in_idx]);
      lp_bind_ssbo(sh, "lp_cell_value_out", lp_cell_value_ssbo_[out_idx]);
      lp_bind_ssbo(sh, "lp_tmp", lp_tmp_ssbo_);
      lp_bind_ssbo(sh, "lp_scratch", lp_scratch_ssbo_);
      lp_bind_ssbo(sh, "lp_tmp_count", lp_tmp_count_ssbo_);

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
  }

  void draw_lp_trace()
  {
    gpu::Shader *sh = lp_trace_sh();
    if (sh == nullptr) {
      return;
    }

    lp_ensure_grid_buffers(lp_enable_pruning_ ? lp_grid_level_ : 2);
    const bool culling = lp_enable_pruning_ && lp_grid_valid_ && !lp_nodes_.is_empty();

    GPU_shader_bind(sh);

    lp_bind_ssbo(sh, "lp_prims", lp_prims_ssbo_);
    lp_bind_ssbo(sh, "lp_nodes", lp_nodes_ssbo_);
    lp_bind_ssbo(sh, "lp_binary_ops", lp_binary_ops_ssbo_);
    lp_bind_ssbo(sh, "lp_active_nodes", culling ? lp_active_ssbo_[lp_final_idx_] : lp_active_init_ssbo_);
    lp_bind_ssbo(sh, "lp_cells_offset", lp_cell_offset_ssbo_[lp_final_idx_]);
    lp_bind_ssbo(sh, "lp_cells_num_active", lp_num_active_ssbo_[lp_final_idx_]);
    lp_bind_ssbo(sh, "lp_cell_value", lp_cell_value_ssbo_[lp_final_idx_]);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));

    GPU_shader_uniform_3fv(sh, "aabb_min", lp_aabb_min_);
    GPU_shader_uniform_3fv(sh, "aabb_max", lp_aabb_max_);
    GPU_shader_uniform_1i(sh, "grid_size", culling ? (1 << lp_grid_level_) : 1);
    GPU_shader_uniform_1i(sh, "total_num_nodes", int(lp_nodes_.size()));
    GPU_shader_uniform_1i(sh, "culling_enabled", culling ? 1 : 0);
    GPU_shader_uniform_1i(sh, "shading_mode", lp_shading_mode_);
    GPU_shader_uniform_1f(sh, "viz_max", float(lp_colormap_max_));
    GPU_shader_uniform_1i(sh, "max_steps", sdf_max_steps_);
    GPU_shader_uniform_1f(sh, "ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    const int dispatch_x = (render_size_.x + 7) / 8;
    const int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_texture_image_unbind(gbuf_normal_tx_);
    GPU_shader_unbind();
  }

  /** \} */

  void draw(Manager & /*manager*/) final
  {
    if (objects_.is_empty()) {
      /* Clear stale output so previous frame's SDF doesn't persist */
      if (comp_color_tx_) {
        float clear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        GPU_texture_clear(comp_color_tx_, GPU_DATA_FLOAT, clear);
      }
      if (comp_depth_tx_) {
        float clear[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        GPU_texture_clear(comp_depth_tx_, GPU_DATA_FLOAT, clear);
      }
      compute_valid_ = false;
      clear_exported_state();
      return;
    }

    ensure_shaders();
    if (trace_comp_sh() == nullptr) {
      clear_exported_state();
      return;
    }

    sync_shading();

    /* Detect shading changes (matcap, lighting, studiolight) */
    bool shading_changed = false;
    {
      uint64_t sh = 0xcbf29ce484222325ULL;
      auto hash_shading = [&](const void *data, size_t size) {
        const uint8_t *p = static_cast<const uint8_t *>(data);
        for (size_t i = 0; i < size; i++) {
          sh ^= p[i];
          sh *= 0x100000001b3ULL;
        }
      };
      hash_shading(&lighting_type_, sizeof(lighting_type_));
      hash_shading(&use_specular_, sizeof(use_specular_));
      hash_shading(&use_matcap_flip_, sizeof(use_matcap_flip_));
      hash_shading(studio_light_dir_, sizeof(studio_light_dir_));
      hash_shading(studio_light_col_, sizeof(studio_light_col_));
      hash_shading(studio_light_spec_, sizeof(studio_light_spec_));
      hash_shading(&studio_ambient_, sizeof(studio_ambient_));
      hash_shading(current_matcap_.data(), current_matcap_.size());
      hash_shading(&debug_bvh_views_, sizeof(debug_bvh_views_));
      hash_shading(&use_cone_trace_, sizeof(use_cone_trace_));
      hash_shading(&sdf_max_steps_, sizeof(sdf_max_steps_));
      hash_shading(&sdf_ray_epsilon_, sizeof(sdf_ray_epsilon_));
      hash_shading(&sdf_over_relaxation_, sizeof(sdf_over_relaxation_));
      hash_shading(&sdf_cone_aperture_, sizeof(sdf_cone_aperture_));
      hash_shading(&sdf_cone_steps_, sizeof(sdf_cone_steps_));
      hash_shading(&use_frustum_cull_, sizeof(use_frustum_cull_));
      hash_shading(&fxaa_enabled_, sizeof(fxaa_enabled_));
      hash_shading(&engine_mode_, sizeof(engine_mode_));
      hash_shading(&lp_enable_pruning_, sizeof(lp_enable_pruning_));
      hash_shading(&lp_recompute_pruning_, sizeof(lp_recompute_pruning_));
      hash_shading(&lp_shading_mode_, sizeof(lp_shading_mode_));
      hash_shading(&lp_grid_level_, sizeof(lp_grid_level_));
      hash_shading(&lp_colormap_max_, sizeof(lp_colormap_max_));
      hash_shading(&lp_aabb_auto_, sizeof(lp_aabb_auto_));
      hash_shading(lp_aabb_min_user_, sizeof(lp_aabb_min_user_));
      hash_shading(lp_aabb_max_user_, sizeof(lp_aabb_max_user_));
      shading_changed = (sh != prev_shading_hash_);
      prev_shading_hash_ = sh;
    }

    DRW_submission_start();

    /* Detect scene and view changes for adaptive resolution */
    {
      uint64_t h = 0xcbf29ce484222325ULL;
      auto hash_bytes = [&](const void *data, size_t size) {
        const uint8_t *p = static_cast<const uint8_t *>(data);
        for (size_t i = 0; i < size; i++) {
          h ^= p[i];
          h *= 0x100000001b3ULL;
        }
      };
      hash_bytes(objects_.data(), objects_.size() * sizeof(SDFObjectGPU));
      hash_bytes(modifiers_.data(), modifiers_.size() * sizeof(SDFModifierGPU));
      hash_bytes(groups_gpu_.data(), groups_gpu_.size() * sizeof(SDFGroupGPU));
      hash_bytes(
          polygon_points_.data(), polygon_points_.size() * sizeof(SDFPolygonPointGPU));
      scene_changed_ = (h != prev_data_hash_);
      prev_data_hash_ = h;

      uint64_t mh = 0xcbf29ce484222325ULL;
      auto hash_mesh = [&](const void *data, size_t size) {
        const uint8_t *p = static_cast<const uint8_t *>(data);
        for (size_t i = 0; i < size; i++) {
          mh ^= p[i];
          mh *= 0x100000001b3ULL;
        }
      };
      hash_mesh(mesh_transforms_.data(), mesh_transforms_.size() * sizeof(float4x4));
      mesh_changed_ = (mh != prev_mesh_hash_);
      bool mesh_changed = mesh_changed_;
      prev_mesh_hash_ = mh;

      uint64_t mesh_data_hash = 0xcbf29ce484222325ULL;
      const uint8_t *mesh_bytes = reinterpret_cast<const uint8_t *>(mesh_gpu_data_.data());
      for (size_t i = 0; i < mesh_gpu_data_.size() * sizeof(uint4); i++) {
        mesh_data_hash ^= mesh_bytes[i];
        mesh_data_hash *= 0x100000001b3ULL;
      }
      mesh_data_changed_ = mesh_data_hash != prev_mesh_data_hash_;
      prev_mesh_data_hash_ = mesh_data_hash;

      const View &view = View::default_get();
      const float4x4 &cur_viewmat = view.viewmat();
      const float4x4 &cur_winmat = view.winmat();
      {
        const float eps = 1e-6f;
        bool vdiff = false;
        for (int c = 0; c < 4 && !vdiff; c++) {
          for (int r = 0; r < 4 && !vdiff; r++) {
            if (math::abs(cur_viewmat[c][r] - prev_viewmat_[c][r]) > eps ||
                math::abs(cur_winmat[c][r] - prev_winmat_[c][r]) > eps)
            {
              vdiff = true;
            }
          }
        }
        view_changed_ = vdiff;
      }
      prev_viewmat_ = cur_viewmat;
      prev_winmat_ = cur_winmat;

      if (scene_changed_ || view_changed_ || mesh_changed || shading_changed) {
        scroll_cooldown_ = 3;
        idle_frames_ = 0;
      }
      else if (scroll_cooldown_ > 0) {
        scroll_cooldown_--;
        idle_frames_ = 0;
      }
      else {
        idle_frames_++;
      }
    }

    bool profiling = s_profile_pending;

    if (profiling) {
      s_profile_result = {};
      s_profile_result.render_width = render_size_.x;
      s_profile_result.render_height = render_size_.y;
      s_profile_result.resolution_scale = resolution_scale_;
      s_profile_result.object_count = int(objects_.size());
      s_profile_result.group_count = int(groups_gpu_.size());
      s_profile_result.modifier_total = int(modifiers_.size());
      s_profile_result.bvh_node_count = int(bvh_tree_.nodes().size());
      collect_profile_objects();

      needs_upload_ = true;
      compute_valid_ = false;
    }

    ensure_profile_buffers(profiling);

    if (profiling) {
      s_profiler.begin();
    }

#define PROF_START(name) if (profiling) { s_profiler.mark_start(name); }
#define PROF_END()       if (profiling) { s_profiler.mark_end(); }

    if (needs_upload_ && (scene_changed_ || mesh_changed_ || !compute_valid_)) {
      PROF_START("Upload");
      GPU_debug_group_begin("SDF Upload");
      upload_objects();
      GPU_debug_group_end();
      PROF_END();
      needs_upload_ = false;
    }

    ensure_compute_targets();

    /* Lipschitz pruning: resolve the effective grid AABB and flag the pruning
     * grid dirty when the scene, grid level, or AABB inputs changed. */
    if (engine_mode_ == 1) {
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
          scene_changed_)
      {
        lp_grid_dirty_ = true;
        lp_aabb_min_ = want_min;
        lp_aabb_max_ = want_max;
      }
    }

    bool res_changed = (render_size_ != prev_render_size_);
    bool force_compute = (G.debug & G_DEBUG_GPU_SDF) != 0;
    bool need_compute = force_compute || !compute_valid_ || scene_changed_ || view_changed_ ||
                         res_changed || shading_changed;

    if (need_compute) {
      if (engine_mode_ == 1) {
        /* Lipschitz pruning path: build/refresh the pruning grid, then march
         * against the per-cell active node lists. Shading reuses the classic
         * shade pass (lighting forced off for debug modes in draw_shade). */
        if (lp_enable_pruning_ && !lp_nodes_.is_empty() &&
            (lp_recompute_pruning_ || lp_grid_dirty_ || !lp_grid_valid_))
        {
          PROF_START("LP Prune");
          GPU_debug_group_begin("SDF LP Prune");
          lp_ensure_grid_buffers(lp_grid_level_);
          draw_lp_prune();
          GPU_debug_group_end();
          GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
          PROF_END();
          lp_grid_dirty_ = false;
        }

        PROF_START("LP Trace");
        GPU_debug_group_begin("SDF LP Trace");
        draw_lp_trace();
        GPU_debug_group_end();
        GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
        PROF_END();

        PROF_START("Shade");
        GPU_debug_group_begin("SDF Shade");
        draw_shade();
        GPU_debug_group_end();
        GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
        PROF_END();

        compute_valid_ = true;
      }
      else {
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

        PROF_START("Cone March");
        GPU_debug_group_begin("SDF Cone March");
        draw_cone_march();
        GPU_debug_group_end();
        GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
        PROF_END();

        PROF_START("Trace");
        GPU_debug_group_begin("SDF Trace");
        draw_trace();
        GPU_debug_group_end();
        GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
        PROF_END();

        PROF_START("Color Resolve");
        GPU_debug_group_begin("SDF Color Resolve");
        draw_color_resolve();
        GPU_debug_group_end();
        GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
        PROF_END();

        PROF_START("Normal");
        GPU_debug_group_begin("SDF Normal");
        draw_normal();
        GPU_debug_group_end();
        GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
        PROF_END();

        PROF_START("Shade");
        GPU_debug_group_begin("SDF Shade");
        draw_shade();
        GPU_debug_group_end();
        GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
        PROF_END();

        compute_valid_ = true;
      }
    }

    PROF_START("Blit");
    GPU_debug_group_begin("SDF Blit");
    draw_blit();
    GPU_debug_group_end();
    PROF_END();

    PROF_START("FXAA");
    GPU_debug_group_begin("SDF FXAA");
    draw_fxaa();
    GPU_debug_group_end();
    PROF_END();

#undef PROF_START
#undef PROF_END

    if (profiling) {
      s_profiler.finish(s_profile_result);

      /* Read back eval counts + trace stats */
      GPU_memory_barrier(GPU_BARRIER_BUFFER_UPDATE);
      if (prof_eval_ssbo_ && prof_eval_ssbo_count_ > 0) {
        Vector<uint32_t> eval_counts(prof_eval_ssbo_count_);
        GPU_storagebuf_read(prof_eval_ssbo_, eval_counts.data());
        std::copy_n(eval_counts.data(),
                    math::min(prof_eval_ssbo_count_, SDF_PROFILE_MAX_OBJECTS),
                    s_profile_result.eval_counts);
      }
      if (prof_stats_ssbo_) {
        GPU_storagebuf_read(prof_stats_ssbo_, &s_profile_result.trace_stats);
      }

      s_profile_result.valid = true;
      s_profile_pending = false;
    }

    prev_render_size_ = render_size_;

    DRW_submission_end();

    if ((G.debug & G_DEBUG_GPU_SDF) ||
        (adaptive_resolution_ && render_size_ != texture_size_))
    {
      DRW_viewport_request_redraw();
    }

    /* Update static globals for overlay */
    s_object_count = int(objects_.size());
    s_object_ssbo = object_ssbo_;
    s_modifier_ssbo = modifier_ssbo_;
    s_polygon_ssbo = polygon_ssbo_;
    s_group_ssbo = group_ssbo_;
    s_bvh_ssbo = bvh_nodes_ssbo_;
    s_mesh_data_ssbo = mesh_data_ssbo_;
    s_bvh_root = bvh_tree_.root();
    s_depth_tx = comp_depth_tx_;
    s_gbuf_color_tx = gbuf_color_tx_;
    s_render_size = render_size_;
    s_texture_size = texture_size_;
    s_viewport_key = draw_ctx_->region;
    s_objects_cpu = objects_.data();
    s_objects_cpu_count = int(objects_.size());
    s_polygon_pts_cpu = polygon_points_.data();
    s_polygon_pts_count = int(polygon_points_.size());
    s_modifiers_cpu = modifiers_.data();
    s_modifier_count = int(modifiers_.size());
    s_group_count = int(groups_gpu_.size());
  }

 private:
  void collect_profile_objects()
  {
    s_profile_result.profiled_object_count = 0;
    for (int i = 0; i < int(objects_.size()) && i < SDF_PROFILE_MAX_OBJECTS; i++) {
      auto &dst = s_profile_result.objects[i];
      const SDFObjectGPU &src = objects_[i];
      dst.sdf_type = src.sdf_type;
      dst.blend_type = src.blend_type;
      dst.csg_operation = src.csg_operation;
      dst.blend = src.blend;
      dst.bevel = src.bevel;
      dst.size[0] = src.sdf_size.x;
      dst.size[1] = src.sdf_size.y;
      dst.size[2] = src.sdf_size.z;

      if (i < int(object_ptrs_.size()) && object_ptrs_[i]) {
        BLI_strncpy(dst.name, object_ptrs_[i]->id.name + 2, sizeof(dst.name));
      }
      else {
        BLI_snprintf(dst.name, sizeof(dst.name), "Object_%d", i);
      }

      dst.modifier_count = std::min(src.modifier_count, SDF_PROFILE_MAX_MODS_PER_OBJ);
      for (int m = 0; m < dst.modifier_count; m++) {
        int mi = src.modifier_start + m;
        if (mi >= 0 && mi < int(modifiers_.size())) {
          dst.modifiers[m].type = modifiers_[mi].header.x;
          dst.modifiers[m].flags = modifiers_[mi].header.y;
        }
      }
      s_profile_result.profiled_object_count++;
    }
  }

  void sync_sdf_settings()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      return;
    }

    const View3DShading &s = v3d->shading;

    use_bvh_ = 1;
    /* Values 1-5 are GPU debug views, 6+ are overlay-only. */
    debug_bvh_views_ = (s.sdf_bvh_debug_view <= 5) ? s.sdf_bvh_debug_view : 0;
    debug_fd_normals_ = s.sdf_fd_normals ? 1 : 0;
    use_cone_trace_ = s.sdf_use_cone_trace ? 1 : 0;
    sdf_max_steps_ = s.sdf_max_steps > 0 ? s.sdf_max_steps : 128;
    sdf_ray_epsilon_ = s.sdf_ray_epsilon > 0.0f ? s.sdf_ray_epsilon : 0.005f;
    ui_sdf_max_steps_ = sdf_max_steps_;
    ui_sdf_ray_epsilon_ = sdf_ray_epsilon_;
    sdf_over_relaxation_ = s.sdf_over_relaxation >= 1.0f ? s.sdf_over_relaxation : 1.3f;
    sdf_cone_aperture_ = s.sdf_cone_aperture > 0.0f ? s.sdf_cone_aperture : 0.5f;
    sdf_cone_steps_ = s.sdf_cone_steps > 0 ? s.sdf_cone_steps : 32;

    float scale_pct = s.sdf_resolution_scale;
    resolution_scale_ = (scale_pct >= 20.0f) ? scale_pct / 100.0f : 1.0f;
    adaptive_resolution_ = s.sdf_adaptive_resolution != 0;
    adaptive_precision_ = s.sdf_adaptive_precision != 0;
    use_frustum_cull_ = s.sdf_frustum_cull != 0;

    /* Lipschitz pruning engine settings. */
    engine_mode_ = (s.sdf_engine_mode == 1) ? 1 : 0;
    lp_enable_pruning_ = s.sdf_lp_enable_pruning != 0;
    lp_recompute_pruning_ = s.sdf_lp_recompute_pruning != 0;
    lp_shading_mode_ = s.sdf_lp_shading_mode;
    lp_grid_level_ = s.sdf_lp_grid_level >= 2 ? s.sdf_lp_grid_level : 6;
    lp_colormap_max_ = s.sdf_lp_colormap_max > 0 ? s.sdf_lp_colormap_max : 25;
    lp_aabb_auto_ = s.sdf_lp_aabb_auto != 0;
    lp_aabb_min_user_ = float3(s.sdf_lp_aabb_min);
    lp_aabb_max_user_ = float3(s.sdf_lp_aabb_max);

    bool new_fxaa = (U.sdf_fxaa != 0);
    if (!new_fxaa && fxaa_enabled_) {
      if (march_color_tx_) {
        GPU_texture_free(march_color_tx_);
        march_color_tx_ = nullptr;
      }
      if (march_fb_) {
        GPU_framebuffer_free(march_fb_);
        march_fb_ = nullptr;
      }
      fxaa_size_ = int2(0);
    }
    fxaa_enabled_ = new_fxaa;
  }

  void sync_shading()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      lighting_type_ = V3D_LIGHTING_FLAT;
      use_specular_ = 0;
      use_matcap_flip_ = 0;
      return;
    }

    const View3DShading &shading = v3d->shading;
    lighting_type_ = int(shading.light);
    use_specular_ = 0;

    if (lighting_type_ == V3D_LIGHTING_FLAT) {
      return;
    }

    if (lighting_type_ == V3D_LIGHTING_STUDIO) {
      StudioLight *sl = BKE_studiolight_find(shading.studio_light, STUDIOLIGHT_TYPE_STUDIO);
      if (sl == nullptr) {
        sl = BKE_studiolight_find_default(STUDIOLIGHT_TYPE_STUDIO);
      }

      float4x4 world_shading_rotation = float4x4::identity();
      if (shading.flag & V3D_SHADING_WORLD_ORIENTATION) {
        float4x4 V = blender::draw::View::default_get().viewmat();
        float R[4][4];
        axis_angle_to_mat4_single(R, 'Z', -shading.studiolight_rot_z);
        mul_m4_m4m4(R, V.ptr(), R);
        swap_v3_v3(R[2], R[1]);
        negate_v3(R[2]);
        world_shading_rotation = float4x4(R);
      }

      if (sl != nullptr) {
        use_specular_ = ((shading.flag & V3D_SHADING_SPECULAR_HIGHLIGHT) &&
                         (sl->flag & STUDIOLIGHT_SPECULAR_HIGHLIGHT_PASS)) ?
                            1 :
                            0;
        for (int i = 0; i < 4; i++) {
          const SolidLight &light = sl->light[i];
          if (light.flag) {
            float3 dir = math::transform_direction(world_shading_rotation,
                                                   float3(light.vec));
            studio_light_dir_[i] = float4(dir, 0.0f);
            studio_light_col_[i] = float4(light.col[0], light.col[1], light.col[2], light.smooth);
            studio_light_spec_[i] = float4(light.spec[0], light.spec[1], light.spec[2], 0.0f);
          }
          else {
            studio_light_dir_[i] = float4(0.0f);
            studio_light_col_[i] = float4(0.0f);
            studio_light_spec_[i] = float4(0.0f);
          }
        }
        studio_ambient_ = float3(sl->light_ambient[0], sl->light_ambient[1], sl->light_ambient[2]);
      }
      else {
        float3 fallback_dir = math::transform_direction(
            world_shading_rotation, math::normalize(float3(0.5f, 0.7f, 1.0f)));
        studio_light_dir_[0] = float4(fallback_dir, 0.0f);
        studio_light_col_[0] = float4(0.8f, 0.8f, 0.8f, 0.0f);
        studio_light_spec_[0] = float4(1.0f, 1.0f, 1.0f, 0.0f);
        for (int i = 1; i < 4; i++) {
          studio_light_dir_[i] = float4(0.0f);
          studio_light_col_[i] = float4(0.0f);
          studio_light_spec_[i] = float4(0.0f);
        }
        studio_ambient_ = float3(0.15f);
      }
    }
    else {
      StudioLight *sl = BKE_studiolight_find(shading.matcap, STUDIOLIGHT_TYPE_MATCAP);
      if (sl == nullptr) {
        sl = BKE_studiolight_find_default(STUDIOLIGHT_TYPE_MATCAP);
      }
      if (sl == nullptr) {
        lighting_type_ = V3D_LIGHTING_FLAT;
        return;
      }

      use_specular_ = ((shading.flag & V3D_SHADING_SPECULAR_HIGHLIGHT) &&
                       (sl->flag & STUDIOLIGHT_SPECULAR_HIGHLIGHT_PASS)) ?
                          1 :
                          0;
      use_matcap_flip_ = (shading.flag & V3D_SHADING_MATCAP_FLIP_X) ? 1 : 0;

      if (std::string(sl->name) != current_matcap_) {
        BKE_studiolight_ensure_flag(
            sl, STUDIOLIGHT_MATCAP_DIFFUSE_GPUTEXTURE | STUDIOLIGHT_MATCAP_SPECULAR_GPUTEXTURE);

        ImBuf *diffuse_ibuf = sl->matcap_diffuse.ibuf;
        ImBuf *specular_ibuf = sl->matcap_specular.ibuf;

        if (diffuse_ibuf && diffuse_ibuf->float_buffer.data) {
          int w = diffuse_ibuf->x;
          int h = diffuse_ibuf->y;
          int pixel_count = w * h * 4;
          int layers = 1;
          float *buffer = diffuse_ibuf->float_buffer.data;
          Vector<float> combined;

          if (specular_ibuf && specular_ibuf->float_buffer.data) {
            combined.extend(diffuse_ibuf->float_buffer.data, pixel_count);
            combined.extend(specular_ibuf->float_buffer.data, pixel_count);
            buffer = combined.begin();
            layers = 2;
          }

          if (matcap_tx_) {
            GPU_texture_free(matcap_tx_);
          }

          matcap_tx_ = GPU_texture_create_2d_array("sdf_matcap",
                                                   w,
                                                   h,
                                                   layers,
                                                   1,
                                                   gpu::TextureFormat::SFLOAT_16_16_16_16,
                                                   GPU_TEXTURE_USAGE_SHADER_READ,
                                                   buffer);
          if (matcap_tx_) {
            GPU_texture_filter_mode(matcap_tx_, true);
          }
        }

        current_matcap_ = sl->name;
      }

      if (matcap_tx_ == nullptr) {
        float white[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        matcap_tx_ = GPU_texture_create_2d_array("sdf_matcap_fallback",
                                                 1,
                                                 1,
                                                 1,
                                                 1,
                                                 gpu::TextureFormat::SFLOAT_16_16_16_16,
                                                 GPU_TEXTURE_USAGE_SHADER_READ,
                                                 white);
      }
    }

    /* Always ensure a valid matcap texture exists — the shader declares the
     * sampler unconditionally, so even flat/studio lighting needs a dummy. */
    if (matcap_tx_ == nullptr) {
      float white[4] = {1.0f, 1.0f, 1.0f, 1.0f};
      matcap_tx_ = GPU_texture_create_2d_array("sdf_matcap_fallback",
                                                1,
                                                1,
                                                1,
                                                1,
                                                gpu::TextureFormat::SFLOAT_16_16_16_16,
                                                GPU_TEXTURE_USAGE_SHADER_READ,
                                                white);
    }
  }

  void ensure_shaders()
  {
    sdf_shaders_ensure();
  }

  void ensure_profile_buffers(bool clear)
  {
    const int obj_count = int(objects_.size());
    const int n = math::max(obj_count, 1);

    if (prof_eval_ssbo_ && prof_eval_ssbo_count_ != obj_count) {
      GPU_storagebuf_free(prof_eval_ssbo_);
      prof_eval_ssbo_ = nullptr;
    }

    Vector<uint32_t> zeros(n, 0);
    const size_t buf_sz = n * sizeof(uint32_t);
    if (prof_eval_ssbo_ == nullptr) {
      prof_eval_ssbo_ = GPU_storagebuf_create_ex(
          buf_sz, zeros.data(), GPU_USAGE_DYNAMIC, "sdf_prof_eval_counts");
      prof_eval_ssbo_count_ = obj_count;
    }
    else if (clear) {
      GPU_storagebuf_update(prof_eval_ssbo_, zeros.data());
    }

    SdfTraceStats zero_stats = {};
    if (prof_stats_ssbo_ == nullptr) {
      prof_stats_ssbo_ = GPU_storagebuf_create_ex(
          sizeof(SdfTraceStats), &zero_stats, GPU_USAGE_DYNAMIC, "sdf_prof_trace_stats");
    }
    else if (clear) {
      GPU_storagebuf_update(prof_stats_ssbo_, &zero_stats);
    }
  }

  void upload_objects()
  {
    const int count = int(objects_.size());
    const size_t buf_size = count * sizeof(SDFObjectGPU);

    if (object_ssbo_ != nullptr && object_ssbo_count_ != count) {
      GPU_storagebuf_free(object_ssbo_);
      object_ssbo_ = nullptr;
    }

    if (object_ssbo_ == nullptr) {
      object_ssbo_ = GPU_storagebuf_create_ex(
          buf_size, objects_.data(), GPU_USAGE_DYNAMIC, "sdf_objects_ssbo");
      object_ssbo_count_ = count;
    }
    else {
      GPU_storagebuf_update(object_ssbo_, objects_.data());
    }

    /* AABB hot buffer */
    const size_t aabb_buf_size = count * sizeof(SDFObjectAABB);
    if (object_aabb_ssbo_ != nullptr && object_aabb_ssbo_count_ != count) {
      GPU_storagebuf_free(object_aabb_ssbo_);
      object_aabb_ssbo_ = nullptr;
    }
    if (object_aabb_ssbo_ == nullptr) {
      object_aabb_ssbo_ = GPU_storagebuf_create_ex(
          aabb_buf_size, object_aabbs_.data(), GPU_USAGE_DYNAMIC, "sdf_object_aabbs_ssbo");
      object_aabb_ssbo_count_ = count;
    }
    else {
      GPU_storagebuf_update(object_aabb_ssbo_, object_aabbs_.data());
    }

    const int mod_count = math::max(int(modifiers_.size()), 1);
    const size_t mod_buf_size = mod_count * sizeof(SDFModifierGPU);

    if (modifier_ssbo_ != nullptr && modifier_ssbo_count_ != mod_count) {
      GPU_storagebuf_free(modifier_ssbo_);
      modifier_ssbo_ = nullptr;
    }

    if (modifier_ssbo_ == nullptr) {
      if (modifiers_.is_empty()) {
        SDFModifierGPU dummy = {};
        modifier_ssbo_ = GPU_storagebuf_create_ex(
            mod_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_modifiers_ssbo");
      }
      else {
        modifier_ssbo_ = GPU_storagebuf_create_ex(
            mod_buf_size, modifiers_.data(), GPU_USAGE_DYNAMIC, "sdf_modifiers_ssbo");
      }
      modifier_ssbo_count_ = mod_count;
    }
    else {
      if (!modifiers_.is_empty()) {
        GPU_storagebuf_update(modifier_ssbo_, modifiers_.data());
      }
    }

    /* Polygon points SSBO */
    const int poly_count = math::max(int(polygon_points_.size()), 1);
    const size_t poly_buf_size = poly_count * sizeof(SDFPolygonPointGPU);

    if (polygon_ssbo_ != nullptr && polygon_ssbo_count_ != poly_count) {
      GPU_storagebuf_free(polygon_ssbo_);
      polygon_ssbo_ = nullptr;
    }

    if (polygon_ssbo_ == nullptr) {
      if (polygon_points_.is_empty()) {
        SDFPolygonPointGPU dummy = {};
        polygon_ssbo_ = GPU_storagebuf_create_ex(
            poly_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_polygon_points_ssbo");
      }
      else {
        polygon_ssbo_ = GPU_storagebuf_create_ex(
            poly_buf_size, polygon_points_.data(), GPU_USAGE_DYNAMIC, "sdf_polygon_points_ssbo");
      }
      polygon_ssbo_count_ = poly_count;
    }
    else {
      if (!polygon_points_.is_empty()) {
        GPU_storagebuf_update(polygon_ssbo_, polygon_points_.data());
      }
    }

    const int mesh_data_count = math::max(int(mesh_gpu_data_.size()), 1);
    if (mesh_data_ssbo_ != nullptr && mesh_data_ssbo_count_ != mesh_data_count) {
      GPU_storagebuf_free(mesh_data_ssbo_);
      mesh_data_ssbo_ = nullptr;
    }
    if (mesh_data_ssbo_ == nullptr) {
      const uint4 dummy(0);
      mesh_data_ssbo_ = GPU_storagebuf_create_ex(mesh_data_count * sizeof(uint4),
                                                 mesh_gpu_data_.is_empty() ? &dummy :
                                                                             mesh_gpu_data_.data(),
                                                 GPU_USAGE_DYNAMIC,
                                                 "sdf_mesh_data_ssbo");
      mesh_data_ssbo_count_ = mesh_data_count;
    }
    else if (mesh_data_changed_ && !mesh_gpu_data_.is_empty()) {
      GPU_storagebuf_update(mesh_data_ssbo_, mesh_gpu_data_.data());
    }

    const int grp_count = math::max(int(groups_gpu_.size()), 1);
    const size_t grp_buf_size = grp_count * sizeof(SDFGroupGPU);

    if (group_ssbo_ != nullptr && group_ssbo_count_ != grp_count) {
      GPU_storagebuf_free(group_ssbo_);
      group_ssbo_ = nullptr;
    }

    if (group_ssbo_ == nullptr) {
      if (groups_gpu_.is_empty()) {
        SDFGroupGPU dummy = {};
        group_ssbo_ = GPU_storagebuf_create_ex(
            grp_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_groups_ssbo");
      }
      else {
        group_ssbo_ = GPU_storagebuf_create_ex(
            grp_buf_size, groups_gpu_.data(), GPU_USAGE_DYNAMIC, "sdf_groups_ssbo");
      }
      group_ssbo_count_ = grp_count;
    }
    else {
      if (!groups_gpu_.is_empty()) {
        GPU_storagebuf_update(group_ssbo_, groups_gpu_.data());
      }
    }

    Vector<SdfAabbNodeGPU> gpu_nodes = bvh_tree_.build_gpu_nodes(0.0f);
    const int bvh_count = math::max(int(gpu_nodes.size()), 1);
    const size_t bvh_buf_size = bvh_count * sizeof(SdfAabbNodeGPU);

    if (bvh_nodes_ssbo_ != nullptr && bvh_nodes_ssbo_count_ != bvh_count) {
      GPU_storagebuf_free(bvh_nodes_ssbo_);
      bvh_nodes_ssbo_ = nullptr;
    }

    if (bvh_nodes_ssbo_ == nullptr) {
      if (gpu_nodes.is_empty()) {
        SdfAabbNodeGPU dummy = {};
        bvh_nodes_ssbo_ = GPU_storagebuf_create_ex(
            bvh_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_bvh_ssbo");
      }
      else {
        bvh_nodes_ssbo_ = GPU_storagebuf_create_ex(
            bvh_buf_size, gpu_nodes.data(), GPU_USAGE_DYNAMIC, "sdf_bvh_ssbo");
      }
      bvh_nodes_ssbo_count_ = bvh_count;
    }
    else {
      if (!gpu_nodes.is_empty()) {
        GPU_storagebuf_update(bvh_nodes_ssbo_, gpu_nodes.data());
      }
    }

    if (engine_mode_ == 1) {
      lp_upload_tree();
    }
  }

  void ensure_compute_targets()
  {
    const int2 vp = int2(draw_ctx_->viewport_size_get());
    viewport_size_ = vp;

    /* Stay at low res until idle for 2+ frames (avoids GPU stall on re-grab) */
    float scale = resolution_scale_;
    bool is_interacting = scroll_cooldown_ > 0 || idle_frames_ < 2;
    bool adaptive_lowres = adaptive_resolution_ && is_interacting;
    if (adaptive_lowres) {
      scale *= 0.25f;
      scale = math::max(scale, 0.1f);
    }
    render_size_ = int2(math::max(int(vp.x * scale), 1),
                        math::max(int(vp.y * scale), 1));

    /* Coarsen the marcher while rendering at quarter-res: precision doesn't
     * matter when pixels are 4x as large, and the relaxed step/epsilon combo
     * converges much faster (helps frame rate while navigating). When we
     * drop back to full-res, restore the UI-cached values. sync_sdf_settings
     * runs every frame and clobbers these, so re-apply unconditionally. Only
     * relax when adaptive_precision_ is on; otherwise use UI precision. */
    if (adaptive_lowres && adaptive_precision_) {
      sdf_max_steps_ = 128;
      sdf_ray_epsilon_ = 0.01f;
    }
    else {
      sdf_max_steps_ = ui_sdf_max_steps_;
      sdf_ray_epsilon_ = ui_sdf_ray_epsilon_;
    }
    adaptive_lowres_active_ = adaptive_lowres;

    /* Textures sized to full viewport — only reallocate on viewport resize */
    int2 tex_size = int2(math::max(int(vp.x * resolution_scale_), 1),
                         math::max(int(vp.y * resolution_scale_), 1));
    if (comp_color_tx_ != nullptr && texture_size_ == tex_size) {
      return;
    }
    compute_valid_ = false;

    if (comp_color_tx_) {
      GPU_texture_free(comp_color_tx_);
    }
    if (comp_depth_tx_) {
      GPU_texture_free(comp_depth_tx_);
    }
    if (gbuf_pos_tx_) {
      GPU_texture_free(gbuf_pos_tx_);
    }
    if (gbuf_color_tx_) {
      GPU_texture_free(gbuf_color_tx_);
    }
    if (gbuf_normal_tx_) {
      GPU_texture_free(gbuf_normal_tx_);
    }
    texture_size_ = tex_size;

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE |
                             GPU_TEXTURE_USAGE_ATTACHMENT;
    eGPUTextureUsage usage_readback = usage | GPU_TEXTURE_USAGE_HOST_READ;
    comp_color_tx_ = GPU_texture_create_2d("sdf_comp_color", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);
    comp_depth_tx_ = GPU_texture_create_2d("sdf_comp_depth", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_32, usage_readback, nullptr);
    gbuf_pos_tx_ = GPU_texture_create_2d("sdf_gbuf_pos", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_32_32_32_32, usage, nullptr);
    gbuf_color_tx_ = GPU_texture_create_2d("sdf_gbuf_color", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage_readback, nullptr);
    gbuf_normal_tx_ = GPU_texture_create_2d("sdf_gbuf_normal", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);
  }

  void bind_ssbos(gpu::Shader *sh)
  {
    if (object_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "objects");
      if (slot >= 0) {
        GPU_storagebuf_bind(object_ssbo_, slot);
      }
    }
    if (modifier_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "sdf_modifiers");
      if (slot >= 0) {
        GPU_storagebuf_bind(modifier_ssbo_, slot);
      }
    }
    if (polygon_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "polygon_points");
      if (slot >= 0) {
        GPU_storagebuf_bind(polygon_ssbo_, slot);
      }
    }
    if (group_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "groups");
      if (slot >= 0) {
        GPU_storagebuf_bind(group_ssbo_, slot);
      }
    }
    if (bvh_nodes_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "aabb_nodes");
      if (slot >= 0) {
        GPU_storagebuf_bind(bvh_nodes_ssbo_, slot);
      }
    }
    if (object_aabb_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "object_aabbs");
      if (slot >= 0) {
        GPU_storagebuf_bind(object_aabb_ssbo_, slot);
      }
    }
    if (mesh_data_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "mesh_data_buf");
      if (slot >= 0) {
        GPU_storagebuf_bind(mesh_data_ssbo_, slot);
      }
    }
  }

  static constexpr int kMaxTileObjects = 256;

  void ensure_screen_aabbs_ssbo(int obj_count)
  {
    if (screen_aabbs_ssbo_ != nullptr && screen_aabbs_ssbo_count_ != obj_count) {
      GPU_storagebuf_free(screen_aabbs_ssbo_);
      screen_aabbs_ssbo_ = nullptr;
    }
    if (screen_aabbs_ssbo_ == nullptr && obj_count > 0) {
      screen_aabbs_ssbo_ = GPU_storagebuf_create_ex(
          obj_count * sizeof(int[4]), nullptr, GPU_USAGE_DYNAMIC, "sdf_screen_aabbs");
      screen_aabbs_ssbo_count_ = obj_count;
    }
  }

  void ensure_tile_ssbos(int total_tiles)
  {
    /* Grow-only: allocate at peak tile count to avoid thrashing on resolution transitions */
    if (total_tiles <= tile_prim_counts_ssbo_tiles_) {
      return;
    }

    if (tile_prim_counts_ssbo_) {
      GPU_storagebuf_free(tile_prim_counts_ssbo_);
    }
    if (tile_prim_lists_ssbo_) {
      GPU_storagebuf_free(tile_prim_lists_ssbo_);
    }
    if (cone_hit_ssbo_) {
      GPU_storagebuf_free(cone_hit_ssbo_);
    }
    if (tile_far_hint_ssbo_) {
      GPU_storagebuf_free(tile_far_hint_ssbo_);
    }

    tile_prim_counts_ssbo_ = GPU_storagebuf_create_ex(
        total_tiles * sizeof(int), nullptr, GPU_USAGE_DYNAMIC, "sdf_tile_prim_counts");
    tile_prim_lists_ssbo_ = GPU_storagebuf_create_ex(
        total_tiles * kMaxTileObjects * sizeof(int),
        nullptr,
        GPU_USAGE_DYNAMIC,
        "sdf_tile_prim_lists");
    cone_hit_ssbo_ = GPU_storagebuf_create_ex(
        total_tiles * sizeof(float[4]), nullptr, GPU_USAGE_DYNAMIC, "sdf_cone_hit_ssbo");
    tile_far_hint_ssbo_ = GPU_storagebuf_create_ex(
        total_tiles * sizeof(float), nullptr, GPU_USAGE_DYNAMIC, "sdf_tile_far_hint");

    tile_prim_counts_ssbo_tiles_ = total_tiles;
  }

  void draw_aabb_project()
  {
    if (!aabb_project_sh() || objects_.is_empty()) {
      return;
    }
    int obj_count = int(objects_.size());
    ensure_screen_aabbs_ssbo(obj_count);
    gpu::Shader *sh = aabb_project_sh();
    GPU_shader_bind(sh);
    if (object_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "objects");
      if (slot >= 0) GPU_storagebuf_bind(object_ssbo_, slot);
    }
    if (screen_aabbs_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "screen_aabbs");
      if (slot >= 0) GPU_storagebuf_bind(screen_aabbs_ssbo_, slot);
    }
    GPU_shader_uniform_1i(sh, "object_count", obj_count);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);
    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);
    GPU_compute_dispatch(sh, (obj_count + 63) / 64, 1, 1);
    GPU_shader_unbind();
  }

  void draw_tile_cull()
  {
    if (!tile_cull_sh() || use_bvh_ == 0) {
      return;
    }
    ensure_compute_targets();
    int tiles_x = (render_size_.x + 7) / 8;
    int tiles_y = (render_size_.y + 7) / 8;
    int total_tiles = tiles_x * tiles_y;
    ensure_tile_ssbos(total_tiles);

    gpu::Shader *sh = tile_cull_sh();
    GPU_shader_bind(sh);
    if (object_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "objects");
      if (slot >= 0) GPU_storagebuf_bind(object_ssbo_, slot);
    }
    if (screen_aabbs_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "screen_aabbs");
      if (slot >= 0) GPU_storagebuf_bind(screen_aabbs_ssbo_, slot);
    }
    if (tile_prim_counts_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_counts");
      if (slot >= 0) GPU_storagebuf_bind(tile_prim_counts_ssbo_, slot);
    }
    if (tile_prim_lists_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_lists");
      if (slot >= 0) GPU_storagebuf_bind(tile_prim_lists_ssbo_, slot);
    }
    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);
    GPU_compute_dispatch(sh, tiles_x, tiles_y, 1);
    GPU_shader_unbind();
  }

  void draw_cone_march()
  {
    if (use_cone_trace_ == 0 || !cone_march_sh()) {
      return;
    }

    int tiles_x = (render_size_.x + 7) / 8;
    int tiles_y = (render_size_.y + 7) / 8;

    gpu::Shader *sh = cone_march_sh();
    GPU_shader_bind(sh);
    bind_ssbos(sh);

    if (object_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "objects");
      if (slot >= 0) GPU_storagebuf_bind(object_ssbo_, slot);
    }
    if (modifier_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "sdf_modifiers");
      if (slot >= 0) GPU_storagebuf_bind(modifier_ssbo_, slot);
    }
    if (polygon_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "polygon_points");
      if (slot >= 0) GPU_storagebuf_bind(polygon_ssbo_, slot);
    }
    if (group_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "groups");
      if (slot >= 0) GPU_storagebuf_bind(group_ssbo_, slot);
    }
    if (object_aabb_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "object_aabbs");
      if (slot >= 0) GPU_storagebuf_bind(object_aabb_ssbo_, slot);
    }
    if (cone_hit_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_hit_pos");
      if (slot >= 0) GPU_storagebuf_bind(cone_hit_ssbo_, slot);
    }
    if (tile_prim_counts_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_counts");
      if (slot >= 0) GPU_storagebuf_bind(tile_prim_counts_ssbo_, slot);
    }
    if (tile_prim_lists_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_lists");
      if (slot >= 0) GPU_storagebuf_bind(tile_prim_lists_ssbo_, slot);
    }
    if (tile_far_hint_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_far_hint");
      if (slot >= 0) GPU_storagebuf_bind(tile_far_hint_ssbo_, slot);
    }

    if (bvh_nodes_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "aabb_nodes");
      if (slot >= 0) GPU_storagebuf_bind(bvh_nodes_ssbo_, slot);
    }

    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(sh, "group_count", int(groups_gpu_.size()));
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_1i(sh, "use_bvh", use_bvh_);
    GPU_shader_uniform_1i(sh, "bvh_root", bvh_tree_.root());
    GPU_shader_uniform_3fv(sh, "scene_aabb_min", scene_min_);
    GPU_shader_uniform_3fv(sh, "scene_aabb_max", scene_max_);
    GPU_shader_uniform_1f(sh, "sdf_cone_aperture", sdf_cone_aperture_);
    GPU_shader_uniform_1i(sh, "sdf_cone_steps", sdf_cone_steps_);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    int dispatch_x = (tiles_x + 7) / 8;
    int dispatch_y = (tiles_y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);
    GPU_shader_unbind();
  }

  void draw_trace()
  {
    ensure_compute_targets();

    gpu::Shader *sh = (use_bvh_ != 0 && trace_tile_sh()) ? trace_tile_sh() : trace_comp_sh();
    if (!sh) {
      return;
    }

    GPU_shader_bind(sh);
    bind_ssbos(sh);

    /* Profiling SSBOs + uniforms */
    if (prof_eval_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "prof_eval_counts");
      if (slot >= 0) GPU_storagebuf_bind(prof_eval_ssbo_, slot);
    }
    if (prof_stats_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "prof_trace_stats");
      if (slot >= 0) GPU_storagebuf_bind(prof_stats_ssbo_, slot);
    }
    GPU_shader_uniform_1i(sh, "prof_enabled", s_profile_pending ? 1 : 0);
    GPU_shader_uniform_1i(sh, "skip_object", -1);

    /* Bind tile SSBOs (tile-culled path) */
    if (cone_hit_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_hit_pos");
      if (slot >= 0) {
        GPU_storagebuf_bind(cone_hit_ssbo_, slot);
      }
    }
    if (tile_prim_counts_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_counts");
      if (slot >= 0) {
        GPU_storagebuf_bind(tile_prim_counts_ssbo_, slot);
      }
    }
    if (tile_prim_lists_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_lists");
      if (slot >= 0) {
        GPU_storagebuf_bind(tile_prim_lists_ssbo_, slot);
      }
    }
    if (tile_far_hint_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_far_hint");
      if (slot >= 0) {
        GPU_storagebuf_bind(tile_far_hint_ssbo_, slot);
      }
    }

    /* Bind images: out_color/out_depth for debug views, G-buffer for hit data */
    GPU_texture_image_bind(comp_color_tx_, GPU_shader_get_sampler_binding(sh, "out_color_img"));
    GPU_texture_image_bind(comp_depth_tx_, GPU_shader_get_sampler_binding(sh, "out_depth_img"));
    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));

    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(sh, "group_count", int(groups_gpu_.size()));
    GPU_shader_uniform_1i(sh, "use_bvh", use_bvh_);
    GPU_shader_uniform_1i(sh, "use_cone_trace", use_cone_trace_);
    GPU_shader_uniform_1i(sh, "debug_bvh_views", debug_bvh_views_);
    GPU_shader_uniform_1i(sh, "sdf_max_steps", sdf_max_steps_);
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_1f(sh, "sdf_over_relaxation", sdf_over_relaxation_);
    GPU_shader_uniform_1f(sh, "sdf_step_factor", step_factor_);
    GPU_shader_uniform_1i(sh, "bvh_root", bvh_tree_.root());
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);
    GPU_shader_uniform_3fv(sh, "scene_aabb_min", scene_min_);
    GPU_shader_uniform_3fv(sh, "scene_aabb_max", scene_max_);

    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    int dispatch_x = (render_size_.x + 7) / 8;
    int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(comp_color_tx_);
    GPU_texture_image_unbind(comp_depth_tx_);
    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_shader_unbind();
  }

  void draw_color_resolve()
  {
    if (debug_bvh_views_ != 0) {
      return;
    }
    gpu::Shader *sh = color_resolve_sh();
    if (!sh) {
      return;
    }
    GPU_shader_bind(sh);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));

    bind_ssbos(sh);

    if (tile_prim_counts_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_counts");
      if (slot >= 0) GPU_storagebuf_bind(tile_prim_counts_ssbo_, slot);
    }
    if (tile_prim_lists_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_lists");
      if (slot >= 0) GPU_storagebuf_bind(tile_prim_lists_ssbo_, slot);
    }

    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(sh, "group_count", int(groups_gpu_.size()));
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_1i(sh, "use_bvh", use_bvh_);
    GPU_shader_uniform_1i(sh, "bvh_root", bvh_tree_.root());
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    int dispatch_x = (render_size_.x + 7) / 8;
    int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_texture_image_unbind(gbuf_normal_tx_);
    GPU_shader_unbind();
  }

  void draw_normal()
  {
    if (debug_bvh_views_ != 0) {
      return;
    }
    gpu::Shader *sh = normal_comp_sh();
    if (!sh) {
      return;
    }
    GPU_shader_bind(sh);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));

    bind_ssbos(sh);

    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    int dispatch_x = (render_size_.x + 7) / 8;
    int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_normal_tx_);
    GPU_shader_unbind();
  }

  void draw_shade()
  {
    if (debug_bvh_views_ != 0) {
      return;
    }

    gpu::Shader *sh = shade_comp_sh();
    if (!sh) {
      return;
    }

    GPU_shader_bind(sh);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));
    GPU_texture_image_bind(comp_color_tx_, GPU_shader_get_sampler_binding(sh, "out_color_img"));
    GPU_texture_image_bind(comp_depth_tx_, GPU_shader_get_sampler_binding(sh, "out_depth_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));

    int matcap_slot = GPU_shader_get_sampler_binding(sh, "matcap_tx");
    GPU_texture_bind(matcap_tx_, matcap_slot);

    /* LP debug modes (heatmap/normals) bake the final color into gbuf_color;
     * force the shade pass to pass-through so lighting is not applied twice. */
    const int effective_lighting = (engine_mode_ == 1 && lp_shading_mode_ != 0) ? 0 :
                                                                              lighting_type_;
    GPU_shader_uniform_1i(sh, "lighting_type", effective_lighting);
    GPU_shader_uniform_1i(sh, "use_specular", use_specular_);
    GPU_shader_uniform_1i(sh, "use_matcap_flip", use_matcap_flip_);
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    SDFShadingDataGPU shading_data;
    for (int i = 0; i < 4; i++) {
      shading_data.studio_light[i] = studio_light_dir_[i];
      shading_data.studio_color[i] = studio_light_col_[i];
      shading_data.studio_spec[i] = studio_light_spec_[i];
    }
    shading_data.studio_ambient = float4(studio_ambient_, 0.0f);

    if (shading_ubo_ == nullptr) {
      shading_ubo_ = GPU_uniformbuf_create_ex(
          sizeof(SDFShadingDataGPU), nullptr, "sdf_shading_data");
    }
    GPU_uniformbuf_update(shading_ubo_, &shading_data);
    GPU_uniformbuf_bind(shading_ubo_, 1);

    View &view = View::default_get();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    int dispatch_x = (render_size_.x + 7) / 8;
    int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_texture_image_unbind(comp_color_tx_);
    GPU_texture_image_unbind(comp_depth_tx_);
    GPU_texture_image_unbind(gbuf_normal_tx_);
    GPU_texture_unbind(matcap_tx_);
    GPU_shader_unbind();
  }

  void draw_blit()
  {
    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();
    if (draw_ctx_->is_depth() || (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER)) {
      GPU_framebuffer_bind(dfbl->depth_only_fb);
    }
    else if (fxaa_enabled_) {
      ensure_fxaa_target();
      GPU_framebuffer_bind(march_fb_);
      float clear_color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      GPU_framebuffer_clear_color(march_fb_, clear_color);
    }
    else {
      GPU_framebuffer_bind(dfbl->default_fb);
    }

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    GPU_shader_bind(blit_sh());
    GPU_shader_uniform_1i(blit_sh(), "debug_bvh_views", debug_bvh_views_);

    float bg[3] = {0.0f, 0.0f, 0.0f};
    if (draw_ctx_->scene && draw_ctx_->v3d) {
      ED_view3d_background_color_get(draw_ctx_->scene, draw_ctx_->v3d, bg);
    }
    GPU_shader_uniform_3fv(blit_sh(), "bg_color", bg);

    float uv_sc[2] = {float(render_size_.x) / float(texture_size_.x),
                       float(render_size_.y) / float(texture_size_.y)};
    GPU_shader_uniform_2fv(blit_sh(), "uv_scale", uv_sc);

    int color_slot = GPU_shader_get_sampler_binding(blit_sh(), "color_tx");
    GPU_texture_filter_mode(comp_color_tx_, true);
    GPU_texture_bind(comp_color_tx_, color_slot);
    int depth_slot = GPU_shader_get_sampler_binding(blit_sh(), "depth_tx");
    GPU_texture_filter_mode(comp_depth_tx_, false);
    GPU_texture_bind(comp_depth_tx_, depth_slot);

    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, blit_sh());
    GPU_batch_draw(fullscreen_batch_);

    GPU_texture_unbind(comp_color_tx_);
    GPU_texture_unbind(comp_depth_tx_);
    GPU_shader_unbind();
  }

  /* FXAA post-process */

  void ensure_fxaa_target()
  {
    const int2 vp = int2(draw_ctx_->viewport_size_get());
    if (march_color_tx_ != nullptr && fxaa_size_ == vp) {
      return;
    }

    if (march_color_tx_) {
      GPU_texture_free(march_color_tx_);
      march_color_tx_ = nullptr;
    }
    if (march_fb_) {
      GPU_framebuffer_free(march_fb_);
      march_fb_ = nullptr;
    }

    fxaa_size_ = vp;

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT;
    march_color_tx_ = GPU_texture_create_2d(
        "sdf_march_color", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);

    DefaultTextureList *dtxl = draw_ctx_->viewport_texture_list_get();
    march_fb_ = GPU_framebuffer_create("sdf_march_fb");
    GPU_framebuffer_texture_attach(march_fb_, march_color_tx_, 0, 0);
    GPU_framebuffer_texture_attach(march_fb_, dtxl->depth, 0, 0);
  }

  void draw_fxaa()
  {
    if (!fxaa_enabled_ || fxaa_sh() == nullptr) {
      return;
    }

    if (draw_ctx_->is_depth() || (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER)) {
      return;
    }

    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();
    GPU_framebuffer_bind(dfbl->default_fb);

    GPU_depth_test(GPU_DEPTH_NONE);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    GPU_shader_bind(fxaa_sh());

    int color_slot = GPU_shader_get_sampler_binding(fxaa_sh(), "color_tx");
    GPU_texture_filter_mode(march_color_tx_, true);
    GPU_texture_bind(march_color_tx_, color_slot);

    float2 rcp = float2(1.0f / float(viewport_size_.x), 1.0f / float(viewport_size_.y));
    GPU_shader_uniform_2fv(fxaa_sh(), "rcpFrame", rcp);

    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, fxaa_sh());
    GPU_batch_draw(fullscreen_batch_);

    GPU_texture_unbind(march_color_tx_);
    GPU_shader_unbind();

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
  }

 public:
  ~Instance() override
  {
    clear_exported_state();

    /* GPU context may already be torn down during shutdown */
    if (!GPU_context_active_get()) {
      return;
    }

    if (comp_color_tx_) {
      GPU_texture_free(comp_color_tx_);
    }
    if (comp_depth_tx_) {
      GPU_texture_free(comp_depth_tx_);
    }
    if (gbuf_pos_tx_) {
      GPU_texture_free(gbuf_pos_tx_);
    }
    if (gbuf_color_tx_) {
      GPU_texture_free(gbuf_color_tx_);
    }
    if (gbuf_normal_tx_) {
      GPU_texture_free(gbuf_normal_tx_);
    }
    if (march_color_tx_) {
      GPU_texture_free(march_color_tx_);
    }
    if (march_fb_) {
      GPU_framebuffer_free(march_fb_);
    }
    if (matcap_tx_) {
      GPU_texture_free(matcap_tx_);
    }
    if (object_ssbo_) {
      GPU_storagebuf_free(object_ssbo_);
    }
    if (object_aabb_ssbo_) {
      GPU_storagebuf_free(object_aabb_ssbo_);
    }
    if (modifier_ssbo_) {
      GPU_storagebuf_free(modifier_ssbo_);
    }
    if (polygon_ssbo_) {
      GPU_storagebuf_free(polygon_ssbo_);
    }
    if (mesh_data_ssbo_) {
      GPU_storagebuf_free(mesh_data_ssbo_);
    }
    if (group_ssbo_) {
      GPU_storagebuf_free(group_ssbo_);
    }
    if (bvh_nodes_ssbo_) {
      GPU_storagebuf_free(bvh_nodes_ssbo_);
    }
    if (cone_hit_ssbo_) {
      GPU_storagebuf_free(cone_hit_ssbo_);
    }
    if (tile_far_hint_ssbo_) {
      GPU_storagebuf_free(tile_far_hint_ssbo_);
    }
    if (screen_aabbs_ssbo_) {
      GPU_storagebuf_free(screen_aabbs_ssbo_);
    }
    if (tile_prim_counts_ssbo_) {
      GPU_storagebuf_free(tile_prim_counts_ssbo_);
    }
    if (tile_prim_lists_ssbo_) {
      GPU_storagebuf_free(tile_prim_lists_ssbo_);
    }
    if (prof_eval_ssbo_) {
      GPU_storagebuf_free(prof_eval_ssbo_);
    }
    if (prof_stats_ssbo_) {
      GPU_storagebuf_free(prof_stats_ssbo_);
    }
    if (shading_ubo_) {
      GPU_uniformbuf_free(shading_ubo_);
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
    if (lp_parents_init_ssbo_) {
      GPU_storagebuf_free(lp_parents_init_ssbo_);
    }
    if (lp_active_init_ssbo_) {
      GPU_storagebuf_free(lp_active_init_ssbo_);
    }
    for (int i = 0; i < 2; i++) {
      if (lp_active_ssbo_[i]) {
        GPU_storagebuf_free(lp_active_ssbo_[i]);
      }
      if (lp_parents_ssbo_[i]) {
        GPU_storagebuf_free(lp_parents_ssbo_[i]);
      }
      if (lp_num_active_ssbo_[i]) {
        GPU_storagebuf_free(lp_num_active_ssbo_[i]);
      }
      if (lp_cell_offset_ssbo_[i]) {
        GPU_storagebuf_free(lp_cell_offset_ssbo_[i]);
      }
      if (lp_cell_value_ssbo_[i]) {
        GPU_storagebuf_free(lp_cell_value_ssbo_[i]);
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
    if (lp_tmp_count_ssbo_) {
      GPU_storagebuf_free(lp_tmp_count_ssbo_);
    }

    if (fullscreen_batch_) {
      GPU_batch_discard(fullscreen_batch_);
    }
  }
};

/* Public API */

int sdf_object_count_get()
{
  return s_object_count;
}

int sdf_group_count_get()
{
  return s_group_count;
}

const SDFObjectGPU *sdf_objects_cpu_get()
{
  return s_objects_cpu;
}

const int *sdf_depsgraph_to_sorted_get(int *out_count)
{
  if (s_depsgraph_to_sorted.is_empty()) {
    *out_count = 0;
    return nullptr;
  }
  *out_count = int(s_depsgraph_to_sorted.size());
  return s_depsgraph_to_sorted.data();
}

int sdf_sorted_index_for_object(const Object *ob)
{
  const int *val = s_object_to_sorted.lookup_ptr(ob);
  return val ? *val : -1;
}

const Object *const *sdf_sorted_object_ptrs_get(int *out_count)
{
  *out_count = int(s_sorted_object_ptrs.size());
  return s_sorted_object_ptrs.data();
}

gpu::Texture *sdf_depth_texture_get()
{
  return s_depth_tx;
}

gpu::Texture *sdf_gbuf_color_texture_get()
{
  return s_gbuf_color_tx;
}

bool sdf_object_at_pixel(const int2 pixel,
                         const int2 viewport_size,
                         const void *viewport_key,
                         const Object **r_object,
                         float *r_depth)
{
  *r_object = nullptr;
  *r_depth = 1.0f;
  if (viewport_key != s_viewport_key || s_gbuf_color_tx == nullptr || s_depth_tx == nullptr ||
      s_render_size.x <= 0 ||
      s_render_size.y <= 0 || s_texture_size.x <= 0 || s_texture_size.y <= 0 ||
      viewport_size.x <= 0 || viewport_size.y <= 0 || s_sorted_object_ptrs.is_empty() ||
      s_sorted_object_ptrs.size() > 2048)
  {
    return false;
  }

  const int x = math::clamp(pixel.x * s_render_size.x / viewport_size.x, 0, s_render_size.x - 1);
  const int y = math::clamp(pixel.y * s_render_size.y / viewport_size.y, 0, s_render_size.y - 1);
  const int index = y * s_texture_size.x + x;
  float4 *colors = static_cast<float4 *>(
      GPU_texture_read(s_gbuf_color_tx, GPU_DATA_FLOAT, 0));
  float *depths = static_cast<float *>(GPU_texture_read(s_depth_tx, GPU_DATA_FLOAT, 0));
  if (colors == nullptr || depths == nullptr) {
    MEM_SAFE_DELETE(colors);
    MEM_SAFE_DELETE(depths);
    return false;
  }

  const int object_index = int(colors[index].w + 0.5f);
  const float depth = depths[index];
  MEM_delete(colors);
  MEM_delete(depths);
  if (depth <= 0.0f || object_index < 0 || object_index >= s_sorted_object_ptrs.size()) {
    return false;
  }
  *r_object = s_sorted_object_ptrs[object_index];
  *r_depth = depth;
  return *r_object != nullptr;
}

float2 sdf_uv_scale_get()
{
  if (s_texture_size.x == 0 || s_texture_size.y == 0) {
    return float2(1.0f);
  }
  return float2(float(s_render_size.x) / float(s_texture_size.x),
                float(s_render_size.y) / float(s_texture_size.y));
}

bool sdf_object_bbox_get(int sdf_index, const float3 &hint_pos,
                         float3 &out_min, float3 &out_max,
                         float4x4 &out_rot, float3 &out_pos)
{
  /* Find the GPU object closest to hint_pos among those with matching index. */
  int best = -1;
  float best_dist = 1e30f;
  for (int i = 0; i < s_objects_cpu_count; i++) {
    if (s_objects_cpu[i].original_index == sdf_index) {
      float d = math::length(float3(s_objects_cpu[i].position) - hint_pos);
      if (d < best_dist) {
        best_dist = d;
        best = i;
      }
    }
  }

  if (best >= 0) {
    int i = best;
    {
      const SDFObjectGPU &obj = s_objects_cpu[i];
      /* sdf_size is BASE (unscaled); apply per-axis object scale to get world extent. */
      float3 sz(obj.sdf_size);
      float3 oscale(obj.obj_scale);
      float3 pos = float3(obj.position);
      float4x4 rot = math::transpose(obj.inverse_matrix);

      if (obj.sdf_type == SDF_TYPE_MESH) {
        out_min = float3(obj.mesh_bounds_min) * oscale;
        out_max = float3(obj.mesh_bounds_max) * oscale;
        out_rot = rot;
        out_pos = pos;
        return true;
      }

      /* Rotation-stable search region: compute in local space from primitive. */
      float3 ext;
      switch (obj.sdf_type) {
        case SDF_TYPE_CONE: { float r = std::max(sz.x, sz.z); ext = float3(r, r, sz.y); break; }
        case SDF_TYPE_CAPSULE: ext = float3(sz.x, sz.x, sz.y + sz.x); break;
        case SDF_TYPE_TORUS: ext = float3(sz.x + sz.y, sz.x + sz.y, sz.y); break;
        case SDF_TYPE_NGON: ext = float3(sz.x, sz.x, sz.z); break;
        case SDF_TYPE_POLYGON: {
          /* CPU eval doesn't support polygon — use polygon point bounds directly. */
          float3 mn(1e30f), mx(-1e30f);
          if (obj.polygon_point_count > 0) {
            for (int pi = 0; pi < obj.polygon_point_count; pi++) {
              int idx = obj.polygon_point_start + pi;
              if (idx >= 0 && idx < s_polygon_pts_count) {
                float2 vi(s_polygon_pts_cpu[idx].vi_edge.x, s_polygon_pts_cpu[idx].vi_edge.y);
                mn.x = std::min(mn.x, vi.x);
                mn.y = std::min(mn.y, vi.y);
                mx.x = std::max(mx.x, vi.x);
                mx.y = std::max(mx.y, vi.y);
              }
            }
          }
          mn.z = -sz.z;
          mx.z = sz.z;
          out_min = mn * oscale;
          out_max = mx * oscale;
          out_rot = rot;
          out_pos = pos;
          return true;
        }
        default: ext = sz; break;
      }
      ext = ext * oscale;
      /* Search region: expand by non-mirror modifiers only.
       * Mirror bbox copies are drawn by the overlay's copy system. */
      float3 search_ext = ext;
      for (int mi = obj.modifier_start; mi < obj.modifier_start + obj.modifier_count; mi++) {
        if (mi < 0 || mi >= s_modifier_count) { break; }
        const SDFModifierGPU &mod = s_modifiers_cpu[mi];
        int mt = mod.header.x;
        if (mt == SDF_MOD_ELONGATE) {
          search_ext += float3(mod.params.x, mod.params.y, mod.params.z);
        }
        else if (mt == SDF_MOD_TWIST) {
          int axis = int(mod.params.y);
          if (axis == 1) {
            float xz = math::sqrt(search_ext.x * search_ext.x +
                                  search_ext.z * search_ext.z);
            search_ext.x = xz;
            search_ext.z = xz;
          }
          else if (axis == 2) {
            float yz = math::sqrt(search_ext.y * search_ext.y +
                                  search_ext.z * search_ext.z);
            search_ext.y = yz;
            search_ext.z = yz;
          }
          else {
            float xy = math::sqrt(search_ext.x * search_ext.x +
                                  search_ext.y * search_ext.y);
            search_ext.x = xy;
            search_ext.y = xy;
          }
        }
        else if (mt == SDF_MOD_BEND) {
          float diag = math::length(search_ext);
          search_ext = float3(diag);
        }
        else if (mt == SDF_MOD_SOLIDIFY || mt == SDF_MOD_ONION) {
          /* Never expand — these only carve inward. */
        }
        else if (mt == SDF_MOD_DISPLACE) {
          search_ext += float3(fabsf(mod.params.x));
        }
        else if (mt == SDF_MOD_ROUND || mt == SDF_MOD_BEVEL) {
          search_ext = math::max(search_ext + float3(mod.params.x), float3(0.0f));
        }
      }
      search_ext *= 1.1f;
      float3 search_min = -search_ext;
      float3 search_max = search_ext;

      constexpr int RES = 20;
      float3 cell = (search_max - search_min) / float(RES);
      float threshold = std::max(cell.x, std::max(cell.y, cell.z)) * 0.6f;
      float3 bb_min(1e30f), bb_max(-1e30f);
      bool found = false;

      for (int iz = 0; iz <= RES; iz++) {
        for (int iy = 0; iy <= RES; iy++) {
          for (int ix = 0; ix <= RES; ix++) {
            float3 lp = search_min + float3(float(ix), float(iy), float(iz)) * cell;
            float d = sdf_cpu::evalObjectSDF(obj, s_modifiers_cpu, lp, true, true);
            if (d >= 0.0f && d < threshold) {
              bb_min = math::min(bb_min, lp);
              bb_max = math::max(bb_max, lp);
              found = true;
            }
          }
        }
      }

      if (!found) {
        bb_min = -ext;
        bb_max = ext;
      }
      else {
        bb_min -= cell * 0.25f;
        bb_max += cell * 0.25f;
      }

      out_min = bb_min;
      out_max = bb_max;
      out_rot = rot;
      out_pos = pos;
      return true;
    }
  }
  return false;
}

DrawEngine *Engine::create_instance()
{
  return new Instance();
}

std::string sdf_dual_contour_to_mesh(int grid_res,
                                      Vector<float3> &out_positions,
                                      Vector<float3> &out_normals,
                                      Vector<int3> &out_tris,
                                      Vector<float4> &out_colors,
                                      int *out_vert_count,
                                      int *out_tri_count)
{
  *out_vert_count = 0;
  *out_tri_count = 0;

  if (s_object_count == 0 || !s_object_ssbo) {
    return "No SDF data. Render viewport first.";
  }

  const float cell_size = 1.0f / float(grid_res);
  const int CHUNK = 256;
  const int PAD = 1;
  const int PADDED = CHUNK + 2 * PAD;
  const int PADDED_GV = PADDED + 1;

  /* Scene AABB */
  Vector<SDFObjectGPU> objs(s_object_count);
  GPU_storagebuf_read(s_object_ssbo, objs.data());

  float3 scene_min(1e30f), scene_max(-1e30f);
  for (int i = 0; i < s_object_count; i++) {
    scene_min = math::min(scene_min, float3(objs[i].bbox_min));
    scene_max = math::max(scene_max, float3(objs[i].bbox_max));
  }
  scene_min -= float3(cell_size * 2.0f);
  scene_max += float3(cell_size * 2.0f);

  int3 grid_cells;
  grid_cells.x = int(ceilf((scene_max.x - scene_min.x) / cell_size));
  grid_cells.y = int(ceilf((scene_max.y - scene_min.y) / cell_size));
  grid_cells.z = int(ceilf((scene_max.z - scene_min.z) / cell_size));
  float3 grid_origin = scene_min;

  int3 n_chunks;
  n_chunks.x = (grid_cells.x + CHUNK - 1) / CHUNK;
  n_chunks.y = (grid_cells.y + CHUNK - 1) / CHUNK;
  n_chunks.z = (grid_cells.z + CHUNK - 1) / CHUNK;

  /* Compile shaders (cached) */
  gpu::Shader *grid_sh = GPU_shader_create_from_info_name("sdf_grid_eval_comp");
  gpu::Shader *dc_sh = GPU_shader_create_from_info_name("sdf_dc_contour_comp");
  gpu::Shader *tri_sh = GPU_shader_create_from_info_name("sdf_dc_triangulate_comp");
  gpu::Shader *color_sh = GPU_shader_create_from_info_name("sdf_dc_vertex_color_comp");
  if (!grid_sh || !dc_sh || !tri_sh || !color_sh) return "Shader compile failed";

  /* Scale output buffers with resolution: surface verts grow ~O(res^2).
   * Base 4M/8M sized for res=32; scale quadratically, cap at 32M/64M. */
  const int base_verts = 4 * 1024 * 1024;
  const int base_tris = 8 * 1024 * 1024;
  const float res_scale = float(grid_res) / 32.0f;
  const float area_scale = math::max(res_scale * res_scale, 1.0f);
  const int global_max_verts = math::min(int(base_verts * area_scale), 32 * 1024 * 1024);
  const int global_max_tris = math::min(int(base_tris * area_scale), 64 * 1024 * 1024);

  gpu::StorageBuf *vert_ssbo = GPU_storagebuf_create_ex(
      global_max_verts * sizeof(DCVertexGPU), nullptr, GPU_USAGE_DYNAMIC, "dc_vertices");
  gpu::StorageBuf *counter_ssbo = GPU_storagebuf_create_ex(
      2 * sizeof(int), nullptr, GPU_USAGE_DYNAMIC, "dc_counters");
  GPU_storagebuf_clear_to_zero(counter_ssbo);
  gpu::StorageBuf *tri_ssbo = GPU_storagebuf_create_ex(
      global_max_tris * sizeof(int) * 4, nullptr, GPU_USAGE_DYNAMIC, "dc_triangles");

  /* Per-chunk buffers (reused) */
  int padded_grid_total = PADDED_GV * PADDED_GV * PADDED_GV;
  int padded_cells_total = PADDED * PADDED * PADDED;
  gpu::StorageBuf *grid_ssbo = GPU_storagebuf_create_ex(
      padded_grid_total * sizeof(float), nullptr, GPU_USAGE_DYNAMIC, "dc_grid_values");
  gpu::StorageBuf *cell_ssbo = GPU_storagebuf_create_ex(
      padded_cells_total * sizeof(int), nullptr, GPU_USAGE_DYNAMIC, "dc_cell_verts");

  /* Color output buffer (one float4 per vertex) */
  gpu::StorageBuf *color_ssbo = GPU_storagebuf_create_ex(
      global_max_verts * sizeof(float4), nullptr, GPU_USAGE_DYNAMIC, "dc_colors");

  auto cleanup = [&]() {
    GPU_storagebuf_free(grid_ssbo);
    GPU_storagebuf_free(vert_ssbo);
    GPU_storagebuf_free(counter_ssbo);
    GPU_storagebuf_free(cell_ssbo);
    GPU_storagebuf_free(tri_ssbo);
    GPU_storagebuf_free(color_ssbo);
    GPU_shader_free(grid_sh);
    GPU_shader_free(dc_sh);
    GPU_shader_free(tri_sh);
    GPU_shader_free(color_sh);
  };

  /* Pre-bind SSBO slots that don't change between chunks */
  int grid_obj_slot = GPU_shader_get_ssbo_binding(grid_sh, "objects");
  int grid_mod_slot = GPU_shader_get_ssbo_binding(grid_sh, "sdf_modifiers");
  int grid_grp_slot = GPU_shader_get_ssbo_binding(grid_sh, "groups");
  int grid_poly_slot = s_polygon_ssbo ?
                           GPU_shader_get_ssbo_binding(grid_sh, "polygon_points") :
                           -1;
  int grid_val_slot = GPU_shader_get_ssbo_binding(grid_sh, "grid_values");
  int grid_bvh_slot = GPU_shader_get_ssbo_binding(grid_sh, "aabb_nodes");
  int grid_mesh_data_slot = GPU_shader_get_ssbo_binding(grid_sh, "mesh_data_buf");
  int has_bvh = (s_bvh_ssbo && s_bvh_root >= 0) ? 1 : 0;

  int dc_gv_slot = GPU_shader_get_ssbo_binding(dc_sh, "grid_values");
  int dc_v_slot = GPU_shader_get_ssbo_binding(dc_sh, "dc_vertices");
  int dc_c_slot = GPU_shader_get_ssbo_binding(dc_sh, "dc_counters");
  int dc_cv_slot = GPU_shader_get_ssbo_binding(dc_sh, "dc_cell_verts");

  int tr_gv_slot = GPU_shader_get_ssbo_binding(tri_sh, "grid_values");
  int tr_t_slot = GPU_shader_get_ssbo_binding(tri_sh, "dc_triangles");
  int tr_c_slot = GPU_shader_get_ssbo_binding(tri_sh, "dc_counters");
  int tr_cv_slot = GPU_shader_get_ssbo_binding(tri_sh, "dc_cell_verts");

  int gd = (PADDED_GV + 3) / 4;
  int cd = (PADDED + 3) / 4;
  int chunk_done = 0, chunk_skipped = 0;

  for (int cz = 0; cz < n_chunks.z; cz++) {
    for (int cy = 0; cy < n_chunks.y; cy++) {
      for (int cx = 0; cx < n_chunks.x; cx++) {
        int3 cell_start = int3(cx, cy, cz) * CHUNK - int3(PAD);
        float3 chunk_origin = grid_origin + float3(cell_start) * cell_size;
        float3 chunk_min = chunk_origin;
        float3 chunk_max = chunk_origin + float3(float(PADDED_GV)) * cell_size;

        /* AABB cull: skip chunks with no nearby SDF objects */
        bool has_overlap = false;
        for (int i = 0; i < s_object_count; i++) {
          float3 obj_min = float3(objs[i].bbox_min);
          float3 obj_max = float3(objs[i].bbox_max);
          if (chunk_min.x <= obj_max.x && chunk_max.x >= obj_min.x &&
              chunk_min.y <= obj_max.y && chunk_max.y >= obj_min.y &&
              chunk_min.z <= obj_max.z && chunk_max.z >= obj_min.z)
          {
            has_overlap = true;
            break;
          }
        }
        if (!has_overlap) {
          chunk_skipped++;
          chunk_done++;
          continue;
        }

        GPU_storagebuf_clear(cell_ssbo, 0xFFFFFFFFu);

        /* Grid eval */
        GPU_shader_bind(grid_sh);
        if (s_object_ssbo && grid_obj_slot >= 0) GPU_storagebuf_bind(s_object_ssbo, grid_obj_slot);
        if (s_modifier_ssbo && grid_mod_slot >= 0) GPU_storagebuf_bind(s_modifier_ssbo, grid_mod_slot);
        if (s_group_ssbo && grid_grp_slot >= 0) GPU_storagebuf_bind(s_group_ssbo, grid_grp_slot);
        if (grid_poly_slot >= 0) GPU_storagebuf_bind(s_polygon_ssbo, grid_poly_slot);
        if (grid_val_slot >= 0) GPU_storagebuf_bind(grid_ssbo, grid_val_slot);
        if (has_bvh && grid_bvh_slot >= 0) GPU_storagebuf_bind(s_bvh_ssbo, grid_bvh_slot);
        if (s_mesh_data_ssbo && grid_mesh_data_slot >= 0) {
          GPU_storagebuf_bind(s_mesh_data_ssbo, grid_mesh_data_slot);
        }
        GPU_shader_uniform_1i(grid_sh, "object_count", s_object_count);
        GPU_shader_uniform_1i(grid_sh, "group_count", s_group_count);
        GPU_shader_uniform_1i(grid_sh, "grid_verts", PADDED_GV);
        GPU_shader_uniform_3fv(grid_sh, "grid_origin", chunk_origin);
        GPU_shader_uniform_1f(grid_sh, "cell_size", cell_size);
        GPU_shader_uniform_1i(grid_sh, "use_bvh", has_bvh);
        GPU_shader_uniform_1i(grid_sh, "bvh_root", s_bvh_root);
        GPU_compute_dispatch(grid_sh, gd, gd, gd);
        GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

        /* DC contour → writes to GLOBAL vert buffer + local cell map */
        GPU_shader_bind(dc_sh);
        if (dc_gv_slot >= 0) GPU_storagebuf_bind(grid_ssbo, dc_gv_slot);
        if (dc_v_slot >= 0) GPU_storagebuf_bind(vert_ssbo, dc_v_slot);
        if (dc_c_slot >= 0) GPU_storagebuf_bind(counter_ssbo, dc_c_slot);
        if (dc_cv_slot >= 0) GPU_storagebuf_bind(cell_ssbo, dc_cv_slot);
        GPU_shader_uniform_1i(dc_sh, "grid_verts", PADDED_GV);
        GPU_shader_uniform_3fv(dc_sh, "grid_origin", chunk_origin);
        GPU_shader_uniform_1f(dc_sh, "cell_size", cell_size);
        GPU_shader_uniform_1i(dc_sh, "max_verts", global_max_verts);
        GPU_compute_dispatch(dc_sh, cd, cd, cd);
        GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

        /* Triangulation → only inner cells, writes to GLOBAL tri buffer */
        GPU_shader_bind(tri_sh);
        if (tr_gv_slot >= 0) GPU_storagebuf_bind(grid_ssbo, tr_gv_slot);
        if (tr_t_slot >= 0) GPU_storagebuf_bind(tri_ssbo, tr_t_slot);
        if (tr_c_slot >= 0) GPU_storagebuf_bind(counter_ssbo, tr_c_slot);
        if (tr_cv_slot >= 0) GPU_storagebuf_bind(cell_ssbo, tr_cv_slot);
        GPU_shader_uniform_1i(tri_sh, "grid_verts", PADDED_GV);
        GPU_shader_uniform_1i(tri_sh, "inner_start", PAD);
        GPU_shader_uniform_1i(tri_sh, "inner_end", PAD + CHUNK);
        GPU_shader_uniform_1i(tri_sh, "max_tris", global_max_tris);
        GPU_compute_dispatch(tri_sh, cd, cd, cd);
        GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

        chunk_done++;
      }
    }
  }
  GPU_shader_unbind();

  /* Single readback at end */
  Vector<int> counters(2);
  GPU_storagebuf_read(counter_ssbo, counters.data());
  int vert_count = math::min(counters[0], global_max_verts);
  int tri_count = math::min(counters[1], global_max_tris);

  if (counters[0] > global_max_verts || counters[1] > global_max_tris) {
    printf("SDF DC: buffer overflow — %d/%d verts, %d/%d tris (res=%d). Mesh truncated.\n",
           counters[0], global_max_verts, counters[1], global_max_tris, grid_res);
  }

  if (vert_count == 0) {
    cleanup();
    return "DC produced 0 vertices";
  }

  /* Vertex color pass: evaluate SDF color at each DC vertex position */
  {
    GPU_shader_bind(color_sh);
    int col_obj_slot = GPU_shader_get_ssbo_binding(color_sh, "objects");
    int col_mod_slot = GPU_shader_get_ssbo_binding(color_sh, "sdf_modifiers");
    int col_grp_slot = GPU_shader_get_ssbo_binding(color_sh, "groups");
    int col_poly_slot = s_polygon_ssbo ?
                            GPU_shader_get_ssbo_binding(color_sh, "polygon_points") :
                            -1;
    int col_pos_slot = GPU_shader_get_ssbo_binding(color_sh, "dc_positions");
    int col_out_slot = GPU_shader_get_ssbo_binding(color_sh, "dc_colors");
    int col_bvh_slot = GPU_shader_get_ssbo_binding(color_sh, "aabb_nodes");
    int col_mesh_data_slot = GPU_shader_get_ssbo_binding(color_sh, "mesh_data_buf");

    if (s_object_ssbo && col_obj_slot >= 0) GPU_storagebuf_bind(s_object_ssbo, col_obj_slot);
    if (s_modifier_ssbo && col_mod_slot >= 0) GPU_storagebuf_bind(s_modifier_ssbo, col_mod_slot);
    if (s_group_ssbo && col_grp_slot >= 0) GPU_storagebuf_bind(s_group_ssbo, col_grp_slot);
    if (col_poly_slot >= 0) GPU_storagebuf_bind(s_polygon_ssbo, col_poly_slot);
    if (col_pos_slot >= 0) GPU_storagebuf_bind(vert_ssbo, col_pos_slot);
    if (col_out_slot >= 0) GPU_storagebuf_bind(color_ssbo, col_out_slot);
    if (has_bvh && col_bvh_slot >= 0) GPU_storagebuf_bind(s_bvh_ssbo, col_bvh_slot);
    if (s_mesh_data_ssbo && col_mesh_data_slot >= 0) {
      GPU_storagebuf_bind(s_mesh_data_ssbo, col_mesh_data_slot);
    }

    GPU_shader_uniform_1i(color_sh, "object_count", s_object_count);
    GPU_shader_uniform_1i(color_sh, "group_count", s_group_count);
    GPU_shader_uniform_1i(color_sh, "vert_count", vert_count);
    GPU_shader_uniform_1i(color_sh, "use_bvh", has_bvh);
    GPU_shader_uniform_1i(color_sh, "bvh_root", s_bvh_root);

    int color_groups = (vert_count + 63) / 64;
    GPU_compute_dispatch(color_sh, color_groups, 1, 1);
    GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
    GPU_shader_unbind();
  }

  Vector<DCVertexGPU> gpu_verts(global_max_verts);
  GPU_storagebuf_read(vert_ssbo, gpu_verts.data());

  out_positions.resize(vert_count);
  out_normals.resize(vert_count);
  for (int i = 0; i < vert_count; i++) {
    out_positions[i] = float3(gpu_verts[i].position);
    out_normals[i] = float3(gpu_verts[i].normal);
  }

  /* Read back vertex colors */
  Vector<float4> gpu_colors(global_max_verts);
  GPU_storagebuf_read(color_ssbo, gpu_colors.data());
  out_colors.resize(vert_count);
  for (int i = 0; i < vert_count; i++) {
    out_colors[i] = gpu_colors[i];
  }

  Vector<int4> gpu_tris(global_max_tris);
  GPU_storagebuf_read(tri_ssbo, gpu_tris.data());

  out_tris.reserve(tri_count);
  for (int i = 0; i < tri_count; i++) {
    int a = gpu_tris[i].x, b = gpu_tris[i].y, c = gpu_tris[i].z;
    if (a >= 0 && a < vert_count && b >= 0 && b < vert_count && c >= 0 && c < vert_count) {
      out_tris.append(int3(a, b, c));
    }
  }

  *out_vert_count = vert_count;
  *out_tri_count = out_tris.size();

  cleanup();
  return "";
}

void sdf_profile_request()
{
  s_profile_pending = true;
  s_profile_result.valid = false;
}

bool sdf_profile_is_ready()
{
  return s_profile_result.valid;
}

bool sdf_profile_is_pending()
{
  return s_profile_pending;
}

static const char *sdf_type_name(int t)
{
  switch (t) {
    case SDF_TYPE_BOX: return "Box";
    case SDF_TYPE_SPHERE: return "Sphere";
    case SDF_TYPE_CYLINDER: return "Cylinder";
    case SDF_TYPE_CONE: return "Cone";
    case SDF_TYPE_CAPSULE: return "Capsule";
    case SDF_TYPE_TORUS: return "Torus";
    case SDF_TYPE_NGON: return "Ngon";
    case SDF_TYPE_POLYGON: return "Polygon";
    case SDF_TYPE_GROUP: return "Group";
    default: return "Unknown";
  }
}

static const char *sdf_blend_name(int t)
{
  switch (t) {
    case SDF_BLEND_LINEAR: return "Linear";
    case SDF_BLEND_SMOOTH: return "Smooth";
    case SDF_BLEND_CHAMFER: return "Chamfer";
    case SDF_BLEND_ROUND: return "Round";
    default: return "Unknown";
  }
}

static const char *sdf_csg_name(int t)
{
  switch (t) {
    case SDF_CSG_UNION: return "Union";
    case SDF_CSG_SUBTRACT: return "Subtract";
    case SDF_CSG_INTERSECT: return "Intersect";
    case SDF_CSG_SHELL: return "Shell";
    case SDF_CSG_PUSH: return "Push";
    case SDF_CSG_AVOID: return "Avoid";
    case SDF_CSG_PAINT: return "Paint";
    default: return "Unknown";
  }
}

static const char *sdf_mod_name(int t)
{
  switch (t) {
    case SDF_MOD_MIRROR: return "Mirror";
    case SDF_MOD_TWIST: return "Twist";
    case SDF_MOD_BEND: return "Bend";
    case SDF_MOD_ELONGATE: return "Elongate";
    case SDF_MOD_SOLIDIFY: return "Solidify";
    case SDF_MOD_ROUND: return "Round";
    case SDF_MOD_ONION: return "Onion";
    case SDF_MOD_BEVEL: return "Bevel";
    case SDF_MOD_ARRAY: return "Array";
    case SDF_MOD_DISPLACE: return "Displace";
    default: return "Unknown";
  }
}

std::string sdf_profile_format_text()
{
  if (!s_profile_result.valid) {
    return "No profiling data available.\n";
  }

  const SdfProfileResult &r = s_profile_result;
  std::string out;
  out.reserve(4096);

  char buf[512];

  out += "================================================================\n";
  out += "SDF FRAME PROFILE\n";
  out += "================================================================\n\n";

  snprintf(buf, sizeof(buf),
           "Resolution:   %d x %d (scale: %.0f%%)\n",
           r.render_width, r.render_height, r.resolution_scale * 100.0f);
  out += buf;

  snprintf(buf, sizeof(buf),
           "Objects:      %d\n"
           "Groups:       %d\n"
           "Modifiers:    %d\n"
           "BVH Nodes:    %d\n\n",
           r.object_count, r.group_count, r.modifier_total, r.bvh_node_count);
  out += buf;

  /* Pass timing */
  out += "PASS TIMING\n";
  out += "----------------------------------------------------------------\n";

  double total_pass_ns = 0.0;
  for (int i = 0; i < r.pass_count; i++) {
    total_pass_ns += r.passes[i].time_ns;
  }

  for (int i = 0; i < r.pass_count; i++) {
    const auto &p = r.passes[i];
    double ms = p.time_ns / 1e6;
    double us = p.time_ns / 1e3;
    double pct = (r.total_ns > 0.0) ? (p.time_ns / r.total_ns * 100.0) : 0.0;

    if (ms >= 1.0) {
      snprintf(buf, sizeof(buf), "  %-18s %10.3f ms  (%5.1f%%)\n", p.name, ms, pct);
    }
    else {
      snprintf(buf, sizeof(buf), "  %-18s %10.3f us  (%5.1f%%)\n", p.name, us, pct);
    }
    out += buf;
  }

  out += "----------------------------------------------------------------\n";
  double total_ms = r.total_ns / 1e6;
  double fps = (total_ms > 0.0) ? (1000.0 / total_ms) : 0.0;
  snprintf(buf, sizeof(buf),
           "  TOTAL              %10.3f ms  (%.1f FPS)\n\n", total_ms, fps);
  out += buf;

  /* Trace diagnostics */
  const SdfTraceStats &s = r.trace_stats;
  uint64_t total_evals = 0;
  for (int i = 0; i < r.profiled_object_count; i++) {
    total_evals += r.eval_counts[i];
  }

  int total_pixels = r.render_width * r.render_height;
  double steps_per_ray = (s.total_rays > 0) ? double(s.total_steps) / double(s.total_rays) : 0;
  double evals_per_step = (s.total_steps > 0) ? double(total_evals) / double(s.total_steps) : 0;
  double evals_per_ray = (s.total_rays > 0) ? double(total_evals) / double(s.total_rays) : 0;
  double hit_rate = (s.total_rays > 0) ? double(s.total_hits) / double(s.total_rays) * 100.0 : 0;
  double empty_pct = (s.total_steps + s.total_empty_steps > 0) ?
      double(s.total_empty_steps) / double(s.total_steps + s.total_empty_steps) * 100.0 : 0;
  double sor_pct = (s.total_steps > 0) ?
      double(s.total_sor_failures) / double(s.total_steps) * 100.0 : 0;
  double skip_rate = (total_evals + s.total_aabb_skips > 0) ?
      double(s.total_aabb_skips) / double(total_evals + s.total_aabb_skips) * 100.0 : 0;

  out += "TRACE DIAGNOSTICS\n";
  out += "----------------------------------------------------------------\n";
  snprintf(buf, sizeof(buf),
           "  Rays:               %u (of %d pixels)\n"
           "  Hits:               %u (%.1f%% hit rate)\n"
           "  Steps:              %u (%.1f steps/ray, max %u)\n"
           "  Empty steps:        %u (%.1f%% of all steps)\n"
           "  SOR failures:       %u (%.1f%% of steps)\n"
           "  Object evals:       %llu (%.1f evals/step, %.1f evals/ray)\n"
           "  AABB skips:         %u (%.1f%% skip rate)\n",
           s.total_rays, total_pixels,
           s.total_hits, hit_rate,
           s.total_steps, steps_per_ray, s.max_steps_any_ray,
           s.total_empty_steps, empty_pct,
           s.total_sor_failures, sor_pct,
           (unsigned long long)total_evals, evals_per_step, evals_per_ray,
           s.total_aabb_skips, skip_rate);
  out += buf;

  /* Diagnosis */
  out += "\n  BOTTLENECK ANALYSIS:\n";
  if (steps_per_ray > 80) {
    snprintf(buf, sizeof(buf),
             "  >> HIGH STEP COUNT (%.0f/ray) — step_factor too low or SOR failing\n", steps_per_ray);
    out += buf;
  }
  if (evals_per_step > 20) {
    snprintf(buf, sizeof(buf),
             "  >> HIGH EVALS/STEP (%.0f) — tile culling not eliminating objects\n", evals_per_step);
    out += buf;
  }
  if (sor_pct > 10) {
    snprintf(buf, sizeof(buf),
             "  >> HIGH SOR FAILURE (%.1f%%) — over_relaxation too aggressive\n", sor_pct);
    out += buf;
  }
  if (empty_pct > 30) {
    snprintf(buf, sizeof(buf),
             "  >> HIGH EMPTY STEPS (%.0f%%) — rays traversing void, scene AABBs too large\n", empty_pct);
    out += buf;
  }
  if (skip_rate < 50 && r.object_count > 10) {
    snprintf(buf, sizeof(buf),
             "  >> LOW AABB SKIP RATE (%.0f%%) — per-step AABB culling ineffective\n", skip_rate);
    out += buf;
  }

  /* Per-object eval counts (sorted by evals) */
  out += "\nPER-OBJECT EVALS (sorted by evaluation count)\n";
  out += "----------------------------------------------------------------\n";

  Vector<int> sorted(r.profiled_object_count);
  for (int i = 0; i < r.profiled_object_count; i++) sorted[i] = i;
  std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
    return r.eval_counts[a] > r.eval_counts[b];
  });

  snprintf(buf, sizeof(buf),
           "  %-4s %-20s %-10s %-10s %-12s %10s %7s  %s\n",
           "#", "Name", "Type", "CSG", "Blend", "Evals", "Share", "Mods");
  out += buf;
  out += "  ";
  for (int c = 0; c < 100; c++) out += "-";
  out += "\n";

  for (int si = 0; si < r.profiled_object_count; si++) {
    int i = sorted[si];
    const auto &obj = r.objects[i];
    uint32_t evals = r.eval_counts[i];
    double pct = (total_evals > 0) ? (double(evals) / double(total_evals) * 100.0) : 0.0;

    char mods_str[128] = "";
    if (obj.modifier_count > 0) {
      int pos = 0;
      for (int m = 0; m < obj.modifier_count && pos < 120; m++) {
        if (m > 0) pos += snprintf(mods_str + pos, sizeof(mods_str) - pos, ",");
        pos += snprintf(mods_str + pos, sizeof(mods_str) - pos, "%s",
                        sdf_mod_name(obj.modifiers[m].type));
      }
    }

    snprintf(buf, sizeof(buf),
             "  %-4d %-20s %-10s %-10s %-6s(%.2f) %10u %5.1f%%  %s\n",
             i, obj.name,
             sdf_type_name(obj.sdf_type),
             sdf_csg_name(obj.csg_operation),
             sdf_blend_name(obj.blend_type),
             obj.blend, evals, pct, mods_str);
    out += buf;
  }

  out += "================================================================\n";
  return out;
}

}  // namespace blender::draw::sdf
