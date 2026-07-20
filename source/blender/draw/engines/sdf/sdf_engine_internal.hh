/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine internals: state and the #SdfInstanceBase class shared by the
 * classic engine (sdf_engine.cc) and the Lipschitz pruning engine (sdf_lp_engine.cc).
 */

#pragma once

#include <algorithm>
#include <bit>
#include <functional>

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
#include "BKE_sdf_text.hh"
#include "BKE_studiolight.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"
#include "DNA_userdef_types.h"
#include "DNA_view3d_enums.h"
#include "DNA_view3d_types.h"

#include "CLG_log.h"

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
#include "BLI_time.h"

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

inline CLG_LogRef LOG = {"draw.sdf"};

/* Static state shared with overlay/selection code */
inline int s_object_count = 0;
inline gpu::StorageBuf *s_object_ssbo = nullptr;
inline gpu::StorageBuf *s_modifier_ssbo = nullptr;
inline gpu::StorageBuf *s_polygon_ssbo = nullptr;
inline gpu::StorageBuf *s_group_ssbo = nullptr;
inline gpu::StorageBuf *s_bvh_ssbo = nullptr;
inline gpu::StorageBuf *s_mesh_data_ssbo = nullptr;
inline gpu::StorageBuf *s_bake_dist_ssbo = nullptr;
inline gpu::StorageBuf *s_bake_nrm_ssbo = nullptr;
inline gpu::StorageBuf *s_bake_col_ssbo = nullptr;
inline int s_bvh_root = -1;
inline int s_group_count = 0;
inline Vector<int> s_depsgraph_to_sorted;
inline Map<const Object *, int> s_object_to_sorted;
inline Vector<const Object *> s_sorted_object_ptrs;
inline gpu::Texture *s_depth_tx = nullptr;
inline gpu::Texture *s_gbuf_color_tx = nullptr;
inline int2 s_render_size = {0, 0};
inline int2 s_texture_size = {0, 0};
inline const void *s_viewport_key = nullptr;
inline const SDFObjectGPU *s_objects_cpu = nullptr;
inline int s_objects_cpu_count = 0;
inline const SDFPolygonPointGPU *s_polygon_pts_cpu = nullptr;
inline int s_polygon_pts_count = 0;
inline const SDFModifierGPU *s_modifiers_cpu = nullptr;
inline int s_modifier_count = 0;

inline void clear_exported_state()
{
  s_object_count = 0;
  s_object_ssbo = nullptr;
  s_modifier_ssbo = nullptr;
  s_polygon_ssbo = nullptr;
  s_group_ssbo = nullptr;
  s_bvh_ssbo = nullptr;
  s_mesh_data_ssbo = nullptr;
  s_bake_dist_ssbo = nullptr;
  s_bake_nrm_ssbo = nullptr;
  s_bake_col_ssbo = nullptr;
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
  /* LP prune pass diagnostics (only when the profiled frame ran a prune). */
  bool lp_pruned;
  int64_t lp_active_capacity;
  int64_t lp_tmp_capacity;
  int lp_active_overflow;
  int lp_tmp_overflow;
};

inline bool s_profile_pending = false;
inline SdfProfileResult s_profile_result = {};

/* Backend-agnostic frame profiler: drains the pipeline with GPU_finish()
 * around each pass and measures wall time. Works on any GPU backend (the
 * previous GL timestamp queries crashed on non-OpenGL backends). Passes are
 * timed in isolation, so the profiled frame totals slightly more than a real
 * frame (no CPU/GPU overlap). */
static constexpr int PROF_MAX_SLOTS = 16;

struct SdfFrameProfiler {
  bool active = false;
  int slot_count = 0;
  char names[PROF_MAX_SLOTS][64];
  double start_s[PROF_MAX_SLOTS];
  double time_ns[PROF_MAX_SLOTS];

  void begin()
  {
    slot_count = 0;
    active = true;
  }

  void mark_start(const char *name)
  {
    if (!active || slot_count >= PROF_MAX_SLOTS) return;
    BLI_strncpy(names[slot_count], name, sizeof(names[slot_count]));
    GPU_finish();
    start_s[slot_count] = BLI_time_now_seconds();
  }

  void mark_end()
  {
    if (!active || slot_count >= PROF_MAX_SLOTS) return;
    GPU_finish();
    time_ns[slot_count] = (BLI_time_now_seconds() - start_s[slot_count]) * 1e9;
    slot_count++;
  }

  void finish(SdfProfileResult &result)
  {
    if (!active) return;
    active = false;

    double total_ns = 0.0;
    for (int i = 0; i < slot_count; i++) {
      auto &p = result.passes[i];
      BLI_strncpy(p.name, names[i], sizeof(p.name));
      p.time_ns = time_ns[i];
      total_ns += time_ns[i];
    }
    result.pass_count = slot_count;
    result.total_ns = total_ns;
  }
};

inline SdfFrameProfiler s_profiler = {};

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
  SH_LP_MARCH_COMP,
  SH_LP_DEBUG_COMP,
  SH_MESH_BAKE_COMP,
  SH_COUNT,
};

/* The cache itself lives in sdf_engine.cc and is shared by both engines.
 * Each engine only ensures/compiles the shaders it actually uses. */

/* Kick async compilation for the given shader indices (no-op once scheduled). */
void sdf_shaders_ensure_list(const Span<const int> &idx);
/* Number of shaders in the list that finished (or failed) compiling. */
int sdf_shaders_count_ready(const Span<const int> &idx);
/* Non-blocking readiness check for a single shader. */
bool sdf_shader_is_ready(int index);
/* Blocking get — only call for shaders that are ready. */
gpu::Shader *sdf_shader_get(int index);
/* Viewport compile-progress text (shown as an overlay while non-empty).
 * Reports ready/total for the list plus the name of the first pending shader. */
void sdf_compile_status_set(const Span<const int> &idx);
void sdf_compile_status_clear();

class SdfInstanceBase : public DrawEngine {
 protected:
  struct MeshOffsets {
    int vertex_start;
    int triangle_start;
    int triangle_count;
    int bvh_start;
    int vertex_count;
    int bvh_count;
    /* uint4-record offset into mesh_color_gpu_data_/mesh_color_ssbo_. */
    int color_start;
  };

  /* Per-mesh-payload bake inputs (keyed like mesh_offsets_: payload pointer
   * for OB_MESH objects, the SDF ID for OB_SDF mesh objects). Consumed by the
   * LP engine's mesh volume bake manager (sdf_lp_engine.cc). */
  struct MeshBakeInfo {
    uint64_t revision;
    float3 bounds_min;
    float3 bounds_max;
    bool has_colors;
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
  gpu::Shader *mesh_bake_sh() { return sdf_shader_get(SH_MESH_BAKE_COMP); }

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
  bool smooth_upscale_ = true;
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

  /* Per-triangle corner colors, parallel to the triangle records in
   * mesh_gpu_data_ (uint4 per triangle: xyz = 3 packed RGBA8 corner colors,
   * w = 0; a single white record for meshes without colors). */
  Vector<uint4> mesh_color_gpu_data_;
  gpu::StorageBuf *mesh_color_ssbo_ = nullptr;
  int mesh_color_ssbo_count_ = 0;
  Map<const void *, MeshBakeInfo> mesh_bake_info_;
  /* Parallel to objects_/object_ptrs_: mesh payload key (payload pointer or
   * SDF ID) for SDF_TYPE_MESH objects, nullptr otherwise. */
  Vector<const void *> object_mesh_keys_;

  /* ---- Mesh volume bake (dense per-mesh SDF/normal/color voxel grids) ----
   * Three shared append-only pools, uint per element:
   * - bake_dist: 1 uint/voxel = packHalf2x16(vec2(distance, 0)), clamped to
   *   the narrow band (+/-band);
   * - bake_nrm: 2 uints/voxel = packHalf2x16(vec2(n.x,n.y)) +
   *   packHalf2x16(vec2(n.z,0)), baked smooth (corner-normal) shading normal;
   * - bake_col: 1 uint/voxel = RGBA8 barycentric corner color (white when the
   *   mesh has no color attribute).
   * All three pools share the same voxel indexing: voxel (i,j,k) of a record
   * lives at base + i + res.x*(j + res.y*k); its center is
   * origin + (vec3(i,j,k)+0.5)*voxel_size in local UNSCALED mesh space.
   * Shared by both engines (the GLSL fast path lives in sdf_mesh_lib.glsl and
   * sdf_lp_common.glsl, gated on SDF_LP_MESH_FLAG_BAKED). */
  struct MeshBakeGrid {
    int3 res = int3(0);
    /* First voxel index (same in all three pools). */
    int base = 0;
    /* Local unscaled mesh space min corner of the voxel grid. */
    float3 origin = float3(0.0f);
    /* Cubic voxel edge length. */
    float voxel_size = 0.0f;
    /* Narrow band half-width (4 * voxel_size). */
    float band = 0.0f;
    /* Coarse far-field level (fp32, unclamped out to the blend reach;
     * cres.x == 0 when the scene has no blends). */
    int3 cres = int3(0);
    int cbase = 0;
    float3 corigin = float3(0.0f);
    float cvoxel = 0.0f;
    /* Bake completion of the two levels. */
    bool ready = false;
    bool cready = false;
  };

  struct MeshBakeRecord {
    /* The grid the runtime samples. Stays valid while a rebake is in flight
     * — mesh objects never disappear during rebakes; the stale shape renders
     * until the new bake flips in. */
    MeshBakeGrid live;
    /* Payload revision / object sdf_voxel_resolution setting / scene blend
     * reach the live grid was produced from. */
    uint64_t revision = 0;
    int voxel_resolution = -1;
    float reach = 0.0f;
    /* The grid being baked (work_active while incomplete). Swaps into
     * `live` when both levels complete; the old live grid then rides
     * along in `work` as the SPARE range, reused by the next rebake whose
     * layout matches (steady edit streams append nothing to the pools).
     * Never baked in place over the live range, so the live grid is
     * never corrupted mid-bake. */
    MeshBakeGrid work;
    uint64_t w_revision = 0;
    int w_setting = -1;
    float w_reach = 0.0f;
    /* Progressive bake progress of the work grid (absolute z of the next
     * unbaked slice per level). */
    int next_z = 0;
    int next_cz = 0;
    bool work_active = false;
  };

  /* Keyed by the mesh payload key (payload pointer / SDF ID; same keys as
   * mesh_offsets_). Records are kept for payloads that leave the scene: the
   * pools are append-only, so a stale record just wastes its voxels until the
   * next pool regrow. */
  Map<const void *, MeshBakeRecord> bake_records_;
  gpu::StorageBuf *bake_dist_ssbo_ = nullptr;
  gpu::StorageBuf *bake_nrm_ssbo_ = nullptr;
  gpu::StorageBuf *bake_col_ssbo_ = nullptr;
  /* Pool capacity/usage in voxels (dist & col: 1 uint/voxel; nrm: 2). */
  int64_t bake_pool_capacity_ = 0;
  int64_t bake_pool_used_ = 0;
  bool bake_overflow_warned_ = false;

  /* Const accessor for the bake record of a mesh payload key (nullptr when
   * no record exists). */
  const MeshBakeRecord *bake_record(const void *key) const
  {
    return bake_records_.lookup_ptr(key);
  }

  /* Set or clear the baked-volume fields of a mesh object from the current
   * bake records. The flag is set whenever the record's LIVE grid is
   * complete — the live grid stays valid while a rebake is in flight (the
   * stale shape renders until the new bake flips in; mesh objects never
   * disappear during rebakes). Only the very first bake of a payload leaves
   * the object invisible. Returns true when the object changed. */
  bool apply_baked_fields(SDFObjectGPU &gpu_obj, const void *key) const
  {
    const int old_flags = gpu_obj.mesh_settings.y;
    const int4 old_grid = gpu_obj.bake_grid;
    const int4 old_cgrid = gpu_obj.bake_coarse_grid;
    gpu_obj.mesh_settings.y &= ~SDF_LP_MESH_FLAG_BAKED;
    gpu_obj.bake_origin = float4(0.0f);
    gpu_obj.bake_params = float4(0.0f, 0.0f, 0.0f, 0.0f);
    gpu_obj.bake_grid = int4(0);
    gpu_obj.bake_coarse_origin = float4(0.0f);
    gpu_obj.bake_coarse_grid = int4(0);
    if (const MeshBakeRecord *rec = bake_records_.lookup_ptr(key)) {
      if (rec->live.ready) {
        gpu_obj.mesh_settings.y |= SDF_LP_MESH_FLAG_BAKED;
        gpu_obj.bake_origin = float4(rec->live.origin, rec->live.voxel_size);
        gpu_obj.bake_params = float4(rec->live.band, 0.0f, 0.0f, 0.0f);
        gpu_obj.bake_grid = int4(rec->live.res, rec->live.base);
        if (rec->live.cready && rec->live.cres.x > 0) {
          gpu_obj.bake_coarse_origin = float4(rec->live.corigin, rec->live.cvoxel);
          gpu_obj.bake_coarse_grid = int4(rec->live.cres, rec->live.cbase);
        }
      }
    }
    return gpu_obj.mesh_settings.y != old_flags || gpu_obj.bake_grid != old_grid ||
           gpu_obj.bake_coarse_grid != old_cgrid;
  }

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

 public:
  SdfInstanceBase() {}

  blender::StringRefNull name_get() override
  {
    return "SDF";
  }

  void init() final
  {
    draw_ctx_ = DRW_context_get();

    sync_sdf_settings();

    ensure_engine_shaders();
  }

  /* -------------------------------------------------------------------- */
  /** \name Per-engine hooks (classic vs. Lipschitz pruning)
   * \{ */

  /** Shaders this engine needs (indices into the static shader cache). */
  virtual Span<const int> engine_shader_list() const = 0;

  /** Engine-specific compute pipeline, run when a recompute is needed.
   * Sets compute_valid_ on success. */
  virtual void draw_trace_pipeline(bool profiling) = 0;

  /** Called at the end of end_sync (LP: rebuild the CSG tree). */
  virtual void sync_extra() {}
  /** Called from sync_sdf_settings (LP: read its viewport settings). */
  virtual void sync_engine_settings(const View3DShading & /*s*/) {}
  /** Called at the end of upload_objects (LP: upload the CSG tree buffers). */
  virtual void upload_extra() {}
  /** Called before the compute dispatch (LP: resolve grid AABB / dirty flag). */
  virtual void pre_trace_hook() {}
  /** Fold engine-specific settings into the shading-change hash. */
  virtual void hash_shading_extra(uint64_t & /*sh*/) {}
  /** Effective lighting for the shade pass (LP debug modes force pass-through). */
  virtual int effective_lighting(int lighting)
  {
    return lighting;
  }

  /** Kick async compilation of this engine's shaders (no-op once scheduled). */
  void ensure_engine_shaders()
  {
    sdf_shaders_ensure_list(engine_shader_list());
  }

  /** True when all of this engine's shaders finished (or failed) compiling. */
  bool engine_shaders_ready()
  {
    const Span<const int> list = engine_shader_list();
    return sdf_shaders_count_ready(list) == int(list.size());
  }

  /** \} */

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
    mesh_color_gpu_data_.clear();
    mesh_bake_info_.clear();
    object_mesh_keys_.clear();
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
    const SDFMeshPayload *mesh_payload = nullptr;
    const unsigned int *mesh_corner_colors = nullptr;
    if (ob->type == OB_MESH) {
      SDFMeshRuntimeSnapshot snapshot;
      if (!BKE_sdf_mesh_runtime_snapshot(*ob, snapshot)) {
        return;
      }
      mesh_transforms_.append(ob->object_to_world());
      live_mesh_payloads_.append(snapshot.payload);
      const SDFMeshPayload &payload = *snapshot.payload;
      mesh_payload = snapshot.payload.get();
      mesh_corner_colors = payload.corner_colors;
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
      /* Bake record key: the ORIGINAL object — stable across depsgraph
       * rebuilds. Keying by the payload pointer (as before) lost the bake
       * record every time the payload was rebuilt (edit-mode entry/exit,
       * modifier re-evaluation), making the mesh vanish until the fresh
       * bake completed. */
      mesh_payload_key = DEG_get_original(ob);
    }
    else if (ob->type == OB_SDF) {
      sdf_data = id_cast<const SDF *>(ob->data);
      mesh_payload_key = sdf_data;
      mesh_corner_colors = sdf_data->mesh_corner_colors;
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

    /* Text: tessellate glyph outlines once (runtime-cached); used for the
     * AABB and the polygon SSBO ingestion below. Empty text renders nothing. */
    const SDFTextContours *text_contours = nullptr;
    if (sdf_data->sdf_type == SDF_TYPE_TEXT) {
      text_contours = BKE_sdf_text_get_contours(ob);
      if (text_contours == nullptr) {
        return;
      }
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
        const size_t new_color_record_count = mesh_color_gpu_data_.size() +
                                              (mesh_corner_colors != nullptr ?
                                                   size_t(sdf_data->mesh_triangle_count) :
                                                   1);
        if (new_record_count * sizeof(uint4) > max_ssbo_size ||
            new_color_record_count * sizeof(uint4) > max_ssbo_size)
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
        /* Corner colors, reordered identically to the triangles above: one
         * uint4 per triangle (xyz = 3 packed RGBA8 corner colors, w = 0), or a
         * single white record when the mesh has no active color attribute. */
        offsets.color_start = int(mesh_color_gpu_data_.size());
        if (mesh_corner_colors != nullptr) {
          for (int i = 0; i < sdf_data->mesh_triangle_count; i++) {
            mesh_color_gpu_data_.append(uint4(mesh_corner_colors[3 * i],
                                              mesh_corner_colors[3 * i + 1],
                                              mesh_corner_colors[3 * i + 2],
                                              0u));
          }
        }
        else {
          mesh_color_gpu_data_.append(uint4(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0u));
        }
        mesh_offsets_.add(mesh_payload_key, offsets);
      }

      /* Bake inputs for the LP mesh volume bake (sdf_lp_engine.cc). */
      MeshBakeInfo bake_info;
      bake_info.revision = (mesh_payload != nullptr) ? mesh_payload->revision :
                                                       uint64_t(sdf_data->mesh_data_version);
      bake_info.bounds_min = float3(sdf_data->mesh_bounds_min);
      bake_info.bounds_max = float3(sdf_data->mesh_bounds_max);
      bake_info.has_colors = mesh_corner_colors != nullptr;
      mesh_bake_info_.add_overwrite(mesh_payload_key, bake_info);

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

      /* Baked volume fast path (GPU-only; the CPU evaluator in
       * sdf_cpu_eval.hh always stays analytic): flag the object and fill the
       * baked grid fields from the record's live grid. The live grid stays
       * attached while a rebake is in flight; update_mesh_bakes() re-resolves
       * these fields after any flip and re-uploads the object buffer, so the
       * pool base uploaded here never goes stale mid-frame. */
      apply_baked_fields(gpu_obj, mesh_payload_key);
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
        case SDF_TYPE_TEXT: {
          float ext_x = math::max(fabsf(text_contours->bounds_min.x),
                                  fabsf(text_contours->bounds_max.x));
          float ext_y = math::max(fabsf(text_contours->bounds_min.y),
                                  fabsf(text_contours->bounds_max.y));
          float stroke_pad = sdf_data->text_thickness * 0.5f + sdf_data->text_corner;
          local_extent = float3(ext_x + stroke_pad, ext_y + stroke_pad, sz.z);
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
    else if (sdf_data->sdf_type == SDF_TYPE_POLYGON || sdf_data->sdf_type == SDF_TYPE_TEXT) {
      const bool is_text = (sdf_data->sdf_type == SDF_TYPE_TEXT);
      float poly_taper = sdf_data->polygon_taper;
      gpu_obj.polygon_point_start = int(polygon_points_.size());
      gpu_obj.polygon_point_count = 0;

      /* Per-axis XY scale so S+X / S+Y stretch the polygon */
      float corner_scale = math::min(scale.x, scale.y);

      /* Closed contours with per-point corner radii and optional quadratic
       * arc edges. The DNA polygon is a single straight-edge contour
       * (optionally line-expanded); text contributes one contour per glyph
       * loop with exact quadratic arcs (holes work via opposite winding). */
      struct IngestContour {
        Vector<float2> pts;
        Vector<float2> ctrls;
        Vector<char> is_arc;
        Vector<float> crn;
      };
      Vector<IngestContour> contours;
      if (is_text) {
        for (const SDFTextContour &src : text_contours->contours) {
          IngestContour ct;
          const int n = int(src.points.size());
          for (int i = 0; i < n; i++) {
            ct.pts.append(float2(src.points[i].x * scale.x, src.points[i].y * scale.y));
            ct.ctrls.append(float2(src.ctrls[i].x * scale.x, src.ctrls[i].y * scale.y));
            ct.is_arc.append(src.is_arc[i]);
            /* Corner rounding only at original font knots between two
             * straight edges (arc joints are already smooth). */
            const bool roundable = (src.is_knot[i] != 0) && (src.is_arc[i] == 0) &&
                                   (src.is_arc[(i - 1 + n) % n] == 0);
            ct.crn.append(roundable ? sdf_data->text_corner * corner_scale : 0.0f);
          }
          contours.append(std::move(ct));
        }
      }
      else {
        IngestContour ct;
        Vector<float2> &pts = ct.pts;
        Vector<float> &crn = ct.crn;
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
        }

        ct.is_arc = Vector<char>(ct.pts.size(), char(0));
        contours.append(std::move(ct));
      }

      float max_corner = 0.0f;

      /* Build each contour's GPU edges into a temp list, then serialize
       * either flat (contour headers + edges, cheap for few edges) or as a
       * 2D edge BVH (sub-linear for whole paragraphs) — see sdPolygon2D and
       * sdPolygon2DBVH in sdf_lib.glsl. */
      struct ContourEdges {
        Vector<SDFPolygonPointGPU> edges;
        Vector<float2> leaf_min;
        Vector<float2> leaf_max;
        float pad;
      };
      Vector<ContourEdges> built;
      int total_edges = 0;

      for (IngestContour &ct : contours) {
        Vector<float2> &pts = ct.pts;
        Vector<float> &crn = ct.crn;
        int pc = int(pts.size());
        if (pc < 3) {
          continue;
        }

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
        ContourEdges ce;
        ce.pad = 1e-4f;
        for (int i = 0; i < pc; i++) {
          ce.pad = math::max(ce.pad, crn[i]);
        }

        for (int i = 0; i < pc; i++) {
          int j = (i + 1) % pc;
          int ip = (i - 1 + pc) % pc;

          /* Quadratic arc edge (glyph outlines): vertex/control/end packing,
           * corner arcs only exist between straight edges (crn == 0 here). */
          if (ct.is_arc[i]) {
            SDFPolygonPointGPU gpu_pt = {};
            gpu_pt.vi_edge = float4(pts[i].x, pts[i].y, ct.ctrls[i].x, ct.ctrls[i].y);
            gpu_pt.arc_data = float4(pts[j].x, pts[j].y, 0.0f, 0.0f);
            gpu_pt.arc_bounds = float4(1.0f, 0.0f, 0.0f, -1.0f);
            ce.edges.append(gpu_pt);
            float2 mn = math::min(pts[i], math::min(ct.ctrls[i], pts[j])) - float2(ce.pad);
            float2 mx = math::max(pts[i], math::max(ct.ctrls[i], pts[j])) + float2(ce.pad);
            ce.leaf_min.append(mn);
            ce.leaf_max.append(mx);
            continue;
          }

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

          ce.edges.append(gpu_pt);
          float2 mn = math::min(pts[i], pts[j]) - float2(ce.pad);
          float2 mx = math::max(pts[i], pts[j]) + float2(ce.pad);
          ce.leaf_min.append(mn);
          ce.leaf_max.append(mx);
        }
        total_edges += int(ce.edges.size());
        built.append(std::move(ce));
      }

      /* Serialize: flat contour layout for few edges, 2D edge BVH for many. */
      if (total_edges < 24) {
        for (ContourEdges &ce : built) {
          /* Contour header: padded AABB + edge count for shader-side culling. */
          float2 cmin(1e30f), cmax(-1e30f);
          for (int i = 0; i < ce.leaf_min.size(); i++) {
            cmin = math::min(cmin, ce.leaf_min[i]);
            cmax = math::max(cmax, ce.leaf_max[i]);
          }
          SDFPolygonPointGPU header = {};
          header.vi_edge = float4(cmin.x, cmin.y, 0.0f, 0.0f);
          header.arc_data = float4(cmax.x, cmax.y, 0.0f, 0.0f);
          header.arc_bounds = float4(float(ce.edges.size()), 0.0f, 0.0f, -2.0f);
          polygon_points_.append(header);
          gpu_obj.polygon_point_count++;
          for (const SDFPolygonPointGPU &e : ce.edges) {
            polygon_points_.append(e);
            gpu_obj.polygon_point_count++;
          }
        }
      }
      else if (total_edges > 0) {
        /* Exact 2D SDF of the built contours (straight + quadratic arc
         * edges), used to bake the coarse far-field grid. Mirrors
         * sdPolygon2D in sdf_lib.glsl (including its winding rule). */
        auto eval_d2d_exact = [&](float2 p) -> float {
          float d = 1e20f;
          int winding = 0;
          for (const IngestContour &ct : contours) {
            const int pc = int(ct.pts.size());
            for (int i = 0; i < pc; i++) {
              const int j = (i + 1) % pc;
              const float2 &a = ct.pts[i];
              const float2 &c = ct.pts[j];
              if (ct.is_arc[i]) {
                const float2 &b = ct.ctrls[i];
                /* Distance: coarse samples + golden-section refine. */
                auto quad_pt = [&](float t) -> float2 {
                  const float u = 1.0f - t;
                  return a * (u * u) + b * (2.0f * u * t) + c * (t * t);
                };
                float best_t = 0.0f, best_d = 1e30f;
                for (int k = 0; k <= 16; k++) {
                  const float t = float(k) / 16.0f;
                  const float dd = math::distance_squared(quad_pt(t), p);
                  if (dd < best_d) {
                    best_d = dd;
                    best_t = t;
                  }
                }
                float lo = math::max(0.0f, best_t - 1.0f / 16.0f);
                float hi = math::min(1.0f, best_t + 1.0f / 16.0f);
                for (int it = 0; it < 16; it++) {
                  const float m1 = lo + (hi - lo) * 0.381966f;
                  const float m2 = hi - (hi - lo) * 0.381966f;
                  if (math::distance_squared(quad_pt(m1), p) <
                      math::distance_squared(quad_pt(m2), p))
                  {
                    hi = m2;
                  }
                  else {
                    lo = m1;
                  }
                }
                best_d = math::min(
                    best_d, math::distance_squared(quad_pt(0.5f * (lo + hi)), p));
                d = math::min(d, sqrtf(best_d));

                /* Winding: roots of the y-component (same rule as the shader). */
                const float y0 = math::min(a.y, math::min(b.y, c.y));
                const float y1 = math::max(a.y, math::max(b.y, c.y));
                if (p.y < y0 || p.y > y1) {
                  continue;
                }
                const float A = a.y - 2.0f * b.y + c.y;
                const float B = 2.0f * (b.y - a.y);
                const float C = a.y - p.y;
                float roots[2];
                int nroots = 0;
                if (fabsf(A) < 1e-4f) {
                  if (fabsf(B) > 1e-12f) {
                    roots[nroots++] = -C / B;
                  }
                }
                else {
                  const float disc = B * B - 4.0f * A * C;
                  if (disc > 0.0f) {
                    const float s = sqrtf(disc);
                    roots[nroots++] = (-B - s) / (2.0f * A);
                    roots[nroots++] = (-B + s) / (2.0f * A);
                  }
                }
                for (int r = 0; r < nroots; r++) {
                  const float t = roots[r];
                  if (t < 0.0f || t > 1.0f) {
                    continue;
                  }
                  const float u = 1.0f - t;
                  const float2 q = quad_pt(t);
                  const float2 v = 2.0f * (u * (b - a) + t * (c - b));
                  const float cr = v.x * (p.y - q.y) - v.y * (p.x - q.x);
                  if (v.y > 0.0f && p.y != c.y && cr > 0.0f) {
                    winding++;
                  }
                  if (v.y < 0.0f && p.y != a.y && cr < 0.0f) {
                    winding--;
                  }
                }
              }
              else {
                const float2 e = c - a;
                const float2 w = p - a;
                const float ee = math::dot(e, e);
                if (ee > 1e-20f) {
                  const float2 bq = w - e * math::clamp(math::dot(w, e) / ee, 0.0f, 1.0f);
                  d = math::min(d, math::length(bq));
                }
                const float cross = e.x * w.y - e.y * w.x;
                if (a.y <= p.y && c.y > p.y && cross > 0.0f) {
                  winding++;
                }
                if (a.y > p.y && c.y <= p.y && cross < 0.0f) {
                  winding--;
                }
              }
            }
          }
          return (winding != 0) ? -d : d;
        };
        /* 2D edge BVH: node entries (arc_bounds.w == -3) with child indices
         * into the same array; leaves are the edges built above. Node
         * indices are serialized before leaves; the uber-header at
         * polygon_point_start points at the root. */
        struct NodeRec {
          float2 mn, mx;
          /* >= 0: index into nodes; < 0: -(leaf index) - 1. */
          int left, right;
        };
        Vector<NodeRec> nodes;
        Vector<int> order(total_edges);
        for (int i = 0; i < total_edges; i++) {
          order[i] = i;
        }

        /* Flatten leaves across contours. */
        Vector<const SDFPolygonPointGPU *> leaf_ptrs(total_edges);
        Vector<float2> leaf_mn(total_edges), leaf_mx(total_edges);
        {
          int li = 0;
          for (const ContourEdges &ce : built) {
            for (int i = 0; i < ce.edges.size(); i++) {
              leaf_ptrs[li] = &ce.edges[i];
              leaf_mn[li] = ce.leaf_min[i];
              leaf_mx[li] = ce.leaf_max[i];
              li++;
            }
          }
        }

        std::function<int(int, int)> build = [&](int lo, int hi) -> int {
          /* Single leaf: no node, reference the leaf directly (a node with two
           * identical children would double-count its winding). */
          if (hi - lo == 1) {
            return -order[lo] - 1;
          }
          float2 mn(1e30f), mx(-1e30f);
          for (int i = lo; i < hi; i++) {
            mn = math::min(mn, leaf_mn[order[i]]);
            mx = math::max(mx, leaf_mx[order[i]]);
          }
          NodeRec rec;
          rec.mn = mn;
          rec.mx = mx;
          const int count = hi - lo;
          if (count == 2) {
            rec.left = -order[lo] - 1;
            rec.right = -order[lo + 1] - 1;
          }
          else {
            const int axis = (mx.x - mn.x) >= (mx.y - mn.y) ? 0 : 1;
            const int mid = (lo + hi) / 2;
            std::nth_element(order.begin() + lo,
                             order.begin() + mid,
                             order.begin() + hi,
                             [&](int a, int b) {
                               return (leaf_mn[a][axis] + leaf_mx[a][axis]) <
                                      (leaf_mn[b][axis] + leaf_mx[b][axis]);
                             });
            rec.left = build(lo, mid);
            rec.right = build(mid, hi);
          }
          nodes.append(rec);
          return int(nodes.size()) - 1;
        };
        const int root = build(0, total_edges);

        const int base = gpu_obj.polygon_point_start;
        const int num_nodes = int(nodes.size());
        SDFPolygonPointGPU uber = {};
        uber.arc_data = float4(float(base + 1 + root), 0.0f, 0.0f, 0.0f);
        uber.arc_bounds = float4(0.0f, 0.0f, 0.0f, -3.0f);
        polygon_points_.append(uber);
        gpu_obj.polygon_point_count++;

        for (const NodeRec &n : nodes) {
          SDFPolygonPointGPU e = {};
          e.vi_edge = float4(n.mn.x, n.mn.y, 0.0f, 0.0f);
          e.arc_data = float4(n.mx.x, n.mx.y, 0.0f, 0.0f);
          auto remap = [&](int ref) -> float {
            if (ref >= 0) {
              return float(base + 1 + ref);
            }
            return float(base + 1 + num_nodes + (-ref - 1));
          };
          e.arc_bounds = float4(remap(n.left), remap(n.right), 0.0f, -3.0f);
          polygon_points_.append(e);
          gpu_obj.polygon_point_count++;
        }
        for (int i = 0; i < total_edges; i++) {
          polygon_points_.append(*leaf_ptrs[i]);
          gpu_obj.polygon_point_count++;
        }

        /* Coarse far-field grid: conservative signed-distance values on a
         * 64x64 cell grid (12 values packed per entry). The shader uses the
         * cell value only when it provably cannot affect any visible surface
         * (beyond the silhouette + bevel/edge/outline/blend reach), otherwise
         * it falls back to the exact BVH traversal — quality is untouched,
         * far-field cost is O(1) regardless of the edge count. */
        const int GRID_RES = 64;
        float2 gmn(1e30f), gmx(-1e30f);
        for (int i = 0; i < total_edges; i++) {
          gmn = math::min(gmn, leaf_mn[i]);
          gmx = math::max(gmx, leaf_mx[i]);
        }
        gmn -= float2(max_corner + 1e-3f);
        gmx += float2(max_corner + 1e-3f);
        const float cell = math::max(gmx.x - gmn.x, gmx.y - gmn.y) / float(GRID_RES);
        const float half_diag = cell * 0.70710678f;
        const int grid_first = int(polygon_points_.size());
        {
          SDFPolygonPointGPU ge = {};
          int slot = 0;
          auto flush = [&]() {
            if (slot > 0) {
              polygon_points_.append(ge);
              gpu_obj.polygon_point_count++;
              ge = {};
              slot = 0;
            }
          };
          for (int gy = 0; gy < GRID_RES; gy++) {
            for (int gx = 0; gx < GRID_RES; gx++) {
              const float2 p(gmn.x + (float(gx) + 0.5f) * cell,
                             gmn.y + (float(gy) + 0.5f) * cell);
              const float dc = eval_d2d_exact(p);
              const float v = (dc >= 0.0f) ? (dc - half_diag) : (dc + half_diag);
              if (slot < 4) {
                ge.vi_edge[slot] = v;
              }
              else if (slot < 8) {
                ge.arc_data[slot - 4] = v;
              }
              else {
                ge.arc_bounds[slot - 8] = v;
              }
              if (++slot == 12) {
                flush();
              }
            }
          }
          flush();
        }
        /* Patch the uber-header with the grid metadata. */
        polygon_points_[base].arc_data.y = float(grid_first);
        polygon_points_[base].arc_data.z = float(GRID_RES);
        polygon_points_[base].arc_data.w = float(GRID_RES);
        polygon_points_[base].arc_bounds.x = gmn.x;
        polygon_points_[base].arc_bounds.y = gmn.y;
        polygon_points_[base].arc_bounds.z = cell;
      }

      gpu_obj.box_corners = float4(max_corner, 0.0f, 0.0f, 0.0f);
      gpu_obj.box_edges = float4(sdf_data->polygon_edge_top,
                                 sdf_data->polygon_edge_bottom,
                                 math::max(poly_taper, 0.0f),
                                 math::max(-poly_taper, 0.0f));
      gpu_obj.box_modes = int4(0, sdf_data->polygon_edge_mode, 0, 0);

      if (is_text) {
        /* Text evaluates through the polygon GPU path (multi-contour outlines
         * with the non-zero winding rule). box_corners.y carries the optional
         * outline stroke half-width (0 = filled glyphs). */
        gpu_obj.sdf_type = SDF_GPU_TYPE_POLYGON;
        float half_thickness = sdf_data->text_thickness * 0.5f * corner_scale;
        gpu_obj.box_corners.y = half_thickness;
        gpu_obj.box_modes.z = (half_thickness > 0.0f) ? 1 : 0;
      }
    }
    else if (sdf_data->sdf_type == SDF_TYPE_TORUS) {
      float angle_rad = sdf_data->torus_angle;
      float half_rad = angle_rad * 0.5f;
      gpu_obj.box_corners = float4(sinf(half_rad), cosf(half_rad), 0.0f, 0.0f);
      gpu_obj.box_edges = float4(0.0f);
      gpu_obj.box_modes = int4(0, 0, 0, (angle_rad < (float(M_PI) * 2.0f) - 0.001f) ? 1 : 0);
    }
    else if (sdf_data->sdf_type == SDF_TYPE_CYLINDER) {
      float cyl_taper = sdf_data->cylinder_taper;
      gpu_obj.box_corners = float4(0.0f);
      gpu_obj.box_edges = float4(sdf_data->cylinder_edge_top,
                                 sdf_data->cylinder_edge_bottom,
                                 math::max(cyl_taper, 0.0f),
                                 math::max(-cyl_taper, 0.0f));
      gpu_obj.box_modes = int4(0, sdf_data->cylinder_edge_mode, 0, 0);
    }
    else if (sdf_data->sdf_type == SDF_TYPE_CONE) {
      gpu_obj.box_corners = float4(0.0f);
      gpu_obj.box_edges = float4(sdf_data->cone_edge_top,
                                 sdf_data->cone_edge_bottom,
                                 0.0f, 0.0f);
      gpu_obj.box_modes = int4(0, 0, 0, 0);
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
        case SDF_TYPE_TEXT: {
          float ext_x = math::max(fabsf(text_contours->bounds_min.x),
                                  fabsf(text_contours->bounds_max.x));
          float ext_y = math::max(fabsf(text_contours->bounds_min.y),
                                  fabsf(text_contours->bounds_max.y));
          float stroke_pad = sdf_data->text_thickness * 0.5f + sdf_data->text_corner;
          base_local_extent = float3(ext_x + stroke_pad, ext_y + stroke_pad, sz.z);
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
          case SDF_TYPE_TEXT: {
            float ext_x = math::max(fabsf(text_contours->bounds_min.x),
                                    fabsf(text_contours->bounds_max.x)) *
                          copy_scale.x;
            float ext_y = math::max(fabsf(text_contours->bounds_min.y),
                                    fabsf(text_contours->bounds_max.y)) *
                          copy_scale.y;
            float stroke_pad = sdf_data->text_thickness * 0.5f + sdf_data->text_corner;
            copy_local_extent = float3(ext_x + stroke_pad + copy_pad,
                                       ext_y + stroke_pad + copy_pad,
                                       copy_sz.z + copy_pad);
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
      object_mesh_keys_.append(copy_obj.sdf_type == SDF_TYPE_MESH ? mesh_payload_key : nullptr);
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
          Vector<Object *> sorted_ptrs(n);
          Vector<const void *> sorted_mesh_keys(n);
          Vector<int> old_to_new(n);
          for (int new_idx = 0; new_idx < n; new_idx++) {
            int old_idx = sort_pairs[new_idx].second;
            sorted[new_idx] = objects_[old_idx];
            sorted_ptrs[new_idx] = object_ptrs_[old_idx];
            sorted_mesh_keys[new_idx] = object_mesh_keys_[old_idx];
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

          /* Keep the parallel arrays aligned with the sorted objects_:
           * update_mesh_bakes pairs objects_[i] with object_ptrs_[i] /
           * object_mesh_keys_[i]; leaving them in sync order resolved every
           * mesh's baked fields against the WRONG bake record whenever the
           * sort reordered (groups / custom sdf_index), making meshes
           * sample a foreign bake or vanish until a rebake. Done after the
           * mapping loops above, which index object_ptrs_ pre-sort. */
          object_ptrs_ = std::move(sorted_ptrs);
          object_mesh_keys_ = std::move(sorted_mesh_keys);
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

        /* Compact the parallel arrays the same way (they were reordered
         * along with objects_ by the sort above; update_mesh_bakes pairs
         * objects_[i] with object_ptrs_[i] / object_mesh_keys_[i]). */
        Vector<Object *> compact_ptrs;
        Vector<const void *> compact_mesh_keys;
        compact_ptrs.reserve(visible_count);
        compact_mesh_keys.reserve(visible_count);
        for (int i = 0; i < int(objects_.size()); i++) {
          if (obj_remap[i] >= 0) {
            compact_ptrs.append(object_ptrs_[i]);
            compact_mesh_keys.append(object_mesh_keys_[i]);
          }
        }
        object_ptrs_ = std::move(compact_ptrs);
        object_mesh_keys_ = std::move(compact_mesh_keys);

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

    sync_extra();

    needs_upload_ = true;

  }

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
      sdf_compile_status_clear();
      clear_exported_state();
      return;
    }

    ensure_engine_shaders();
    if (!engine_shaders_ready()) {
      /* Shaders are still compiling in the background worker: skip the compute
       * passes, keep showing the previous frame, report progress, and retry on
       * the next redraw. */
      ensure_engine_shaders();
      const Span<const int> list = engine_shader_list();
      sdf_compile_status_set(list);
      DRW_viewport_request_redraw();

      ensure_compute_targets();
      DRW_submission_start();
      if (sdf_shader_is_ready(SH_BLIT)) {
        GPU_debug_group_begin("SDF Blit");
        draw_blit();
        GPU_debug_group_end();
        if (sdf_shader_is_ready(SH_FXAA)) {
          GPU_debug_group_begin("SDF FXAA");
          draw_fxaa();
          GPU_debug_group_end();
        }
      }
      DRW_submission_end();
      return;
    }
    sdf_compile_status_clear();

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
      hash_shading_extra(sh);
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
      /* Corner colors ride the same dirty flag: a pure recolor leaves
       * mesh_gpu_data_ byte-identical, so hash the color records too. */
      const uint8_t *color_bytes = reinterpret_cast<const uint8_t *>(
          mesh_color_gpu_data_.data());
      for (size_t i = 0; i < mesh_color_gpu_data_.size() * sizeof(uint4); i++) {
        mesh_data_hash ^= color_bytes[i];
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

    pre_trace_hook();

    bool res_changed = (render_size_ != prev_render_size_);
    bool force_compute = (G.debug & G_DEBUG_GPU_SDF) != 0;
    /* bake_in_flight(): keep the pipeline running while a progressive mesh
     * bake has slices left — a static scene would otherwise never re-run
     * the pipeline, stalling the bake (and the mesh's first appearance
     * after conversion) until the view changes. */
    bool need_compute = force_compute || !compute_valid_ || scene_changed_ || view_changed_ ||
                         res_changed || shading_changed || bake_in_flight();

    if (need_compute) {
      draw_trace_pipeline(profiling);
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
        (adaptive_resolution_ && render_size_ != texture_size_) || bake_in_flight())
    {
      /* The bake_in_flight() arm keeps redraws coming until every
       * progressive mesh bake has flipped ready (see need_compute above). */
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
    s_bake_dist_ssbo = bake_dist_ssbo_;
    s_bake_nrm_ssbo = bake_nrm_ssbo_;
    s_bake_col_ssbo = bake_col_ssbo_;
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

 protected:
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
    smooth_upscale_ = (U.sdf_smooth_upscale != 0);
    use_frustum_cull_ = s.sdf_frustum_cull != 0;

    sync_engine_settings(s);

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

    /* Corner color SSBO: same lifetime/dirty conditions as mesh data. */
    const int mesh_color_count = math::max(int(mesh_color_gpu_data_.size()), 1);
    if (mesh_color_ssbo_ != nullptr && mesh_color_ssbo_count_ != mesh_color_count) {
      GPU_storagebuf_free(mesh_color_ssbo_);
      mesh_color_ssbo_ = nullptr;
    }
    if (mesh_color_ssbo_ == nullptr) {
      const uint4 dummy(0xFFFFFFFFu);
      mesh_color_ssbo_ = GPU_storagebuf_create_ex(mesh_color_count * sizeof(uint4),
                                                  mesh_color_gpu_data_.is_empty() ?
                                                      &dummy :
                                                      mesh_color_gpu_data_.data(),
                                                  GPU_USAGE_DYNAMIC,
                                                  "sdf_mesh_color_ssbo");
      mesh_color_ssbo_count_ = mesh_color_count;
    }
    else if (mesh_data_changed_ && !mesh_color_gpu_data_.is_empty()) {
      GPU_storagebuf_update(mesh_color_ssbo_, mesh_color_gpu_data_.data());
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

    upload_extra();
  }

  /* -------------------------------------------------------------------- */
  /** \name Mesh volume bake manager
   *
   * Shared by both engines: (re)bakes every live mesh payload into the voxel
   * pools; runtime sampling is the SDF_LP_MESH_FLAG_BAKED fast path in
   * sdf_mesh_lib.glsl (classic) and sdf_lp_common.glsl (LP). Self-contained
   * and idempotent.
   * \{ */

  /* Total bake pool budget in voxels (dist+col 1 uint/voxel, nrm 2: 16
   * bytes/voxel -> 1 GiB at 64 Mvoxels). The previous cap derived from
   * GPU_max_storage_buffer_size allowed multi-gigabyte pools and VRAM OOM
   * crashes at high resolutions. */
  static constexpr int64_t kMaxBakePoolVoxels = int64_t(64) << 20;
  /* Progressive bake budget: voxels baked per frame. A full 256^3 bake is
   * spread over ~8 frames; objects stay analytic until their bake completes,
   * so editing a mesh never stalls the viewport on one giant dispatch. */
  static constexpr int64_t kBakeVoxelsPerFrame = int64_t(2) << 20;

  /* Grow the pools until `needed` voxels fit (doubling), recreating the
   * SSBOs. Returns true when the pools were recreated — the new buffers lost
   * all contents, so every ready record must be re-dispatched. On allocation
   * failure the old pools are kept (returns false; capacity unchanged). */
  bool bake_pools_ensure(int64_t needed)
  {
    if (needed <= bake_pool_capacity_) {
      return false;
    }
    int64_t new_capacity = math::max(bake_pool_capacity_, int64_t(1) << 18);
    while (new_capacity < needed) {
      new_capacity *= 2;
    }
    gpu::StorageBuf *dist = GPU_storagebuf_create_ex(
        new_capacity * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_bake_dist");
    gpu::StorageBuf *nrm = GPU_storagebuf_create_ex(
        2 * new_capacity * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_bake_nrm");
    gpu::StorageBuf *col = GPU_storagebuf_create_ex(
        new_capacity * sizeof(uint32_t), nullptr, GPU_USAGE_DYNAMIC, "sdf_bake_col");
    if (dist == nullptr || nrm == nullptr || col == nullptr) {
      if (dist) GPU_storagebuf_free(dist);
      if (nrm) GPU_storagebuf_free(nrm);
      if (col) GPU_storagebuf_free(col);
      CLOG_WARN(&LOG,
                "SDF mesh bake: failed to allocate %d Mvoxel pools; keeping the "
                "current pools. Lower the mesh voxel resolution",
                int(new_capacity >> 20));
      return false;
    }
    if (bake_dist_ssbo_) GPU_storagebuf_free(bake_dist_ssbo_);
    if (bake_nrm_ssbo_) GPU_storagebuf_free(bake_nrm_ssbo_);
    if (bake_col_ssbo_) GPU_storagebuf_free(bake_col_ssbo_);
    bake_dist_ssbo_ = dist;
    bake_nrm_ssbo_ = nrm;
    bake_col_ssbo_ = col;
    bake_pool_capacity_ = new_capacity;
    return true;
  }

  /* Dispatch one z-slice range of the bake for a single grid: rewrites
   * voxels [0,res) x [0,res) x [z_begin, z_begin+z_count) of the grid's
   * range (idempotent). dist_only skips the normal/color pool writes
   * (coarse far-field level). The shader must be bound. */
  void dispatch_mesh_bake(gpu::Shader *sh,
                          int3 res,
                          int base,
                          float3 origin,
                          float voxel_size,
                          float band,
                          bool dist_only,
                          const MeshOffsets &offsets,
                          bool has_colors,
                          int z_begin,
                          int z_count)
  {
    const int4 mesh_data(offsets.vertex_start,
                         offsets.triangle_start,
                         offsets.triangle_count,
                         offsets.bvh_start);
    const int4 res_and_base(res.x, res.y, res.z, base);
    GPU_shader_uniform_int_ex(
        sh, GPU_shader_get_uniform(sh, "mesh_data"), 4, 1, &mesh_data.x);
    GPU_shader_uniform_1i(sh, "mesh_node_count", offsets.bvh_count);
    GPU_shader_uniform_1i(sh, "color_start", offsets.color_start);
    GPU_shader_uniform_int_ex(
        sh, GPU_shader_get_uniform(sh, "res_and_base"), 4, 1, &res_and_base.x);
    GPU_shader_uniform_3fv(sh, "origin", origin);
    GPU_shader_uniform_1f(sh, "voxel_size", voxel_size);
    GPU_shader_uniform_1f(sh, "band", band);
    GPU_shader_uniform_1i(sh, "has_colors", has_colors ? 1 : 0);
    GPU_shader_uniform_1i(sh, "z_offset", z_begin);
    GPU_shader_uniform_1i(sh, "dist_only", dist_only ? 1 : 0);
    GPU_compute_dispatch(sh, (res.x + 3) / 4, (res.y + 3) / 4, (z_count + 3) / 4);
  }

  /* True while any mesh bake is still in flight AND can make progress
   * (the pools are large enough for the committed layout — the dispatch
   * gate in update_mesh_bakes). Drives the redraw loop in draw(): the
   * progressive bake only advances inside draw_trace_pipeline, which a
   * static scene never schedules — without this a freshly converted mesh
   * stays invisible until the user moves the view. */
  bool bake_in_flight() const
  {
    if (bake_pool_capacity_ < bake_pool_used_) {
      return false;
    }
    for (const auto &item : bake_records_.items()) {
      if (item.value.work_active) {
        return true;
      }
    }
    return false;
  }

  /* Per-frame update: (re)bake every live mesh payload that has no record
   * yet, whose revision changed, or whose sdf_voxel_resolution setting
   * changed. Called from draw_trace_pipeline, after the shared mesh SSBOs
   * were uploaded (a payload change always flips scene_changed_, which gates
   * both upload_objects and draw_trace_pipeline).
   * Returns true when the baked fields of objects_ changed (records flipped
   * to ready or pool ranges were reassigned): the LP engine must rebuild its
   * CSG tree then (make_leaf captures the baked fields into lp_prims_). */
  bool update_mesh_bakes()
  {
    /* The trace/prune/march shaders declare the bake pools unconditionally
     * (the baked-volume sampler is compiled in even when no object uses it),
     * and on Vulkan every declared SSBO binding must have a buffer bound —
     * binding nothing hits the unreachable-code assert in
     * vk_descriptor_set.cc and writes a null descriptor. Keep minimal pools
     * allocated even with no live meshes. */
    bake_pools_ensure(1);
    if (mesh_gpu_data_.is_empty()) {
      return false;
    }
    gpu::Shader *sh = mesh_bake_sh();
    if (sh == nullptr) {
      return false;
    }

    /* Scene blend reach: the coarse far-field level covers the mesh bounds
     * plus this distance, so CSG blends of any radius up to it see a true
     * distance field (blends beyond it soften instead of breaking). */
    const float reach = max_blend_ + max_shell_distance_;

    /* ---- 1. Flip completed work grids into live. The old live grid is
     * kept in `work` as the SPARE: phase 2 reuses its pool range for the
     * next rebake whose layout matches, so a steady edit stream appends
     * nothing to the pools (appends used to thrash the pool into
     * compaction, which clears the live grids — the edit-mode flicker).
     * ---- */
    for (const auto &item : bake_records_.items()) {
      MeshBakeRecord &rec = item.value;
      if (rec.work_active && rec.work.ready &&
          (rec.work.cres.x == 0 || rec.work.cready))
      {
        std::swap(rec.live, rec.work);
        rec.revision = rec.w_revision;
        rec.voxel_resolution = rec.w_setting;
        rec.reach = rec.w_reach;
        rec.work_active = false;
      }
    }

    /* ---- 2. (Re)start work grids for payloads whose desired grid differs
     * from the live one. ---- */
    bool any_started = false;
    for (int i = 0; i < int(objects_.size()); i++) {
      if (objects_[i].sdf_type != SDF_GPU_TYPE_MESH) {
        continue;
      }
      const void *key = object_mesh_keys_[i];
      if (key == nullptr) {
        continue;
      }
      const MeshBakeInfo *info = mesh_bake_info_.lookup_ptr(key);
      if (info == nullptr || !mesh_offsets_.contains(key)) {
        continue;
      }
      const int setting = std::clamp(object_ptrs_[i]->sdf_voxel_resolution, 0, 256);

      const MeshBakeRecord *rec = bake_records_.lookup_ptr(key);
      /* Live grid up to date: nothing to do. */
      if (rec != nullptr && rec->revision == info->revision &&
          rec->voxel_resolution == setting && rec->reach == reach)
      {
        continue;
      }
      /* Already baking exactly this grid: the progressive dispatch below
       * finishes it; restarting would throw away progress every frame. */
      if (rec != nullptr && rec->work_active && rec->w_revision == info->revision &&
          rec->w_setting == setting && rec->w_reach == reach)
      {
        continue;
      }

      /* Fine grid layout: voxel_size targets R voxels across the largest
       * bounds axis (R = object setting, 64 when 0/auto) with
       * `margin_voxels` of padding voxels on both sides; the grid covers the
       * bounds expanded by (band + voxel_size) on every side,
       * band = 4 * voxel_size. */
      constexpr int margin_voxels = 5;
      const int res_target = std::clamp(setting > 0 ? setting : 64, 2 * margin_voxels + 8, 256);
      const float3 extent = math::max(info->bounds_max - info->bounds_min, float3(1e-6f));
      const float max_extent = math::reduce_max(extent);
      const float voxel_size = max_extent / float(res_target - 2 * margin_voxels - 1);
      const float band = 4.0f * voxel_size;
      const float pad = band + voxel_size;

      MeshBakeRecord &r = bake_records_.lookup_or_add_default(key);
      const int prev_w_setting = r.w_setting;
      const float prev_w_reach = r.w_reach;
      /* Spare range (the pre-flip live grid; see phase 1) available for
       * reuse when the new layout matches — captured before `work` is
       * overwritten below. Meaningless while a bake is in flight. */
      const int3 spare_res = r.work.res;
      const int spare_base = r.work.base;
      const int3 spare_cres = r.work.cres;
      const int spare_cbase = r.work.cbase;
      r.w_revision = info->revision;
      r.w_setting = setting;
      r.w_reach = reach;

      /* Layout stabilization: continuous editing nudges the mesh bounds
       * every stroke; re-deriving the grid from the new bounds each
       * revision would change res/origin/voxel_size every time, defeating
       * the range reuse below and thrashing the append-only pool into
       * compaction (which clears the live grids — the edit-mode flicker).
       * When the new padded bounds still fit inside the current grid (the
       * in-flight work grid, else the live one) at an unchanged resolution
       * setting, keep that grid's layout; relayout only when the bounds
       * outgrow the grid or shrink below half its voxel pitch. */
      int3 work_res = math::clamp(int3(math::ceil((extent + float3(2.0f * pad)) / voxel_size)),
                                  8,
                                  256);
      float3 work_origin = info->bounds_min - float3(pad);
      float work_voxel = voxel_size;
      const MeshBakeGrid *stable = (r.work_active && prev_w_setting == setting) ?
                                       &r.work :
                                   (r.live.ready && r.voxel_resolution == setting) ?
                                       &r.live :
                                       nullptr;
      if (stable != nullptr && stable->res.x > 0 && voxel_size > 0.5f * stable->voxel_size) {
        const float stable_pad = stable->band + stable->voxel_size;
        const float3 fit_min = info->bounds_min - float3(stable_pad);
        const float3 fit_max = info->bounds_max + float3(stable_pad);
        const float3 grid_max = stable->origin + float3(stable->res) * stable->voxel_size;
        const bool fits = fit_min.x >= stable->origin.x && fit_min.y >= stable->origin.y &&
                          fit_min.z >= stable->origin.z && fit_max.x <= grid_max.x &&
                          fit_max.y <= grid_max.y && fit_max.z <= grid_max.z;
        if (fits) {
          work_res = stable->res;
          work_origin = stable->origin;
          work_voxel = stable->voxel_size;
        }
      }
      r.work.res = work_res;
      r.work.origin = work_origin;
      r.work.voxel_size = work_voxel;
      r.work.band = 4.0f * work_voxel;

      /* Coarse far-field level: ~48 cells across the padded bounds, fp32
       * unclamped distance out to the blend reach. Skipped (cres = 0) when
       * the scene has no blends at all. The coarse voxel is capped at 8
       * fine voxels: trilinear interpolation of the coarse field
       * overestimates the true distance near convex features by up to
       * ~(cvoxel^2 / 8) * surface curvature, and at blend-reach-sized
       * voxels that error breaks the 1.5-Lipschitz step bound the LP
       * marcher relies on near the fine -> coarse defer boundary
       * (overshoot -> torn patches in wide blends). Capped at 8 fine
       * voxels the error stays below half the narrow band for any surface
       * the fine grid resolves (>= 4 voxels per curvature radius). */
      if (reach > 0.0f) {
        const float pad_c = reach + r.work.band + work_voxel;
        const float3 extent_c = extent + float3(2.0f * pad_c);
        const float cvoxel = math::max(math::reduce_max(extent_c) / 48.0f, 8.0f * work_voxel);
        r.work.cres = math::clamp(int3(math::ceil(extent_c / cvoxel)), 8, 64);
        /* Center the grid on the bounds: when the 64-cell clamp keeps the
         * grid from covering bounds + reach, coverage stays centered on
         * the mesh instead of being clipped on the max side. */
        r.work.corigin = (info->bounds_min + info->bounds_max) * 0.5f -
                         float3(r.work.cres) * (0.5f * cvoxel);
        r.work.cvoxel = cvoxel;
        /* Coarse layout stabilization (same rationale as the fine grid):
         * reuse the current coarse grid when the padded bounds still fit
         * at an unchanged blend reach. */
        const bool stable_reach = (stable == &r.work) ? (prev_w_reach == reach) :
                                  (stable == &r.live) ? (r.reach == reach) :
                                                        false;
        if (stable_reach && stable->cres.x > 0) {
          const float cpad = reach + stable->band + stable->voxel_size;
          const float3 cfit_min = info->bounds_min - float3(cpad);
          const float3 cfit_max = info->bounds_max + float3(cpad);
          const float3 cgrid_max = stable->corigin + float3(stable->cres) * stable->cvoxel;
          const bool cfits = cfit_min.x >= stable->corigin.x && cfit_min.y >= stable->corigin.y &&
                             cfit_min.z >= stable->corigin.z && cfit_max.x <= cgrid_max.x &&
                             cfit_max.y <= cgrid_max.y && cfit_max.z <= cgrid_max.z;
          if (cfits) {
            r.work.cres = stable->cres;
            r.work.corigin = stable->corigin;
            r.work.cvoxel = stable->cvoxel;
          }
        }
      }
      else {
        r.work.cres = int3(0);
        r.work.cbase = 0;
      }

      /* Pool ranges — the live grid is never rewritten mid-bake. When
       * only the blend reach changed (payload and fine grid identical), the
       * live fine bake doubles as the work fine bake (same for the coarse
       * level when its grid is unchanged), so only the coarse level
       * rebakes. When RETARGETING an in-flight bake (continuous editing),
       * reuse the work ranges. Otherwise reuse the SPARE range (the
       * pre-flip live grid kept in `work`) when its size matches — a fresh
       * append every completed bake would thrash the pool into compaction.
       * The spare is only valid while no bake is in flight and must not
       * alias the live range (the reach-only fast path shares it). */
      const bool content_same = r.live.ready && r.revision == info->revision &&
                                r.voxel_resolution == setting;
      if (content_same && r.live.res == r.work.res) {
        r.work.base = r.live.base;
        r.work.ready = true;
        r.next_z = r.work.res.z;
      }
      else if (rec != nullptr && rec->work_active && rec->work.res == r.work.res) {
        r.work.base = rec->work.base;
        r.work.ready = false;
        r.next_z = 0;
      }
      else if (!r.work_active && spare_res.x > 0 && spare_res == r.work.res &&
               spare_base != r.live.base)
      {
        r.work.base = spare_base;
        r.work.ready = false;
        r.next_z = 0;
      }
      else {
        r.work.base = int(bake_pool_used_);
        bake_pool_used_ += int64_t(r.work.res.x) * r.work.res.y * r.work.res.z;
        r.work.ready = false;
        r.next_z = 0;
      }
      if (r.work.cres.x > 0) {
        if (content_same && r.live.cready && r.live.cres == r.work.cres) {
          r.work.cbase = r.live.cbase;
          r.work.cready = true;
          r.next_cz = r.work.cres.z;
        }
        else if (rec != nullptr && rec->work_active && rec->work.cres == r.work.cres) {
          r.work.cbase = rec->work.cbase;
          r.work.cready = false;
          r.next_cz = 0;
        }
        else if (!r.work_active && spare_cres.x > 0 && spare_cres == r.work.cres &&
                 spare_cbase != r.live.cbase)
        {
          r.work.cbase = spare_cbase;
          r.work.cready = false;
          r.next_cz = 0;
        }
        else {
          r.work.cbase = int(bake_pool_used_);
          bake_pool_used_ += int64_t(r.work.cres.x) * r.work.cres.y * r.work.cres.z;
          r.work.cready = false;
          r.next_cz = 0;
        }
      }
      else {
        r.work.cready = false;
        r.next_cz = 0;
      }
      r.work_active = !(r.work.ready && (r.work.cres.x == 0 || r.work.cready));
      /* Nothing to bake (grids identical to live): flip immediately.
       * Swap (as in phase 1) so the old live grid stays around as the
       * spare for the next rebake. */
      if (!r.work_active) {
        std::swap(r.live, r.work);
        r.revision = r.w_revision;
        r.voxel_resolution = r.w_setting;
        r.reach = r.w_reach;
      }
      any_started = true;
    }

    if (any_started) {
      /* Budget/compaction: when the append-only layout overflows the pool
       * budget, drop records with no live mesh data and repack the survivors
       * from zero (reclaiming abandoned ranges), then mark everything for
       * rebake (the old bases' data is unreachable after the repack).
       * Records that still do not fit are removed; their objects stay
       * invisible. */
      if (bake_pool_used_ > kMaxBakePoolVoxels) {
        Vector<const void *> stale;
        for (const auto &item : bake_records_.items()) {
          if (!mesh_offsets_.contains(item.key)) {
            stale.append(item.key);
          }
        }
        for (const void *key : stale) {
          bake_records_.remove(key);
        }
        int64_t base = 0;
        for (const auto &item : bake_records_.items()) {
          MeshBakeRecord &rec = item.value;
          /* Repack: work range (or live when no work) gets fresh bases;
           * everything must rebake into them. */
          if (rec.work_active) {
            rec.live.ready = false;
            rec.live.cready = false;
          }
          else {
            rec.work = rec.live;
            rec.w_revision = rec.revision;
            rec.w_setting = rec.voxel_resolution;
            rec.w_reach = rec.reach;
            rec.live.ready = false;
            rec.live.cready = false;
            rec.work_active = true;
          }
          rec.work.base = int(base);
          rec.work.ready = false;
          rec.next_z = 0;
          base += int64_t(rec.work.res.x) * rec.work.res.y * rec.work.res.z;
          if (rec.work.cres.x > 0) {
            rec.work.cbase = int(base);
            rec.work.cready = false;
            rec.next_cz = 0;
            base += int64_t(rec.work.cres.x) * rec.work.cres.y * rec.work.cres.z;
          }
        }
        bake_pool_used_ = base;
        while (bake_pool_used_ > kMaxBakePoolVoxels && !bake_records_.is_empty()) {
          /* Drop the largest record until the rest fits the budget. */
          const void *largest_key = nullptr;
          int64_t largest_voxels = -1;
          for (const auto &item : bake_records_.items()) {
            const MeshBakeGrid &g = item.value.work_active ? item.value.work : item.value.live;
            const int64_t v = int64_t(g.res.x) * g.res.y * g.res.z +
                              int64_t(g.cres.x) * g.cres.y * g.cres.z;
            if (v > largest_voxels) {
              largest_voxels = v;
              largest_key = item.key;
            }
          }
          bake_records_.remove(largest_key);
          bake_pool_used_ -= largest_voxels;
        }
        if (!bake_overflow_warned_) {
          bake_overflow_warned_ = true;
          CLOG_WARN(&LOG,
                    "SDF mesh bake: voxel pool budget (%d Mvoxels) exceeded; "
                    "some meshes stay invisible. Lower the mesh voxel "
                    "resolution",
                    int(kMaxBakePoolVoxels >> 20));
        }
      }

      if (bake_pools_ensure(bake_pool_used_)) {
        /* The recreated buffers lost all contents: every record must rebake
         * (progressively — no single-frame stall). Records with no work in
         * flight get their live grid restarted as work; objects go
         * invisible until the rebake lands (regrow is rare). */
        for (const auto &item : bake_records_.items()) {
          MeshBakeRecord &rec = item.value;
          if (!rec.work_active) {
            rec.work = rec.live;
            rec.w_revision = rec.revision;
            rec.w_setting = rec.voxel_resolution;
            rec.w_reach = rec.reach;
            rec.work_active = true;
          }
          rec.work.ready = false;
          rec.work.cready = false;
          rec.next_z = 0;
          rec.next_cz = 0;
          rec.live.ready = false;
          rec.live.cready = false;
        }
      }
    }

    /* Progressive dispatch: advance every work grid by the per-frame voxel
     * budget, one or more z-slices at a time. Skipped while the pools are
     * too small for the committed layout (allocation failure): records stay
     * unready and the objects keep their previous live grid. */
    bool any_unready = false;
    for (const auto &item : bake_records_.items()) {
      if (item.value.work_active) {
        any_unready = true;
        break;
      }
    }
    if (any_unready && bake_pool_capacity_ >= bake_pool_used_ &&
        bake_dist_ssbo_ != nullptr)
    {
      GPU_shader_bind(sh);
      /* Buffers referenced by dead code in sdf_lp_common.glsl (the bake only
       * walks the mesh BVH); declared in the create info, bound for
       * completeness. The lp_* scene buffers only exist in the LP engine —
       * bound via the engine hook (nullptr-safe no-ops for the classic
       * engine); the dead code that references them never executes. */
      bind_bake_dead_ssbos(sh);
      sdf_bind_ssbo(sh, "sdf_modifiers", modifier_ssbo_);
      sdf_bind_ssbo(sh, "polygon_points", polygon_ssbo_);
      sdf_bind_ssbo(sh, "mesh_data_buf", mesh_data_ssbo_);
      sdf_bind_ssbo(sh, "mesh_color_buf", mesh_color_ssbo_);
      sdf_bind_ssbo(sh, "bake_dist", bake_dist_ssbo_);
      sdf_bind_ssbo(sh, "bake_nrm", bake_nrm_ssbo_);
      sdf_bind_ssbo(sh, "bake_col", bake_col_ssbo_);

      int64_t budget = kBakeVoxelsPerFrame;
      for (const auto &item : bake_records_.items()) {
        if (budget <= 0) {
          break;
        }
        MeshBakeRecord &rec = item.value;
        if (!rec.work_active) {
          continue;
        }
        const MeshOffsets *offsets = mesh_offsets_.lookup_ptr(item.key);
        const MeshBakeInfo *info = mesh_bake_info_.lookup_ptr(item.key);
        if (offsets == nullptr || info == nullptr) {
          continue;
        }
        /* Coarse far-field level first (tiny): fp32 unclamped distance,
         * distance pool only. */
        if (rec.work.cres.x > 0 && !rec.work.cready) {
          const int64_t cslice = int64_t(rec.work.cres.x) * rec.work.cres.y;
          while (rec.next_cz < rec.work.cres.z && budget > 0) {
            const int z_left = rec.work.cres.z - rec.next_cz;
            const int z_count = int(
                std::min<int64_t>(z_left, std::max<int64_t>(budget / cslice, 1)));
            dispatch_mesh_bake(sh,
                               rec.work.cres,
                               rec.work.cbase,
                               rec.work.corigin,
                               rec.work.cvoxel,
                               60000.0f,
                               true,
                               *offsets,
                               info->has_colors,
                               rec.next_cz,
                               z_count);
            rec.next_cz += z_count;
            budget -= z_count * cslice;
          }
          if (rec.next_cz >= rec.work.cres.z) {
            rec.work.cready = true;
          }
        }
        if (!rec.work.ready) {
          const int64_t slice_voxels = int64_t(rec.work.res.x) * rec.work.res.y;
          while (rec.next_z < rec.work.res.z && budget > 0) {
            const int z_left = rec.work.res.z - rec.next_z;
            const int z_count = int(std::min<int64_t>(
                z_left, std::max<int64_t>(budget / slice_voxels, 1)));
            dispatch_mesh_bake(sh,
                               rec.work.res,
                               rec.work.base,
                               rec.work.origin,
                               rec.work.voxel_size,
                               rec.work.band,
                               false,
                               *offsets,
                               info->has_colors,
                               rec.next_z,
                               z_count);
            rec.next_z += z_count;
            budget -= z_count * slice_voxels;
          }
          if (rec.next_z >= rec.work.res.z) {
            rec.work.ready = true;
          }
        }
      }
      GPU_shader_unbind();
      GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
    }

    /* The dispatch may have (re)assigned pool ranges (regrow compaction or
     * fresh appends) and work grids may have flipped to live: re-resolve the
     * baked fields of every mesh object and re-upload the object buffer when
     * anything changed, so the flags/bases uploaded earlier this frame never
     * reference stale ranges. */
    bool objects_changed = false;
    for (int i = 0; i < int(objects_.size()); i++) {
      if (objects_[i].sdf_type != SDF_GPU_TYPE_MESH || object_mesh_keys_[i] == nullptr) {
        continue;
      }
      objects_changed |= apply_baked_fields(objects_[i], object_mesh_keys_[i]);
    }
    if (objects_changed && object_ssbo_ != nullptr &&
        object_ssbo_count_ == int(objects_.size()))
    {
      GPU_storagebuf_update(object_ssbo_, objects_.data());
    }
    return objects_changed;
  }

  /** \} */

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

  static void sdf_bind_ssbo(gpu::Shader *sh, const char *name, gpu::StorageBuf *buf)
  {
    if (buf == nullptr) {
      return;
    }
    const int slot = GPU_shader_get_ssbo_binding(sh, name);
    if (slot >= 0) {
      GPU_storagebuf_bind(buf, slot);
    }
  }

  /* Engine hook: bind the lp_* scene buffers the bake shader declares (dead
   * code there) during update_mesh_bakes; overridden by the LP engine. */
  virtual void bind_bake_dead_ssbos(gpu::Shader * /*sh*/) {}

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
    /* Baked mesh volume pools (nullptr until the first bake; the
     * SDF_LP_MESH_FLAG_BAKED flag is only set on objects with a ready
     * record, which implies the pools exist). */
    sdf_bind_ssbo(sh, "bake_dist", bake_dist_ssbo_);
    sdf_bind_ssbo(sh, "bake_nrm", bake_nrm_ssbo_);
    sdf_bind_ssbo(sh, "bake_col", bake_col_ssbo_);
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
    const int effective = effective_lighting(lighting_type_);
    GPU_shader_uniform_1i(sh, "lighting_type", effective);
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
    if (blit_sh() == nullptr) {
      return;
    }

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
    GPU_texture_filter_mode(comp_color_tx_, smooth_upscale_);
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
  ~SdfInstanceBase() override
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
    if (mesh_color_ssbo_) {
      GPU_storagebuf_free(mesh_color_ssbo_);
    }
    if (bake_dist_ssbo_) {
      GPU_storagebuf_free(bake_dist_ssbo_);
    }
    if (bake_nrm_ssbo_) {
      GPU_storagebuf_free(bake_nrm_ssbo_);
    }
    if (bake_col_ssbo_) {
      GPU_storagebuf_free(bake_col_ssbo_);
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

    if (fullscreen_batch_) {
      GPU_batch_discard(fullscreen_batch_);
    }
  }
};

}  // namespace blender::draw::sdf
