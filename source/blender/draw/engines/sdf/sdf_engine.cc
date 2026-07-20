/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine: analytical sphere tracer — evaluates SDF primitives directly per-pixel.
 * Classic tile/BVH pipeline; see sdf_lp_engine.cc for the Lipschitz pruning variant.
 */

#include "sdf_engine_internal.hh"

#include <cstdlib>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "BLI_index_range.hh"
#include "BLI_task.hh"

#include "meshing/dcsdd_contouring.hh"

namespace blender::draw::sdf {

/* Static shader cache — survives engine instance destruction (mode switches). */

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
    "sdf_lp_march_comp",
    "sdf_lp_debug_comp",
    "sdf_mesh_bake_comp",
};

static gpu::StaticShader s_shaders[SH_COUNT];
static bool s_shaders_initialized = false;
/* Per-shader compile timing (wall clock from first schedule to ready). */
static double s_compile_start[SH_COUNT] = {};
static bool s_compile_logged[SH_COUNT] = {};

static void sdf_shaders_lazy_init()
{
  if (!s_shaders_initialized) {
    for (int i = 0; i < SH_COUNT; i++) {
      s_shaders[i] = gpu::StaticShader(s_shader_info_names[i]);
    }
    s_shaders_initialized = true;
  }
}

void sdf_shaders_ensure_list(const Span<const int> &idx)
{
  sdf_shaders_lazy_init();
  for (const int i : idx) {
    if (s_shaders[i].is_ready()) {
      continue;
    }
    if (s_compile_start[i] == 0.0) {
      s_compile_start[i] = BLI_time_now_seconds();
    }
    s_shaders[i].ensure_compile_async();
    if (s_shaders[i].is_ready() && !s_compile_logged[i]) {
      /* Finalized this call: log the wall time from first schedule to ready.
       * Large values mean the driver did a full compile; sub-second values
       * mean the driver's disk shader cache served it. */
      s_compile_logged[i] = true;
      CLOG_INFO(&LOG,
                "SDF shader '%s' ready after %.2f s",
                s_shader_info_names[i],
                BLI_time_now_seconds() - s_compile_start[i]);
    }
  }
}

int sdf_shaders_count_ready(const Span<const int> &idx)
{
  int ready = 0;
  for (const int i : idx) {
    if (s_shaders[i].is_ready()) {
      ready++;
    }
  }
  return ready;
}

bool sdf_shader_is_ready(int index)
{
  return s_shaders[index].is_ready();
}

gpu::Shader *sdf_shader_get(int index)
{
  return s_shaders[index].get();
}

void sdf_shaders_free()
{
  for (int i = 0; i < SH_COUNT; i++) {
    s_shaders[i] = {};
    s_compile_start[i] = 0.0;
    s_compile_logged[i] = false;
  }
  s_shaders_initialized = false;
}

/* Viewport compile-progress text (empty when idle). */
static std::string s_compile_status;

void sdf_compile_status_set(const Span<const int> &idx)
{
  int ready = 0;
  const char *pending = nullptr;
  for (const int i : idx) {
    if (s_shaders[i].is_ready()) {
      ready++;
    }
    else if (pending == nullptr) {
      pending = s_shader_info_names[i];
    }
  }
  char buf[128];
  if (pending != nullptr) {
    /* Two short lines so the overlay text (almost) never clips. */
    BLI_snprintf(buf,
                 sizeof(buf),
                 "Compiling SDF shaders %d/%d\n%s...",
                 ready,
                 int(idx.size()),
                 pending);
  }
  else {
    BLI_snprintf(buf, sizeof(buf), "Compiling SDF shaders %d/%d...", ready, int(idx.size()));
  }
  s_compile_status = buf;
}

void sdf_compile_status_clear()
{
  s_compile_status.clear();
}

const char *sdf_shader_compile_status_get()
{
  return s_compile_status.empty() ? nullptr : s_compile_status.c_str();
}

/* Classic engine instance: tile/BVH trace pipeline only. */

static constexpr int kClassicShaders[] = {
    SH_TRACE_COMP,
    SH_TRACE_TILE_COMP,
    SH_AABB_PROJECT_COMP,
    SH_TILE_CULL_COMP,
    SH_CONE_MARCH_COMP,
    SH_COLOR_RESOLVE_COMP,
    SH_NORMAL_COMP,
    SH_SHADE_COMP,
    SH_BLIT,
    SH_FXAA,
    SH_MESH_BAKE_COMP,
};

class Instance : public SdfInstanceBase {
 private:
  Span<const int> engine_shader_list() const override
  {
    return Span<const int>(kClassicShaders, ARRAY_SIZE(kClassicShaders));
  }

  void draw_trace_pipeline(bool profiling) override
  {
#define PROF_START(name) if (profiling) { s_profiler.mark_start(name); }
#define PROF_END()       if (profiling) { s_profiler.mark_end(); }

    /* Dense per-mesh volume bakes (shared with the LP engine; runtime
     * sampling is the SDF_LP_MESH_FLAG_BAKED fast path in sdf_mesh_lib). */
    PROF_START("Mesh Bake");
    GPU_debug_group_begin("SDF Mesh Bake");
    update_mesh_bakes();
    GPU_debug_group_end();
    PROF_END();

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

#undef PROF_START
#undef PROF_END
  }
};

DrawEngine *Engine::create_instance()
{
  return new Instance();
}

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
            if (s_polygon_pts_cpu[obj.polygon_point_start].arc_bounds.w <= -2.5f) {
              /* BVH mode: the root node holds the padded 2D bounds. */
              int root = int(s_polygon_pts_cpu[obj.polygon_point_start].arc_data.x);
              mn.x = s_polygon_pts_cpu[root].vi_edge.x;
              mn.y = s_polygon_pts_cpu[root].vi_edge.y;
              mx.x = s_polygon_pts_cpu[root].arc_data.x;
              mx.y = s_polygon_pts_cpu[root].arc_data.y;
            }
            else {
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

std::string sdf_bake_to_mesh(int grid_res,
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

  /* Grid-vertex dimensions and a safety cap: the CPU mesher holds the whole grid. */
  const int3 gv = grid_cells + int3(1);
  const int64_t total_gv = int64_t(gv.x) * int64_t(gv.y) * int64_t(gv.z);
  if (gv.x > 1025 || gv.y > 1025 || gv.z > 1025 || total_gv > int64_t(512) * 512 * 512) {
    return "Grid too large (" + std::to_string(gv.x) + " x " + std::to_string(gv.y) + " x " +
           std::to_string(gv.z) + " grid vertices). Lower the resolution.";
  }

  /* Grid SDF values (S) and grid vertex positions (GV) for the CPU DC-SDD mesher.
   * Indexing matches meshing::index3D: x-fastest, idx = x + gv.x * (y + gv.y * z). */
  Eigen::VectorXf S(total_gv);
  Eigen::MatrixXf GV(total_gv, 3);
  S.fill(1e6f); /* Large positive = far outside; covers AABB-skipped chunks. */
  threading::parallel_for(IndexRange(int64_t(gv.z)), 1, [&](const IndexRange range) {
    for (const int64_t k : range) {
      for (int64_t j = 0; j < gv.y; j++) {
        for (int64_t i = 0; i < gv.x; i++) {
          const int64_t idx = i + int64_t(gv.x) * (j + int64_t(gv.y) * k);
          GV.row(idx) = Eigen::Vector3f(grid_origin.x + float(i) * cell_size,
                                        grid_origin.y + float(j) * cell_size,
                                        grid_origin.z + float(k) * cell_size);
        }
      }
    }
  });

  /* Compile shaders (cached) */
  gpu::Shader *grid_sh = GPU_shader_create_from_info_name("sdf_grid_eval_comp");
  gpu::Shader *color_sh = GPU_shader_create_from_info_name("sdf_dc_vertex_color_comp");
  if (!grid_sh || !color_sh) {
    return "Shader compile failed";
  }

  /* Per-chunk grid value buffer (reused) */
  const int padded_grid_total = PADDED_GV * PADDED_GV * PADDED_GV;
  gpu::StorageBuf *grid_ssbo = GPU_storagebuf_create_ex(
      padded_grid_total * sizeof(float), nullptr, GPU_USAGE_DYNAMIC, "dc_grid_values");
  Vector<float> chunk_vals(padded_grid_total);

  /* Vertex color buffers (allocated after the CPU contouring, when vert_count is known) */
  gpu::StorageBuf *pos_ssbo = nullptr;
  gpu::StorageBuf *color_ssbo = nullptr;

  auto cleanup = [&]() {
    GPU_storagebuf_free(grid_ssbo);
    if (pos_ssbo) {
      GPU_storagebuf_free(pos_ssbo);
    }
    if (color_ssbo) {
      GPU_storagebuf_free(color_ssbo);
    }
    GPU_shader_free(grid_sh);
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
  int grid_bake_dist_slot = GPU_shader_get_ssbo_binding(grid_sh, "bake_dist");
  int grid_bake_nrm_slot = GPU_shader_get_ssbo_binding(grid_sh, "bake_nrm");
  int grid_bake_col_slot = GPU_shader_get_ssbo_binding(grid_sh, "bake_col");
  int has_bvh = (s_bvh_ssbo && s_bvh_root >= 0) ? 1 : 0;

  int gd = (PADDED_GV + 3) / 4;

  for (int cz = 0; cz < n_chunks.z; cz++) {
    for (int cy = 0; cy < n_chunks.y; cy++) {
      for (int cx = 0; cx < n_chunks.x; cx++) {
        int3 cell_start = int3(cx, cy, cz) * CHUNK - int3(PAD);
        float3 chunk_origin = grid_origin + float3(cell_start) * cell_size;
        float3 chunk_min = chunk_origin;
        float3 chunk_max = chunk_origin + float3(float(PADDED_GV)) * cell_size;

        /* AABB cull: skip chunks with no nearby SDF objects (S stays at the 1e6f fill) */
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
          continue;
        }

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
        if (s_bake_dist_ssbo && grid_bake_dist_slot >= 0) {
          GPU_storagebuf_bind(s_bake_dist_ssbo, grid_bake_dist_slot);
        }
        if (s_bake_nrm_ssbo && grid_bake_nrm_slot >= 0) {
          GPU_storagebuf_bind(s_bake_nrm_ssbo, grid_bake_nrm_slot);
        }
        if (s_bake_col_ssbo && grid_bake_col_slot >= 0) {
          GPU_storagebuf_bind(s_bake_col_ssbo, grid_bake_col_slot);
        }
        GPU_shader_uniform_1i(grid_sh, "object_count", s_object_count);
        GPU_shader_uniform_1i(grid_sh, "group_count", s_group_count);
        GPU_shader_uniform_1i(grid_sh, "grid_verts", PADDED_GV);
        GPU_shader_uniform_3fv(grid_sh, "grid_origin", chunk_origin);
        GPU_shader_uniform_1f(grid_sh, "cell_size", cell_size);
        GPU_shader_uniform_1i(grid_sh, "use_bvh", has_bvh);
        GPU_shader_uniform_1i(grid_sh, "bvh_root", s_bvh_root);
        GPU_shader_uniform_1f(grid_sh, "sdf_ray_epsilon", 0.005f);
        GPU_compute_dispatch(grid_sh, gd, gd, gd);
        GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

        /* Read back the chunk grid and scatter into the global S array.
         * Global grid-vertex coords g = cell_start + local; skip out-of-range verts. */
        GPU_storagebuf_read(grid_ssbo, chunk_vals.data());
        for (int lz = 0; lz < PADDED_GV; lz++) {
          for (int ly = 0; ly < PADDED_GV; ly++) {
            for (int lx = 0; lx < PADDED_GV; lx++) {
              const int3 g = cell_start + int3(lx, ly, lz);
              if (g.x < 0 || g.y < 0 || g.z < 0 || g.x >= gv.x || g.y >= gv.y || g.z >= gv.z) {
                continue;
              }
              const int64_t sidx = g.x + int64_t(gv.x) * (g.y + int64_t(gv.y) * g.z);
              const int lidx = lx + PADDED_GV * (ly + PADDED_GV * lz);
              S(sidx) = chunk_vals[lidx];
            }
          }
        }
      }
    }
  }
  GPU_shader_unbind();

  /* CPU DC-SDD contouring (replaces the old GPU QEF + triangulation passes). */
  namespace meshing = blender::sdf::meshing;
  meshing::ContouringOptions options;
  options.method = meshing::ContouringMethod::Ours;
  options.verbose = getenv("SDF_DCSDD_VERBOSE") != nullptr;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  meshing::contouring(S, GV, gv.x, gv.y, gv.z, 0.0, V, F, options);

  const int vert_count = int(V.rows());
  if (vert_count == 0) {
    cleanup();
    return "DC-SDD produced no surface (empty scene or resolution too low)";
  }

  /* Guarantee outward orientation: flip all triangles if the signed volume is negative. */
  double signed_volume = 0.0;
  for (int f = 0; f < F.rows(); f++) {
    const Eigen::Vector3d v0 = V.row(F(f, 0)).transpose();
    const Eigen::Vector3d v1 = V.row(F(f, 1)).transpose();
    const Eigen::Vector3d v2 = V.row(F(f, 2)).transpose();
    signed_volume += v0.dot(v1.cross(v2)) / 6.0;
  }
  if (signed_volume < 0.0) {
    for (int f = 0; f < F.rows(); f++) {
      std::swap(F(f, 1), F(f, 2));
    }
  }

  out_positions.resize(vert_count);
  for (int i = 0; i < vert_count; i++) {
    out_positions[i] = float3(float(V(i, 0)), float(V(i, 1)), float(V(i, 2)));
  }

  out_tris.reserve(F.rows());
  for (int f = 0; f < F.rows(); f++) {
    const int a = F(f, 0), b = F(f, 1), c = F(f, 2);
    if (a >= 0 && a < vert_count && b >= 0 && b < vert_count && c >= 0 && c < vert_count) {
      out_tris.append(int3(a, b, c));
    }
  }

  /* Area-weighted per-vertex normals from the triangles. */
  out_normals.resize(vert_count);
  for (float3 &n : out_normals) {
    n = float3(0.0f);
  }
  for (const int3 &tri : out_tris) {
    const float3 e1 = out_positions[tri.y] - out_positions[tri.x];
    const float3 e2 = out_positions[tri.z] - out_positions[tri.x];
    const float3 n = math::cross(e1, e2);
    out_normals[tri.x] += n;
    out_normals[tri.y] += n;
    out_normals[tri.z] += n;
  }
  threading::parallel_for(IndexRange(int64_t(vert_count)), 4096, [&](const IndexRange range) {
    for (const int64_t i : range) {
      if (math::dot(out_normals[i], out_normals[i]) > 1e-20f) {
        out_normals[i] = math::normalize(out_normals[i]);
      }
      else {
        out_normals[i] = float3(0.0f, 0.0f, 1.0f);
      }
    }
  });

  /* Vertex color pass: evaluate SDF color at each mesh vertex position. */
  Vector<float4> pos4(vert_count);
  for (int i = 0; i < vert_count; i++) {
    pos4[i] = float4(out_positions[i], 1.0f);
  }
  pos_ssbo = GPU_storagebuf_create_ex(
      vert_count * sizeof(float4), nullptr, GPU_USAGE_DYNAMIC, "dc_positions");
  GPU_storagebuf_update(pos_ssbo, pos4.data());
  color_ssbo = GPU_storagebuf_create_ex(
      vert_count * sizeof(float4), nullptr, GPU_USAGE_DYNAMIC, "dc_colors");

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
    int col_bake_dist_slot = GPU_shader_get_ssbo_binding(color_sh, "bake_dist");
    int col_bake_nrm_slot = GPU_shader_get_ssbo_binding(color_sh, "bake_nrm");
    int col_bake_col_slot = GPU_shader_get_ssbo_binding(color_sh, "bake_col");

    if (s_object_ssbo && col_obj_slot >= 0) GPU_storagebuf_bind(s_object_ssbo, col_obj_slot);
    if (s_modifier_ssbo && col_mod_slot >= 0) GPU_storagebuf_bind(s_modifier_ssbo, col_mod_slot);
    if (s_group_ssbo && col_grp_slot >= 0) GPU_storagebuf_bind(s_group_ssbo, col_grp_slot);
    if (col_poly_slot >= 0) GPU_storagebuf_bind(s_polygon_ssbo, col_poly_slot);
    if (col_pos_slot >= 0) GPU_storagebuf_bind(pos_ssbo, col_pos_slot);
    if (col_out_slot >= 0) GPU_storagebuf_bind(color_ssbo, col_out_slot);
    if (has_bvh && col_bvh_slot >= 0) GPU_storagebuf_bind(s_bvh_ssbo, col_bvh_slot);
    if (s_mesh_data_ssbo && col_mesh_data_slot >= 0) {
      GPU_storagebuf_bind(s_mesh_data_ssbo, col_mesh_data_slot);
    }
    if (s_bake_dist_ssbo && col_bake_dist_slot >= 0) {
      GPU_storagebuf_bind(s_bake_dist_ssbo, col_bake_dist_slot);
    }
    if (s_bake_nrm_ssbo && col_bake_nrm_slot >= 0) {
      GPU_storagebuf_bind(s_bake_nrm_ssbo, col_bake_nrm_slot);
    }
    if (s_bake_col_ssbo && col_bake_col_slot >= 0) {
      GPU_storagebuf_bind(s_bake_col_ssbo, col_bake_col_slot);
    }

    GPU_shader_uniform_1i(color_sh, "object_count", s_object_count);
    GPU_shader_uniform_1i(color_sh, "group_count", s_group_count);
    GPU_shader_uniform_1i(color_sh, "vert_count", vert_count);
    GPU_shader_uniform_1i(color_sh, "use_bvh", has_bvh);
    GPU_shader_uniform_1i(color_sh, "bvh_root", s_bvh_root);
    GPU_shader_uniform_1f(color_sh, "sdf_ray_epsilon", 0.005f);

    int color_groups = (vert_count + 63) / 64;
    GPU_compute_dispatch(color_sh, color_groups, 1, 1);
    GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
    GPU_shader_unbind();
  }

  out_colors.resize(vert_count);
  GPU_storagebuf_read(color_ssbo, out_colors.data());

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
  out += "PASS TIMING (serialized per pass: GPU_finish between passes,\n";
  out += "so the total is an upper bound of the real frame time)\n";
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

  /* LP prune diagnostics (only present when the profiled frame ran a prune). */
  if (r.lp_pruned) {
    out += "LP PRUNING\n";
    out += "----------------------------------------------------------------\n";
    snprintf(buf, sizeof(buf),
             "  Active-list pool: %lld entries, %d cell(s) overflowed\n"
             "  Tmp pool:         %lld entries, %d cell(s) overflowed\n",
             (long long)r.lp_active_capacity, r.lp_active_overflow,
             (long long)r.lp_tmp_capacity, r.lp_tmp_overflow);
    out += buf;
    if (r.lp_active_overflow > 0 || r.lp_tmp_overflow > 0) {
      out += "  >> LP POOL OVERFLOW — overflowed cells trace the FULL CSG tree\n"
             "     (exact but much slower). Enlarge pool capacities or lower the grid level.\n";
    }
    out += "\n";
  }

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