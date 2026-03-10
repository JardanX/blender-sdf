/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine: bakes SDF objects into a sparse brick atlas, then ray-marches it.
 */

#include "BLI_map.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_set.hh"

#include "BKE_geometry_set.hh"
#include "BKE_idprop.hh"
#include "BKE_object.hh"
#include "BKE_object_types.hh"
#include "BKE_studiolight.h"
#include "BKE_volume.hh"
#include "BKE_volume_render.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_tree_interface_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"
#include "DNA_userdef_types.h"
#include "DNA_view3d_enums.h"
#include "DNA_view3d_types.h"

#include "IMB_imbuf_types.hh"

#include "MEM_guardedalloc.h"

#include "GPU_batch.hh"
#include "GPU_compute.hh"
#include "GPU_framebuffer.hh"
#include "GPU_shader.hh"
#include "GPU_shader_builtin.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_uniform_buffer.hh"

#include "draw_defines.hh"
#include "draw_manager.hh"
#include "draw_view.hh"
#include "draw_view_data.hh"

#include "sdf_private.hh"

#include "sdf_engine.h" /* Own include. */

#include "BLI_time.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace blender::draw::sdf {

using namespace draw;

/* ---- Shape fingerprinting ---- */

/**
 * Compute a 64-bit fingerprint for an SDF shape.
 * Two objects with the same fingerprint can share atlas data (instancing).
 *
 * The fingerprint captures normalized shape geometry:
 * - SDF type (box, sphere, capsule, torus)
 * - Aspect ratio (size normalized so max component = 1.0)
 * - Relative bevel (bevel / max_size)
 *
 * This means an object with size (2,1,1) at scale 1 and size (1,0.5,0.5) at scale 2
 * produce the same fingerprint — they're the same shape at different scales.
 */
static uint64_t sdf_shape_fingerprint(int sdf_type,
                                       const float3 &effective_size,
                                       float effective_bevel,
                                       int ngon_sides = 0,
                                       float torus_angle = 360.0f,
                                       float ngon_star = 0.0f)
{
  /* Normalize by max component to make scale-independent. */
  float max_dim = math::reduce_max(math::abs(effective_size));
  if (max_dim < 1e-8f) {
    max_dim = 1.0f;
  }

  /* Quantize to avoid floating-point comparison issues.
   * 10000x gives 0.01% precision — more than enough for visual identity. */
  const int Q = 10000;
  int sx = int(roundf(effective_size.x / max_dim * Q));
  int sy = int(roundf(effective_size.y / max_dim * Q));
  int sz = int(roundf(effective_size.z / max_dim * Q));
  int sb = int(roundf(effective_bevel / max_dim * Q));

  /* FNV-1a 64-bit hash. */
  uint64_t h = 14695981039346656037ULL;
  auto mix = [&](uint64_t v) { h ^= v; h *= 1099511628211ULL; };
  mix(uint64_t(sdf_type));
  mix(uint64_t(sx));
  mix(uint64_t(sy));
  mix(uint64_t(sz));
  mix(uint64_t(sb));
  if (sdf_type == SDF_TYPE_NGON) {
    mix(uint64_t(ngon_sides));
    mix(uint64_t(int(roundf(ngon_star * Q))));
  }
  if (sdf_type == SDF_TYPE_TORUS) {
    mix(uint64_t(int(roundf(torus_angle * Q))));
  }
  return h;
}

/* ---- Performance overlay static state ---- */

/** Number of per-pass elapsed-time queries: classify, bake, grid blend, march, fxaa. */
static constexpr int PERF_PASS_COUNT = 5;
static constexpr int PERF_PASS_CLASSIFY = 0;
static constexpr int PERF_PASS_BAKE = 1;
static constexpr int PERF_PASS_GRID = 2;
static constexpr int PERF_PASS_MARCH = 3;
static constexpr int PERF_PASS_FXAA = 4;
/** Number of FPS samples for smoothing. */
static constexpr int PERF_FPS_SAMPLES = 8;

/** Formatted performance text, shared with the overlay drawing code. */
static char s_perf_text[SDF_PERF_BUF_SIZE] = "";
/** Whether the performance data is valid for display. */
static bool s_perf_active = false;

/* ---- Static atlas state for cross-engine access (overlay selection) ---- */
static gpu::Texture *s_compact_atlas = nullptr;
static gpu::Texture *s_indirection = nullptr;
static gpu::Texture *s_object_id_atlas = nullptr;
static float s_voxel_size = 0.0f;
static float3 s_atlas_origin = float3(0);
static float3 s_atlas_extent = float3(0);
static int3 s_grid_resolution = int3(0);
static int s_bricks_per_axis = 0;
static int s_object_count = 0;
static gpu::StorageBuf *s_object_ssbo = nullptr;

class Instance : public DrawEngine {
 private:
  /** Collected SDF objects for this frame. */
  Vector<SDFObjectGPU> objects_;

  /** Scene AABB accumulated from all SDF objects. */
  float3 scene_min_ = float3(1e30f);
  float3 scene_max_ = float3(-1e30f);

  /** Brick grid indirection texture (R32I, grid_res^3). */
  gpu::Texture *indirection_tx_ = nullptr;
  /** Compact atlas (RGBA16F, sized for active bricks only). */
  gpu::Texture *compact_atlas_tx_ = nullptr;
  /** Object ID atlas (R32I, same dimensions as compact atlas). */
  gpu::Texture *object_id_tx_ = nullptr;
  /** Brick counter SSBO (1 uint + padding). */
  gpu::StorageBuf *brick_counter_ = nullptr;
  /** Active brick coordinates SSBO (written by classify, read by bake/grid_blend). */
  gpu::StorageBuf *active_bricks_ = nullptr;
  int active_bricks_capacity_ = 0;

  /** Atlas parameters. */
  float3 atlas_origin_ = float3(0.0f);
  float3 atlas_extent_ = float3(1.0f);
  float voxel_size_ = 1.0f / 256.0f;

  /** Grid resolution in bricks per axis (per-axis, world-aligned). */
  int3 grid_res_ = int3(32);
  /** Total voxel resolution from UI. */
  int sdf_resolution_ = 128;
  /** Active brick count (set after bake via deferred readback). */
  int active_brick_count_ = 0;
  /** Previous frame's active brick count (for atlas pre-sizing). */
  int prev_active_brick_count_ = 0;
  /** Grow-only atlas capacity (active bricks that fit in current atlas). */
  int atlas_capacity_ = 0;
  /** Bricks per axis in compact atlas layout. */
  int bricks_per_axis_ = 1;
  /** Debug grid mode from UI. */
  int debug_grid_ = 0;
  /** Whether FXAA post-processing is enabled (from UI toggle). */
  bool fxaa_enabled_ = true;
  /** Surface margin multiplier (1.0 = default, from UI percentage). */
  float surface_margin_ = 1.0f;
  /** Max blend radius across all objects (computed in classify, used in bake). */
  float max_blend_ = 0.0f;
  /** Max shell distance across all shell objects (for candidate AABB expansion). */
  float max_shell_distance_ = 0.0f;

  /** Dirty tracking: hash of object data for the current frame. */
  uint64_t scene_hash_ = 0;
  /** Separate hash for grid objects (computed from lightweight data before
   * the expensive dense-float extraction, so we can skip it when unchanged). */
  uint64_t grid_hash_ = 0;
  bool needs_bake_ = true;
  /** Whether object/BVH SSBOs need re-upload (set when scene hash changes). */
  bool needs_upload_ = true;
  /** True when running in a depth-only pass (gizmo picking, snapping).
   * In depth mode the engine reuses the cached atlas from the last regular
   * draw and only runs the ray-march for depth writes. */
  bool depth_mode_ = false;

  /** Mesh-to-SDF grid objects pending processing. */
  struct PendingGridObject {
    const Object *ob;
  };
  Vector<PendingGridObject> pending_grid_objects_;

  /** Processed grid objects ready for GPU dispatch. */
  struct GridObject {
    gpu::Texture *texture = nullptr; /* R32F 3D texture */
    float4x4 world_to_texture;      /* Combined transform: world -> [0,1]^3 */
    float4 color;                    /* Object display color */
    float blend;                     /* Blend amount from modifier */
    int blend_type;                  /* Blend function type (eSDFBlendType) */
    int csg_operation;               /* CSG operation (eSDFCSGOperation) */
    float shell_distance;            /* Shell/extrusion thickness */
  };
  Vector<GridObject> grid_objects_;

  /** Cached shaders. */
  gpu::Shader *classify_sh_ = nullptr;
  gpu::Shader *bake_sh_ = nullptr;
  gpu::Shader *shape_bake_sh_ = nullptr;
  gpu::Shader *march_sh_ = nullptr;
  gpu::Shader *grid_blend_sh_ = nullptr;
  gpu::Shader *fxaa_sh_ = nullptr;

  /** FXAA offscreen target: march renders here, then FXAA composites to default FB. */
  gpu::Texture *march_color_tx_ = nullptr;
  gpu::FrameBuffer *march_fb_ = nullptr;
  int2 fxaa_viewport_size_ = int2(0);

  /** Object SSBO. */
  gpu::StorageBuf *object_ssbo_ = nullptr;
  int object_ssbo_count_ = 0;

  /** Modifier data for GPU upload. */
  Vector<SDFModifierGPU> modifiers_;
  gpu::StorageBuf *modifier_ssbo_ = nullptr;
  int modifier_ssbo_count_ = 0;

  /** BVH for object AABB culling on GPU. */
  Vector<BVHNodeGPU> bvh_nodes_;
  gpu::StorageBuf *bvh_ssbo_ = nullptr;
  int bvh_ssbo_count_ = 0;


  /** Shape table: unique SDF shapes identified by fingerprint. */
  struct ShapeInfo {
    uint64_t fingerprint;
    int sdf_type;
    int ngon_sides = 6;     /* Number of polygon sides (for SDF_TYPE_NGON). */
    float ngon_star = 0.0f;  /* Star factor (for SDF_TYPE_NGON). */
    float torus_angle = 360.0f; /* Angle aperture in degrees (for SDF_TYPE_TORUS). */
    float3 size_normalized; /* Aspect ratio (max = 1.0). */
    float bevel_normalized; /* bevel / max_size. */
    float world_scale;      /* Max effective size for first instance (representative). */

    /* Per-shape atlas params (filled by compute_shape_atlas_params / shape_classify_cpu). */
    int3 grid_res = int3(0);        /* Per-axis brick grid resolution. */
    float3 local_origin = float3(0); /* Local atlas origin. */
    float local_voxel_size = 0.0f;  /* Voxel size in local space. */
    int indir_offset = 0;           /* Offset into shape_indir_data_. */
    int slot_offset = 0;            /* First compact atlas slot for this shape. */
    int active_brick_count = 0;     /* Number of active bricks. */
  };
  Vector<ShapeInfo> shapes_;
  /** Fingerprint → shape index mapping. */
  Map<uint64_t, int> shape_fingerprint_map_;

  /** Instance table: per-object instance referencing a shape. */
  struct InstanceInfo {
    int shape_id;        /* Index into shapes_. */
    int object_id;       /* Index into objects_. */
    float4x4 world_to_local;
    float4x4 local_to_world;
    float4 color;
    float blend;
    int blend_type;      /* Blend function type (eSDFBlendType). */
    int csg_operation;   /* CSG operation (eSDFCSGOperation). */
    float world_scale;   /* max(effective_size) for this specific instance. */
  };
  Vector<InstanceInfo> instances_;

  /** GPU SSBOs for shape/instance tables. */
  gpu::StorageBuf *shape_ssbo_ = nullptr;
  int shape_ssbo_count_ = 0;
  gpu::StorageBuf *instance_ssbo_ = nullptr;
  int instance_ssbo_count_ = 0;

  /** Instanced rendering mode: per-shape local atlas + BVH instance traversal.
   * Activated when no objects use smooth blending (blend > 0). */
  bool use_instanced_ = false;

  /** Per-shape indirection data (flat int array, all shapes concatenated).
   * shape_indir_data_[shapes_[i].indir_offset + bx + by*gx + bz*gx*gy] = slot or -1. */
  Vector<int> shape_indir_data_;
  gpu::StorageBuf *shape_indir_ssbo_ = nullptr;
  int shape_indir_ssbo_count_ = 0;

  /** Per-shape active brick lists (concatenated, with global slot offsets). */
  Vector<ActiveBrick> shape_active_bricks_;

  /** Total active bricks across all shapes (for instanced mode). */
  int total_shape_active_bricks_ = 0;

  /** Previous frame's shape fingerprints for incremental instanced bake.
   * If a shape's fingerprint matches the previous frame, skip its rebake. */
  Set<uint64_t> prev_shape_fingerprints_;

  /** Fullscreen triangle batch (cached). */
  gpu::Batch *fullscreen_batch_ = nullptr;

  /** Debug grid wireframe batch (GPU_PRIM_LINES). */
  gpu::Batch *grid_batch_ = nullptr;
  int grid_batch_mode_ = 0;
  int3 grid_batch_res_ = int3(0);

  /** Debug BVH wireframe batch (GPU_PRIM_LINES, per-vertex color). */
  gpu::Batch *bvh_batch_ = nullptr;

  const DRWContext *draw_ctx_ = nullptr;

  /** Matcap / shading state. */
  gpu::Texture *matcap_tx_ = nullptr;
  std::string current_matcap_;
  int lighting_type_ = V3D_LIGHTING_STUDIO;
  int use_specular_ = 0;
  int use_matcap_flip_ = 0;

  /** Studio light data (4 directional lights + ambient). */
  float4 studio_light_dir_[4] = {};
  float4 studio_light_col_[4] = {};
  float4 studio_light_spec_[4] = {};
  float3 studio_ambient_ = float3(0.0f);

  /** Performance overlay state. */
  bool perf_enabled_ = false;
  bool perf_queries_created_ = false;
  /** Per-pass wall-clock start timestamps (set by perf_begin_pass). */
  double perf_pass_start_[PERF_PASS_COUNT] = {};
  /** Which passes were active this frame. */
  bool perf_pass_active_[PERF_PASS_COUNT] = {};
  /** Whether the current frame is baking. */
  bool perf_currently_baking_ = false;
  /** Whether the DISPLAYED data is from a bake frame. */
  bool perf_prev_baked_ = false;
  /** Timing results (from previous frame's queries). */
  double perf_classify_ms_ = 0.0;
  double perf_bake_ms_ = 0.0;
  double perf_grid_ms_ = 0.0;
  double perf_march_ms_ = 0.0;
  double perf_fxaa_ms_ = 0.0;
  double perf_frame_ms_ = 0.0;
  double perf_fps_ = 0.0;
  /** Persistent last-bake timing (updated only when a bake frame is read). */
  double perf_last_classify_ms_ = 0.0;
  double perf_last_bake_ms_ = 0.0;
  double perf_last_grid_ms_ = 0.0;
  /** Whether we have ever read valid bake timing data. */
  bool perf_has_bake_data_ = false;
  /** Wall-clock time of last draw() call for FPS. */
  double perf_last_draw_time_ = 0.0;
  /** Circular buffer for FPS smoothing. */
  double perf_fps_samples_[PERF_FPS_SAMPLES] = {};
  int perf_fps_sample_idx_ = 0;
  int perf_fps_sample_count_ = 0;

 public:
  Instance()
  {
    if (G.debug & G_DEBUG_GPU) {
      printf("[SDF] engine instance created (default res=%d)\n", sdf_resolution_);
    }
  }

  blender::StringRefNull name_get() final
  {
    return "SDF";
  }

  void init() final
  {
    draw_ctx_ = DRW_context_get();

    /* Read SDF settings early (before end_sync computes grid params and hash).
     * Previously this ran in draw(), after end_sync had already computed the
     * grid using stale sdf_resolution_ / surface_margin_ defaults. */
    sync_sdf_settings();
  }

  void begin_sync() final
  {
    /* In depth mode, preserve all cached state so depth passes don't
     * invalidate the bake cache with a different object subset. */
    depth_mode_ = draw_ctx_->is_depth();
    if (depth_mode_) {
      return;
    }

    objects_.clear();
    modifiers_.clear();
    pending_grid_objects_.clear();
    shapes_.clear();
    shape_fingerprint_map_.clear();
    instances_.clear();
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
    max_blend_ = 0.0f;
    max_shell_distance_ = 0.0f;
  }

  void object_sync(ObjectRef &ob_ref, Manager & /*manager*/) final
  {
    if (depth_mode_) {
      return;
    }

    Object *ob = ob_ref.object;

    /* Detect mesh objects with volume grids (from "Mesh to SDF Grid" modifier). */
    if (ob->type == OB_MESH && ob->runtime && ob->runtime->geometry_set_eval) {
      const bke::GeometrySet &geo = *ob->runtime->geometry_set_eval;
      if (geo.has_volume()) {
        const Volume *volume = geo.get_volume();
        if (volume && BKE_volume_num_grids(volume) > 0) {
          pending_grid_objects_.append({ob});

          /* Accumulate AABB from the object's local bounds transformed to world. */
          const std::optional<Bounds<float3>> bounds = BKE_object_boundbox_get(ob);
          if (bounds) {
            /* Transform all 8 corners of the local AABB to world space. */
            const float3 &lo = bounds->min;
            const float3 &hi = bounds->max;
            for (int c = 0; c < 8; c++) {
              float3 corner = float3((c & 1) ? hi.x : lo.x,
                                     (c & 2) ? hi.y : lo.y,
                                     (c & 4) ? hi.z : lo.z);
              float3 wc = math::transform_point(ob->object_to_world(), corner);
              scene_min_ = math::min(scene_min_, wc);
              scene_max_ = math::max(scene_max_, wc);
            }
          }
        }
      }
    }

    if (ob->type != OB_SDF) {
      return;
    }

    const SDF *sdf_data = static_cast<const SDF *>(ob->data);
    if (sdf_data == nullptr) {
      return;
    }

    /* Decompose object_to_world into rotation + scale.
     * Scale is baked into sdf_size; inverse matrix is rotation-only. */
    const float4x4 &mat = ob->object_to_world();

    float3 scale;
    scale.x = math::length(float3(mat[0]));
    scale.y = math::length(float3(mat[1]));
    scale.z = math::length(float3(mat[2]));

    /* Build rotation-only matrix. */
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

    /* Pack object data. */
    SDFObjectGPU gpu_obj = {};
    gpu_obj.inverse_matrix = inv_rot;
    gpu_obj.position = float4(mat[3].x, mat[3].y, mat[3].z, 0.0f);

    /* Bake scale into SDF size. */
    gpu_obj.sdf_size = float4(sdf_data->size[0] * scale.x,
                              sdf_data->size[1] * scale.y,
                              sdf_data->size[2] * scale.z,
                              0.0f);

    /* Compute bevel from modifier stack (sum of all enabled bevel modifiers). */
    float bevel = 0.0f;
    LISTBASE_FOREACH (const SDFModifier *, bmod, &sdf_data->modifiers) {
      if (bmod->show_viewport && bmod->type == SDF_MOD_BEVEL) {
        bevel += bmod->params[0];
      }
    }
    bevel *= math::reduce_min(scale);
    gpu_obj.bevel = bevel;

    gpu_obj.blend = sdf_data->blend;
    gpu_obj.sdf_type = sdf_data->sdf_type;
    gpu_obj.blend_type = sdf_data->blend_type;
    gpu_obj.csg_operation = sdf_data->csg_operation;
    gpu_obj.shell_distance = sdf_data->shell_distance;

    gpu_obj.color = float4(
        sdf_data->color[0], sdf_data->color[1], sdf_data->color[2], sdf_data->color[3]);

    /* Shape-specific data (box, ngon, torus share GPU struct fields). */
    if (sdf_data->sdf_type == SDF_TYPE_NGON) {
      float ngon_taper = sdf_data->ngon_taper;
      gpu_obj.box_corners = float4(sdf_data->ngon_corner, sdf_data->ngon_star, 0.0f, 0.0f);
      gpu_obj.box_edges = float4(sdf_data->ngon_edge_top,
                                 sdf_data->ngon_edge_bottom,
                                 math::max(ngon_taper, 0.0f),
                                 math::max(-ngon_taper, 0.0f));
      gpu_obj.box_modes = int4(0, sdf_data->ngon_edge_mode, sdf_data->ngon_sides, 0);
    }
    else if (sdf_data->sdf_type == SDF_TYPE_TORUS) {
      /* Precompute sin/cos of half-angle for capped torus. */
      float angle_deg = sdf_data->torus_angle;
      float half_rad = angle_deg * 0.5f * float(M_PI) / 180.0f;
      gpu_obj.box_corners = float4(sinf(half_rad), cosf(half_rad), 0.0f, 0.0f);
      gpu_obj.box_edges = float4(0.0f);
      gpu_obj.box_modes = int4(0, 0, 0, (angle_deg < 359.9f) ? 1 : 0);
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

    /* Pack modifiers for this object. */
    gpu_obj.modifier_start = int(modifiers_.size());
    gpu_obj.modifier_count = 0;
    LISTBASE_FOREACH (const SDFModifier *, mod, &sdf_data->modifiers) {
      if (!mod->show_viewport) {
        continue;
      }
      SDFModifierGPU gpu_mod = {};
      gpu_mod.header = int4(mod->type, mod->flag, 0, 0);
      gpu_mod.params = float4(mod->params[0], mod->params[1], mod->params[2], mod->params[3]);
      modifiers_.append(gpu_mod);
      gpu_obj.modifier_count++;
    }

    /* Compute shape-specific local extent, then add blend/shell padding.
     * Each primitive has different actual extents from its size parameters. */
    float shell_expand = (sdf_data->csg_operation == SDF_CSG_SHELL) ?
                              fabsf(sdf_data->shell_distance) :
                              0.0f;
    float pad = sdf_data->blend + shell_expand;
    float3 local_extent;
    float3 sz = float3(gpu_obj.sdf_size);
    switch (sdf_data->sdf_type) {
      case SDF_TYPE_CAPSULE: {
        /* Capsule: radius sz.x in XY, half-height sz.y along Z, caps add radius. */
        float r = sz.x;
        float h = math::max(sz.y - bevel, 0.0f);
        local_extent = float3(r + pad, r + pad, h + r + pad);
        break;
      }
      case SDF_TYPE_CYLINDER: {
        /* Elliptical cylinder: XY radii + bevel + pad, Z half-height + bevel + pad. */
        local_extent = sz + float3(bevel + pad);
        break;
      }
      case SDF_TYPE_CONE: {
        /* Cone: base radius sz.x in XY, half-height sz.y along Z. */
        float r = sz.x;
        float h = sz.y;
        local_extent = float3(r + bevel + pad, r + bevel + pad, h + bevel + pad);
        break;
      }
      case SDF_TYPE_TORUS: {
        /* Torus: major radius sz.x in XY, minor sz.y is tube radius along Z. */
        float outer = sz.x + sz.y;
        local_extent = float3(outer + pad, outer + pad, sz.y + pad);
        break;
      }
      case SDF_TYPE_NGON: {
        /* N-Gon: circumradius sz.x in XY, half-height sz.z along Z. */
        float r = sz.x;
        local_extent = float3(r + bevel + pad, r + bevel + pad, sz.z + bevel + pad);
        break;
      }
      default:
        /* Box / Sphere: symmetric extent = size + bevel + pad. */
        local_extent = sz + float3(bevel + pad);
        break;
    }

    /* Expand AABB for domain modifiers. */
    LISTBASE_FOREACH (const SDFModifier *, mod, &sdf_data->modifiers) {
      if (!mod->show_viewport) {
        continue;
      }
      switch (mod->type) {
        case SDF_MOD_ELONGATE:
          local_extent += float3(mod->params[0], mod->params[1], mod->params[2]);
          break;
        case SDF_MOD_HOLLOW:
        case SDF_MOD_ONION:
          local_extent += float3(mod->params[0]);
          break;
        case SDF_MOD_ROUND:
          local_extent += float3(mod->params[0]);
          break;
        default:
          break;
      }
    }

    /* Transform local AABB corners to world to get tight world AABB. */
    float3 world_min = float3(1e30f);
    float3 world_max = float3(-1e30f);
    for (int corner = 0; corner < 8; corner++) {
      float3 local_corner = float3((corner & 1) ? local_extent.x : -local_extent.x,
                                   (corner & 2) ? local_extent.y : -local_extent.y,
                                   (corner & 4) ? local_extent.z : -local_extent.z);
      float3 world_corner = float3(rot_mat * float4(local_corner, 0.0f)) +
                             float3(mat[3].x, mat[3].y, mat[3].z);
      world_min = math::min(world_min, world_corner);
      world_max = math::max(world_max, world_corner);
    }

    /* Intersection objects must be evaluated everywhere: outside the intersector
     * the result should be "outside" (large positive), clipping the base geometry.
     * Expand their AABB to cover the entire scene so they're always BVH candidates. */
    if (sdf_data->csg_operation == SDF_CSG_INTERSECT) {
      gpu_obj.bbox_min = float4(-1e10f, -1e10f, -1e10f, 0.0f);
      gpu_obj.bbox_max = float4(1e10f, 1e10f, 1e10f, 0.0f);
    }
    else {
      gpu_obj.bbox_min = float4(world_min, 0.0f);
      gpu_obj.bbox_max = float4(world_max, 0.0f);
    }

    /* Rotation-invariant scene extent: circumscribed sphere of local AABB.
     * radius = length(local_extent) is the tightest rotation-invariant bound.
     * Grid resolution stays constant regardless of object rotation. */
    float sphere_radius = math::length(local_extent);
    float3 obj_center = float3(mat[3].x, mat[3].y, mat[3].z);
    scene_min_ = math::min(scene_min_, obj_center - float3(sphere_radius));
    scene_max_ = math::max(scene_max_, obj_center + float3(sphere_radius));

    max_blend_ = math::max(max_blend_, gpu_obj.blend);
    if (sdf_data->csg_operation == SDF_CSG_SHELL) {
      max_shell_distance_ = math::max(max_shell_distance_, fabsf(sdf_data->shell_distance));
    }

    const int object_index = objects_.size();
    objects_.append(gpu_obj);

    /* Build shape fingerprint and instance entry. */
    float3 effective_size = float3(gpu_obj.sdf_size);
    float effective_bevel = gpu_obj.bevel;
    int ngon_sides = (sdf_data->sdf_type == SDF_TYPE_NGON) ? sdf_data->ngon_sides : 0;
    float ngon_star = (sdf_data->sdf_type == SDF_TYPE_NGON) ? sdf_data->ngon_star : 0.0f;
    float torus_angle = (sdf_data->sdf_type == SDF_TYPE_TORUS) ? sdf_data->torus_angle : 360.0f;
    uint64_t fp = sdf_shape_fingerprint(
        sdf_data->sdf_type, effective_size, effective_bevel, ngon_sides, torus_angle, ngon_star);

    int shape_id;
    const int *existing = shape_fingerprint_map_.lookup_ptr(fp);
    if (existing) {
      shape_id = *existing;
    }
    else {
      /* New unique shape. */
      float max_dim = math::reduce_max(math::abs(effective_size));
      if (max_dim < 1e-8f) {
        max_dim = 1.0f;
      }

      ShapeInfo shape;
      shape.fingerprint = fp;
      shape.sdf_type = sdf_data->sdf_type;
      shape.ngon_sides = (sdf_data->sdf_type == SDF_TYPE_NGON) ? sdf_data->ngon_sides : 6;
      shape.ngon_star = (sdf_data->sdf_type == SDF_TYPE_NGON) ? sdf_data->ngon_star : 0.0f;
      shape.torus_angle = (sdf_data->sdf_type == SDF_TYPE_TORUS) ? sdf_data->torus_angle : 360.0f;
      shape.size_normalized = effective_size / max_dim;
      shape.bevel_normalized = effective_bevel / max_dim;
      shape.world_scale = max_dim;

      shape_id = shapes_.size();
      shapes_.append(shape);
      shape_fingerprint_map_.add(fp, shape_id);
    }

    /* Build local-space transforms.
     * The shape atlas is in normalized space (max dimension = 1.0).
     * world_to_local must include 1/max_dim scaling so ray coordinates
     * match the atlas coordinate system.
     * local_to_world is the inverse: scales from normalized back to world. */
    float max_dim = math::reduce_max(math::abs(effective_size));
    if (max_dim < 1e-8f) {
      max_dim = 1.0f;
    }

    /* Build normalization scale matrix: maps Blender-unit local → normalized local. */
    float inv_max = 1.0f / max_dim;
    float4x4 norm_scale = float4x4::identity();
    norm_scale[0][0] = inv_max;
    norm_scale[1][1] = inv_max;
    norm_scale[2][2] = inv_max;

    float4x4 inv_norm_scale = float4x4::identity();
    inv_norm_scale[0][0] = max_dim;
    inv_norm_scale[1][1] = max_dim;
    inv_norm_scale[2][2] = max_dim;

    InstanceInfo inst;
    inst.shape_id = shape_id;
    inst.object_id = object_index;
    inst.world_to_local = norm_scale * math::invert(mat);
    inst.local_to_world = mat * inv_norm_scale;
    inst.color = gpu_obj.color;
    inst.blend = sdf_data->blend;
    inst.blend_type = sdf_data->blend_type;
    inst.csg_operation = sdf_data->csg_operation;
    inst.world_scale = max_dim;

    instances_.append(inst);
  }

  void end_sync() final
  {
    /* In depth mode, reuse cached atlas — don't recompute anything. */
    if (depth_mode_) {
      needs_bake_ = false;
      return;
    }


    /* Compute a lightweight hash from pending grid objects BEFORE doing the
     * expensive dense-float extraction. This lets us skip grid processing
     * entirely when nothing changed (saves 40ms+ per frame at fine voxel sizes).
     *
     * Hash includes: object count, transforms, bounds (changes with voxel_size),
     * color, and blend from modifier. */
    uint64_t grid_hash = uint64_t(pending_grid_objects_.size()) * 997;
    for (const PendingGridObject &pending : pending_grid_objects_) {
      const Object *ob = pending.ob;
      const float4x4 &mat = ob->object_to_world();

      /* Hash object transform (covers move/rotate/scale). */
      for (int i = 0; i < 4; i++) {
        grid_hash = grid_hash * 6364136223846793005ULL +
                    float_as_uint(mat[i][0]);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    float_as_uint(mat[i][1]);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    float_as_uint(mat[i][2]);
      }
      /* Hash bounds (changes when voxel_size or mesh changes). */
      const std::optional<Bounds<float3>> bounds = BKE_object_boundbox_get(ob);
      if (bounds) {
        grid_hash = grid_hash * 6364136223846793005ULL +
                    float_as_uint(bounds->min.x);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    float_as_uint(bounds->max.x);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    float_as_uint(bounds->min.y);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    float_as_uint(bounds->max.y);
      }
      /* Hash object color. */
      grid_hash = grid_hash * 6364136223846793005ULL +
                  float_as_uint(ob->color[0]);
      grid_hash = grid_hash * 6364136223846793005ULL +
                  float_as_uint(ob->color[1]);
      /* Hash session UID (detect object add/remove/replace). */
      grid_hash = grid_hash * 6364136223846793005ULL + uint64_t(ob->id.session_uid);
    }

    bool grids_changed = (grid_hash != grid_hash_);
    grid_hash_ = grid_hash;

    if (grids_changed) {
      /* Grid data changed: re-extract dense floats and create GPU textures. */
      free_grid_objects();
      for (const PendingGridObject &pending : pending_grid_objects_) {
        process_grid_object(pending.ob);
      }
    }

    if (objects_.is_empty() && grid_objects_.is_empty()) {
      needs_bake_ = false;
      return;
    }

    /* Compute atlas parameters. Resolution = voxels per Blender unit (fixed density).
     * voxel_size is constant for a given resolution setting. The grid adapts to
     * scene size with per-axis brick counts, snapped to world-aligned chunks.
     * Active-brick-only dispatch ensures bake cost is proportional to active bricks
     * (~5% of grid volume), not total grid_res^3. */
    float3 scene_size = scene_max_ - scene_min_;
    float margin = math::max(math::reduce_max(scene_size) * 0.1f, 0.5f);

    /* Fixed voxel density: resolution = voxels per BU.
     * Smooth auto-coarsen: if the padded scene extent would exceed
     * SDF_MAX_GRID_RES bricks, grow voxel_size by the exact amount needed.
     * Using max(base, extent-based) gives a perfectly linear ramp —
     * no snapping at all.  The -2 absorbs worst-case floor/ceil snap
     * overhead (one extra brick per side), guaranteeing the grid fits. */
    float base_voxel = 1.0f / float(sdf_resolution_);
    float3 padded_min = scene_min_ - float3(margin);
    float3 padded_max = scene_max_ + float3(margin);
    float max_extent = math::reduce_max(padded_max - padded_min);
    float min_voxel = max_extent /
                      (float(SDF_BRICK_SIZE) * float(SDF_MAX_GRID_RES - 2));
    voxel_size_ = math::max(base_voxel, min_voxel);
    float chunk_size = float(SDF_BRICK_SIZE) * voxel_size_;

    /* Snap to world-aligned chunk boundaries. */
    float3 grid_min = math::floor(padded_min / chunk_size) * chunk_size;
    float3 grid_max = math::ceil(padded_max / chunk_size) * chunk_size;

    /* Per-axis grid resolution in bricks. */
    int3 new_grid_res = int3(math::round((grid_max - grid_min) / chunk_size));
    new_grid_res = math::clamp(new_grid_res, int3(1), int3(SDF_MAX_GRID_RES));

    atlas_origin_ = grid_min;
    atlas_extent_ = grid_max - grid_min;

    /* Free textures only if grid dimensions actually changed. */
    if (new_grid_res != grid_res_) {
      grid_res_ = new_grid_res;
      if (indirection_tx_) {
        GPU_texture_free(indirection_tx_);
        indirection_tx_ = nullptr;
      }
      if (compact_atlas_tx_) {
        GPU_texture_free(compact_atlas_tx_);
        compact_atlas_tx_ = nullptr;
      }
      if (object_id_tx_) {
        GPU_texture_free(object_id_tx_);
        object_id_tx_ = nullptr;
      }
      atlas_capacity_ = 0;
      prev_active_brick_count_ = 0;
    }

    /* Compute scene hash for dirty tracking (analytic + grid combined). */
    uint64_t hash = uint64_t(objects_.size()) + grid_hash;
    for (const SDFObjectGPU &obj : objects_) {
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.position.x);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.position.y);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.position.z);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.sdf_size.x);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.sdf_size.y);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.sdf_size.z);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.bevel);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.blend);
      hash = hash * 6364136223846793005ULL + uint64_t(obj.blend_type);
      hash = hash * 6364136223846793005ULL + uint64_t(obj.sdf_type);
      hash = hash * 6364136223846793005ULL + uint64_t(obj.csg_operation);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.color.x);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.color.y);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.color.z);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.color.w);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.shell_distance);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.box_corners.x);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.box_corners.y);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.box_corners.z);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.box_corners.w);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.box_edges.x);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.box_edges.y);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.box_edges.z);
      hash = hash * 6364136223846793005ULL + float_as_uint(obj.box_edges.w);
      hash = hash * 6364136223846793005ULL + uint64_t(obj.box_modes.x);
      hash = hash * 6364136223846793005ULL + uint64_t(obj.box_modes.y);
      hash = hash * 6364136223846793005ULL + uint64_t(obj.box_modes.z);
      hash = hash * 6364136223846793005ULL +
             float_as_uint(obj.inverse_matrix[0][0]);
      hash = hash * 6364136223846793005ULL +
             float_as_uint(obj.inverse_matrix[1][1]);
      hash = hash * 6364136223846793005ULL +
             float_as_uint(obj.inverse_matrix[2][2]);
    }
    /* Include modifier data in hash. */
    for (const SDFModifierGPU &mod : modifiers_) {
      hash = hash * 6364136223846793005ULL + uint64_t(mod.header.x);
      hash = hash * 6364136223846793005ULL + uint64_t(mod.header.y);
      hash = hash * 6364136223846793005ULL + float_as_uint(mod.params.x);
      hash = hash * 6364136223846793005ULL + float_as_uint(mod.params.y);
      hash = hash * 6364136223846793005ULL + float_as_uint(mod.params.z);
      hash = hash * 6364136223846793005ULL + float_as_uint(mod.params.w);
    }
    /* Include surface margin so changes trigger rebake. */
    hash = hash * 6364136223846793005ULL + float_as_uint(surface_margin_);
    /* Include chunk-snapped grid parameters instead of raw scene bounds.
     * Small object movements within the same grid bounds only trigger
     * rebake from object data changes, not grid reallocation. */
    hash = hash * 6364136223846793005ULL + float_as_uint(atlas_origin_.x);
    hash = hash * 6364136223846793005ULL + float_as_uint(atlas_origin_.y);
    hash = hash * 6364136223846793005ULL + float_as_uint(atlas_origin_.z);
    hash = hash * 6364136223846793005ULL + uint64_t(grid_res_.x);
    hash = hash * 6364136223846793005ULL + uint64_t(grid_res_.y);
    hash = hash * 6364136223846793005ULL + uint64_t(grid_res_.z);

    if (hash != scene_hash_) {
      if (G.debug & G_DEBUG_GPU) {
        printf("[SDF] scene hash changed: %016llx -> %016llx  (objs=%d grids=%d res=%d)\n",
               (unsigned long long)scene_hash_,
               (unsigned long long)hash,
               int(objects_.size()),
               int(grid_objects_.size()),
               sdf_resolution_);
      }
      scene_hash_ = hash;
      needs_bake_ = true;
      needs_upload_ = true;
    }
    else {
      needs_bake_ = false;
    }

    /* Decide instanced vs world-space mode.
     * Instanced mode: per-shape local atlas + BVH instance traversal.
     * Requires: no smooth blending, at least one shape, no grid objects. */
    bool any_blend = false;
    for (const InstanceInfo &inst : instances_) {
      if (inst.blend > 0.0f) {
        any_blend = true;
        break;
      }
    }
    /* TODO: instanced mode disabled until coordinate/bake pipeline is verified.
     * Force world-space mode for now. */
    use_instanced_ = false && !any_blend && !shapes_.is_empty() && grid_objects_.is_empty() &&
                     pending_grid_objects_.is_empty();

    /* Compute per-shape atlas parameters and CPU classify when in instanced mode. */
    if (use_instanced_ && needs_bake_) {
      compute_shape_atlas_params();
      shape_classify_cpu();
    }
  }

  void draw(Manager & /*manager*/) final
  {
    if (objects_.is_empty() && grid_objects_.is_empty()) {
      return;
    }

    /* In depth mode, skip bake — only march with cached atlas for depth. */
    if (depth_mode_) {
      if (compact_atlas_tx_ == nullptr) {
        return; /* No atlas yet — nothing to march. */
      }
      sync_shading();
      ensure_shaders();
      if (march_sh_ == nullptr) {
        return;
      }
      DRW_submission_start();
      draw_march();
      DRW_submission_end();
      return;
    }

    /* sync_sdf_settings() moved to init() so end_sync() uses correct values. */
    sync_shading();
    ensure_shaders();

    /* Check if perf overlay is enabled. Only issue GL queries when active. */
    perf_enabled_ = draw_ctx_->v3d &&
                    (draw_ctx_->v3d->overlay.flag & V3D_OVERLAY_SDF_PERF) &&
                    !(draw_ctx_->v3d->flag2 & V3D_HIDE_OVERLAYS);
    if (perf_enabled_) {
      perf_init();
      perf_begin_frame();
    }

    /* Bail if critical shaders failed. */
    if (march_sh_ == nullptr) {
      return;
    }
    if (!objects_.is_empty() && (classify_sh_ == nullptr || bake_sh_ == nullptr)) {
      return;
    }
    if (!grid_objects_.is_empty() && grid_blend_sh_ == nullptr) {
      return;
    }

    DRW_submission_start();

    if (!objects_.is_empty() && needs_upload_) {
      upload_objects();
      upload_shapes_instances();
      build_bvh();
      upload_bvh();
      needs_upload_ = false;
    }

    /* Reset per-frame pass activity flags and record bake status. */
    if (perf_enabled_) {
      for (int i = 0; i < PERF_PASS_COUNT; i++) {
        perf_pass_active_[i] = false;
      }
      perf_currently_baking_ = needs_bake_;
    }

    if (needs_bake_) {
      /* Invalidate debug batches: atlas/BVH geometry may have changed. */
      if (grid_batch_) {
        GPU_batch_discard(grid_batch_);
        grid_batch_ = nullptr;
      }
      if (bvh_batch_) {
        GPU_batch_discard(bvh_batch_);
        bvh_batch_ = nullptr;
      }

      if (use_instanced_) {
        /* ---- Instanced pipeline: per-shape local atlas ---- */
        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_CLASSIFY);
        }
        /* shape_classify_cpu() was already called in end_sync(). */
        upload_shape_indirection();
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_CLASSIFY);
        }

        /* Size compact atlas from total shape bricks. */
        int new_bpa = int(std::ceil(std::cbrt(double(total_shape_active_bricks_))));
        if (new_bpa < 1) {
          new_bpa = 1;
        }
        bricks_per_axis_ = new_bpa;

        /* Create world-space indirection texture (needed for non-instanced fallback
         * paths like grid objects). Ensure it exists but don't classify into it. */
        ensure_indirection();
        int32_t clear_val = -1;
        GPU_texture_clear(indirection_tx_, GPU_DATA_INT, &clear_val);

        ensure_compact_atlas();

        /* Bake shapes (incremental: skip shapes whose fingerprints haven't changed). */
        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_BAKE);
        }
        if (!prev_shape_fingerprints_.is_empty()) {
          /* Find shapes that are new or changed since last frame. */
          Set<uint64_t> dirty_shapes;
          for (const ShapeInfo &shape : shapes_) {
            if (!prev_shape_fingerprints_.contains(shape.fingerprint)) {
              dirty_shapes.add(shape.fingerprint);
            }
          }
          if (dirty_shapes.is_empty()) {
            /* All shapes existed before — only instance transforms changed.
             * No rebake needed (atlas data is reusable). */
          }
          else {
            dispatch_shape_bake_all(&dirty_shapes);
          }
        }
        else {
          dispatch_shape_bake_all();
        }
        /* Update previous shape fingerprints for next frame. */
        prev_shape_fingerprints_.clear();
        for (const ShapeInfo &shape : shapes_) {
          prev_shape_fingerprints_.add(shape.fingerprint);
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_BAKE);
        }

        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_GRID);
          perf_end_pass(PERF_PASS_GRID);
        }
      }
      else {
        /* ---- World-space pipeline ---- */

        /* Phase 1: Classify analytic SDFs (sparse brick allocation). */
        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_CLASSIFY);
        }
        ensure_indirection();
        if (!objects_.is_empty()) {
          int32_t clear_val = -1;
          GPU_texture_clear(indirection_tx_, GPU_DATA_INT, &clear_val);
          dispatch_classify();
        }
        else {
          clear_indirection();
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_CLASSIFY);
        }

        if (!grid_objects_.is_empty()) {
          augment_indirection_for_grids();
        }

        /* Pre-size atlas using previous frame's count with headroom.
         * The bake shader reads the actual count from brick_counter SSBO,
         * so the atlas just needs to be large enough. Grow-only policy:
         * never shrink the atlas capacity to avoid thrashing. */
        {
          int estimated = math::max(active_brick_count_, prev_active_brick_count_);
          /* Add 50% headroom (minimum 64) to absorb count fluctuations. */
          int capacity = math::max(estimated + estimated / 2, 64);
          /* Never shrink below previous allocation. */
          if (capacity > atlas_capacity_) {
            atlas_capacity_ = capacity;
          }
          int new_bpa = int(std::ceil(std::cbrt(double(atlas_capacity_))));
          if (new_bpa < 1) {
            new_bpa = 1;
          }
          bricks_per_axis_ = new_bpa;
        }

        ensure_compact_atlas();

        /* Phase 2: Bake SDFs into atlas. */
        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_BAKE);
        }
        if (!objects_.is_empty()) {
          dispatch_bake();
          /* dispatch_bake does deferred readback: active_brick_count_ is now set. */

          /* If actual count exceeds atlas capacity, resize and re-bake. */
          if (active_brick_count_ > atlas_capacity_) {
            atlas_capacity_ = active_brick_count_ + active_brick_count_ / 2;
            int new_bpa = int(std::ceil(std::cbrt(double(atlas_capacity_))));
            if (new_bpa < 1) {
              new_bpa = 1;
            }
            bricks_per_axis_ = new_bpa;
            ensure_compact_atlas();
            dispatch_bake();
          }
          prev_active_brick_count_ = active_brick_count_;
        }
        else {
          clear_compact_atlas();
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_BAKE);
        }

        /* Blend grid objects into atlas. */
        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_GRID);
        }
        if (!grid_objects_.is_empty()) {
          dispatch_grid_blends();
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_GRID);
        }
      }

    }

    /* Ray march (runs every frame, even when cached). */
    if (perf_enabled_) {
      perf_begin_pass(PERF_PASS_MARCH);
    }
    draw_march();
    if (perf_enabled_) {
      perf_end_pass(PERF_PASS_MARCH);
    }

    /* FXAA post-process (runs every frame after march). */
    if (perf_enabled_) {
      perf_begin_pass(PERF_PASS_FXAA);
    }
    draw_fxaa();
    if (perf_enabled_) {
      perf_end_pass(PERF_PASS_FXAA);
    }

    /* Skip debug grid in Cycles rendered view — it draws on top of the
     * path-traced image since the SDF draw engine only provides depth. */
    if (!draw_ctx_->v3d || draw_ctx_->v3d->shading.type != OB_RENDER) {
      draw_debug_grid();
      draw_debug_bvh();
    }

    DRW_submission_end();

    /* Update static atlas state for cross-engine access (overlay selection). */
    s_compact_atlas = compact_atlas_tx_;
    s_indirection = indirection_tx_;
    s_object_id_atlas = object_id_tx_;
    s_voxel_size = voxel_size_;
    s_atlas_origin = atlas_origin_;
    s_atlas_extent = atlas_extent_;
    s_grid_resolution = grid_res_;
    s_bricks_per_axis = bricks_per_axis_;
    s_object_count = int(objects_.size());
    s_object_ssbo = object_ssbo_;

    if (perf_enabled_) {
      perf_end_frame(needs_bake_);
    }
    else {
      s_perf_active = false;
    }
  }

 private:
  void sync_sdf_settings()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      return;
    }

    const View3DShading &shading = v3d->shading;

    /* Read resolution setting. Default to 256 if unset (0).
     * Resolution = voxels per Blender unit (fixed density). Grid dimensions
     * are computed in end_sync() from chunk-snapped scene bounds. */
    int new_res = int(shading.sdf_resolution);
    if (new_res == 0) {
      new_res = 128;
    }
    new_res = math::clamp(new_res, 32, 128);

    if (new_res != sdf_resolution_) {
      if (G.debug & G_DEBUG_GPU) {
        printf("[SDF] resolution changed: %d -> %d\n", sdf_resolution_, new_res);
      }
      sdf_resolution_ = new_res;
      needs_bake_ = true;
      needs_upload_ = true;
      scene_hash_ = 0;
      atlas_capacity_ = 0;
      prev_active_brick_count_ = 0;
    }

    debug_grid_ = int(shading.sdf_debug_grid);
    bool new_fxaa = (U.sdf_fxaa != 0);
    if (!new_fxaa && fxaa_enabled_) {
      /* FXAA toggled off — free offscreen target to reclaim VRAM. */
      if (march_color_tx_) {
        GPU_texture_free(march_color_tx_);
        march_color_tx_ = nullptr;
      }
      if (march_fb_) {
        GPU_framebuffer_free(march_fb_);
        march_fb_ = nullptr;
      }
      fxaa_viewport_size_ = int2(0);
    }
    fxaa_enabled_ = new_fxaa;

    /* Surface margin: percentage → multiplier. Treat 0 (unset) as 100%. */
    int margin_pct = int(shading.sdf_surface_margin);
    if (margin_pct <= 0) {
      margin_pct = 100;
    }
    surface_margin_ = float(margin_pct) / 100.0f;
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
      if (sl != nullptr) {
        use_specular_ = ((shading.flag & V3D_SHADING_SPECULAR_HIGHLIGHT) &&
                         (sl->flag & STUDIOLIGHT_SPECULAR_HIGHLIGHT_PASS)) ?
                            1 :
                            0;
        for (int i = 0; i < 4; i++) {
          const SolidLight &light = sl->light[i];
          if (light.flag) {
            studio_light_dir_[i] = float4(light.vec[0], light.vec[1], light.vec[2], 0.0f);
            studio_light_col_[i] = float4(
                light.col[0], light.col[1], light.col[2], light.smooth);
            studio_light_spec_[i] = float4(light.spec[0], light.spec[1], light.spec[2], 0.0f);
          }
          else {
            studio_light_dir_[i] = float4(0.0f);
            studio_light_col_[i] = float4(0.0f);
            studio_light_spec_[i] = float4(0.0f);
          }
        }
        studio_ambient_ = float3(
            sl->light_ambient[0], sl->light_ambient[1], sl->light_ambient[2]);
      }
      else {
        studio_light_dir_[0] = float4(math::normalize(float3(0.5f, 0.7f, 1.0f)), 0.0f);
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
  }

  void ensure_shaders()
  {
    if (classify_sh_ == nullptr) {
      classify_sh_ = GPU_shader_create_from_info_name("sdf_classify");
    }
    if (bake_sh_ == nullptr) {
      bake_sh_ = GPU_shader_create_from_info_name("sdf_bake");
    }
    if (shape_bake_sh_ == nullptr) {
      shape_bake_sh_ = GPU_shader_create_from_info_name("sdf_shape_bake");
    }
    if (march_sh_ == nullptr) {
      march_sh_ = GPU_shader_create_from_info_name("sdf_march");
    }
    if (grid_blend_sh_ == nullptr) {
      grid_blend_sh_ = GPU_shader_create_from_info_name("sdf_grid_blend");
    }
    if (fxaa_sh_ == nullptr) {
      fxaa_sh_ = GPU_shader_create_from_info_name("sdf_fxaa");
    }
  }

  void ensure_indirection()
  {
    if (indirection_tx_ != nullptr) {
      return;
    }
    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE;
    indirection_tx_ = GPU_texture_create_3d("sdf_indirection",
                                            grid_res_.x,
                                            grid_res_.y,
                                            grid_res_.z,
                                            1,
                                            gpu::TextureFormat::SINT_32,
                                            usage,
                                            nullptr);
  }

  /** Ensure compact atlas and object ID atlas exist with correct dimensions. */
  void ensure_compact_atlas()
  {
    if (bricks_per_axis_ < 1) {
      bricks_per_axis_ = 1;
    }
    int atlas_dim = bricks_per_axis_ * SDF_BRICK_STORAGE;

    /* Check if existing textures have correct dimensions. */
    if (compact_atlas_tx_ != nullptr) {
      int existing_dim = GPU_texture_width(compact_atlas_tx_);
      if (existing_dim == atlas_dim) {
        return;
      }
      /* Dimensions changed: must recreate. */
      GPU_texture_free(compact_atlas_tx_);
      compact_atlas_tx_ = nullptr;
      if (object_id_tx_) {
        GPU_texture_free(object_id_tx_);
        object_id_tx_ = nullptr;
      }
    }

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE;
    compact_atlas_tx_ = GPU_texture_create_3d("sdf_compact_atlas",
                                              atlas_dim,
                                              atlas_dim,
                                              atlas_dim,
                                              1,
                                              gpu::TextureFormat::SFLOAT_16_16_16_16,
                                              usage,
                                              nullptr);
    GPU_texture_filter_mode(compact_atlas_tx_, true);
    object_id_tx_ = GPU_texture_create_3d("sdf_object_id_atlas",
                                          atlas_dim,
                                          atlas_dim,
                                          atlas_dim,
                                          1,
                                          gpu::TextureFormat::SINT_32,
                                          usage,
                                          nullptr);
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

    /* Upload modifier SSBO. Ensure at least 1 element for binding. */
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
  }

  /** Upload shape/instance tables to GPU SSBOs. */
  void upload_shapes_instances()
  {
    /* Build GPU shape array. */
    Vector<SDFShapeGPU> gpu_shapes(shapes_.size());
    for (int i = 0; i < shapes_.size(); i++) {
      SDFShapeGPU &gs = gpu_shapes[i];
      gs.size_normalized = float4(shapes_[i].size_normalized, 0.0f);
      gs.bevel_normalized = shapes_[i].bevel_normalized;
      gs.sdf_type = shapes_[i].sdf_type;
      gs.slot_offset = shapes_[i].slot_offset;
      gs.world_scale = shapes_[i].world_scale;
      gs.grid_params = int4(shapes_[i].grid_res, shapes_[i].indir_offset);
      gs.local_params = float4(shapes_[i].local_origin, shapes_[i].local_voxel_size);
      gs.atlas_params = int4(bricks_per_axis_, shapes_[i].active_brick_count, 0, 0);
    }

    /* Build GPU instance array. */
    Vector<SDFInstanceGPU> gpu_instances(instances_.size());
    for (int i = 0; i < instances_.size(); i++) {
      SDFInstanceGPU &gi = gpu_instances[i];
      gi.world_to_local = instances_[i].world_to_local;
      gi.local_to_world = instances_[i].local_to_world;
      gi.color = instances_[i].color;
      gi.blend = instances_[i].blend;
      gi.shape_id = instances_[i].shape_id;
      gi.object_id = instances_[i].object_id;
      gi.blend_type = instances_[i].blend_type;
      gi.csg_operation = instances_[i].csg_operation;
      gi._pad0 = 0.0f;
      gi._pad1 = 0.0f;
      gi._pad2 = 0.0f;
    }

    /* Upload shapes SSBO (reuse when count matches). */
    {
      const int count = int(gpu_shapes.size());
      if (shape_ssbo_ != nullptr && shape_ssbo_count_ != count) {
        GPU_storagebuf_free(shape_ssbo_);
        shape_ssbo_ = nullptr;
      }
      const size_t buf_size = math::max(count, 1) * sizeof(SDFShapeGPU);
      if (shape_ssbo_ == nullptr) {
        shape_ssbo_ = GPU_storagebuf_create_ex(
            buf_size,
            gpu_shapes.is_empty() ? nullptr : gpu_shapes.data(),
            GPU_USAGE_DYNAMIC,
            "sdf_shapes_ssbo");
        shape_ssbo_count_ = count;
      }
      else {
        GPU_storagebuf_update(shape_ssbo_, gpu_shapes.data());
      }
    }

    /* Upload instances SSBO (reuse when count matches). */
    {
      const int count = int(gpu_instances.size());
      if (instance_ssbo_ != nullptr && instance_ssbo_count_ != count) {
        GPU_storagebuf_free(instance_ssbo_);
        instance_ssbo_ = nullptr;
      }
      const size_t buf_size = math::max(count, 1) * sizeof(SDFInstanceGPU);
      if (instance_ssbo_ == nullptr) {
        instance_ssbo_ = GPU_storagebuf_create_ex(
            buf_size,
            gpu_instances.is_empty() ? nullptr : gpu_instances.data(),
            GPU_USAGE_DYNAMIC,
            "sdf_instances_ssbo");
        instance_ssbo_count_ = count;
      }
      else {
        GPU_storagebuf_update(instance_ssbo_, gpu_instances.data());
      }
    }
  }

  /* ---- SAH BVH Builder ---- */

  /** Build a top-down SAH BVH over object AABBs (8-bin partitioning).
   * Produces a flat array of BVHNodeGPU nodes suitable for stack-based GPU traversal. */
  void build_bvh()
  {
    bvh_nodes_.clear();
    const int n = int(objects_.size());
    if (n == 0) {
      return;
    }

    /* Build array of object indices for partitioning. */
    Vector<int> indices(n);
    for (int i = 0; i < n; i++) {
      indices[i] = i;
    }

    /* Precompute centroids for SAH binning. */
    Vector<float3> centroids(n);
    for (int i = 0; i < n; i++) {
      centroids[i] = (float3(objects_[i].bbox_min) + float3(objects_[i].bbox_max)) * 0.5f;
    }

    /* Reserve reasonable space: 2*N-1 for a complete binary tree. */
    bvh_nodes_.reserve(2 * n);

    build_bvh_recursive(indices.data(), n, centroids);
  }

  /** Recursively build BVH subtree. Returns the index of the root node for this subtree. */
  int build_bvh_recursive(int *indices, int count, const Vector<float3> &centroids)
  {
    /* Compute AABB of all objects in this subset. */
    float3 node_min = float3(1e30f);
    float3 node_max = float3(-1e30f);
    for (int i = 0; i < count; i++) {
      const SDFObjectGPU &obj = objects_[indices[i]];
      node_min = math::min(node_min, float3(obj.bbox_min));
      node_max = math::max(node_max, float3(obj.bbox_max));
    }

    if (count == 1) {
      /* Leaf node. */
      int node_idx = int(bvh_nodes_.size());
      BVHNodeGPU node = {};
      node.min_and_left = float4(node_min, 0.0f);
      node.max_and_right = float4(node_max, 0.0f);
      /* Encode: left = -1 (leaf marker), right = object index. */
      reinterpret_cast<int &>(node.min_and_left.w) = -1;
      reinterpret_cast<int &>(node.max_and_right.w) = indices[0];
      bvh_nodes_.append(node);
      return node_idx;
    }

    /* SAH 8-bin partitioning: find best split axis and position. */
    float3 centroid_min = float3(1e30f);
    float3 centroid_max = float3(-1e30f);
    for (int i = 0; i < count; i++) {
      centroid_min = math::min(centroid_min, centroids[indices[i]]);
      centroid_max = math::max(centroid_max, centroids[indices[i]]);
    }

    float3 centroid_extent = centroid_max - centroid_min;

    /* If all centroids are coincident, split in half. */
    if (math::reduce_max(centroid_extent) < 1e-6f) {
      int mid = count / 2;
      int node_idx = int(bvh_nodes_.size());
      bvh_nodes_.append({}); /* Placeholder. */

      int left = build_bvh_recursive(indices, mid, centroids);
      int right = build_bvh_recursive(indices + mid, count - mid, centroids);

      BVHNodeGPU &node = bvh_nodes_[node_idx];
      node.min_and_left = float4(node_min, 0.0f);
      node.max_and_right = float4(node_max, 0.0f);
      reinterpret_cast<int &>(node.min_and_left.w) = left;
      reinterpret_cast<int &>(node.max_and_right.w) = right;
      return node_idx;
    }

    constexpr int NUM_BINS = 8;
    float best_cost = 1e30f;
    int best_axis = 0;
    int best_split = -1;

    /* Surface area helper. */
    auto surface_area = [](float3 lo, float3 hi) -> float {
      float3 d = hi - lo;
      return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    };

    float parent_area = surface_area(node_min, node_max);

    for (int axis = 0; axis < 3; axis++) {
      if (centroid_extent[axis] < 1e-6f) {
        continue;
      }

      /* Bin objects by centroid along this axis. */
      struct Bin {
        float3 lo = float3(1e30f);
        float3 hi = float3(-1e30f);
        int count = 0;
      };
      Bin bins[NUM_BINS];

      float inv_extent = float(NUM_BINS) / centroid_extent[axis];
      for (int i = 0; i < count; i++) {
        int b = int((centroids[indices[i]][axis] - centroid_min[axis]) * inv_extent);
        b = math::clamp(b, 0, NUM_BINS - 1);
        bins[b].lo = math::min(bins[b].lo, float3(objects_[indices[i]].bbox_min));
        bins[b].hi = math::max(bins[b].hi, float3(objects_[indices[i]].bbox_max));
        bins[b].count++;
      }

      /* Sweep from left to find cumulative AABB and count for each split. */
      float3 left_min[NUM_BINS - 1];
      float3 left_max[NUM_BINS - 1];
      int left_count[NUM_BINS - 1];
      {
        float3 running_min = float3(1e30f);
        float3 running_max = float3(-1e30f);
        int running_count = 0;
        for (int i = 0; i < NUM_BINS - 1; i++) {
          running_min = math::min(running_min, bins[i].lo);
          running_max = math::max(running_max, bins[i].hi);
          running_count += bins[i].count;
          left_min[i] = running_min;
          left_max[i] = running_max;
          left_count[i] = running_count;
        }
      }

      /* Sweep from right and evaluate SAH cost for each split position. */
      {
        float3 running_min = float3(1e30f);
        float3 running_max = float3(-1e30f);
        int running_count = 0;
        for (int i = NUM_BINS - 1; i >= 1; i--) {
          running_min = math::min(running_min, bins[i].lo);
          running_max = math::max(running_max, bins[i].hi);
          running_count += bins[i].count;

          if (left_count[i - 1] > 0 && running_count > 0) {
            float left_area = surface_area(left_min[i - 1], left_max[i - 1]);
            float right_area = surface_area(running_min, running_max);
            float cost = 1.0f + (left_count[i - 1] * left_area + running_count * right_area) /
                                     parent_area;
            if (cost < best_cost) {
              best_cost = cost;
              best_axis = axis;
              best_split = i;
            }
          }
        }
      }
    }

    /* Partition indices around the best split. */
    int mid;
    if (best_split < 0) {
      /* No valid split found: fallback to equal split. */
      mid = count / 2;
    }
    else {
      float inv_extent = float(NUM_BINS) / centroid_extent[best_axis];
      /* Partition in-place: objects with bin < best_split go left. */
      int left_end = 0;
      for (int i = 0; i < count; i++) {
        int b = int((centroids[indices[i]][best_axis] - centroid_min[best_axis]) * inv_extent);
        b = math::clamp(b, 0, NUM_BINS - 1);
        if (b < best_split) {
          /* Swap to left partition. */
          int tmp = indices[left_end];
          indices[left_end] = indices[i];
          indices[i] = tmp;
          left_end++;
        }
      }
      mid = left_end;
      /* Safety: ensure non-empty partitions. */
      if (mid == 0) {
        mid = 1;
      }
      if (mid == count) {
        mid = count - 1;
      }
    }

    /* Allocate node with placeholder, then recurse. */
    int node_idx = int(bvh_nodes_.size());
    bvh_nodes_.append({}); /* Placeholder. */

    int left = build_bvh_recursive(indices, mid, centroids);
    int right = build_bvh_recursive(indices + mid, count - mid, centroids);

    BVHNodeGPU &node = bvh_nodes_[node_idx];
    node.min_and_left = float4(node_min, 0.0f);
    node.max_and_right = float4(node_max, 0.0f);
    reinterpret_cast<int &>(node.min_and_left.w) = left;
    reinterpret_cast<int &>(node.max_and_right.w) = right;
    return node_idx;
  }

  /** Upload BVH nodes to GPU SSBO. */
  void upload_bvh()
  {
    const int count = int(bvh_nodes_.size());
    if (count == 0) {
      return;
    }
    const size_t buf_size = count * sizeof(BVHNodeGPU);

    if (bvh_ssbo_ != nullptr && bvh_ssbo_count_ != count) {
      GPU_storagebuf_free(bvh_ssbo_);
      bvh_ssbo_ = nullptr;
    }

    if (bvh_ssbo_ == nullptr) {
      bvh_ssbo_ = GPU_storagebuf_create_ex(
          buf_size, bvh_nodes_.data(), GPU_USAGE_DYNAMIC, "sdf_bvh_ssbo");
      bvh_ssbo_count_ = count;
    }
    else {
      GPU_storagebuf_update(bvh_ssbo_, bvh_nodes_.data());
    }
  }


  /* ---- Per-Shape Atlas Pipeline (instanced mode) ---- */

  /** Compute per-shape local atlas parameters.
   * For each unique shape, determines the local grid resolution and voxel size
   * based on the shape's world_scale and the global resolution setting. */
  void compute_shape_atlas_params()
  {
    float base_voxel = 1.0f / float(sdf_resolution_);

    for (ShapeInfo &shape : shapes_) {
      /* Local voxel size: world voxel density scaled by shape's world size. */
      float local_vs = base_voxel / math::max(shape.world_scale, 1e-6f);

      /* Local extent per axis: size_normalized + bevel margin + overlap border.
       * The shape's SDF spans [-size, +size] in local space (half-extents).
       * Add bevel, brick_half_diag for classify margin, and 2 voxels for overlap. */
      float brick_half_diag = float(SDF_BRICK_SIZE) * local_vs * 0.866025f;
      float margin = brick_half_diag + 2.0f * local_vs;
      float3 half_extent = shape.size_normalized + float3(shape.bevel_normalized + margin);

      /* Grid resolution per axis. */
      float chunk = float(SDF_BRICK_SIZE) * local_vs;
      int3 gr;
      gr.x = math::clamp(int(std::ceil(2.0f * half_extent.x / chunk)), 1, SDF_MAX_SHAPE_GRID_RES);
      gr.y = math::clamp(int(std::ceil(2.0f * half_extent.y / chunk)), 1, SDF_MAX_SHAPE_GRID_RES);
      gr.z = math::clamp(int(std::ceil(2.0f * half_extent.z / chunk)), 1, SDF_MAX_SHAPE_GRID_RES);

      /* Snap local origin to brick boundaries (centered on shape origin). */
      float3 local_origin = -float3(gr) * float(SDF_BRICK_SIZE) * local_vs * 0.5f;

      shape.grid_res = gr;
      shape.local_origin = local_origin;
      shape.local_voxel_size = local_vs;
    }
  }

  /** CPU-side classify for all shapes.
   * For each shape, evaluates sdBox at each brick center to determine active bricks.
   * Builds the flat indirection array and active brick lists with global slot offsets. */
  void shape_classify_cpu()
  {
    shape_indir_data_.clear();
    shape_active_bricks_.clear();
    total_shape_active_bricks_ = 0;

    int global_indir_offset = 0;
    int global_slot_offset = 0;

    for (ShapeInfo &shape : shapes_) {
      int3 gr = shape.grid_res;
      float local_vs = shape.local_voxel_size;
      float3 local_orig = shape.local_origin;
      int grid_volume = gr.x * gr.y * gr.z;

      /* Record offset into flat indirection array. */
      shape.indir_offset = global_indir_offset;
      shape.slot_offset = global_slot_offset;

      /* Allocate space in the flat indirection array. */
      int indir_start = int(shape_indir_data_.size());
      shape_indir_data_.resize(indir_start + grid_volume, -1);

      /* Classify: evaluate single SDF at each brick center. */
      float3 sz = math::max(shape.size_normalized - float3(shape.bevel_normalized), float3(0.001f));
      float bev = shape.bevel_normalized;

      float brick_half_diag = float(SDF_BRICK_SIZE) * local_vs * 0.866025f;
      brick_half_diag *= surface_margin_;

      int shape_active = 0;
      for (int bz = 0; bz < gr.z; bz++) {
        for (int by = 0; by < gr.y; by++) {
          for (int bx = 0; bx < gr.x; bx++) {
            float3 brick_center = local_orig +
                                  (float3(float(bx), float(by), float(bz)) *
                                       float(SDF_BRICK_SIZE) +
                                   float(SDF_BRICK_SIZE) * 0.5f) *
                                      local_vs;

            /* Evaluate SDF based on shape type. */
            float dist;
            float3 p = brick_center;
            switch (shape.sdf_type) {
              case SDF_TYPE_SPHERE:
                dist = math::length(p) - sz.x;
                break;
              case SDF_TYPE_CYLINDER: {
                float2 pn = float2(p.x / sz.x, p.y / sz.y);
                float rn = math::length(pn);
                float2 g = float2(pn.x / sz.x, pn.y / sz.y) / math::max(rn, 1e-6f);
                float radial = (rn - 1.0f) / math::max(math::length(g), 1e-6f);
                float vertical = math::abs(p.z) - sz.z;
                dist = math::length(math::max(float2(radial, vertical), float2(0.0f))) +
                       math::min(math::max(radial, vertical), 0.0f);
                break;
              }
              case SDF_TYPE_CONE: {
                float cr = math::max(shape.size_normalized.x - bev, 0.001f);
                float ch = math::max(shape.size_normalized.y - bev, 0.001f);
                float2 q = float2(math::length(float2(p.x, p.y)), p.z);
                float2 k1 = float2(0.0f, ch);
                float2 k2 = float2(-cr, 2.0f * ch);
                float2 ca = float2(q.x - math::min(q.x, (q.y < 0.0f) ? cr : 0.0f),
                                   math::abs(q.y) - ch);
                float t_clamped = math::clamp(math::dot(k1 - q, k2) / math::dot(k2, k2),
                                              0.0f,
                                              1.0f);
                float2 cb = q - k1 + k2 * t_clamped;
                float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
                dist = s * std::sqrt(math::min(math::dot(ca, ca), math::dot(cb, cb)));
                break;
              }
              case SDF_TYPE_CAPSULE: {
                float3 pc = p;
                pc.z -= math::clamp(pc.z, -sz.y, sz.y);
                dist = math::length(pc) - sz.x;
                break;
              }
              case SDF_TYPE_TORUS: {
                float major = math::max(shape.size_normalized.x - bev, 0.001f);
                float minor = math::max(shape.size_normalized.y - bev, 0.001f);
                if (shape.torus_angle < 359.9f) {
                  /* Capped torus. */
                  float half_rad = shape.torus_angle * 0.5f * float(M_PI) / 180.0f;
                  float sc_x = sinf(half_rad);
                  float sc_y = cosf(half_rad);
                  float px = math::abs(p.x);
                  float k = (sc_y * px > sc_x * p.y)
                                ? (px * sc_y + p.y * sc_x)
                                : math::length(float2(px, p.y));
                  dist = std::sqrt(math::dot(float3(p.x, p.y, p.z), float3(p.x, p.y, p.z)) +
                                   major * major - 2.0f * major * k) -
                         minor;
                }
                else {
                  float2 qt = float2(math::length(float2(p.x, p.y)) - major, p.z);
                  dist = math::length(qt) - minor;
                }
                break;
              }
              case SDF_TYPE_NGON: {
                /* Simple regular polygon prism (no corner/edge/taper in instanced path). */
                float R = math::max(sz.x - bev, 0.001f);
                int sides = shape.ngon_sides;
                float an = M_PI / float(sides);
                float r_apothem = R * std::cos(an);
                float he = R * std::sin(an);
                /* Rotate 90 degrees: swap x,y. */
                float2 pp = float2(-p.y, p.x);
                float bn = an * std::floor((std::atan2(pp.y, pp.x) + an) / an / 2.0f) * 2.0f;
                float cs_x = std::cos(bn), cs_y = std::sin(bn);
                float2 pp2 = float2(cs_x * pp.x + cs_y * pp.y,
                                    -cs_y * pp.x + cs_x * pp.y);
                float d2d = math::length(
                                float2(pp2.x - r_apothem,
                                       pp2.y - math::clamp(pp2.y, -he, he))) *
                            ((pp2.x > r_apothem) ? 1.0f : -1.0f);
                float dz = math::abs(p.z) - math::max(sz.z - bev, 0.001f);
                dist = math::length(math::max(float2(d2d, dz), float2(0.0f))) +
                       math::min(math::max(d2d, dz), 0.0f);
                break;
              }
              default: {
                /* sdBox */
                float3 q = math::abs(p) - sz;
                dist = math::length(math::max(q, float3(0.0f))) +
                       math::min(math::max(q.x, math::max(q.y, q.z)), 0.0f);
                break;
              }
            }
            dist -= bev;

            if (math::abs(dist) < brick_half_diag) {
              /* Active brick: assign global slot. */
              int slot = global_slot_offset + shape_active;
              int flat_idx = bx + by * gr.x + bz * gr.x * gr.y;
              shape_indir_data_[indir_start + flat_idx] = slot;

              ActiveBrick ab;
              ab.coord = int4(bx, by, bz, slot);
              shape_active_bricks_.append(ab);
              shape_active++;
            }
          }
        }
      }

      shape.active_brick_count = shape_active;
      global_slot_offset += shape_active;
      global_indir_offset += grid_volume;
    }

    total_shape_active_bricks_ = global_slot_offset;
  }

  /** Upload the flat shape indirection SSBO. */
  void upload_shape_indirection()
  {
    if (shape_indir_data_.is_empty()) {
      return;
    }
    const int count = int(shape_indir_data_.size());
    const size_t buf_size = count * sizeof(int);
    if (shape_indir_ssbo_ != nullptr && shape_indir_ssbo_count_ != count) {
      GPU_storagebuf_free(shape_indir_ssbo_);
      shape_indir_ssbo_ = nullptr;
    }
    if (shape_indir_ssbo_ == nullptr) {
      shape_indir_ssbo_ = GPU_storagebuf_create_ex(
          buf_size, shape_indir_data_.data(), GPU_USAGE_DYNAMIC, "sdf_shape_indir_ssbo");
      shape_indir_ssbo_count_ = count;
    }
    else {
      GPU_storagebuf_update(shape_indir_ssbo_, shape_indir_data_.data());
    }
  }

  /** Dispatch per-shape bake for all shapes in instanced mode. */
  /** Dispatch bake for shapes, optionally skipping clean shapes.
   * @param dirty_only If non-null, only bake shapes whose fingerprints are in this set. */
  void dispatch_shape_bake_all(const Set<uint64_t> *dirty_only = nullptr)
  {
    if (total_shape_active_bricks_ <= 0 || shape_bake_sh_ == nullptr) {
      return;
    }

    /* Upload the concatenated active bricks SSBO once for all shapes.
     * Each shape uses a brick_offset uniform to index into its range,
     * eliminating per-shape SSBO alloc/free overhead. */
    gpu::StorageBuf *shape_ab_ssbo = GPU_storagebuf_create_ex(
        shape_active_bricks_.size() * sizeof(ActiveBrick),
        shape_active_bricks_.data(),
        GPU_USAGE_DEVICE_ONLY,
        "sdf_shape_active_bricks");

    GPU_shader_bind(shape_bake_sh_);

    /* Bind active bricks SSBO (shared across all shapes). */
    int ab_slot = GPU_shader_get_ssbo_binding(shape_bake_sh_, "active_bricks");
    GPU_storagebuf_bind(shape_ab_ssbo, ab_slot);

    /* Bind modifier SSBO (declared in shader info, may not be accessed). */
    if (modifier_ssbo_) {
      int mod_slot = GPU_shader_get_ssbo_binding(shape_bake_sh_, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, mod_slot);
    }

    /* Bind compact atlas as image. */
    GPU_texture_image_bind(compact_atlas_tx_, 0);

    /* Dispatch once per shape. Shapes write to non-overlapping atlas
     * slots, so no barrier is needed between dispatches. */
    int cur_brick_offset = 0;
    for (const ShapeInfo &shape : shapes_) {
      if (shape.active_brick_count <= 0) {
        continue;
      }

      bool skip = dirty_only && !dirty_only->contains(shape.fingerprint);

      if (!skip) {
        /* Push shape-specific constants. */
        GPU_shader_uniform_3fv(shape_bake_sh_, "shape_size", shape.size_normalized);
        GPU_shader_uniform_1f(shape_bake_sh_, "shape_bevel", shape.bevel_normalized);
        GPU_shader_uniform_1i(shape_bake_sh_, "shape_type", shape.sdf_type);
        GPU_shader_uniform_1i(shape_bake_sh_, "shape_sides", shape.ngon_sides);
        GPU_shader_uniform_1f(shape_bake_sh_, "shape_star", shape.ngon_star);
        {
          float half_rad = shape.torus_angle * 0.5f * float(M_PI) / 180.0f;
          float sc[2] = {sinf(half_rad), cosf(half_rad)};
          GPU_shader_uniform_2fv(shape_bake_sh_, "shape_torus_sc", sc);
        }
        GPU_shader_uniform_3fv(shape_bake_sh_, "local_origin", shape.local_origin);
        GPU_shader_uniform_1f(shape_bake_sh_, "local_voxel_size", shape.local_voxel_size);
        GPU_shader_uniform_1i(shape_bake_sh_, "bricks_per_axis", bricks_per_axis_);
        GPU_shader_uniform_1i(shape_bake_sh_, "active_brick_count", shape.active_brick_count);
        GPU_shader_uniform_1i(shape_bake_sh_, "brick_offset", cur_brick_offset);

        /* 2D dispatch to avoid 65535 limit. */
        uint bake_x = uint(math::min(shape.active_brick_count, 65535));
        uint bake_y = uint(divide_ceil_u(shape.active_brick_count, 65535));
        GPU_shader_uniform_1i(shape_bake_sh_, "dispatch_width", int(bake_x));
        GPU_compute_dispatch(shape_bake_sh_, bake_x, bake_y, 1);
      }

      /* Advance brick offset regardless of skip — offsets are computed for all shapes. */
      cur_brick_offset += shape.active_brick_count;
    }

    /* Single barrier after all shapes are baked (before march reads atlas). */
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);

    GPU_texture_image_unbind(compact_atlas_tx_);
    GPU_shader_unbind();
    GPU_storagebuf_free(shape_ab_ssbo);
  }


  void dispatch_classify()
  {
    /* Create / reset brick counter SSBO. */
    BrickCounter zero_counter = {};
    zero_counter.count = 0;
    zero_counter._pad0 = 0;
    zero_counter._pad1 = 0;
    zero_counter._pad2 = 0;

    if (brick_counter_ == nullptr) {
      brick_counter_ = GPU_storagebuf_create_ex(
          sizeof(BrickCounter), &zero_counter, GPU_USAGE_DYNAMIC, "sdf_brick_counter");
    }
    else {
      GPU_storagebuf_update(brick_counter_, &zero_counter);
    }

    /* Ensure active bricks SSBO is large enough for worst case (all bricks active). */
    int max_bricks = grid_res_.x * grid_res_.y * grid_res_.z;
    if (max_bricks < 1) {
      max_bricks = 1;
    }
    if (active_bricks_ == nullptr || active_bricks_capacity_ < max_bricks) {
      if (active_bricks_) {
        GPU_storagebuf_free(active_bricks_);
      }
      active_bricks_ = GPU_storagebuf_create_ex(
          max_bricks * sizeof(ActiveBrick), nullptr, GPU_USAGE_DYNAMIC, "sdf_active_bricks");
      active_bricks_capacity_ = max_bricks;
    }

    GPU_shader_bind(classify_sh_);

    /* Bind SSBOs. */
    int obj_slot = GPU_shader_get_ssbo_binding(classify_sh_, "objects");
    GPU_storagebuf_bind(object_ssbo_, obj_slot);

    int counter_slot = GPU_shader_get_ssbo_binding(classify_sh_, "brick_counter");
    GPU_storagebuf_bind(brick_counter_, counter_slot);

    int ab_slot = GPU_shader_get_ssbo_binding(classify_sh_, "active_bricks");
    GPU_storagebuf_bind(active_bricks_, ab_slot);

    /* Bind BVH SSBO. */
    if (bvh_ssbo_) {
      int bvh_slot = GPU_shader_get_ssbo_binding(classify_sh_, "bvh_nodes");
      GPU_storagebuf_bind(bvh_ssbo_, bvh_slot);
    }

    /* Bind modifier SSBO. */
    if (modifier_ssbo_) {
      int mod_slot = GPU_shader_get_ssbo_binding(classify_sh_, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, mod_slot);
    }

    /* Bind indirection texture as image. */
    GPU_texture_image_bind(indirection_tx_, 0);

    /* Push constants. */
    GPU_shader_uniform_1i(classify_sh_, "object_count", int(objects_.size()));
    GPU_shader_uniform_1f(classify_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(classify_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_3iv(classify_sh_, "grid_resolution", grid_res_);

    /* Brick half-diagonal: conservative surface test distance.
     * The shell operation in combineCSG produces correct thin-wall distances
     * via two-stage blend, so no extra global expansion is needed.
     * surface_margin_ (UI "Surface Margin" %) further widens the shell. */
    float brick_half_diag = float(SDF_BRICK_SIZE) * voxel_size_ * 0.866025f; /* sqrt(3)/2 */
    brick_half_diag *= surface_margin_;
    GPU_shader_uniform_1f(classify_sh_, "brick_half_diag", brick_half_diag);
    GPU_shader_uniform_1f(classify_sh_, "max_blend", max_blend_);
    GPU_shader_uniform_1f(classify_sh_, "max_shell_distance", max_shell_distance_);
    GPU_shader_uniform_1i(classify_sh_, "bvh_node_count", int(bvh_nodes_.size()));

    /* Dispatch: one thread per brick, local group size is 4x4x4. */
    GPU_compute_dispatch(classify_sh_,
                         divide_ceil_u(grid_res_.x, 4),
                         divide_ceil_u(grid_res_.y, 4),
                         divide_ceil_u(grid_res_.z, 4));

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_SHADER_STORAGE);
    GPU_texture_image_unbind(indirection_tx_);
    GPU_shader_unbind();

    /* Deferred readback: the bake shader reads brick_counter.count from the
     * SSBO directly, avoiding the GPU→CPU pipeline stall in the common case.
     * active_brick_count_ is updated after bake in dispatch_bake().
     *
     * Exception: when grid objects need augmenting, we must know the count now
     * because augment_indirection_for_grids() assigns sequential slots starting
     * from active_brick_count_. In that case, do the synchronous readback. */
    if (!grid_objects_.is_empty()) {
      BrickCounter readback = {};
      GPU_storagebuf_read(brick_counter_, &readback);
      active_brick_count_ = int(readback.count);
    }
  }

  void dispatch_bake()
  {
    /* Over-dispatch with capacity: the bake shader reads the actual active
     * count from brick_counter SSBO and early-exits surplus workgroups.
     * This avoids the GPU→CPU readback stall between classify and bake. */
    int dispatch_count = active_bricks_capacity_;
    if (dispatch_count <= 0) {
      return;
    }

    GPU_shader_bind(bake_sh_);

    /* Bind SSBOs. */
    int ssbo_slot = GPU_shader_get_ssbo_binding(bake_sh_, "objects");
    GPU_storagebuf_bind(object_ssbo_, ssbo_slot);

    int ab_slot = GPU_shader_get_ssbo_binding(bake_sh_, "active_bricks");
    GPU_storagebuf_bind(active_bricks_, ab_slot);

    if (bvh_ssbo_) {
      int bvh_slot = GPU_shader_get_ssbo_binding(bake_sh_, "bvh_nodes");
      GPU_storagebuf_bind(bvh_ssbo_, bvh_slot);
    }

    /* Bind brick counter SSBO (bake shader reads count from here). */
    int counter_slot = GPU_shader_get_ssbo_binding(bake_sh_, "brick_counter");
    GPU_storagebuf_bind(brick_counter_, counter_slot);

    /* Bind modifier SSBO. */
    if (modifier_ssbo_) {
      int mod_slot = GPU_shader_get_ssbo_binding(bake_sh_, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, mod_slot);
    }

    /* Bind atlas images. */
    GPU_texture_image_bind(compact_atlas_tx_, 0);
    if (object_id_tx_) {
      GPU_texture_image_bind(object_id_tx_, 1);
    }

    /* Push constants. */
    GPU_shader_uniform_1i(bake_sh_, "object_count", int(objects_.size()));
    GPU_shader_uniform_1f(bake_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(bake_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_1i(bake_sh_, "bricks_per_axis", bricks_per_axis_);
    GPU_shader_uniform_1f(bake_sh_, "max_blend", max_blend_);
    GPU_shader_uniform_1f(bake_sh_, "max_shell_distance", max_shell_distance_);
    GPU_shader_uniform_1i(bake_sh_, "bvh_node_count", int(bvh_nodes_.size()));

    /* Over-dispatch: one workgroup per capacity slot. Surplus workgroups
     * early-exit after reading brick_counter.count from SSBO. */
    uint bake_x = uint(math::min(dispatch_count, 65535));
    uint bake_y = uint(divide_ceil_u(dispatch_count, 65535));
    GPU_shader_uniform_1i(bake_sh_, "dispatch_width", int(bake_x));
    GPU_compute_dispatch(bake_sh_, bake_x, bake_y, 1);

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_SHADER_STORAGE);
    GPU_texture_image_unbind(compact_atlas_tx_);
    if (object_id_tx_) {
      GPU_texture_image_unbind(object_id_tx_);
    }
    GPU_shader_unbind();

    /* Deferred readback: now that bake is dispatched and barrier issued,
     * read the actual brick count for atlas sizing and perf stats.
     * No stall here — the bake barrier already flushed the pipeline. */
    BrickCounter readback = {};
    GPU_storagebuf_read(brick_counter_, &readback);
    active_brick_count_ = int(readback.count);
  }



  void draw_march()
  {
    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();

    if (draw_ctx_->is_depth() ||
        (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER))
    {
      /* Depth-only: either a depth prepass, or rendered mode where Cycles
       * provides the color and we only contribute depth for the overlay grid. */
      GPU_framebuffer_bind(dfbl->depth_only_fb);
    }
    else if (fxaa_enabled_) {
      /* Solid / wireframe with FXAA: render to offscreen texture for post-processing.
       * Depth is shared with the viewport so depth writes still land correctly. */
      ensure_fxaa_target();
      GPU_framebuffer_bind(march_fb_);
      float clear_color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      GPU_framebuffer_clear_color(march_fb_, clear_color);
    }
    else {
      /* Solid / wireframe without FXAA: render directly to default framebuffer. */
      GPU_framebuffer_bind(dfbl->default_fb);
    }

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    /* TODO: Scissor rect optimization disabled — needs correct AABB projection.
     * Was clipping SDF rendering incorrectly. Re-enable once debugged. */
    bool scissor_set = false;

    GPU_shader_bind(march_sh_);

    /* Bind compact atlas as sampler. */
    int atlas_slot = GPU_shader_get_sampler_binding(march_sh_, "compact_atlas");
    if (compact_atlas_tx_) {
      GPU_texture_bind(compact_atlas_tx_, atlas_slot);
    }

    /* Bind indirection as sampler. */
    int indir_slot = GPU_shader_get_sampler_binding(march_sh_, "indirection_tx");
    if (indirection_tx_) {
      GPU_texture_bind(indirection_tx_, indir_slot);
    }

    /* Bind object ID atlas for debug visualization. */
    int objid_slot = GPU_shader_get_sampler_binding(march_sh_, "object_id_tx");
    if (object_id_tx_) {
      GPU_texture_bind(object_id_tx_, objid_slot);
    }

    /* Bind instanced mode SSBOs. */
    if (shape_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "shapes");
      GPU_storagebuf_bind(shape_ssbo_, slot);
    }
    if (instance_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "instances");
      GPU_storagebuf_bind(instance_ssbo_, slot);
    }
    if (bvh_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "bvh_nodes");
      GPU_storagebuf_bind(bvh_ssbo_, slot);
    }
    if (shape_indir_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "shape_indir");
      GPU_storagebuf_bind(shape_indir_ssbo_, slot);
    }
    if (modifier_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, slot);
    }

    /* Push constants. */
    GPU_shader_uniform_1f(march_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(march_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_3fv(march_sh_, "atlas_extent", atlas_extent_);
    GPU_shader_uniform_3iv(march_sh_, "grid_resolution", grid_res_);
    GPU_shader_uniform_1i(march_sh_, "bricks_per_axis", bricks_per_axis_);
    GPU_shader_uniform_1i(march_sh_, "lighting_type", lighting_type_);
    GPU_shader_uniform_1i(march_sh_, "use_specular", use_specular_);
    GPU_shader_uniform_1i(march_sh_, "use_matcap_flip", use_matcap_flip_);
    GPU_shader_uniform_1i(march_sh_, "normal_quality", 1); /* 0=fast, 1=smooth */
    GPU_shader_uniform_1i(march_sh_, "debug_mode", (debug_grid_ == 2) ? 1 : 0);
    GPU_shader_uniform_1i(march_sh_, "use_instanced", use_instanced_ ? 1 : 0);
    GPU_shader_uniform_1i(march_sh_, "instance_count", int(instances_.size()));
    GPU_shader_uniform_1i(march_sh_, "bvh_node_count", int(bvh_nodes_.size()));

    GPU_shader_uniform_4fv(march_sh_, "studio_light0", studio_light_dir_[0]);
    GPU_shader_uniform_4fv(march_sh_, "studio_light1", studio_light_dir_[1]);
    GPU_shader_uniform_4fv(march_sh_, "studio_light2", studio_light_dir_[2]);
    GPU_shader_uniform_4fv(march_sh_, "studio_light3", studio_light_dir_[3]);
    GPU_shader_uniform_4fv(march_sh_, "studio_color0", studio_light_col_[0]);
    GPU_shader_uniform_4fv(march_sh_, "studio_color1", studio_light_col_[1]);
    GPU_shader_uniform_4fv(march_sh_, "studio_color2", studio_light_col_[2]);
    GPU_shader_uniform_4fv(march_sh_, "studio_color3", studio_light_col_[3]);
    GPU_shader_uniform_4fv(march_sh_, "studio_spec0", studio_light_spec_[0]);
    GPU_shader_uniform_4fv(march_sh_, "studio_spec1", studio_light_spec_[1]);
    GPU_shader_uniform_4fv(march_sh_, "studio_spec2", studio_light_spec_[2]);
    GPU_shader_uniform_4fv(march_sh_, "studio_spec3", studio_light_spec_[3]);
    GPU_shader_uniform_3fv(march_sh_, "studio_ambient", studio_ambient_);

    /* Bind matcap texture. */
    if (matcap_tx_) {
      int matcap_slot = GPU_shader_get_sampler_binding(march_sh_, "matcap_tx");
      GPU_texture_bind(matcap_tx_, matcap_slot);
    }

    /* Bind the view UBO so the fragment shader can access camera matrices.
     * Must call push_update() before binding — this uploads the current frame's
     * matrices to the GPU. Without it, the GPU-side UBO contains stale data from
     * the previous frame, causing incorrect gl_FragDepth during camera movement.
     * View::bind() does this internally but is protected; we call push_update()
     * directly on the public UBO reference. */
    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, march_sh_);
    GPU_batch_draw(fullscreen_batch_);

    if (scissor_set) {
      GPU_scissor_test(false);
    }

    if (compact_atlas_tx_) {
      GPU_texture_unbind(compact_atlas_tx_);
    }
    if (indirection_tx_) {
      GPU_texture_unbind(indirection_tx_);
    }
    if (object_id_tx_) {
      GPU_texture_unbind(object_id_tx_);
    }
    if (matcap_tx_) {
      GPU_texture_unbind(matcap_tx_);
    }
    GPU_shader_unbind();
  }

  /* ---- FXAA post-process ---- */

  /** Ensure the offscreen color texture and framebuffer for march → FXAA exist
   * and match the current viewport size. */
  void ensure_fxaa_target()
  {
    const int2 vp = int2(draw_ctx_->viewport_size_get());
    if (march_color_tx_ != nullptr && fxaa_viewport_size_ == vp) {
      return;
    }

    /* (Re)create on size change. */
    if (march_color_tx_) {
      GPU_texture_free(march_color_tx_);
      march_color_tx_ = nullptr;
    }
    if (march_fb_) {
      GPU_framebuffer_free(march_fb_);
      march_fb_ = nullptr;
    }

    fxaa_viewport_size_ = vp;

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT;
    march_color_tx_ = GPU_texture_create_2d(
        "sdf_march_color", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);

    /* Build framebuffer: offscreen color + viewport depth (shared). */
    DefaultTextureList *dtxl = draw_ctx_->viewport_texture_list_get();
    march_fb_ = GPU_framebuffer_create("sdf_march_fb");
    GPU_framebuffer_texture_attach(march_fb_, march_color_tx_, 0, 0);
    GPU_framebuffer_texture_attach(march_fb_, dtxl->depth, 0, 0);
  }

  /** Apply FXAA to the offscreen march result and composite to the default FB. */
  void draw_fxaa()
  {
    if (!fxaa_enabled_ || fxaa_sh_ == nullptr) {
      return;
    }

    /* Skip in depth-only or Cycles rendered mode (matches draw_march behavior). */
    if (draw_ctx_->is_depth() ||
        (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER))
    {
      return;
    }

    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();
    GPU_framebuffer_bind(dfbl->default_fb);

    /* FXAA is screen-space color only — depth was already written by march. */
    GPU_depth_test(GPU_DEPTH_NONE);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    GPU_shader_bind(fxaa_sh_);

    int color_slot = GPU_shader_get_sampler_binding(fxaa_sh_, "color_tx");
    GPU_texture_filter_mode(march_color_tx_, true);
    GPU_texture_bind(march_color_tx_, color_slot);

    float2 rcp = float2(1.0f / float(fxaa_viewport_size_.x),
                        1.0f / float(fxaa_viewport_size_.y));
    GPU_shader_uniform_2fv(fxaa_sh_, "rcpFrame", rcp);

    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, fxaa_sh_);
    GPU_batch_draw(fullscreen_batch_);

    GPU_texture_unbind(march_color_tx_);
    GPU_shader_unbind();

    /* Restore depth state for subsequent passes (debug grid, overlays). */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
  }

  /* ---- Debug grid wireframe ---- */

  gpu::Batch *create_line_batch(const float3 *positions, int vert_count)
  {
    if (vert_count <= 0) {
      return nullptr;
    }

    GPUVertFormat format = {};
    GPU_vertformat_attr_add(&format, "pos", gpu::VertAttrType::SFLOAT_32_32_32);

    gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
    GPU_vertbuf_data_alloc(*vbo, uint(vert_count));
    GPU_vertbuf_attr_fill(vbo, 0, positions);

    return GPU_batch_create_ex(GPU_PRIM_LINES, vbo, nullptr, GPU_BATCH_OWNS_VBO);
  }

  /** Create a GPU_PRIM_LINES batch with per-vertex position and color. */
  gpu::Batch *create_colored_line_batch(const float3 *positions,
                                        const float4 *colors,
                                        int vert_count)
  {
    if (vert_count <= 0) {
      return nullptr;
    }

    GPUVertFormat format = {};
    uint pos_attr = GPU_vertformat_attr_add(
        &format, "pos", gpu::VertAttrType::SFLOAT_32_32_32);
    uint col_attr = GPU_vertformat_attr_add(
        &format, "color", gpu::VertAttrType::SFLOAT_32_32_32_32);

    gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
    GPU_vertbuf_data_alloc(*vbo, uint(vert_count));
    GPU_vertbuf_attr_fill(vbo, pos_attr, positions);
    GPU_vertbuf_attr_fill(vbo, col_attr, colors);

    return GPU_batch_create_ex(GPU_PRIM_LINES, vbo, nullptr, GPU_BATCH_OWNS_VBO);
  }

  void rebuild_grid_batch_active()
  {
    if (!indirection_tx_) {
      return;
    }

    /* Read back indirection texture to find active bricks. */
    int32_t *data = static_cast<int32_t *>(
        GPU_texture_read(indirection_tx_, GPU_DATA_INT, 0));
    if (!data) {
      return;
    }

    int3 n = grid_res_;
    float brick_world = float(SDF_BRICK_SIZE) * voxel_size_;

    /* Count active bricks. */
    int total = n.x * n.y * n.z;
    int active_count = 0;
    for (int i = 0; i < total; i++) {
      if (data[i] >= 0) {
        active_count++;
      }
    }

    if (active_count == 0) {
      MEM_freeN(data);
      return;
    }

    /* 12 edges per cube, 2 vertices per edge = 24 vertices per active brick. */
    Vector<float3> positions(active_count * 24);
    int vi = 0;

    for (int z = 0; z < n.z; z++) {
      for (int y = 0; y < n.y; y++) {
        for (int x = 0; x < n.x; x++) {
          int idx = x + y * n.x + z * n.x * n.y;
          if (data[idx] < 0) {
            continue;
          }

          float3 lo = atlas_origin_ + float3(float(x), float(y), float(z)) * brick_world;
          float3 hi = lo + float3(brick_world);

          /* Bottom face edges. */
          positions[vi++] = float3(lo.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, hi.y, lo.z);
          positions[vi++] = float3(hi.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, lo.y, lo.z);

          /* Top face edges. */
          positions[vi++] = float3(lo.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, hi.y, hi.z);
          positions[vi++] = float3(hi.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, lo.y, hi.z);

          /* Vertical edges. */
          positions[vi++] = float3(lo.x, lo.y, lo.z);
          positions[vi++] = float3(lo.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, hi.y, lo.z);
          positions[vi++] = float3(hi.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, hi.y, hi.z);
        }
      }
    }

    MEM_freeN(data);
    grid_batch_ = create_line_batch(positions.data(), vi);
  }

  /** Emit 12 wireframe edges (24 vertices) for an axis-aligned box. */
  static void emit_box_edges(Vector<float3> &positions,
                             Vector<float4> &colors,
                             const float3 &lo,
                             const float3 &hi,
                             const float4 &col)
  {
    /* Bottom face. */
    positions.append(float3(lo.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, lo.y, lo.z));
    colors.append(col);

    /* Top face. */
    positions.append(float3(lo.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(lo.x, lo.y, hi.z));
    colors.append(col);

    /* Vertical edges. */
    positions.append(float3(lo.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, hi.z));
    colors.append(col);
  }

  /** HSV to RGB (H in [0,1], S in [0,1], V in [0,1]). */
  static float3 hsv_to_rgb(float h, float s, float v)
  {
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h * 6.0f, 2.0f) - 1.0f));
    float m = v - c;
    float3 rgb;
    if (h < 1.0f / 6.0f) {
      rgb = float3(c, x, 0);
    }
    else if (h < 2.0f / 6.0f) {
      rgb = float3(x, c, 0);
    }
    else if (h < 3.0f / 6.0f) {
      rgb = float3(0, c, x);
    }
    else if (h < 4.0f / 6.0f) {
      rgb = float3(0, x, c);
    }
    else if (h < 5.0f / 6.0f) {
      rgb = float3(x, 0, c);
    }
    else {
      rgb = float3(c, 0, x);
    }
    return rgb + float3(m);
  }

  /** Build a colored wireframe batch from the BVH tree and per-object OBBs. */
  void rebuild_bvh_batch()
  {
    if (bvh_nodes_.is_empty()) {
      return;
    }

    const int node_count = int(bvh_nodes_.size());
    const int obj_count = int(objects_.size());

    /* --- Compute depth for each node via iterative stack-based DFS --- */
    Vector<int> depth(node_count, 0);
    int max_depth = 0;

    struct StackEntry {
      int node_idx;
      int node_depth;
    };
    Vector<StackEntry> stack;
    stack.append({0, 0});

    while (!stack.is_empty()) {
      StackEntry entry = stack.last();
      stack.remove_last();

      if (entry.node_idx < 0 || entry.node_idx >= node_count) {
        continue;
      }

      depth[entry.node_idx] = entry.node_depth;
      max_depth = max_ii(max_depth, entry.node_depth);

      const BVHNodeGPU &node = bvh_nodes_[entry.node_idx];
      int left = reinterpret_cast<const int &>(node.min_and_left.w);
      int right = reinterpret_cast<const int &>(node.max_and_right.w);

      if (left >= 0) {
        /* Interior node: push children. */
        stack.append({left, entry.node_depth + 1});
        stack.append({right, entry.node_depth + 1});
      }
      /* Leaf: left == -1, nothing to push. */
    }

    /* Preallocate: 24 verts per BVH node + 24 verts per object OBB. */
    Vector<float3> positions;
    Vector<float4> colors;
    positions.reserve((node_count + obj_count) * 24);
    colors.reserve((node_count + obj_count) * 24);

    /* --- BVH node AABBs: depth-colored red→blue --- */
    float depth_range = max_ii(max_depth, 1);
    for (int i = 0; i < node_count; i++) {
      const BVHNodeGPU &node = bvh_nodes_[i];
      float3 lo = float3(node.min_and_left);
      float3 hi = float3(node.max_and_right);

      float t = float(depth[i]) / depth_range;
      float hue = t * 0.67f; /* 0 = red, 0.67 = blue. */
      float3 rgb = hsv_to_rgb(hue, 0.85f, 0.9f);
      float4 col = float4(rgb, 0.85f);

      emit_box_edges(positions, colors, lo, hi, col);
    }

    /* --- Per-object OBBs: white, rotated wireframes --- */
    float4 white = float4(1.0f, 1.0f, 1.0f, 0.9f);
    for (int i = 0; i < obj_count; i++) {
      const SDFObjectGPU &obj = objects_[i];

      /* Reconstruct rotation matrix from inverse_matrix.
       * inverse_matrix is the inverse of the rotation-only matrix.
       * For pure rotation: inverse = transpose, so rot = transpose(inv). */
      float3 rot_col0 = float3(obj.inverse_matrix[0][0],
                               obj.inverse_matrix[1][0],
                               obj.inverse_matrix[2][0]);
      float3 rot_col1 = float3(obj.inverse_matrix[0][1],
                               obj.inverse_matrix[1][1],
                               obj.inverse_matrix[2][1]);
      float3 rot_col2 = float3(obj.inverse_matrix[0][2],
                               obj.inverse_matrix[1][2],
                               obj.inverse_matrix[2][2]);

      float3 pos = float3(obj.position);
      float3 extent = float3(obj.sdf_size) + float3(obj.bevel + obj.blend);

      /* Generate 8 OBB corners. */
      float3 corners[8];
      for (int c = 0; c < 8; c++) {
        float3 local = float3((c & 1) ? extent.x : -extent.x,
                              (c & 2) ? extent.y : -extent.y,
                              (c & 4) ? extent.z : -extent.z);
        corners[c] = rot_col0 * local.x + rot_col1 * local.y + rot_col2 * local.z + pos;
      }

      /* 12 edges of a box: bottom(4), top(4), vertical(4).
       * Corner indices: bottom = 0,1,3,2  top = 4,5,7,6 */
      const int edges[12][2] = {
          {0, 1}, {1, 3}, {3, 2}, {2, 0}, /* bottom */
          {4, 5}, {5, 7}, {7, 6}, {6, 4}, /* top */
          {0, 4}, {1, 5}, {3, 7}, {2, 6}, /* vertical */
      };
      for (int e = 0; e < 12; e++) {
        positions.append(corners[edges[e][0]]);
        colors.append(white);
        positions.append(corners[edges[e][1]]);
        colors.append(white);
      }
    }

    bvh_batch_ = create_colored_line_batch(
        positions.data(), colors.data(), int(positions.size()));
  }

  void rebuild_grid_batch()
  {
    if (grid_batch_) {
      GPU_batch_discard(grid_batch_);
      grid_batch_ = nullptr;
    }
    if (bvh_batch_) {
      GPU_batch_discard(bvh_batch_);
      bvh_batch_ = nullptr;
    }

    grid_batch_mode_ = debug_grid_;
    grid_batch_res_ = grid_res_;

    if (debug_grid_ == 1) {
      rebuild_grid_batch_active();
    }
    else if (debug_grid_ == 3) {
      rebuild_bvh_batch();
    }
  }

  void draw_debug_grid()
  {
    if (debug_grid_ != 1) {
      return;
    }

    /* Rebuild batch if settings changed. */
    if (grid_batch_ == nullptr || grid_batch_mode_ != debug_grid_ ||
        grid_batch_res_ != grid_res_)
    {
      rebuild_grid_batch();
    }
    if (grid_batch_ == nullptr) {
      return;
    }

    /* Use builtin 3D uniform color shader. */
    gpu::Shader *shader = GPU_shader_get_builtin_shader(GPU_SHADER_3D_UNIFORM_COLOR);
    GPU_shader_bind(shader);

    /* MVP = persmat (lines are in world space, no model transform). */
    View &view = View::default_get();
    float4x4 mvp = view.persmat();
    GPU_shader_uniform_mat4(shader, "ModelViewProjectionMatrix", mvp.ptr());

    GPU_blend(GPU_BLEND_ALPHA);
    GPU_depth_mask(false); /* Grid lines don't write depth. */

    GPU_batch_set_shader(grid_batch_, shader);

    /* --- Pass 1: Occluded lines (behind SDF surface) ---
     * Drawn first so front lines overdraw on top. Faint ghost
     * lines give spatial context without visual clutter. */
    GPU_depth_test(GPU_DEPTH_GREATER);
    GPU_line_width(1.0f);
    GPU_shader_uniform_4f(shader, "color", 0.0f, 0.6f, 0.0f, 0.1f);
    GPU_batch_draw(grid_batch_);

    /* --- Pass 2: Front lines (in front of SDF surface) ---
     * Bright and slightly thicker for clear readability. */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_line_width(1.5f);
    GPU_shader_uniform_4f(shader, "color", 0.0f, 1.0f, 0.0f, 0.8f);
    GPU_batch_draw(grid_batch_);

    /* Restore state. */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_line_width(1.0f);
    GPU_shader_unbind();
  }

  void draw_debug_bvh()
  {
    if (debug_grid_ != 3) {
      return;
    }

    /* Rebuild batch if settings changed or batch was invalidated. */
    if (bvh_batch_ == nullptr || grid_batch_mode_ != debug_grid_) {
      rebuild_grid_batch();
    }
    if (bvh_batch_ == nullptr) {
      return;
    }

    /* Use builtin flat-color shader (per-vertex color). */
    gpu::Shader *shader = GPU_shader_get_builtin_shader(GPU_SHADER_3D_FLAT_COLOR);
    GPU_shader_bind(shader);

    /* MVP = persmat (lines are in world space). */
    View &view = View::default_get();
    float4x4 mvp = view.persmat();
    GPU_shader_uniform_mat4(shader, "ModelViewProjectionMatrix", mvp.ptr());

    GPU_blend(GPU_BLEND_ALPHA);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(false);
    GPU_line_width(1.5f);

    GPU_batch_set_shader(bvh_batch_, shader);
    GPU_batch_draw(bvh_batch_);

    /* Restore state. */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_line_width(1.0f);
    GPU_shader_unbind();
  }

  void process_grid_object(const Object *ob)
  {
    if (!ob->runtime || !ob->runtime->geometry_set_eval) {
      return;
    }

    const bke::GeometrySet &geo = *ob->runtime->geometry_set_eval;
    const Volume *volume = geo.get_volume();
    if (!volume || BKE_volume_num_grids(volume) == 0) {
      return;
    }

    const bke::VolumeGridData *vgrid = BKE_volume_grid_get(volume, 0);
    if (!vgrid) {
      return;
    }

    /* Extract dense float voxels. */
    DenseFloatVolumeGrid dense = {};
    if (!BKE_volume_grid_dense_floats(volume, vgrid, &dense)) {
      return;
    }

    int w = dense.resolution[0];
    int h = dense.resolution[1];
    int d = dense.resolution[2];
    if (w <= 0 || h <= 0 || d <= 0) {
      BKE_volume_dense_float_grid_clear(&dense);
      return;
    }

    /* Copy the texture-to-object matrix before we free the dense data. */
    float4x4 tex_to_obj;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        tex_to_obj[i][j] = dense.texture_to_object[i][j];
      }
    }

    /* Create GPU 3D texture from dense data (R32F for full precision). */
    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ;
    gpu::Texture *tex = GPU_texture_create_3d(
        "sdf_grid", w, h, d, 1, gpu::TextureFormat::SFLOAT_32, usage, dense.voxels);
    BKE_volume_dense_float_grid_clear(&dense);

    if (!tex) {
      return;
    }
    GPU_texture_filter_mode(tex, true);

    /* Compute world_to_texture transform.
     * DenseFloatVolumeGrid::texture_to_object maps normalized [0,1]^3 -> object space.
     * We need: world_to_texture = inverse(object_to_world * texture_to_object) */
    float4x4 tex_to_world = ob->object_to_world() * tex_to_obj;
    float4x4 world_to_tex = math::invert(tex_to_world);

    /* Scene AABB is accumulated in object_sync() via BKE_object_boundbox_get().
     * Do NOT expand it here — this function only runs when grids change,
     * so expanding here creates AABB inconsistency between bake frames (larger)
     * and cached frames (smaller), causing objects to appear at wrong sizes. */

    /* Read blend from the "Mesh to SDF Grid" modifier's IDProperties. */
    float blend_val = 0.0f;
    LISTBASE_FOREACH (ModifierData *, md, &ob->modifiers) {
      if (md->type != eModifierType_Nodes) {
        continue;
      }
      NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
      if (!nmd->node_group) {
        continue;
      }
      /* Check if this is our "Mesh to SDF Grid" node group. */
      const char *group_name = nmd->node_group->id.name + 2; /* Skip "NT" prefix. */
      if (std::strcmp(group_name, "Mesh to SDF Grid") != 0) {
        continue;
      }
      /* Find the "Blend" socket and read its value from IDProperties. */
      if (nmd->settings.properties) {
        /* The socket identifier for the Blend input. We iterate interface_inputs
         * to find it, but as a simpler approach, just search all properties for
         * one named with the identifier that has a float type. The Blend socket
         * is the 4th input (index 3), but socket identifiers are assigned by
         * Blender and may vary. Search by iterating all float properties and
         * matching the node group's interface. */
        const bNodeTreeInterface &iface = nmd->node_group->tree_interface;
        iface.foreach_item([&](const bNodeTreeInterfaceItem &item) {
          if (item.item_type != NODE_INTERFACE_SOCKET) {
            return true; /* Continue iteration. */
          }
          const auto &socket = reinterpret_cast<const bNodeTreeInterfaceSocket &>(item);
          if (StringRef(socket.name) == "Blend" &&
              StringRef(socket.socket_type) == "NodeSocketFloat")
          {
            IDProperty *prop = IDP_GetPropertyFromGroup(nmd->settings.properties,
                                                        socket.identifier);
            if (prop && prop->type == IDP_FLOAT) {
              blend_val = IDP_float_get(prop);
            }
            else if (prop && prop->type == IDP_DOUBLE) {
              blend_val = float(IDP_double_get(prop));
            }
            return false; /* Stop iteration. */
          }
          return true; /* Continue iteration. */
        });
      }
      break;
    }

    /* Read object color. */
    float4 color = float4(ob->color[0], ob->color[1], ob->color[2], ob->color[3]);

    GridObject grid_obj = {};
    grid_obj.texture = tex;
    grid_obj.world_to_texture = world_to_tex;
    grid_obj.color = color;
    grid_obj.blend = blend_val;
    grid_obj.blend_type = 0;      /* Default to linear for grid objects. */
    grid_obj.csg_operation = 0;   /* Default to union for grid objects. */
    grid_obj.shell_distance = 0.0f;

    /* If the object has SDF data, use its blend_type and csg_operation. */
    if (ob->type == OB_SDF && ob->data) {
      const SDF *sdf_data = static_cast<const SDF *>(ob->data);
      grid_obj.blend_type = sdf_data->blend_type;
      grid_obj.csg_operation = sdf_data->csg_operation;
      grid_obj.shell_distance = sdf_data->shell_distance;
    }

    grid_objects_.append(grid_obj);
  }

  void free_grid_objects()
  {
    for (GridObject &grid : grid_objects_) {
      if (grid.texture) {
        GPU_texture_free(grid.texture);
      }
    }
    grid_objects_.clear();
  }

  void clear_indirection()
  {
    /* Initialize all bricks as outside (-1).
     * Use GPU_texture_clear instead of allocating a CPU-side vector
     * (which could be up to 8MB at grid_res 128^3). */
    int neg_one = -1;
    GPU_texture_clear(indirection_tx_, GPU_DATA_INT, &neg_one);
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    active_brick_count_ = 0;
  }

  void augment_indirection_for_grids()
  {
    /* Mark all bricks overlapping each grid object's world AABB as active.
     * Uses simple sequential slot assignment. */
    float brick_world = voxel_size_ * float(SDF_BRICK_SIZE);

    /* Read current indirection (already written by classify or clear). */
    int32_t *data = static_cast<int32_t *>(
        GPU_texture_read(indirection_tx_, GPU_DATA_INT, 0));

    Vector<ActiveBrick> new_bricks;
    int next_slot = active_brick_count_;

    for (const GridObject &grid : grid_objects_) {
      float4x4 tex_to_world = math::invert(grid.world_to_texture);
      float3 gmin = float3(1e30f);
      float3 gmax = float3(-1e30f);
      for (int c = 0; c < 8; c++) {
        float3 tc = float3(
            (c & 1) ? 1.0f : 0.0f, (c & 2) ? 1.0f : 0.0f, (c & 4) ? 1.0f : 0.0f);
        float3 wc = math::transform_point(tex_to_world, tc);
        gmin = math::min(gmin, wc);
        gmax = math::max(gmax, wc);
      }

      int3 bmin = int3(math::floor((gmin - atlas_origin_) / brick_world));
      int3 bmax = int3(math::floor((gmax - atlas_origin_) / brick_world));
      bmin = math::max(bmin, int3(0));
      bmax = math::min(bmax, grid_res_ - int3(1));

      for (int bz = bmin.z; bz <= bmax.z; bz++) {
        for (int by = bmin.y; by <= bmax.y; by++) {
          for (int bx = bmin.x; bx <= bmax.x; bx++) {
            int idx = bx + by * grid_res_.x + bz * grid_res_.x * grid_res_.y;
            if (data[idx] >= 0) {
              continue; /* Already active from analytic classify. */
            }
            int slot = next_slot++;
            data[idx] = slot;

            ActiveBrick ab = {};
            ab.coord = int4(bx, by, bz, slot);
            new_bricks.append(ab);
          }
        }
      }
    }

    active_brick_count_ = next_slot;

    /* Re-upload modified indirection. */
    GPU_texture_update(indirection_tx_, GPU_DATA_INT, data);
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    MEM_freeN(data);

    /* Append grid bricks to the active_bricks SSBO. */
    if (!new_bricks.is_empty()) {
      int classify_count = next_slot - int(new_bricks.size());

      /* Read existing bricks from the SSBO. The read returns the full SSBO
       * (active_bricks_capacity_ entries), so the buffer must be large enough. */
      Vector<ActiveBrick> all_bricks(math::max(active_brick_count_, active_bricks_capacity_));
      if (classify_count > 0 && active_bricks_) {
        GPU_storagebuf_read(active_bricks_, all_bricks.data());
      }
      for (int i = 0; i < int(new_bricks.size()); i++) {
        all_bricks[classify_count + i] = new_bricks[i];
      }

      if (active_bricks_) {
        GPU_storagebuf_free(active_bricks_);
      }
      active_bricks_ = GPU_storagebuf_create_ex(active_brick_count_ * sizeof(ActiveBrick),
                                                 all_bricks.data(),
                                                 GPU_USAGE_DYNAMIC,
                                                 "sdf_active_bricks");
      active_bricks_capacity_ = active_brick_count_;
      GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
    }
  }

  void clear_compact_atlas()
  {
    /* For grid-only scenes, initialize the compact atlas to large distance (1e10)
     * so that grid blend's min-union works correctly.
     * R = 1e10 (large distance), GBA = 0 (no color). */
    if (!compact_atlas_tx_) {
      return;
    }
    float clear_val[4] = {1e10f, 0.0f, 0.0f, 0.0f};
    GPU_texture_clear(compact_atlas_tx_, GPU_DATA_FLOAT, clear_val);
    GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }

  void dispatch_grid_blends()
  {
    if (grid_blend_sh_ == nullptr || compact_atlas_tx_ == nullptr ||
        active_brick_count_ <= 0 || active_bricks_ == nullptr)
    {
      return;
    }

    GPU_shader_bind(grid_blend_sh_);

    for (const GridObject &grid : grid_objects_) {
      /* Bind active bricks SSBO. */
      int ab_slot = GPU_shader_get_ssbo_binding(grid_blend_sh_, "active_bricks");
      GPU_storagebuf_bind(active_bricks_, ab_slot);

      /* Bind modifier SSBO (declared in shader info, may not be accessed). */
      if (modifier_ssbo_) {
        int mod_slot = GPU_shader_get_ssbo_binding(grid_blend_sh_, "sdf_modifiers");
        GPU_storagebuf_bind(modifier_ssbo_, mod_slot);
      }

      /* Bind compact atlas as read-write image. */
      GPU_texture_image_bind(compact_atlas_tx_, 0);

      /* Bind grid texture as sampler. */
      int grid_slot = GPU_shader_get_sampler_binding(grid_blend_sh_, "sdf_grid");
      GPU_texture_bind(grid.texture, grid_slot);

      /* Push constants. */
      GPU_shader_uniform_1f(grid_blend_sh_, "voxel_size", voxel_size_);
      GPU_shader_uniform_3fv(grid_blend_sh_, "atlas_origin", atlas_origin_);
      GPU_shader_uniform_1i(grid_blend_sh_, "bricks_per_axis", bricks_per_axis_);
      GPU_shader_uniform_mat4(
          grid_blend_sh_, "grid_world_to_texture", grid.world_to_texture.ptr());
      GPU_shader_uniform_4fv(grid_blend_sh_, "grid_color", grid.color);
      GPU_shader_uniform_1f(grid_blend_sh_, "grid_blend", grid.blend);
      GPU_shader_uniform_1i(grid_blend_sh_, "grid_blend_type", grid.blend_type);
      GPU_shader_uniform_1i(grid_blend_sh_, "grid_csg_operation", grid.csg_operation);
      GPU_shader_uniform_1f(grid_blend_sh_, "grid_shell_distance", grid.shell_distance);
      GPU_shader_uniform_1i(grid_blend_sh_, "active_brick_count", active_brick_count_);

      /* Active-brick-only dispatch: one workgroup per active brick.
       * Use 2D dispatch to avoid GL's 65535 workgroup limit per axis. */
      uint gb_x = uint(math::min(active_brick_count_, 65535));
      uint gb_y = uint(divide_ceil_u(active_brick_count_, 65535));
      GPU_shader_uniform_1i(grid_blend_sh_, "dispatch_width", int(gb_x));
      GPU_compute_dispatch(grid_blend_sh_, gb_x, gb_y, 1);

      /* Barrier between dispatches so next grid reads the updated atlas. */
      GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);

      GPU_texture_unbind(grid.texture);
      GPU_texture_image_unbind(compact_atlas_tx_);
    }

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    GPU_shader_unbind();
  }

  /* ---- Performance overlay helpers ---- */

  /** Initialize perf tracking state (idempotent). */
  void perf_init()
  {
    if (perf_queries_created_) {
      return;
    }
    perf_queries_created_ = true;
  }

  /** Update wall-clock FPS rolling average. */
  void perf_begin_frame()
  {
    double now = BLI_time_now_seconds();
    if (perf_last_draw_time_ > 0.0) {
      double dt = now - perf_last_draw_time_;
      if (dt > 0.0) {
        double instant_fps = 1.0 / dt;
        perf_fps_samples_[perf_fps_sample_idx_] = instant_fps;
        perf_fps_sample_idx_ = (perf_fps_sample_idx_ + 1) % PERF_FPS_SAMPLES;
        if (perf_fps_sample_count_ < PERF_FPS_SAMPLES) {
          perf_fps_sample_count_++;
        }
        double sum = 0.0;
        for (int i = 0; i < perf_fps_sample_count_; i++) {
          sum += perf_fps_samples_[i];
        }
        perf_fps_ = sum / double(perf_fps_sample_count_);
        perf_frame_ms_ = dt * 1000.0;
      }
    }
    perf_last_draw_time_ = now;
  }

  /** Begin timing for the given pass. Snapshots wall-clock time. */
  void perf_begin_pass(int pass)
  {
    perf_pass_start_[pass] = BLI_time_now_seconds();
    perf_pass_active_[pass] = true;
  }

  /** End timing for the given pass.
   * Flushes command queue and waits for GPU completion, then computes elapsed time. */
  void perf_end_pass(int pass)
  {
    GPU_flush();
    GPU_finish();
    double elapsed_ms = (BLI_time_now_seconds() - perf_pass_start_[pass]) * 1000.0;
    switch (pass) {
      case PERF_PASS_CLASSIFY:
        perf_classify_ms_ = elapsed_ms;
        break;
      case PERF_PASS_BAKE:
        perf_bake_ms_ = elapsed_ms;
        break;
      case PERF_PASS_GRID:
        perf_grid_ms_ = elapsed_ms;
        break;
      case PERF_PASS_MARCH:
        perf_march_ms_ = elapsed_ms;
        break;
      case PERF_PASS_FXAA:
        perf_fxaa_ms_ = elapsed_ms;
        break;
    }
  }

  /** Format results and update bake cache. */
  void perf_end_frame(bool /*currently_baking*/)
  {
    perf_prev_baked_ = perf_currently_baking_;
    if (perf_prev_baked_) {
      perf_last_classify_ms_ = perf_classify_ms_;
      perf_last_bake_ms_ = perf_bake_ms_;
      perf_last_grid_ms_ = perf_grid_ms_;
      perf_has_bake_data_ = true;
    }

    /* Format the text. */
    int total_bricks = grid_res_.x * grid_res_.y * grid_res_.z;
    float brick_pct = (total_bricks > 0) ?
                          100.0f * float(active_brick_count_) / float(total_bricks) :
                          0.0f;

    /* Display logic:
     * - perf_prev_baked_: previous frame's readback had bake data → show live timing.
     * - !perf_prev_baked_ && perf_has_bake_data_: idle but have cached values.
     * - Neither: never baked yet → dashes. */
    char classify_str[48], bake_str[48], grid_str[48];
    if (perf_prev_baked_) {
      /* Live data from the frame we just read back. */
      std::snprintf(classify_str, sizeof(classify_str), "%.2f ms", perf_classify_ms_);
      std::snprintf(bake_str, sizeof(bake_str), "%.2f ms", perf_bake_ms_);
      std::snprintf(grid_str, sizeof(grid_str), "%.2f ms", perf_grid_ms_);
    }
    else if (perf_has_bake_data_) {
      /* Idle: show last bake cost so user can still see it. */
      std::snprintf(
          classify_str, sizeof(classify_str), "%.2f ms", perf_last_classify_ms_);
      std::snprintf(bake_str, sizeof(bake_str), "%.2f ms", perf_last_bake_ms_);
      std::snprintf(grid_str, sizeof(grid_str), "%.2f ms", perf_last_grid_ms_);
    }
    else {
      std::snprintf(classify_str, sizeof(classify_str), "%s", "\xe2\x80\x94");
      std::snprintf(bake_str, sizeof(bake_str), "%s", "\xe2\x80\x94");
      std::snprintf(grid_str, sizeof(grid_str), "%s", "\xe2\x80\x94");
    }

    std::snprintf(s_perf_text,
                  SDF_PERF_BUF_SIZE,
                  "SDF Performance\n"
                  "  fps: %.1f  frame: %.1f ms\n"
                  "  classify: %s\n"
                  "  bake: %s\n"
                  "  grid blend: %s\n"
                  "  march: %.2f ms\n"
                  "  fxaa: %.2f ms\n"
                  "  bricks: %d / %d (%.1f%%)",
                  perf_fps_,
                  perf_frame_ms_,
                  classify_str,
                  bake_str,
                  grid_str,
                  perf_march_ms_,
                  perf_fxaa_ms_,
                  active_brick_count_,
                  total_bricks,
                  brick_pct);
    s_perf_active = true;
  }

  /** Reset perf state. */
  void perf_cleanup()
  {
    perf_queries_created_ = false;
    s_perf_active = false;
    s_perf_text[0] = '\0';
  }

 public:
  ~Instance() override
  {
    perf_cleanup();
    free_grid_objects();

    /* Clear static pointers BEFORE freeing resources to prevent
     * dangling access from overlay/selection code. */
    s_compact_atlas = nullptr;
    s_indirection = nullptr;
    s_object_id_atlas = nullptr;
    s_object_ssbo = nullptr;
    s_perf_active = false;
    if (march_color_tx_) {
      GPU_texture_free(march_color_tx_);
    }
    if (march_fb_) {
      GPU_framebuffer_free(march_fb_);
    }
    if (indirection_tx_) {
      GPU_texture_free(indirection_tx_);
    }
    if (compact_atlas_tx_) {
      GPU_texture_free(compact_atlas_tx_);
    }
    if (object_id_tx_) {
      GPU_texture_free(object_id_tx_);
    }
    if (brick_counter_) {
      GPU_storagebuf_free(brick_counter_);
    }
    if (active_bricks_) {
      GPU_storagebuf_free(active_bricks_);
    }
    if (matcap_tx_) {
      GPU_texture_free(matcap_tx_);
    }
    if (object_ssbo_) {
      GPU_storagebuf_free(object_ssbo_);
    }
    if (modifier_ssbo_) {
      GPU_storagebuf_free(modifier_ssbo_);
    }
    if (bvh_ssbo_) {
      GPU_storagebuf_free(bvh_ssbo_);
    }
    if (shape_ssbo_) {
      GPU_storagebuf_free(shape_ssbo_);
    }
    if (instance_ssbo_) {
      GPU_storagebuf_free(instance_ssbo_);
    }
    if (shape_indir_ssbo_) {
      GPU_storagebuf_free(shape_indir_ssbo_);
    }
    if (fullscreen_batch_) {
      GPU_batch_discard(fullscreen_batch_);
    }
    if (grid_batch_) {
      GPU_batch_discard(grid_batch_);
    }
    if (bvh_batch_) {
      GPU_batch_discard(bvh_batch_);
    }
  }
};

/* ---- Public perf API ---- */

const char *sdf_perf_info_get()
{
  return s_perf_active ? s_perf_text : nullptr;
}

bool sdf_perf_active()
{
  return s_perf_active;
}

/* ---- Public atlas API ---- */

gpu::Texture *sdf_atlas_get()
{
  return s_compact_atlas;
}

gpu::Texture *sdf_indirection_get()
{
  return s_indirection;
}

gpu::Texture *sdf_object_id_atlas_get()
{
  return s_object_id_atlas;
}

void sdf_atlas_params_get(
    float *voxel_size, float3 *origin, float3 *extent, int3 *grid_resolution, int *bricks_per_axis)
{
  *voxel_size = s_voxel_size;
  *origin = s_atlas_origin;
  *extent = s_atlas_extent;
  *grid_resolution = s_grid_resolution;
  *bricks_per_axis = s_bricks_per_axis;
}

int sdf_object_count_get()
{
  return s_object_count;
}

gpu::StorageBuf *sdf_objects_ssbo_get()
{
  return s_object_ssbo;
}

DrawEngine *Engine::create_instance()
{
  return new Instance();
}

}  // namespace blender::draw::sdf
