/* SPDX-FileCopyrightText: 2025 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* Blender SDF sync: converts Blender SDF objects into a sparse brick atlas
 * for Cycles ray marching. Mirrors the draw engine's classify→bake pipeline
 * but runs on CPU instead of GPU compute shaders. */

#include "scene/sdf.h"
#include "scene/image.h"
#include "scene/object.h"
#include "scene/scene.h"
#include "scene/shader.h"

#include "blender/sync.h"
#include "blender/util.h"

#include "util/hash.h"
#include "util/log.h"
#include "util/map.h"
#include "util/math.h"
#include "util/task.h"
#include "util/transform.h"
#include "util/vector.h"

CCL_NAMESPACE_BEGIN

/* -------------------------------------------------------------------- */
/* uint16 distance encoding. */

/* Range in voxel-units: covers full brick storage diagonal (12*sqrt(3)/2 ≈ 10.4). */
static constexpr float SDF_DIST16_RANGE = 12.0f;

static inline uint16_t sdf_encode_dist16(float d, float voxel_size)
{
  float d_voxels = d / voxel_size;
  float t = (d_voxels + SDF_DIST16_RANGE) / (2.0f * SDF_DIST16_RANGE);
  t = max(0.0f, min(1.0f, t));
  return (uint16_t)(t * 65535.0f + 0.5f);
}

/* -------------------------------------------------------------------- */
/* SDF math primitives (ported from sdf_lib.glsl). */

static float sdf_box(const float3 p, const float3 b)
{
  float3 q = make_float3(fabsf(p.x) - b.x, fabsf(p.y) - b.y, fabsf(p.z) - b.z);
  float3 q_pos = make_float3(max(q.x, 0.0f), max(q.y, 0.0f), max(q.z, 0.0f));
  return len(q_pos) + min(max(q.x, max(q.y, q.z)), 0.0f);
}

static float sdf_sphere(const float3 p, const float r)
{
  return len(p) - r;
}

static float sdf_capsule(const float3 p, const float h, const float r)
{
  float3 pc = p;
  pc.z -= clamp(pc.z, -h, h);
  return len(pc) - r;
}

static float sdf_torus(const float3 p, const float R, const float r)
{
  float q_x = sqrtf(p.x * p.x + p.y * p.y) - R;
  return sqrtf(q_x * q_x + p.z * p.z) - r;
}

static float op_smooth_union(float d1, float d2, float k)
{
  if (k <= 0.0f) {
    return min(d1, d2);
  }
  float h = clamp(0.5f + 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
  return d2 * (1.0f - h) + d1 * h - k * h * (1.0f - h);
}

/* -------------------------------------------------------------------- */
/* Per-object SDF data collected from Blender RNA. */

struct SDFObjectData {
  Transform itfm;     /* Rotation-only inverse (scale baked into size). */
  float3 position;    /* World-space translation. */
  float3 sdf_size;    /* SDF size with object scale. */
  float bevel;        /* Bevel radius (world-space). */
  float blend;        /* Smooth blend radius. */
  float3 color;       /* Object display color. */
  float3 bbox_min;    /* World AABB min. */
  float3 bbox_max;    /* World AABB max. */
  int sdf_type;       /* eSDFType enum. */
  int shader_index;   /* Cycles shader index for this object. */
  Shader *shader;     /* Resolved Cycles shader (from material slot). */
};

/* Evaluate SDF distance for a single object at a world-space point. */
static float evaluate_sdf_object(const SDFObjectData &obj, const float3 world_pos)
{
  /* Transform to object-local space. */
  float3 local_pos = world_pos - obj.position;
  local_pos = transform_direction(&obj.itfm, local_pos);

  float3 size = obj.sdf_size - make_float3(obj.bevel, obj.bevel, obj.bevel);
  size = max(size, make_float3(0.001f, 0.001f, 0.001f));

  float dist;
  switch (obj.sdf_type) {
    case 1: /* SDF_TYPE_SPHERE */
      dist = sdf_sphere(local_pos, size.x) - obj.bevel;
      break;
    case 4: /* SDF_TYPE_CAPSULE */
      dist = sdf_capsule(local_pos, size.z, size.x) - obj.bevel;
      break;
    case 5: /* SDF_TYPE_TORUS */
      dist = sdf_torus(local_pos, size.x, size.y) - obj.bevel;
      break;
    default: /* SDF_TYPE_BOX (0) */
      dist = sdf_box(local_pos, size) - obj.bevel;
      break;
  }
  return dist;
}

/* -------------------------------------------------------------------- */
/* Shape fingerprinting for instancing. */

static uint64_t sdf_shape_fingerprint(int sdf_type, const float3 &effective_size, float effective_bevel)
{
  /* Normalize by max component for scale-independence. */
  float max_dim = max(max(fabsf(effective_size.x), fabsf(effective_size.y)),
                      fabsf(effective_size.z));
  if (max_dim < 1e-8f) {
    max_dim = 1.0f;
  }

  /* Quantize to 0.01% precision. */
  const int Q = 10000;
  int sx = (int)roundf(effective_size.x / max_dim * Q);
  int sy = (int)roundf(effective_size.y / max_dim * Q);
  int sz = (int)roundf(effective_size.z / max_dim * Q);
  int sb = (int)roundf(effective_bevel / max_dim * Q);

  /* FNV-1a 64-bit hash. */
  uint64_t h = 14695981039346656037ULL;
  h ^= (uint64_t)sdf_type;  h *= 1099511628211ULL;
  h ^= (uint64_t)sx;        h *= 1099511628211ULL;
  h ^= (uint64_t)sy;        h *= 1099511628211ULL;
  h ^= (uint64_t)sz;        h *= 1099511628211ULL;
  h ^= (uint64_t)sb;        h *= 1099511628211ULL;
  return h;
}

/* -------------------------------------------------------------------- */
/* Constants matching the draw engine. */

static constexpr int BRICK_SIZE = 8;
static constexpr int BRICK_STORAGE = 12; /* 8 interior + 2-voxel border on each side. */
static constexpr int BRICK_BORDER = 2;

/* -------------------------------------------------------------------- */
/* Main sync function. */

void BlenderSync::sync_sdf(BObjectInfo &b_ob_info, SDFGeometry *sdf_geom)
{
  /* Collect ALL SDF objects in the scene (not just this one).
   * All SDFs are baked into one shared atlas. */
  vector<SDFObjectData> sdf_objects;
  float3 scene_min_acc = make_float3(1e30f, 1e30f, 1e30f);
  float3 scene_max_acc = make_float3(-1e30f, -1e30f, -1e30f);

  /* Iterate over all depsgraph objects.
   * b_depsgraph_ptr is set by sync_objects() before geometry sync starts. */
  assert(b_depsgraph_ptr != nullptr);
  BL::Depsgraph &b_depsgraph = *b_depsgraph_ptr;
  BL::Depsgraph::object_instances_iterator b_instance_iter;
  for (b_depsgraph.object_instances.begin(b_instance_iter);
       b_instance_iter != b_depsgraph.object_instances.end();
       ++b_instance_iter)
  {
    BL::DepsgraphObjectInstance b_instance = *b_instance_iter;
    BL::Object b_ob = b_instance.object();

    if (b_ob.type() != BL::Object::type_SDF) {
      continue;
    }
    if (!b_instance.show_self()) {
      continue;
    }

    BL::ID b_data = b_ob.data();
    if (!b_data) {
      continue;
    }

    /* Read object transform. */
    Transform tfm = get_transform(b_ob.matrix_world());

    /* Decompose: extract per-axis scale from column lengths of the 3x3 part.
     * For M = T * R * S, column j has length s_j. */
    float3 scale;
    scale.x = len(make_float3(tfm.x.x, tfm.y.x, tfm.z.x));
    scale.y = len(make_float3(tfm.x.y, tfm.y.y, tfm.z.y));
    scale.z = len(make_float3(tfm.x.z, tfm.y.z, tfm.z.z));

    /* Build rotation-only matrix by dividing each column by its scale. */
    Transform rot_tfm = tfm;
    if (scale.x > 0.0f) {
      rot_tfm.x.x /= scale.x;
      rot_tfm.y.x /= scale.x;
      rot_tfm.z.x /= scale.x;
    }
    if (scale.y > 0.0f) {
      rot_tfm.x.y /= scale.y;
      rot_tfm.y.y /= scale.y;
      rot_tfm.z.y /= scale.y;
    }
    if (scale.z > 0.0f) {
      rot_tfm.x.z /= scale.z;
      rot_tfm.y.z /= scale.z;
      rot_tfm.z.z /= scale.z;
    }
    /* Zero translation for the rotation matrix. */
    rot_tfm.x.w = 0.0f;
    rot_tfm.y.w = 0.0f;
    rot_tfm.z.w = 0.0f;

    Transform inv_rot = transform_inverse(rot_tfm);

    /* Read SDF data via RNA. */
    SDFObjectData obj;
    obj.itfm = inv_rot;
    obj.position = make_float3(tfm.x.w, tfm.y.w, tfm.z.w);

    PointerRNA sdf_ptr = b_data.ptr;
    float size_arr[3];
    RNA_float_get_array(&sdf_ptr, "size", size_arr);
    obj.sdf_size = make_float3(size_arr[0] * scale.x,
                               size_arr[1] * scale.y,
                               size_arr[2] * scale.z);
    obj.bevel = RNA_float_get(&sdf_ptr, "bevel") * max(max(scale.x, scale.y), scale.z);
    obj.blend = RNA_float_get(&sdf_ptr, "blend");
    obj.sdf_type = RNA_enum_get(&sdf_ptr, "sdf_type");

    float color_arr[4];
    RNA_float_get_array(&sdf_ptr, "color", color_arr);
    obj.color = make_float3(color_arr[0], color_arr[1], color_arr[2]);

    /* Compute world AABB. */
    float3 half = obj.sdf_size + make_float3(obj.bevel, obj.bevel, obj.bevel);
    float3 corners[8];
    for (int c = 0; c < 8; c++) {
      float3 lc = make_float3(
          (c & 1) ? half.x : -half.x,
          (c & 2) ? half.y : -half.y,
          (c & 4) ? half.z : -half.z);
      corners[c] = obj.position + transform_direction(&rot_tfm, lc);
    }

    obj.bbox_min = corners[0];
    obj.bbox_max = corners[0];
    for (int c = 1; c < 8; c++) {
      obj.bbox_min = min(obj.bbox_min, corners[c]);
      obj.bbox_max = max(obj.bbox_max, corners[c]);
    }

    scene_min_acc = min(scene_min_acc, obj.bbox_min);
    scene_max_acc = max(scene_max_acc, obj.bbox_max);

    /* Resolve per-object shader from material slot. */
    obj.shader_index = 0;
    {
      array<Node *> used = find_used_shaders(b_ob);
      obj.shader = used.size() > 0 ? static_cast<Shader *>(used[0]) : scene->default_surface;
    }

    sdf_objects.push_back(obj);
  }

  if (sdf_objects.empty()) {
    sdf_geom->clear();
    sdf_geom->prev_scene_hash = 0;
    return;
  }

  /* ---- Register per-object shaders on the SDFGeometry ---- */
  {
    array<Node *> unique_shaders;
    for (size_t i = 0; i < sdf_objects.size(); i++) {
      Shader *s = sdf_objects[i].shader;
      bool found = false;
      for (size_t j = 0; j < unique_shaders.size(); j++) {
        if (unique_shaders[j] == s) {
          found = true;
          break;
        }
      }
      if (!found) {
        unique_shaders.push_back_slow(s);
      }
    }
    sdf_geom->set_used_shaders(unique_shaders);
  }

  /* ---- Scene hash for change detection ---- */
  {
    uint64_t h = uint64_t(sdf_objects.size()) * 997;
    for (const SDFObjectData &obj : sdf_objects) {
      const auto mix = [&](float v) {
        h = h * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&v);
      };
      mix(obj.position.x);
      mix(obj.position.y);
      mix(obj.position.z);
      mix(obj.sdf_size.x);
      mix(obj.sdf_size.y);
      mix(obj.sdf_size.z);
      mix(obj.bevel);
      mix(obj.blend);
      mix(obj.itfm.x.x);
      mix(obj.itfm.x.y);
      mix(obj.itfm.x.z);
      mix(obj.itfm.y.x);
      mix(obj.itfm.y.y);
      mix(obj.itfm.y.z);
      mix(obj.itfm.z.x);
      mix(obj.itfm.z.y);
      mix(obj.itfm.z.z);
      mix(obj.color.x);
      mix(obj.color.y);
      mix(obj.color.z);
      h = h * 6364136223846793005ULL + uint64_t(obj.sdf_type);
      /* Include shader pointer so material changes invalidate the hash. */
      h = h * 6364136223846793005ULL + uint64_t(uintptr_t(obj.shader));
    }
    const bool has_atlas_data = !sdf_geom->shapes.empty() ||
                                !sdf_geom->indirection_data.empty();
    if (h == sdf_geom->prev_scene_hash && has_atlas_data) {
      /* Nothing changed: skip expensive bake, keep existing atlas data.
       * Refresh shader pointers and IDs so device upload resolves correctly.
       * (IDs cached here may still be stale — device upload re-resolves
       * from the Shader* pointers after shader_manager assigns final IDs.) */
      sdf_geom->object_shader_ids.resize(sdf_objects.size());
      sdf_geom->object_shaders.resize(sdf_objects.size());
      for (size_t i = 0; i < sdf_objects.size(); i++) {
        sdf_geom->object_shaders[i] = sdf_objects[i].shader;
        sdf_geom->object_shader_ids[i] = scene->shader_manager->get_shader_id(
            sdf_objects[i].shader);
      }
      /* Also refresh instanced mode shader pointers/IDs. */
      for (size_t ii = 0; ii < sdf_geom->instances.size(); ii++) {
        int obj_idx = sdf_geom->instances[ii].object_id;
        if (obj_idx >= 0 && obj_idx < (int)sdf_objects.size()) {
          sdf_geom->instances[ii].shader = sdf_objects[obj_idx].shader;
          sdf_geom->instances[ii].shader_id = scene->shader_manager->get_shader_id(
              sdf_objects[obj_idx].shader);
        }
      }
      /* Tag geometry so the device re-uploads the shader map. */
      sdf_geom->tag_update(scene, true);
      return;
    }
    sdf_geom->prev_scene_hash = h;
  }

  /* ---- Build shape/instance tables ---- */
  sdf_geom->shapes.clear();
  sdf_geom->instances.clear();
  unordered_map<uint64_t, int> fingerprint_to_shape;

  bool any_blend = false;
  for (size_t i = 0; i < sdf_objects.size(); i++) {
    const SDFObjectData &obj = sdf_objects[i];
    uint64_t fp = sdf_shape_fingerprint(obj.sdf_type, obj.sdf_size, obj.bevel);

    int shape_id;
    auto it = fingerprint_to_shape.find(fp);
    if (it != fingerprint_to_shape.end()) {
      shape_id = it->second;
    }
    else {
      float max_dim = max(max(fabsf(obj.sdf_size.x), fabsf(obj.sdf_size.y)),
                          fabsf(obj.sdf_size.z));
      if (max_dim < 1e-8f) {
        max_dim = 1.0f;
      }

      SDFGeometry::ShapeInfo shape;
      shape.fingerprint = fp;
      shape.sdf_type = obj.sdf_type;
      shape.size_normalized = obj.sdf_size / max_dim;
      shape.bevel_normalized = obj.bevel / max_dim;
      shape.world_scale = max_dim;
      shape.atlas_index = -1;
      shape.grid_res = make_int3(0, 0, 0);
      shape.voxel_size = 0.0f;
      shape.origin = zero_float3();
      shape.bricks_per_axis = 0;
      shape.active_bricks = 0;
      shape.indirection_offset = 0;
      shape.atlas_offset = 0;
      shape.brick_map_offset = 0;

      shape_id = (int)sdf_geom->shapes.size();
      sdf_geom->shapes.push_back(shape);
      fingerprint_to_shape[fp] = shape_id;
    }

    SDFGeometry::InstanceInfo inst;
    inst.shape_id = shape_id;
    inst.object_id = (int)i;
    inst.shader = obj.shader;
    inst.shader_id = scene->shader_manager->get_shader_id(obj.shader);
    inst.color = make_float4(obj.color.x, obj.color.y, obj.color.z, 1.0f);
    inst.blend = obj.blend;
    float max_dim = max(max(fabsf(obj.sdf_size.x), fabsf(obj.sdf_size.y)),
                        fabsf(obj.sdf_size.z));
    inst.world_scale = max(max_dim, 1e-8f);

    /* Compute full local_to_world: maps normalized local space to world.
     * Local space has the SDF centered at origin with normalized size.
     * local_to_world = translate(position) * rot * scale(world_scale). */
    {
      float ws = inst.world_scale;
      Transform rot = transform_inverse(obj.itfm);
      Transform scale_tfm = transform_scale(ws, ws, ws);
      Transform translate = transform_translate(obj.position);

      inst.local_to_world = translate * rot * scale_tfm;
      inst.world_to_local = transform_inverse(inst.local_to_world);
    }

    if (obj.blend > 0.0f) {
      any_blend = true;
    }

    sdf_geom->instances.push_back(inst);
  }

  /* Decide mode: instanced when no objects use smooth blending. */
  sdf_geom->use_instanced = !any_blend && sdf_geom->shapes.size() > 0;

  /* ---- Per-shape instanced baking (TLAS/BLAS path) ---- */
  if (sdf_geom->use_instanced) {
    sdf_geom->shape_indirection_data.clear();
    sdf_geom->shape_atlas_data.clear();
    sdf_geom->shape_brick_map_data.clear();

    constexpr int MAX_SHAPE_GRID_RES = 32;
    const float base_voxel = 1.0f / 256.0f; /* Match world-space density: 256 voxels/unit. */

    int global_indir_offset = 0;
    int global_atlas_offset = 0;
    int global_brick_map_offset = 0;

    for (size_t si = 0; si < sdf_geom->shapes.size(); si++) {
      SDFGeometry::ShapeInfo &shape = sdf_geom->shapes[si];

      /* Local voxel size based on world_scale. */
      float local_vs = base_voxel / max(shape.world_scale, 1e-6f);

      /* Local extent per axis: size_normalized + bevel + margin. */
      float brick_half_diag = float(BRICK_SIZE) * local_vs * 0.866025f;
      float margin = brick_half_diag + 2.0f * local_vs;
      float3 half_extent = shape.size_normalized +
                           make_float3(shape.bevel_normalized + margin,
                                       shape.bevel_normalized + margin,
                                       shape.bevel_normalized + margin);

      /* Grid resolution per axis (non-cubic, capped). */
      float chunk = float(BRICK_SIZE) * local_vs;
      int3 gr;
      gr.x = clamp((int)ceilf(2.0f * half_extent.x / chunk), 1, MAX_SHAPE_GRID_RES);
      gr.y = clamp((int)ceilf(2.0f * half_extent.y / chunk), 1, MAX_SHAPE_GRID_RES);
      gr.z = clamp((int)ceilf(2.0f * half_extent.z / chunk), 1, MAX_SHAPE_GRID_RES);

      /* When grid resolution is capped, increase voxel size so the grid still
       * covers the full SDF extent. Without this, the surface can lie entirely
       * outside the grid, producing zero active bricks and invisible objects. */
      {
        float needed_vs = max(2.0f * half_extent.x / float(gr.x * BRICK_SIZE),
                              max(2.0f * half_extent.y / float(gr.y * BRICK_SIZE),
                                  2.0f * half_extent.z / float(gr.z * BRICK_SIZE)));
        if (needed_vs > local_vs) {
          local_vs = needed_vs;
          brick_half_diag = float(BRICK_SIZE) * local_vs * 0.866025f;
        }
      }

      float3 local_origin = make_float3(-float(gr.x), -float(gr.y), -float(gr.z)) *
                             float(BRICK_SIZE) * local_vs * 0.5f;

      /* Classify bricks: evaluate sdBox at each brick center. */
      int grid_volume = gr.x * gr.y * gr.z;
      vector<int> shape_indir(grid_volume, -1);
      vector<int4> shape_brick_entries;

      float3 box_size = shape.size_normalized -
                         make_float3(shape.bevel_normalized,
                                     shape.bevel_normalized,
                                     shape.bevel_normalized);
      box_size = max(box_size, make_float3(0.001f, 0.001f, 0.001f));

      int active_count = 0;
      for (int bz = 0; bz < gr.z; bz++) {
        for (int by = 0; by < gr.y; by++) {
          for (int bx = 0; bx < gr.x; bx++) {
            float3 brick_center = local_origin +
                                  make_float3(float(bx) * BRICK_SIZE + BRICK_SIZE * 0.5f,
                                              float(by) * BRICK_SIZE + BRICK_SIZE * 0.5f,
                                              float(bz) * BRICK_SIZE + BRICK_SIZE * 0.5f) *
                                      local_vs;

            float dist = sdf_box(brick_center, box_size) - shape.bevel_normalized;

            int flat_idx = bx + by * gr.x + bz * gr.x * gr.y;
            if (fabsf(dist) < brick_half_diag * 1.5f) {
              shape_indir[flat_idx] = active_count;
              int brick_linear = bx + by * gr.x + bz * gr.x * gr.y;
              shape_brick_entries.push_back(
                  make_int4(brick_linear, active_count, (int)si, 0));
              active_count++;
            }
            else if (dist < 0.0f) {
              shape_indir[flat_idx] = -2; /* Fully inside. */
              int brick_linear = bx + by * gr.x + bz * gr.x * gr.y;
              shape_brick_entries.push_back(make_int4(brick_linear, -2, (int)si, 0));
            }
          }
        }
      }

      if (active_count == 0) {
        shape.grid_res = gr;
        shape.voxel_size = local_vs;
        shape.origin = local_origin;
        shape.bricks_per_axis = 1;
        shape.active_bricks = 0;
        shape.indirection_offset = global_indir_offset;
        shape.atlas_offset = global_atlas_offset;
        shape.brick_map_offset = global_brick_map_offset;

        /* Still append indirection data. */
        sdf_geom->shape_indirection_data.insert(
            sdf_geom->shape_indirection_data.end(),
            shape_indir.begin(), shape_indir.end());
        global_indir_offset += grid_volume;
        continue;
      }

      int bpa = max((int)ceilf(cbrtf((float)active_count)), 1);
      int atlas_dim = bpa * BRICK_STORAGE;
      size_t atlas_total = (size_t)atlas_dim * atlas_dim * atlas_dim;

      /* Bake per-shape local atlas (uint16 encoded distances). */
      const uint16_t dist16_max = sdf_encode_dist16(100.0f * local_vs, local_vs);
      vector<uint16_t> shape_atlas(atlas_total, dist16_max);

      parallel_for(0, grid_volume, [&](int idx) {
        int slot = shape_indir[idx];
        if (slot < 0) {
          return;
        }

        int bx = idx % gr.x;
        int by = (idx / gr.x) % gr.y;
        int bz = idx / (gr.x * gr.y);

        int sx = slot % bpa;
        int sy = (slot / bpa) % bpa;
        int sz = slot / (bpa * bpa);
        int3 slot_origin = make_int3(sx * BRICK_STORAGE, sy * BRICK_STORAGE, sz * BRICK_STORAGE);

        for (int lz = 0; lz < BRICK_STORAGE; lz++) {
          for (int ly = 0; ly < BRICK_STORAGE; ly++) {
            for (int lx = 0; lx < BRICK_STORAGE; lx++) {
              float3 local_pos = local_origin +
                                 make_float3(float(bx * BRICK_SIZE + lx - BRICK_BORDER) + 0.5f,
                                             float(by * BRICK_SIZE + ly - BRICK_BORDER) + 0.5f,
                                             float(bz * BRICK_SIZE + lz - BRICK_BORDER) + 0.5f) *
                                     local_vs;

              float d = sdf_box(local_pos, box_size) - shape.bevel_normalized;

              int3 ac = slot_origin + make_int3(lx, ly, lz);
              size_t ai = (size_t)ac.z * atlas_dim * atlas_dim +
                          (size_t)ac.y * atlas_dim + (size_t)ac.x;
              shape_atlas[ai] = sdf_encode_dist16(d, local_vs);
            }
          }
        }
      });

      /* Store shape parameters. */
      shape.grid_res = gr;
      shape.voxel_size = local_vs;
      shape.origin = local_origin;
      shape.bricks_per_axis = bpa;
      shape.active_bricks = active_count;
      shape.indirection_offset = global_indir_offset;
      shape.atlas_offset = global_atlas_offset;
      shape.brick_map_offset = global_brick_map_offset;

      /* Append to concatenated arrays. */
      sdf_geom->shape_indirection_data.insert(
          sdf_geom->shape_indirection_data.end(),
          shape_indir.begin(), shape_indir.end());

      sdf_geom->shape_atlas_data.insert(
          sdf_geom->shape_atlas_data.end(),
          shape_atlas.begin(), shape_atlas.end());

      sdf_geom->shape_brick_map_data.insert(
          sdf_geom->shape_brick_map_data.end(),
          shape_brick_entries.begin(), shape_brick_entries.end());

      global_indir_offset += grid_volume;
      global_atlas_offset += (int)atlas_total;
      global_brick_map_offset += (int)shape_brick_entries.size();
    }

    /* For instanced mode, we still need minimal world-space data for fallback.
     * Set grid parameters but skip the expensive world-space bake. */
    sdf_geom->origin = scene_min_acc;
    sdf_geom->voxel_size = 0.0f;
    sdf_geom->grid_res = 0;
    sdf_geom->bricks_per_axis = 0;
    sdf_geom->num_objects = (int)sdf_objects.size();
    sdf_geom->scene_min = scene_min_acc;
    sdf_geom->scene_max = scene_max_acc;
    sdf_geom->indirection_data.clear();
    sdf_geom->atlas_data.clear();
    sdf_geom->matid_data.clear();

    sdf_geom->object_shader_ids.resize(sdf_objects.size());
    sdf_geom->object_shaders.resize(sdf_objects.size());
    for (size_t i = 0; i < sdf_objects.size(); i++) {
      sdf_geom->object_shaders[i] = sdf_objects[i].shader;
      sdf_geom->object_shader_ids[i] = scene->shader_manager->get_shader_id(
          sdf_objects[i].shader);
    }

    sdf_geom->compute_bounds();
    sdf_geom->tag_update(scene, true);
    return;
  }

  /* ---- World-space baking (original path, used when blend > 0) ---- */

  /* Compute atlas parameters. */
  const int grid_res = 32; /* Bricks per axis (32 * 8 = 256 voxels per axis). */
  const int total_res = grid_res * BRICK_SIZE;

  float3 scene_center = (scene_min_acc + scene_max_acc) * 0.5f;
  float3 scene_size = scene_max_acc - scene_min_acc;
  float margin = max(max(scene_size.x, max(scene_size.y, scene_size.z)) * 0.1f, 0.5f);
  scene_size = scene_size + make_float3(margin * 2.0f, margin * 2.0f, margin * 2.0f);

  float max_axis = max(scene_size.x, max(scene_size.y, scene_size.z));
  float voxel_size = max_axis / float(total_res);
  float3 atlas_origin = scene_center - make_float3(max_axis * 0.5f, max_axis * 0.5f, max_axis * 0.5f);

  /* ---- Phase 1: Classify bricks (with per-brick AABB culling) ---- */
  const int num_bricks = grid_res * grid_res * grid_res;
  vector<int> indirection(num_bricks, -1);
  int active_count = 0;

  const float brick_world = float(BRICK_SIZE) * voxel_size;
  const float brick_half_diag = brick_world * 0.866025f; /* sqrt(3)/2 */

  /* Compute max blend radius for AABB expansion. */
  float max_blend = 0.0f;
  for (size_t i = 0; i < sdf_objects.size(); i++) {
    max_blend = max(max_blend, sdf_objects[i].blend);
  }

  for (int bz = 0; bz < grid_res; bz++) {
    for (int by = 0; by < grid_res; by++) {
      for (int bx = 0; bx < grid_res; bx++) {
        /* Brick center in world space. */
        float3 brick_center = atlas_origin +
                              make_float3(float(bx) * BRICK_SIZE + BRICK_SIZE * 0.5f,
                                          float(by) * BRICK_SIZE + BRICK_SIZE * 0.5f,
                                          float(bz) * BRICK_SIZE + BRICK_SIZE * 0.5f) *
                                  voxel_size;

        /* Expanded brick AABB for object culling: half-diagonal + blend radius. */
        float expand = brick_half_diag + max_blend;
        float3 brick_min = brick_center - make_float3(expand, expand, expand);
        float3 brick_max = brick_center + make_float3(expand, expand, expand);

        /* Evaluate SDF at brick center (only overlapping objects). */
        float acc_dist = 1e10f;
        for (size_t i = 0; i < sdf_objects.size(); i++) {
          /* AABB culling: skip objects that can't contribute to this brick. */
          if (sdf_objects[i].bbox_min.x > brick_max.x ||
              sdf_objects[i].bbox_min.y > brick_max.y ||
              sdf_objects[i].bbox_min.z > brick_max.z ||
              sdf_objects[i].bbox_max.x < brick_min.x ||
              sdf_objects[i].bbox_max.y < brick_min.y ||
              sdf_objects[i].bbox_max.z < brick_min.z)
          {
            continue;
          }

          float dist = evaluate_sdf_object(sdf_objects[i], brick_center);
          float k = sdf_objects[i].blend;
          if (k > 0.0f && acc_dist < 1e9f) {
            acc_dist = op_smooth_union(acc_dist, dist, k);
          }
          else {
            acc_dist = min(acc_dist, dist);
          }
        }

        int idx = bz * grid_res * grid_res + by * grid_res + bx;
        if (fabsf(acc_dist) < brick_half_diag) {
          indirection[idx] = active_count++;
        }
        else if (acc_dist < 0.0f) {
          indirection[idx] = -2; /* Fully inside. */
        }
        /* else -1: fully outside (default). */
      }
    }
  }

  if (active_count == 0) {
    /* No surface bricks found — SDF is either fully inside or outside. */
    sdf_geom->clear();
    return;
  }

  /* Compact atlas layout. */
  int bricks_per_axis = (int)ceilf(cbrtf((float)active_count));
  bricks_per_axis = max(bricks_per_axis, 1);
  int atlas_dim = bricks_per_axis * BRICK_STORAGE;
  size_t atlas_total = (size_t)atlas_dim * atlas_dim * atlas_dim;

  /* ---- Phase 2: Bake atlas (multithreaded) ---- */
  vector<float4> atlas(atlas_total, make_float4(100.0f, 0.0f, 0.0f, 0.0f));
  vector<int> matid(atlas_total, -1);
  vector<int> blend_id(atlas_total, -1);
  vector<float> blend_fac(atlas_total, 0.0f);

  /* Each brick writes to its own atlas region, so bricks are independent.
   * Parallelize over the flat brick index for good load balance. */
  float4 *atlas_data = atlas.data();
  int *matid_data = matid.data();
  int *blend_id_data = blend_id.data();
  float *blend_fac_data = blend_fac.data();

  parallel_for(0, num_bricks, [&](int idx) {
    int slot = indirection[idx];
    if (slot < 0) {
      return;
    }

    int bx = idx % grid_res;
    int by = (idx / grid_res) % grid_res;
    int bz = idx / (grid_res * grid_res);

    int sx = slot % bricks_per_axis;
    int sy = (slot / bricks_per_axis) % bricks_per_axis;
    int sz = slot / (bricks_per_axis * bricks_per_axis);
    int3 slot_origin = make_int3(sx * BRICK_STORAGE, sy * BRICK_STORAGE, sz * BRICK_STORAGE);

    /* Per-brick AABB including 2-voxel border + blend expansion.
     * Pre-filter candidates once per brick instead of evaluating all objects
     * for every voxel (mirrors the GPU bake shader's BVH culling). */
    float3 brick_min_w = atlas_origin +
                         make_float3(float(bx * BRICK_SIZE - BRICK_BORDER),
                                     float(by * BRICK_SIZE - BRICK_BORDER),
                                     float(bz * BRICK_SIZE - BRICK_BORDER)) *
                             voxel_size -
                         make_float3(max_blend, max_blend, max_blend);
    float3 brick_max_w = atlas_origin +
                         make_float3(float(bx * BRICK_SIZE + BRICK_SIZE + BRICK_BORDER),
                                     float(by * BRICK_SIZE + BRICK_SIZE + BRICK_BORDER),
                                     float(bz * BRICK_SIZE + BRICK_SIZE + BRICK_BORDER)) *
                             voxel_size +
                         make_float3(max_blend, max_blend, max_blend);

    /* Build per-brick candidate list. */
    vector<int> candidates;
    candidates.reserve(sdf_objects.size());
    for (size_t i = 0; i < sdf_objects.size(); i++) {
      if (sdf_objects[i].bbox_min.x > brick_max_w.x ||
          sdf_objects[i].bbox_min.y > brick_max_w.y ||
          sdf_objects[i].bbox_min.z > brick_max_w.z ||
          sdf_objects[i].bbox_max.x < brick_min_w.x ||
          sdf_objects[i].bbox_max.y < brick_min_w.y ||
          sdf_objects[i].bbox_max.z < brick_min_w.z)
      {
        continue;
      }
      candidates.push_back((int)i);
    }

    for (int lz = 0; lz < BRICK_STORAGE; lz++) {
      for (int ly = 0; ly < BRICK_STORAGE; ly++) {
        for (int lx = 0; lx < BRICK_STORAGE; lx++) {
          float3 world_pos = atlas_origin +
                             make_float3(float(bx * BRICK_SIZE + lx - BRICK_BORDER) + 0.5f,
                                         float(by * BRICK_SIZE + ly - BRICK_BORDER) + 0.5f,
                                         float(bz * BRICK_SIZE + lz - BRICK_BORDER) + 0.5f) *
                                 voxel_size;

          float acc_dist = 1e10f;
          float3 acc_color = zero_float3();

          /* Track two closest objects for material blending. */
          int obj_a = -1, obj_b = -1;
          float dist_a = 1e10f, dist_b = 1e10f;

          for (size_t c = 0; c < candidates.size(); c++) {
            int i = candidates[c];
            float dist = evaluate_sdf_object(sdf_objects[i], world_pos);
            float k = sdf_objects[i].blend;

            if (k > 0.0f && acc_dist < 1e9f) {
              float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
              acc_color = acc_color * (1.0f - h) + sdf_objects[i].color * h;
              acc_dist = acc_dist * (1.0f - h) + dist * h - k * h * (1.0f - h);
            }
            else {
              if (dist < acc_dist) {
                acc_color = sdf_objects[i].color;
                acc_dist = dist;
              }
            }

            /* Maintain two closest objects by raw distance. */
            if (dist < dist_a) {
              dist_b = dist_a;
              obj_b = obj_a;
              dist_a = dist;
              obj_a = i;
            }
            else if (dist < dist_b) {
              dist_b = dist;
              obj_b = i;
            }
          }

          /* Compute blend factor between the two closest objects.
           * Uses smooth union weight: obj_b contributes when within blend radius. */
          float mat_blend_fac = 0.0f;
          if (obj_a >= 0 && obj_b >= 0) {
            float k = max(sdf_objects[obj_a].blend, sdf_objects[obj_b].blend);
            if (k > 0.0f) {
              float h = clamp(0.5f + 0.5f * (dist_b - dist_a) / k, 0.0f, 1.0f);
              mat_blend_fac = 1.0f - h; /* Weight of secondary object (obj_b). */
            }
          }

          int3 atlas_coord = slot_origin + make_int3(lx, ly, lz);
          size_t atlas_idx = (size_t)atlas_coord.z * atlas_dim * atlas_dim +
                             (size_t)atlas_coord.y * atlas_dim +
                             (size_t)atlas_coord.x;

          atlas_data[atlas_idx] = make_float4(
              acc_dist, acc_color.x, acc_color.y, acc_color.z);
          matid_data[atlas_idx] = max(obj_a, 0);
          blend_id_data[atlas_idx] = obj_b;
          blend_fac_data[atlas_idx] = mat_blend_fac;
        }
      }
    }
  });

  /* ---- Store results in SDFGeometry ---- */
  sdf_geom->origin = atlas_origin;
  sdf_geom->voxel_size = voxel_size;
  sdf_geom->grid_res = grid_res;
  sdf_geom->bricks_per_axis = bricks_per_axis;
  sdf_geom->num_objects = (int)sdf_objects.size();
  sdf_geom->scene_min = scene_min_acc;
  sdf_geom->scene_max = scene_max_acc;

  sdf_geom->indirection_data = std::move(indirection);
  sdf_geom->atlas_data = std::move(atlas);
  sdf_geom->matid_data = std::move(matid);
  sdf_geom->blend_id_data = std::move(blend_id);
  sdf_geom->blend_factor_data = std::move(blend_fac);

  /* Per-object shader mapping: resolve each object's material to a shader.
   * Store both shader IDs (for early-return refresh) and Shader pointers
   * (for re-resolving at device upload, since IDs may be stale). */
  sdf_geom->object_shader_ids.resize(sdf_objects.size());
  sdf_geom->object_shaders.resize(sdf_objects.size());
  for (size_t i = 0; i < sdf_objects.size(); i++) {
    sdf_geom->object_shaders[i] = sdf_objects[i].shader;
    sdf_geom->object_shader_ids[i] = scene->shader_manager->get_shader_id(sdf_objects[i].shader);
  }

  sdf_geom->compute_bounds();
  sdf_geom->tag_update(scene, true);
}

CCL_NAMESPACE_END
