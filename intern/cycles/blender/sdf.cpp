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

    /* Decompose: extract scale, build rotation-only inverse. */
    float3 scale;
    scale.x = len(make_float3(tfm.x.x, tfm.x.y, tfm.x.z));
    scale.y = len(make_float3(tfm.y.x, tfm.y.y, tfm.y.z));
    scale.z = len(make_float3(tfm.z.x, tfm.z.y, tfm.z.z));

    /* Build rotation-only matrix. */
    Transform rot_tfm = tfm;
    if (scale.x > 0.0f) {
      rot_tfm.x.x /= scale.x;
      rot_tfm.x.y /= scale.x;
      rot_tfm.x.z /= scale.x;
    }
    if (scale.y > 0.0f) {
      rot_tfm.y.x /= scale.y;
      rot_tfm.y.y /= scale.y;
      rot_tfm.y.z /= scale.y;
    }
    if (scale.z > 0.0f) {
      rot_tfm.z.x /= scale.z;
      rot_tfm.z.y /= scale.z;
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

    /* Shader index will be resolved later. For now store 0. */
    obj.shader_index = 0;

    sdf_objects.push_back(obj);
  }

  if (sdf_objects.empty()) {
    sdf_geom->clear();
    return;
  }

  /* ---- Build shape/instance tables ---- */
  sdf_geom->shapes.clear();
  sdf_geom->instances.clear();
  unordered_map<uint64_t, int> fingerprint_to_shape;

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

      shape_id = (int)sdf_geom->shapes.size();
      sdf_geom->shapes.push_back(shape);
      fingerprint_to_shape[fp] = shape_id;
    }

    SDFGeometry::InstanceInfo inst;
    inst.shape_id = shape_id;
    inst.object_id = (int)i;
    /* Full transform for world_to_local / local_to_world will be needed in Step 4.
     * For now, store the rotation-only inverse and position for reference. */
    inst.local_to_world = transform_identity();
    inst.world_to_local = transform_identity();
    inst.color = make_float4(obj.color.x, obj.color.y, obj.color.z, 1.0f);
    inst.blend = obj.blend;
    float max_dim = max(max(fabsf(obj.sdf_size.x), fabsf(obj.sdf_size.y)),
                        fabsf(obj.sdf_size.z));
    inst.world_scale = max(max_dim, 1e-8f);

    sdf_geom->instances.push_back(inst);
  }

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

  /* Each brick writes to its own atlas region, so bricks are independent.
   * Parallelize over the flat brick index for good load balance. */
  float4 *atlas_data = atlas.data();
  int *matid_data = matid.data();

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
          int closest_obj = 0;
          float closest_raw = 1e10f;

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

            if (dist < closest_raw) {
              closest_raw = dist;
              closest_obj = i;
            }
          }

          int3 atlas_coord = slot_origin + make_int3(lx, ly, lz);
          size_t atlas_idx = (size_t)atlas_coord.z * atlas_dim * atlas_dim +
                             (size_t)atlas_coord.y * atlas_dim +
                             (size_t)atlas_coord.x;

          atlas_data[atlas_idx] = make_float4(
              acc_dist, acc_color.x, acc_color.y, acc_color.z);
          matid_data[atlas_idx] = closest_obj;
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

  /* Per-object shader mapping (will be resolved during device_update). */
  sdf_geom->object_shader_ids.resize(sdf_objects.size());
  for (size_t i = 0; i < sdf_objects.size(); i++) {
    sdf_geom->object_shader_ids[i] = 0; /* Default shader. */
  }

  sdf_geom->compute_bounds();
  sdf_geom->tag_update(scene, true);
}

CCL_NAMESPACE_END
