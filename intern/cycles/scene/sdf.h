/* SPDX-FileCopyrightText: 2025 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "scene/geometry.h"
#include "scene/image.h"

CCL_NAMESPACE_BEGIN

/* SDFGeometry
 *
 * Geometry type for SDF (Signed Distance Field) objects. All SDF objects
 * in the scene are baked into a single shared sparse brick atlas, similar
 * to the draw engine's approach. The atlas is uploaded to Cycles as 3D
 * textures via ImageManager. */

class SDFGeometry : public Geometry {
 public:
  NODE_DECLARE

  /* Grid parameters (set during sync). */
  float3 origin;       /* World-space atlas origin. */
  float voxel_size;    /* World-space voxel size. */
  int grid_res;        /* Bricks per axis in the indirection grid. */
  int bricks_per_axis; /* ceil(cbrt(active_bricks)) for compact atlas layout. */
  int num_objects;     /* Number of SDF Blender objects contributing to atlas. */

  /* Atlas textures (via ImageManager). */
  ImageHandle indirection_handle; /* R32I: brick -> slot mapping. */
  ImageHandle atlas_handle;       /* RGBA16F: distance + color. */
  ImageHandle matid_handle;       /* R16I: per-voxel closest object index. */

  /* Baked data (CPU-side, uploaded to device as flat arrays). */
  vector<int> indirection_data;   /* grid_res^3 ints. */
  vector<float4> atlas_data;      /* Compact atlas voxels (dist, r, g, b). */
  vector<int> matid_data;         /* Compact atlas material IDs. */

  /* Per-object shader mapping: [blender_obj_index] -> cycles_shader_id. */
  vector<int> object_shader_ids;

  /* Scene bounding box. */
  float3 scene_min;
  float3 scene_max;

  /* Set by GeometryManager::device_update to mark which SDFGeometry instance
   * was actually uploaded to the device. device_impl.cpp must use this same
   * instance for building BLAS AABBs so the brick_map stays in sync. */
  bool is_active_atlas = false;

  /* Shape/instance tables for instanced rendering.
   * Each unique SDF shape (identified by fingerprint) can be shared across
   * multiple instances, reducing atlas memory and enabling TLAS/BLAS. */
  struct ShapeInfo {
    uint64_t fingerprint;
    int sdf_type;
    float3 size_normalized;   /* Aspect ratio (max = 1.0). */
    float bevel_normalized;   /* bevel / max_size. */
    float world_scale;        /* max(effective_size) for representative instance. */
    int atlas_index;          /* Per-shape atlas slot (-1 = not yet baked). */

    /* Per-shape local atlas parameters (set by per-shape baking). */
    int3 grid_res;            /* Bricks per axis (non-cubic, capped at 32). */
    float voxel_size;         /* Local-space voxel size. */
    float3 origin;            /* Local-space atlas origin. */
    int bricks_per_axis;      /* ceil(cbrt(active_bricks)) for compact layout. */
    int active_bricks;        /* Number of active (surface) bricks. */
    int indirection_offset;   /* Offset into shape_indirection_data. */
    int atlas_offset;         /* Offset into shape_atlas_data. */
    int brick_map_offset;     /* Offset into shape_brick_map_data. */
  };
  vector<ShapeInfo> shapes;

  struct InstanceInfo {
    int shape_id;             /* Index into shapes[]. */
    int object_id;            /* Blender object index in the SDF scene. */
    Transform world_to_local;
    Transform local_to_world;
    float4 color;
    float blend;
    float world_scale;
  };
  vector<InstanceInfo> instances;

  /* Per-shape instanced mode (TLAS/BLAS). Active when use_instanced = true. */
  bool use_instanced = false;
  vector<int> shape_indirection_data;    /* Concatenated per-shape indirection. */
  vector<float> shape_atlas_data;        /* Concatenated per-shape atlas (distance only). */
  vector<int4> shape_brick_map_data;     /* Concatenated per-shape brick maps. */

  SDFGeometry();
  ~SDFGeometry() override;

  void clear(bool preserve_shaders = false) override;
  void compute_bounds() override;
  void apply_transform(const Transform &tfm, const bool apply_to_motion) override;
  void get_uv_tiles(ustring map, unordered_set<int> &tiles) override;

  PrimitiveType primitive_type() const override
  {
    return PRIMITIVE_SDF;
  }

 private:
  friend class GeometryManager;
};

CCL_NAMESPACE_END
