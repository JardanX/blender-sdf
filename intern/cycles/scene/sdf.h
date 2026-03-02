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
