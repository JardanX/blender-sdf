/* SPDX-FileCopyrightText: 2025 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "scene/sdf.h"
#include "scene/scene.h"

CCL_NAMESPACE_BEGIN

/* SDFGeometry */

NODE_DEFINE(SDFGeometry)
{
  NodeType *type = NodeType::add(
      "sdf_geometry", create, NodeType::NONE, Geometry::get_node_base_type());

  return type;
}

SDFGeometry::SDFGeometry() : Geometry(get_node_type(), Geometry::SDF)
{
  origin = zero_float3();
  voxel_size = 1.0f / 256.0f;
  grid_res = 32;
  bricks_per_axis = 1;
  num_objects = 0;
  scene_min = make_float3(1e30f, 1e30f, 1e30f);
  scene_max = make_float3(-1e30f, -1e30f, -1e30f);
}

SDFGeometry::~SDFGeometry() = default;

void SDFGeometry::clear(bool preserve_shaders)
{
  Geometry::clear(preserve_shaders);

  indirection_handle = ImageHandle();
  atlas_handle = ImageHandle();
  matid_handle = ImageHandle();

  indirection_data.clear();
  atlas_data.clear();
  matid_data.clear();
  object_shader_ids.clear();
}

void SDFGeometry::compute_bounds()
{
  /* Bounds computed from atlas origin + total voxel extent. */
  BoundBox bnds = BoundBox::empty;

  if (grid_res > 0 && voxel_size > 0.0f) {
    const float extent = float(grid_res * 8) * voxel_size;
    bnds.grow(origin);
    bnds.grow(origin + make_float3(extent, extent, extent));
  }

  if (!bnds.valid()) {
    bnds = BoundBox::empty;
  }

  bounds = bnds;
}

void SDFGeometry::apply_transform(const Transform & /*tfm*/, const bool /*apply_to_motion*/)
{
  /* SDF is baked in world space, no transform to apply. */
}

void SDFGeometry::get_uv_tiles(ustring /*map*/, unordered_set<int> & /*tiles*/)
{
  /* SDF has no UV tiles. */
}

CCL_NAMESPACE_END
