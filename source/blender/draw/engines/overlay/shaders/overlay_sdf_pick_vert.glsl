/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/overlay_sdf_infos.hh"

VERTEX_SHADER_CREATE_INFO(overlay_sdf_pick)

#include "draw_model_lib.glsl"
#include "draw_view_lib.glsl"
#include "select_lib.glsl"

void main()
{
  select_id_set(drw_custom_id());

  /* Pass resource index to the fragment shader so it can access drw_modelinv(). */
  drw_ResourceID_iface.resource_index = drw_resource_id_raw();

  /* Scale unit cube [-1,1] by sdf_bound to create the bounding volume.
   * The model matrix is the unscaled object transform, so object-space
   * coordinates match SDF evaluation space directly. */
  world_pos = drw_point_object_to_world(pos * sdf_bound);
  gl_Position = drw_point_world_to_homogenous(world_pos);
}
