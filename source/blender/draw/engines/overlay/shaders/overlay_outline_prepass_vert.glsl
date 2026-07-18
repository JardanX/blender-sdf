/* SPDX-FileCopyrightText: 2018-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/overlay_outline_infos.hh"

VERTEX_SHADER_CREATE_INFO(overlay_outline_prepass_mesh)

#include "draw_model_lib.glsl"
#include "draw_object_infos_lib.glsl"
#include "draw_view_clipping_lib.glsl"
#include "draw_view_lib.glsl"
#include "gpu_shader_utildefines_lib.glsl"

uint outline_colorid_get()
{
  if (outline_color_override >= 0) {
    return uint(outline_color_override);
  }

  eObjectInfoFlag ob_flag = drw_object_infos().flag;
  bool is_active = flag_test(ob_flag, OBJECT_ACTIVE);

  if (is_transform) {
    return 0u; /* theme.colors.transform */
  }
  else if (is_active) {
    return 3u; /* theme.colors.active */
  }
  else {
    return 1u; /* theme.colors.object_select */
  }

  return 0u;
}

void main()
{
  float3 world_pos = drw_point_object_to_world(pos);

  gl_Position = drw_point_world_to_homogenous(world_pos);

  /* No Z-bias: writing the exact scene depth lets the SDF outline prepass (which
   * writes its true sdf depth) win at the real intersection, so the outline ends
   * exactly on the geometry edge instead of being offset by the previous 1e-3
   * clip-space bias. The resolve occlusion check tolerates float precision via
   * its existing 3/8388608 epsilon, so this no longer self-occludes either. */

  /* ID 0 is nothing (background) */
  interp.ob_id = uint(drw_resource_id() + 1);

  /* Should be 2 bits only [0..3]. */
  uint outline_id = outline_colorid_get();

  /* Combine for 16bit uint target. */
  interp.ob_id = outline_id_pack(outline_id, interp.ob_id);

  view_clipping_distances(world_pos);
}
