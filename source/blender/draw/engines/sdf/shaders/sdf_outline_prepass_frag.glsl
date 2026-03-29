/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_outline_prepass)

void main()
{
  float2 sdf_uv = screen_uv * uv_scale;

  float sdf_depth = texture(sdf_depth_tx, sdf_uv).r;
  if (sdf_depth <= 0.0f || sdf_depth >= 1.0f) {
    discard;
    return;
  }

  float obj_id_f = texture(sdf_gbuf_color_tx, sdf_uv).a;
  int obj_id = int(obj_id_f + 0.5f);
  if (obj_id < 0) {
    discard;
    return;
  }

  uint oid = outline_ids[obj_id];
  if (oid == 0u) {
    discard;
    return;
  }

  out_object_id = oid;
  gl_FragDepth = sdf_depth;
}
