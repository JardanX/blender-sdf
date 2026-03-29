/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF picking: reads SDF engine G-buffer to identify hit object,
 * writes to Blender's native selection buffer.
 * Only processes pixels near the cursor to match mesh pick behavior.
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_analytical_pick)

void main()
{
  /* Only process pixels near the cursor — matches mesh implicit spatial locality. */
  int2 cursor_delta = abs(int2(gl_FragCoord.xy) - select_info_buf.cursor);
  int pixel_dist = max(cursor_delta.x, cursor_delta.y);

  if (select_info_buf.mode == SELECT_ALL) {
    /* Box/circle select: use radius. */
    if (select_info_buf.radius > 0) {
      uint dist_sq = uint(cursor_delta.x * cursor_delta.x +
                          cursor_delta.y * cursor_delta.y);
      uint rad_sq = uint(select_info_buf.radius) * uint(select_info_buf.radius);
      if (dist_sq > rad_sq) {
        discard;
        return;
      }
    }
  }
  else {
    /* Click select: only the cursor pixel. */
    if (pixel_dist > 1) {
      discard;
      return;
    }
  }

  float2 uv = screen_uv;
  float2 sdf_uv = uv * uv_scale;

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

  uint sel_id = select_id_map_buf[obj_id];
  if (sel_id == uint(-1)) {
    discard;
    return;
  }

  gl_FragDepth = sdf_depth;

  if (select_info_buf.mode == SELECT_ALL) {
    atomicOr(out_select_buf[sel_id / 32u], 1u << (sel_id % 32u));
  }
  else if (select_info_buf.mode == SELECT_PICK_ALL) {
    atomicMin(out_select_buf[sel_id], floatBitsToUint(sdf_depth));
  }
  else if (select_info_buf.mode == SELECT_PICK_NEAREST) {
    uint dist = uint(pixel_dist);
    uint depth = uint(sdf_depth * float(0x00FFFFFFu));
    if (dist < 0xFFu) {
      atomicMin(out_select_buf[sel_id], (depth << 8u) | dist);
    }
  }
}
