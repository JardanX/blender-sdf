/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning debug colorize pass: recolors hit pixels for the LP debug
 * shading modes after the shared color resolve + normal passes.
 * - HEATMAP: per-cell active node count from the pruning grid metadata.
 * - NORMALS: the gbuf_normal produced by the shared normal pass.
 * Neither mode evaluates the SDF tree, so this shader stays tiny (no
 * sdf_lp_common.glsl) and compiles in milliseconds. Miss pixels are skipped
 * (the color resolve pass already cleared them); gbuf_color.a (the object id
 * used by picking) is preserved. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_lp_debug_comp)

#include "sdf_lp_cell_lib.glsl"

#define LP_SHADING_HEATMAP 1
#define LP_SHADING_NORMALS 2

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);
  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float4 pos = imageLoad(gbuf_pos_img, pixel);
  if (pos.w <= 0.0f) {
    return;
  }

  float3 out_color;
  if (shading_mode == LP_SHADING_HEATMAP) {
    float num_active = 0.0f;
    if (culling_enabled != 0) {
      /* Divide by 2 to approximate the number of primitives (binary tree).
       * Fallback cells show as the maximum (useful overflow diagnostic). */
      float3 cell_size = (aabb_max - aabb_min) / float(grid_size);
      int3 hit_cell = lp_cell_from_pos(pos.xyz, aabb_min, cell_size, grid_size);
      int cell_count = lp_cell_meta[int(lp_cell_idx(hit_cell))].x;
      num_active = float(cell_count >= 0 ? cell_count : total_num_nodes) + 1.0f;
      num_active *= 0.5f;
    }
    else {
      num_active = float(total_num_nodes + 1) * 0.5f;
    }
    out_color = lp_inferno(clamp(num_active / max(viz_max, 1.0f), 0.0f, 1.0f));
  }
  else {
    out_color = imageLoad(gbuf_normal_img, pixel).xyz * 0.5f + 0.5f;
  }

  float obj_id = imageLoad(gbuf_color_img, pixel).a;
  imageStore(gbuf_color_img, pixel, float4(out_color, obj_id));
}
