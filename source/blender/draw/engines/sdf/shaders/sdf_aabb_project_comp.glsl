/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Pre-project object AABBs to screen-space pixel bounds. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_aabb_project_comp)

#include "draw_view_lib.glsl"

void main()
{
  int idx = int(gl_GlobalInvocationID.x);
  if (idx >= object_count) {
    return;
  }

  if (objects[idx]._pad2 == 0) {
    screen_aabbs[idx] = int4(-1, -1, -2, -2);
    return;
  }

  /* Intersection (2) / push (4) must be in every tile — force fullscreen. */
  int csg_op = objects[idx].csg_operation;
  if (csg_op == 2 || csg_op == 4) {
    screen_aabbs[idx] = int4(0, 0, screen_size.x, screen_size.y);
    return;
  }

  ViewMatrices vm = drw_view();
  float4x4 viewproj = vm.winmat * vm.viewmat;

  float3 bmin = objects[idx].bbox_min.xyz;
  float3 bmax = objects[idx].bbox_max.xyz;

  float2 proj_min = float2(1e30f);
  float2 proj_max = float2(-1e30f);
  int behind_count = 0;

  for (int i = 0; i < 8; i++) {
    float3 corner = float3((i & 1) != 0 ? bmax.x : bmin.x,
                           (i & 2) != 0 ? bmax.y : bmin.y,
                           (i & 4) != 0 ? bmax.z : bmin.z);
    float4 clip = viewproj * float4(corner, 1.0f);
    if (clip.w <= 0.0f) {
      behind_count++;
      continue;
    }
    float2 ndc = clip.xy / clip.w;
    proj_min = min(proj_min, ndc);
    proj_max = max(proj_max, ndc);
  }

  if (behind_count == 8) {
    screen_aabbs[idx] = int4(-1, -1, -2, -2);
    return;
  }

  if (behind_count > 0) {
    screen_aabbs[idx] = int4(0, 0, screen_size.x, screen_size.y);
    return;
  }

  float2 pmin = (proj_min * 0.5f + 0.5f) * float2(screen_size);
  float2 pmax = (proj_max * 0.5f + 0.5f) * float2(screen_size);

  screen_aabbs[idx] = int4(max(int(floor(pmin.x)), 0),
                           max(int(floor(pmin.y)), 0),
                           min(int(ceil(pmax.x)), screen_size.x),
                           min(int(ceil(pmax.y)), screen_size.y));
}
