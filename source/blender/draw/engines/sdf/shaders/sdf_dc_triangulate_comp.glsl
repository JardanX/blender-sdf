/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Dual Contouring pass 2: emit quads for sign-change edges.
 * Only emits for cells in [inner_start, inner_end) to avoid
 * duplicate triangles at chunk boundaries. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_dc_triangulate_comp)

float grid_val(int x, int y, int z)
{
  return grid_values[z * grid_verts * grid_verts + y * grid_verts + x];
}

int cell_vert(int cx, int cy, int cz)
{
  int N = grid_verts - 1;
  if (cx < 0 || cy < 0 || cz < 0 || cx >= N || cy >= N || cz >= N) {
    return -1;
  }
  return dc_cell_verts[cz * N * N + cy * N + cx];
}

void emit_quad(int a, int b, int c, int d, bool flip)
{
  int ti = atomicAdd(dc_counters[1], 2);
  if (ti + 1 >= max_tris) return;
  if (flip) {
    dc_triangles[ti] = ivec4(a, c, b, 0);
    dc_triangles[ti + 1] = ivec4(a, d, c, 0);
  }
  else {
    dc_triangles[ti] = ivec4(a, b, c, 0);
    dc_triangles[ti + 1] = ivec4(a, c, d, 0);
  }
}

void main()
{
  ivec3 cid = ivec3(gl_GlobalInvocationID.xyz);
  int N = grid_verts - 1;
  if (cid.x >= N || cid.y >= N || cid.z >= N) return;

  /* Only emit for inner cells (avoids duplicates at chunk boundaries) */
  if (cid.x < inner_start || cid.y < inner_start || cid.z < inner_start ||
      cid.x >= inner_end || cid.y >= inner_end || cid.z >= inner_end)
  {
    return;
  }

  float v0 = grid_val(cid.x, cid.y, cid.z);

  if (cid.y > 0 && cid.z > 0) {
    float v1 = grid_val(cid.x + 1, cid.y, cid.z);
    if ((v0 < 0.0) != (v1 < 0.0)) {
      int a = cell_vert(cid.x, cid.y, cid.z);
      int b = cell_vert(cid.x, cid.y - 1, cid.z);
      int c = cell_vert(cid.x, cid.y - 1, cid.z - 1);
      int d = cell_vert(cid.x, cid.y, cid.z - 1);
      if (a >= 0 && b >= 0 && c >= 0 && d >= 0) emit_quad(a, b, c, d, v0 < 0.0);
    }
  }

  if (cid.x > 0 && cid.z > 0) {
    float v1 = grid_val(cid.x, cid.y + 1, cid.z);
    if ((v0 < 0.0) != (v1 < 0.0)) {
      int a = cell_vert(cid.x, cid.y, cid.z);
      int b = cell_vert(cid.x, cid.y, cid.z - 1);
      int c = cell_vert(cid.x - 1, cid.y, cid.z - 1);
      int d = cell_vert(cid.x - 1, cid.y, cid.z);
      if (a >= 0 && b >= 0 && c >= 0 && d >= 0) emit_quad(a, b, c, d, v0 < 0.0);
    }
  }

  if (cid.x > 0 && cid.y > 0) {
    float v1 = grid_val(cid.x, cid.y, cid.z + 1);
    if ((v0 < 0.0) != (v1 < 0.0)) {
      int a = cell_vert(cid.x, cid.y, cid.z);
      int b = cell_vert(cid.x - 1, cid.y, cid.z);
      int c = cell_vert(cid.x - 1, cid.y - 1, cid.z);
      int d = cell_vert(cid.x, cid.y - 1, cid.z);
      if (a >= 0 && b >= 0 && c >= 0 && d >= 0) emit_quad(a, b, c, d, v0 < 0.0);
    }
  }
}
