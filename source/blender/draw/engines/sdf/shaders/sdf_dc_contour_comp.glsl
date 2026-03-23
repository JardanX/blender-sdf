/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Dual Contouring pass 1: place one vertex per cell via QEF.
 * Uses trilinear interpolation for precise normals and eigenvalue-adaptive
 * regularization: zero for sharp corners, minimal for curved surfaces. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_dc_contour_comp)

#define DC_PI 3.14159265358979

float gv(int x, int y, int z)
{
  return grid_values[clamp(z, 0, grid_verts - 1) * grid_verts * grid_verts +
                     clamp(y, 0, grid_verts - 1) * grid_verts +
                     clamp(x, 0, grid_verts - 1)];
}

float sample_grid(vec3 p)
{
  p = clamp(p, vec3(0.0), vec3(float(grid_verts - 1)));
  ivec3 i = clamp(ivec3(floor(p)), ivec3(0), ivec3(grid_verts - 2));
  vec3 f = p - vec3(i);
  float c000 = gv(i.x, i.y, i.z), c100 = gv(i.x + 1, i.y, i.z);
  float c010 = gv(i.x, i.y + 1, i.z), c110 = gv(i.x + 1, i.y + 1, i.z);
  float c001 = gv(i.x, i.y, i.z + 1), c101 = gv(i.x + 1, i.y, i.z + 1);
  float c011 = gv(i.x, i.y + 1, i.z + 1), c111 = gv(i.x + 1, i.y + 1, i.z + 1);
  return mix(mix(mix(c000, c100, f.x), mix(c010, c110, f.x), f.y),
             mix(mix(c001, c101, f.x), mix(c011, c111, f.x), f.y), f.z);
}

vec3 grid_normal(vec3 p)
{
  float eps = 0.5;
  return vec3(sample_grid(p + vec3(eps, 0, 0)) - sample_grid(p - vec3(eps, 0, 0)),
              sample_grid(p + vec3(0, eps, 0)) - sample_grid(p - vec3(0, eps, 0)),
              sample_grid(p + vec3(0, 0, eps)) - sample_grid(p - vec3(0, 0, eps)));
}

/* Eigenvalues of a 3x3 symmetric matrix (sorted e1 >= e2 >= e3) */
vec3 sym3_eigenvalues(mat3 A)
{
  float p1 = A[0][1] * A[0][1] + A[0][2] * A[0][2] + A[1][2] * A[1][2];
  float q = (A[0][0] + A[1][1] + A[2][2]) / 3.0;

  if (p1 < 1e-30) {
    vec3 e = vec3(A[0][0], A[1][1], A[2][2]);
    if (e.x < e.y) e.xy = e.yx;
    if (e.x < e.z) e.xz = e.zx;
    if (e.y < e.z) e.yz = e.zy;
    return e;
  }

  float b0 = A[0][0] - q, b1 = A[1][1] - q, b2 = A[2][2] - q;
  float p = sqrt((b0 * b0 + b1 * b1 + b2 * b2 + 2.0 * p1) / 6.0);
  mat3 B = (A - mat3(q)) / p;
  float detB = B[0][0] * (B[1][1] * B[2][2] - B[1][2] * B[2][1]) -
               B[0][1] * (B[1][0] * B[2][2] - B[1][2] * B[2][0]) +
               B[0][2] * (B[1][0] * B[2][1] - B[1][1] * B[2][0]);
  float phi = acos(clamp(detB * 0.5, -1.0, 1.0)) / 3.0;

  float e1 = q + 2.0 * p * cos(phi);
  float e3 = q + 2.0 * p * cos(phi + 2.0 * DC_PI / 3.0);
  float e2 = 3.0 * q - e1 - e3;
  return vec3(e1, e2, e3);
}

void main()
{
  ivec3 cid = ivec3(gl_GlobalInvocationID.xyz);
  int N = grid_verts - 1;
  if (cid.x >= N || cid.y >= N || cid.z >= N) return;

  int ci = cid.z * N * N + cid.y * N + cid.x;
  int cx = cid.x, cy = cid.y, cz = cid.z;

  float v000 = gv(cx, cy, cz), v100 = gv(cx + 1, cy, cz);
  float v010 = gv(cx, cy + 1, cz), v110 = gv(cx + 1, cy + 1, cz);
  float v001 = gv(cx, cy, cz + 1), v101 = gv(cx + 1, cy, cz + 1);
  float v011 = gv(cx, cy + 1, cz + 1), v111 = gv(cx + 1, cy + 1, cz + 1);

  vec3 o = vec3(grid_origin) + vec3(cid) * cell_size;
  float cs = cell_size;

  mat3 ata = mat3(0.0);
  vec3 atb = vec3(0.0);
  vec3 mp = vec3(0.0);
  vec3 an = vec3(0.0);
  int ec = 0;

#define EDGE(ox1, oy1, oz1, ox2, oy2, oz2, va, vb)                                   \
  if ((va < 0.0) != (vb < 0.0)) {                                                    \
    float t = va / (va - vb);                                                         \
    vec3 p = o + mix(vec3(float(ox1), float(oy1), float(oz1)),                        \
                     vec3(float(ox2), float(oy2), float(oz2)), t) * cs;               \
    vec3 pg = vec3(cid) + mix(vec3(float(ox1), float(oy1), float(oz1)),               \
                               vec3(float(ox2), float(oy2), float(oz2)), t);          \
    vec3 n = grid_normal(pg);                                                         \
    float len = length(n);                                                            \
    if (len > 1e-8) {                                                                 \
      n /= len;                                                                       \
      ata[0][0] += n.x * n.x; ata[0][1] += n.x * n.y; ata[0][2] += n.x * n.z;      \
      ata[1][1] += n.y * n.y; ata[1][2] += n.y * n.z; ata[2][2] += n.z * n.z;       \
      atb += n * dot(n, p);                                                           \
      mp += p; an += n; ec++;                                                         \
    }                                                                                 \
  }

  EDGE(0,0,0, 1,0,0, v000, v100)
  EDGE(0,1,0, 1,1,0, v010, v110)
  EDGE(0,0,1, 1,0,1, v001, v101)
  EDGE(0,1,1, 1,1,1, v011, v111)
  EDGE(0,0,0, 0,1,0, v000, v010)
  EDGE(1,0,0, 1,1,0, v100, v110)
  EDGE(0,0,1, 0,1,1, v001, v011)
  EDGE(1,0,1, 1,1,1, v101, v111)
  EDGE(0,0,0, 0,0,1, v000, v001)
  EDGE(1,0,0, 1,0,1, v100, v101)
  EDGE(0,1,0, 0,1,1, v010, v011)
  EDGE(1,1,0, 1,1,1, v110, v111)
#undef EDGE

  if (ec == 0) {
    dc_cell_verts[ci] = -1;
    return;
  }

  mp /= float(ec);
  ata[1][0] = ata[0][1];
  ata[2][0] = ata[0][2];
  ata[2][1] = ata[1][2];

  /* Adaptive regularization via eigenvalue analysis.
   * Sharp corners (rank 3): lambda ~ 0 → exact vertex placement.
   * Sharp edges (rank 2): regularize 1 deficient direction.
   * Flat surfaces (rank 1): regularize 2 deficient directions → mass point on plane. */
  vec3 eig = sym3_eigenvalues(ata);
  float threshold = max(eig.x, 1e-6) * 0.1;
  float lam = 0.0;
  if (eig.z < threshold) lam = max(lam, threshold - eig.z);
  if (eig.y < threshold) lam = max(lam, threshold - eig.y);

  mat3 A = ata + mat3(lam);
  vec3 b = atb - ata * mp;

  float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
              A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
              A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

  vec3 v;
  if (abs(det) > 1e-20) {
    float inv = 1.0 / det;
    v.x = mp.x + ((b.x * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                    A[0][1] * (b.y * A[2][2] - A[1][2] * b.z) +
                    A[0][2] * (b.y * A[2][1] - A[1][1] * b.z)) * inv);
    v.y = mp.y + ((A[0][0] * (b.y * A[2][2] - A[1][2] * b.z) -
                    b.x * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                    A[0][2] * (A[1][0] * b.z - b.y * A[2][0])) * inv);
    v.z = mp.z + ((A[0][0] * (A[1][1] * b.z - b.y * A[2][1]) -
                    A[0][1] * (A[1][0] * b.z - b.y * A[2][0]) +
                    b.x * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) * inv);
  }
  else {
    v = mp;
  }

  v = clamp(v, o, o + vec3(cs));

  an = normalize(an);
  if (any(isnan(an))) an = vec3(0.0, 1.0, 0.0);

  int vi = atomicAdd(dc_counters[0], 1);
  if (vi >= max_verts) {
    dc_cell_verts[ci] = -1;
    return;
  }
  dc_vertices[vi * 2] = vec4(v, 0.0);
  dc_vertices[vi * 2 + 1] = vec4(an, 0.0);
  dc_cell_verts[ci] = vi;
}
