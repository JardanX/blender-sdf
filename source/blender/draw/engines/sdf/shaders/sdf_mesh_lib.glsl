/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#define SDF_MESH_NORMAL_SMOOTH 1
#define SDF_MESH_FLAG_CORNER_NORMALS (1 << 3)
#define SDF_MESH_HINT_CACHE_SIZE 8

int g_sdf_mesh_hint_keys[SDF_MESH_HINT_CACHE_SIZE];
int g_sdf_mesh_hint_triangles[SDF_MESH_HINT_CACHE_SIZE];
int g_sdf_mesh_last_triangle;
float3 g_sdf_mesh_last_barycentric;
float3 g_sdf_mesh_last_geometric_normal;
/* Local UNSCALED mesh-space position of the last baked volume sample (set by
 * the SDF_LP_MESH_FLAG_BAKED branch of sdTriangleMesh); consumed by
 * sdfMeshLastWorldNormals to fetch the baked normal at the hit. */
float3 g_sdf_mesh_last_baked_pos;

float sdfMeshPointAabbDistSquared(float3 p, float3 bounds_min, float3 bounds_max)
{
  float3 delta = max(bounds_min - p, max(p - bounds_max, float3(0.0f)));
  return dot(delta, delta);
}

float3 sdfMeshUnpackNormal(uint packed_normal)
{
  float2 oct = unpackSnorm2x16(packed_normal);
  float3 normal = float3(oct, 1.0f - abs(oct.x) - abs(oct.y));
  if (normal.z < 0.0f) {
    float2 sign = float2(oct.x >= 0.0f ? 1.0f : -1.0f,
                         oct.y >= 0.0f ? 1.0f : -1.0f);
    normal.xy = (float2(1.0f) - abs(oct.yx)) * sign;
  }
  return normalize(normal);
}

/* ------------------------------------------------------------------ */
/** \name Baked volume sampling (SDF_LP_MESH_FLAG_BAKED)
 *
 * Trilinear sampling of the per-mesh baked voxel pools (bake_dist/bake_nrm/
 * bake_col, written by sdf_mesh_bake_comp). All coordinates are in the
 * payload's UNSCALED local mesh space; the grid convention (voxel centers,
 * linear index) matches the bake shader exactly.
 * \{ */

/* Manual IEEE-754 float16 -> float32 decode (bit layout mirrors
 * sdf_bake_f32_to_f16_bits in sdf_mesh_bake_comp.glsl; written longhand
 * because unpackHalf2x16 has no other user in the tree and its Metal
 * translation is unverified). */
float sdfBakedF16(uint h)
{
  uint mag = h & 0x7FFFu;
  float f;
  if (mag == 0u) {
    f = 0.0f;
  }
  else if (mag >= 0x7C00u) {
    /* Half inf/nan: never produced by the bake (values are band-clamped). */
    f = 1e30f;
  }
  else if ((mag >> 10u) == 0u) {
    /* Subnormal half: mag * 2^-24 (exactly representable). */
    f = float(mag) * 5.9604644775390625e-8f;
  }
  else {
    /* Normal half: rebias the exponent (15 -> 127), shift the mantissa. */
    f = uintBitsToFloat((((mag >> 10u) + 112u) << 23u) | ((mag & 0x3FFu) << 13u));
  }
  return ((h & 0x8000u) != 0u) ? -f : f;
}

/* Maps p to the containing trilinear cell. Returns false when p is outside
 * the voxel-center range [-0.5, res-0.5] on any axis (the far path). On
 * success `vi` is the linear index of the (0,0,0) corner WITHOUT the pool
 * base, and f the fractional position inside the cell. */
bool sdfBakedGridCoords(float3 origin,
                        float voxel_size,
                        int3 res,
                        float3 p,
                        out int vi,
                        out int stride_y,
                        out int stride_z,
                        out float3 f)
{
  float3 uvw = (p - origin) / voxel_size - 0.5f;
  float3 resf = float3(res);
  if (any(lessThan(uvw, float3(-0.5f))) || any(greaterThan(uvw, resf - 0.5f))) {
    return false;
  }
  int3 i0 = clamp(int3(floor(uvw)), int3(0), res - 2);
  f = clamp(uvw - float3(i0), float3(0.0f), float3(1.0f));
  stride_y = res.x;
  stride_z = res.x * res.y;
  vi = i0.x + i0.y * stride_y + i0.z * stride_z;
  return true;
}

/* Baked signed distance at p (unscaled local mesh space). Far path: the
 * point-to-volume-AABB distance — a proven conservative lower bound (the
 * mesh lies strictly inside the padded volume box), no pool reads. Near
 * path: manual trilinear over 8 fp16 corner distances, scaled by 0.95 as a
 * Lipschitz safety margin for fp16 quantization + interpolation error. */
float sdfBakedSample(float3 origin, float voxel_size, int3 res, int base, float3 p)
{
  int vi;
  int stride_y;
  int stride_z;
  float3 f;
  if (!sdfBakedGridCoords(origin, voxel_size, res, p, vi, stride_y, stride_z, f)) {
    float3 box_max = origin + float3(res) * voxel_size;
    float3 delta = max(origin - p, max(p - box_max, float3(0.0f)));
    return length(delta);
  }
  vi += base;
  float d000 = sdfBakedF16(bake_dist[vi] & 0xFFFFu);
  float d100 = sdfBakedF16(bake_dist[vi + 1] & 0xFFFFu);
  float d010 = sdfBakedF16(bake_dist[vi + stride_y] & 0xFFFFu);
  float d110 = sdfBakedF16(bake_dist[vi + stride_y + 1] & 0xFFFFu);
  float d001 = sdfBakedF16(bake_dist[vi + stride_z] & 0xFFFFu);
  float d101 = sdfBakedF16(bake_dist[vi + stride_z + 1] & 0xFFFFu);
  float d011 = sdfBakedF16(bake_dist[vi + stride_y + stride_z] & 0xFFFFu);
  float d111 = sdfBakedF16(bake_dist[vi + stride_y + stride_z + 1] & 0xFFFFu);
  float d = mix(mix(mix(d000, d100, f.x), mix(d010, d110, f.x), f.y),
                mix(mix(d001, d101, f.x), mix(d011, d111, f.x), f.y),
                f.z);
  return d * 0.95f;
}

/* Baked smooth (corner-normal) shading normal at p. No far path: only
 * called at/near the traced surface, which lies inside the grid. */
float3 sdfBakedNormal(float3 origin, float voxel_size, int3 res, int base, float3 p)
{
  int vi;
  int stride_y;
  int stride_z;
  float3 f;
  if (!sdfBakedGridCoords(origin, voxel_size, res, p, vi, stride_y, stride_z, f)) {
    return float3(0.0f, 0.0f, 1.0f);
  }
  vi += base;
  int c000 = 2 * vi;
  int c100 = 2 * (vi + 1);
  int c010 = 2 * (vi + stride_y);
  int c110 = 2 * (vi + stride_y + 1);
  int c001 = 2 * (vi + stride_z);
  int c101 = 2 * (vi + stride_z + 1);
  int c011 = 2 * (vi + stride_y + stride_z);
  int c111 = 2 * (vi + stride_y + stride_z + 1);
  float3 n000 = float3(sdfBakedF16(bake_nrm[c000] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c000] >> 16u),
                       sdfBakedF16(bake_nrm[c000 + 1] & 0xFFFFu));
  float3 n100 = float3(sdfBakedF16(bake_nrm[c100] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c100] >> 16u),
                       sdfBakedF16(bake_nrm[c100 + 1] & 0xFFFFu));
  float3 n010 = float3(sdfBakedF16(bake_nrm[c010] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c010] >> 16u),
                       sdfBakedF16(bake_nrm[c010 + 1] & 0xFFFFu));
  float3 n110 = float3(sdfBakedF16(bake_nrm[c110] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c110] >> 16u),
                       sdfBakedF16(bake_nrm[c110 + 1] & 0xFFFFu));
  float3 n001 = float3(sdfBakedF16(bake_nrm[c001] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c001] >> 16u),
                       sdfBakedF16(bake_nrm[c001 + 1] & 0xFFFFu));
  float3 n101 = float3(sdfBakedF16(bake_nrm[c101] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c101] >> 16u),
                       sdfBakedF16(bake_nrm[c101 + 1] & 0xFFFFu));
  float3 n011 = float3(sdfBakedF16(bake_nrm[c011] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c011] >> 16u),
                       sdfBakedF16(bake_nrm[c011 + 1] & 0xFFFFu));
  float3 n111 = float3(sdfBakedF16(bake_nrm[c111] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c111] >> 16u),
                       sdfBakedF16(bake_nrm[c111 + 1] & 0xFFFFu));
  return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
             mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y),
             f.z);
}

/* Baked barycentric corner color (RGBA8, linear) at p. Same near-surface
 * assumption as sdfBakedNormal. */
float4 sdfBakedColor(float3 origin, float voxel_size, int3 res, int base, float3 p)
{
  int vi;
  int stride_y;
  int stride_z;
  float3 f;
  if (!sdfBakedGridCoords(origin, voxel_size, res, p, vi, stride_y, stride_z, f)) {
    return float4(1.0f);
  }
  vi += base;
  float4 c000 = unpackUnorm4x8(bake_col[vi]);
  float4 c100 = unpackUnorm4x8(bake_col[vi + 1]);
  float4 c010 = unpackUnorm4x8(bake_col[vi + stride_y]);
  float4 c110 = unpackUnorm4x8(bake_col[vi + stride_y + 1]);
  float4 c001 = unpackUnorm4x8(bake_col[vi + stride_z]);
  float4 c101 = unpackUnorm4x8(bake_col[vi + stride_z + 1]);
  float4 c011 = unpackUnorm4x8(bake_col[vi + stride_y + stride_z]);
  float4 c111 = unpackUnorm4x8(bake_col[vi + stride_y + stride_z + 1]);
  return mix(mix(mix(c000, c100, f.x), mix(c010, c110, f.x), f.y),
             mix(mix(c001, c101, f.x), mix(c011, c111, f.x), f.y),
             f.z);
}

/** \} */

float3 sdfMeshVertexPositionBase(SDFObjectGPU obj, uint vertex)
{
  return uintBitsToFloat(mesh_data_buf[obj.mesh_data.x + int(vertex)].xyz);
}

float3 sdfMeshVertexPosition(SDFObjectGPU obj, uint vertex)
{
  return sdfMeshVertexPositionBase(obj, vertex) * obj.obj_scale.xyz;
}

float3 sdfMeshVertexPseudonormal(SDFObjectGPU obj, uint vertex)
{
  uint packed_normal = mesh_data_buf[obj.mesh_data.x + int(vertex)].w;
  return normalize(sdfMeshUnpackNormal(packed_normal) / obj.obj_scale.xyz);
}

SDFMeshTriangleGPU sdfMeshTriangleLoad(SDFObjectGPU obj, int triangle)
{
  int start = obj.mesh_data.y + triangle * 3;
  SDFMeshTriangleGPU result;
  result.vertices_and_material = mesh_data_buf[start];
  result.corner_normals = mesh_data_buf[start + 1];
  result.edge_normals = mesh_data_buf[start + 2];
  return result;
}

bool sdfMeshTriangleStableLess(SDFMeshTriangleGPU a, SDFMeshTriangleGPU b)
{
  uint a_min = min(a.vertices_and_material.x,
                   min(a.vertices_and_material.y, a.vertices_and_material.z));
  uint a_max = max(a.vertices_and_material.x,
                   max(a.vertices_and_material.y, a.vertices_and_material.z));
  uint a_mid = a.vertices_and_material.x + a.vertices_and_material.y +
               a.vertices_and_material.z - a_min - a_max;
  uint b_min = min(b.vertices_and_material.x,
                   min(b.vertices_and_material.y, b.vertices_and_material.z));
  uint b_max = max(b.vertices_and_material.x,
                   max(b.vertices_and_material.y, b.vertices_and_material.z));
  uint b_mid = b.vertices_and_material.x + b.vertices_and_material.y +
               b.vertices_and_material.z - b_min - b_max;
  if (a_min != b_min) {
    return a_min < b_min;
  }
  if (a_mid != b_mid) {
    return a_mid < b_mid;
  }
  return a_max < b_max;
}

BVHNodeGPU sdfMeshNodeLoad(SDFObjectGPU obj, int node)
{
  int start = obj.mesh_data.w + node * 2;
  uint4 data_min = mesh_data_buf[start];
  uint4 data_max = mesh_data_buf[start + 1];
  BVHNodeGPU result;
  result.min_and_left = uintBitsToFloat(data_min);
  result.max_and_right = uintBitsToFloat(data_max);
  return result;
}

float3 sdfMeshClosestPoint(float3 p,
                           float3 a,
                           float3 b,
                           float3 c,
                           out float3 barycentric)
{
  float3 ab = b - a;
  float3 ac = c - a;
  float3 ap = p - a;
  float d1 = dot(ab, ap);
  float d2 = dot(ac, ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    barycentric = float3(1.0f, 0.0f, 0.0f);
    return a;
  }

  float3 bp = p - b;
  float d3 = dot(ab, bp);
  float d4 = dot(ac, bp);
  if (d3 >= 0.0f && d4 <= d3) {
    barycentric = float3(0.0f, 1.0f, 0.0f);
    return b;
  }

  float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    float v = d1 / (d1 - d3);
    barycentric = float3(1.0f - v, v, 0.0f);
    return a + v * ab;
  }

  float3 cp = p - c;
  float d5 = dot(ab, cp);
  float d6 = dot(ac, cp);
  if (d6 >= 0.0f && d5 <= d6) {
    barycentric = float3(0.0f, 0.0f, 1.0f);
    return c;
  }

  float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    float w = d2 / (d2 - d6);
    barycentric = float3(1.0f - w, 0.0f, w);
    return a + w * ac;
  }

  float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    barycentric = float3(0.0f, 1.0f - w, w);
    return b + w * (c - b);
  }

  float denom = 1.0f / (va + vb + vc);
  float v = vb * denom;
  float w = vc * denom;
  barycentric = float3(1.0f - v - w, v, w);
  return a + ab * v + ac * w;
}

float3 sdfMeshFeatureNormal(SDFObjectGPU obj,
                            SDFMeshTriangleGPU triangle,
                            float3 a,
                            float3 b,
                            float3 c,
                            float3 barycentric)
{
  const float feature_epsilon = 1e-5f;
  if (barycentric.x >= 1.0f - feature_epsilon) {
    return sdfMeshVertexPseudonormal(obj, triangle.vertices_and_material.x);
  }
  if (barycentric.y >= 1.0f - feature_epsilon) {
    return sdfMeshVertexPseudonormal(obj, triangle.vertices_and_material.y);
  }
  if (barycentric.z >= 1.0f - feature_epsilon) {
    return sdfMeshVertexPseudonormal(obj, triangle.vertices_and_material.z);
  }
  if (barycentric.x <= feature_epsilon) {
    return normalize(sdfMeshUnpackNormal(triangle.edge_normals.x) / obj.obj_scale.xyz);
  }
  if (barycentric.y <= feature_epsilon) {
    return normalize(sdfMeshUnpackNormal(triangle.edge_normals.y) / obj.obj_scale.xyz);
  }
  if (barycentric.z <= feature_epsilon) {
    return normalize(sdfMeshUnpackNormal(triangle.edge_normals.z) / obj.obj_scale.xyz);
  }
  return normalize(cross(b - a, c - a));
}

bool sdfMeshNearest(SDFObjectGPU obj,
                    float3 p,
                    out float distance_squared,
                    out int triangle_index,
                    out float3 closest_point,
                    out float3 barycentric,
                    out float3 feature_normal)
{
  distance_squared = 3.402823466e+38f;
  triangle_index = -1;
  closest_point = float3(0.0f);
  barycentric = float3(1.0f, 0.0f, 0.0f);
  feature_normal = float3(0.0f, 0.0f, 1.0f);
  bool feature_normal_valid = false;
  if (obj.mesh_data.z <= 0 || obj.mesh_settings.z <= 0) {
    return false;
  }

  int cache_key = obj.mesh_data.y;
  int cache_slot = (cache_key * 17) & (SDF_MESH_HINT_CACHE_SIZE - 1);
  int hint_triangle = g_sdf_mesh_hint_triangles[cache_slot];
  if (g_sdf_mesh_hint_keys[cache_slot] == cache_key && hint_triangle >= 0 &&
      hint_triangle < obj.mesh_data.z)
  {
    SDFMeshTriangleGPU triangle = sdfMeshTriangleLoad(obj, hint_triangle);
    float3 a = sdfMeshVertexPosition(obj, triangle.vertices_and_material.x);
    float3 b = sdfMeshVertexPosition(obj, triangle.vertices_and_material.y);
    float3 c = sdfMeshVertexPosition(obj, triangle.vertices_and_material.z);
    closest_point = sdfMeshClosestPoint(p, a, b, c, barycentric);
    distance_squared = dot(p - closest_point, p - closest_point);
    triangle_index = hint_triangle;
    feature_normal = sdfMeshFeatureNormal(obj, triangle, a, b, c, barycentric);
    feature_normal_valid = true;
  }

  int local_node = 0;
  while (local_node < obj.mesh_settings.z) {
    BVHNodeGPU node = sdfMeshNodeLoad(obj, local_node);
    int child_or_escape = floatBitsToInt(node.min_and_left.w);
    if (sdfMeshPointAabbDistSquared(p,
                                    node.min_and_left.xyz * obj.obj_scale.xyz,
                                    node.max_and_right.xyz * obj.obj_scale.xyz) >
        distance_squared)
    {
      local_node = child_or_escape >= 0 ? child_or_escape : local_node + 1;
      continue;
    }

    int child_b = floatBitsToInt(node.max_and_right.w);
    if (child_or_escape < 0) {
      int first = -child_or_escape - 1;
      for (int i = 0; i < child_b; i++) {
        int tri_i = first + i;
        SDFMeshTriangleGPU triangle = sdfMeshTriangleLoad(obj, tri_i);
        float3 a = sdfMeshVertexPosition(obj, triangle.vertices_and_material.x);
        float3 b = sdfMeshVertexPosition(obj, triangle.vertices_and_material.y);
        float3 c = sdfMeshVertexPosition(obj, triangle.vertices_and_material.z);
        float3 tri_barycentric;
        float3 tri_closest = sdfMeshClosestPoint(p, a, b, c, tri_barycentric);
        float tri_distance_squared = dot(p - tri_closest, p - tri_closest);
        float tie_epsilon = max(1e-12f,
                                max(tri_distance_squared, distance_squared) * 1e-7f);
        bool is_near_tie = abs(tri_distance_squared - distance_squared) <= tie_epsilon;
        bool is_closer = tri_distance_squared < distance_squared && !is_near_tie;
        bool is_stable_tie = false;
        if (is_near_tie && triangle_index >= 0 && tri_i != triangle_index) {
          SDFMeshTriangleGPU current_triangle = sdfMeshTriangleLoad(obj, triangle_index);
          is_stable_tie = sdfMeshTriangleStableLess(triangle, current_triangle);
        }
        if (is_closer || is_stable_tie) {
          distance_squared = tri_distance_squared;
          triangle_index = tri_i;
          closest_point = tri_closest;
          barycentric = tri_barycentric;
          feature_normal_valid = false;
        }
      }
    }
    local_node++;
  }
  if (triangle_index >= 0) {
    if (!feature_normal_valid) {
      SDFMeshTriangleGPU triangle = sdfMeshTriangleLoad(obj, triangle_index);
      float3 a = sdfMeshVertexPosition(obj, triangle.vertices_and_material.x);
      float3 b = sdfMeshVertexPosition(obj, triangle.vertices_and_material.y);
      float3 c = sdfMeshVertexPosition(obj, triangle.vertices_and_material.z);
      feature_normal = sdfMeshFeatureNormal(obj, triangle, a, b, c, barycentric);
    }
    g_sdf_mesh_hint_keys[cache_slot] = cache_key;
    g_sdf_mesh_hint_triangles[cache_slot] = triangle_index;
  }
  return triangle_index >= 0;
}

float sdTriangleMesh(float3 p, SDFObjectGPU obj)
{
  if ((obj.mesh_settings.y & SDF_LP_MESH_FLAG_BAKED) != 0) {
    /* Baked volume fast path: p is the rotation-only local position; the
     * analytic path scale-multiplies the vertices, so dividing maps into the
     * unscaled volume frame. The min-scale factor converts the sampled
     * distance back to the scaled frame (exact for uniform scale,
     * conservative otherwise). */
    float3 p_unscaled = p / obj.obj_scale.xyz;
    g_sdf_mesh_last_triangle = -1;
    g_sdf_mesh_last_baked_pos = p_unscaled;
    float d = sdfBakedSample(obj.bake_origin.xyz,
                             obj.bake_origin.w,
                             obj.bake_grid.xyz,
                             obj.bake_grid.w,
                             p_unscaled);
    return d * min(min(obj.obj_scale.x, obj.obj_scale.y), obj.obj_scale.z);
  }
  g_sdf_mesh_last_triangle = -1;
  float distance_squared;
  int triangle_index;
  float3 closest_point;
  float3 barycentric;
  float3 feature_normal;
  if (!sdfMeshNearest(
          obj, p, distance_squared, triangle_index, closest_point, barycentric, feature_normal))
  {
    return 1e10f;
  }

  g_sdf_mesh_last_triangle = triangle_index;
  g_sdf_mesh_last_barycentric = barycentric;
  float sign_value = dot(p - closest_point, feature_normal) < 0.0f ? -1.0f : 1.0f;
  g_sdf_mesh_last_geometric_normal = distance_squared > 1e-12f ?
                                         normalize(p - closest_point) * sign_value :
                                         feature_normal;
  return sqrt(max(distance_squared, 0.0f)) * sign_value;
}

float3 sdfMeshShadingNormal(SDFObjectGPU obj, int triangle_index, float3 barycentric)
{
  SDFMeshTriangleGPU triangle = sdfMeshTriangleLoad(obj, triangle_index);
  float3 a = sdfMeshVertexPosition(obj, triangle.vertices_and_material.x);
  float3 b = sdfMeshVertexPosition(obj, triangle.vertices_and_material.y);
  float3 c = sdfMeshVertexPosition(obj, triangle.vertices_and_material.z);
  if ((obj.mesh_settings.y & SDF_MESH_FLAG_CORNER_NORMALS) == 0 &&
      obj.mesh_settings.x != SDF_MESH_NORMAL_SMOOTH)
  {
    return normalize(cross(b - a, c - a));
  }
  float3 n0 = sdfMeshUnpackNormal(triangle.corner_normals.x);
  float3 n1 = sdfMeshUnpackNormal(triangle.corner_normals.y);
  float3 n2 = sdfMeshUnpackNormal(triangle.corner_normals.z);
  return normalize(n0 * barycentric.x + n1 * barycentric.y + n2 * barycentric.z);
}

bool sdfMeshLastWorldNormals(SDFObjectGPU obj,
                             out float3 shading_normal,
                             out float3 geometric_normal)
{
  if ((obj.mesh_settings.y & SDF_LP_MESH_FLAG_BAKED) != 0) {
    /* Baked volume: fetch the baked smooth normal at the last sampled
     * position and return it as both shading and geometric normal. */
    float3 n = sdfBakedNormal(obj.bake_origin.xyz,
                              obj.bake_origin.w,
                              obj.bake_grid.xyz,
                              obj.bake_grid.w,
                              g_sdf_mesh_last_baked_pos);
    float3x3 normal_to_world = transpose(to_float3x3(obj.inverse_matrix));
    float3 world_n = normal_to_world * (n / obj.obj_scale.xyz);
    float len_squared = dot(world_n, world_n);
    if (len_squared <= 1e-12f || any(isnan(world_n))) {
      shading_normal = float3(0.0f);
      geometric_normal = float3(0.0f);
      return false;
    }
    world_n *= inversesqrt(len_squared);
    shading_normal = world_n;
    geometric_normal = world_n;
    return true;
  }
  if (g_sdf_mesh_last_triangle < 0) {
    shading_normal = float3(0.0f);
    geometric_normal = float3(0.0f);
    return false;
  }

  float3x3 normal_to_world = transpose(to_float3x3(obj.inverse_matrix));
  geometric_normal = normal_to_world * g_sdf_mesh_last_geometric_normal * obj.obj_scale.w;
  float geometric_len_squared = dot(geometric_normal, geometric_normal);
  if (geometric_len_squared <= 1e-12f || any(isnan(geometric_normal))) {
    shading_normal = float3(0.0f);
    geometric_normal = float3(0.0f);
    return false;
  }
  bool has_smooth_normals = (obj.mesh_settings.y & SDF_MESH_FLAG_CORNER_NORMALS) != 0 ||
                            obj.mesh_settings.x == SDF_MESH_NORMAL_SMOOTH;
  if (has_smooth_normals) {
    SDFMeshTriangleGPU triangle = sdfMeshTriangleLoad(obj, g_sdf_mesh_last_triangle);
    float3 n0 = normalize(normal_to_world *
                          (sdfMeshUnpackNormal(triangle.corner_normals.x) / obj.obj_scale.xyz));
    float3 n1 = normalize(normal_to_world *
                          (sdfMeshUnpackNormal(triangle.corner_normals.y) / obj.obj_scale.xyz));
    float3 n2 = normalize(normal_to_world *
                          (sdfMeshUnpackNormal(triangle.corner_normals.z) / obj.obj_scale.xyz));
    shading_normal = n0 * g_sdf_mesh_last_barycentric.x +
                     n1 * g_sdf_mesh_last_barycentric.y +
                     n2 * g_sdf_mesh_last_barycentric.z;
  }
  else {
    shading_normal = normal_to_world * sdfMeshShadingNormal(
                                           obj,
                                           g_sdf_mesh_last_triangle,
                                           g_sdf_mesh_last_barycentric);
  }
  float shading_len_squared = dot(shading_normal, shading_normal);
  if (shading_len_squared <= 1e-12f || any(isnan(shading_normal))) {
    shading_normal = geometric_normal;
    shading_len_squared = geometric_len_squared;
  }
  shading_normal *= inversesqrt(shading_len_squared);
  float alignment = dot(shading_normal, geometric_normal) / geometric_len_squared;
  if (alignment < 0.0f) {
    shading_normal -= 2.0f * alignment * geometric_normal;
  }
  shading_normal *= sqrt(geometric_len_squared);
  return true;
}
