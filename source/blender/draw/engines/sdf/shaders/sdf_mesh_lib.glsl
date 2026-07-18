/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#define SDF_MESH_NORMAL_SMOOTH 1
#define SDF_MESH_HINT_CACHE_SIZE 8

int g_sdf_mesh_hint_keys[SDF_MESH_HINT_CACHE_SIZE];
int g_sdf_mesh_hint_triangles[SDF_MESH_HINT_CACHE_SIZE];

float sdfMeshPointAabbDistSquared(float3 p, float3 bounds_min, float3 bounds_max)
{
  float3 delta = max(bounds_min - p, max(p - bounds_max, float3(0.0f)));
  return dot(delta, delta);
}

float3 sdfMeshUnpackNormal(uint packed)
{
  float2 oct = unpackSnorm2x16(packed);
  float3 normal = float3(oct, 1.0f - abs(oct.x) - abs(oct.y));
  if (normal.z < 0.0f) {
    float2 sign = float2(oct.x >= 0.0f ? 1.0f : -1.0f,
                         oct.y >= 0.0f ? 1.0f : -1.0f);
    normal.xy = (float2(1.0f) - abs(oct.yx)) * sign;
  }
  return normalize(normal);
}

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
  uint packed = mesh_data_buf[obj.mesh_data.x + int(vertex)].w;
  return normalize(sdfMeshUnpackNormal(packed) / obj.obj_scale.xyz);
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

bool sdfMeshNearest(SDFObjectGPU obj,
                    float3 p,
                    out float distance_squared,
                    out int triangle_index,
                    out float3 closest_point,
                    out float3 barycentric)
{
  distance_squared = 3.402823466e+38f;
  triangle_index = -1;
  closest_point = float3(0.0f);
  barycentric = float3(1.0f, 0.0f, 0.0f);
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
        if (tri_distance_squared < distance_squared) {
          distance_squared = tri_distance_squared;
          triangle_index = tri_i;
          closest_point = tri_closest;
          barycentric = tri_barycentric;
        }
      }
    }
    local_node++;
  }
  if (triangle_index >= 0) {
    g_sdf_mesh_hint_keys[cache_slot] = cache_key;
    g_sdf_mesh_hint_triangles[cache_slot] = triangle_index;
  }
  return triangle_index >= 0;
}

float3 sdfMeshSignNormal(SDFObjectGPU obj,
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

float sdTriangleMesh(float3 p, SDFObjectGPU obj)
{
  float distance_squared;
  int triangle_index;
  float3 closest_point;
  float3 barycentric;
  if (!sdfMeshNearest(
          obj, p, distance_squared, triangle_index, closest_point, barycentric))
  {
    return 1e10f;
  }

  SDFMeshTriangleGPU triangle = sdfMeshTriangleLoad(obj, triangle_index);
  float3 a = sdfMeshVertexPosition(obj, triangle.vertices_and_material.x);
  float3 b = sdfMeshVertexPosition(obj, triangle.vertices_and_material.y);
  float3 c = sdfMeshVertexPosition(obj, triangle.vertices_and_material.z);
  float3 sign_normal = sdfMeshSignNormal(obj, triangle, a, b, c, barycentric);
  float sign_value = dot(p - closest_point, sign_normal) < 0.0f ? -1.0f : 1.0f;
  return sqrt(max(distance_squared, 0.0f)) * sign_value;
}

float3 sdfMeshShadingNormal(SDFObjectGPU obj, int triangle_index, float3 barycentric)
{
  SDFMeshTriangleGPU triangle = sdfMeshTriangleLoad(obj, triangle_index);
  float3 a = sdfMeshVertexPosition(obj, triangle.vertices_and_material.x);
  float3 b = sdfMeshVertexPosition(obj, triangle.vertices_and_material.y);
  float3 c = sdfMeshVertexPosition(obj, triangle.vertices_and_material.z);
  if (obj.mesh_settings.x == SDF_MESH_NORMAL_SMOOTH) {
    float3 n0 = sdfMeshUnpackNormal(triangle.corner_normals.x);
    float3 n1 = sdfMeshUnpackNormal(triangle.corner_normals.y);
    float3 n2 = sdfMeshUnpackNormal(triangle.corner_normals.z);
    return normalize(n0 * barycentric.x + n1 * barycentric.y + n2 * barycentric.z);
  }
  return normalize(cross(b - a, c - a));
}
