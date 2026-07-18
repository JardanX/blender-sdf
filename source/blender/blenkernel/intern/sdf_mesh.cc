/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>

#include "MEM_guardedalloc.h"

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_listbase.h"
#include "BLI_map.hh"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "BKE_mesh.hh"
#include "BKE_object_types.hh"
#include "BKE_sdf.hh"

namespace blender {

template<typename T> static void sdf_mesh_array_free(T *&data)
{
  if (data != nullptr) {
    MEM_delete_void(static_cast<void *>(data));
    data = nullptr;
  }
}

struct bke::SDFMeshObjectRuntime {
  mutable std::mutex mutex;
  std::shared_ptr<const SDFMeshPayload> payload;
  SDFMeshBuildResult result = SDFMeshBuildResult::NoTriangles;
  int degenerate_triangles = 0;
};

static std::atomic<uint64_t> sdf_mesh_revision(1);

SDFMeshPayload::~SDFMeshPayload()
{
  sdf_mesh_array_free(vertices);
  sdf_mesh_array_free(triangles);
  sdf_mesh_array_free(bvh_nodes);
}

static bool sdf_mesh_bvh_flatten(const SDFMeshBVHNode *source,
                                 const int node_count,
                                 Vector<SDFMeshBVHNode> &r_nodes)
{
  if (source == nullptr || node_count <= 0) {
    return false;
  }
  Array<uint8_t> visited(node_count, 0);
  r_nodes.reserve(node_count);
  const auto flatten = [&](auto &&self, const int source_index, const int depth) -> bool {
    if (source_index < 0 || source_index >= node_count || depth > 64 || visited[source_index]) {
      return false;
    }
    visited[source_index] = 1;
    const SDFMeshBVHNode source_node = source[source_index];
    const int output_index = int(r_nodes.size());
    r_nodes.append(source_node);
    if (source_node.child_or_first >= 0) {
      if (!self(self, source_node.child_or_first, depth + 1) ||
          !self(self, source_node.child_or_count, depth + 1))
      {
        return false;
      }
      r_nodes[output_index].child_or_first = int(r_nodes.size());
      r_nodes[output_index].child_or_count = 0;
    }
    return true;
  };
  return flatten(flatten, 0, 0) && r_nodes.size() == node_count;
}

static bool sdf_mesh_bvh_validate(const SDF &sdf)
{
  if (sdf.mesh_vertices == nullptr || sdf.mesh_triangles == nullptr ||
      sdf.mesh_bvh_nodes == nullptr || sdf.mesh_vertex_count <= 0 ||
      sdf.mesh_triangle_count <= 0 || sdf.mesh_bvh_node_count <= 0)
  {
    return false;
  }
  for (int i = 0; i < sdf.mesh_bvh_node_count; i++) {
    const SDFMeshBVHNode &node = sdf.mesh_bvh_nodes[i];
    for (int axis = 0; axis < 3; axis++) {
      if (!std::isfinite(node.bounds_min[axis]) || !std::isfinite(node.bounds_max[axis]) ||
          node.bounds_min[axis] > node.bounds_max[axis])
      {
        return false;
      }
    }
    if (node.child_or_first >= 0) {
      if (node.child_or_first <= i || node.child_or_first > sdf.mesh_bvh_node_count ||
          node.child_or_count != 0)
      {
        return false;
      }
    }
    else {
      const int first = -node.child_or_first - 1;
      if (first < 0 || node.child_or_count <= 0 ||
          first + node.child_or_count > sdf.mesh_triangle_count)
      {
        return false;
      }
    }
  }
  for (int i = 0; i < sdf.mesh_triangle_count; i++) {
    for (const uint32_t vertex : sdf.mesh_triangles[i].vertices) {
      if (vertex >= uint32_t(sdf.mesh_vertex_count)) {
        return false;
      }
    }
  }
  return true;
}

bool BKE_sdf_mesh_bvh_make_stackless(SDF *sdf)
{
  if (sdf == nullptr) {
    return false;
  }
  if (sdf->mesh_flags & SDF_MESH_FLAG_THREADED_BVH) {
    return sdf_mesh_bvh_validate(*sdf);
  }
  Vector<SDFMeshBVHNode> threaded_nodes;
  if (!sdf_mesh_bvh_flatten(
          sdf->mesh_bvh_nodes, sdf->mesh_bvh_node_count, threaded_nodes))
  {
    return false;
  }
  SDFMeshBVHNode *nodes = MEM_new_array_uninitialized<SDFMeshBVHNode>(
      threaded_nodes.size(), __func__);
  memcpy(nodes, threaded_nodes.data(), sizeof(SDFMeshBVHNode) * threaded_nodes.size());
  sdf_mesh_array_free(sdf->mesh_bvh_nodes);
  sdf->mesh_bvh_nodes = nodes;
  sdf->mesh_flags |= SDF_MESH_FLAG_THREADED_BVH;
  return sdf_mesh_bvh_validate(*sdf);
}

static uint32_t sdf_mesh_pack_normal(float3 normal)
{
  const float len = math::length(normal);
  if (!(len > 1e-20f) || !std::isfinite(len)) {
    normal = float3(0.0f, 0.0f, 1.0f);
  }
  else {
    normal /= len;
  }

  float2 oct = float2(normal.x, normal.y) /
               (std::abs(normal.x) + std::abs(normal.y) + std::abs(normal.z));
  if (normal.z < 0.0f) {
    const float2 sign = float2(oct.x >= 0.0f ? 1.0f : -1.0f,
                               oct.y >= 0.0f ? 1.0f : -1.0f);
    oct = (float2(1.0f) - math::abs(float2(oct.y, oct.x))) * sign;
  }

  const int16_t x = int16_t(std::lround(std::clamp(oct.x, -1.0f, 1.0f) * 32767.0f));
  const int16_t y = int16_t(std::lround(std::clamp(oct.y, -1.0f, 1.0f) * 32767.0f));
  return uint32_t(uint16_t(x)) | (uint32_t(uint16_t(y)) << 16);
}

static float sdf_mesh_bounds_area(const float3 &bounds_min, const float3 &bounds_max)
{
  const float3 extent = math::max(bounds_max - bounds_min, float3(0.0f));
  return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
}

SDFMeshBuildResult BKE_sdf_mesh_build(SDF *sdf,
                                      const Mesh *mesh,
                                      int *r_degenerate_triangles)
{
  if (r_degenerate_triangles) {
    *r_degenerate_triangles = 0;
  }

  const Span<float3> positions = mesh->vert_positions();
  const OffsetIndices<int> faces = mesh->faces();
  const Span<int2> edges = mesh->edges();
  const Span<int> corner_verts = mesh->corner_verts();
  const Span<int> corner_edges = mesh->corner_edges();
  const Span<int3> corner_tris = mesh->corner_tris();
  if (positions.is_empty() || corner_tris.is_empty()) {
    return SDFMeshBuildResult::NoTriangles;
  }
  for (const float3 position : positions) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
      return SDFMeshBuildResult::InvalidTopology;
    }
  }

  Array<int> edge_face_count(edges.size());
  Array<int> edge_orientation(edges.size());
  Array<int2> edge_faces(edges.size(), int2(-1));
  edge_face_count.fill(0);
  edge_orientation.fill(0);

  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    for (const int corner : face) {
      const int next_corner = (corner == face.last()) ? face.first() : corner + 1;
      const int edge_i = corner_edges[corner];
      if (edge_i < 0 || edge_i >= edges.size()) {
        return SDFMeshBuildResult::InvalidTopology;
      }
      const int2 edge = edges[edge_i];
      const int v0 = corner_verts[corner];
      const int v1 = corner_verts[next_corner];
      if (!((v0 == edge.x && v1 == edge.y) || (v0 == edge.y && v1 == edge.x))) {
        return SDFMeshBuildResult::InvalidTopology;
      }
      if (edge_face_count[edge_i] == 0) {
        edge_faces[edge_i].x = face_i;
      }
      else if (edge_face_count[edge_i] == 1) {
        edge_faces[edge_i].y = face_i;
      }
      edge_face_count[edge_i]++;
      edge_orientation[edge_i] += (v0 == edge.x) ? 1 : -1;
    }
  }

  for (const int edge_i : edges.index_range()) {
    if (edge_face_count[edge_i] != 0 &&
        (edge_face_count[edge_i] != 2 || edge_orientation[edge_i] != 0))
    {
      return SDFMeshBuildResult::InvalidTopology;
    }
  }

  const GroupedSpan<int> vert_to_face_map = mesh->vert_to_face_map();
  for (const int vert : positions.index_range()) {
    const Span<int> vert_faces = vert_to_face_map[vert];
    if (vert_faces.is_empty()) {
      continue;
    }
    const int first_face = vert_faces.first();
    int current_face = first_face;
    int incoming_edge = -1;
    int visited_faces = 0;
    while (true) {
      const IndexRange face = faces[current_face];
      int incident_edges[2] = {-1, -1};
      int matching_corners = 0;
      for (const int corner : face) {
        if (corner_verts[corner] != vert) {
          continue;
        }
        const int previous_corner = corner == face.first() ? face.last() : corner - 1;
        incident_edges[0] = corner_edges[previous_corner];
        incident_edges[1] = corner_edges[corner];
        matching_corners++;
      }
      if (matching_corners != 1) {
        return SDFMeshBuildResult::InvalidTopology;
      }

      int outgoing_edge;
      if (incoming_edge < 0 || incoming_edge == incident_edges[0]) {
        outgoing_edge = incident_edges[1];
      }
      else if (incoming_edge == incident_edges[1]) {
        outgoing_edge = incident_edges[0];
      }
      else {
        return SDFMeshBuildResult::InvalidTopology;
      }
      const int2 adjacent_faces = edge_faces[outgoing_edge];
      const int next_face = adjacent_faces.x == current_face ? adjacent_faces.y :
                                                                  adjacent_faces.x;
      if (next_face < 0 || next_face == current_face) {
        return SDFMeshBuildResult::InvalidTopology;
      }
      visited_faces++;
      if (next_face == first_face) {
        if (visited_faces != vert_faces.size()) {
          return SDFMeshBuildResult::InvalidTopology;
        }
        break;
      }
      if (visited_faces >= vert_faces.size()) {
        return SDFMeshBuildResult::InvalidTopology;
      }
      incoming_edge = outgoing_edge;
      current_face = next_face;
    }
  }

  Array<int> face_components(faces.size(), -1);
  int component_count = 0;
  Vector<int> face_stack;
  for (const int first_face : faces.index_range()) {
    if (face_components[first_face] >= 0) {
      continue;
    }
    face_components[first_face] = component_count;
    face_stack.append(first_face);
    while (!face_stack.is_empty()) {
      const int face_i = face_stack.pop_last();
      for (const int corner : faces[face_i]) {
        const int2 adjacent_faces = edge_faces[corner_edges[corner]];
        const int adjacent_face = adjacent_faces.x == face_i ? adjacent_faces.y :
                                                                   adjacent_faces.x;
        if (face_components[adjacent_face] < 0) {
          face_components[adjacent_face] = component_count;
          face_stack.append(adjacent_face);
        }
      }
    }
    component_count++;
  }

  const std::optional<Bounds<float3>> mesh_bounds = mesh->bounds_min_max();
  const float3 volume_origin = mesh_bounds ? math::midpoint(mesh_bounds->min, mesh_bounds->max) :
                                             float3(0.0f);
  Array<double> signed_volumes(component_count, 0.0);
  const Span<int> corner_tri_faces = mesh->corner_tri_faces();
  for (const int tri_i : corner_tris.index_range()) {
    const int3 corners = corner_tris[tri_i];
    const float3 a = positions[corner_verts[corners.x]] - volume_origin;
    const float3 b = positions[corner_verts[corners.y]] - volume_origin;
    const float3 c = positions[corner_verts[corners.z]] - volume_origin;
    const double volume =
        (double(a.x) * (double(b.y) * double(c.z) - double(b.z) * double(c.y)) +
         double(a.y) * (double(b.z) * double(c.x) - double(b.x) * double(c.z)) +
         double(a.z) * (double(b.x) * double(c.y) - double(b.y) * double(c.x))) /
        6.0;
    signed_volumes[face_components[corner_tri_faces[tri_i]]] += volume;
  }

  const float diagonal = mesh_bounds ? math::length(mesh_bounds->max - mesh_bounds->min) : 1.0f;
  const double volume_epsilon = std::max(double(diagonal) * double(diagonal) *
                                             double(diagonal) * 1e-12,
                                          1e-30);
  int orientation = 0;
  for (const double signed_volume : signed_volumes) {
    if (std::abs(signed_volume) <= volume_epsilon) {
      return SDFMeshBuildResult::InvalidTopology;
    }
    const int component_orientation = signed_volume < 0.0 ? -1 : 1;
    if (orientation != 0 && orientation != component_orientation) {
      return SDFMeshBuildResult::InvalidTopology;
    }
    orientation = component_orientation;
  }

  const Span<float3> corner_normals = mesh->corner_normals();
  const Span<float3> vertex_normals = mesh->vert_normals();
  Array<float3> vertex_pseudonormals(positions.size());
  vertex_pseudonormals.fill(float3(0.0f));

  Vector<SDFMeshTriangle> source_triangles;
  source_triangles.reserve(corner_tris.size());
  float3 bounds_min(std::numeric_limits<float>::max());
  float3 bounds_max(std::numeric_limits<float>::lowest());
  int degenerate_triangles = 0;

  for (const int tri_i : corner_tris.index_range()) {
    int3 corners = corner_tris[tri_i];
    int3 verts = int3(corner_verts[corners.x],
                      corner_verts[corners.y],
                      corner_verts[corners.z]);
    if (orientation < 0) {
      std::swap(corners.y, corners.z);
      std::swap(verts.y, verts.z);
    }
    const float3 a = positions[verts.x];
    const float3 b = positions[verts.y];
    const float3 c = positions[verts.z];
    const float3 cross = math::cross(b - a, c - a);
    const float cross_len = math::length(cross);
    if (!(cross_len > 1e-20f) || !std::isfinite(cross_len)) {
      degenerate_triangles++;
      continue;
    }
    const float3 face_normal = cross / cross_len;

    SDFMeshTriangle triangle = {};
    triangle.vertices[0] = uint32_t(verts.x);
    triangle.vertices[1] = uint32_t(verts.y);
    triangle.vertices[2] = uint32_t(verts.z);
    triangle.material_index = 0;
    for (int i = 0; i < 3; i++) {
      triangle.corner_normals[i] = sdf_mesh_pack_normal(corner_normals[corners[i]] *
                                                        float(orientation));
      bounds_min = math::min(bounds_min, positions[verts[i]]);
      bounds_max = math::max(bounds_max, positions[verts[i]]);

      const float3 e0 = positions[verts[(i + 1) % 3]] - positions[verts[i]];
      const float3 e1 = positions[verts[(i + 2) % 3]] - positions[verts[i]];
      const float angle = std::atan2(math::length(math::cross(e0, e1)), math::dot(e0, e1));
      vertex_pseudonormals[verts[i]] += face_normal * angle;
    }

    source_triangles.append(triangle);
  }

  if (r_degenerate_triangles) {
    *r_degenerate_triangles = degenerate_triangles;
  }
  if (degenerate_triangles > 0) {
    return SDFMeshBuildResult::DegenerateTriangles;
  }
  if (source_triangles.is_empty()) {
    return SDFMeshBuildResult::NoTriangles;
  }

  Map<uint64_t, float3> edge_pseudonormals;
  for (const SDFMeshTriangle &triangle : source_triangles) {
    const float3 a = positions[triangle.vertices[0]];
    const float3 b = positions[triangle.vertices[1]];
    const float3 c = positions[triangle.vertices[2]];
    const float3 face_normal = math::normalize(math::cross(b - a, c - a));
    for (int i = 0; i < 3; i++) {
      const uint32_t va = triangle.vertices[(i + 1) % 3];
      const uint32_t vb = triangle.vertices[(i + 2) % 3];
      const uint32_t v_min = std::min(va, vb);
      const uint32_t v_max = std::max(va, vb);
      const uint64_t key = (uint64_t(v_min) << 32) | uint64_t(v_max);
      edge_pseudonormals.lookup_or_add(key, float3(0.0f)) += face_normal;
    }
  }
  for (SDFMeshTriangle &triangle : source_triangles) {
    for (int i = 0; i < 3; i++) {
      const uint32_t va = triangle.vertices[(i + 1) % 3];
      const uint32_t vb = triangle.vertices[(i + 2) % 3];
      const uint32_t v_min = std::min(va, vb);
      const uint32_t v_max = std::max(va, vb);
      const uint64_t key = (uint64_t(v_min) << 32) | uint64_t(v_max);
      triangle.edge_normals[i] = sdf_mesh_pack_normal(edge_pseudonormals.lookup(key));
    }
  }

  struct PrimitiveBounds {
    float3 min;
    float3 max;
    float3 centroid;
  };

  const int triangle_count = int(source_triangles.size());
  Array<PrimitiveBounds> primitive_bounds(triangle_count);
  Array<int> primitive_order(triangle_count);
  threading::parallel_for(IndexRange(triangle_count), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      const SDFMeshTriangle &triangle = source_triangles[i];
      const float3 a = positions[triangle.vertices[0]];
      const float3 b = positions[triangle.vertices[1]];
      const float3 c = positions[triangle.vertices[2]];
      primitive_bounds[i].min = math::min(a, math::min(b, c));
      primitive_bounds[i].max = math::max(a, math::max(b, c));
      primitive_bounds[i].centroid = (a + b + c) / 3.0f;
      primitive_order[i] = i;
    }
  });

  Array<SDFMeshBVHNode> build_nodes(size_t(triangle_count) * 2);
  Array<SDFMeshTriangle> ordered_triangles(triangle_count);
  std::atomic<int> next_node(1);
  std::atomic<int> next_triangle(0);

  const auto build_node = [&](auto &&self,
                              const int node_i,
                              const int start,
                              const int count,
                              const int depth) -> void {
    float3 node_min(std::numeric_limits<float>::max());
    float3 node_max(std::numeric_limits<float>::lowest());
    float3 centroid_min(std::numeric_limits<float>::max());
    float3 centroid_max(std::numeric_limits<float>::lowest());
    for (int i = start; i < start + count; i++) {
      const PrimitiveBounds &bounds = primitive_bounds[primitive_order[i]];
      node_min = math::min(node_min, bounds.min);
      node_max = math::max(node_max, bounds.max);
      centroid_min = math::min(centroid_min, bounds.centroid);
      centroid_max = math::max(centroid_max, bounds.centroid);
    }

    SDFMeshBVHNode &node = build_nodes[node_i];
    for (int axis = 0; axis < 3; axis++) {
      node.bounds_min[axis] = std::nextafter(
          node_min[axis], -std::numeric_limits<float>::infinity());
      node.bounds_max[axis] = std::nextafter(
          node_max[axis], std::numeric_limits<float>::infinity());
    }

    if (count <= 4 || depth >= 60) {
      const int first = next_triangle.fetch_add(count, std::memory_order_relaxed);
      for (int i = 0; i < count; i++) {
        ordered_triangles[first + i] = source_triangles[primitive_order[start + i]];
      }
      node.child_or_first = -first - 1;
      node.child_or_count = count;
      return;
    }

    constexpr int bin_count = 16;
    int best_axis = -1;
    int best_split = -1;
    float best_cost = std::numeric_limits<float>::max();

    struct Bin {
      float3 min;
      float3 max;
      int count;
    };

    for (int axis = 0; axis < 3; axis++) {
      const float extent = centroid_max[axis] - centroid_min[axis];
      if (!(extent > 1e-20f)) {
        continue;
      }
      Bin bins[bin_count];
      for (Bin &bin : bins) {
        bin.min = float3(std::numeric_limits<float>::max());
        bin.max = float3(std::numeric_limits<float>::lowest());
        bin.count = 0;
      }
      const float scale = float(bin_count) / extent;
      for (int i = start; i < start + count; i++) {
        const PrimitiveBounds &bounds = primitive_bounds[primitive_order[i]];
        const int bin_i = std::clamp(
            int((bounds.centroid[axis] - centroid_min[axis]) * scale), 0, bin_count - 1);
        bins[bin_i].min = math::min(bins[bin_i].min, bounds.min);
        bins[bin_i].max = math::max(bins[bin_i].max, bounds.max);
        bins[bin_i].count++;
      }

      float3 left_min[bin_count - 1], left_max[bin_count - 1];
      float3 right_min[bin_count - 1], right_max[bin_count - 1];
      int left_count[bin_count - 1], right_count[bin_count - 1];
      float3 running_min(std::numeric_limits<float>::max());
      float3 running_max(std::numeric_limits<float>::lowest());
      int running_count = 0;
      for (int i = 0; i < bin_count - 1; i++) {
        if (bins[i].count > 0) {
          running_min = math::min(running_min, bins[i].min);
          running_max = math::max(running_max, bins[i].max);
        }
        running_count += bins[i].count;
        left_min[i] = running_min;
        left_max[i] = running_max;
        left_count[i] = running_count;
      }
      running_min = float3(std::numeric_limits<float>::max());
      running_max = float3(std::numeric_limits<float>::lowest());
      running_count = 0;
      for (int i = bin_count - 1; i > 0; i--) {
        if (bins[i].count > 0) {
          running_min = math::min(running_min, bins[i].min);
          running_max = math::max(running_max, bins[i].max);
        }
        running_count += bins[i].count;
        right_min[i - 1] = running_min;
        right_max[i - 1] = running_max;
        right_count[i - 1] = running_count;
      }

      for (int split = 0; split < bin_count - 1; split++) {
        if (left_count[split] == 0 || right_count[split] == 0) {
          continue;
        }
        const float cost = float(left_count[split]) *
                               sdf_mesh_bounds_area(left_min[split], left_max[split]) +
                           float(right_count[split]) *
                               sdf_mesh_bounds_area(right_min[split], right_max[split]);
        if (cost < best_cost) {
          best_cost = cost;
          best_axis = axis;
          best_split = split;
        }
      }
    }

    int middle = start;
    if (best_axis >= 0) {
      const float extent = centroid_max[best_axis] - centroid_min[best_axis];
      const float split_position = centroid_min[best_axis] +
                                   extent * float(best_split + 1) / float(bin_count);
      const auto middle_it = std::partition(
          primitive_order.begin() + start,
          primitive_order.begin() + start + count,
          [&](const int primitive_i) {
            return primitive_bounds[primitive_i].centroid[best_axis] < split_position;
          });
      middle = int(middle_it - primitive_order.begin());
    }

    if (middle == start || middle == start + count) {
      const float3 centroid_extent = centroid_max - centroid_min;
      int axis = 0;
      if (centroid_extent.y > centroid_extent.x) {
        axis = 1;
      }
      if (centroid_extent.z > centroid_extent[axis]) {
        axis = 2;
      }
      middle = start + count / 2;
      std::nth_element(primitive_order.begin() + start,
                       primitive_order.begin() + middle,
                       primitive_order.begin() + start + count,
                       [&](const int a, const int b) {
                         return primitive_bounds[a].centroid[axis] <
                                primitive_bounds[b].centroid[axis];
                       });
    }

    const int left_count = middle - start;
    const int right_count = count - left_count;
    const int left_node = next_node.fetch_add(2, std::memory_order_relaxed);
    const int right_node = left_node + 1;
    node.child_or_first = left_node;
    node.child_or_count = right_node;

    threading::parallel_invoke(
        count >= 32768,
        [&]() { self(self, left_node, start, left_count, depth + 1); },
        [&]() { self(self, right_node, middle, right_count, depth + 1); });
  };

  build_node(build_node, 0, 0, triangle_count, 0);

  const int node_count = next_node.load(std::memory_order_relaxed);
  Vector<SDFMeshBVHNode> threaded_nodes;
  if (!sdf_mesh_bvh_flatten(build_nodes.data(), node_count, threaded_nodes)) {
    return SDFMeshBuildResult::InvalidTopology;
  }
  SDFMeshVertex *mesh_vertices = MEM_new_array_uninitialized<SDFMeshVertex>(positions.size(),
                                                                            __func__);
  SDFMeshTriangle *mesh_triangles = MEM_new_array_uninitialized<SDFMeshTriangle>(triangle_count,
                                                                                 __func__);
  SDFMeshBVHNode *mesh_nodes = MEM_new_array_uninitialized<SDFMeshBVHNode>(node_count, __func__);

  threading::parallel_for(positions.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      mesh_vertices[i].co[0] = positions[i].x;
      mesh_vertices[i].co[1] = positions[i].y;
      mesh_vertices[i].co[2] = positions[i].z;
      const float3 pseudonormal = math::length_squared(vertex_pseudonormals[i]) > 1e-20f ?
                                      vertex_pseudonormals[i] :
                                      vertex_normals[i] * float(orientation);
      mesh_vertices[i].pseudonormal = sdf_mesh_pack_normal(pseudonormal);
    }
  });
  memcpy(mesh_triangles, ordered_triangles.data(), sizeof(SDFMeshTriangle) * triangle_count);
  memcpy(mesh_nodes, threaded_nodes.data(), sizeof(SDFMeshBVHNode) * node_count);

  sdf_mesh_array_free(sdf->mesh_vertices);
  sdf_mesh_array_free(sdf->mesh_triangles);
  sdf_mesh_array_free(sdf->mesh_bvh_nodes);
  sdf->mesh_vertices = mesh_vertices;
  sdf->mesh_triangles = mesh_triangles;
  sdf->mesh_bvh_nodes = mesh_nodes;
  sdf->mesh_vertex_count = int(positions.size());
  sdf->mesh_triangle_count = triangle_count;
  sdf->mesh_bvh_node_count = node_count;
  /* Shading uses the evaluated mesh corner normals, including sharp edges. */
  sdf->mesh_normal_mode = SDF_MESH_NORMAL_SMOOTH;
  sdf->mesh_flags = SDF_MESH_FLAG_CLOSED | SDF_MESH_FLAG_ORIENTED |
                    SDF_MESH_FLAG_THREADED_BVH | SDF_MESH_FLAG_CORNER_NORMALS;
  sdf->blend_type = SDF_BLEND_LINEAR;
  sdf->blend = 0.0f;
  sdf->mesh_data_version++;
  for (int axis = 0; axis < 3; axis++) {
    sdf->mesh_bounds_min[axis] = bounds_min[axis];
    sdf->mesh_bounds_max[axis] = bounds_max[axis];
  }
  sdf->sdf_type = SDF_TYPE_MESH;
  return SDFMeshBuildResult::Success;
}

bool BKE_sdf_object_is_enabled(const Object &object)
{
  return object.type == OB_MESH && object.is_sdf;
}

void BKE_sdf_mesh_runtime_update(Object &object, const Mesh &mesh)
{
  if (!BKE_sdf_object_is_enabled(object) || !object.runtime) {
    BKE_sdf_mesh_runtime_clear(object);
    return;
  }

  std::shared_ptr<bke::SDFMeshObjectRuntime> runtime = object.runtime->sdf_mesh;
  if (!runtime) {
    runtime = std::make_shared<bke::SDFMeshObjectRuntime>();
    object.runtime->sdf_mesh = runtime;
  }

  SDF temporary_sdf = {};
  int degenerate_triangles = 0;
  const SDFMeshBuildResult result = BKE_sdf_mesh_build(
      &temporary_sdf, &mesh, &degenerate_triangles);

  std::shared_ptr<SDFMeshPayload> payload;
  if (result == SDFMeshBuildResult::Success) {
    payload = std::make_shared<SDFMeshPayload>();
    payload->vertices = temporary_sdf.mesh_vertices;
    payload->triangles = temporary_sdf.mesh_triangles;
    payload->bvh_nodes = temporary_sdf.mesh_bvh_nodes;
    payload->vertex_count = temporary_sdf.mesh_vertex_count;
    payload->triangle_count = temporary_sdf.mesh_triangle_count;
    payload->bvh_node_count = temporary_sdf.mesh_bvh_node_count;
    payload->flags = temporary_sdf.mesh_flags;
    copy_v3_v3(payload->bounds_min, temporary_sdf.mesh_bounds_min);
    copy_v3_v3(payload->bounds_max, temporary_sdf.mesh_bounds_max);
    payload->revision = sdf_mesh_revision.fetch_add(1, std::memory_order_relaxed);
  }

  {
    std::lock_guard lock(runtime->mutex);
    runtime->payload = std::move(payload);
    runtime->result = result;
    runtime->degenerate_triangles = degenerate_triangles;
  }
}

void BKE_sdf_mesh_runtime_clear(Object &object)
{
  if (object.runtime) {
    object.runtime->sdf_mesh.reset();
  }
}

bool BKE_sdf_mesh_runtime_snapshot(const Object &object, SDFMeshRuntimeSnapshot &r_snapshot)
{
  if (!BKE_sdf_object_is_enabled(object) || !object.runtime) {
    return false;
  }
  const std::shared_ptr<bke::SDFMeshObjectRuntime> runtime = object.runtime->sdf_mesh;
  if (!runtime) {
    return false;
  }
  std::lock_guard lock(runtime->mutex);
  r_snapshot.payload = runtime->payload;
  r_snapshot.result = runtime->result;
  r_snapshot.degenerate_triangles = runtime->degenerate_triangles;
  return r_snapshot.result == SDFMeshBuildResult::Success && r_snapshot.payload != nullptr;
}

}  // namespace blender
