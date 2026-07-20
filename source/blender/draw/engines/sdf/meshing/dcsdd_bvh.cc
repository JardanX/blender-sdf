/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Xiana Carrera and contributors.
 * Ported from https://github.com/xianacarrera/dual-contouring-of-signed-distance-data
 * Paper: "Dual Contouring of Signed Distance Data", Carrera, Wang, Batty, Stein,
 * Sellán, SIGGRAPH 2026. */

#include "dcsdd_bvh.hh"

#include <algorithm>
#include <limits>
#include <numeric>

#include "BLI_index_range.hh"
#include "BLI_task.hh"

namespace blender::sdf::meshing {

/* Closest point on a triangle to a point (Ericson, "Real-Time Collision
 * Detection", section 5.1.5). */
static Eigen::Vector3d closest_point_on_triangle(const Eigen::Vector3d &p,
                                                 const Eigen::Vector3d &a,
                                                 const Eigen::Vector3d &b,
                                                 const Eigen::Vector3d &c)
{
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d ap = p - a;

  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }

  const Eigen::Vector3d bp = p - b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return b;
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    return a + (d1 / (d1 - d3)) * ab;
  }

  const Eigen::Vector3d cp = p - c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return c;
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    return a + (d2 / (d2 - d6)) * ac;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
  }

  const double denom = 1.0 / (va + vb + vc);
  return a + ab * (vb * denom) + ac * (vc * denom);
}

static double squared_distance_to_aabb(const Eigen::Vector3d &p,
                                       const Eigen::Vector3d &bbox_min,
                                       const Eigen::Vector3d &bbox_max)
{
  const Eigen::Vector3d clamped = p.cwiseMax(bbox_min).cwiseMin(bbox_max);
  return (p - clamped).squaredNorm();
}

void TriMeshBVH::init(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F)
{
  verts_.clear();
  tris_.clear();
  tri_indices_.clear();
  nodes_.clear();

  verts_.resize(V.rows());
  for (int i = 0; i < V.rows(); ++i) {
    verts_[i] = V.row(i).transpose();
  }

  if (F.cols() == 3) {
    tris_.resize(F.rows());
    for (int i = 0; i < F.rows(); ++i) {
      tris_[i] = F.row(i).transpose();
    }
  }
  else if (F.cols() == 4) {
    /* Split quads along the (a, c) diagonal, matching triangulate_v1. */
    tris_.resize(F.rows() * 2);
    for (int i = 0; i < F.rows(); ++i) {
      tris_[2 * i] = Eigen::Vector3i(F(i, 0), F(i, 1), F(i, 2));
      tris_[2 * i + 1] = Eigen::Vector3i(F(i, 0), F(i, 2), F(i, 3));
    }
  }
  else {
    return;
  }

  const int n_tris = int(tris_.size());
  if (n_tris == 0) {
    return;
  }

  std::vector<Eigen::Vector3d> centroids(n_tris);
  for (int t = 0; t < n_tris; ++t) {
    centroids[t] = (verts_[tris_[t](0)] + verts_[tris_[t](1)] + verts_[tris_[t](2)]) / 3.0;
  }

  tri_indices_.resize(n_tris);
  std::iota(tri_indices_.begin(), tri_indices_.end(), 0);

  nodes_.reserve(2 * n_tris);
  build_recursive(centroids, 0, n_tris);
}

int TriMeshBVH::build_recursive(const std::vector<Eigen::Vector3d> &centroids,
                                int start,
                                int end)
{
  constexpr int leaf_size = 8;

  const int node_idx = int(nodes_.size());
  nodes_.emplace_back();

  /* Bounding box of the triangles in this range. */
  Eigen::Vector3d bbox_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d bbox_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
  Eigen::Vector3d centroid_min = bbox_min;
  Eigen::Vector3d centroid_max = bbox_max;
  for (int t = start; t < end; ++t) {
    const Eigen::Vector3i &tri = tris_[tri_indices_[t]];
    for (int v = 0; v < 3; ++v) {
      bbox_min = bbox_min.cwiseMin(verts_[tri(v)]);
      bbox_max = bbox_max.cwiseMax(verts_[tri(v)]);
    }
    centroid_min = centroid_min.cwiseMin(centroids[tri_indices_[t]]);
    centroid_max = centroid_max.cwiseMax(centroids[tri_indices_[t]]);
  }

  nodes_[node_idx].bbox_min = bbox_min;
  nodes_[node_idx].bbox_max = bbox_max;

  if (end - start <= leaf_size) {
    nodes_[node_idx].start = start;
    nodes_[node_idx].count = end - start;
    return node_idx;
  }

  /* Median split along the largest axis of the centroid bounding box. */
  const Eigen::Vector3d extent = centroid_max - centroid_min;
  int axis = 0;
  if (extent.y() > extent.x()) {
    axis = 1;
  }
  if (extent.z() > extent[axis]) {
    axis = 2;
  }

  const int mid = start + (end - start) / 2;
  std::nth_element(tri_indices_.begin() + start,
                   tri_indices_.begin() + mid,
                   tri_indices_.begin() + end,
                   [&](int ta, int tb) { return centroids[ta][axis] < centroids[tb][axis]; });

  nodes_[node_idx].left = build_recursive(centroids, start, mid);
  nodes_[node_idx].right = build_recursive(centroids, mid, end);
  return node_idx;
}

void TriMeshBVH::query_recursive(int node_idx,
                                 const Eigen::Vector3d &p,
                                 double &best_sqr,
                                 int &best_tri) const
{
  const Node &node = nodes_[node_idx];

  if (squared_distance_to_aabb(p, node.bbox_min, node.bbox_max) > best_sqr) {
    return;
  }

  if (node.is_leaf()) {
    for (int t = node.start; t < node.start + node.count; ++t) {
      const int tri_idx = tri_indices_[t];
      const Eigen::Vector3i &tri = tris_[tri_idx];
      const Eigen::Vector3d cp = closest_point_on_triangle(
          p, verts_[tri(0)], verts_[tri(1)], verts_[tri(2)]);
      const double sqr = (p - cp).squaredNorm();
      if (sqr < best_sqr) {
        best_sqr = sqr;
        best_tri = tri_idx;
      }
    }
    return;
  }

  /* Visit the nearer child first for better pruning. */
  const double dist_left = squared_distance_to_aabb(
      p, nodes_[node.left].bbox_min, nodes_[node.left].bbox_max);
  const double dist_right = squared_distance_to_aabb(
      p, nodes_[node.right].bbox_min, nodes_[node.right].bbox_max);
  if (dist_left < dist_right) {
    query_recursive(node.left, p, best_sqr, best_tri);
    query_recursive(node.right, p, best_sqr, best_tri);
  }
  else {
    query_recursive(node.right, p, best_sqr, best_tri);
    query_recursive(node.left, p, best_sqr, best_tri);
  }
}

double TriMeshBVH::squared_distance(const Eigen::Vector3d &p,
                                    int &face_idx,
                                    Eigen::Vector3d &closest) const
{
  double best_sqr = std::numeric_limits<double>::max();
  int best_tri = -1;
  if (!nodes_.empty()) {
    query_recursive(0, p, best_sqr, best_tri);
  }
  face_idx = best_tri;
  if (best_tri >= 0) {
    const Eigen::Vector3i &tri = tris_[best_tri];
    closest = closest_point_on_triangle(p, verts_[tri(0)], verts_[tri(1)], verts_[tri(2)]);
  }
  else {
    closest = p;
  }
  return best_sqr;
}

void TriMeshBVH::squared_distance(const Eigen::MatrixXd &P,
                                  Eigen::VectorXd &sqrD,
                                  Eigen::VectorXi &I,
                                  Eigen::MatrixXd &C) const
{
  const int64_t n = P.rows();
  sqrD.resize(n);
  I.resize(n);
  C.resize(n, 3);

  /* Queries are const, so the batch can be evaluated concurrently. */
  threading::parallel_for(IndexRange(n), 1024, [&](const IndexRange range) {
    for (const int64_t i : range) {
      const Eigen::Vector3d p = P.row(i).transpose();
      int face_idx = -1;
      Eigen::Vector3d closest = p;
      sqrD(i) = squared_distance(p, face_idx, closest);
      I(i) = face_idx;
      C.row(i) = closest.transpose();
    }
  });
}

}  // namespace blender::sdf::meshing
