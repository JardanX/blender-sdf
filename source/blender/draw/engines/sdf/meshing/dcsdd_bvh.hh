/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Xiana Carrera and contributors.
 * Ported from https://github.com/xianacarrera/dual-contouring-of-signed-distance-data
 * Paper: "Dual Contouring of Signed Distance Data", Carrera, Wang, Batty, Stein,
 * Sellán, SIGGRAPH 2026. */

#pragma once

#include <vector>

#include <Eigen/Core>

namespace blender::sdf::meshing {

/**
 * Self-contained BVH over a triangle soup, replacing the libigl AABB tree used
 * by the original DC-SDD implementation (`igl::point_mesh_squared_distance` and
 * `igl::AABB<Eigen::MatrixXd, 3>`).
 *
 * The tree is built once from (V, F) and only const queries are performed
 * afterwards, so a single instance is safe to query concurrently from multiple
 * threads.
 */
class TriMeshBVH {
 public:
  TriMeshBVH() = default;
  TriMeshBVH(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F)
  {
    init(V, F);
  }

  /** (Re)build the tree from a triangle mesh. Quad faces are split along the
   * (a, c) diagonal, matching how callers triangulate before querying. */
  void init(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);

  bool empty() const
  {
    return nodes_.empty();
  }

  /**
   * Batch closest-point query, mirroring `igl::point_mesh_squared_distance`:
   * for every row of P, compute the squared distance to the mesh, the index of
   * the closest triangle (row of the F given to init(), with quads mapped to
   * their first split triangle) and the closest point itself.
   */
  void squared_distance(const Eigen::MatrixXd &P,
                        Eigen::VectorXd &sqrD,
                        Eigen::VectorXi &I,
                        Eigen::MatrixXd &C) const;

  /**
   * Single-point closest-point query, mirroring
   * `igl::AABB::squared_distance(V, F, p, face_idx, closest)`.
   * Returns the squared distance to the mesh.
   */
  double squared_distance(const Eigen::Vector3d &p, int &face_idx, Eigen::Vector3d &closest) const;

 private:
  struct Node {
    Eigen::Vector3d bbox_min;
    Eigen::Vector3d bbox_max;
    /* Inner node children (indices into nodes_), -1 for leaves. */
    int left = -1;
    int right = -1;
    /* Leaf triangle range within tri_indices_. */
    int start = 0;
    int count = 0;

    bool is_leaf() const
    {
      return left < 0;
    }
  };

  std::vector<Eigen::Vector3d> verts_;
  std::vector<Eigen::Vector3i> tris_;
  /* Triangle indices permuted so leaf ranges are contiguous. */
  std::vector<int> tri_indices_;
  std::vector<Node> nodes_;

  int build_recursive(const std::vector<Eigen::Vector3d> &centroids, int start, int end);
  void query_recursive(int node_idx,
                       const Eigen::Vector3d &p,
                       double &best_sqr,
                       int &best_tri) const;
};

}  // namespace blender::sdf::meshing
