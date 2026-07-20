/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Xiana Carrera and contributors.
 * Ported from https://github.com/xianacarrera/dual-contouring-of-signed-distance-data
 * Paper: "Dual Contouring of Signed Distance Data", Carrera, Wang, Batty, Stein,
 * Sellán, SIGGRAPH 2026. */

#include "dcsdd_hermite.hh"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <Eigen/SVD>

#include "BLI_index_range.hh"
#include "BLI_task.hh"
#include "BLI_utildefines.h"

#include "dcsdd_contouring.hh"

namespace blender::sdf::meshing {

/*                 7 -------- 6
 *                /|         /|
 *               / |        / |
 *             4 --------- 5  |
 *             |  |       |   |
 *             |  3 ------|-- 2
 *             | /        |  /
 *             |/         | /
 *             0 --------- 1
 */

/* Decode corner index (0..7) into local offsets dx,dy,dz (0 or 1). */
static inline void get_corner_offsets(int corner_idx, int &dx, int &dy, int &dz)
{
  dz = (corner_idx >= 4) ? 1 : 0;
  const int rem = corner_idx % 4;
  dx = (rem == 1 || rem == 2) ? 1 : 0;
  dy = (rem == 2 || rem == 3) ? 1 : 0;
}

static int get_edge_idx_from_grid_vertices(
    Cell &cell, int i0, int j0, int k0, int i1, int j1, int k1)
{
  /* Returns the edge index (0-11) corresponding to the edge between the two given grid
   * vertices (i0,j0,k0) and (i1,j1,k1). */

  /* Calculate local offsets within the cell. */
  const int di0 = i0 - cell.ix;
  const int dj0 = j0 - cell.iy;
  const int dk0 = k0 - cell.iz;
  const int di1 = i1 - cell.ix;
  const int dj1 = j1 - cell.iy;
  const int dk1 = k1 - cell.iz;

  for (int edge_idx = 0; edge_idx < 12; ++edge_idx) {
    /* Get the corner indices for this edge. */
    const int c_a = EDGE_PAIRS[edge_idx][0];
    const int c_b = EDGE_PAIRS[edge_idx][1];

    /* Decode coordinates for corner A. */
    int a_di, a_dj, a_dk;
    get_corner_offsets(c_a, a_di, a_dj, a_dk);

    /* Decode coordinates for corner B. */
    int b_di, b_dj, b_dk;
    get_corner_offsets(c_b, b_di, b_dj, b_dk);

    /* Check if the input points match this edge (order agnostic). */
    const bool match_forward = (di0 == a_di && dj0 == a_dj && dk0 == a_dk) &&
                               (di1 == b_di && dj1 == b_dj && dk1 == b_dk);

    const bool match_backward = (di0 == b_di && dj0 == b_dj && dk0 == b_dk) &&
                                (di1 == a_di && dj1 == a_dj && dk1 == a_dk);

    if (match_forward || match_backward) {
      return edge_idx;
    }
  }
  /* Debug-only logging. */
#ifndef NDEBUG
  std::cerr << "get_edge_idx_from_grid_vertices: could not find edge for local offsets (" << di0
            << "," << dj0 << "," << dk0 << ") to (" << di1 << "," << dj1 << "," << dk1 << ")"
            << std::endl;
#endif

  return -1;
}

bool compute_local_mesh(Cell &cell,
                        Cell &adj_cell1,
                        Cell &adj_cell2,
                        Cell &adj_cell3,
                        int edge_idx,
                        std::pair<int, int> int_dirs,
                        Eigen::MatrixXd &V,
                        int &next_vertex_index,
                        Eigen::MatrixXi &F,
                        int &next_face_index)
{
  const int copy_next_vertex = next_vertex_index;

  const size_t Nc = cell.assigned_spheres.size();
  const size_t Na1 = adj_cell1.assigned_spheres.size();
  const size_t Na2 = adj_cell2.assigned_spheres.size();
  const size_t Na3 = adj_cell3.assigned_spheres.size();
  const size_t N = Nc + Na1 + Na2 + Na3;
  if (N == 0) {
    return false;
  }

  auto it_fip1 = cell.face_intersections.find(int_dirs.first);
  if (it_fip1 == cell.face_intersections.end()) {
    return false;
  }
  auto it_fip2 = cell.face_intersections.find(int_dirs.second);
  if (it_fip2 == cell.face_intersections.end()) {
    return false;
  }

  std::pair<int, int> neg_dirs = {int_dirs.first, int_dirs.second};
  neg_dirs.first = (neg_dirs.first % 2 == 0) ? neg_dirs.first + 1 : neg_dirs.first - 1;
  neg_dirs.second = (neg_dirs.second % 2 == 0) ? neg_dirs.second + 1 : neg_dirs.second - 1;

  auto it_fip3 = adj_cell3.face_intersections.find(neg_dirs.first);
  if (it_fip3 == adj_cell3.face_intersections.end()) {
    return false;
  }
  auto it_fip4 = adj_cell3.face_intersections.find(neg_dirs.second);
  if (it_fip4 == adj_cell3.face_intersections.end()) {
    return false;
  }

  V.conservativeResize(next_vertex_index + 9, 3);
  V.row(next_vertex_index++) = cell.vertex;
  V.row(next_vertex_index++) = adj_cell1.vertex;
  V.row(next_vertex_index++) = adj_cell2.vertex;
  V.row(next_vertex_index++) = adj_cell3.vertex;
  V.row(next_vertex_index++) = cell.hermite_positions[edge_idx];
  V.row(next_vertex_index++) = it_fip1->second;
  V.row(next_vertex_index++) = it_fip2->second;
  V.row(next_vertex_index++) = it_fip3->second;
  V.row(next_vertex_index++) = it_fip4->second;

  F.conservativeResize(next_face_index + 8, 3);
  F.row(next_face_index++) = Eigen::Vector3i(copy_next_vertex, copy_next_vertex + 4, copy_next_vertex + 5);
  F.row(next_face_index++) = Eigen::Vector3i(copy_next_vertex + 1, copy_next_vertex + 4, copy_next_vertex + 5);
  F.row(next_face_index++) = Eigen::Vector3i(copy_next_vertex + 1, copy_next_vertex + 4, copy_next_vertex + 8);
  F.row(next_face_index++) = Eigen::Vector3i(copy_next_vertex + 3, copy_next_vertex + 4, copy_next_vertex + 8);
  F.row(next_face_index++) = Eigen::Vector3i(copy_next_vertex + 3, copy_next_vertex + 4, copy_next_vertex + 7);
  F.row(next_face_index++) = Eigen::Vector3i(copy_next_vertex + 2, copy_next_vertex + 4, copy_next_vertex + 7);
  F.row(next_face_index++) = Eigen::Vector3i(copy_next_vertex + 2, copy_next_vertex + 4, copy_next_vertex + 6);
  F.row(next_face_index++) = Eigen::Vector3i(copy_next_vertex, copy_next_vertex + 4, copy_next_vertex + 6);

  return true;
}

#ifndef NDEBUG
static bool check_cells_are_adjacent(
    int ix1, int ix2, int iy1, int iy2, int iz1, int iz2, int resX, int resY, int resZ)
{
  UNUSED_VARS(resX, resY, resZ);

  /* Check if the cells are adjacent in the grid. */
  return (ix1 == ix2 && iy1 == iy2 && std::abs(iz1 - iz2) == 1) ||
         (ix1 == ix2 && std::abs(iy1 - iy2) == 1 && iz1 == iz2) ||
         (std::abs(ix1 - ix2) == 1 && iy1 == iy2 && iz1 == iz2);
}
#endif

void process_edge(bool just_maps,
                  std::vector<Cell> &cells,
                  int i0,
                  int j0,
                  int k0,
                  int i1,
                  int j1,
                  int k1,
                  int resX,
                  int resY,
                  int resZ,
                  Eigen::MatrixXd &V_global,
                  int &next_vertex_index,
                  Eigen::MatrixXi &F_global,
                  int &next_face_index,
                  std::vector<Eigen::Vector4i> &edge_cells,
                  std::vector<Eigen::Vector4i> &edge_local_edge_idxs,
                  int &edge_ordinal,
                  bool verbose)
{
  /* The local-mesh outputs were only used by compute_local_mesh, which the original
   * disabled as a bottleneck; they are kept in the signature for API compatibility. */
  UNUSED_VARS(just_maps, V_global, next_vertex_index, F_global, next_face_index);

  auto inside_grid = [&](int ii, int jj, int kk) {
    return ii > 0 && ii < (resX - 1) && jj > 0 && jj < (resY - 1) && kk > 0 && kk < (resZ - 1);
  };

  /* Check that i0,j0,k0 and i1,j1,k1 are not both on the boundary. */
  if (!inside_grid(i0, j0, k0) && !inside_grid(i1, j1, k1)) {
    if (verbose) {
      if (i0 == 1 && j0 == 1 && k0 == 1) {
        std::cout << "process_edge: vertex (" << i0 << "," << j0 << "," << k0
                  << ") is at the corner (1,1,1)." << std::endl;
      }
      if (i1 == 1 && j1 == 1 && k1 == 1) {
        std::cout << "process_edge: vertex (" << i1 << "," << j1 << "," << k1
                  << ") is at the corner (1,1,1)." << std::endl;
      }
    }
    return;
  }

  const int cell_idx = cellIndex3D(i0, j0, k0, resX, resY, resZ);
  if (cell_idx < 0 || cell_idx >= (int)cells.size()) {
    return;
  }

  Cell &cell = cells[cell_idx];
  const int edge_idx = get_edge_idx_from_grid_vertices(cell, i0, j0, k0, i1, j1, k1);
  if (edge_idx == -1) {
#ifndef NDEBUG
    std::cerr << "process_edge: could not find edge index for edge between (" << i0 << "," << j0
              << "," << k0 << ") and (" << i1 << "," << j1 << "," << k1 << ")" << std::endl;
#endif
    return;
  }

  auto it = cell.hermite_positions.find(edge_idx);
  if (it == cell.hermite_positions.end()) {
    return;
  }

  const auto face_pair = get_face_normals_from_edge_idx(edge_idx);

  const int dirs[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  const int dir1[3] = {dirs[face_pair.first][0], dirs[face_pair.first][1], dirs[face_pair.first][2]};
  const int dir2[3] = {dirs[face_pair.second][0], dirs[face_pair.second][1], dirs[face_pair.second][2]};

  const int ni_1 = cell.ix + dir1[0];
  const int nj_1 = cell.iy + dir1[1];
  const int nk_1 = cell.iz + dir1[2];

  const int ni_2 = cell.ix + dir2[0];
  const int nj_2 = cell.iy + dir2[1];
  const int nk_2 = cell.iz + dir2[2];

  const int ni_3 = ni_1 + dir2[0];
  const int nj_3 = nj_1 + dir2[1];
  const int nk_3 = nk_1 + dir2[2];

  const int neighbor_idx_1 = cellIndex3D(ni_1, nj_1, nk_1, resX, resY, resZ);
  const int neighbor_idx_2 = cellIndex3D(ni_2, nj_2, nk_2, resX, resY, resZ);
  const int neighbor_idx_3 = cellIndex3D(ni_3, nj_3, nk_3, resX, resY, resZ);

  /* Assert that these are truly neighboring cells (debug-only). */
#ifndef NDEBUG
  {
    const bool neighbors_ok =
        check_cells_are_adjacent(cell.ix,
                                 cells[neighbor_idx_1].ix,
                                 cell.iy,
                                 cells[neighbor_idx_1].iy,
                                 cell.iz,
                                 cells[neighbor_idx_1].iz,
                                 resX,
                                 resY,
                                 resZ) &&
        check_cells_are_adjacent(cell.ix,
                                 cells[neighbor_idx_2].ix,
                                 cell.iy,
                                 cells[neighbor_idx_2].iy,
                                 cell.iz,
                                 cells[neighbor_idx_2].iz,
                                 resX,
                                 resY,
                                 resZ) &&
        check_cells_are_adjacent(cells[neighbor_idx_1].ix,
                                 cells[neighbor_idx_3].ix,
                                 cells[neighbor_idx_1].iy,
                                 cells[neighbor_idx_3].iy,
                                 cells[neighbor_idx_1].iz,
                                 cells[neighbor_idx_3].iz,
                                 resX,
                                 resY,
                                 resZ) &&
        check_cells_are_adjacent(cells[neighbor_idx_2].ix,
                                 cells[neighbor_idx_3].ix,
                                 cells[neighbor_idx_2].iy,
                                 cells[neighbor_idx_3].iy,
                                 cells[neighbor_idx_2].iz,
                                 cells[neighbor_idx_3].iz,
                                 resX,
                                 resY,
                                 resZ);
    assert(neighbors_ok && "process_edge: neighboring cells are not adjacent!");
  }
#endif

  Cell &adj_cell1 = cells[neighbor_idx_1];
  Cell &adj_cell2 = cells[neighbor_idx_2];
  Cell &adj_cell3 = cells[neighbor_idx_3];

  if (!cell.has_vertex || !adj_cell1.has_vertex || !adj_cell2.has_vertex || !adj_cell3.has_vertex)
  {
    if (verbose) {
      std::cout << "process_edge: one or more cells do not have a vertex." << std::endl;
    }
    return;
  }

  const Eigen::Vector4i cells_vec(cell_idx, neighbor_idx_1, neighbor_idx_2, neighbor_idx_3);

  Eigen::Vector4i local_idxs;
  const int idx0 = get_edge_idx_from_grid_vertices(cells[cell_idx], i0, j0, k0, i1, j1, k1);
  const int idx1 = get_edge_idx_from_grid_vertices(cells[neighbor_idx_1], i0, j0, k0, i1, j1, k1);
  const int idx2 = get_edge_idx_from_grid_vertices(cells[neighbor_idx_2], i0, j0, k0, i1, j1, k1);
  const int idx3 = get_edge_idx_from_grid_vertices(cells[neighbor_idx_3], i0, j0, k0, i1, j1, k1);
  local_idxs << idx0, idx1, idx2, idx3;

  edge_cells.push_back(cells_vec);
  edge_local_edge_idxs.push_back(local_idxs);

  edge_ordinal += 1;
}

void average_hermite_normals(std::vector<Cell> &cells, int resX, int resY, int resZ, bool verbose)
{
  Eigen::MatrixXd V_global;
  int next_vertex_index = 0;
  Eigen::MatrixXi F_global;
  int next_face_index = 0;

  /* Collections to record which 4 cells correspond to each processed edge and the
   * local edge index within each of those cells (or -1 if missing). */
  std::vector<Eigen::Vector4i> edge_cells;
  std::vector<Eigen::Vector4i> edge_local_edge_idxs;
  int edge_ordinal = 0; /* ordinal index for each processed edge */

  /* NOTE: the original ran this loop under `#pragma omp parallel for collapse(3)` even
   * though process_edge() appends to the shared edge_cells/edge_local_edge_idxs vectors
   * (a data race). It is kept serial here to preserve well-defined behavior. */
  for (int k = 0; k < resZ; ++k) {
    for (int j = 0; j < resY; ++j) {
      for (int i = 0; i < resX; ++i) {

        if (i < resX - 1) {
          /* Edge: (i,j,k) -> (i+1,j,k). Get the cell containing this edge. */
          process_edge(true,
                       cells,
                       i,
                       j,
                       k,
                       i + 1,
                       j,
                       k,
                       resX,
                       resY,
                       resZ,
                       V_global,
                       next_vertex_index,
                       F_global,
                       next_face_index,
                       edge_cells,
                       edge_local_edge_idxs,
                       edge_ordinal,
                       verbose);
        }

        if (j < resY - 1) {
          /* Edge: (i,j,k) -> (i,j+1,k). */
          process_edge(true,
                       cells,
                       i,
                       j,
                       k,
                       i,
                       j + 1,
                       k,
                       resX,
                       resY,
                       resZ,
                       V_global,
                       next_vertex_index,
                       F_global,
                       next_face_index,
                       edge_cells,
                       edge_local_edge_idxs,
                       edge_ordinal,
                       verbose);
        }

        if (k < resZ - 1) {
          /* Edge: (i,j,k) -> (i,j,k+1). */
          process_edge(true,
                       cells,
                       i,
                       j,
                       k,
                       i,
                       j,
                       k + 1,
                       resX,
                       resY,
                       resZ,
                       V_global,
                       next_vertex_index,
                       F_global,
                       next_face_index,
                       edge_cells,
                       edge_local_edge_idxs,
                       edge_ordinal,
                       verbose);
        }
      }
    }
  }

  if (verbose) {
    std::cout << "edge_cells.size(): " << edge_cells.size() << "\n";
  }

  /* For each cell in edge_cells, accumulate normals from all edges it participates in and
   * average. Each global edge owns a distinct (cell, local edge) pair per entry, so the
   * per-iteration writes touch disjoint map entries (as under the original omp loop). */
  threading::parallel_for(IndexRange(int64_t(edge_cells.size())), 1024, [&](const IndexRange range) {
    for (const int64_t ei : range) {
      const Eigen::Vector4i cell_indices = edge_cells[ei];
      const Eigen::Vector4i local_edge_indices = edge_local_edge_idxs[ei];
      Eigen::Vector3d accumulated_normal = Eigen::Vector3d::Zero();
      int normal_contributions = 0;
      for (int ci = 0; ci < 4; ++ci) {
        const int cell_idx = cell_indices(ci);
        const int local_edge_idx = local_edge_indices(ci);
        if (cell_idx >= 0 && local_edge_idx >= 0) {
          Cell &cell = cells[cell_idx];
          const Eigen::Vector3d hermite_normal = cell.hermite_normals[local_edge_idx];
          if (hermite_normal.norm() > 0) {
            accumulated_normal += hermite_normal;
            normal_contributions += 1;
          }
        }
      }
      accumulated_normal.normalize();
      /* Now write back the averaged normal to each cell's corresponding edge normal. */
      for (int ci = 0; ci < 4; ++ci) {
        const int cell_idx = cell_indices(ci);
        const int local_edge_idx = local_edge_indices(ci);
        if (cell_idx >= 0 && local_edge_idx >= 0) {
          Cell &cell = cells[cell_idx];
          cell.hermite_normals[local_edge_idx] = accumulated_normal;
        }
      }
    }
  });
}

void update_hermite_points_and_normals(std::vector<Cell> &cells,
                                       int resX,
                                       int resY,
                                       int resZ,
                                       double hermite_normal_weight,
                                       double hermite_point_weight,
                                       bool update_points_too,
                                       bool verbose)
{
  Eigen::MatrixXd V_global;
  int next_vertex_index = 0;
  Eigen::MatrixXi F_global;
  int next_face_index = 0;

  /* Collections to record which 4 cells correspond to each processed edge and the
   * local edge index within each of those cells (or -1 if missing). */
  std::vector<Eigen::Vector4i> edge_cells;
  std::vector<Eigen::Vector4i> edge_local_edge_idxs;
  int edge_ordinal = 0; /* ordinal index for each processed edge */

  for (int k = 0; k < resZ; ++k) {
    for (int j = 0; j < resY; ++j) {
      for (int i = 0; i < resX; ++i) {

        if (i < resX - 1) {
          /* Edge: (i,j,k) -> (i+1,j,k). Get the cell containing this edge. */
          process_edge(false,
                       cells,
                       i,
                       j,
                       k,
                       i + 1,
                       j,
                       k,
                       resX,
                       resY,
                       resZ,
                       V_global,
                       next_vertex_index,
                       F_global,
                       next_face_index,
                       edge_cells,
                       edge_local_edge_idxs,
                       edge_ordinal,
                       verbose);
        }

        if (j < resY - 1) {
          /* Edge: (i,j,k) -> (i,j+1,k). */
          process_edge(false,
                       cells,
                       i,
                       j,
                       k,
                       i,
                       j + 1,
                       k,
                       resX,
                       resY,
                       resZ,
                       V_global,
                       next_vertex_index,
                       F_global,
                       next_face_index,
                       edge_cells,
                       edge_local_edge_idxs,
                       edge_ordinal,
                       verbose);
        }

        if (k < resZ - 1) {
          /* Edge: (i,j,k) -> (i,j,k+1). */
          process_edge(false,
                       cells,
                       i,
                       j,
                       k,
                       i,
                       j,
                       k + 1,
                       resX,
                       resY,
                       resZ,
                       V_global,
                       next_vertex_index,
                       F_global,
                       next_face_index,
                       edge_cells,
                       edge_local_edge_idxs,
                       edge_ordinal,
                       verbose);
        }
      }
    }
  }

  for (int global_edge_ind = 0; global_edge_ind < (int)edge_cells.size(); ++global_edge_ind) {
    const Eigen::Vector4i cell_indices = edge_cells[global_edge_ind];
    const Eigen::Vector4i local_edge_indices = edge_local_edge_idxs[global_edge_ind];

    Eigen::MatrixXd all_verts;
    all_verts.resize(4, 3);
    for (int ci = 0; ci < 4; ++ci) {
      const int cell_idx = cell_indices(ci);
      Cell &cell = cells[cell_idx];
      all_verts.row(ci) = cell.vertex;
    }

    /* Use PCA to estimate normal.
     * 1. Calculate the Centroid (Mean). */
    const Eigen::RowVector3d centroid = all_verts.colwise().mean();

    /* 2. Center the data: replicate the centroid row 4 times and subtract it from all_verts. */
    const Eigen::MatrixXd centered_data = all_verts.rowwise() - centroid;

    /* 3. Perform SVD on the centered data.
     * We are interested in the V matrix (right singular vectors).
     * ComputeThinV means we only compute the necessary V components (3x3). */
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered_data,
                                                Eigen::ComputeThinV /* Only need V */);

    /* The V matrix contains the principal components as its columns.
     * The columns are ordered by decreasing singular value.
     * The third column (index 2) is the one corresponding to the smallest variance. */
    const Eigen::Matrix3d V = svd.matrixV();

    /* The normal vector is the last column of V. */
    Eigen::Vector3d normal_vector = V.col(2);
    normal_vector.normalize();

    /* Check sign. */
    Cell &cell0 = cells[cell_indices(0)];
    const int local_edge_idx0 = local_edge_indices(0);
    const Eigen::Vector3d pA = cell0.corners.row(EDGE_PAIRS[local_edge_idx0][0]);
    const double sA = cell0.cornerSDF(EDGE_PAIRS[local_edge_idx0][0]);
    const Eigen::Vector3d pB = cell0.corners.row(EDGE_PAIRS[local_edge_idx0][1]);
    const Eigen::Vector3d outward_edge_dir = (pB - pA).normalized() * (sA < 0 ? -1.0 : 1.0);
    if (normal_vector.dot(outward_edge_dir) > 0) {
      normal_vector = -normal_vector;
    }

    /* Assign this normal to all 4 cells' corresponding edge normals with weighting. */
    for (int ci = 0; ci < 4; ++ci) {
      const int cell_idx = cell_indices(ci);
      const int local_edge_idx = local_edge_indices(ci);
      Cell &cell = cells[cell_idx];
      const Eigen::Vector3d &existing_normal = cell.hermite_normals[local_edge_idx];
      if (normal_vector.norm() > 0) {
        Eigen::Vector3d updated_normal = (1.0 - hermite_normal_weight) * existing_normal +
                                         hermite_normal_weight * normal_vector;
        updated_normal.normalize();
        cell.hermite_normals[local_edge_idx] = updated_normal;
      }
      else {
        cell.hermite_normals[local_edge_idx] = existing_normal;
      }
    }

    if (update_points_too) {
      /* Find the intersection between the edge and the best fit plane from PCA. */
      const Eigen::Vector3d plane_normal = normal_vector;
      const Eigen::Vector3d plane_point = centroid.transpose();

      const Eigen::Vector3d edge_dir = (pB - pA).normalized();
      const double denom = plane_normal.dot(edge_dir);
      if (std::abs(denom) > 1e-6) { /* not parallel */
        double t = plane_normal.dot(plane_point - pA) / denom;
        /* Clamp t to [0, length of edge]. */
        const double edge_length = (pB - pA).norm();
        if (t < 0 || t > edge_length) {
          continue;
        }
        t = std::max(0.0, std::min(t, edge_length));
        const Eigen::Vector3d intersection_point = pA + t * edge_dir;
        /* Assign this intersection point to all 4 cells' corresponding edge hermite positions. */
        for (int ci = 0; ci < 4; ++ci) {
          const int cell_idx = cell_indices(ci);
          const int local_edge_idx = local_edge_indices(ci);
          Cell &cell = cells[cell_idx];
          const Eigen::Vector3d &existing_point = cell.hermite_positions[local_edge_idx];
          cell.hermite_positions[local_edge_idx] =
              (1.0 - hermite_point_weight) * existing_point +
              hermite_point_weight * intersection_point;
        }
      }
    }
  }
}

}  // namespace blender::sdf::meshing
