/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Xiana Carrera and contributors.
 * Ported from https://github.com/xianacarrera/dual-contouring-of-signed-distance-data
 * Paper: "Dual Contouring of Signed Distance Data", Carrera, Wang, Batty, Stein,
 * Sellán, SIGGRAPH 2026. */

#pragma once

#include <utility>
#include <vector>

#include <Eigen/Core>

#include "dcsdd_cell.hh"

namespace blender::sdf::meshing {

bool compute_local_mesh(Cell &cell,
                        Cell &adj_cell1,
                        Cell &adj_cell2,
                        Cell &adj_cell3,
                        int edge_idx,
                        std::pair<int, int> int_dirs,
                        Eigen::MatrixXd &V,
                        int &next_vertex_index,
                        Eigen::MatrixXi &F,
                        int &next_face_index);

void average_hermite_normals(std::vector<Cell> &cells,
                             int resX,
                             int resY,
                             int resZ,
                             bool verbose);

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
                  bool verbose);

void update_hermite_points_and_normals(std::vector<Cell> &cells,
                                       int resX,
                                       int resY,
                                       int resZ,
                                       double hermite_normal_weight,
                                       double hermite_point_weight,
                                       bool update_points_too,
                                       bool verbose);

}  // namespace blender::sdf::meshing
