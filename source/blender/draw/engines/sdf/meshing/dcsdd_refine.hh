/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Xiana Carrera and contributors.
 * Ported from https://github.com/xianacarrera/dual-contouring-of-signed-distance-data
 * Paper: "Dual Contouring of Signed Distance Data", Carrera, Wang, Batty, Stein,
 * Sellán, SIGGRAPH 2026. */

#pragma once

#include <Eigen/Core>

#include "dcsdd_cell.hh"

namespace blender::sdf::meshing {

void refine_vertex_from_face_intersections(Cell &cell);

void closest_points_on_mesh(const Eigen::MatrixXd &V_mesh,
                            const Eigen::MatrixXi &F_mesh,
                            const Eigen::MatrixXd &targets,      /* Nx3 query points */
                            Eigen::MatrixXd &closest_points_out, /* Nx3 closest points */
                            Eigen::VectorXi &face_indices_out);  /* N face indices */

Eigen::Vector3d compute_barycentric_coords(const Eigen::Vector3d &P,
                                           const Eigen::Vector3d &V0,
                                           const Eigen::Vector3d &V1,
                                           const Eigen::Vector3d &V2);

}  // namespace blender::sdf::meshing
