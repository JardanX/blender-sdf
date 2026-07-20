/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Xiana Carrera and contributors.
 * Ported from https://github.com/xianacarrera/dual-contouring-of-signed-distance-data
 * Paper: "Dual Contouring of Signed Distance Data", Carrera, Wang, Batty, Stein,
 * Sellán, SIGGRAPH 2026. */

#pragma once

#include <Eigen/Core>

#include "dcsdd_cell.hh"

namespace blender::sdf::meshing {

/* Analytic SDF factories from the original research repo, usable to build the
 * TrueSdfFunc / TrueSdfGradFunc callbacks accepted by contouring(). */
using SDF = TrueSdfFunc;
using SDFGrad = TrueSdfGradFunc;

/* Compute a finite-difference gradient for any SDF using centered differences.
 * h is the finite-difference step (small positive number). */
SDFGrad finite_difference_gradient(const SDF &f, double h);

/* An SDF object bundles an SDF with its gradient. */
struct SDFObject {
  SDF f;
  SDFGrad grad;
};

/* Factory: rotated axis-aligned box SDF (half-extents b, rotation matrix R).
 * Returns {f, f_grad_fd}. f_grad_fd is a normalized finite-difference gradient
 * using a small step derived from provided h (if h <= 0, default small step used). */
SDFObject make_rotated_box_sdf(const Eigen::RowVector3d &half_extents,
                               const Eigen::Matrix3d &R,
                               double fd_h = -1.0);

SDFObject make_rotated_box_sdf(const Eigen::RowVector3d &b,
                               const Eigen::RowVector3d &center,
                               const Eigen::Matrix3d &R,
                               double fd_h = -1.0);

SDFObject make_torus_sdf(double ra, double rb, double fd_h);

SDFObject make_sphere_sdf(double r, const Eigen::RowVector3d &c, double fd_h);

SDFObject make_cylinder_sdf(double he, double r, double fd_h);

SDFObject make_triangular_prism_sdf(const Eigen::RowVector2d &half_extents,
                                    const Eigen::Matrix3d &R,
                                    double fd_h);

SDFObject make_cut_sphere_sdf(double r, double h, double fd_h);

SDFObject make_octahedron_sdf(double s, const Eigen::Matrix3d &R, double fd_h);

}  // namespace blender::sdf::meshing
