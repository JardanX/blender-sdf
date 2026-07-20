/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Xiana Carrera and contributors.
 * Ported from https://github.com/xianacarrera/dual-contouring-of-signed-distance-data
 * Paper: "Dual Contouring of Signed Distance Data", Carrera, Wang, Batty, Stein,
 * Sellán, SIGGRAPH 2026. */

#include "dcsdd_sdf.hh"

#include <algorithm>
#include <cmath>

namespace blender::sdf::meshing {

SDFGrad finite_difference_gradient(const SDF &f, double h)
{
  return [f, h](const Eigen::RowVector3d &x) -> Eigen::RowVector3d {
    Eigen::RowVector3d g = Eigen::RowVector3d::Zero();
    const double eps = (h > 0.0) ? h : 1e-6;

    for (int c = 0; c < 3; ++c) {
      Eigen::RowVector3d e = Eigen::RowVector3d::Zero();
      e(c) = eps;
      const double fp = f(x + e);
      const double fn = f(x - e);
      g(c) = (fp - fn) / (2.0 * eps);
    }
    const double nrm = g.norm();
    if (nrm > 0.0) {
      g /= nrm;
    }
    return g;
  };
}

SDFObject make_rotated_box_sdf(const Eigen::RowVector3d &b, const Eigen::Matrix3d &R, double fd_h)
{
  SDFObject out;

  /* Analytic SDF for rotated axis-aligned box: evaluate box SDF in local frame. */
  out.f = [b, R](const Eigen::RowVector3d &x_world) -> double {
    const Eigen::Vector3d x = x_world.transpose();
    const Eigen::Vector3d x_local = R.transpose() * x;
    const Eigen::RowVector3d xl = x_local.transpose();
    const Eigen::RowVector3d q = xl.cwiseAbs() - b;
    const Eigen::RowVector3d q_pos = q.cwiseMax(0.0);

    const double outside_dist = q_pos.norm();
    const double inside_dist = std::min(std::max(q.x(), std::max(q.y(), q.z())), 0.0);
    return outside_dist + inside_dist;
  };

  /* Gradient: use FD with a small h; caller may override fd_h=-1 to pick default later. */
  out.grad = finite_difference_gradient(out.f, fd_h > 0.0 ? fd_h : 1e-6);

  return out;
}

/* Overloaded version with center parameter. */
SDFObject make_rotated_box_sdf(const Eigen::RowVector3d &b,
                               const Eigen::RowVector3d &center,
                               const Eigen::Matrix3d &R,
                               double fd_h)
{
  SDFObject out;

  /* Account for center in SDF. */
  out.f = [b, center, R](const Eigen::RowVector3d &x_world) -> double {
    /* Shift to box center. */
    const Eigen::Vector3d x_translated = (x_world - center).transpose();

    /* Rotate to local frame. */
    const Eigen::Vector3d x_local = R.transpose() * x_translated;
    const Eigen::RowVector3d xl = x_local.transpose();

    const Eigen::RowVector3d q = xl.cwiseAbs() - b;
    const Eigen::RowVector3d q_pos = q.cwiseMax(0.0);

    const double outside_dist = q_pos.norm();
    const double inside_dist = std::min(std::max(q.x(), std::max(q.y(), q.z())), 0.0);
    return outside_dist + inside_dist;
  };

  out.grad = finite_difference_gradient(out.f, fd_h > 0.0 ? fd_h : 1e-6);

  return out;
}

SDFObject make_torus_sdf(double ra, double rb, double fd_h)
{
  SDFObject torus_obj;

  torus_obj.f = [ra, rb](const Eigen::RowVector3d &p) -> double {
    /* Reference: https://iquilezles.org/articles/distgradfunctions3d/ */
    const double h = std::sqrt(p(0) * p(0) + p(2) * p(2));
    return std::sqrt((h - ra) * (h - ra) + p(1) * p(1)) - rb;
  };

  torus_obj.grad = finite_difference_gradient(torus_obj.f, fd_h > 0.0 ? fd_h : 1e-6);

  return torus_obj;
}

SDFObject make_sphere_sdf(double r, const Eigen::RowVector3d &c, double fd_h)
{
  SDFObject sphere_obj;

  sphere_obj.f = [r, c](const Eigen::RowVector3d &p) -> double {
    return (p - c).norm() - r;
  };

  sphere_obj.grad = finite_difference_gradient(sphere_obj.f, fd_h > 0.0 ? fd_h : 1e-6);

  return sphere_obj;
}

SDFObject make_cylinder_sdf(double he, double r, double fd_h)
{
  /* Reference: https://iquilezles.org/articles/distgradfunctions3d/ */

  SDFObject cylinder_obj;

  cylinder_obj.f = [r, he](const Eigen::RowVector3d &p) -> double {
    /* Distance from the Y-axis (radius in XZ plane). */
    const double l = std::sqrt(p(0) * p(0) + p(2) * p(2));

    /* e.x: distance to the infinite cylinder wall (l - r)
     * e.y: distance to the end caps (abs(p.y) - he / 2.0) */
    const double half_he = he / 2.0;

    /* Distance when inside the cylinder. */
    const double g = std::max(l - r, std::abs(p(1)) - half_he);

    /* Amount outside along each axis. */
    Eigen::Vector2d h;
    h(0) = std::max(l - r, 0.0);
    h(1) = std::max(std::abs(p(1)) - half_he, 0.0);

    /* Distance when outside the cylinder. */
    const double f = std::sqrt(h(0) * h(0) + h(1) * h(1));

    /* g if inside, f if outside. */
    return (g <= 0.0) ? g : f;
  };

  cylinder_obj.grad = finite_difference_gradient(cylinder_obj.f, fd_h > 0.0 ? fd_h : 1e-6);

  return cylinder_obj;
}

SDFObject make_triangular_prism_sdf(const Eigen::RowVector2d &half_extents,
                                    const Eigen::Matrix3d &R,
                                    double fd_h)
{
  SDFObject prism_obj;

  prism_obj.f = [half_extents, R](const Eigen::RowVector3d &p_world) -> double {
    const Eigen::RowVector3d p = p_world * R.transpose(); /* Rotate point into prism local frame. */

    /* Constants for 30 degrees (equilateral triangle). */
    const double cos30 = 0.866025; /* std::sqrt(3.0) / 2.0 */
    const double sin30 = 0.5;      /* 1.0 / 2.0 */

    const Eigen::RowVector3d q = p.cwiseAbs();

    const double d1 = q(2) - half_extents(1); /* Distance from end caps along Z. */
    const double d2 = std::max(q(0) * cos30 + p(1) * sin30, -p(1)) -
                      half_extents(0) * sin30; /* Distance from side faces (along XY-plane). */
    return std::max(d1, d2);
  };

  prism_obj.grad = finite_difference_gradient(prism_obj.f, fd_h > 0.0 ? fd_h : 1e-6);
  return prism_obj;
}

SDFObject make_cut_sphere_sdf(double r, double h, double fd_h)
{
  SDFObject sphere_obj;

  sphere_obj.f = [r, h](const Eigen::RowVector3d &p) -> double {
    if (h > r) {
      return p.norm() - r;
    }

    const double w = std::sqrt(r * r - h * h);

    Eigen::Vector2d q;
    q(0) = std::sqrt(p(0) * p(0) + p(2) * p(2));
    q(1) = p(1);

    const double s = std::max((h - r) * q(0) * q(0) + w * w * (h + r - 2.0 * q(1)),
                              h * q(0) - w * q(1));

    if (s < 0.0) {
      return q.norm() - r;
    }
    if (q(0) < w) {
      return h - q(1);
    }
    return (q - Eigen::Vector2d(w, h)).norm();
  };

  const double h_step = (fd_h > 0.0) ? fd_h : 1e-6;

  sphere_obj.grad = finite_difference_gradient(sphere_obj.f, h_step);

  return sphere_obj;
}

static double sdOctahedron(const Eigen::RowVector3d &p_in, double s)
{
  const Eigen::RowVector3d p = p_in.cwiseAbs();

  const double m = p(0) + p(1) + p(2) - s;

  Eigen::RowVector3d q;
  const double m_check = 3.0 * m;

  if (3.0 * p(0) < m_check) {
    q = p;
  }
  else if (3.0 * p(1) < m_check) {
    q << p(1), p(2), p(0);
  }
  else if (3.0 * p(2) < m_check) {
    q << p(2), p(0), p(1);
  }
  else {
    const double inv_sqrt3 = 0.57735026919;
    return m * inv_sqrt3;
  }

  const double k = std::max(0.0, std::min(s, 0.5 * (q(2) - q(1) + s)));

  const double d_x = q(0);
  const double d_y = q(1) - s + k;
  const double d_z = q(2) - k;

  return std::sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
}

SDFObject make_octahedron_sdf(double s, const Eigen::Matrix3d &R, double fd_h)
{
  SDFObject octahedron_obj;

  octahedron_obj.f = [s, R](const Eigen::RowVector3d &p) -> double {
    return sdOctahedron(R * p.transpose(), s);
  };

  const double h_step = (fd_h > 0.0) ? fd_h : 1e-6;

  octahedron_obj.grad = finite_difference_gradient(octahedron_obj.f, h_step);

  return octahedron_obj;
}

}  // namespace blender::sdf::meshing
