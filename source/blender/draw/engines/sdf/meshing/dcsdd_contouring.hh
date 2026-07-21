/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Xiana Carrera and contributors.
 * Ported from https://github.com/xianacarrera/dual-contouring-of-signed-distance-data
 * Paper: "Dual Contouring of Signed Distance Data", Carrera, Wang, Batty, Stein,
 * Sellán, SIGGRAPH 2026. */

#pragma once

#include <functional>
#include <vector>

#include <Eigen/Core>

#include "dcsdd_cell.hh"

namespace blender::sdf::meshing {

inline int index3D(int i, int j, int k, int resX, int resY, int /*resZ*/)
{
  return i + resX * (j + resY * k);
}

inline int cellIndex3D(int i, int j, int k, int resX, int resY, int /*resZ*/)
{
  /* There are (resX-1) x (resY-1) x (resZ-1) cells. */
  return i + (resX - 1) * (j + (resY - 1) * k);
}

/* Function type used to triangulate a face matrix F when needed.
 * Takes the original face matrix and the vertex matrix V and returns a triangle-only face matrix. */
using TriangulateFunc = std::function<Eigen::MatrixXi(const Eigen::MatrixXi &,
                                                      const Eigen::MatrixXd &)>;

/* Public triangulation helpers (choose one when calling assign_spheres_to_cells). */
Eigen::MatrixXi triangulate_v1(const Eigen::MatrixXi &F, const Eigen::MatrixXd &V);
Eigen::MatrixXi triangulate_v2(const Eigen::MatrixXi &F, const Eigen::MatrixXd &V);

/* Contouring method: dual contouring (optionally with the DC-SDD optimization loop).
 * The marching-cubes path of the original (libigl-based) was dropped in the port. */
enum class ContouringMethod {
  DualContouring,
  Ours,
};

/* Options struct — defaults match the paper. */
struct ContouringOptions {
  ContouringMethod method = ContouringMethod::DualContouring;
  bool verbose = false;
  double mu = 0.1;
  double dc_weight = 0.02;
  double sphere_weight = 1.0;
  double svd_threshold = 0.01;
  int outer_iters = 100;
  int inner_iters = 100;
  bool hermite_update = true;
  double new_hermite_pos_weight = 0.2;
  double new_face_pos_weight = 0.2;
  double new_hermite_normal_weight = 0.2;
  int batch_size = 200000;
};

/**
 * Main entry point: Dual Contouring of Signed Distance Data.
 * S holds the SDF values at the resX*resY*resZ grid vertices (float, x-fastest),
 * GV their positions (float, Nx3). The output mesh has double vertices V (Nx3)
 * and quad faces F (Mx4).
 * true_sdf / true_sdf_grad may be empty (the grid-only overload passes nullptrs,
 * in which case Hermite data comes from the grid via finite differences).
 */
void contouring(const Eigen::VectorXf &S,
                const Eigen::MatrixXf &GV,
                int resX,
                int resY,
                int resZ,
                double isoValue,
                Eigen::MatrixXd &V,
                Eigen::MatrixXi &F,
                const ContouringOptions &options,
                const TrueSdfFunc &true_sdf,
                const TrueSdfGradFunc &true_sdf_grad);

/* Overload without true_sdf and true_sdf_grad. */
void contouring(const Eigen::VectorXf &S,
                const Eigen::MatrixXf &GV,
                int resX,
                int resY,
                int resZ,
                double isoValue,
                Eigen::MatrixXd &V,
                Eigen::MatrixXi &F,
                const ContouringOptions &options);

std::vector<Cell> generate_cells(const Eigen::VectorXf &S,
                                 const Eigen::MatrixXf &GV,
                                 int resX,
                                 int resY,
                                 int resZ,
                                 const TrueSdfFunc &true_sdf,
                                 const TrueSdfGradFunc &true_sdf_grad);

Eigen::Vector3d gradientAt(
    const Eigen::MatrixXf &GV, const Eigen::VectorXf &S, int i, int j, int k, int resX, int resY, int resZ);

void assign_spheres_to_cells(const Eigen::VectorXf &S,
                             const Eigen::MatrixXf &GV,
                             const Eigen::MatrixXd &V,
                             const Eigen::MatrixXi &F,
                             std::vector<Cell> &cells,
                             int resX,
                             int resY,
                             int resZ,
                             int batch_size,
                             Eigen::MatrixXi &TriF,
                             const std::pair<std::vector<int>, std::vector<int>> *precomputed =
                                 nullptr);

void compute_face_cell_intersections(std::vector<Cell> &cells,
                                     int resX,
                                     int resY,
                                     int resZ,
                                     double weight_new_pos);

/* Extract mesh vertices & quad faces from cells. GV is the grid vertex positions used for
 * gradient/normals and spatial computations. */
void extract_mesh_from_cells(const std::vector<Cell> &cells,
                             const Eigen::VectorXf &S,
                             const Eigen::MatrixXf &GV,
                             int resX,
                             int resY,
                             int resZ,
                             Eigen::MatrixXd &V,
                             Eigen::MatrixXi &F,
                             std::vector<Eigen::Vector3d> &hermite_normals);

void extract_mesh_from_cells(const std::vector<Cell> &cells,
                             const Eigen::VectorXf &S,
                             const Eigen::MatrixXf &GV,
                             int resX,
                             int resY,
                             int resZ,
                             Eigen::MatrixXd &V,
                             Eigen::MatrixXi &F);

/* Variant that also returns a per-vertex mapping to the originating cell index. */
void extract_mesh_from_cells(const std::vector<Cell> &cells,
                             const Eigen::VectorXf &S,
                             const Eigen::MatrixXf &GV,
                             int resX,
                             int resY,
                             int resZ,
                             Eigen::MatrixXd &V,
                             Eigen::MatrixXi &F,
                             std::vector<int> *vertexCellIndex);

/* Sum the per-cell energies, print under verbose, and return the total. */
double show_total_energy(const std::vector<Cell> &cells, bool verbose);

void optimize_triangulation(const Eigen::VectorXf &S,
                            const Eigen::MatrixXf &GV,
                            const Eigen::MatrixXd &V,
                            const Eigen::MatrixXi &F,
                            Eigen::MatrixXi &TriF,
                            int resX,
                            int resY,
                            int resZ);

/* Triangulate F choosing the diagonal that is the most orthogonal to the
 * corresponding hermite normal. */
void triangulate_based_on_hermite_normal(const Eigen::MatrixXd &V,
                                         const Eigen::MatrixXi &F,
                                         Eigen::MatrixXi &TriF,
                                         const std::vector<Eigen::Vector3d> &hermite_normals);

void orient_triangles_for_quad(Eigen::MatrixXi &TriF,
                               int tri_row0,
                               int tri_row1,
                               int a,
                               int b,
                               int c,
                               int d,
                               const Eigen::MatrixXd &V);

double compute_total_distance_to_spheres(const Eigen::VectorXf &S,
                                         const Eigen::MatrixXf &GV,
                                         const Eigen::MatrixXd &V,
                                         const Eigen::MatrixXi &TriF,
                                         int resX,
                                         int resY,
                                         int resZ);

double cell_diagonal(const Eigen::MatrixXf &GV,
                     int resX,
                     int resY,
                     int resZ,
                     double &min_x,
                     double &min_y,
                     double &min_z,
                     double &dx,
                     double &dy,
                     double &dz);

}  // namespace blender::sdf::meshing
