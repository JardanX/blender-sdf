/* SPDX-FileCopyrightText: 2025 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* SDF math primitives for Cycles kernel.
 * Ported from the draw engine's sdf_lib.glsl. */

#pragma once

CCL_NAMESPACE_BEGIN

/* -------------------------------------------------------------------- */
/* Trilinear Interpolation Coefficients. */

/* Compute trilinear polynomial coefficients from 8 corner SDF values.
 *
 * f(x,y,z) = k[0] + k[1]*x + k[2]*y + k[3]*z
 *           + k[4]*x*y + k[5]*x*z + k[6]*y*z + k[7]*x*y*z
 *
 * Corner ordering: s[0]=s000, s[1]=s100, s[2]=s010, s[3]=s110,
 *                  s[4]=s001, s[5]=s101, s[6]=s011, s[7]=s111 */
ccl_device_inline void sdf_trilinear_coeffs(const float s[8], float k[8])
{
  k[0] = s[0];
  k[1] = s[1] - s[0];
  k[2] = s[2] - s[0];
  k[3] = s[4] - s[0];
  k[4] = s[3] - s[1] - s[2] + s[0];
  k[5] = s[5] - s[1] - s[4] + s[0];
  k[6] = s[6] - s[2] - s[4] + s[0];
  k[7] = s[7] - s[6] - s[5] - s[3] + s[4] + s[2] + s[1] - s[0];
}

/* -------------------------------------------------------------------- */
/* Ray-Cubic Polynomial. */

/* Compute cubic polynomial coefficients for ray-trilinear intersection.
 * Substitutes ray equation p(t) = o + t*d into trilinear polynomial.
 * Result: f(t) = c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3 */
ccl_device_inline void sdf_cubic_coeffs(const float k[8],
                                         const float3 o,
                                         const float3 d,
                                         float c[4])
{
  c[0] = k[0] + k[1] * o.x + k[2] * o.y + k[3] * o.z + k[4] * o.x * o.y +
         k[5] * o.x * o.z + k[6] * o.y * o.z + k[7] * o.x * o.y * o.z;

  c[1] = k[1] * d.x + k[2] * d.y + k[3] * d.z +
         k[4] * (o.x * d.y + o.y * d.x) + k[5] * (o.x * d.z + o.z * d.x) +
         k[6] * (o.y * d.z + o.z * d.y) +
         k[7] * (o.x * o.y * d.z + o.x * o.z * d.y + o.y * o.z * d.x);

  c[2] = k[4] * d.x * d.y + k[5] * d.x * d.z + k[6] * d.y * d.z +
         k[7] * (o.x * d.y * d.z + o.y * d.x * d.z + o.z * d.x * d.y);

  c[3] = k[7] * d.x * d.y * d.z;
}

/* Evaluate cubic polynomial at parameter t (Horner's method). */
ccl_device_inline float sdf_eval_cubic(const float c[4], const float t)
{
  return c[0] + t * (c[1] + t * (c[2] + t * c[3]));
}

/* Evaluate cubic polynomial derivative. */
ccl_device_inline float sdf_eval_cubic_deriv(const float c[4], const float t)
{
  return c[1] + t * (2.0f * c[2] + t * 3.0f * c[3]);
}

/* -------------------------------------------------------------------- */
/* Marmitt + Newton-Raphson Cubic Solver. */

/* Splits [0, tfar] into monotone intervals using g'(t) = 0 roots,
 * then uses Newton-Raphson refinement in subintervals with sign changes.
 * Returns smallest root in [0, tfar], or -1.0 if no root found. */
ccl_device float sdf_solve_cubic(const float c[4], const float tfar)
{
  /* Find critical points: g'(t) = c[1] + 2*c[2]*t + 3*c[3]*t^2 = 0 */
  const float da = 3.0f * c[3];
  const float db = 2.0f * c[2];
  const float dc = c[1];

  /* Build sorted interval boundaries. */
  float bounds[4];
  int n = 0;
  bounds[n++] = 0.0f;

  if (fabsf(da) > 1e-8f) {
    float disc = db * db - 4.0f * da * dc;
    if (disc >= 0.0f) {
      disc = sqrtf(disc);
      float t1 = (-db - disc) / (2.0f * da);
      float t2 = (-db + disc) / (2.0f * da);
      if (t1 > t2) {
        float tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 1e-6f && t1 < tfar - 1e-6f) {
        bounds[n++] = t1;
      }
      if (t2 > 1e-6f && t2 < tfar - 1e-6f && t2 - t1 > 1e-6f) {
        bounds[n++] = t2;
      }
    }
  }
  else if (fabsf(db) > 1e-8f) {
    float t1 = -dc / db;
    if (t1 > 1e-6f && t1 < tfar - 1e-6f) {
      bounds[n++] = t1;
    }
  }

  bounds[n++] = tfar;

  /* Check each monotone interval for a sign change. */
  float fa = sdf_eval_cubic(c, bounds[0]);
  for (int i = 0; i < n - 1; i++) {
    float fb = sdf_eval_cubic(c, bounds[i + 1]);
    if (fa * fb <= 0.0f) {
      /* Sign change: Newton-Raphson from midpoint. */
      float t = (bounds[i] + bounds[i + 1]) * 0.5f;
      for (int j = 0; j < 6; j++) {
        float f = sdf_eval_cubic(c, t);
        float fp = sdf_eval_cubic_deriv(c, t);
        if (fabsf(fp) < 1e-12f) {
          break;
        }
        t -= f / fp;
        t = clamp(t, bounds[i], bounds[i + 1]);
      }
      if (t >= 0.0f && t <= tfar) {
        return t;
      }
    }
    fa = fb;
  }

  return -1.0f;
}

/* Shadow ray optimization: detect root existence without numeric refinement.
 * Based on the shadow ray optimization from "Ray Tracing of SDF Grids"
 * (Hansson-Soderlund et al., JCGT 2022, Section 2):
 * If a monotone subinterval has a sign change, a root exists — skip NR. */
ccl_device bool sdf_has_cubic_root(const float c[4], const float tfar)
{
  /* Find critical points: g'(t) = c[1] + 2*c[2]*t + 3*c[3]*t^2 = 0 */
  const float da = 3.0f * c[3];
  const float db = 2.0f * c[2];
  const float dc = c[1];

  /* Build sorted interval boundaries. */
  float bounds[4];
  int n = 0;
  bounds[n++] = 0.0f;

  if (fabsf(da) > 1e-8f) {
    float disc = db * db - 4.0f * da * dc;
    if (disc >= 0.0f) {
      disc = sqrtf(disc);
      float t1 = (-db - disc) / (2.0f * da);
      float t2 = (-db + disc) / (2.0f * da);
      if (t1 > t2) {
        float tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 1e-6f && t1 < tfar - 1e-6f) {
        bounds[n++] = t1;
      }
      if (t2 > 1e-6f && t2 < tfar - 1e-6f && t2 - t1 > 1e-6f) {
        bounds[n++] = t2;
      }
    }
  }
  else if (fabsf(db) > 1e-8f) {
    float t1 = -dc / db;
    if (t1 > 1e-6f && t1 < tfar - 1e-6f) {
      bounds[n++] = t1;
    }
  }

  bounds[n++] = tfar;

  /* Check each monotone interval for a sign change. */
  float fa = sdf_eval_cubic(c, bounds[0]);
  for (int i = 0; i < n - 1; i++) {
    float fb = sdf_eval_cubic(c, bounds[i + 1]);
    if (fa * fb <= 0.0f) {
      return true; /* Root exists — no refinement needed for shadow rays. */
    }
    fa = fb;
  }

  return false;
}

/* -------------------------------------------------------------------- */
/* Trilinear Gradient (for normals). */

/* Compute the gradient of trilinear interpolation at point p.
 * p is typically in [0,1]^3 but may extend outside for dual voxel evaluation. */
ccl_device_inline float3 sdf_trilinear_gradient(const float s[8], const float3 p)
{
  const float k1 = s[1] - s[0];
  const float k2 = s[2] - s[0];
  const float k3 = s[4] - s[0];
  const float k4 = s[3] - s[1] - s[2] + s[0];
  const float k5 = s[5] - s[1] - s[4] + s[0];
  const float k6 = s[6] - s[2] - s[4] + s[0];
  const float k7 = s[7] - s[6] - s[5] - s[3] + s[4] + s[2] + s[1] - s[0];

  return make_float3(k1 + k4 * p.y + k5 * p.z + k7 * p.y * p.z,
                     k2 + k4 * p.x + k6 * p.z + k7 * p.x * p.z,
                     k3 + k5 * p.x + k6 * p.y + k7 * p.x * p.y);
}

/* -------------------------------------------------------------------- */
/* Dual Voxel Normal (Section 3.2, Hansson-Soderlund et al. JCGT 2022). */

/* Compute smooth normals by blending analytic trilinear gradients from
 * the 2x2x2 voxels overlapping the dual voxel that contains the hit point.
 *
 * vals[27]: pre-fetched 3x3x3 neighborhood of SDF values, indexed as
 *   vals[k*9 + j*3 + i] where (i,j,k) ∈ {0,1,2}^3.
 * grid_pos: hit point in brick-local grid space (continuous coordinates).
 * dc: dual voxel center = floor(grid_pos + 0.5).
 *
 * Each of the 8 overlapping voxels has its analytic gradient evaluated at the
 * hit point (which may lie outside [0,1]^3 for 7 of 8 voxels), normalized,
 * then trilinearly blended using the hit point's position within the dual voxel.
 * This yields C0-continuous normals across voxel boundaries. */
ccl_device_inline float3 sdf_dual_voxel_normal(const float vals[27],
                                                const float3 grid_pos,
                                                const int3 dc)
{
  /* Position within the dual voxel [0,1]^3. */
  const float3 uvw = make_float3(grid_pos.x + 0.5f - float(dc.x),
                                  grid_pos.y + 0.5f - float(dc.y),
                                  grid_pos.z + 0.5f - float(dc.z));

  float3 blended = zero_float3();

  for (int kk = 0; kk < 2; kk++) {
    for (int jj = 0; jj < 2; jj++) {
      for (int ii = 0; ii < 2; ii++) {
        /* Extract 8 corners for voxel (ii,jj,kk) from the 3x3x3 grid. */
        float s[8];
        s[0] = vals[(kk) * 9 + (jj) * 3 + (ii)];
        s[1] = vals[(kk) * 9 + (jj) * 3 + (ii + 1)];
        s[2] = vals[(kk) * 9 + (jj + 1) * 3 + (ii)];
        s[3] = vals[(kk) * 9 + (jj + 1) * 3 + (ii + 1)];
        s[4] = vals[(kk + 1) * 9 + (jj) * 3 + (ii)];
        s[5] = vals[(kk + 1) * 9 + (jj) * 3 + (ii + 1)];
        s[6] = vals[(kk + 1) * 9 + (jj + 1) * 3 + (ii)];
        s[7] = vals[(kk + 1) * 9 + (jj + 1) * 3 + (ii + 1)];

        /* Hit point in this voxel's local space.
         * May be outside [0,1]^3 for 7 of 8 voxels — that's intended. */
        float3 local_p = make_float3(grid_pos.x - float(dc.x - 1 + ii),
                                      grid_pos.y - float(dc.y - 1 + jj),
                                      grid_pos.z - float(dc.z - 1 + kk));

        float3 grad = sdf_trilinear_gradient(s, local_p);
        float l = len(grad);
        float3 n = (l > 1e-8f) ? grad / l : make_float3(0.0f, 0.0f, 1.0f);

        /* Trilinear blend weight. */
        float w = ((ii == 0) ? (1.0f - uvw.x) : uvw.x) *
                  ((jj == 0) ? (1.0f - uvw.y) : uvw.y) *
                  ((kk == 0) ? (1.0f - uvw.z) : uvw.z);

        blended = blended + n * w;
      }
    }
  }

  float l = len(blended);
  return (l > 1e-8f) ? blended / l : make_float3(0.0f, 0.0f, 1.0f);
}

CCL_NAMESPACE_END
