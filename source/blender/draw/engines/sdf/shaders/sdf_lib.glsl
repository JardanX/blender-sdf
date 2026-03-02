/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF primitives and analytic voxel intersection library.
 *
 * Provides:
 * - sdBox, opSmoothUnion: core SDF primitives
 * - Trilinear coefficient computation (Eq. 3 from Hansson-Soderlund et al. 2022)
 * - Ray-cubic coefficient computation (Eq. 6-7)
 * - Analytic cubic solver (Vieta's trigonometric method)
 * - Marmitt + Newton-Raphson fallback solver
 * - Analytical trilinear gradient for normals (Eq. 9-11)
 */

#pragma once

#include "gpu_shader_compat.hh"

#define SDF_PI 3.14159265358979323846f

/* -------------------------------------------------------------------- */
/** \name SDF Primitives
 * \{ */

/**
 * Signed distance to an axis-aligned box centered at origin.
 * \param p: sample point in local space.
 * \param b: box half-extents.
 * \return signed distance (negative inside).
 */
float sdBox(float3 p, float3 b)
{
  float3 q = abs(p) - b;
  return length(max(q, float3(0.0f))) + min(max(q.x, max(q.y, q.z)), 0.0f);
}

/**
 * Smooth union (Inigo Quilez pattern).
 * Blends two SDF distances with smooth radius k.
 * \param d1: first distance.
 * \param d2: second distance.
 * \param k: blend radius (0 = hard union).
 * \return blended distance.
 */
float opSmoothUnion(float d1, float d2, float k)
{
  float h = clamp(0.5f + 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
  return mix(d2, d1, h) - k * h * (1.0f - h);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Trilinear Interpolation Coefficients
 * \{ */

/**
 * Compute trilinear polynomial coefficients from 8 corner SDF values.
 *
 * f(x,y,z) = k[0] + k[1]*x + k[2]*y + k[3]*z
 *           + k[4]*x*y + k[5]*x*z + k[6]*y*z + k[7]*x*y*z
 *
 * Corner ordering: s[0]=s000, s[1]=s100, s[2]=s010, s[3]=s110,
 *                  s[4]=s001, s[5]=s101, s[6]=s011, s[7]=s111
 */
void computeTrilinearCoeffs(float s[8], out float k[8])
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

/** \} */

/* -------------------------------------------------------------------- */
/** \name Ray-Cubic Polynomial
 * \{ */

/**
 * Compute cubic polynomial coefficients for ray-trilinear intersection.
 * Substitutes ray equation p(t) = o + t*d into trilinear polynomial.
 * Result: f(t) = c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3
 *
 * \param k: trilinear coefficients from computeTrilinearCoeffs().
 * \param o: ray origin in [0,1]^3 voxel-local space.
 * \param d: ray direction in grid space.
 */
void computeCubicCoeffs(float k[8], float3 o, float3 d, out float c[4])
{
  c[0] = k[0] + k[1] * o.x + k[2] * o.y + k[3] * o.z
       + k[4] * o.x * o.y + k[5] * o.x * o.z + k[6] * o.y * o.z
       + k[7] * o.x * o.y * o.z;

  c[1] = k[1] * d.x + k[2] * d.y + k[3] * d.z
       + k[4] * (o.x * d.y + o.y * d.x)
       + k[5] * (o.x * d.z + o.z * d.x)
       + k[6] * (o.y * d.z + o.z * d.y)
       + k[7] * (o.x * o.y * d.z + o.x * o.z * d.y + o.y * o.z * d.x);

  c[2] = k[4] * d.x * d.y + k[5] * d.x * d.z + k[6] * d.y * d.z
       + k[7] * (o.x * d.y * d.z + o.y * d.x * d.z + o.z * d.x * d.y);

  c[3] = k[7] * d.x * d.y * d.z;
}

/**
 * Evaluate cubic polynomial at parameter t (Horner's method).
 */
float evalCubic(float c[4], float t)
{
  return c[0] + t * (c[1] + t * (c[2] + t * c[3]));
}

/**
 * Evaluate cubic polynomial derivative at parameter t.
 * g'(t) = c[1] + 2*c[2]*t + 3*c[3]*t^2
 */
float evalCubicDeriv(float c[4], float t)
{
  return c[1] + t * (2.0f * c[2] + t * 3.0f * c[3]);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Cubic Solvers
 * \{ */

/** Safe cube root that handles negative values. */
float sdf_cbrt(float x)
{
  return sign(x) * pow(abs(x), 1.0f / 3.0f);
}

/**
 * Analytic cubic solver using Vieta's trigonometric method.
 * Finds the smallest real root of c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3 = 0
 * in the interval [0, tfar].
 *
 * Handles degenerate cases: c3~0 -> quadratic, c3~0 && c2~0 -> linear.
 * For the three-real-roots case (discriminant <= 0), uses the trigonometric
 * solution to avoid complex intermediate values.
 *
 * \return smallest root in [0, tfar], or -1.0 if no root found.
 */
float solveCubicFirstRoot(float c[4], float tfar)
{
  float c0 = c[0], c1 = c[1], c2 = c[2], c3 = c[3];

  /* Degenerate: c3 ~ 0 -> solve as quadratic or linear. */
  if (abs(c3) < 1e-7f) {
    if (abs(c2) < 1e-7f) {
      /* Linear: c1*t + c0 = 0 */
      if (abs(c1) < 1e-7f) {
        return -1.0f;
      }
      float t = -c0 / c1;
      return (t >= 0.0f && t <= tfar) ? t : -1.0f;
    }
    /* Quadratic: c2*t^2 + c1*t + c0 = 0 */
    float disc = c1 * c1 - 4.0f * c2 * c0;
    if (disc < 0.0f) {
      return -1.0f;
    }
    disc = sqrt(disc);
    float inv2a = 0.5f / c2;
    float t1 = (-c1 - disc) * inv2a;
    float t2 = (-c1 + disc) * inv2a;
    float lo = min(t1, t2);
    float hi = max(t1, t2);
    if (lo >= 0.0f && lo <= tfar) {
      return lo;
    }
    if (hi >= 0.0f && hi <= tfar) {
      return hi;
    }
    return -1.0f;
  }

  /* Normalize: t^3 + a*t^2 + b*t + cc = 0 */
  float inv_c3 = 1.0f / c3;
  float a = c2 * inv_c3;
  float b = c1 * inv_c3;
  float cc = c0 * inv_c3;

  /* Depress: substitute t = u - a/3
   * Yields: u^3 + p*u + q = 0
   * p = b - a^2/3
   * q = (2*a^3 - 9*a*b + 27*cc) / 27 */
  float a_3 = a / 3.0f;
  float a2 = a * a;
  float p = b - a2 / 3.0f;
  float q = (2.0f * a2 * a - 9.0f * a * b + 27.0f * cc) / 27.0f;

  float p_3 = p / 3.0f;
  float q_2 = q / 2.0f;
  float D = q_2 * q_2 + p_3 * p_3 * p_3;

  float roots[3];
  int n_roots;

  if (D > 1e-10f) {
    /* One real root (Cardano's formula). */
    float sqrtD = sqrt(D);
    roots[0] = sdf_cbrt(-q_2 + sqrtD) + sdf_cbrt(-q_2 - sqrtD) - a_3;
    n_roots = 1;
  }
  else if (D < -1e-10f) {
    /* Three real roots (Vieta's trigonometric solution).
     * Since D < 0, p must be negative, so -p/3 > 0. */
    float neg_p_3 = -p_3;
    float m = 2.0f * sqrt(neg_p_3);
    float r = neg_p_3 * sqrt(neg_p_3); /* (-p/3)^(3/2) */
    float theta = acos(clamp(-q_2 / r, -1.0f, 1.0f));
    roots[0] = m * cos(theta / 3.0f) - a_3;
    roots[1] = m * cos((theta + 2.0f * SDF_PI) / 3.0f) - a_3;
    roots[2] = m * cos((theta + 4.0f * SDF_PI) / 3.0f) - a_3;
    n_roots = 3;
  }
  else {
    /* D ~ 0: repeated root case. */
    if (abs(q_2) < 1e-10f) {
      /* Triple root at u = 0. */
      roots[0] = -a_3;
      n_roots = 1;
    }
    else {
      float u = sdf_cbrt(-q_2);
      roots[0] = 2.0f * u - a_3;  /* distinct root */
      roots[1] = -u - a_3;        /* repeated root */
      n_roots = 2;
    }
  }

  /* Select smallest root in [0, tfar]. */
  float best = -1.0f;
  for (int i = 0; i < 3; i++) {
    if (i >= n_roots) {
      break;
    }
    float root = roots[i];
    if (root >= -1e-5f && root <= tfar + 1e-5f) {
      root = clamp(root, 0.0f, tfar);
      if (best < 0.0f || root < best) {
        best = root;
      }
    }
  }
  return best;
}

/**
 * Marmitt + Newton-Raphson cubic solver.
 * Splits [0, tfar] into monotone intervals using g'(t) = 0 roots,
 * then uses Newton-Raphson refinement in subintervals with sign changes.
 * More robust than the analytic solver for edge cases.
 *
 * \return smallest root in [0, tfar], or -1.0 if no root found.
 */
float solveCubicMarmittNR(float c[4], float tfar)
{
  /* Find critical points: g'(t) = c[1] + 2*c[2]*t + 3*c[3]*t^2 = 0 */
  float da = 3.0f * c[3];
  float db = 2.0f * c[2];
  float dc = c[1];

  /* Build sorted interval boundaries: [0, crit1?, crit2?, tfar]. */
  float bounds[4];
  int n = 0;
  bounds[n++] = 0.0f;

  if (abs(da) > 1e-8f) {
    float disc = db * db - 4.0f * da * dc;
    if (disc >= 0.0f) {
      disc = sqrt(disc);
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
  else if (abs(db) > 1e-8f) {
    float t1 = -dc / db;
    if (t1 > 1e-6f && t1 < tfar - 1e-6f) {
      bounds[n++] = t1;
    }
  }

  bounds[n++] = tfar;

  /* Check each monotone interval for a sign change. */
  float fa = evalCubic(c, bounds[0]);
  for (int i = 0; i < n - 1; i++) {
    float fb = evalCubic(c, bounds[i + 1]);
    if (fa * fb <= 0.0f) {
      /* Sign change found: Newton-Raphson from midpoint. */
      float t = (bounds[i] + bounds[i + 1]) * 0.5f;
      for (int j = 0; j < 6; j++) {
        float f = evalCubic(c, t);
        float fp = evalCubicDeriv(c, t);
        if (abs(fp) < 1e-12f) {
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

/** \} */

/* -------------------------------------------------------------------- */
/** \name Analytical Trilinear Gradient
 * \{ */

/**
 * Compute the gradient of trilinear interpolation at point p in [0,1]^3.
 * Uses the 8 corner SDF values directly -- no texture reads needed.
 * ~12 FMAs vs 6 texture reads + 3 subtractions for central differences.
 *
 * \param s: corner SDF values (same ordering as computeTrilinearCoeffs).
 * \param p: evaluation point in [0,1]^3 local voxel space.
 * \return unnormalized gradient vector.
 */
float3 trilinearGradient(float s[8], float3 p)
{
  float k1 = s[1] - s[0];
  float k2 = s[2] - s[0];
  float k3 = s[4] - s[0];
  float k4 = s[3] - s[1] - s[2] + s[0];
  float k5 = s[5] - s[1] - s[4] + s[0];
  float k6 = s[6] - s[2] - s[4] + s[0];
  float k7 = s[7] - s[6] - s[5] - s[3] + s[4] + s[2] + s[1] - s[0];

  float3 grad;
  grad.x = k1 + k4 * p.y + k5 * p.z + k7 * p.y * p.z;
  grad.y = k2 + k4 * p.x + k6 * p.z + k7 * p.x * p.z;
  grad.z = k3 + k5 * p.x + k6 * p.y + k7 * p.x * p.y;
  return grad;
}

/** \} */
