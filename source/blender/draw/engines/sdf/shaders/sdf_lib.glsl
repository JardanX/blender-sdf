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
/** \name BVH Traversal Helpers
 * \{ */

/** Maximum BVH traversal stack depth (supports ~4 billion nodes). */
#define BVH_MAX_STACK 32

/** Decode an integer packed via intBitsToFloat in a BVH node field. */
int bvh_decode_int(float encoded)
{
  return floatBitsToInt(encoded);
}

/** Test whether two AABBs overlap (inclusive). */
bool aabb_overlap(float3 a_min, float3 a_max, float3 b_min, float3 b_max)
{
  return all(lessThanEqual(a_min, b_max)) && all(greaterThanEqual(a_max, b_min));
}

/** Ray-AABB intersection test (slab method).
 * Returns true if the ray intersects the AABB, with entry/exit t values.
 * \param origin: ray origin.
 * \param inv_dir: 1.0 / ray_direction (precomputed).
 * \param aabb_min: AABB minimum corner.
 * \param aabb_max: AABB maximum corner.
 * \param t_near: output entry t (may be negative if origin is inside).
 * \param t_far: output exit t.
 */
bool ray_aabb_intersect(float3 origin,
                        float3 inv_dir,
                        float3 aabb_min,
                        float3 aabb_max,
                        out float t_near,
                        out float t_far)
{
  float3 t0 = (aabb_min - origin) * inv_dir;
  float3 t1 = (aabb_max - origin) * inv_dir;
  float3 tlo = min(t0, t1);
  float3 thi = max(t0, t1);
  t_near = max(max(tlo.x, tlo.y), tlo.z);
  t_far = min(min(thi.x, thi.y), thi.z);
  return t_near <= t_far && t_far > 0.0f;
}

/** \} */

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

/* ---- Smooth blend operations ---- */

float opSmoothUnion(float d1, float d2, float k)
{
  if (k <= 0.0001f) {
    return min(d1, d2);
  }
  float h = clamp(0.5f + 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
  return mix(d2, d1, h) - k * h * (1.0f - h);
}

float opSmoothSubtraction(float d1, float d2, float k)
{
  if (k <= 0.0001f) {
    return max(-d1, d2);
  }
  float h = clamp(0.5f - 0.5f * (d2 + d1) / k, 0.0f, 1.0f);
  return mix(d2, -d1, h) + k * h * (1.0f - h);
}

float opSmoothIntersection(float d1, float d2, float k)
{
  if (k <= 0.0001f) {
    return max(d1, d2);
  }
  float h = clamp(0.5f - 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
  return mix(d2, d1, h) + k * h * (1.0f - h);
}

/* ---- Chamfer blend operations ---- */

float opChamferUnion(float a, float b, float r)
{
  return min(min(a, b), (a - r + b) * 0.70710678f);
}

float opChamferIntersection(float a, float b, float r)
{
  return max(max(a, b), (a + r + b) * 0.70710678f);
}

float opChamferSubtraction(float d1, float d2, float r)
{
  return opChamferIntersection(d2, -d1, r);
}

float opSmoothChamferUnion(float d1, float d2, float k, float k2, float k3)
{
  float chamfer_plane = (d1 + d2 - k) * 0.70710678f;
  float term1 = opSmoothUnion(d1, chamfer_plane, k2);
  float term2 = opSmoothUnion(d2, chamfer_plane, k3);
  return min(term1, term2);
}

float opSmoothChamferSubtraction(float d1, float d2, float k, float k2, float k3)
{
  float A = -d1;
  float B = d2;
  float chamfer_plane = (A + B + k) * 0.70710678f;
  float term1 = opSmoothIntersection(A, chamfer_plane, k2);
  float term2 = opSmoothIntersection(B, chamfer_plane, k3);
  return max(term1, term2);
}

float opSmoothChamferIntersection(float d1, float d2, float k, float k2, float k3)
{
  float chamfer_plane = (d1 + d2 + k) * 0.70710678f;
  float term1 = opSmoothIntersection(d1, chamfer_plane, k2);
  float term2 = opSmoothIntersection(d2, chamfer_plane, k3);
  return max(term1, term2);
}

/* ---- 2D mirror helper (for round blend operations) ---- */

float2 sdf_mirror2D(float2 p, float2 N)
{
  float proj = min(dot(p, N), 0.0f);
  return p - 2.0f * N * proj;
}

/* ---- Round (spherical) blend operations ---- */

/* Core building block: inward-rounding difference (concave fillet).
 * Uses 2D distance-field geometry in the (a, b) plane with mirror2D. */
float opDifferenceIRound(float a, float b, float r)
{
  float2 q = float2(a, b);
  q = sdf_mirror2D(q, normalize(float2(1.0f, 1.0f)));
  q.y -= r;
  q.y = min(0.0f, q.y);
  float ad = sign(q.x) * length(q);
  float2 s = float2(min(a, 0.0f), max(b, 0.0f));
  float corn = -(length(s) - r);
  return max(ad, corn);
}

/* Inward-rounding union (concave fillet). Kept for future use. */
float opUnionIRound(float a, float b, float r)
{
  float2 q = float2(a, b);
  q = sdf_mirror2D(q, normalize(float2(-1.0f, 1.0f)));
  q.y -= r;
  q.y = min(0.0f, q.y);
  float ad = sign(q.x) * length(q);
  float2 s = float2(max(a, 0.0f), max(b, 0.0f));
  float corn = length(s) - r;
  return min(ad, corn);
}

float opIntersectIRound(float a, float b, float r)
{
  return opDifferenceIRound(a, -b, r);
}

/* Outward-expanding round operations (convex fillet at junctions).
 * opRoundUnion uses opUnionIRound directly — produces an outward convex
 * bulge at the union seam. Subtraction and intersection derived by duality. */

float opRoundUnion(float d1, float d2, float r)
{
  return opUnionIRound(d1, d2, r);
}

float opRoundSubtraction(float d1, float d2, float r)
{
  return -opUnionIRound(d1, -d2, r);
}

float opRoundIntersection(float d1, float d2, float r)
{
  return -opUnionIRound(-d1, -d2, r);
}

/* Smooth + round dual-radius blends. */

float opSmoothRoundUnion(float a, float b, float r, float k2, float k3)
{
  float2 s = float2(max(a, 0.0f), max(b, 0.0f));
  float corner = length(s) - r;
  float term1 = opSmoothUnion(a, corner, k2);
  float term2 = opSmoothUnion(b, corner, k3);
  return min(term1, term2);
}

float opSmoothRoundSubtraction(float d1, float d2, float r, float k2, float k3)
{
  float a = d2;
  float b = d1;
  float2 s = float2(min(a, 0.0f), max(b, 0.0f));
  float corner = r - length(s);
  float term1 = opSmoothIntersection(a, corner, k2);
  float term2 = opSmoothIntersection(-b, corner, k3);
  return max(term1, term2);
}

float opSmoothRoundIntersection(float d1, float d2, float r, float k2, float k3)
{
  float2 s = float2(min(d1, 0.0f), max(-d2, 0.0f));
  float corner = r - length(s);
  float term1 = opSmoothIntersection(d1, corner, k2);
  float term2 = opSmoothIntersection(d2, corner, k3);
  return max(term1, term2);
}

/* Smooth + round inverted (outward) union. */
float opSmoothRoundUnionInverted(float a, float b, float r, float k2, float k3)
{
  float diff = opSmoothRoundSubtraction(b, a, r, k2, k3);
  return min(diff, b);
}

/* ---- Shell (onion) helper ---- */

float opOnion(float d, float thickness)
{
  return abs(d) - thickness;
}

/* ---- CSG dispatch ---- */

/** CSG operation IDs (must match eSDFCSGOperation in DNA_sdf_types.h). */
#define SDF_CSG_OP_UNION 0
#define SDF_CSG_OP_SUBTRACT 1
#define SDF_CSG_OP_INTERSECT 2
#define SDF_CSG_OP_SHELL 3

/** Blend type IDs (must match eSDFBlendType in DNA_sdf_types.h). */
#define SDF_BLEND_TYPE_LINEAR 0
#define SDF_BLEND_TYPE_SMOOTH 1
#define SDF_BLEND_TYPE_CHAMFER 2
#define SDF_BLEND_TYPE_ROUND 3

/**
 * Combine two SDF distances using the specified CSG operation and blend type.
 * \param d1: accumulated distance field.
 * \param d2: new object's distance.
 * \param op: CSG operation (0=union, 1=subtract, 2=intersect, 3=shell).
 * \param bt: blend type (0=linear, 1=smooth, 2=chamfer, 3=round).
 * \param k: blend radius for smooth/chamfer/round transitions.
 * \param shell_dist: shell expansion thickness (only used when op == SHELL).
 * \return combined distance.
 */
float combineCSG(float d1, float d2, int op, int bt, float k, float shell_dist)
{
  if (op == SDF_CSG_OP_UNION) {
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        return opSmoothUnion(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        return opChamferUnion(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        return opRoundUnion(d1, d2, k);
      }
    }
    return min(d1, d2);
  }
  else if (op == SDF_CSG_OP_SUBTRACT) {
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        return opSmoothSubtraction(d2, d1, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        return opChamferSubtraction(d2, d1, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        return opRoundSubtraction(d2, d1, k);
      }
    }
    return max(d1, -d2);
  }
  else if (op == SDF_CSG_OP_INTERSECT) {
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        return opSmoothIntersection(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        return opChamferIntersection(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        return opRoundIntersection(d1, d2, k);
      }
    }
    return max(d1, d2);
  }
  /* SDF_CSG_OP_SHELL: expand the intersection region between base and shape.
   * 1) Union the new shape into the base (using blend type).
   * 2) Intersect with a limit surface (base offset inward by k).
   * Result: only the overlap region is kept, expanded outward by k. */
  {
    /* Clamp blend for shell: 0 causes hard edges, too high overwhelms geometry. */
    k = clamp(k, 0.01f, abs(shell_dist) * 2.0f);

    float d_union;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        d_union = opSmoothUnion(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        d_union = opChamferUnion(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        d_union = opRoundUnion(d1, d2, k);
      }
      else {
        d_union = min(d1, d2);
      }
    }
    else {
      d_union = min(d1, d2);
    }

    float lim = d1 - abs(shell_dist);

    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        return opSmoothIntersection(d_union, lim, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        return opChamferIntersection(d_union, lim, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        return opRoundIntersection(d_union, lim, k);
      }
    }
    return max(d_union, lim);
  }
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
