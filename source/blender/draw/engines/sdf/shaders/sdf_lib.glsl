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

/**
 * 2D rounded box with per-corner radii.
 * \param r: corner radii (x=+x+y, y=+x-y, z=-x+y, w=-x-y).
 */
float sdRoundBox2D(float2 p, float2 b, float4 r)
{
  float rx = (p.x > 0.0f) ? r.x : r.z;
  float ry = (p.x > 0.0f) ? r.y : r.w;
  float rc = (p.y > 0.0f) ? rx : ry;

  float2 q = abs(p) - b + rc;
  return min(max(q.x, q.y), 0.0f) + length(max(q, float2(0.0f))) - rc;
}

/**
 * 2D chamfered box with per-corner radii.
 */
float sdChamferBox2D(float2 p, float2 b, float4 r)
{
  float rx = (p.x > 0.0f) ? r.x : r.z;
  float ry = (p.x > 0.0f) ? r.y : r.w;
  float rc = (p.y > 0.0f) ? rx : ry;

  float2 q = abs(p) - b;
  float rr = rc;
  if (rr < 0.001f) {
    return length(max(q, float2(0.0f))) + min(max(q.x, q.y), 0.0f);
  }
  float d_cham = (q.x + q.y + rr) * 0.70710678f;
  float d_box = max(q.x, q.y);
  float d = max(d_box, d_cham);
  if (d <= 0.0f) {
    return d;
  }
  if (d_box <= 0.0f) {
    return d_cham;
  }
  if (q.y <= -rr || q.x <= -rr) {
    return length(max(q, float2(0.0f)));
  }
  float t = (-q.x + q.y + rr) / (2.0f * rr);
  if (t <= 0.0f) {
    return length(q - float2(0.0f, -rr));
  }
  if (t >= 1.0f) {
    return length(q - float2(-rr, 0.0f));
  }
  return d_cham;
}

/**
 * Advanced box SDF with per-corner bevels, top/bottom edge chamfer, and taper.
 * \param corners: per-corner bevel radii (normalized 0–1).
 * \param edgeTop: top face edge chamfer (normalized 0–1).
 * \param edgeBot: bottom face edge chamfer (normalized 0–1).
 * \param tapTop: top taper amount (>0 shrinks top).
 * \param tapBot: bottom taper amount (>0 shrinks bottom).
 * \param cornerMode: 0=smooth, 1=chamfer.
 * \param edgeMode: 0=smooth, 1=chamfer.
 * \param taperZ: Z half-size for taper normalization.
 */
float sdAdvancedBox(float3 p,
                    float3 size,
                    float4 corners,
                    float edgeTop,
                    float edgeBot,
                    float tapTop,
                    float tapBot,
                    int cornerMode,
                    int edgeMode,
                    float taperZ)
{
  float zn = clamp(p.z / max(taperZ, 0.001f), -1.0f, 1.0f);
  float t = (zn + 1.0f) * 0.5f;
  float tapFactor = max(1.0f - tapTop * t - tapBot * (1.0f - t), 0.001f);
  float2 sz = size.xy * tapFactor;

  float maxR = min(sz.x, sz.y);
  float4 r = corners * maxR;

  float d2d;
  if (cornerMode == 0) {
    d2d = sdRoundBox2D(p.xy, sz, r);
  }
  else {
    d2d = sdChamferBox2D(p.xy, sz, r);
  }

  float tc = clamp(t, 0.0f, 1.0f);
  float tapFace = max(1.0f - tapTop * tc - tapBot * (1.0f - tc), 0.001f);
  float maxR_face = min(size.x * tapFace, size.y * tapFace);

  float dz = abs(p.z) - size.z;
  float edgeR = (p.z > 0.0f) ? edgeTop * min(maxR_face, size.z)
                              : edgeBot * min(maxR_face, size.z);

  if (edgeR > 0.001f) {
    if (edgeMode == 0) {
      float2 dd = float2(d2d + edgeR, dz + edgeR);
      return min(max(dd.x, dd.y), 0.0f) + length(max(dd, float2(0.0f))) - edgeR;
    }
    else {
      float base = max(d2d, dz);
      float cham = (d2d + dz + edgeR) * 0.70710678f;
      float dd = max(base, cham);
      if (dd <= 0.0f) {
        return dd;
      }
      if (d2d <= 0.0f && dz <= 0.0f) {
        return cham;
      }
      if (dz <= -edgeR) {
        return d2d;
      }
      if (d2d <= -edgeR) {
        return dz;
      }
      float tc2 = (-d2d + dz + edgeR) / (2.0f * edgeR);
      if (tc2 <= 0.0f) {
        return length(float2(d2d, dz + edgeR));
      }
      if (tc2 >= 1.0f) {
        return length(float2(d2d + edgeR, dz));
      }
      return cham;
    }
  }
  else {
    float2 dd = float2(d2d, dz);
    return length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
  }
}

/**
 * 2D regular polygon SDF (centered at origin).
 * \param p: 2D sample point.
 * \param R: circumradius.
 * \param n: number of sides (3+).
 */
float sdRegularPolygon2D(float2 p, float R, int n)
{
  float an = SDF_PI / float(n);
  float r = R * cos(an);
  float he = R * sin(an);
  p = float2(-p.y, p.x);
  float bn = an * floor((atan(p.y, p.x) + an) / an / 2.0f) * 2.0f;
  float2 cs = float2(cos(bn), sin(bn));
  p = float2(cs.x * p.x + cs.y * p.y, -cs.y * p.x + cs.x * p.y);
  return length(p - float2(r, clamp(p.y, -he, he))) * sign(p.x - r);
}

/**
 * Capped torus SDF (Inigo Quilez).
 * \param p: 3D sample point.
 * \param sc: float2(sin(half_angle), cos(half_angle)).
 * \param ra: major radius.
 * \param rb: minor (tube) radius.
 */
float sdCappedTorus(float3 p, float2 sc, float ra, float rb)
{
  p.x = abs(p.x);
  float k = (sc.y * p.x > sc.x * p.y) ? dot(p.xy, sc) : length(p.xy);
  return sqrt(dot(p, p) + ra * ra - 2.0f * ra * k) - rb;
}

/**
 * Star polygon 2D SDF with n outer tips and n inner valleys.
 * Uses sector-folding: folds into a half-sector [0, PI/n], then computes
 * distance to the edge from outer tip (R, 0) to inner valley at angle PI/n.
 * \param R: outer tip radius (circumradius).
 * \param n: number of star points.
 * \param star: 0 = regular polygon, 1 = valleys reach center.
 */
float sdStarPolygon2D(float2 p, float R, int n, float star)
{
  if (star < 0.001f) {
    return sdRegularPolygon2D(p, R, n);
  }

  float an = SDF_PI / float(n);
  /* Inner valley radius: star=0 → apothem (matches polygon), star=1 → near center. */
  float r = R * cos(an) * max(1.0f - star, 0.01f);

  /* Fold into fundamental half-sector centered on nearest outer tip.
   * Outer tips at angles k*2*an; inner valleys at (k+0.5)*2*an = (2k+1)*an. */
  float angle = atan(p.y, p.x);
  float bn = floor((angle + an) / (2.0f * an)) * 2.0f * an;
  float2 cs = float2(cos(bn), sin(bn));
  float2 q = float2(cs.x * p.x + cs.y * p.y, -cs.y * p.x + cs.x * p.y);
  /* Mirror to [0, an] half-sector (rotated angle is in [-an, an]). */
  q.y = abs(q.y);

  /* Edge from outer tip A=(R, 0) to inner valley B=(r*cos(an), r*sin(an)). */
  float2 A = float2(R, 0.0f);
  float2 B = float2(r * cos(an), r * sin(an));
  float2 AB = B - A;
  float t = clamp(dot(q - A, AB) / dot(AB, AB), 0.0f, 1.0f);
  float dist = length(q - (A + AB * t));

  /* Sign: cross product determines inside (toward center) vs outside. */
  float cross_val = AB.x * (q.y - A.y) - AB.y * (q.x - A.x);
  return (cross_val > 0.0f) ? -dist : dist;
}

/**
 * Advanced N-Gon prism SDF with corner bevel, edge chamfer, and taper.
 * \param p: 3D sample point in local space.
 * \param R: circumradius of the polygon.
 * \param halfH: half-height along Z.
 * \param sides: number of polygon sides.
 * \param corner: corner bevel amount (0–1, fraction of apothem).
 * \param edgeTop: top face edge chamfer (0–1).
 * \param edgeBot: bottom face edge chamfer (0–1).
 * \param tapTop: top taper amount.
 * \param tapBot: bottom taper amount.
 * \param edgeMode: 0=smooth, 1=chamfer.
 * \param taperH: Z half-size for taper normalization.
 */
float sdAdvancedNgon(float3 p,
                     float R,
                     float halfH,
                     int sides,
                     float corner,
                     float edgeTop,
                     float edgeBot,
                     float tapTop,
                     float tapBot,
                     int edgeMode,
                     float taperH,
                     float star)
{
  float zn = clamp(p.z / max(taperH, 0.001f), -1.0f, 1.0f);
  float t = (zn + 1.0f) * 0.5f;
  float tapFactor = max(1.0f - tapTop * t - tapBot * (1.0f - t), 0.001f);
  float scaledR = R * tapFactor;

  float an = SDF_PI / float(sides);
  float apothem = scaledR * cos(an);
  float bevelR = corner * apothem;
  float innerR = scaledR - bevelR / max(cos(an), 0.001f);
  float d2d;
  if (star > 0.001f) {
    d2d = sdStarPolygon2D(p.xy, innerR, sides, star) - bevelR;
  }
  else {
    d2d = sdRegularPolygon2D(p.xy, innerR, sides) - bevelR;
  }

  float tc = clamp(t, 0.0f, 1.0f);
  float tapFace = max(1.0f - tapTop * tc - tapBot * (1.0f - tc), 0.001f);
  float apothem_face = R * tapFace * cos(an);

  float dz = abs(p.z) - halfH;
  float edgeR = (p.z > 0.0f) ? edgeTop * min(apothem_face, halfH)
                              : edgeBot * min(apothem_face, halfH);

  if (edgeR > 0.001f) {
    if (edgeMode == 0) {
      float2 dd = float2(d2d + edgeR, dz + edgeR);
      return min(max(dd.x, dd.y), 0.0f) + length(max(dd, float2(0.0f))) - edgeR;
    }
    else {
      float base = max(d2d, dz);
      float cham = (d2d + dz + edgeR) * 0.70710678f;
      float dd = max(base, cham);
      if (dd <= 0.0f) {
        return dd;
      }
      if (d2d <= 0.0f && dz <= 0.0f) {
        return cham;
      }
      if (dz <= -edgeR) {
        return d2d;
      }
      if (d2d <= -edgeR) {
        return dz;
      }
      float tc2 = (-d2d + dz + edgeR) / (2.0f * edgeR);
      if (tc2 <= 0.0f) {
        return length(float2(d2d, dz + edgeR));
      }
      if (tc2 >= 1.0f) {
        return length(float2(d2d + edgeR, dz));
      }
      return cham;
    }
  }
  else {
    float2 dd = float2(d2d, dz);
    return length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
  }
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

/* ---- 2D mirror helper (for round blend operations) ---- */

float2 sdf_mirror2D(float2 p, float2 N)
{
  float proj = min(dot(p, N), 0.0f);
  return p - 2.0f * N * proj;
}

/* ---- Round (spherical) blend operations ---- */

/* Inward-rounding union (concave fillet). */
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

/* ---- SDF modifier evaluation ---- */

/** Modifier type IDs (must match eSDFModifierType in DNA_sdf_types.h). */
#define SDF_MOD_MIRROR 0
#define SDF_MOD_TWIST 1
#define SDF_MOD_BEND 2
#define SDF_MOD_ELONGATE 3
#define SDF_MOD_HOLLOW 4
#define SDF_MOD_ROUND 5
#define SDF_MOD_ONION 6
#define SDF_MOD_BEVEL 7
#define SDF_MOD_ARRAY 8

/** Mirror axis flags. */
#define SDF_MOD_MIRROR_X 1
#define SDF_MOD_MIRROR_Y 2
#define SDF_MOD_MIRROR_Z 4

/** Array types. */
#define SDF_MOD_ARRAY_LINEAR 0
#define SDF_MOD_ARRAY_RADIAL 1

/**
 * Smooth absolute value for soft mirroring.
 */
float sabs(float x, float k)
{
  if (k <= 0.0001f) {
    return abs(x);
  }
  float h = clamp(0.5f + 0.5f * x / k, 0.0f, 1.0f);
  return x * (2.0f * h - 1.0f) + k * h * (1.0f - h);
}

/**
 * Smooth domain repetition staircase for soft arrays.
 */
float sround(float x, float k)
{
  if (k <= 0.0001f) {
    return round(x);
  }
  float i = floor(x + 0.5f - k * 0.5f);
  float f = fract(x + 0.5f - k * 0.5f);
  float step_val = smoothstep(0.0f, k, f);
  return i + step_val;
}

/**
 * Apply domain modifiers to local position (before SDF primitive evaluation).
 * Domain modifiers warp the sampling space.
 * Applied in reverse order so visual stack order is top-to-bottom.
 */
float3 applyDomainModifiers(float3 p, int mod_start, int mod_count)
{
  for (int i = mod_start + mod_count - 1; i >= mod_start; i--) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;
    int mflags = smod.header.y;

    if (mtype == SDF_MOD_MIRROR) {
      float offset = smod.params.x;
      float blend = smod.params.y;
      if ((mflags & SDF_MOD_MIRROR_X) != 0) {
        p.x = sabs(p.x, blend) - offset;
      }
      if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
        p.y = sabs(p.y, blend) - offset;
      }
      if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
        p.z = sabs(p.z, blend) - offset;
      }
    }
    else if (mtype == SDF_MOD_TWIST) {
      float k = smod.params.x;
      float angle = k * p.z;
      float c = cos(angle);
      float s = sin(angle);
      p = float3(c * p.x - s * p.y, s * p.x + c * p.y, p.z);
    }
    else if (mtype == SDF_MOD_BEND) {
      float k = smod.params.x;
      float angle = k * p.x;
      float c = cos(angle);
      float s = sin(angle);
      p = float3(s * p.y + c * p.x, c * p.y - s * p.x, p.z);
    }
    else if (mtype == SDF_MOD_ELONGATE) {
      float3 h = smod.params.xyz;
      p = p - clamp(p, -h, h);
    }
    else if (mtype == SDF_MOD_ARRAY) {
      float count = smod.params.x;
      float blend = smod.params2.x; /* blend is at params[4] = params2.x */
      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        float3 offset = smod.params.yzw;
        float spacing = length(offset);
        if (spacing > 0.0001f && count > 0.5f) {
          float3 dir = offset / spacing;
          float t = dot(p, dir);
          float id = sround(t / spacing, clamp(blend, 0.0f, 1.0f));
          id = clamp(id, 0.0f, max(0.0f, count - 1.0f));
          p = p - dir * id * spacing;
        }
      }
      else if (mflags == SDF_MOD_ARRAY_RADIAL) {
        float radius = smod.params.y;
        if (count > 0.5f) {
          float a = (2.0f * SDF_PI) / count;
          float current_a = atan(p.y, p.x);
          float id = sround(current_a / a, clamp(blend, 0.0f, 1.0f));
          float final_a = current_a - id * a;
          float r = length(p.xy);
          p.x = r * cos(final_a) - radius;
          p.y = r * sin(final_a);
        }
      }
    }
  }
  return p;
}

/**
 * Cylinder SDF.
 * \param size: xy = radii, z = half-height.
 */
float sdCylinder(float3 p, float3 size)
{
  float2 e = max(size.xy, float2(0.001f));
  float2 pn = p.xy / e;
  float rn = length(pn);
  float2 g = pn / (e * max(rn, 1e-6f));
  float radial = (rn - 1.0f) / max(length(g), 1e-6f);
  float vertical = abs(p.z) - size.z;
  float2 d = float2(radial, vertical);
  return length(max(d, float2(0.0f))) + min(max(d.x, d.y), 0.0f);
}

/**
 * Cone SDF.
 */
float sdCone(float3 p, float r, float h)
{
  /* Capped cone: base radius r at z=-h, apex at z=+h.
   * Based on Inigo Quilez's sdCappedCone.
   * Must match CPU evaluation in shape_classify_cpu(). */
  float2 q = float2(length(p.xy), p.z);
  float2 k1 = float2(0.0f, h);
  float2 k2 = float2(-r, 2.0f * h);
  float2 ca = float2(q.x - min(q.x, (q.y < 0.0f) ? r : 0.0f), abs(q.y) - h);
  float2 cb = q - k1 + k2 * clamp(dot(k1 - q, k2) / dot(k2, k2), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * sqrt(min(dot(ca, ca), dot(cb, cb)));
}

/**
 * Capsule SDF.
 */
float sdCapsule(float3 p, float3 size)
{
  float h = size.y;
  float r = size.x;
  p.z -= clamp(p.z, -h, h);
  return length(p) - r;
}

/**
 * Torus SDF.
 */
float sdTorus(float3 p, float2 t)
{
  float2 q = float2(length(p.xy) - t.x, p.z);
  return length(q) - t.y;
}

/**
 * Sphere SDF.
 */
float sdSphere(float3 p, float r)
{
  return length(p) - r;
}

/**
 * Common primitive data struct for cross-shader evaluation.
 */
struct SDFPrimitiveData {
  int sdf_type;
  float3 size;
  float bevel;
  float4 box_corners;
  float4 box_edges;
  int4 box_modes;
  int modifier_start;
  int modifier_count;
};

/* ---- CSG dispatch ---- */

/** CSG operation IDs (must match eSDFCSGOperation in DNA_sdf_types.h). */
#define SDF_CSG_OP_UNION 0
#define SDF_CSG_OP_SUBTRACT 1
#define SDF_CSG_OP_INTERSECT 2
#define SDF_CSG_OP_SHELL 3
#define SDF_CSG_OP_PUSH 4
#define SDF_CSG_OP_AVOID 5

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
  else if (op == SDF_CSG_OP_PUSH) {
    /* Push: subtract d2 from base (with blend), then hard-union d2 back.
     * Effect: the push object dents the base but remains visible as solid. */
    float subtracted;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        subtracted = opSmoothSubtraction(d2, d1, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        subtracted = opChamferSubtraction(d2, d1, k);
      }
      else {
        subtracted = opRoundSubtraction(d2, d1, k);
      }
    }
    else {
      subtracted = max(d1, -d2);
    }
    return min(subtracted, d2);
  }
  else if (op == SDF_CSG_OP_AVOID) {
    /* Avoid: subtract base from avoid object, then hard union with base.
     * Effect: the avoid object is carved by the base geometry. */
    float carved;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        carved = opSmoothSubtraction(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        carved = opChamferSubtraction(d1, d2, k);
      }
      else {
        carved = opRoundSubtraction(d1, d2, k);
      }
    }
    else {
      carved = max(d2, -d1);
    }
    return min(d1, carved);
  }
  else if (op == SDF_CSG_OP_SHELL) {
  /* SDF_CSG_OP_SHELL (extrusion): two-stage blend matching MathOPS algorithm.
   * Finds the intersection between two SDF fields and extrudes/insets a thin
   * wall of thickness |shell_dist| with correct blending at both stages.
   *
   * Stage 1 (shape blend) uses the full k for smooth transitions.
   * Stage 2 (limit blend) clamps k to shell thickness so the blend region
   * doesn't extend beyond the wall, which causes surface distortion.
   *
   * Positive shell_dist (extrusion): union shape into base, intersect with limit.
   * Negative shell_dist (inset): subtract shape from base, union with limit. */
    float h = abs(shell_dist);
    float lk = min(k, h);

    if (shell_dist < 0.0f) {
      float d_sub;
      if (k > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) {
          d_sub = opSmoothSubtraction(d2, d1, k);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          d_sub = opChamferSubtraction(d2, d1, k);
        }
        else if (bt == SDF_BLEND_TYPE_ROUND) {
          d_sub = opRoundSubtraction(d2, d1, k);
        }
        else {
          d_sub = max(d1, -d2);
        }
      }
      else {
        d_sub = max(d1, -d2);
      }

      float lim = d1 + h;
      if (lk > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) {
          return opSmoothUnion(d_sub, lim, lk);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          return opChamferUnion(d_sub, lim, lk);
        }
        else if (bt == SDF_BLEND_TYPE_ROUND) {
          return opRoundUnion(d_sub, lim, lk);
        }
      }
      return min(d_sub, lim);
    }
    else {
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

      float lim = d1 - h;
      if (lk > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) {
          return opSmoothIntersection(d_union, lim, lk);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          return opChamferIntersection(d_union, lim, lk);
        }
        else if (bt == SDF_BLEND_TYPE_ROUND) {
          return opRoundIntersection(d_union, lim, lk);
        }
      }
      return max(d_union, lim);
    }
  }
  return d1; /* Fallback for unknown ops. */
}

/**
 * Compute color blend weight for a CSG combination.
 * Returns h in [0,1]: h=0 keeps d1's color, h=1 uses d2's color.
 * Usage: color = mix(color1, color2, csgColorWeight(d1, d2, op, bt, k, sd));
 *
 * Uses the smooth h-factor as a universal proxy for all blend types
 * (smooth/chamfer/round). The actual distance functions differ, but
 * the smooth h produces visually correct color transitions.
 *
 * For Push and Avoid, the intermediate subtraction is recomputed
 * because the color weight depends on both the subtract and union stages.
 */
float csgColorWeight(float d1, float d2, int op, int bt, float k, float shell_dist)
{
  if (op == SDF_CSG_OP_UNION) {
    if (k > 0.0f && bt > 0) {
      return clamp(0.5f + 0.5f * (d1 - d2) / k, 0.0f, 1.0f);
    }
    return (d2 < d1) ? 1.0f : 0.0f;
  }
  else if (op == SDF_CSG_OP_SUBTRACT) {
    if (k > 0.0f && bt > 0) {
      return clamp(0.5f - 0.5f * (d1 + d2) / k, 0.0f, 1.0f);
    }
    return (d1 + d2 < 0.0f) ? 1.0f : 0.0f;
  }
  else if (op == SDF_CSG_OP_INTERSECT) {
    if (k > 0.0f && bt > 0) {
      return clamp(0.5f + 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
    }
    return (d2 > d1) ? 1.0f : 0.0f;
  }
  else if (op == SDF_CSG_OP_SHELL) {
    /* Extrusion (positive) uses union semantics, inset (negative) uses subtraction. */
    if (shell_dist < 0.0f) {
      if (k > 0.0f && bt > 0) {
        return clamp(0.5f - 0.5f * (d1 + d2) / k, 0.0f, 1.0f);
      }
      return (d1 + d2 < 0.0f) ? 1.0f : 0.0f;
    }
    if (k > 0.0f && bt > 0) {
      return clamp(0.5f + 0.5f * (d1 - d2) / k, 0.0f, 1.0f);
    }
    return (d2 < d1) ? 1.0f : 0.0f;
  }
  else if (op == SDF_CSG_OP_PUSH) {
    /* Push = subtract(d2 from d1) then min(subtracted, d2).
     * Color: union weight between subtracted base and push object. */
    float subtracted;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        subtracted = opSmoothSubtraction(d2, d1, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        subtracted = opChamferSubtraction(d2, d1, k);
      }
      else {
        subtracted = opRoundSubtraction(d2, d1, k);
      }
      return clamp(0.5f + 0.5f * (subtracted - d2) / k, 0.0f, 1.0f);
    }
    return (d2 <= max(d1, -d2)) ? 1.0f : 0.0f;
  }
  else if (op == SDF_CSG_OP_AVOID) {
    /* Avoid = subtract(d1 from d2) then min(d1, carved).
     * Color: union weight between base and carved avoid object. */
    float carved;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        carved = opSmoothSubtraction(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        carved = opChamferSubtraction(d1, d2, k);
      }
      else {
        carved = opRoundSubtraction(d1, d2, k);
      }
      return clamp(0.5f + 0.5f * (d1 - carved) / k, 0.0f, 1.0f);
    }
    return (max(d2, -d1) < d1) ? 1.0f : 0.0f;
  }
  return 0.0f;
}

/**
 * Evaluate the base primitive shape only.
 */
float evalPrimitiveOnly(SDFPrimitiveData obj, float3 local_pos)
{
  float3 size = obj.size;
  float bevel = obj.bevel;
  float dist;

  if (obj.sdf_type == 1) { /* SPHERE */
    dist = sdSphere(local_pos, size.x - bevel);
  }
  else if (obj.sdf_type == 2) { /* CYLINDER */
    float3 cyl_size = size - float3(bevel);
    cyl_size = max(cyl_size, float3(0.001f));
    dist = sdCylinder(local_pos, cyl_size);
  }
  else if (obj.sdf_type == 3) { /* CONE */
    float cone_r = max(size.x - bevel, 0.001f);
    float cone_h = max(size.y - bevel, 0.001f);
    dist = sdCone(local_pos, cone_r, cone_h);
  }
  else if (obj.sdf_type == 4) { /* CAPSULE */
    float3 cap_size = size - float3(bevel);
    cap_size = max(cap_size, float3(0.001f));
    dist = sdCapsule(local_pos, cap_size);
  }
  else if (obj.sdf_type == 5) { /* TORUS */
    float major = size.x - bevel;
    float minor = size.y - bevel;
    major = max(major, 0.001f);
    minor = max(minor, 0.001f);
    if (obj.box_modes.w != 0) {
      dist = sdCappedTorus(local_pos, obj.box_corners.xy, major, minor);
    }
    else {
      dist = sdTorus(local_pos, float2(major, minor));
    }
  }
  else if (obj.sdf_type == 6) { /* NGON */
    float R = max(size.x - bevel, 0.001f);
    float halfH = max(size.z - bevel, 0.001f);
    int sides = obj.box_modes.z;
    float corner = obj.box_corners.x;
    float star = obj.box_corners.y;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    int edgeMode = obj.box_modes.y;
    bool hasAdvanced = (corner + edgeTop + edgeBot + tapTop + tapBot + star) > 0.001f;
    if (hasAdvanced) {
      dist = sdAdvancedNgon(local_pos, R, halfH, sides, corner, edgeTop, edgeBot, tapTop, tapBot, edgeMode, halfH, star);
    }
    else {
      float d2d = sdRegularPolygon2D(local_pos.xy, R, sides);
      float dz = abs(local_pos.z) - halfH;
      float2 dd = float2(d2d, dz);
      dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
    }
  }
  else { /* BOX (default) */
    float4 corners = obj.box_corners;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    bool hasAdvanced = (corners.x + corners.y + corners.z + corners.w + edgeTop + edgeBot + tapTop + tapBot) > 0.001f;
    if (hasAdvanced) {
      float3 box_size = size - float3(bevel);
      box_size = max(box_size, float3(0.001f));
      dist = sdAdvancedBox(local_pos, box_size, corners, edgeTop, edgeBot, tapTop, tapBot, obj.box_modes.x, obj.box_modes.y, box_size.z);
    }
    else {
      float3 box_size = size - float3(bevel);
      box_size = max(box_size, float3(0.001f));
      dist = sdBox(local_pos, box_size);
    }
  }

  return dist - bevel;
}

/**
 * Apply distance modifiers to SDF distance (after primitive evaluation).
 * Applied in forward stack order.
 */
float applyDistanceModifiers(float dist, int mod_start, int mod_count)
{
  for (int i = mod_start; i < mod_start + mod_count; i++) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;

    if (mtype == SDF_MOD_HOLLOW) {
      dist = abs(dist) - smod.params.x;
    }
    else if (mtype == SDF_MOD_ROUND) {
      dist -= smod.params.x;
    }
    else if (mtype == SDF_MOD_ONION) {
      dist = abs(dist) - smod.params.x;
    }
  }
  return dist;
}

/**
 * Evaluate the full SDF for an object with robust branching for CSG modifiers.
 * Uses exact CSG distance evaluation for up to ONE Array/Mirror for artifact-free smooth blending.
 */
float evalObjectSDF(SDFPrimitiveData obj, float3 p)
{
  int fork_idx = -1;
  /* Find the highest (applied last to geometry, evaluated first in domain) forking modifier. */
  for (int i = obj.modifier_start; i < obj.modifier_start + obj.modifier_count; i++) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;
    /* If it has CSG (y), blend_type (z), or blend radius (x), it is configured for true CSG blending. */
    if ((mtype == SDF_MOD_ARRAY || mtype == SDF_MOD_MIRROR) && (smod.params2.y > 0.0f || smod.params2.z > 0.0f || smod.params2.x > 0.0f)) {
      fork_idx = i;
    }
  }

  if (fork_idx == -1) {
    p = applyDomainModifiers(p, obj.modifier_start, obj.modifier_count);
    float d = evalPrimitiveOnly(obj, p);
    return applyDistanceModifiers(d, obj.modifier_start, obj.modifier_count);
  }

  /* Forking path: apply domain modifiers UP TO the forking modifier. */
  int top_count = (obj.modifier_start + obj.modifier_count) - (fork_idx + 1);
  if (top_count > 0) {
    p = applyDomainModifiers(p, fork_idx + 1, top_count);
  }

  SDFModifierGPU fork_mod = sdf_modifiers[fork_idx];
  int mtype = fork_mod.header.x;
  int mflags = fork_mod.header.y;
  float blend = fork_mod.params2.x;
  int csg_op = int(fork_mod.params2.y);
  int blend_type = int(fork_mod.params2.z);

  float final_d = (csg_op == SDF_CSG_OP_INTERSECT) ? -1e10f : 1e10f;
  int bottom_count = fork_idx - obj.modifier_start;

  if (mtype == SDF_MOD_MIRROR) {
    int mirrors = 1;
    if ((mflags & SDF_MOD_MIRROR_X) != 0) mirrors *= 2;
    if ((mflags & SDF_MOD_MIRROR_Y) != 0) mirrors *= 2;
    if ((mflags & SDF_MOD_MIRROR_Z) != 0) mirrors *= 2;

    bool first = true;
    for (int i = 0; i < 8; i++) {
      if (i >= mirrors) break;
      float3 cell_p = p;
      int flip_idx = i;
      if ((mflags & SDF_MOD_MIRROR_X) != 0) { if ((flip_idx & 1) != 0) cell_p.x = -cell_p.x; flip_idx >>= 1; }
      if ((mflags & SDF_MOD_MIRROR_Y) != 0) { if ((flip_idx & 1) != 0) cell_p.y = -cell_p.y; flip_idx >>= 1; }
      if ((mflags & SDF_MOD_MIRROR_Z) != 0) { if ((flip_idx & 1) != 0) cell_p.z = -cell_p.z; }

      float offset = fork_mod.params.x;
      if ((mflags & SDF_MOD_MIRROR_X) != 0) cell_p.x -= offset;
      if ((mflags & SDF_MOD_MIRROR_Y) != 0) cell_p.y -= offset;
      if ((mflags & SDF_MOD_MIRROR_Z) != 0) cell_p.z -= offset;

      if (bottom_count > 0) cell_p = applyDomainModifiers(cell_p, obj.modifier_start, bottom_count);
      float d = applyDistanceModifiers(evalPrimitiveOnly(obj, cell_p), obj.modifier_start, obj.modifier_count);
      
      if (first) { final_d = d; first = false; }
      else final_d = combineCSG(final_d, d, csg_op, blend_type, blend, 0.0f);
    }
  }
  else if (mtype == SDF_MOD_ARRAY) {
    float count = fork_mod.params.x;
    float id;
    float3 dir = float3(1,0,0);
    float spacing = 1.0f;
    if (mflags == SDF_MOD_ARRAY_LINEAR) {
      float3 offset = fork_mod.params.yzw;
      spacing = length(offset);
      if (spacing > 0.0001f) {
        dir = offset / spacing;
        id = round(dot(p, dir) / spacing);
      } else id = 0.0f;
    } else {
      float a = (2.0f * SDF_PI) / max(count, 1e-4f);
      id = round(atan(p.y, p.x) / a);
    }

    float last_cid = -9999.0f;
    bool first = true;
    for(int i = -1; i <= 1; i++) {
      float cid = id + float(i);
      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        cid = clamp(cid, 0.0f, max(0.0f, count - 1.0f));
      }
      if (cid == last_cid) continue;
      last_cid = cid;

      float3 cell_p = p;
      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        if (spacing > 0.0001f) cell_p -= dir * cid * spacing;
      } else {
        float a = (2.0f * SDF_PI) / count;
        float final_a = atan(p.y, p.x) - cid * a;
        float r = length(p.xy);
        float radius = fork_mod.params.y;
        cell_p.x = r * cos(final_a) - radius;
        cell_p.y = r * sin(final_a);
      }

      if (bottom_count > 0) cell_p = applyDomainModifiers(cell_p, obj.modifier_start, bottom_count);
      float d = applyDistanceModifiers(evalPrimitiveOnly(obj, cell_p), obj.modifier_start, obj.modifier_count);
      
      if (first) { final_d = d; first = false; }
      else final_d = combineCSG(final_d, d, csg_op, blend_type, blend, 0.0f);
    }
  }

  return final_d;
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
