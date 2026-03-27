/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF primitives, CSG operations, and domain modifiers.
 */

#pragma once

#include "gpu_shader_compat.hh"

#define SDF_PI 3.14159265358979323846f

/* -------------------------------------------------------------------- */
/** \name BVH Traversal Helpers
 * \{ */

/** BVH traversal stack depth. 64 handles SAH trees up to ~1M leaves.
 * An 8-bin SAH tree with N leaves has worst-case depth ≈ 7*log(N),
 * and DFS stack usage ≈ depth + 2. */
#define BVH_MAX_STACK 64

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

/** Unsigned distance from a point to an AABB (0 if inside). */
float point_aabb_dist(float3 p, float3 bmin, float3 bmax)
{
  float3 q = max(bmin - p, max(p - bmax, float3(0.0f)));
  return length(q);
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

  /* Lipschitz correction: the per-slice taper creates a gradient magnitude of
   * sqrt(1 + slope^2) on the side walls. Dividing uniformly by this value
   * ensures the SDF is globally Lipschitz-1, which is required for safe
   * over-relaxation sphere tracing. This slightly underestimates distance
   * at the caps, costing a few extra steps, but avoids crack artifacts. */
  float slope = max(size.x, size.y) * (tapTop + tapBot) / (2.0f * max(taperZ, 0.001f));
  float lipschitz = sqrt(1.0f + slope * slope);

  float maxR = min(sz.x, sz.y);
  float4 r = corners * maxR;

  float d2d;
  if (cornerMode == 0) {
    d2d = sdRoundBox2D(p.xy, sz, r);
  }
  else {
    d2d = sdChamferBox2D(p.xy, sz, r);
  }

  float maxR_face = min(sz.x, sz.y);

  float dz = abs(p.z) - size.z;
  float edgeR = (p.z > 0.0f) ? edgeTop * min(maxR_face, size.z)
                              : edgeBot * min(maxR_face, size.z);

  if (edgeR > 0.001f) {
    if (edgeMode == 0) {
      float2 dd = float2(d2d + edgeR, dz + edgeR);
      return (min(max(dd.x, dd.y), 0.0f) + length(max(dd, float2(0.0f))) - edgeR) / lipschitz;
    }
    else {
      float base = max(d2d, dz);
      float cham = (d2d + dz + edgeR) * 0.70710678f;
      float dd = max(base, cham);
      if (dd <= 0.0f) {
        return dd / lipschitz;
      }
      if (d2d <= 0.0f && dz <= 0.0f) {
        return cham / lipschitz;
      }
      if (dz <= -edgeR) {
        return d2d / lipschitz;
      }
      if (d2d <= -edgeR) {
        return dz / lipschitz;
      }
      float tc2 = (-d2d + dz + edgeR) / (2.0f * edgeR);
      if (tc2 <= 0.0f) {
        return length(float2(d2d, dz + edgeR)) / lipschitz;
      }
      if (tc2 >= 1.0f) {
        return length(float2(d2d + edgeR, dz)) / lipschitz;
      }
      return cham / lipschitz;
    }
  }
  else {
    float2 dd = float2(d2d, dz);
    return (length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f)) / lipschitz;
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

  /* Lipschitz correction (same rationale as sdAdvancedBox). */
  float slope = R * (tapTop + tapBot) / (2.0f * max(taperH, 0.001f));
  float lipschitz = sqrt(1.0f + slope * slope);

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

  float dz = abs(p.z) - halfH;
  float edgeR = (p.z > 0.0f) ? edgeTop * min(apothem, halfH)
                              : edgeBot * min(apothem, halfH);

  if (edgeR > 0.001f) {
    if (edgeMode == 0) {
      float2 dd = float2(d2d + edgeR, dz + edgeR);
      return (min(max(dd.x, dd.y), 0.0f) + length(max(dd, float2(0.0f))) - edgeR) / lipschitz;
    }
    else {
      float base = max(d2d, dz);
      float cham = (d2d + dz + edgeR) * 0.70710678f;
      float dd = max(base, cham);
      if (dd <= 0.0f) {
        return dd / lipschitz;
      }
      if (d2d <= 0.0f && dz <= 0.0f) {
        return cham / lipschitz;
      }
      if (dz <= -edgeR) {
        return d2d / lipschitz;
      }
      if (d2d <= -edgeR) {
        return dz / lipschitz;
      }
      float tc2 = (-d2d + dz + edgeR) / (2.0f * edgeR);
      if (tc2 <= 0.0f) {
        return length(float2(d2d, dz + edgeR)) / lipschitz;
      }
      if (tc2 >= 1.0f) {
        return length(float2(d2d + edgeR, dz)) / lipschitz;
      }
      return cham / lipschitz;
    }
  }
  else {
    float2 dd = float2(d2d, dz);
    return (length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f)) / lipschitz;
  }
}

/* ---- Quadratic bezier distance (Inigo Quilez) ---- */

float sdBezier2D(float2 A, float2 B, float2 C, float2 pos)
{
  float2 a = B - A;
  float2 b = A - 2.0f * B + C;
  float2 c = a * 2.0f;
  float2 d = A - pos;

  float kk = 1.0f / dot(b, b);
  float kx = kk * dot(a, b);
  float ky = kk * (2.0f * dot(a, a) + dot(d, b)) / 3.0f;
  float kz = kk * dot(d, a);

  float res = 0.0f;
  float pp = ky - kx * kx;
  float qq = kx * (2.0f * kx * kx - 3.0f * ky) + kz;
  float p3 = pp * pp * pp;
  float q2 = qq * qq;
  float h = q2 + 4.0f * p3;

  if (h >= 0.0f) {
    h = sqrt(h);
    h = (qq < 0.0f) ? h : -h;
    float x = (h - qq) / 2.0f;
    float v = sign(x) * pow(abs(x), 1.0f / 3.0f);
    float t = v - pp / v;
    t -= (t * (t * t + 3.0f * pp) + qq) / (3.0f * t * t + 3.0f * pp);
    t = clamp(t - kx, 0.0f, 1.0f);
    float2 w = d + (c + b * t) * t;
    res = dot(w, w);
  }
  else {
    float z = sqrt(-pp);
    float v = acos(qq / (pp * z * 2.0f)) / 3.0f;
    float m = cos(v);
    float n = sin(v) * sqrt(3.0f);
    float3 t3 = clamp(float3(m + m, -n - m, n - m) * z - kx, 0.0f, 1.0f);
    float2 qx = d + (c + b * t3.x) * t3.x;
    float2 qy = d + (c + b * t3.y) * t3.y;
    res = min(dot(qx, qx), dot(qy, qy));
  }

  return sqrt(res);
}

/* Bezier winding number helpers */

void toggleBezierRoot(float t,
                      float2 a,
                      float2 b,
                      float2 c,
                      float2 p,
                      inout int winding)
{
  if (t < 0.0f || t > 1.0f) { return; }
  float u = 1.0f - t;
  float2 q = u * u * a + 2.0f * u * t * b + t * t * c;
  float2 v = 2.0f * (u * (b - a) + t * (c - b));
  float cr = v.x * (p.y - q.y) - v.y * (p.x - q.x);
  if (v.y > 0.0f && p.y != c.y && cr > 0.0f) { winding--; }
  if (v.y < 0.0f && p.y != a.y && cr < 0.0f) { winding++; }
}

void toggleBezierWinding(float2 a,
                         float2 b,
                         float2 c,
                         float2 p,
                         inout int winding)
{
  float y0 = min(min(a.y, b.y), c.y);
  float y1 = max(max(a.y, b.y), c.y);
  if (p.y < y0 || p.y > y1) { return; }

  float A = a.y - 2.0f * b.y + c.y;
  float B = 2.0f * (b.y - a.y);
  float C = a.y - p.y;

  if (abs(A) < 1e-4f) {
    float t = -C / B;
    toggleBezierRoot(t, a, b, c, p, winding);
  }
  else {
    float root = B * B - 4.0f * A * C;
    if (root > 0.0f) {
      float s = sqrt(root);
      toggleBezierRoot((-B - s) / (2.0f * A), a, b, c, p, winding);
      toggleBezierRoot((-B + s) / (2.0f * A), a, b, c, p, winding);
    }
  }
}

/* ---- Arbitrary polygon 2D SDF (straight + bezier edges) ---- */

float sdPolygon2D(float2 p, int ps, int pc)
{
  float d = 1e20f;
  int winding = 0;
  for (int i = 0; i < pc; i++) {
    float4 ed = polygon_points[ps + i].vi_edge;
    float4 ab = polygon_points[ps + i].arc_bounds;
    float2 vi = ed.xy;

    if (ab.w < 0.0f) {
      float4 ad = polygon_points[ps + i].arc_data;
      /* Bezier segment */
      float2 ctrl = ed.zw;
      float2 end_pt = ad.xy;

      /* AABB cull */
      float2 bmin = min(min(vi, ctrl), end_pt);
      float2 bmax = max(max(vi, ctrl), end_pt);
      float2 dbox = abs(p - 0.5f * (bmin + bmax)) - 0.5f * (bmax - bmin);
      float box_d = length(max(dbox, float2(0.0f))) + min(max(dbox.x, dbox.y), 0.0f);
      if (box_d < d) {
        d = min(d, sdBezier2D(vi, ctrl, end_pt, p));
      }

      toggleBezierWinding(vi, ctrl, end_pt, p, winding);
    }
    else {
      /* Straight segment */
      float2 e = ed.zw;
      float2 w = p - vi;
      float2 b = w - e * clamp(dot(w, e) / dot(e, e), 0.0f, 1.0f);
      d = min(d, length(b));

      /* Winding number (non-zero rule) */
      float vj_y = vi.y + e.y;
      float cross_val = e.x * w.y - e.y * w.x;
      if (vi.y <= p.y && vj_y > p.y && cross_val > 0.0f) { winding++; }
      if (vi.y > p.y && vj_y <= p.y && cross_val < 0.0f) { winding--; }
    }
  }
  return (winding != 0) ? -d : d;
}

float sdPolygon2DRounded(float2 p, int ps, int pc)
{
  float d = 1e20f;
  int winding = 0;

  for (int i = 0; i < pc; i++) {
    float4 ed = polygon_points[ps + i].vi_edge;
    float4 ad = polygon_points[ps + i].arc_data;
    float4 ab = polygon_points[ps + i].arc_bounds;
    if (ab.w < 0.0f) { continue; }

    float2 vi = ed.xy, edge = ed.zw;
    float R_signed = ad.x, R = abs(R_signed);
    float2 C = ad.yz;
    float t_start = ad.w, t_end = ab.x;
    float ang_mid = ab.y, ang_half = ab.z;

    /* Trimmed edge: distance + winding */
    float2 seg_a = vi + edge * t_start;
    float2 seg_b = vi + edge * t_end;
    float2 seg_dir = seg_b - seg_a;
    float seg_len_sq = dot(seg_dir, seg_dir);
    if (seg_len_sq > 1e-10f) {
      float2 w = p - seg_a;
      float t = clamp(dot(w, seg_dir) / seg_len_sq, 0.0f, 1.0f);
      d = min(d, length(w - seg_dir * t));

      float cross_val = seg_dir.x * w.y - seg_dir.y * w.x;
      if (seg_a.y <= p.y && seg_b.y > p.y && cross_val > 0.0f) { winding++; }
      if (seg_a.y > p.y && seg_b.y <= p.y && cross_val < 0.0f) { winding--; }
    }

    /* Arc: distance + winding */
    if (R > 0.001f) {
      float2 to_p = p - C;
      float dist_c = length(to_p);
      float ang_p = atan(to_p.y, to_p.x);
      float ang_diff = ang_p - ang_mid;
      ang_diff -= 6.2831853f * floor((ang_diff + 3.1415927f) / 6.2831853f);

      if (abs(ang_diff) <= ang_half) {
        d = min(d, abs(dist_c - R));
      }
      else {
        float2 ep1 = C + R * float2(cos(ang_mid - ang_half), sin(ang_mid - ang_half));
        float2 ep2 = C + R * float2(cos(ang_mid + ang_half), sin(ang_mid + ang_half));
        d = min(d, min(length(p - ep1), length(p - ep2)));
      }

      /* Arc winding: find crossings of arc with horizontal ray y=p.y, x>p.x */
      float k = (p.y - C.y) / R;
      if (abs(k) < 1.0f) {
        float asin_k = asin(k);
        int dir = (R_signed > 0.0f) ? 1 : -1;

        /* Crossing at θ=asin(k), cos>0 (upward for CCW arc) */
        float th = asin_k;
        float td = th - ang_mid;
        td -= 6.2831853f * floor((td + 3.1415927f) / 6.2831853f);
        if (abs(td) <= ang_half && C.x + R * cos(th) > p.x) {
          winding += dir;
        }

        /* Crossing at θ=π-asin(k), cos<0 (downward for CCW arc) */
        th = 3.1415927f - asin_k;
        td = th - ang_mid;
        td -= 6.2831853f * floor((td + 3.1415927f) / 6.2831853f);
        if (abs(td) <= ang_half && C.x + R * cos(th) > p.x) {
          winding -= dir;
        }
      }
    }
  }

  return (winding != 0) ? -d : d;
}

float sdAdvancedPolygon(float3 p,
                        float halfH,
                        int ps,
                        int pc,
                        float edgeTop,
                        float edgeBot,
                        float tapTop,
                        float tapBot,
                        int edgeMode,
                        float taperH)
{
  float zn = clamp(p.z / max(taperH, 0.001f), -1.0f, 1.0f);
  float t = (zn + 1.0f) * 0.5f;
  float tapFactor = max(1.0f - tapTop * t - tapBot * (1.0f - t), 0.001f);

  float slope = (tapTop + tapBot) / (2.0f * max(taperH, 0.001f));
  float lipschitz = sqrt(1.0f + slope * slope);

  float d2d = sdPolygon2DRounded(p.xy / tapFactor, ps, pc) * tapFactor;

  float dz = abs(p.z) - halfH;
  float edgeR = (p.z > 0.0f) ? edgeTop * min(halfH, halfH)
                              : edgeBot * min(halfH, halfH);

  if (edgeR > 0.001f) {
    if (edgeMode == 0) {
      float2 dd = float2(d2d + edgeR, dz + edgeR);
      return (min(max(dd.x, dd.y), 0.0f) + length(max(dd, float2(0.0f))) - edgeR) / lipschitz;
    }
    else {
      float base = max(d2d, dz);
      float cham = (d2d + dz + edgeR) * 0.70710678f;
      float dd = max(base, cham);
      if (dd <= 0.0f) {
        return dd / lipschitz;
      }
      if (d2d <= 0.0f && dz <= 0.0f) {
        return cham / lipschitz;
      }
      if (dz <= -edgeR) {
        return d2d / lipschitz;
      }
      if (d2d <= -edgeR) {
        return dz / lipschitz;
      }
      float tc2 = (-d2d + dz + edgeR) / (2.0f * edgeR);
      if (tc2 <= 0.0f) {
        return length(float2(d2d, dz + edgeR)) / lipschitz;
      }
      if (tc2 >= 1.0f) {
        return length(float2(d2d + edgeR, dz)) / lipschitz;
      }
      return cham / lipschitz;
    }
  }
  else {
    float2 dd = float2(d2d, dz);
    return (length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f)) / lipschitz;
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

/* ---- Color blend factor ---- */

float colorBlendFactor(float d_prev, float d_new, int blend_type, float blend)
{
  if (blend_type > 0 && blend > 0.0f) {
    float h = clamp(0.5f + 0.5f * (d_new - d_prev) / blend, 0.0f, 1.0f);
    return smoothstep(0.0f, 1.0f, 1.0f - h);
  }
  return (d_new < d_prev) ? 1.0f : 0.0f;
}

/* ---- Chamfer blend operations ---- */

float opChamferUnion(float a, float b, float r)
{
  return min(min(a, b), (a - r + b) * 0.5f);
}

float opChamferIntersection(float a, float b, float r)
{
  return max(max(a, b), (a + r + b) * 0.5f);
}

float opChamferSubtraction(float d1, float d2, float r)
{
  return opChamferIntersection(d2, -d1, r);
}

/* ---- Smooth chamfer (k2/k3 control edge softness) ---- */

float opSmoothChamferUnion(float d1, float d2, float k, float k2, float k3)
{
  float chamfer_plane = (d1 + d2 - k) * 0.5f;
  float term1 = opSmoothUnion(d1, chamfer_plane, k2);
  float term2 = opSmoothUnion(d2, chamfer_plane, k3);
  return min(term1, term2);
}

float opSmoothChamferSubtraction(float d1, float d2, float k, float k2, float k3)
{
  float A = -d1;
  float B = d2;
  float chamfer_plane = (A + B + k) * 0.5f;
  float term1 = opSmoothIntersection(A, chamfer_plane, k2);
  float term2 = opSmoothIntersection(B, chamfer_plane, k3);
  return max(term1, term2);
}

float opSmoothChamferIntersection(float d1, float d2, float k, float k2, float k3)
{
  float chamfer_plane = (d1 + d2 + k) * 0.5f;
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

/* ---- Smooth spherical round (k2/k3 control edge softness) ---- */

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
float4 applyDomainModifiers(float3 p, int mod_start, int mod_count, float4x4 inv_mat)
{
  float scale = 1.0f;
  for (int i = mod_start + mod_count - 1; i >= mod_start; i--) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;
    int mflags = smod.header.y;

    if (mtype == SDF_MOD_MIRROR) {
      float offset = smod.params.x;
      float3 origin = smod.params.yzw;
      if ((mflags & SDF_MOD_MIRROR_X) != 0) {
        float3 N = float3(inv_mat[0]);
        float d = dot(p - origin, N);
        p -= 2.0f * min(d, 0.0f) * N;
        p -= offset * N;
      }
      if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
        float3 N = float3(inv_mat[1]);
        float d = dot(p - origin, N);
        p -= 2.0f * min(d, 0.0f) * N;
        p -= offset * N;
      }
      if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
        float3 N = float3(inv_mat[2]);
        float d = dot(p - origin, N);
        p -= 2.0f * min(d, 0.0f) * N;
        p -= offset * N;
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
      int axis = int(smod.params.y);
      if (abs(k) > 0.0001f) {
        float R = 1.0f / k;
        float sg = sign(R);
        float absR = abs(R);
        float r;
        if (axis == 1) {
          float2 rel = float2(p.y, p.z + R);
          r = length(rel);
          p.y = atan(rel.x * sg, rel.y * sg) / k;
          p.z = sg * r - R;
        }
        else if (axis == 2) {
          float2 rel = float2(p.z, p.x + R);
          r = length(rel);
          p.z = atan(rel.x * sg, rel.y * sg) / k;
          p.x = sg * r - R;
        }
        else {
          float2 rel = float2(p.x, p.y + R);
          r = length(rel);
          p.x = atan(rel.x * sg, rel.y * sg) / k;
          p.y = sg * r - R;
        }
        scale *= min(1.0f, absR / max(r, 0.0001f));
      }
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

          float3 rot = smod.params2.yzw;
          if (abs(rot.x) > 0.0001f) {
            float cx = cos(rot.x), sx = sin(rot.x);
            p = float3(p.x, cx * p.y - sx * p.z, sx * p.y + cx * p.z);
          }
          if (abs(rot.y) > 0.0001f) {
            float cy = cos(rot.y), sy = sin(rot.y);
            p = float3(cy * p.x + sy * p.z, p.y, -sy * p.x + cy * p.z);
          }
          if (abs(rot.z) > 0.0001f) {
            float cz = cos(rot.z), sz = sin(rot.z);
            p = float3(cz * p.x - sz * p.y, sz * p.x + cz * p.y, p.z);
          }
        }
      }
    }
  }
  return float4(p, scale);
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

/* SDFPrimitiveData removed — eval functions take SDFObjectGPU directly. */

/* ---- CSG dispatch ---- */

/** CSG operation IDs (must match eSDFCSGOperation in DNA_sdf_types.h). */
#define SDF_CSG_OP_UNION 0
#define SDF_CSG_OP_SUBTRACT 1
#define SDF_CSG_OP_INTERSECT 2
#define SDF_CSG_OP_SHELL 3
#define SDF_CSG_OP_PUSH 4
#define SDF_CSG_OP_AVOID 5

/** Shell mode IDs (must match eSDFShellMode in DNA_sdf_types.h). */
#define SDF_SHELL_MODE_NORMAL 0
#define SDF_SHELL_MODE_PUSH 1
#define SDF_SHELL_MODE_AVOID 2

#define SDF_SHELL_OP_UNION 0
#define SDF_SHELL_OP_SUBTRACTION 1

/** Blend type IDs (must match eSDFBlendType in DNA_sdf_types.h). */
#define SDF_BLEND_TYPE_LINEAR 0
#define SDF_BLEND_TYPE_SMOOTH 1
#define SDF_BLEND_TYPE_CHAMFER 2
#define SDF_BLEND_TYPE_ROUND 3

/**
 * Combine two SDF distances using the specified CSG operation and blend type.
 * \param k2: chamfer/round edge smoothness for shape 1 (0 = sharp).
 * \param k3: chamfer/round edge smoothness for shape 2 (0 = sharp).
 */
float combineCSG(float d1, float d2, int op, int bt, float k,
                 float shell_dist, int shell_mode, int shell_op,
                 float shell_k_top, float shell_k_bot,
                 float k2, float k3)
{
  bool has_smooth = (k2 > 0.0f || k3 > 0.0f);

  if (op == SDF_CSG_OP_UNION) {
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        return opSmoothUnion(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        if (has_smooth) { return opSmoothChamferUnion(d1, d2, k, k2, k3); }
        return opChamferUnion(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        if (has_smooth) { return opSmoothRoundUnion(d1, d2, k, k2, k3); }
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
        if (has_smooth) { return opSmoothChamferSubtraction(d2, d1, k, k2, k3); }
        return opChamferSubtraction(d2, d1, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        if (has_smooth) { return opSmoothRoundSubtraction(d2, d1, k, k2, k3); }
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
        if (has_smooth) { return opSmoothChamferIntersection(d1, d2, k, k2, k3); }
        return opChamferIntersection(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        if (has_smooth) { return opSmoothRoundIntersection(d1, d2, k, k2, k3); }
        return opRoundIntersection(d1, d2, k);
      }
    }
    return max(d1, d2);
  }
  else if (op == SDF_CSG_OP_PUSH) {
    float subtracted;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        subtracted = opSmoothSubtraction(d2, d1, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        if (has_smooth) { subtracted = opSmoothChamferSubtraction(d2, d1, k, k2, k3); }
        else { subtracted = opChamferSubtraction(d2, d1, k); }
      }
      else {
        if (has_smooth) { subtracted = opSmoothRoundSubtraction(d2, d1, k, k2, k3); }
        else { subtracted = opRoundSubtraction(d2, d1, k); }
      }
    }
    else {
      subtracted = max(d1, -d2);
    }
    return min(subtracted, d2);
  }
  else if (op == SDF_CSG_OP_AVOID) {
    float carved;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        carved = opSmoothSubtraction(d1, d2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        if (has_smooth) { carved = opSmoothChamferSubtraction(d1, d2, k, k2, k3); }
        else { carved = opChamferSubtraction(d1, d2, k); }
      }
      else {
        if (has_smooth) { carved = opSmoothRoundSubtraction(d1, d2, k, k2, k3); }
        else { carved = opRoundSubtraction(d1, d2, k); }
      }
    }
    else {
      carved = max(d2, -d1);
    }
    return min(d1, carved);
  }
  else if (op == SDF_CSG_OP_SHELL) {
    /* Subtraction flips direction: positive dist carves inward instead of outward. */
    float sd = (shell_op == SDF_SHELL_OP_SUBTRACTION) ? -shell_dist : shell_dist;
    float h = abs(sd);

    if (shell_mode == SDF_SHELL_MODE_PUSH) {
      /* Standalone shell field around d2, then push into d1. */
      float d_shell = abs(d2) - h;
      float subtracted;
      if (k > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) {
          subtracted = opSmoothSubtraction(d_shell, d1, k);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          subtracted = opChamferSubtraction(d_shell, d1, k);
        }
        else {
          subtracted = opRoundSubtraction(d_shell, d1, k);
        }
      }
      else {
        subtracted = max(d1, -d_shell);
      }
      return min(subtracted, d_shell);
    }

    /* Normal shell and Avoid: compute the two-stage shell result. */
    float lk_top = min(shell_k_top, h);
    float lk_bot = min(shell_k_bot, h);
    float d_shell;
    if (sd < 0.0f) {
      /* Inward (or subtraction): subtract shape, cap at limit */
      float d_sub;
      if (shell_k_bot > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { d_sub = opSmoothSubtraction(d2, d1, shell_k_bot); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { d_sub = opChamferSubtraction(d2, d1, shell_k_bot); }
        else if (bt == SDF_BLEND_TYPE_ROUND) { d_sub = opRoundSubtraction(d2, d1, shell_k_bot); }
        else { d_sub = max(d1, -d2); }
      }
      else { d_sub = max(d1, -d2); }
      float lim = d1 + h;
      if (lk_top > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { d_shell = opSmoothUnion(d_sub, lim, lk_top); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { d_shell = opChamferUnion(d_sub, lim, lk_top); }
        else if (bt == SDF_BLEND_TYPE_ROUND) { d_shell = opRoundUnion(d_sub, lim, lk_top); }
        else { d_shell = min(d_sub, lim); }
      }
      else { d_shell = min(d_sub, lim); }
    }
    else {
      /* Outward: union = top edge, intersection = bottom edge */
      float d_union;
      if (shell_k_top > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { d_union = opSmoothUnion(d1, d2, shell_k_top); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { d_union = opChamferUnion(d1, d2, shell_k_top); }
        else if (bt == SDF_BLEND_TYPE_ROUND) { d_union = opRoundUnion(d1, d2, shell_k_top); }
        else { d_union = min(d1, d2); }
      }
      else { d_union = min(d1, d2); }
      float lim = d1 - h;
      if (lk_bot > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { d_shell = opSmoothIntersection(d_union, lim, lk_bot); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { d_shell = opChamferIntersection(d_union, lim, lk_bot); }
        else if (bt == SDF_BLEND_TYPE_ROUND) { d_shell = opRoundIntersection(d_union, lim, lk_bot); }
        else { d_shell = max(d_union, lim); }
      }
      else { d_shell = max(d_union, lim); }
    }

    if (shell_mode == SDF_SHELL_MODE_AVOID) {
      /* Full shell computed, now carve it by d1. */
      float carved;
      if (k > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) {
          carved = opSmoothSubtraction(d1, d_shell, k);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          carved = opChamferSubtraction(d1, d_shell, k);
        }
        else {
          carved = opRoundSubtraction(d1, d_shell, k);
        }
      }
      else {
        carved = max(d_shell, -d1);
      }
      return min(d1, carved);
    }

    return d_shell;
  }
  return d1; /* Fallback for unknown ops. */
}

float evalPrimitiveOnly(SDFObjectGPU obj, float3 local_pos)
{
  float3 size = obj.sdf_size.xyz;
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
  else if (obj.sdf_type == 7) { /* POLYGON */
    float halfH = max(size.z - bevel, 0.001f);
    int ps = obj.polygon_point_start;
    int pc = obj.polygon_point_count;
    float maxCorner = obj.box_corners.x;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    int edgeMode = obj.box_modes.y;
    bool hasEdgeTaper = (edgeTop + edgeBot + tapTop + tapBot) > 0.001f;
    if (pc >= 3) {
      if (hasEdgeTaper || maxCorner > 0.001f) {
        dist = sdAdvancedPolygon(local_pos, halfH, ps, pc, edgeTop, edgeBot, tapTop, tapBot, edgeMode, halfH);
      }
      else {
        float d2d = sdPolygon2D(local_pos.xy, ps, pc);
        float dz = abs(local_pos.z) - halfH;
        float2 dd = float2(d2d, dz);
        dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
      }
    }
    else {
      dist = 1e10f;
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
float evalObjectSDF(SDFObjectGPU obj, float3 p)
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
    float4 dm = applyDomainModifiers(p, obj.modifier_start, obj.modifier_count, obj.inverse_matrix);
    p = dm.xyz;
    float d = evalPrimitiveOnly(obj, p) * dm.w;
    return applyDistanceModifiers(d, obj.modifier_start, obj.modifier_count);
  }

  /* Forking path: apply domain modifiers UP TO the forking modifier. */
  float top_scale = 1.0f;
  int top_count = (obj.modifier_start + obj.modifier_count) - (fork_idx + 1);
  if (top_count > 0) {
    float4 dm = applyDomainModifiers(p, fork_idx + 1, top_count, obj.inverse_matrix);
    p = dm.xyz;
    top_scale = dm.w;
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
    if ((mflags & SDF_MOD_MIRROR_X) != 0) { mirrors *= 2; }
    if ((mflags & SDF_MOD_MIRROR_Y) != 0) { mirrors *= 2; }
    if ((mflags & SDF_MOD_MIRROR_Z) != 0) { mirrors *= 2; }

    float3 mirror_origin = fork_mod.params.yzw;
    float offset = fork_mod.params.x;
    float3 N_x = float3(obj.inverse_matrix[0]);
    float3 N_y = float3(obj.inverse_matrix[1]);
    float3 N_z = float3(obj.inverse_matrix[2]);

    bool first = true;
    for (int i = 0; i < 8; i++) {
      if (i >= mirrors) { break; }
      float3 cell_p = p;
      int flip_idx = i;
      if ((mflags & SDF_MOD_MIRROR_X) != 0) {
        if ((flip_idx & 1) != 0) {
          float d = dot(cell_p - mirror_origin, N_x);
          cell_p -= 2.0f * d * N_x;
        }
        flip_idx >>= 1;
        cell_p -= offset * N_x;
      }
      if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
        if ((flip_idx & 1) != 0) {
          float d = dot(cell_p - mirror_origin, N_y);
          cell_p -= 2.0f * d * N_y;
        }
        flip_idx >>= 1;
        cell_p -= offset * N_y;
      }
      if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
        if ((flip_idx & 1) != 0) {
          float d = dot(cell_p - mirror_origin, N_z);
          cell_p -= 2.0f * d * N_z;
        }
        cell_p -= offset * N_z;
      }

      float bot_scale = top_scale;
      if (bottom_count > 0) {
        float4 dm = applyDomainModifiers(cell_p, obj.modifier_start, bottom_count, obj.inverse_matrix);
        cell_p = dm.xyz;
        bot_scale *= dm.w;
      }
      float d = applyDistanceModifiers(evalPrimitiveOnly(obj, cell_p) * bot_scale, obj.modifier_start, obj.modifier_count);

      if (first) { final_d = d; first = false; }
      else { final_d = combineCSG(final_d, d, csg_op, blend_type, blend, 0.0f, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f); }
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
      } else { id = 0.0f; }
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
      if (cid == last_cid) { continue; }
      last_cid = cid;

      float3 cell_p = p;
      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        if (spacing > 0.0001f) { cell_p -= dir * cid * spacing; }
      } else {
        float a = (2.0f * SDF_PI) / count;
        float final_a = atan(p.y, p.x) - cid * a;
        float r = length(p.xy);
        float radius = fork_mod.params.y;
        cell_p.x = r * cos(final_a) - radius;
        cell_p.y = r * sin(final_a);

        float3 rot = fork_mod.params2.yzw;
        if (abs(rot.x) > 0.0001f) {
          float cx = cos(rot.x), sx = sin(rot.x);
          cell_p = float3(cell_p.x, cx * cell_p.y - sx * cell_p.z, sx * cell_p.y + cx * cell_p.z);
        }
        if (abs(rot.y) > 0.0001f) {
          float cy = cos(rot.y), sy = sin(rot.y);
          cell_p = float3(cy * cell_p.x + sy * cell_p.z, cell_p.y, -sy * cell_p.x + cy * cell_p.z);
        }
        if (abs(rot.z) > 0.0001f) {
          float cz = cos(rot.z), sz = sin(rot.z);
          cell_p = float3(cz * cell_p.x - sz * cell_p.y, sz * cell_p.x + cz * cell_p.y, cell_p.z);
        }
      }

      float bot_scale = top_scale;
      if (bottom_count > 0) {
        float4 dm = applyDomainModifiers(cell_p, obj.modifier_start, bottom_count, obj.inverse_matrix);
        cell_p = dm.xyz;
        bot_scale *= dm.w;
      }
      float d = applyDistanceModifiers(evalPrimitiveOnly(obj, cell_p) * bot_scale, obj.modifier_start, obj.modifier_count);

      if (first) { final_d = d; first = false; }
      else { final_d = combineCSG(final_d, d, csg_op, blend_type, blend, 0.0f, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f); }
    }
  }

  return final_d;
}

/* ---- Cross-shader helpers ---- */

float evalPrimitive(float3 local_pos, SDFObjectGPU obj)
{
  return evalObjectSDF(obj, local_pos);
}

void flushGroup(int gid, float grp_dist, float3 grp_color,
                inout float scene_dist, inout float3 out_color)
{
  if (scene_dist >= 1e9f) {
    scene_dist = grp_dist;
    out_color = grp_color;
  }
  else {
    SDFGroupGPU grp = groups[gid];
    float prev = scene_dist;
    scene_dist = combineCSG(
        scene_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend,
        grp.shell_distance, grp.shell_mode, grp.shell_op,
        grp.shell_blend_top, grp.shell_blend_bottom,
        grp.chamfer_k2, grp.chamfer_k3);
    if (grp.csg_operation == 0) {
      float t = colorBlendFactor(prev, grp_dist, grp.blend_type, grp.blend);
      out_color = mix(out_color, grp_color, t);
    }
  }
}

/** \} */

/** \} */

