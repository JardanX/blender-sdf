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

  /* Lipschitz correction: at box corners both X and Y taper slopes contribute
   * to the Z gradient, giving magnitude sqrt(1 + slope_x^2 + slope_y^2).
   * Use length(size.xy) to capture the worst-case corner gradient. */
  float slope = length(size.xy) * (tapTop + tapBot) / (2.0f * max(taperZ, 0.001f));
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

  float kk = 1.0f / max(dot(b, b), 1e-12f);
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
  float edgeR = (p.z > 0.0f) ? edgeTop * halfH
                              : edgeBot * halfH;

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

/** CSG operation IDs (must match eSDFCSGOperation in DNA_sdf_types.h). */
#define SDF_CSG_OP_UNION 0
#define SDF_CSG_OP_SUBTRACT 1
#define SDF_CSG_OP_INTERSECT 2
#define SDF_CSG_OP_SHELL 3
#define SDF_CSG_OP_PUSH 4
#define SDF_CSG_OP_AVOID 5

/** Shell op IDs (must match eSDFShellOp in DNA_sdf_types.h). */
#define SDF_SHELL_OP_UNION 0
#define SDF_SHELL_OP_SUBTRACTION 1

/* ---- Color blend factor ---- */

float colorBlendFactor(float d_prev, float d_new, int blend_type, float blend)
{
  if (blend_type > 0 && blend > 0.0f) {
    float h = clamp(0.5f + 0.5f * (d_new - d_prev) / blend, 0.0f, 1.0f);
    return smoothstep(0.0f, 1.0f, 1.0f - h);
  }
  return (d_new < d_prev) ? 1.0f : 0.0f;
}

/* CSG-aware color blend factor: returns how much of the NEW shape's color to mix in. */
float csgColorFactor(float d_prev, float d_new, int csg_op, int blend_type, float blend,
                     float shell_dist, int shell_op)
{
  bool has_blend = (blend_type > 0 && blend > 0.0f);

  if (csg_op == SDF_CSG_OP_SUBTRACT) {
    /* Carved surface shows the subtractor's color in the blend zone. */
    if (!has_blend) { return (-d_new > d_prev) ? 1.0f : 0.0f; }
    float h = clamp(0.5f - 0.5f * (d_prev + d_new) / blend, 0.0f, 1.0f);
    return smoothstep(0.0f, 1.0f, h);
  }
  else if (csg_op == SDF_CSG_OP_INTERSECT) {
    /* Intersection: color follows the constraining (farther) shape. */
    if (!has_blend) { return (d_new > d_prev) ? 1.0f : 0.0f; }
    float h = clamp(0.5f - 0.5f * (d_new - d_prev) / blend, 0.0f, 1.0f);
    return smoothstep(0.0f, 1.0f, 1.0f - h);
  }
  else if (csg_op == SDF_CSG_OP_SHELL) {
    /* Shell band: abs(d_new) < thickness means inside the shell region. */
    float sd = (shell_op == SDF_SHELL_OP_SUBTRACTION) ? -shell_dist : shell_dist;
    float thickness = abs(sd);
    float d_from_band = abs(d_new) - thickness;
    if (!has_blend) { return (d_from_band < 0.0f) ? 1.0f : 0.0f; }
    return smoothstep(0.0f, 1.0f, clamp(-d_from_band / max(blend, 0.001f), 0.0f, 1.0f));
  }

  /* Union, Push, Avoid: color follows whichever shape is closer. */
  return colorBlendFactor(d_prev, d_new, blend_type, blend);
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

/* Inverted round: convex fillet via subtraction + min */
float opRoundUnionInverted(float a, float b, float r)
{
  float diff = -opUnionIRound(a, -b, r);
  return min(diff, b);
}

float opSmoothRoundUnionInverted(float a, float b, float r, float k2, float k3)
{
  float diff = opSmoothRoundSubtraction(b, a, r, k2, k3);
  return min(diff, b);
}

float opIntersectionRound(float a, float b, float r)
{
  float2 u = max(float2(r + a, r + b), float2(0.0f));
  return min(-r, max(a, b)) + length(u);
}

float opSmoothRoundIntersectionInverted(float d1, float d2, float r, float k2, float k3)
{
  float a = d1;
  float b = -d2;
  float2 s = float2(min(a, 0.0f), max(b, 0.0f));
  float corner = r - length(s);
  float term1 = opSmoothIntersection(a, corner, k2);
  float term2 = opSmoothIntersection(-b, corner, k3);
  return max(term1, term2);
}

/* ---- SDF modifier evaluation ---- */

/** Modifier type IDs (must match eSDFModifierType in DNA_sdf_types.h). */
#define SDF_MOD_MIRROR 0
#define SDF_MOD_TWIST 1
#define SDF_MOD_BEND 2
#define SDF_MOD_ELONGATE 3
#define SDF_MOD_SOLIDIFY 4
#define SDF_MOD_ROUND 5
#define SDF_MOD_ONION 6
#define SDF_MOD_BEVEL 7
#define SDF_MOD_ARRAY 8
#define SDF_MOD_DISPLACE 9

/** Mirror axis flags. */
#define SDF_MOD_MIRROR_X 1
#define SDF_MOD_MIRROR_Y 2
#define SDF_MOD_MIRROR_Z 4

/** Array types. */
#define SDF_MOD_ARRAY_LINEAR 0
#define SDF_MOD_ARRAY_RADIAL 1

/** Displacement noise types. */
#define SDF_MOD_DISPLACE_NOISE 0
#define SDF_MOD_DISPLACE_VORONOI 1
#define SDF_MOD_DISPLACE_TRIANGLE 2
#define SDF_MOD_DISPLACE_POINTS 3

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

/* ---- Noise functions for displacement ---- */

/* Fract-based 3D hash — fast, no sin(), good distribution */
float3 sdf_hash33(float3 p)
{
  p = fract(p * float3(0.1031f, 0.1030f, 0.0973f));
  p += dot(p, p.yxz + 33.33f);
  return fract((p.xxy + p.yxx) * p.zyx);
}

float sdf_hash31(float3 p)
{
  p = fract(p * 0.1031f);
  p += dot(p, p.zyx + 31.32f);
  return fract((p.x + p.y) * p.z);
}

/* Value noise — quintic interpolation */
float sdf_value_noise(float3 p)
{
  float3 i = floor(p);
  float3 f = fract(p);
  float3 u = f * f * f * (f * (f * 6.0f - 15.0f) + 10.0f);

  float a = sdf_hash31(i);
  float b = sdf_hash31(i + float3(1, 0, 0));
  float c = sdf_hash31(i + float3(0, 1, 0));
  float d = sdf_hash31(i + float3(1, 1, 0));
  float e = sdf_hash31(i + float3(0, 0, 1));
  float ff = sdf_hash31(i + float3(1, 0, 1));
  float g = sdf_hash31(i + float3(0, 1, 1));
  float h = sdf_hash31(i + float3(1, 1, 1));

  return mix(mix(mix(a, b, u.x), mix(c, d, u.x), u.y),
             mix(mix(e, ff, u.x), mix(g, h, u.x), u.y), u.z) * 2.0f - 1.0f;
}

float sdf_fbm_noise(float3 p, int octaves, float lacunarity, float roughness)
{
  float sum = 0.0f;
  float amp = 1.0f;
  float max_amp = 0.0f;
  for (int i = 0; i < octaves; i++) {
    sum += sdf_value_noise(p) * amp;
    max_amp += amp;
    p *= lacunarity;
    amp *= roughness;
  }
  return sum / max_amp;
}

/* 3D voronoi with analytical gradient — single pass */
float4 sdf_voronoi_grad(float3 p)
{
  float3 i = floor(p);
  float3 f = fract(p);
  float min_d2 = 100.0f;
  float3 nearest_r = float3(0);
  for (int z = -1; z <= 1; z++) {
    for (int y = -1; y <= 1; y++) {
      for (int x = -1; x <= 1; x++) {
        float3 b = float3(float(x), float(y), float(z));
        float3 r = b - f + sdf_hash33(i + b);
        float d2 = dot(r, r);
        if (d2 < min_d2) {
          min_d2 = d2;
          nearest_r = r;
        }
      }
    }
  }
  float d = sqrt(min_d2);
  float3 grad = nearest_r / max(d, 1e-8f);
  return float4(d, grad);
}

float sdf_voronoi(float3 p) { return sdf_voronoi_grad(p).x; }

float sdf_fbm_voronoi(float3 p, int octaves, float lacunarity, float roughness)
{
  float sum = 0.0f;
  float amp = 1.0f;
  float max_amp = 0.0f;
  for (int i = 0; i < octaves; i++) {
    sum += (1.0f - sdf_voronoi(p)) * amp;
    max_amp += amp;
    p *= lacunarity;
    amp *= roughness;
  }
  return sum / max_amp;
}

/* Analytical voronoi FBM gradient — avoids finite differences */
float4 sdf_fbm_voronoi_grad(float3 p, int octaves, float lacunarity, float roughness)
{
  float sum = 0.0f;
  float3 gsum = float3(0);
  float amp = 1.0f;
  float freq = 1.0f;
  float max_amp = 0.0f;
  for (int i = 0; i < octaves; i++) {
    float4 vg = sdf_voronoi_grad(p * freq);
    sum += (1.0f - vg.x) * amp;
    gsum += vg.yzw * amp * freq;
    max_amp += amp;
    freq *= lacunarity;
    amp *= roughness;
  }
  float inv_ma = 1.0f / max_amp;
  return float4(sum * inv_ma, gsum * inv_ma);
}

/* Diamond knurl grid — MathOPS sdDiamondGrid */
float sdf_diamond_2d(float2 p)
{
  float2 cell = fract(p) - 0.5f;
  return 1.0f - (abs(cell.x) + abs(cell.y)) * 2.0f;
}

float sdf_triangle_grid(float3 p)
{
  float n1 = sdf_diamond_2d(p.xy);
  float n2 = sdf_diamond_2d(p.yz);
  float n3 = sdf_diamond_2d(p.zx);
  float n = max(n1, max(n2, n3));
  float c = clamp(n, 0.0f, 1.0f);
  return c * c;
}

/* Points — 3D spherical dots in a cubic grid */
float sdf_points(float3 p)
{
  float3 f = fract(p) - 0.5f;
  float d = length(f);
  return smoothstep(0.45f, 0.0f, d);
}

float sdf_fbm_triangle(float3 p, int octaves, float lacunarity, float roughness)
{
  float sum = 0.0f;
  float amp = 1.0f;
  float max_amp = 0.0f;
  for (int i = 0; i < octaves; i++) {
    sum += sdf_triangle_grid(p) * amp;
    max_amp += amp;
    p *= lacunarity;
    amp *= roughness;
  }
  return sum / max_amp;
}

/* Full quality displacement (used by gradient/normal pass) */
float sdf_displacement(float3 p, int noise_type, int octaves, float lacunarity, float roughness)
{
  if (noise_type == SDF_MOD_DISPLACE_VORONOI) {
    return sdf_fbm_voronoi(p, octaves, lacunarity, roughness);
  }
  if (noise_type == SDF_MOD_DISPLACE_TRIANGLE) {
    return sdf_fbm_triangle(p, octaves, lacunarity, roughness);
  }
  if (noise_type == SDF_MOD_DISPLACE_POINTS) {
    return sdf_points(p);
  }
  return sdf_fbm_noise(p, octaves, lacunarity, roughness);
}

/* Cheaper displacement for ray march — same noise type, fewer octaves */
float sdf_displacement_fast(float3 p, int noise_type, int octaves, float lacunarity, float roughness)
{
  int fast_oct = max(min(octaves, 2), 1);
  if (noise_type == SDF_MOD_DISPLACE_VORONOI) {
    return sdf_fbm_voronoi(p, 1, lacunarity, roughness);
  }
  if (noise_type == SDF_MOD_DISPLACE_TRIANGLE) {
    return sdf_fbm_triangle(p, fast_oct, lacunarity, roughness);
  }
  if (noise_type == SDF_MOD_DISPLACE_POINTS) {
    return sdf_points(p);
  }
  return sdf_fbm_noise(p, fast_oct, lacunarity, roughness);
}

/* Gradient: analytical for voronoi, finite-difference for others */
float4 sdf_displacement_grad(float3 p, int noise_type, int octaves,
                             float lacunarity, float roughness)
{
  if (noise_type == SDF_MOD_DISPLACE_VORONOI) {
    return sdf_fbm_voronoi_grad(p, octaves, lacunarity, roughness);
  }
  const float e = 0.002f;
  float val = sdf_displacement(p, noise_type, octaves, lacunarity, roughness);
  float inv_e = 1.0f / e;
  float gx = (sdf_displacement(p + float3(e, 0, 0), noise_type, octaves, lacunarity, roughness) - val) * inv_e;
  float gy = (sdf_displacement(p + float3(0, e, 0), noise_type, octaves, lacunarity, roughness) - val) * inv_e;
  float gz = (sdf_displacement(p + float3(0, 0, e), noise_type, octaves, lacunarity, roughness) - val) * inv_e;
  return float4(val, gx, gy, gz);
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
      float blend = smod.params2.x;
      int blend_type = smod.header.z;
      int sides = smod.header.w;
      float bk = (blend_type > 0 && blend > 0.001f) ? blend : 0.0f;
      if ((mflags & SDF_MOD_MIRROR_X) != 0) {
        float3 N = float3(inv_mat[0]);
        float s = ((sides & 1) != 0) ? 1.0f : -1.0f;
        N *= s;
        float nl2 = max(dot(N, N), 1e-12f);
        float d = dot(p - origin, N) / nl2;
        p -= (d - sabs(d, bk)) * N;
        p -= offset * N;
      }
      if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
        float3 N = float3(inv_mat[1]);
        float s = ((sides & 2) != 0) ? 1.0f : -1.0f;
        N *= s;
        float nl2 = max(dot(N, N), 1e-12f);
        float d = dot(p - origin, N) / nl2;
        p -= (d - sabs(d, bk)) * N;
        p -= offset * N;
      }
      if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
        float3 N = float3(inv_mat[2]);
        float s = ((sides & 4) != 0) ? 1.0f : -1.0f;
        N *= s;
        float nl2 = max(dot(N, N), 1e-12f);
        float d = dot(p - origin, N) / nl2;
        p -= (d - sabs(d, bk)) * N;
        p -= offset * N;
      }
      if (bk > 0.001f) { scale *= 0.5f; }
    }
    else if (mtype == SDF_MOD_TWIST) {
      float k = smod.params.x;
      int axis = int(smod.params.y);
      float drive, r;
      if (axis == 1) { drive = p.y; r = length(float2(p.x, p.z)); }
      else if (axis == 2) { drive = p.x; r = length(float2(p.y, p.z)); }
      else { drive = p.z; r = length(float2(p.x, p.y)); }
      float angle = k * drive;
      float c = cos(angle);
      float s = sin(angle);
      if (axis == 1) { p = float3(c * p.x - s * p.z, p.y, s * p.x + c * p.z); }
      else if (axis == 2) { p = float3(p.x, c * p.y - s * p.z, s * p.y + c * p.z); }
      else { p = float3(c * p.x - s * p.y, s * p.x + c * p.y, p.z); }
      scale *= 1.0f / (1.0f + abs(k) * r);
    }
    else if (mtype == SDF_MOD_BEND) {
      float k = smod.params.x;
      int axis = int(smod.params.y);
      float3 origin = float3(smod.params.z, smod.params.w, smod.params2.x);
      p -= origin;
      if (abs(k) > 0.0001f) {
        float drive, curve, r;
        if (axis == 1) {
          drive = p.y; curve = p.z; r = length(float2(p.y, p.z));
          float a = k * drive; float c = cos(a); float s = sin(a);
          p.y = c * drive - s * curve;
          p.z = s * drive + c * curve;
        }
        else if (axis == 2) {
          drive = p.z; curve = p.x; r = length(float2(p.z, p.x));
          float a = k * drive; float c = cos(a); float s = sin(a);
          p.z = c * drive - s * curve;
          p.x = s * drive + c * curve;
        }
        else {
          drive = p.x; curve = p.y; r = length(float2(p.x, p.y));
          float a = k * drive; float c = cos(a); float s = sin(a);
          p.x = c * drive - s * curve;
          p.y = s * drive + c * curve;
        }
        scale *= 1.0f / (1.0f + abs(k) * r);
      }
      p += origin;
    }
    else if (mtype == SDF_MOD_ELONGATE) {
      float3 h = smod.params.xyz;
      p = p - clamp(p, -h, h);
    }
    else if (mtype == SDF_MOD_ARRAY) {
      float count = smod.params.x;
      float blend = smod.params2.x;
      /* Array domain blend is always smooth. */
      float bk = (blend > 0.001f) ? blend : 0.0f;
      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        float3 offset = smod.params.yzw;
        float spacing = length(offset);
        if (spacing > 0.0001f && count > 0.5f) {
          float3 dir = offset / spacing;
          float t = dot(p, dir);
          float norm_t = t / spacing;
          float id = clamp(round(norm_t), 0.0f, count - 1.0f);
          float local = norm_t - id;

          bool mirrored = false;
          if (count > 1.5f && fract(id * 0.5f) > 0.25f) {
            local = -local;
            mirrored = true;
          }

          if (count > 1.5f) {
            float d_r = (0.5f - local) * spacing;
            float pull_r = d_r - sabs(d_r, bk);
            float d_l = (0.5f + local) * spacing;
            float pull_l = d_l - sabs(d_l, bk);
            if (id < 0.5f) { pull_l = 0.0f; }
            if (id > count - 1.5f) {
              if (mirrored) { pull_l = 0.0f; }
              else { pull_r = 0.0f; }
            }
            local += (pull_r - pull_l) / spacing;
          }

          p += dir * (local * spacing - t);
        }
      }
      else if (mflags == SDF_MOD_ARRAY_RADIAL) {
        float radius = smod.params.y;
        if (count > 1.5f) {
          float sector = (2.0f * SDF_PI) / count;
          float angle = atan(p.y, p.x);
          float norm_a = angle / sector;
          float id = round(norm_a);
          float local = norm_a - id;

          {
            bool mir = fract(abs(id) * 0.5f) > 0.25f;
            bool odd = fract(count * 0.5f) > 0.25f;
            /* For odd count, the cell touching ±π has no mirror partner.
             * Use abs() for the entire cell to avoid an intra-cell seam. */
            bool at_defect = odd && (abs(angle) > SDF_PI - sector * 0.5f);
            if (at_defect) {
              local = abs(local);
            }
            else if (mir) {
              local = -local;
            }

            float arc = sector * max(radius, 0.0001f);
            float d_r = (0.5f - local) * arc;
            float pull_r = d_r - sabs(d_r, bk);
            float d_l = (0.5f + local) * arc;
            float pull_l = d_l - sabs(d_l, bk);
            local += (pull_r - pull_l) / arc;
            if (bk > 0.001f) { scale *= 0.5f; }
          }
          float fold_a = local * sector;
          float r = length(p.xy);
          p.x = r * cos(fold_a) - radius;
          p.y = r * sin(fold_a);
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
 * Cone frustum SDF: bottom radius rb at z=-h, top radius rt at z=+h.
 * rt = 0 gives a sharp-apex cone. Inigo Quilez two-radius capped cone.
 * Must match CPU sdConeFrustum() and gradient sdgConeFrustum().
 */
float sdConeFrustum(float3 p, float rb, float rt, float h)
{
  float2 q = float2(length(p.xy), p.z);
  float2 k1 = float2(rt, h);
  float2 k2 = float2(rt - rb, 2.0f * h);
  float2 ca = float2(q.x - min(q.x, (q.y < 0.0f) ? rb : rt), abs(q.y) - h);
  float2 cb = q - k1 + k2 * clamp(dot(k1 - q, k2) / dot(k2, k2), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * sqrt(min(dot(ca, ca), dot(cb, cb)));
}

float sdCone(float3 p, float r, float h)
{
  return sdConeFrustum(p, r, 0.0f, h);
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
 * Ellipsoid SDF — Quilez gradient-corrected approximation.
 */
float sdEllipsoid(float3 p, float3 r)
{
  float k0 = length(p / r);
  if (k0 < 0.0001f) {
    return -min(min(r.x, r.y), r.z);
  }
  float k1 = length(p / (r * r));
  return k0 * (k0 - 1.0f) / k1;
}


/* SDFPrimitiveData removed — eval functions take SDFObjectGPU directly. */

/* ---- CSG dispatch ---- */

/** Shell mode IDs (must match eSDFShellMode in DNA_sdf_types.h). */
#define SDF_SHELL_MODE_NORMAL 0
#define SDF_SHELL_MODE_PUSH 1
#define SDF_SHELL_MODE_AVOID 2

/** Blend type IDs (must match eSDFBlendType in DNA_sdf_types.h). */
#define SDF_BLEND_TYPE_LINEAR 0
#define SDF_BLEND_TYPE_SMOOTH 1
#define SDF_BLEND_TYPE_CHAMFER 2
#define SDF_BLEND_TYPE_ROUND 3

/**
 * Combine two SDF distances using the specified CSG operation and blend type.
 */
float combineCSG(float d1, float d2, int op, int bt, float k,
                 float shell_dist, int shell_mode, int shell_op,
                 float shell_k_top, float shell_k_bot,
                 float k2, float k3, float k4, float k5,
                 int flip_blend, int flip_blend_end)
{
  bool has_smooth = (k2 > 0.0f || k3 > 0.0f);
  bool has_smooth_end = (k4 > 0.0f || k5 > 0.0f);
  bool fb = (flip_blend != 0);
  bool fbe = (flip_blend_end != 0);

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
    float sd = (shell_op == SDF_SHELL_OP_SUBTRACTION) ? -shell_dist : shell_dist;
    float h = abs(sd);

    float d_shell;
    if (sd < 0.0f) {
      /* Inward: start = subtraction (bottom edge), end = union (top edge) */
      float d_sub;
      if (shell_k_top > 0.0f && bt > 0) {
        if (fb) {
          if (bt == SDF_BLEND_TYPE_SMOOTH) { d_sub = opSmoothSubtraction(d2, d1, shell_k_top); }
          else if (bt == SDF_BLEND_TYPE_CHAMFER) {
            if (has_smooth) { d_sub = opSmoothChamferSubtraction(d2, d1, shell_k_top, k2, k3); }
            else { d_sub = opChamferSubtraction(d2, d1, shell_k_top); }
          }
          else {
            if (has_smooth) { d_sub = opSmoothRoundSubtraction(d2, d1, shell_k_top, k2, k3); }
            else { d_sub = opRoundSubtraction(d2, d1, shell_k_top); }
          }
        }
        else if (bt == SDF_BLEND_TYPE_SMOOTH) {
          d_sub = opSmoothSubtraction(d2, d1, shell_k_top);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          if (has_smooth) { d_sub = opSmoothChamferSubtraction(d2, d1, shell_k_top, k2, k3); }
          else { d_sub = opChamferSubtraction(d2, d1, shell_k_top); }
        }
        else if (bt == SDF_BLEND_TYPE_ROUND) {
          d_sub = opIntersectionRound(d1, -d2, shell_k_top);
        }
        else { d_sub = max(d1, -d2); }
      }
      else { d_sub = max(d1, -d2); }

      float lim = d1 + h;
      if (shell_k_bot > 0.0f && bt > 0) {
        if (fbe) {
          if (bt == SDF_BLEND_TYPE_SMOOTH) {
            float esub = opSmoothSubtraction(lim, d_sub, shell_k_bot);
            d_shell = min(esub, lim);
          }
          else if (bt == SDF_BLEND_TYPE_CHAMFER) {
            float esub;
            if (has_smooth_end) { esub = opSmoothChamferSubtraction(lim, d_sub, shell_k_bot, k4, k5); }
            else { esub = opChamferSubtraction(lim, d_sub, shell_k_bot); }
            d_shell = min(esub, lim);
          }
          else {
            if (has_smooth_end) { d_shell = opSmoothRoundUnionInverted(d_sub, lim, shell_k_bot, k4, k5); }
            else { d_shell = opRoundUnionInverted(d_sub, lim, shell_k_bot); }
          }
        }
        else if (bt == SDF_BLEND_TYPE_SMOOTH) {
          d_shell = opSmoothUnion(d_sub, lim, shell_k_bot);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          if (has_smooth_end) { d_shell = opSmoothChamferUnion(d_sub, lim, shell_k_bot, k4, k5); }
          else { d_shell = opChamferUnion(d_sub, lim, shell_k_bot); }
        }
        else if (bt == SDF_BLEND_TYPE_ROUND) {
          if (has_smooth_end) { d_shell = opSmoothRoundUnion(d_sub, lim, shell_k_bot, k4, k5); }
          else { d_shell = opRoundUnion(d_sub, lim, shell_k_bot); }
        }
        else { d_shell = min(d_sub, lim); }
      }
      else { d_shell = min(d_sub, lim); }
    }
    else {
      /* Outward: start = union (top edge), end = intersection (bottom edge) */
      float d_union;
      if (shell_k_top > 0.0f && bt > 0) {
        if (fb) {
          /* Flip start = push-like: subtract then min */
          float sub;
          if (bt == SDF_BLEND_TYPE_SMOOTH) { sub = opSmoothSubtraction(d2, d1, shell_k_top); }
          else if (bt == SDF_BLEND_TYPE_CHAMFER) {
            if (has_smooth) { sub = opSmoothChamferSubtraction(d2, d1, shell_k_top, k2, k3); }
            else { sub = opChamferSubtraction(d2, d1, shell_k_top); }
          }
          else {
            if (has_smooth) { sub = opSmoothRoundSubtraction(d2, d1, shell_k_top, k2, k3); }
            else { sub = opRoundSubtraction(d2, d1, shell_k_top); }
          }
          d_union = min(sub, d2);
        }
        else if (bt == SDF_BLEND_TYPE_SMOOTH) {
          d_union = opSmoothUnion(d1, d2, shell_k_top);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          if (has_smooth) { d_union = opSmoothChamferUnion(d1, d2, shell_k_top, k2, k3); }
          else { d_union = opChamferUnion(d1, d2, shell_k_top); }
        }
        else if (bt == SDF_BLEND_TYPE_ROUND) {
          if (has_smooth) { d_union = opSmoothRoundUnion(d1, d2, shell_k_top, k2, k3); }
          else { d_union = opRoundUnion(d1, d2, shell_k_top); }
        }
        else { d_union = min(d1, d2); }
      }
      else { d_union = min(d1, d2); }

      float lim = d1 - h;
      if (shell_k_bot > 0.0f && bt > 0) {
        if (fbe) {
          if (bt == SDF_BLEND_TYPE_SMOOTH) {
            float esub = opSmoothSubtraction(lim, d_union, shell_k_bot);
            d_shell = max(esub, lim);
          }
          else if (bt == SDF_BLEND_TYPE_CHAMFER) {
            float esub;
            if (has_smooth_end) { esub = opSmoothChamferSubtraction(lim, d_union, shell_k_bot, k4, k5); }
            else { esub = opChamferSubtraction(lim, d_union, shell_k_bot); }
            d_shell = max(esub, lim);
          }
          else {
            if (has_smooth_end) { d_shell = opSmoothRoundIntersectionInverted(d_union, lim, shell_k_bot, k4, k5); }
            else { d_shell = opRoundIntersection(d_union, lim, shell_k_bot); }
          }
        }
        else if (bt == SDF_BLEND_TYPE_SMOOTH) {
          d_shell = opSmoothIntersection(d_union, lim, shell_k_bot);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          if (has_smooth_end) { d_shell = opSmoothChamferIntersection(d_union, lim, shell_k_bot, k4, k5); }
          else { d_shell = opChamferIntersection(d_union, lim, shell_k_bot); }
        }
        else if (bt == SDF_BLEND_TYPE_ROUND) {
          d_shell = opSmoothIntersection(d_union, lim, shell_k_bot);
        }
        else { d_shell = max(d_union, lim); }
      }
      else { d_shell = max(d_union, lim); }
    }

    if (shell_mode == SDF_SHELL_MODE_AVOID) {
      float carved;
      if (k > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) {
          carved = opSmoothSubtraction(d1, d_shell, k);
        }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) {
          if (has_smooth) { carved = opSmoothChamferSubtraction(d1, d_shell, k, k2, k3); }
          else { carved = opChamferSubtraction(d1, d_shell, k); }
        }
        else {
          if (has_smooth) { carved = opSmoothRoundSubtraction(d1, d_shell, k, k2, k3); }
          else { carved = opRoundSubtraction(d1, d_shell, k); }
        }
      }
      else {
        carved = max(d_shell, -d1);
      }
      return min(d1, carved);
    }

    return d_shell;
  }
  return d1;
}

float evalPrimitiveOnly(SDFObjectGPU obj, float3 local_pos)
{
  /* sdf_size.xyz = pre-subtracted BASE (size - bevel), pre-clamped on CPU.
   * sdf_size.w = effective bevel. obj_scale.xyz applies object scale as a
   * coordinate transform so the geometry stretches/squishes. */
  float3 r = obj.sdf_size.xyz;
  float bevel = obj.sdf_size.w;
  local_pos /= obj.obj_scale.xyz;
  float dist;

#ifdef SDF_BENCH_BOX_ONLY
  return sdBox(local_pos, r) - bevel;
#endif

  if (obj.sdf_type == 1) { /* SPHERE / ELLIPSOID */
    dist = (abs(r.x - r.y) < 0.0001f && abs(r.x - r.z) < 0.0001f)
           ? sdSphere(local_pos, r.x) : sdEllipsoid(local_pos, r);
  }
  else if (obj.sdf_type == 2) { /* CYLINDER */
    dist = sdCylinder(local_pos, r);
  }
  else if (obj.sdf_type == 3) { /* CONE / FRUSTUM */
    dist = sdConeFrustum(local_pos, r.x, r.z, r.y);
  }
  else if (obj.sdf_type == 4) { /* CAPSULE */
    dist = sdCapsule(local_pos, r);
  }
  else if (obj.sdf_type == 5) { /* TORUS */
    dist = (obj.box_modes.w != 0)
           ? sdCappedTorus(local_pos, obj.box_corners.xy, r.x, r.y)
           : sdTorus(local_pos, float2(r.x, r.y));
  }
  else if (obj.sdf_type == 6) { /* NGON */
    int sides = obj.box_modes.z;
    float corner = obj.box_corners.x;
    float star = obj.box_corners.y;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    int edgeMode = obj.box_modes.y;
    if ((corner + edgeTop + edgeBot + tapTop + tapBot + star) > 0.001f) {
      dist = sdAdvancedNgon(local_pos, r.x, r.z, sides, corner, edgeTop, edgeBot, tapTop, tapBot, edgeMode, r.z, star);
    }
    else {
      float d2d = sdRegularPolygon2D(local_pos.xy, r.x, sides);
      float dz = abs(local_pos.z) - r.z;
      float2 dd = float2(d2d, dz);
      dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
    }
  }
  else if (obj.sdf_type == 7) { /* POLYGON */
    int ps = obj.polygon_point_start;
    int pc = obj.polygon_point_count;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    int edgeMode = obj.box_modes.y;
    if (pc >= 3) {
      if ((edgeTop + edgeBot + tapTop + tapBot) > 0.001f || obj.box_corners.x > 0.001f) {
        dist = sdAdvancedPolygon(local_pos, r.z, ps, pc, edgeTop, edgeBot, tapTop, tapBot, edgeMode, r.z);
      }
      else {
        float d2d = sdPolygon2D(local_pos.xy, ps, pc);
        float dz = abs(local_pos.z) - r.z;
        float2 dd = float2(d2d, dz);
        dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
      }
    }
    else { dist = 1e10f; }
  }
  else { /* BOX (default) */
    float4 corners = obj.box_corners;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    if ((corners.x + corners.y + corners.z + corners.w + edgeTop + edgeBot + tapTop + tapBot) > 0.001f) {
      dist = sdAdvancedBox(local_pos, r, corners, edgeTop, edgeBot, tapTop, tapBot, obj.box_modes.x, obj.box_modes.y, r.z);
    }
    else {
      dist = sdBox(local_pos, r);
    }
  }

  return dist - bevel;
}

/**
 * Apply distance modifiers to SDF distance (after primitive evaluation).
 * Applied in forward stack order.
 */
float applyDistanceModifiers(float dist, float3 p, SDFObjectGPU obj, int mod_start, int mod_count)
{
  for (int i = mod_start; i < mod_start + mod_count; i++) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;

    if (mtype == SDF_MOD_SOLIDIFY) {
      float thickness = smod.params.x;
      float bevel = smod.params.z;
      int mode = smod.header.y;

      if (mode == 0) {
        /* Closed */
        float d_inner = -(dist + thickness);
        if (bevel > 0.0) {
          dist = opSmoothIntersection(dist, d_inner, bevel);
        }
        else {
          dist = max(dist, d_inner);
        }
      }
      else {
        /* Open: axis-scaled inner to punch through caps */
        int axis = smod.header.z;
        float inner_scale = smod.params.y;
        float3 p_inner = p;
        p_inner[axis] *= inner_scale;
        float d_inner = -(evalPrimitiveOnly(obj, p_inner) + thickness);
        if (bevel > 0.0) {
          dist = opSmoothIntersection(dist, d_inner, bevel);
        }
        else {
          dist = max(dist, d_inner);
        }
      }
    }
    else if (mtype == SDF_MOD_ROUND) {
      dist -= smod.params.x;
    }
    else if (mtype == SDF_MOD_DISPLACE) {
      float strength = smod.params.x;
      float frequency = smod.params.y;
      float lacunarity = smod.params.z;
      float roughness = smod.params.w;
      int noise_type = smod.header.y;
      int octaves = smod.header.z;
      float n = sdf_displacement_fast(p * frequency, noise_type, octaves, lacunarity, roughness);
      /* Per-type Lipschitz bound */
      float lip = 1.0f;
      if (noise_type == SDF_MOD_DISPLACE_POINTS) { lip = 3.5f; }
      else if (noise_type == SDF_MOD_DISPLACE_TRIANGLE) { lip = 3.0f; }
      else if (noise_type == SDF_MOD_DISPLACE_VORONOI) { lip = 1.5f; }
      dist += n * strength;
      dist /= (1.0f + abs(strength) * frequency * lip);
    }
    else if (mtype == SDF_MOD_ONION) {
      int layers = max(smod.header.y, 1);
      float cut_half = max(smod.params.x, 0.001) * 0.5;
      float min_ext = smod.params.y;
      float original_d = dist;
      if (layers > 1) {
        float spacing = min_ext / float(layers);
        float depth = max(-dist, 0.0);
        float max_cut = float(layers - 1) * spacing;
        float nearest = clamp(floor(depth / spacing + 0.5) * spacing, spacing, max_cut);
        float cut_dist = abs(depth - nearest);
        float onion_d = cut_half - cut_dist;
        dist = max(original_d, onion_d);
      }
    }
  }
  return dist;
}

/* Group-level distance modifiers (no per-object evalPrimitiveOnly). */
float applyGroupDistanceModifiers(float dist, float3 p, int mod_start, int mod_count)
{
  for (int i = mod_start; i < mod_start + mod_count; i++) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;

    if (mtype == SDF_MOD_SOLIDIFY) {
      float thickness = smod.params.x;
      float d_inner = -(dist + thickness);
      dist = max(dist, d_inner);
    }
    else if (mtype == SDF_MOD_ROUND) {
      dist -= smod.params.x;
    }
    else if (mtype == SDF_MOD_ONION) {
      int layers = max(smod.header.y, 1);
      float cut_half = max(smod.params.x, 0.001) * 0.5;
      float min_ext = smod.params.y;
      if (layers > 1) {
        float spacing = min_ext / float(layers);
        float depth = max(-dist, 0.0);
        float max_cut = float(layers - 1) * spacing;
        float nearest = clamp(floor(depth / spacing + 0.5) * spacing, spacing, max_cut);
        float cut_dist = abs(depth - nearest);
        float onion_d = cut_half - cut_dist;
        dist = max(dist, onion_d);
      }
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
  /* All modifiers (mirror, array, twist, etc.) use domain folding — single eval path. */
  float4 dm = applyDomainModifiers(p, obj.modifier_start, obj.modifier_count, obj.inverse_matrix);
  p = dm.xyz;
  /* evalPrimitiveOnly returns a BASE-space distance; convert to world via min(scale). */
  float d = evalPrimitiveOnly(obj, p) * obj.obj_scale.w;
  d = applyDistanceModifiers(d, p, obj, obj.modifier_start, obj.modifier_count);
  return d * dm.w;
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
        grp.chamfer_k2, grp.chamfer_k3, grp.chamfer_k4, grp.chamfer_k5,
        grp.flip_blend, grp.flip_blend_end);
    float t = csgColorFactor(prev, grp_dist, grp.csg_operation, grp.blend_type, grp.blend,
                             grp.shell_distance, grp.shell_op);
    out_color = mix(out_color, grp_color, t);
  }
}

void flushGroupDist(int gid, float grp_dist, inout float scene_dist)
{
  if (scene_dist >= 1e9f) {
    scene_dist = grp_dist;
  }
  else {
    SDFGroupGPU grp = groups[gid];
    scene_dist = combineCSG(
        scene_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend,
        grp.shell_distance, grp.shell_mode, grp.shell_op,
        grp.shell_blend_top, grp.shell_blend_bottom,
        grp.chamfer_k2, grp.chamfer_k3, grp.chamfer_k4, grp.chamfer_k5,
        grp.flip_blend, grp.flip_blend_end);
  }
}

/** \} */

/** \} */

