/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning: shared evaluation library.
 *
 * Port of the reference "Lipschitz Pruning" engine (Barbier et al., EG 2025)
 * to the Blender SDF draw engine. A CSG tree is serialized in post-order into
 * flat arrays (lp_nodes / lp_prims / lp_binary_ops); evaluation is a stack
 * machine over those arrays. Per-cell "active node" lists record the pruned
 * sub-tree that is equivalent to the full tree inside that cell.
 *
 * Op coverage (feature parity with the classic engine):
 * - UNION/SUBTRACT/INTERSECT x LINEAR/SMOOTH/CHAMFER/ROUND: all share the
 *   signed form s * op(s*a, s*b) which reduces to the exact winner outside
 *   |a-b| >= k, so the reference 2R+k dominance bound culls them all.
 *   SMOOTH is the quadratic kernel (identical to the classic opSmooth*),
 *   CHAMFER the linear kernel (identical to opChamfer*), ROUND the spherical
 *   fillet (same zero set as opRound*, kept continuous — see
 *   lp_binary_op_eval). Note ROUND is only 1-Lipschitz in the (a,b)-metric;
 *   w.r.t. position it reaches sqrt(2) where the child gradients align
 *   (fillet crest), which the prune pass accounts for via the per-node
 *   Lipschitz constants (SDFLpNode.lipschitz).
 * - PUSH/AVOID/SHELL are desugared by the tree builder into UNION/SUBTRACT/
 *   INTERSECT + OFFSET nodes, so they cull through the same machinery.
 * - PAINT is the only opaque op (geometry = left operand, never culled).
 *
 * Active node packing (uint): bit 31 = negated subtree, bits 30..0 = node index.
 * Binary op packing: see SDF_LP_OP_KMASK comment in sdf_shader_shared.hh.
 *
 * Expects the including shader to declare (via its CREATE_INFO):
 *   lp_prims[], lp_nodes[], lp_binary_ops[], lp_active_in[] (uint2: x = node
 *   word, y = parent), polygon_points[], mesh_data_buf[]
 */

/* Eval stack depth. The scene fold produces left-leaning trees whose
 * post-order evaluation is O(log n) deep, but SHELL/PUSH/AVOID desugaring
 * duplicates subtrees (each becomes two binary ops + OFFSET nodes), so the
 * worst-case depth grows accordingly. 64 covers trees of up to ~2^64 leaves
 * under the left-leaning fold; the stack arrays are statically indexed so the
 * cost is register pressure only. Compile-time constant — change here only. */
#define SDF_LP_STACK_DEPTH 64

#define SDF_LP_PI 3.14159265359f

/* ------------------------------------------------------------------ */
/** \name Binary op decoding / evaluation
 * \{ */

uint lp_op_csg(uint op)
{
  return (op >> 1) & 7u;
}

uint lp_op_blend_type(uint op)
{
  return (op >> 4) & 3u;
}

float lp_op_blend_factor(uint op)
{
  return uintBitsToFloat(op & SDF_LP_OP_KMASK);
}

/* Sign of the min() form: +1 for union, -1 for subtract/intersect. */
float lp_op_sign(uint op)
{
  return -1.0f + 2.0f * float(op & 1u);
}

/* All ops except PAINT have exact winner fallback outside |a-b| >= k. */
bool lp_op_cullable(uint op)
{
  return lp_op_csg(op) != SDF_LP_CSG_PAINT;
}

/* Quadratic smooth kernel (identical to the classic opSmooth* family). */
float lp_kernel(float x, float k)
{
  if (k == 0.0f) {
    return 0.0f;
  }
  float m = max(0.0f, k - x);
  return m * m * 0.25f / k;
}

/* Linear chamfer kernel (identical to the classic opChamfer* family):
 * min(a,b) - (k - |a-b|) / 2 == min(min(a,b), (a + b - k) / 2). */
float lp_kernel_chamfer(float x, float k)
{
  return max(0.0f, k - x) * 0.5f;
}

float2 lp_mirror2D(float2 p, float2 N)
{
  float proj = min(dot(p, N), 0.0f);
  return p - 2.0f * N * proj;
}

/* Spherical fillet (identical to the classic opUnionIRound). */
float lp_op_iround(float a, float b, float r)
{
  float2 q = float2(a, b);
  q = lp_mirror2D(q, float2(-0.70710678f, 0.70710678f));
  q.y -= r;
  q.y = min(0.0f, q.y);
  float ad = sign(q.x) * length(q);
  float2 s = float2(max(a, 0.0f), max(b, 0.0f));
  float corn = length(s) - r;
  return min(ad, corn);
}

/* ---- Smooth (k2/k3 edge-softness) blend variants ----
 * Ported verbatim from sdf_lib.glsl (:661-695, :812-836, :890-951) so the
 * LP engine matches the classic engine when chamfer_k2/chamfer_k3 (start
 * edge) or chamfer_k4/chamfer_k5 (shell end edge) are set. Only CHAMFER and
 * ROUND have smooth variants; SMOOTH ignores k2/k3 in the classic engine. */

float lp_op_smooth_union(float d1, float d2, float k)
{
  if (k <= 0.0001f) {
    return min(d1, d2);
  }
  if (abs(d2 - d1) >= k) {
    return min(d1, d2);
  }
  float h = clamp(0.5f + 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
  return mix(d2, d1, h) - k * h * (1.0f - h);
}

float lp_op_smooth_subtraction(float d1, float d2, float k)
{
  if (k <= 0.0001f) {
    return max(-d1, d2);
  }
  if (abs(d2 + d1) >= k) {
    return max(-d1, d2);
  }
  float h = clamp(0.5f - 0.5f * (d2 + d1) / k, 0.0f, 1.0f);
  return mix(d2, -d1, h) + k * h * (1.0f - h);
}

float lp_op_smooth_intersection(float d1, float d2, float k)
{
  if (k <= 0.0001f) {
    return max(d1, d2);
  }
  if (abs(d2 - d1) >= k) {
    return max(d1, d2);
  }
  float h = clamp(0.5f - 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
  return mix(d2, d1, h) + k * h * (1.0f - h);
}

float lp_op_smooth_chamfer_union(float d1, float d2, float k, float k2, float k3)
{
  float chamfer_plane = (d1 + d2 - k) * 0.5f;
  float term1 = lp_op_smooth_union(d1, chamfer_plane, k2);
  float term2 = lp_op_smooth_union(d2, chamfer_plane, k3);
  return min(term1, term2);
}

float lp_op_smooth_chamfer_subtraction(float d1, float d2, float k, float k2, float k3)
{
  float A = -d1;
  float B = d2;
  float chamfer_plane = (A + B + k) * 0.5f;
  float term1 = lp_op_smooth_intersection(A, chamfer_plane, k2);
  float term2 = lp_op_smooth_intersection(B, chamfer_plane, k3);
  return max(term1, term2);
}

float lp_op_smooth_chamfer_intersection(float d1, float d2, float k, float k2, float k3)
{
  float chamfer_plane = (d1 + d2 + k) * 0.5f;
  float term1 = lp_op_smooth_intersection(d1, chamfer_plane, k2);
  float term2 = lp_op_smooth_intersection(d2, chamfer_plane, k3);
  return max(term1, term2);
}

float lp_op_smooth_round_union(float a, float b, float r, float k2, float k3)
{
  float2 s = float2(max(a, 0.0f), max(b, 0.0f));
  float corner = length(s) - r;
  float term1 = lp_op_smooth_union(a, corner, k2);
  float term2 = lp_op_smooth_union(b, corner, k3);
  return min(term1, term2);
}

float lp_op_smooth_round_subtraction(float d1, float d2, float r, float k2, float k3)
{
  float a = d2;
  float b = d1;
  float2 s = float2(min(a, 0.0f), max(b, 0.0f));
  float corner = r - length(s);
  float term1 = lp_op_smooth_intersection(a, corner, k2);
  float term2 = lp_op_smooth_intersection(-b, corner, k3);
  return max(term1, term2);
}

float lp_op_smooth_round_intersection(float d1, float d2, float r, float k2, float k3)
{
  float2 s = float2(min(d1, 0.0f), max(-d2, 0.0f));
  float corner = r - length(s);
  float term1 = lp_op_smooth_intersection(d1, corner, k2);
  float term2 = lp_op_smooth_intersection(d2, corner, k3);
  return max(term1, term2);
}

/* Quilez round intersection (classic opIntersectionRound, sdf_lib.glsl:919).
 * Used by the inward SHELL start edge (non-flipped ROUND), where the classic
 * engine does NOT use the opRoundSubtraction duality form. */
float lp_op_intersection_round(float a, float b, float r)
{
  float2 u = max(float2(r + a, r + b), float2(0.0f));
  return min(-r, max(a, b)) + length(u);
}

/* Classic opSmoothRoundIntersectionInverted (sdf_lib.glsl:944): outward SHELL
 * end edge with flip_blend_end. */
float lp_op_smooth_round_intersection_inverted(float d1, float d2, float r, float k2, float k3)
{
  float a = d1;
  float b = -d2;
  float2 s = float2(min(a, 0.0f), max(b, 0.0f));
  float corner = r - length(s);
  float term1 = lp_op_smooth_intersection(a, corner, k2);
  float term2 = lp_op_smooth_intersection(-b, corner, k3);
  return max(term1, term2);
}

/* Common signed form for union/subtract/intersect with any blend type.
 * a and b are the operand values including their active-node signs.
 * Verified exact against the classic kernels (sdf_lib.glsl):
 * - SMOOTH: min - max(0,k-|a-b|)^2/(4k) == opSmoothUnion family (:661-695).
 * - CHAMFER: min - max(0,k-|a-b|)/2 == min(min(a,b),(a-k+b)/2) inside the
 *   blend zone and min outside, i.e. exactly opChamferUnion (:789) (the
 *   kernel-subtracted form is algebraic, not an approximation).
 * - ROUND: s*opUnionIRound(s*a,s*b,k) evaluated WITHOUT the classic
 *   |a-b| >= k cutoff. opUnionIRound reduces to the exact winner outside the
 *   blend zone on its own (the q.y clamp makes `ad` coincide with min there),
 *   so the signed form stays continuous; the classic cutoff is discontinuous
 *   in the negative/mixed quadrants, which breaks the per-cell far-field
 *   bound (blocky band artifacts). The zero set matches
 *   opRoundUnion/opRoundSubtraction/opRoundIntersection (:849-887) exactly.
 *   The field is 1-Lipschitz in (a,b) but up to sqrt(2)-Lipschitz w.r.t.
 *   position; see SDFLpNode.lipschitz.
 * - Smooth (k2/k3) CHAMFER/ROUND variants dispatch to the verbatim classic
 *   ports above (combineCSG :1490-1537). SUBTRACT's right operand arrives
 *   negated (b = -d2), so the classic (d2, d1) argument order is (-b, a). */
float lp_binary_op_eval(uint4 opw, float a, float b)
{
  uint op = opw.x;
  if (lp_op_csg(op) == SDF_LP_CSG_PAINT) {
    return a;
  }
  float s = lp_op_sign(op);
  float k = lp_op_blend_factor(op);
  uint bt = lp_op_blend_type(op);
  uint csg = lp_op_csg(op);
  float k2 = uintBitsToFloat(opw.y);
  float k3 = uintBitsToFloat(opw.z);
  bool has_smooth = (k2 > 0.0f || k3 > 0.0f);
  if ((bt == SDF_LP_BLEND_CHAMFER || bt == SDF_LP_BLEND_ROUND) && k > 0.0f) {
    if (has_smooth) {
      if (bt == SDF_LP_BLEND_CHAMFER) {
        if (csg == SDF_LP_CSG_UNION) {
          return lp_op_smooth_chamfer_union(a, b, k, k2, k3);
        }
        if (csg == SDF_LP_CSG_INTERSECT) {
          return lp_op_smooth_chamfer_intersection(a, b, k, k2, k3);
        }
        return lp_op_smooth_chamfer_subtraction(-b, a, k, k2, k3);
      }
      if ((opw.w & SDF_LP_OP_FLAG_INVERTED) != 0u) {
        /* Outward SHELL end edge, flip_blend_end (sdf_lib.glsl:1701). */
        return lp_op_smooth_round_intersection_inverted(a, b, k, k2, k3);
      }
      if (csg == SDF_LP_CSG_UNION) {
        return lp_op_smooth_round_union(a, b, k, k2, k3);
      }
      if (csg == SDF_LP_CSG_INTERSECT) {
        return lp_op_smooth_round_intersection(a, b, k, k2, k3);
      }
      return lp_op_smooth_round_subtraction(-b, a, k, k2, k3);
    }
    if (bt == SDF_LP_BLEND_ROUND && (opw.w & SDF_LP_OP_FLAG_INVERTED) != 0u &&
        csg == SDF_LP_CSG_SUBTRACT)
    {
      /* Inward SHELL start edge, non-flipped ROUND (sdf_lib.glsl:1616). */
      return lp_op_intersection_round(a, b, k);
    }
  }
  if (bt == SDF_LP_BLEND_ROUND) {
    if (k <= 0.0f) {
      return s * min(s * a, s * b);
    }
    /* No winner fallback at |a-b| >= k: lp_op_iround already reduces to the
     * exact winner outside the blend zone (q.y clamps to 0, so `ad` becomes
     * min(s*a, s*b) and `corn` stays above it wherever the zone fallback
     * would be exact), so the fillet evaluates as one continuous field. The
     * classic opRound* cutoff to min() is discontinuous in the negative/mixed
     * quadrants (jump up to ~1.24k), which breaks the per-cell far-field
     * bound this engine stores and shows up as blocky bands along the blend
     * regions. The field is 1-Lipschitz in the Euclidean (a,b)-metric, i.e.
     * up to sqrt(2)-Lipschitz w.r.t. position when both children vary
     * together — the prune pass scales its bounds with the per-node Lipschitz
     * constant (SDFLpNode.lipschitz) to stay conservative. */
    return s * lp_op_iround(s * a, s * b, k);
  }
  float ker = (bt == SDF_LP_BLEND_CHAMFER) ? lp_kernel_chamfer(abs(a - b), k) :
                                             lp_kernel(abs(a - b), k);
  return s * (min(s * a, s * b) - ker);
}

/* Effective dominance margin for the prune pass: outside |a-b| > lp_op_dom_k
 * the op reduces to the exact winner (verified numerically). Smooth variants
 * keep blending through their k2/k3 smin terms until |a-b| exceeds
 * k + 2*max(k2,k3) (the chamfer plane / round corner must clear the winning
 * operand by the full softness radius before the smin is exact). */
float lp_op_dom_k(uint4 opw)
{
  float k = lp_op_blend_factor(opw.x);
  uint bt = lp_op_blend_type(opw.x);
  if (bt == SDF_LP_BLEND_CHAMFER || bt == SDF_LP_BLEND_ROUND) {
    float k2 = uintBitsToFloat(opw.y);
    float k3 = uintBitsToFloat(opw.z);
    if (k2 > 0.0f || k3 > 0.0f) {
      return k + 2.0f * max(k2, k3);
    }
  }
  return k;
}

/* Smooth-min blend returning (distance, color mix factor). */
float2 lp_smin_blend(float a, float b, float k)
{
  if (k <= 0.0f) {
    /* Hard min: the dominant operand takes the color. */
    return (a < b) ? float2(a, 0.0f) : float2(b, 1.0f);
  }
  float h = max(k - abs(a - b), 0.0f) / k;
  float m = h * h * 0.5f;
  float s = m * k * 0.5f;
  return (a < b) ? float2(a - s, m) : float2(b - s, 1.0f - m);
}

/** \} */

/* ------------------------------------------------------------------ */
/** \name Primitive evaluation
 *
 * Mirrors the classic engine (sdf_lib.glsl evalPrimitiveOnly) basic shapes:
 * box, sphere/ellipsoid, cylinder, cone, capsule, torus, ngon prism, polygon
 * prism. Scale is applied as a coordinate transform and the result multiplied
 * by the min axis scale, which keeps the field 1-Lipschitz (conservatively)
 * as required by the pruning bound.
 * \{ */

float lp_sd_box(float3 p, float3 b)
{
  float3 q = abs(p) - b;
  return length(max(q, float3(0.0f))) + min(max(q.x, max(q.y, q.z)), 0.0f);
}

float lp_sd_sphere(float3 p, float r)
{
  return length(p) - r;
}

float lp_sd_ellipsoid(float3 p, float3 r)
{
  float k0 = length(p / r);
  if (k0 < 0.0001f) {
    return -min(min(r.x, r.y), r.z);
  }
  float k1 = length(p / (r * r));
  return k0 * (k0 - 1.0f) / k1;
}

float lp_sd_cylinder(float3 p, float3 size)
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

float lp_sd_cone_frustum(float3 p, float rb, float rt, float h)
{
  float2 q = float2(length(p.xy), p.z);
  float2 k1 = float2(rt, h);
  float2 k2 = float2(rt - rb, 2.0f * h);
  float2 ca = float2(q.x - min(q.x, (q.y < 0.0f) ? rb : rt), abs(q.y) - h);
  float2 cb = q - k1 + k2 * clamp(dot(k1 - q, k2) / dot(k2, k2), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * sqrt(min(dot(ca, ca), dot(cb, cb)));
}

float lp_sd_capsule(float3 p, float3 size)
{
  float h = size.y;
  float r = size.x;
  p.z -= clamp(p.z, -h, h);
  return length(p) - r;
}

float lp_sd_torus(float3 p, float2 t)
{
  float2 q = float2(length(p.xy) - t.x, p.z);
  return length(q) - t.y;
}

/* Capped torus (identical to sdCappedTorus, sdf_lib.glsl:250). */
float lp_sd_capped_torus(float3 p, float2 sc, float ra, float rb)
{
  p.x = abs(p.x);
  float k = (sc.y * p.x > sc.x * p.y) ? dot(p.xy, sc) : length(p.xy);
  return sqrt(dot(p, p) + ra * ra - 2.0f * ra * k) - rb;
}

/* 2D rounded box with per-corner radii (identical to sdRoundBox2D, :92). */
float lp_sd_round_box_2d(float2 p, float2 b, float4 r)
{
  float rx = (p.x > 0.0f) ? r.x : r.z;
  float ry = (p.x > 0.0f) ? r.y : r.w;
  float rc = (p.y > 0.0f) ? rx : ry;

  float2 q = abs(p) - b + rc;
  return min(max(q.x, q.y), 0.0f) + length(max(q, float2(0.0f))) - rc;
}

/* 2D chamfered box with per-corner radii (identical to sdChamferBox2D, :105). */
float lp_sd_chamfer_box_2d(float2 p, float2 b, float4 r)
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

/* Advanced box with per-corner bevels, top/bottom edge chamfer, and Z taper
 * (identical to sdAdvancedBox, sdf_lib.glsl:149). The taper divides by the
 * classic sqrt(1 + slope^2) Lipschitz correction, keeping the result
 * conservative (never under-estimates the true distance). */
float lp_sd_advanced_box(float3 p,
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

  float slope = length(size.xy) * (tapTop + tapBot) / (2.0f * max(taperZ, 0.001f));
  float lipschitz = sqrt(1.0f + slope * slope);

  float maxR = min(sz.x, sz.y);
  float4 r = corners * maxR;

  float d2d;
  if (cornerMode == 0) {
    d2d = lp_sd_round_box_2d(p.xy, sz, r);
  }
  else {
    d2d = lp_sd_chamfer_box_2d(p.xy, sz, r);
  }

  float maxR_face = min(sz.x, sz.y);

  float dz = abs(p.z) - size.z;
  float edgeR = (p.z > 0.0f) ? edgeTop * min(maxR_face, size.z) :
                               edgeBot * min(maxR_face, size.z);

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

/* Star polygon 2D SDF (identical to sdStarPolygon2D, sdf_lib.glsl:265). */
float lp_sd_regular_polygon_2d(float2 p, float R, int n);
float lp_sd_star_polygon_2d(float2 p, float R, int n, float star)
{
  if (star < 0.001f) {
    return lp_sd_regular_polygon_2d(p, R, n);
  }

  float an = SDF_LP_PI / float(n);
  float r = R * cos(an) * max(1.0f - star, 0.01f);

  float angle = atan(p.y, p.x);
  float bn = floor((angle + an) / (2.0f * an)) * 2.0f * an;
  float2 cs = float2(cos(bn), sin(bn));
  float2 q = float2(cs.x * p.x + cs.y * p.y, -cs.y * p.x + cs.x * p.y);
  q.y = abs(q.y);

  float2 A = float2(R, 0.0f);
  float2 B = float2(r * cos(an), r * sin(an));
  float2 AB = B - A;
  float t = clamp(dot(q - A, AB) / dot(AB, AB), 0.0f, 1.0f);
  float dist = length(q - (A + AB * t));

  float cross_val = AB.x * (q.y - A.y) - AB.y * (q.x - A.x);
  return (cross_val > 0.0f) ? -dist : dist;
}

/* Advanced N-gon prism with corner bevel, edge chamfer, star mode, and taper
 * (identical to sdAdvancedNgon, sdf_lib.glsl:310, including the taper
 * sqrt(1 + slope^2) Lipschitz correction). */
float lp_sd_advanced_ngon(float3 p,
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

  float slope = R * (tapTop + tapBot) / (2.0f * max(taperH, 0.001f));
  float lipschitz = sqrt(1.0f + slope * slope);

  float an = SDF_LP_PI / float(sides);
  float apothem = scaledR * cos(an);
  float bevelR = corner * apothem;
  float innerR = scaledR - bevelR / max(cos(an), 0.001f);
  float d2d;
  if (star > 0.001f) {
    d2d = lp_sd_star_polygon_2d(p.xy, innerR, sides, star) - bevelR;
  }
  else {
    d2d = lp_sd_regular_polygon_2d(p.xy, innerR, sides) - bevelR;
  }

  float dz = abs(p.z) - halfH;
  float edgeR = (p.z > 0.0f) ? edgeTop * min(apothem, halfH) :
                               edgeBot * min(apothem, halfH);

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

/* Regular n-gon prism cross-section (identical to sdRegularPolygon2D). */
float lp_sd_regular_polygon_2d(float2 p, float R, int n)
{
  float an = SDF_LP_PI / float(n);
  float r = R * cos(an);
  float he = R * sin(an);
  p = float2(-p.y, p.x);
  float bn = an * floor((atan(p.y, p.x) + an) / an / 2.0f) * 2.0f;
  float2 cs = float2(cos(bn), sin(bn));
  p = float2(cs.x * p.x + cs.y * p.y, -cs.y * p.x + cs.x * p.y);
  return length(p - float2(r, clamp(p.y, -he, he))) * sign(p.x - r);
}

/* Quadratic bezier distance (identical to sdBezier2D). */
float lp_sd_bezier_2d(float2 A, float2 B, float2 C, float2 pos)
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

void lp_toggle_bezier_root(float t,
                           float2 a,
                           float2 b,
                           float2 c,
                           float2 p,
                           inout int winding)
{
  if (t < 0.0f || t > 1.0f) {
    return;
  }
  float u = 1.0f - t;
  float2 q = u * u * a + 2.0f * u * t * b + t * t * c;
  float2 v = 2.0f * (u * (b - a) + t * (c - b));
  float cr = v.x * (p.y - q.y) - v.y * (p.x - q.x);
  if (v.y > 0.0f && p.y != c.y && cr > 0.0f) {
    winding--;
  }
  if (v.y < 0.0f && p.y != a.y && cr < 0.0f) {
    winding++;
  }
}

void lp_toggle_bezier_winding(float2 a,
                              float2 b,
                              float2 c,
                              float2 p,
                              inout int winding)
{
  float y0 = min(min(a.y, b.y), c.y);
  float y1 = max(max(a.y, b.y), c.y);
  if (p.y < y0 || p.y > y1) {
    return;
  }

  float A = a.y - 2.0f * b.y + c.y;
  float B = 2.0f * (b.y - a.y);
  float C = a.y - p.y;

  if (abs(A) < 1e-4f) {
    float t = -C / B;
    lp_toggle_bezier_root(t, a, b, c, p, winding);
  }
  else {
    float root = B * B - 4.0f * A * C;
    if (root > 0.0f) {
      float s = sqrt(root);
      lp_toggle_bezier_root((-B - s) / (2.0f * A), a, b, c, p, winding);
      lp_toggle_bezier_root((-B + s) / (2.0f * A), a, b, c, p, winding);
    }
  }
}

/* Arbitrary polygon 2D SDF with straight + bezier edges (identical to
 * sdPolygon2D). Exact distance, 1-Lipschitz. */
float lp_sd_polygon_2d(float2 p, int ps, int pc)
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
        d = min(d, lp_sd_bezier_2d(vi, ctrl, end_pt, p));
      }

      lp_toggle_bezier_winding(vi, ctrl, end_pt, p, winding);
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
      if (vi.y <= p.y && vj_y > p.y && cross_val > 0.0f) {
        winding++;
      }
      if (vi.y > p.y && vj_y <= p.y && cross_val < 0.0f) {
        winding--;
      }
    }
  }
  return (winding != 0) ? -d : d;
}

/* Corner-rounded variant (identical to sdPolygon2DRounded): straight edges are
 * trimmed and joined by exact arc fillets. Still an exact 1-Lipschitz SDF. */
float lp_sd_polygon_2d_rounded(float2 p, int ps, int pc)
{
  float d = 1e20f;
  int winding = 0;

  for (int i = 0; i < pc; i++) {
    float4 ed = polygon_points[ps + i].vi_edge;
    float4 ad = polygon_points[ps + i].arc_data;
    float4 ab = polygon_points[ps + i].arc_bounds;
    if (ab.w < 0.0f) {
      continue;
    }

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
      if (seg_a.y <= p.y && seg_b.y > p.y && cross_val > 0.0f) {
        winding++;
      }
      if (seg_a.y > p.y && seg_b.y <= p.y && cross_val < 0.0f) {
        winding--;
      }
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

        float th = asin_k;
        float td = th - ang_mid;
        td -= 6.2831853f * floor((td + 3.1415927f) / 6.2831853f);
        if (abs(td) <= ang_half && C.x + R * cos(th) > p.x) {
          winding += dir;
        }

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

/* Advanced polygon prism with rounded outline, edge chamfer, and taper
 * (identical to sdAdvancedPolygon, sdf_lib.glsl:598, including the taper
 * sqrt(1 + slope^2) Lipschitz correction). */
float lp_sd_advanced_polygon(float3 p,
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

  float d2d = lp_sd_polygon_2d_rounded(p.xy / tapFactor, ps, pc) * tapFactor;

  float dz = abs(p.z) - halfH;
  float edgeR = (p.z > 0.0f) ? edgeTop * halfH : edgeBot * halfH;

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

/* ------------------------------------------------------------------ */
/** \name Triangle mesh evaluation (port of sdf_mesh_lib.glsl)
 *
 * Exact signed distance to the mesh triangle set via the same packed BVH as
 * the classic engine. Distance to a triangle soup is 1-Lipschitz, and the
 * pseudonormal sign keeps it an exact SDF, so mesh leaves participate in
 * culling like any analytic primitive. The per-thread hint cache of the
 * classic engine is kept: seeding the walk with the previous hit triangle
 * gives a tight initial distance bound, so the AABB test prunes almost the
 * whole tree (near O(log n)). Without it the walk starts at FLT_MAX and
 * degenerates toward visiting every node (near O(n)).
 * \{ */

#define SDF_LP_MESH_HINT_CACHE_SIZE 8

int g_lp_mesh_hint_keys[SDF_LP_MESH_HINT_CACHE_SIZE];
int g_lp_mesh_hint_triangles[SDF_LP_MESH_HINT_CACHE_SIZE];

float lp_mesh_point_aabb_dist_sq(float3 p, float3 bounds_min, float3 bounds_max)
{
  float3 delta = max(bounds_min - p, max(p - bounds_max, float3(0.0f)));
  return dot(delta, delta);
}

float3 lp_mesh_unpack_normal(uint packed_normal)
{
  float2 oct = unpackSnorm2x16(packed_normal);
  float3 normal = float3(oct, 1.0f - abs(oct.x) - abs(oct.y));
  if (normal.z < 0.0f) {
    float2 s = float2(oct.x >= 0.0f ? 1.0f : -1.0f, oct.y >= 0.0f ? 1.0f : -1.0f);
    normal.xy = (float2(1.0f) - abs(oct.yx)) * s;
  }
  return normalize(normal);
}

float3 lp_mesh_vertex_position(int4 mesh_data, float4 mesh_scale, uint vertex)
{
  return uintBitsToFloat(mesh_data_buf[mesh_data.x + int(vertex)].xyz) * mesh_scale.xyz;
}

float3 lp_mesh_vertex_pseudonormal(int4 mesh_data, float4 mesh_scale, uint vertex)
{
  uint packed_normal = mesh_data_buf[mesh_data.x + int(vertex)].w;
  return normalize(lp_mesh_unpack_normal(packed_normal) / mesh_scale.xyz);
}

SDFMeshTriangleGPU lp_mesh_triangle_load(int4 mesh_data, int triangle)
{
  int start = mesh_data.y + triangle * 3;
  SDFMeshTriangleGPU result;
  result.vertices_and_material = mesh_data_buf[start];
  result.corner_normals = mesh_data_buf[start + 1];
  result.edge_normals = mesh_data_buf[start + 2];
  return result;
}

bool lp_mesh_triangle_stable_less(SDFMeshTriangleGPU a, SDFMeshTriangleGPU b)
{
  uint a_min = min(a.vertices_and_material.x,
                   min(a.vertices_and_material.y, a.vertices_and_material.z));
  uint a_max = max(a.vertices_and_material.x,
                   max(a.vertices_and_material.y, a.vertices_and_material.z));
  uint a_mid = a.vertices_and_material.x + a.vertices_and_material.y +
               a.vertices_and_material.z - a_min - a_max;
  uint b_min = min(b.vertices_and_material.x,
                   min(b.vertices_and_material.y, b.vertices_and_material.z));
  uint b_max = max(b.vertices_and_material.x,
                   max(b.vertices_and_material.y, b.vertices_and_material.z));
  uint b_mid = b.vertices_and_material.x + b.vertices_and_material.y +
               b.vertices_and_material.z - b_min - b_max;
  if (a_min != b_min) {
    return a_min < b_min;
  }
  if (a_mid != b_mid) {
    return a_mid < b_mid;
  }
  return a_max < b_max;
}

BVHNodeGPU lp_mesh_node_load(int4 mesh_data, int node)
{
  int start = mesh_data.w + node * 2;
  uint4 data_min = mesh_data_buf[start];
  uint4 data_max = mesh_data_buf[start + 1];
  BVHNodeGPU result;
  result.min_and_left = uintBitsToFloat(data_min);
  result.max_and_right = uintBitsToFloat(data_max);
  return result;
}

float3 lp_mesh_closest_point(float3 p,
                             float3 a,
                             float3 b,
                             float3 c,
                             out float3 barycentric)
{
  float3 ab = b - a;
  float3 ac = c - a;
  float3 ap = p - a;
  float d1 = dot(ab, ap);
  float d2 = dot(ac, ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    barycentric = float3(1.0f, 0.0f, 0.0f);
    return a;
  }

  float3 bp = p - b;
  float d3 = dot(ab, bp);
  float d4 = dot(ac, bp);
  if (d3 >= 0.0f && d4 <= d3) {
    barycentric = float3(0.0f, 1.0f, 0.0f);
    return b;
  }

  float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    float v = d1 / (d1 - d3);
    barycentric = float3(1.0f - v, v, 0.0f);
    return a + v * ab;
  }

  float3 cp = p - c;
  float d5 = dot(ab, cp);
  float d6 = dot(ac, cp);
  if (d6 >= 0.0f && d5 <= d6) {
    barycentric = float3(0.0f, 0.0f, 1.0f);
    return c;
  }

  float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    float w = d2 / (d2 - d6);
    barycentric = float3(1.0f - w, 0.0f, w);
    return a + w * ac;
  }

  float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    barycentric = float3(0.0f, 1.0f - w, w);
    return b + w * (c - b);
  }

  float denom = 1.0f / (va + vb + vc);
  float v = vb * denom;
  float w = vc * denom;
  barycentric = float3(1.0f - v - w, v, w);
  return a + ab * v + ac * w;
}

float3 lp_mesh_feature_normal(int4 mesh_data,
                              float4 mesh_scale,
                              SDFMeshTriangleGPU triangle,
                              float3 a,
                              float3 b,
                              float3 c,
                              float3 barycentric)
{
  const float feature_epsilon = 1e-5f;
  if (barycentric.x >= 1.0f - feature_epsilon) {
    return lp_mesh_vertex_pseudonormal(mesh_data, mesh_scale, triangle.vertices_and_material.x);
  }
  if (barycentric.y >= 1.0f - feature_epsilon) {
    return lp_mesh_vertex_pseudonormal(mesh_data, mesh_scale, triangle.vertices_and_material.y);
  }
  if (barycentric.z >= 1.0f - feature_epsilon) {
    return lp_mesh_vertex_pseudonormal(mesh_data, mesh_scale, triangle.vertices_and_material.z);
  }
  if (barycentric.x <= feature_epsilon) {
    return normalize(lp_mesh_unpack_normal(triangle.edge_normals.x) / mesh_scale.xyz);
  }
  if (barycentric.y <= feature_epsilon) {
    return normalize(lp_mesh_unpack_normal(triangle.edge_normals.y) / mesh_scale.xyz);
  }
  if (barycentric.z <= feature_epsilon) {
    return normalize(lp_mesh_unpack_normal(triangle.edge_normals.z) / mesh_scale.xyz);
  }
  return normalize(cross(b - a, c - a));
}

/* Last mesh hit on this thread (written by lp_sd_triangle_mesh), mirroring
 * the classic engine's g_sdf_mesh_last_*: lets the trace pass derive the
 * geometric/smooth normal from the winning triangle instead of paying for 4
 * finite-difference gradient evaluations per pixel. */
int g_lp_mesh_last_triangle;
float3 g_lp_mesh_last_barycentric;
float3 g_lp_mesh_last_geometric_normal;

bool lp_mesh_nearest(int4 mesh_data,
                     int mesh_node_count,
                     float4 mesh_scale,
                     float3 p,
                     out float distance_squared,
                     out int triangle_index,
                     out float3 closest_point,
                     out float3 barycentric,
                     out float3 feature_normal)
{
  distance_squared = 3.402823466e+38f;
  triangle_index = -1;
  closest_point = float3(0.0f);
  barycentric = float3(1.0f, 0.0f, 0.0f);
  feature_normal = float3(0.0f, 0.0f, 1.0f);
  if (mesh_data.z <= 0 || mesh_node_count <= 0) {
    return false;
  }

  /* Seed with the last triangle that won for this mesh: successive SDF
   * evaluations (sphere-tracing steps, gradient taps, prune cells) are
   * spatially coherent, so the hint gives an almost-tight distance bound up
   * front and the AABB test below prunes nearly the whole tree. */
  int cache_key = mesh_data.y;
  int cache_slot = (cache_key * 17) & (SDF_LP_MESH_HINT_CACHE_SIZE - 1);
  int hint_triangle = g_lp_mesh_hint_triangles[cache_slot];
  if (g_lp_mesh_hint_keys[cache_slot] == cache_key && hint_triangle >= 0 &&
      hint_triangle < mesh_data.z)
  {
    SDFMeshTriangleGPU triangle = lp_mesh_triangle_load(mesh_data, hint_triangle);
    float3 a = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.x);
    float3 b = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.y);
    float3 c = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.z);
    closest_point = lp_mesh_closest_point(p, a, b, c, barycentric);
    distance_squared = dot(p - closest_point, p - closest_point);
    triangle_index = hint_triangle;

    /* O(1) early-out: every triangle lies inside the root AABB, so its
     * point-to-box distance is a lower bound for the whole mesh. If the hint
     * is strictly closer than that bound, it is the unique closest triangle
     * and the full walk would return exactly this result. */
    BVHNodeGPU root_node = lp_mesh_node_load(mesh_data, 0);
    float root_dist_sq = lp_mesh_point_aabb_dist_sq(p,
                                                    root_node.min_and_left.xyz * mesh_scale.xyz,
                                                    root_node.max_and_right.xyz * mesh_scale.xyz);
    if (root_dist_sq > distance_squared * (1.0f + 1e-6f) + 1e-12f) {
      feature_normal = lp_mesh_feature_normal(
          mesh_data, mesh_scale, triangle, a, b, c, barycentric);
      return true;
    }
  }

  int local_node = 0;
  while (local_node < mesh_node_count) {
    BVHNodeGPU node = lp_mesh_node_load(mesh_data, local_node);
    int child_or_escape = floatBitsToInt(node.min_and_left.w);
    if (lp_mesh_point_aabb_dist_sq(p,
                                   node.min_and_left.xyz * mesh_scale.xyz,
                                   node.max_and_right.xyz * mesh_scale.xyz) >
        distance_squared)
    {
      local_node = child_or_escape >= 0 ? child_or_escape : local_node + 1;
      continue;
    }

    int child_b = floatBitsToInt(node.max_and_right.w);
    if (child_or_escape < 0) {
      int first = -child_or_escape - 1;
      for (int i = 0; i < child_b; i++) {
        int tri_i = first + i;
        SDFMeshTriangleGPU triangle = lp_mesh_triangle_load(mesh_data, tri_i);
        float3 a = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.x);
        float3 b = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.y);
        float3 c = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.z);
        float3 tri_barycentric = float3(0.0f);
        float3 tri_closest = lp_mesh_closest_point(p, a, b, c, tri_barycentric);
        float tri_distance_squared = dot(p - tri_closest, p - tri_closest);
        float tie_epsilon = max(1e-12f,
                                max(tri_distance_squared, distance_squared) * 1e-7f);
        bool is_near_tie = abs(tri_distance_squared - distance_squared) <= tie_epsilon;
        bool is_closer = tri_distance_squared < distance_squared && !is_near_tie;
        bool is_stable_tie = false;
        if (is_near_tie && triangle_index >= 0 && tri_i != triangle_index) {
          SDFMeshTriangleGPU current_triangle = lp_mesh_triangle_load(mesh_data, triangle_index);
          is_stable_tie = lp_mesh_triangle_stable_less(triangle, current_triangle);
        }
        if (is_closer || is_stable_tie) {
          distance_squared = tri_distance_squared;
          triangle_index = tri_i;
          closest_point = tri_closest;
          barycentric = tri_barycentric;
        }
      }
    }
    local_node++;
  }
  if (triangle_index >= 0) {
    /* Feature normal only for the final winner (up to 3 vertex loads +
     * unpacks); computing it per winner inside the walk is wasted work. */
    SDFMeshTriangleGPU triangle = lp_mesh_triangle_load(mesh_data, triangle_index);
    float3 a = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.x);
    float3 b = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.y);
    float3 c = lp_mesh_vertex_position(mesh_data, mesh_scale, triangle.vertices_and_material.z);
    feature_normal = lp_mesh_feature_normal(
        mesh_data, mesh_scale, triangle, a, b, c, barycentric);
    g_lp_mesh_hint_keys[cache_slot] = cache_key;
    g_lp_mesh_hint_triangles[cache_slot] = triangle_index;
  }
  return triangle_index >= 0;
}

/* Signed distance to a triangle mesh (identical to sdTriangleMesh). Also
 * records the winning triangle for lp_mesh_last_world_normals. */
float lp_sd_triangle_mesh(float3 p, int4 mesh_data, int mesh_node_count, float4 mesh_scale)
{
  g_lp_mesh_last_triangle = -1;
  float distance_squared;
  int triangle_index;
  float3 closest_point;
  float3 barycentric;
  float3 feature_normal;
  if (!lp_mesh_nearest(mesh_data,
                       mesh_node_count,
                       mesh_scale,
                       p,
                       distance_squared,
                       triangle_index,
                       closest_point,
                       barycentric,
                       feature_normal))
  {
    return 1e10f;
  }
  float sign_value = dot(p - closest_point, feature_normal) < 0.0f ? -1.0f : 1.0f;
  g_lp_mesh_last_triangle = triangle_index;
  g_lp_mesh_last_barycentric = barycentric;
  g_lp_mesh_last_geometric_normal = distance_squared > 1e-12f ?
                                        normalize(p - closest_point) * sign_value :
                                        feature_normal;
  return sqrt(max(distance_squared, 0.0f)) * sign_value;
}

/* World-space normals of the last mesh hit (port of sdfMeshLastWorldNormals,
 * sdf_mesh_lib.glsl:318), built from data already fetched by the distance
 * evaluation: one extra triangle record load, zero extra SDF evaluations.
 * shading_normal interpolates the triangle's corner normals — the smooth
 * normal a regular mesh would shade with; geometric_normal is the true SDF
 * gradient direction. Returns false when there is no valid mesh hit, in
 * which case the caller falls back to the finite-difference gradient. */
bool lp_mesh_last_world_normals(SDFObjectGPU obj,
                                out float3 shading_normal,
                                out float3 geometric_normal)
{
  if (g_lp_mesh_last_triangle < 0) {
    shading_normal = float3(0.0f);
    geometric_normal = float3(0.0f);
    return false;
  }

  /* transpose(to_float3x3(obj.inverse_matrix)), written out so this file
   * does not depend on the matrix conversion lib. */
  float3x3 normal_to_world = float3x3(
      float3(obj.inverse_matrix[0][0], obj.inverse_matrix[1][0], obj.inverse_matrix[2][0]),
      float3(obj.inverse_matrix[0][1], obj.inverse_matrix[1][1], obj.inverse_matrix[2][1]),
      float3(obj.inverse_matrix[0][2], obj.inverse_matrix[1][2], obj.inverse_matrix[2][2]));
  geometric_normal = normal_to_world * g_lp_mesh_last_geometric_normal * obj.obj_scale.w;
  float geometric_len_squared = dot(geometric_normal, geometric_normal);
  if (geometric_len_squared <= 1e-12f || any(isnan(geometric_normal))) {
    shading_normal = float3(0.0f);
    geometric_normal = float3(0.0f);
    return false;
  }
  SDFMeshTriangleGPU triangle = lp_mesh_triangle_load(obj.mesh_data, g_lp_mesh_last_triangle);
  float3 n0 = normalize(normal_to_world *
                        (lp_mesh_unpack_normal(triangle.corner_normals.x) / obj.obj_scale.xyz));
  float3 n1 = normalize(normal_to_world *
                        (lp_mesh_unpack_normal(triangle.corner_normals.y) / obj.obj_scale.xyz));
  float3 n2 = normalize(normal_to_world *
                        (lp_mesh_unpack_normal(triangle.corner_normals.z) / obj.obj_scale.xyz));
  shading_normal = n0 * g_lp_mesh_last_barycentric.x + n1 * g_lp_mesh_last_barycentric.y +
                   n2 * g_lp_mesh_last_barycentric.z;
  float shading_len_squared = dot(shading_normal, shading_normal);
  if (shading_len_squared <= 1e-12f || any(isnan(shading_normal))) {
    shading_normal = geometric_normal;
    shading_len_squared = geometric_len_squared;
  }
  shading_normal *= inversesqrt(shading_len_squared);
  float alignment = dot(shading_normal, geometric_normal) / geometric_len_squared;
  if (alignment < 0.0f) {
    shading_normal -= 2.0f * alignment * geometric_normal;
  }
  shading_normal *= sqrt(geometric_len_squared);
  return true;
}

/** \} */

/* ------------------------------------------------------------------ */
/** \name Object modifiers (port of sdf_lib.glsl applyDomainModifiers :1208
 * and applyDistanceModifiers :1865)
 *
 * Modifier type ids (must match eSDFModifierType in DNA_sdf_types.h). */
#define SDF_LP_MOD_MIRROR 0
#define SDF_LP_MOD_TWIST 1
#define SDF_LP_MOD_BEND 2
#define SDF_LP_MOD_ELONGATE 3
#define SDF_LP_MOD_SOLIDIFY 4
#define SDF_LP_MOD_ROUND 5
#define SDF_LP_MOD_ONION 6
#define SDF_LP_MOD_BEVEL 7
#define SDF_LP_MOD_ARRAY 8
#define SDF_LP_MOD_DISPLACE 9

#define SDF_LP_MOD_MIRROR_X 1
#define SDF_LP_MOD_MIRROR_Y 2
#define SDF_LP_MOD_MIRROR_Z 4

#define SDF_LP_MOD_ARRAY_LINEAR 0
#define SDF_LP_MOD_ARRAY_RADIAL 1

#define SDF_LP_MOD_DISPLACE_NOISE 0
#define SDF_LP_MOD_DISPLACE_VORONOI 1
#define SDF_LP_MOD_DISPLACE_TRIANGLE 2
#define SDF_LP_MOD_DISPLACE_POINTS 3

/* Smooth absolute value (identical to sabs, sdf_lib.glsl:982). */
float lp_sabs(float x, float k)
{
  if (k <= 0.0001f) {
    return abs(x);
  }
  float h = clamp(0.5f + 0.5f * x / k, 0.0f, 1.0f);
  return x * (2.0f * h - 1.0f) + k * h * (1.0f - h);
}

/* ---- Noise functions for displacement (sdf_lib.glsl:1005-1185) ---- */

float3 lp_sdf_hash33(float3 p)
{
  p = fract(p * float3(0.1031f, 0.1030f, 0.0973f));
  p += dot(p, p.yxz + 33.33f);
  return fract((p.xxy + p.yxx) * p.zyx);
}

float lp_sdf_hash31(float3 p)
{
  p = fract(p * 0.1031f);
  p += dot(p, p.zyx + 31.32f);
  return fract((p.x + p.y) * p.z);
}

float lp_sdf_value_noise(float3 p)
{
  float3 i = floor(p);
  float3 f = fract(p);
  float3 u = f * f * f * (f * (f * 6.0f - 15.0f) + 10.0f);

  float a = lp_sdf_hash31(i);
  float b = lp_sdf_hash31(i + float3(1, 0, 0));
  float c = lp_sdf_hash31(i + float3(0, 1, 0));
  float d = lp_sdf_hash31(i + float3(1, 1, 0));
  float e = lp_sdf_hash31(i + float3(0, 0, 1));
  float ff = lp_sdf_hash31(i + float3(1, 0, 1));
  float g = lp_sdf_hash31(i + float3(0, 1, 1));
  float h = lp_sdf_hash31(i + float3(1, 1, 1));

  return mix(mix(mix(a, b, u.x), mix(c, d, u.x), u.y),
             mix(mix(e, ff, u.x), mix(g, h, u.x), u.y), u.z) * 2.0f - 1.0f;
}

float lp_sdf_fbm_noise(float3 p, int octaves, float lacunarity, float roughness)
{
  float sum = 0.0f;
  float amp = 1.0f;
  float max_amp = 0.0f;
  for (int i = 0; i < octaves; i++) {
    sum += lp_sdf_value_noise(p) * amp;
    max_amp += amp;
    p *= lacunarity;
    amp *= roughness;
  }
  return sum / max_amp;
}

float4 lp_sdf_voronoi_grad(float3 p)
{
  float3 i = floor(p);
  float3 f = fract(p);
  float min_d2 = 100.0f;
  float3 nearest_r = float3(0);
  for (int z = -1; z <= 1; z++) {
    for (int y = -1; y <= 1; y++) {
      for (int x = -1; x <= 1; x++) {
        float3 b = float3(float(x), float(y), float(z));
        float3 r = b - f + lp_sdf_hash33(i + b);
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

float lp_sdf_voronoi(float3 p)
{
  return lp_sdf_voronoi_grad(p).x;
}

float lp_sdf_fbm_voronoi(float3 p, int octaves, float lacunarity, float roughness)
{
  float sum = 0.0f;
  float amp = 1.0f;
  float max_amp = 0.0f;
  for (int i = 0; i < octaves; i++) {
    sum += (1.0f - lp_sdf_voronoi(p)) * amp;
    max_amp += amp;
    p *= lacunarity;
    amp *= roughness;
  }
  return sum / max_amp;
}

float lp_sdf_diamond_2d(float2 p)
{
  float2 cell = fract(p) - 0.5f;
  return 1.0f - (abs(cell.x) + abs(cell.y)) * 2.0f;
}

float lp_sdf_triangle_grid(float3 p)
{
  float n1 = lp_sdf_diamond_2d(p.xy);
  float n2 = lp_sdf_diamond_2d(p.yz);
  float n3 = lp_sdf_diamond_2d(p.zx);
  float n = max(n1, max(n2, n3));
  float c = clamp(n, 0.0f, 1.0f);
  return c * c;
}

float lp_sdf_points(float3 p)
{
  float3 f = fract(p) - 0.5f;
  float d = length(f);
  return smoothstep(0.45f, 0.0f, d);
}

float lp_sdf_fbm_triangle(float3 p, int octaves, float lacunarity, float roughness)
{
  float sum = 0.0f;
  float amp = 1.0f;
  float max_amp = 0.0f;
  for (int i = 0; i < octaves; i++) {
    sum += lp_sdf_triangle_grid(p) * amp;
    max_amp += amp;
    p *= lacunarity;
    amp *= roughness;
  }
  return sum / max_amp;
}

/* Cheap displacement for ray march (identical to sdf_displacement_fast,
 * sdf_lib.glsl:1172). */
float lp_sdf_displacement_fast(float3 p, int noise_type, int octaves, float lacunarity, float roughness)
{
  int fast_oct = max(min(octaves, 2), 1);
  if (noise_type == SDF_LP_MOD_DISPLACE_VORONOI) {
    return lp_sdf_fbm_voronoi(p, 1, lacunarity, roughness);
  }
  if (noise_type == SDF_LP_MOD_DISPLACE_TRIANGLE) {
    return lp_sdf_fbm_triangle(p, fast_oct, lacunarity, roughness);
  }
  if (noise_type == SDF_LP_MOD_DISPLACE_POINTS) {
    return lp_sdf_points(p);
  }
  return lp_sdf_fbm_noise(p, fast_oct, lacunarity, roughness);
}

/* Domain modifiers warp the sampling space (applied in reverse order, like
 * the classic engine). Returns the warped position and a conservative
 * distance-scale correction (soft mirror/array x0.5, twist/bend
 * 1/(1+|k|*r)) in .w. Identical to applyDomainModifiers (sdf_lib.glsl:1208);
 * the inverse-matrix columns used by MIRROR are reconstructed from the
 * primitive's stored rows (col_i = (m_row0[i], m_row1[i], m_row2[i])). */
float4 lp_apply_domain_modifiers(float3 p, SDFLpPrimitive prim)
{
  float scale = 1.0f;
  float3 inv_col0 = float3(prim.m_row0.x, prim.m_row1.x, prim.m_row2.x);
  float3 inv_col1 = float3(prim.m_row0.y, prim.m_row1.y, prim.m_row2.y);
  float3 inv_col2 = float3(prim.m_row0.z, prim.m_row1.z, prim.m_row2.z);
  for (int i = prim.modifier_start + prim.modifier_count - 1; i >= prim.modifier_start; i--) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;
    int mflags = smod.header.y;

    if (mtype == SDF_LP_MOD_MIRROR) {
      float offset = smod.params.x;
      float3 origin = smod.params.yzw;
      float blend = smod.params2.x;
      int blend_type = smod.header.z;
      int sides = smod.header.w;
      float bk = (blend_type > 0 && blend > 0.001f) ? blend : 0.0f;
      if ((mflags & SDF_LP_MOD_MIRROR_X) != 0) {
        float3 N = inv_col0;
        float s = ((sides & 1) != 0) ? 1.0f : -1.0f;
        N *= s;
        float nl2 = max(dot(N, N), 1e-12f);
        float d = dot(p - origin, N) / nl2;
        p -= (d - lp_sabs(d, bk)) * N;
        p -= offset * N;
      }
      if ((mflags & SDF_LP_MOD_MIRROR_Y) != 0) {
        float3 N = inv_col1;
        float s = ((sides & 2) != 0) ? 1.0f : -1.0f;
        N *= s;
        float nl2 = max(dot(N, N), 1e-12f);
        float d = dot(p - origin, N) / nl2;
        p -= (d - lp_sabs(d, bk)) * N;
        p -= offset * N;
      }
      if ((mflags & SDF_LP_MOD_MIRROR_Z) != 0) {
        float3 N = inv_col2;
        float s = ((sides & 4) != 0) ? 1.0f : -1.0f;
        N *= s;
        float nl2 = max(dot(N, N), 1e-12f);
        float d = dot(p - origin, N) / nl2;
        p -= (d - lp_sabs(d, bk)) * N;
        p -= offset * N;
      }
      if (bk > 0.001f) {
        scale *= 0.5f;
      }
    }
    else if (mtype == SDF_LP_MOD_TWIST) {
      float k = smod.params.x;
      int axis = int(smod.params.y);
      float drive, r;
      if (axis == 1) {
        drive = p.y;
        r = length(float2(p.x, p.z));
      }
      else if (axis == 2) {
        drive = p.x;
        r = length(float2(p.y, p.z));
      }
      else {
        drive = p.z;
        r = length(float2(p.x, p.y));
      }
      float angle = k * drive;
      float c = cos(angle);
      float s = sin(angle);
      if (axis == 1) {
        p = float3(c * p.x - s * p.z, p.y, s * p.x + c * p.z);
      }
      else if (axis == 2) {
        p = float3(p.x, c * p.y - s * p.z, s * p.y + c * p.z);
      }
      else {
        p = float3(c * p.x - s * p.y, s * p.x + c * p.y, p.z);
      }
      scale *= 1.0f / (1.0f + abs(k) * r);
    }
    else if (mtype == SDF_LP_MOD_BEND) {
      float k = smod.params.x;
      int axis = int(smod.params.y);
      float3 origin = float3(smod.params.z, smod.params.w, smod.params2.x);
      p -= origin;
      if (abs(k) > 0.0001f) {
        float drive, curve, r;
        if (axis == 1) {
          drive = p.y;
          curve = p.z;
          r = length(float2(p.y, p.z));
          float a = k * drive;
          float c = cos(a);
          float s = sin(a);
          p.y = c * drive - s * curve;
          p.z = s * drive + c * curve;
        }
        else if (axis == 2) {
          drive = p.z;
          curve = p.x;
          r = length(float2(p.z, p.x));
          float a = k * drive;
          float c = cos(a);
          float s = sin(a);
          p.z = c * drive - s * curve;
          p.x = s * drive + c * curve;
        }
        else {
          drive = p.x;
          curve = p.y;
          r = length(float2(p.x, p.y));
          float a = k * drive;
          float c = cos(a);
          float s = sin(a);
          p.x = c * drive - s * curve;
          p.y = s * drive + c * curve;
        }
        scale *= 1.0f / (1.0f + abs(k) * r);
      }
      p += origin;
    }
    else if (mtype == SDF_LP_MOD_ELONGATE) {
      float3 h = smod.params.xyz;
      p = p - clamp(p, -h, h);
    }
    else if (mtype == SDF_LP_MOD_ARRAY) {
      float count = smod.params.x;
      float blend = smod.params2.x;
      /* Array domain blend is always smooth. */
      float bk = (blend > 0.001f) ? blend : 0.0f;
      if (mflags == SDF_LP_MOD_ARRAY_LINEAR) {
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
            float pull_r = d_r - lp_sabs(d_r, bk);
            float d_l = (0.5f + local) * spacing;
            float pull_l = d_l - lp_sabs(d_l, bk);
            if (id < 0.5f) {
              pull_l = 0.0f;
            }
            if (id > count - 1.5f) {
              if (mirrored) {
                pull_l = 0.0f;
              }
              else {
                pull_r = 0.0f;
              }
            }
            local += (pull_r - pull_l) / spacing;
          }

          p += dir * (local * spacing - t);
        }
      }
      else if (mflags == SDF_LP_MOD_ARRAY_RADIAL) {
        float radius = smod.params.y;
        if (count > 1.5f) {
          float sector = (2.0f * SDF_LP_PI) / count;
          float angle = atan(p.y, p.x);
          float norm_a = angle / sector;
          float id = round(norm_a);
          float local = norm_a - id;

          {
            bool mir = fract(abs(id) * 0.5f) > 0.25f;
            bool odd = fract(count * 0.5f) > 0.25f;
            bool at_defect = odd && (abs(angle) > SDF_LP_PI - sector * 0.5f);
            if (at_defect) {
              local = abs(local);
            }
            else if (mir) {
              local = -local;
            }

            float arc = sector * max(radius, 0.0001f);
            float d_r = (0.5f - local) * arc;
            float pull_r = d_r - lp_sabs(d_r, bk);
            float d_l = (0.5f + local) * arc;
            float pull_l = d_l - lp_sabs(d_l, bk);
            local += (pull_r - pull_l) / arc;
            if (bk > 0.001f) {
              scale *= 0.5f;
            }
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

/** \} */

/* Smooth intersection (identical to opSmoothIntersection, sdf_lib.glsl:685);
 * used by the SOLIDIFY distance modifier's bevel. */
float lp_op_smooth_intersection(float d1, float d2, float k)
{
  if (k <= 0.0001f) {
    return max(d1, d2);
  }
  if (abs(d2 - d1) >= k) {
    return max(d1, d2);
  }
  float h = clamp(0.5f - 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
  return mix(d2, d1, h) + k * h * (1.0f - h);
}

/* Primitive-only evaluation (identical to evalPrimitiveOnly,
 * sdf_lib.glsl:1739-1859): dispatches on the primitive type including the
 * advanced variants, applies the coordinate scale, and subtracts the
 * effective bevel. The caller multiplies by prim.scale.w. Takes the local
 * (pre-scale) position. */
float lp_eval_prim_base(float3 p, SDFLpPrimitive prim)
{
  float3 lp = p;

  float3 r = prim.size.xyz;
  float dist;
  if (prim.type == SDF_GPU_TYPE_MESH) {
    /* Mesh vertices carry the object scale themselves (classic parity). */
    dist = lp_sd_triangle_mesh(lp, prim.mesh_data, prim.mesh_node_count, prim.scale);
  }
  else {
    lp /= prim.scale.xyz;
    if (prim.type == SDF_GPU_TYPE_SPHERE) {
      dist = (abs(r.x - r.y) < 0.0001f && abs(r.x - r.z) < 0.0001f) ?
                 lp_sd_sphere(lp, r.x) :
                 lp_sd_ellipsoid(lp, r);
    }
    else if (prim.type == SDF_GPU_TYPE_CYLINDER) {
      dist = lp_sd_cylinder(lp, r);
    }
    else if (prim.type == SDF_GPU_TYPE_CONE) {
      dist = lp_sd_cone_frustum(lp, r.x, r.z, r.y);
    }
    else if (prim.type == SDF_GPU_TYPE_CAPSULE) {
      dist = lp_sd_capsule(lp, r);
    }
    else if (prim.type == SDF_GPU_TYPE_TORUS) {
      dist = (prim.box_modes.w != 0) ?
                 lp_sd_capped_torus(lp, prim.box_corners.xy, r.x, r.y) :
                 lp_sd_torus(lp, float2(r.x, r.y));
    }
    else if (prim.type == SDF_GPU_TYPE_NGON) {
      int sides = prim.box_modes.z;
      float corner = prim.box_corners.x;
      float star = prim.box_corners.y;
      float edgeTop = prim.box_edges.x;
      float edgeBot = prim.box_edges.y;
      float tapTop = prim.box_edges.z;
      float tapBot = prim.box_edges.w;
      int edgeMode = prim.box_modes.y;
      if ((corner + edgeTop + edgeBot + tapTop + tapBot + star) > 0.001f) {
        dist = lp_sd_advanced_ngon(
            lp, r.x, r.z, sides, corner, edgeTop, edgeBot, tapTop, tapBot, edgeMode, r.z, star);
      }
      else {
        float d2d = lp_sd_regular_polygon_2d(lp.xy, r.x, sides);
        float dz = abs(lp.z) - r.z;
        float2 dd = float2(d2d, dz);
        dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
      }
    }
    else if (prim.type == SDF_GPU_TYPE_POLYGON) {
      int ps = prim.aux0;
      int pc = prim.aux1;
      float edgeTop = prim.box_edges.x;
      float edgeBot = prim.box_edges.y;
      float tapTop = prim.box_edges.z;
      float tapBot = prim.box_edges.w;
      int edgeMode = prim.box_modes.y;
      if (pc >= 3) {
        if ((edgeTop + edgeBot + tapTop + tapBot) > 0.001f || prim.box_corners.x > 0.001f) {
          dist = lp_sd_advanced_polygon(
              lp, r.z, ps, pc, edgeTop, edgeBot, tapTop, tapBot, edgeMode, r.z);
        }
        else {
          float d2d = (prim.auxf > 0.001f) ? lp_sd_polygon_2d_rounded(lp.xy, ps, pc) :
                                             lp_sd_polygon_2d(lp.xy, ps, pc);
          float dz = abs(lp.z) - r.z;
          float2 dd = float2(d2d, dz);
          dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
        }
      }
      else {
        dist = 1e10f;
      }
    }
    else { /* SDF_GPU_TYPE_BOX and fallback */
      float4 corners = prim.box_corners;
      float edgeTop = prim.box_edges.x;
      float edgeBot = prim.box_edges.y;
      float tapTop = prim.box_edges.z;
      float tapBot = prim.box_edges.w;
      if ((corners.x + corners.y + corners.z + corners.w + edgeTop + edgeBot + tapTop + tapBot) >
          0.001f)
      {
        dist = lp_sd_advanced_box(lp,
                                  r,
                                  corners,
                                  edgeTop,
                                  edgeBot,
                                  tapTop,
                                  tapBot,
                                  prim.box_modes.x,
                                  prim.box_modes.y,
                                  r.z);
      }
      else {
        dist = lp_sd_box(lp, r);
      }
    }
  }

  return dist - prim.size.w;
}

/* Distance modifiers (forward stack order), identical to
 * applyDistanceModifiers (sdf_lib.glsl:1865-1940). DISPLACE keeps the
 * classic renormalization dist /= 1 + |strength|*frequency*lip so the field
 * stays conservative. SOLIDIFY open re-evaluates the primitive at an
 * axis-scaled point via lp_eval_prim_base (no recursion through modifiers). */
float lp_apply_distance_modifiers(float dist, float3 p, SDFLpPrimitive prim)
{
  for (int i = prim.modifier_start; i < prim.modifier_start + prim.modifier_count; i++) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;

    if (mtype == SDF_LP_MOD_SOLIDIFY) {
      float thickness = smod.params.x;
      float bevel = smod.params.z;
      int mode = smod.header.y;

      if (mode == 0) {
        /* Closed */
        float d_inner = -(dist + thickness);
        if (bevel > 0.0f) {
          dist = lp_op_smooth_intersection(dist, d_inner, bevel);
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
        float d_inner = -(lp_eval_prim_base(p_inner, prim) + thickness);
        if (bevel > 0.0f) {
          dist = lp_op_smooth_intersection(dist, d_inner, bevel);
        }
        else {
          dist = max(dist, d_inner);
        }
      }
    }
    else if (mtype == SDF_LP_MOD_ROUND) {
      dist -= smod.params.x;
    }
    else if (mtype == SDF_LP_MOD_BEVEL && prim.type == SDF_GPU_TYPE_MESH) {
      dist -= smod.params.x;
    }
    else if (mtype == SDF_LP_MOD_DISPLACE) {
      float strength = smod.params.x;
      float frequency = smod.params.y;
      float lacunarity = smod.params.z;
      float roughness = smod.params.w;
      int noise_type = smod.header.y;
      int octaves = smod.header.z;
      float n = lp_sdf_displacement_fast(p * frequency, noise_type, octaves, lacunarity, roughness);
      /* Per-type Lipschitz bound */
      float lip = 1.0f;
      if (noise_type == SDF_LP_MOD_DISPLACE_POINTS) {
        lip = 3.5f;
      }
      else if (noise_type == SDF_LP_MOD_DISPLACE_TRIANGLE) {
        lip = 3.0f;
      }
      else if (noise_type == SDF_LP_MOD_DISPLACE_VORONOI) {
        lip = 1.5f;
      }
      dist += n * strength;
      dist /= (1.0f + abs(strength) * frequency * lip);
    }
    else if (mtype == SDF_LP_MOD_ONION) {
      int layers = max(smod.header.y, 1);
      float cut_half = max(smod.params.x, 0.001f) * 0.5f;
      float min_ext = smod.params.y;
      float original_d = dist;
      if (layers > 1) {
        float spacing = min_ext / float(layers);
        float depth = max(-dist, 0.0f);
        float max_cut = float(layers - 1) * spacing;
        float nearest = clamp(floor(depth / spacing + 0.5f) * spacing, spacing, max_cut);
        float cut_dist = abs(depth - nearest);
        float onion_d = cut_half - cut_dist;
        dist = max(original_d, onion_d);
      }
    }
  }
  return dist;
}

/* Full primitive evaluation (identical to evalObjectSDF, sdf_lib.glsl:1979):
 * domain modifiers -> primitive eval -> x scale.w -> distance modifiers ->
 * x domain-warp correction scale. The modifier chain is skipped entirely
 * when the object has no modifiers (branch-cheap fast path). */
float lp_eval_prim(float3 p, SDFLpPrimitive prim)
{
  float3 d4 = p - prim.position.xyz;
  float3 lp = float3(dot(prim.m_row0, float4(d4, 1.0f)),
                     dot(prim.m_row1, float4(d4, 1.0f)),
                     dot(prim.m_row2, float4(d4, 1.0f)));

  if (prim.modifier_count == 0) {
    return lp_eval_prim_base(lp, prim) * prim.scale.w;
  }

  float4 dm = lp_apply_domain_modifiers(lp, prim);
  float3 pw = dm.xyz;
  float d = lp_eval_prim_base(pw, prim) * prim.scale.w;
  d = lp_apply_distance_modifiers(d, pw, prim);
  return d * dm.w;
}

/** \} */

/* ------------------------------------------------------------------ */
/** \name Active node list evaluation (post-order stack machine)
 *
 * KNOWN GAP vs the classic engine: group-level distance modifiers
 * (applyGroupDistanceModifiers, sdf_lib.glsl:1943-1973) are NOT evaluated
 * here. LP folds each group into a plain CSG subtree on the CPU, which has
 * no hook to attach the group's modifier stack to the folded distance.
 * Objects inside groups with distance modifiers will evaluate without those
 * modifiers. Not implemented by design (would need per-subtree modifier
 * ranges in the serialized node format).
 * \{ */

uint lp_active_node_index(uint n)
{
  return n & ~SDF_LP_SIGN_BIT;
}

float lp_active_node_sign(uint n)
{
  return ((n & SDF_LP_SIGN_BIT) == 0u) ? 1.0f : -1.0f;
}

float lp_offset_node_value(SDFLpNode node)
{
  return uintBitsToFloat(uint(node.idx_in_type));
}

/* Evaluate `count` nodes of the active list starting at `base`.
 * The stack is deliberately NOT zero-initialized: the post-order list only
 * ever reads entries it has pushed (same as the reference eval.glsl), and the
 * init loop costs SDF_LP_STACK_DEPTH writes on every trace step. */
float lp_list_eval(float3 p, int count, int base)
{
  float stack[SDF_LP_STACK_DEPTH];
  int stack_idx = 0;

  for (int i = 0; i < count; i++) {
    uint active_node = lp_active_in[base + i].x;
    SDFLpNode node = lp_nodes[lp_active_node_index(active_node)];

    float d;
    if (node.type == SDF_LP_NODETYPE_BINARY) {
      float left_val = stack[stack_idx - 2];
      float right_val = stack[stack_idx - 1];
      stack_idx -= 2;
      uint op = lp_binary_ops[node.idx_in_type];
      d = lp_binary_op_eval(op, left_val, right_val);
    }
    else if (node.type == SDF_LP_NODETYPE_OFFSET) {
      d = stack[stack_idx - 1] + lp_offset_node_value(node);
      stack_idx -= 1;
    }
    else {
      SDFLpPrimitive prim = lp_prims[node.idx_in_type];
      d = lp_eval_prim(p, prim);
    }

    d *= lp_active_node_sign(active_node);
    if (stack_idx >= SDF_LP_STACK_DEPTH) {
      return 1e20f;
    }
    stack[stack_idx++] = d;
  }

  return stack[0];
}

/* Folded color evaluation: primitive albedos blended with smooth-min weights.
 * Also folds the sorted object id (dominant operand wins). */
float4 lp_list_eval_color(float3 p, int count, int base, out float out_obj_id)
{
  float d_stack[SDF_LP_STACK_DEPTH];
  float3 c_stack[SDF_LP_STACK_DEPTH];
  float i_stack[SDF_LP_STACK_DEPTH];
  /* No stack initialization: see lp_list_eval. */
  int stack_idx = 0;

  for (int i = 0; i < count; i++) {
    uint active_node = lp_active_in[base + i].x;
    SDFLpNode node = lp_nodes[lp_active_node_index(active_node)];

    float d;
    float3 albedo;
    float obj_id;
    if (node.type == SDF_LP_NODETYPE_BINARY) {
      float left_val = d_stack[stack_idx - 2];
      float right_val = d_stack[stack_idx - 1];
      float3 left_col = c_stack[stack_idx - 2];
      float3 right_col = c_stack[stack_idx - 1];
      float left_id = i_stack[stack_idx - 2];
      float right_id = i_stack[stack_idx - 1];
      stack_idx -= 2;
      uint op = lp_binary_ops[node.idx_in_type];
      d = lp_binary_op_eval(op, left_val, right_val);
      if (lp_op_csg(op) == SDF_LP_CSG_PAINT) {
        /* Paint only recolors where the right operand is inside-ish. */
        float cb = lp_op_blend_factor(op);
        float t = (cb > 0.0f) ?
                      1.0f - smoothstep(-cb, cb, right_val) :
                      ((right_val < 0.0f) ? 1.0f : 0.0f);
        albedo = mix(left_col, right_col, t);
        obj_id = (t >= 0.5f) ? right_id : left_id;
      }
      else {
        float s = lp_op_sign(op);
        float k = lp_op_blend_factor(op);
        float2 v = s * lp_smin_blend(s * left_val, s * right_val, k);
        albedo = mix(left_col, right_col, v.y);
        obj_id = (v.y < 0.5f) ? left_id : right_id;
      }
    }
    else if (node.type == SDF_LP_NODETYPE_OFFSET) {
      d = d_stack[stack_idx - 1] + lp_offset_node_value(node);
      albedo = c_stack[stack_idx - 1];
      obj_id = i_stack[stack_idx - 1];
      stack_idx -= 1;
    }
    else {
      SDFLpPrimitive prim = lp_prims[node.idx_in_type];
      d = lp_eval_prim(p, prim);
      albedo = prim.color.rgb;
      obj_id = prim.color.w;
    }

    d *= lp_active_node_sign(active_node);
    if (stack_idx >= SDF_LP_STACK_DEPTH) {
      out_obj_id = -1.0f;
      return float4(0.0f);
    }
    d_stack[stack_idx] = d;
    c_stack[stack_idx] = albedo;
    i_stack[stack_idx] = obj_id;
    stack_idx++;
  }

  out_obj_id = i_stack[0];
  return float4(c_stack[0], 1.0f);
}

/** \} */

/* ------------------------------------------------------------------ */
/** \name Morton cell indexing (64 consecutive codes per 4x4x4 block,
 * so parent cell = cell_idx / 64)
 * \{ */

uint lp_part1by2(uint x)
{
  x &= 0x000003ffu;
  x = (x ^ (x << 16)) & 0xff0000ffu;
  x = (x ^ (x << 8)) & 0x0300f00fu;
  x = (x ^ (x << 4)) & 0x030c30c3u;
  x = (x ^ (x << 2)) & 0x09249249u;
  return x;
}

uint lp_cell_idx(int3 cell)
{
  return (lp_part1by2(uint(cell.z)) << 2) + (lp_part1by2(uint(cell.y)) << 1) +
         lp_part1by2(uint(cell.x));
}

int3 lp_cell_from_pos(float3 p, float3 aabb_min, float3 cell_size, int grid_size)
{
  int3 cell = int3((p - aabb_min) / cell_size);
  return clamp(cell, int3(0), int3(grid_size - 1));
}

/** \} */

/* ------------------------------------------------------------------ */
/** \name Heatmap (inferno polynomial approximation)
 * \{ */

float3 lp_inferno(float t)
{
  const float3 c0 = float3(0.0002189403691192265f, 0.001651004631001012f, -0.01948089843709184f);
  const float3 c1 = float3(0.1065134194856116f, 0.5639564367884091f, 3.932712388889277f);
  const float3 c2 = float3(11.60249308247187f, -3.972853965665698f, -15.9423941062914f);
  const float3 c3 = float3(-41.70399613139459f, 17.43639888205313f, 44.35414519872813f);
  const float3 c4 = float3(77.162935699427f, -33.40235894210092f, -81.80730925738993f);
  const float3 c5 = float3(-71.31942824499214f, 32.62606426397723f, 73.20951985803202f);
  const float3 c6 = float3(25.13112622477341f, -12.24266895238567f, -23.07032500287172f);
  return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));
}

/** \} */
