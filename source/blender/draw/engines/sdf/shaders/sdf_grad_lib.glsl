/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/* Analytical SDF gradient library.
 * Every function returns float4(distance, gradient.xyz).
 * Reference: iquilezles.org/articles/distgradfunctions3d */

/* ---- Primitive gradients ---- */

float4 sdgSphere(float3 p, float r)
{
  float l = length(p);
  return float4(l - r, p / max(l, 1e-8f));
}

float4 sdgEllipsoid(float3 p, float3 r)
{
  float3 pr = p / r;
  float k0 = length(pr);
  float3 pr2 = p / (r * r);
  float k1 = length(pr2);
  float dist = k0 * (k0 - 1.0f) / max(k1, 1e-8f);
  float3 grad = pr2 / max(k1, 1e-8f);
  return float4(dist, grad);
}

float4 sdgBox(float3 p, float3 b)
{
  float3 w = abs(p) - b;
  float3 s = sign(p);
  float g = max(w.x, max(w.y, w.z));
  float3 q = max(w, float3(0.0f));
  float l = length(q);
  float4 f = (g > 0.0f)
      ? float4(l, q / max(l, 1e-8f))
      : float4(g, float3(
            (w.x == g) ? 1.0f : 0.0f,
            (w.y == g) ? 1.0f : 0.0f,
            (w.z == g) ? 1.0f : 0.0f));
  return float4(f.x, f.yzw * s);
}

float4 sdgCylinder(float3 p, float3 size)
{
  /* Must match sdCylinder's distance decomposition exactly for correct
   * bevel corner blending. Uses (rn-1)/length(g) for radial, not rn-1. */
  float2 e = max(size.xy, float2(0.001f));
  float2 pn = p.xy / e;
  float rn = max(length(pn), 1e-6f);
  float2 g = pn / (e * rn);
  float gl = max(length(g), 1e-6f);
  float radial = (rn - 1.0f) / gl;
  float vertical = abs(p.z) - size.z;

  float3 du = float3(g / gl, 0.0f);
  float3 dv = float3(0.0f, 0.0f, sign(p.z));

  float2 dd = float2(radial, vertical);
  float2 hh = max(dd, float2(0.0f));
  float fl = length(hh);
  float fg = max(dd.x, dd.y);

  if (fg <= 0.0f) {
    return float4(fg, (dd.x > dd.y) ? du : dv);
  }
  return float4(fl, (hh.x * du + hh.y * dv) / max(fl, 1e-8f));
}

float4 sdgCone(float3 p, float r, float h)
{
  /* Capped cone (Z-up): base radius r at z=-h, apex at z=+h.
   * Side gradient uses b/|b| (from closest point) instead of fixed perpendicular —
   * this is correct when the closest side point is at an endpoint (clamped t),
   * and naturally agrees with the cap gradient at the junction. */
  float2 k = float2(0.0f - r, 2.0f * h);
  float m = dot(k, k);
  float l = length(p.xy);
  float2 q = float2(0.0f - l, h - p.z);
  float2 a = float2(l - min(l, (p.z < 0.0f) ? r : 0.0f), abs(p.z) - h);
  float t_raw = dot(q, k) / m;
  float2 b = k * clamp(t_raw, 0.0f, 1.0f) - q;
  float s = (b.x < 0.0f && a.y < 0.0f) ? -1.0f : 1.0f;
  float la = dot(a, a);
  float lb = dot(b, b);
  float dist = s * sqrt(min(la, lb));

  float2 radial = p.xy / max(l, 1e-8f);

  /* Gradient = normalize(query - closest_surface_point).
   * Using the actual closest point (not a/b tricks) ensures both cap and side
   * branches compute the same gradient at the junction — no discontinuity. */
  float2 closest;
  if (la < lb) {
    float r_cap = (p.z < 0.0f) ? r : 0.0f;
    closest = float2(min(l, r_cap), sign(p.z) * h);
  }
  else {
    float tc = clamp(t_raw, 0.0f, 1.0f);
    closest = float2(tc * r, h - 2.0f * tc * h);
  }
  float2 delta = float2(l, p.z) - closest;
  float dl2 = length(delta);
  float3 grad;
  if (dl2 > 1e-6f) {
    grad = s * float3(radial * delta.x, delta.y) / dl2;
  }
  else {
    grad = s * float3(radial * k.y, -k.x) / sqrt(m);
  }

  return float4(dist, grad);
}

float4 sdgCapsule(float3 p, float3 size)
{
  /* Z-axis capsule via IQ sdgSegment: a=(0,0,-h), b=(0,0,h), radius=r. */
  float h = size.y;
  float r = size.x;
  float3 ba = float3(0.0f, 0.0f, 2.0f * h);
  float3 pa = float3(p.x, p.y, p.z + h);
  float t = clamp(dot(pa, ba) / dot(ba, ba), 0.0f, 1.0f);
  float3 q = pa - t * ba;
  float d = length(q);
  return float4(d - r, q / max(d, 1e-8f));
}

float4 sdgTorus(float3 p, float2 t)
{
  float h = length(p.xy);
  float3 raw = p * float3(h - t.x, h - t.x, h);
  float rl = length(raw);
  float3 grad = (rl > 1e-8f) ? raw / rl : float3(0.0f, 0.0f, sign(p.z));
  float2 q = float2(h - t.x, p.z);
  float d = length(q) - t.y;
  return float4(d, grad);
}

float4 sdgCappedTorus(float3 p, float2 sc, float ra, float rb)
{
  float3 ap = float3(abs(p.x), p.y, p.z);
  float k = (sc.y * ap.x > sc.x * ap.y) ? dot(ap.xy, sc) : length(ap.xy);
  float d = sqrt(dot(p, p) + ra * ra - 2.0f * ra * k) - rb;

  float3 grad;
  if (sc.y * ap.x > sc.x * ap.y) {
    grad = float3(p.x - sign(p.x) * ra * sc.x,
                  p.y - ra * sc.y,
                  p.z);
  }
  else {
    float lxy = max(length(p.xy), 1e-8f);
    grad = float3(p.x * (1.0f - ra / lxy),
                  p.y * (1.0f - ra / lxy),
                  p.z);
  }
  return float4(d, grad / max(length(grad), 1e-8f));
}

/* ---- 2D extrusion gradient (for NGON / POLYGON) ---- */

float4 sdgExtrude2D(float d2d, float3 grad2d_and_z, float pz, float halfH)
{
  float dz = abs(pz) - halfH;
  float2 dd = float2(d2d, dz);
  float2 hh = max(dd, float2(0.0f));
  float fl = length(hh);
  float fg = max(dd.x, dd.y);
  float3 gxy = grad2d_and_z;
  float3 gz = float3(0.0f, 0.0f, sign(pz));

  if (fg <= 0.0f) {
    return float4(fg, (dd.x > dd.y) ? gxy : gz);
  }
  return float4(fl, (hh.x * gxy + hh.y * gz) / max(fl, 1e-8f));
}

/* ---- CSG blend operation gradients ---- */

/* Smooth union with gradient. */
float4 opSmoothUnionGrad(float4 dg1, float4 dg2, float k)
{
  if (k <= 0.0001f) {
    return (dg1.x < dg2.x) ? dg1 : dg2;
  }
  float h = clamp(0.5f + 0.5f * (dg2.x - dg1.x) / k, 0.0f, 1.0f);
  float d = mix(dg2.x, dg1.x, h) - k * h * (1.0f - h);
  float3 g = mix(dg2.yzw, dg1.yzw, h);
  return float4(d, g);
}

/* Smooth subtraction with gradient (d1 subtracted FROM d2). */
float4 opSmoothSubtractionGrad(float4 dg1, float4 dg2, float k)
{
  if (k <= 0.0001f) {
    return (-dg1.x > dg2.x) ? float4(-dg1.x, -dg1.yzw) : dg2;
  }
  float h = clamp(0.5f - 0.5f * (dg2.x + dg1.x) / k, 0.0f, 1.0f);
  float d = mix(dg2.x, -dg1.x, h) + k * h * (1.0f - h);
  float3 g = mix(dg2.yzw, -dg1.yzw, h);
  return float4(d, g);
}

/* Smooth intersection with gradient. */
float4 opSmoothIntersectionGrad(float4 dg1, float4 dg2, float k)
{
  if (k <= 0.0001f) {
    return (dg1.x > dg2.x) ? dg1 : dg2;
  }
  float h = clamp(0.5f - 0.5f * (dg2.x - dg1.x) / k, 0.0f, 1.0f);
  float d = mix(dg2.x, dg1.x, h) + k * h * (1.0f - h);
  float3 g = mix(dg2.yzw, dg1.yzw, h);
  return float4(d, g);
}

float4 negGrad(float4 a) { return float4(-a.x, -a.yzw); }

/* Sharp min/max with gradient — hard selection, no smoothing. */
float4 opUnionGrad(float4 a, float4 b) { return (a.x < b.x) ? a : b; }
float4 opIntersectionGrad(float4 a, float4 b) { return (a.x > b.x) ? a : b; }
float4 opSubtractionGrad(float4 a, float4 b) { return (-b.x > a.x) ? negGrad(b) : a; }

/* Chamfer union with gradient. */
float4 opChamferUnionGrad(float4 a, float4 b, float r)
{
  float chamfer = (a.x - r + b.x) * 0.5f;
  float3 cg = (a.yzw + b.yzw) * 0.5f;
  float4 cp = float4(chamfer, cg);
  return opUnionGrad(opUnionGrad(a, b), cp);
}

/* Chamfer intersection with gradient. */
float4 opChamferIntersectionGrad(float4 a, float4 b, float r)
{
  float chamfer = (a.x + r + b.x) * 0.5f;
  float3 cg = (a.yzw + b.yzw) * 0.5f;
  float4 cp = float4(chamfer, cg);
  return opIntersectionGrad(opIntersectionGrad(a, b), cp);
}

float4 opChamferSubtractionGrad(float4 a, float4 b, float r)
{
  return opChamferIntersectionGrad(b, negGrad(a), r);
}

/* Round union with gradient. */
float4 opRoundUnionGrad(float4 dg1, float4 dg2, float r)
{
  float a = dg1.x, b = dg2.x;
  float3 ga = dg1.yzw, gb = dg2.yzw;
  float2 s = float2(max(a, 0.0f), max(b, 0.0f));
  float ls = length(s);
  float corn = ls - r;
  float3 gc = float3(0.0f);
  if (ls > 1e-8f) {
    gc = (s.x * ga + s.y * gb) / ls;
  }

  float2 q = float2(a, b);
  float2 N = normalize(float2(-1.0f, 1.0f));
  float proj = min(dot(q, N), 0.0f);
  q -= 2.0f * N * proj;
  q.y -= r;
  q.y = min(0.0f, q.y);
  float ad = sign(q.x) * length(q);
  float3 gad;
  if (abs(q.x) < 1e-8f && abs(q.y) < 1e-8f) {
    gad = ga;
  }
  else {
    float lq = max(length(q), 1e-8f);
    float dqdx, dqdy;
    if (proj < 0.0f) {
      dqdx = 0.5f;
      dqdy = 0.5f;
    }
    else {
      dqdx = 1.0f;
      dqdy = 0.0f;
    }
    float qy_clamped = (q.y < 0.0f) ? 1.0f : 0.0f;
    gad = (sign(q.x) / lq) * (q.x * (dqdx * ga + dqdy * gb) +
                               q.y * qy_clamped * (dqdy * ga + dqdx * gb));
  }

  return (ad < corn) ? float4(ad, gad) : float4(corn, gc);
}

float4 opRoundSubtractionGrad(float4 dg1, float4 dg2, float r)
{
  float4 result = opRoundUnionGrad(dg1, float4(-dg2.x, -dg2.yzw), r);
  return float4(-result.x, -result.yzw);
}

float4 opRoundIntersectionGrad(float4 dg1, float4 dg2, float r)
{
  float4 result = opRoundUnionGrad(float4(-dg1.x, -dg1.yzw),
                                   float4(-dg2.x, -dg2.yzw), r);
  return float4(-result.x, -result.yzw);
}

/* Smooth chamfer with gradient. */
float4 opSmoothChamferUnionGrad(float4 dg1, float4 dg2, float k, float k2, float k3)
{
  float cp = (dg1.x + dg2.x - k) * 0.5f;
  float3 gc = (dg1.yzw + dg2.yzw) * 0.5f;
  float4 dgcp = float4(cp, gc);
  float4 t1 = opSmoothUnionGrad(dg1, dgcp, k2);
  float4 t2 = opSmoothUnionGrad(dg2, dgcp, k3);
  return opUnionGrad(t1, t2);
}

float4 opSmoothChamferSubtractionGrad(float4 dg1, float4 dg2, float k, float k2, float k3)
{
  float4 A = float4(-dg1.x, -dg1.yzw);
  float4 B = dg2;
  float cp = (A.x + B.x + k) * 0.5f;
  float3 gc = (A.yzw + B.yzw) * 0.5f;
  float4 dgcp = float4(cp, gc);
  float4 t1 = opSmoothIntersectionGrad(A, dgcp, k2);
  float4 t2 = opSmoothIntersectionGrad(B, dgcp, k3);
  return opIntersectionGrad(t1, t2);
}

float4 opSmoothChamferIntersectionGrad(float4 dg1, float4 dg2, float k, float k2, float k3)
{
  float cp = (dg1.x + dg2.x + k) * 0.5f;
  float3 gc = (dg1.yzw + dg2.yzw) * 0.5f;
  float4 dgcp = float4(cp, gc);
  float4 t1 = opSmoothIntersectionGrad(dg1, dgcp, k2);
  float4 t2 = opSmoothIntersectionGrad(dg2, dgcp, k3);
  return opIntersectionGrad(t1, t2);
}

/* Smooth round with gradient. */
float4 opSmoothRoundUnionGrad(float4 dg1, float4 dg2, float r, float k2, float k3)
{
  float a = dg1.x, b = dg2.x;
  float2 s = float2(max(a, 0.0f), max(b, 0.0f));
  float ls = max(length(s), 1e-8f);
  float corn = ls - r;
  float3 gc = (s.x * dg1.yzw + s.y * dg2.yzw) / ls;
  float4 dgcorn = float4(corn, gc);
  float4 t1 = opSmoothUnionGrad(dg1, dgcorn, k2);
  float4 t2 = opSmoothUnionGrad(dg2, dgcorn, k3);
  return opUnionGrad(t1, t2);
}

float4 opSmoothRoundSubtractionGrad(float4 dg1, float4 dg2, float r, float k2, float k3)
{
  float a = dg2.x, b = dg1.x;
  float2 s = float2(min(a, 0.0f), max(b, 0.0f));
  float ls = max(length(s), 1e-8f);
  float corn = r - ls;
  float3 gc = -(s.x * dg2.yzw + s.y * dg1.yzw) / ls;
  float4 dgcorn = float4(corn, gc);
  float4 t1 = opSmoothIntersectionGrad(dg2, dgcorn, k2);
  float4 t2 = opSmoothIntersectionGrad(float4(-dg1.x, -dg1.yzw), dgcorn, k3);
  return opIntersectionGrad(t1, t2);
}

float4 opSmoothRoundIntersectionGrad(float4 dg1, float4 dg2, float r, float k2, float k3)
{
  float2 s = float2(min(dg1.x, 0.0f), max(-dg2.x, 0.0f));
  float ls = max(length(s), 1e-8f);
  float corn = r - ls;
  float3 gc = -(s.x * dg1.yzw - s.y * dg2.yzw) / ls;
  float4 dgcorn = float4(corn, gc);
  float4 t1 = opSmoothIntersectionGrad(dg1, dgcorn, k2);
  float4 t2 = opSmoothIntersectionGrad(dg2, dgcorn, k3);
  return opIntersectionGrad(t1, t2);
}

/* ---- Master CSG gradient combiner ---- */

float4 combineCSGGrad(float4 dg1, float4 dg2, int op, int bt, float k,
                      float shell_dist, int shell_mode, int shell_op,
                      float shell_k_top, float shell_k_bot,
                      float k2, float k3)
{
  bool has_smooth = (k2 > 0.0f || k3 > 0.0f);

  if (op == SDF_CSG_OP_UNION) {
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        return opSmoothUnionGrad(dg1, dg2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        if (has_smooth) { return opSmoothChamferUnionGrad(dg1, dg2, k, k2, k3); }
        return opChamferUnionGrad(dg1, dg2, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        if (has_smooth) { return opSmoothRoundUnionGrad(dg1, dg2, k, k2, k3); }
        return opRoundUnionGrad(dg1, dg2, k);
      }
    }
    return opUnionGrad(dg1, dg2);
  }
  else if (op == SDF_CSG_OP_SUBTRACT) {
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        return opSmoothSubtractionGrad(dg2, dg1, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        if (has_smooth) { return opSmoothChamferSubtractionGrad(dg2, dg1, k, k2, k3); }
        return opChamferSubtractionGrad(dg2, dg1, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        if (has_smooth) { return opSmoothRoundSubtractionGrad(dg2, dg1, k, k2, k3); }
        return opRoundSubtractionGrad(dg2, dg1, k);
      }
    }
    return opSubtractionGrad(dg1, dg2);
  }
  else if (op == SDF_CSG_OP_INTERSECT) {
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        return opSmoothIntersectionGrad(dg1, dg2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        if (has_smooth) { return opSmoothChamferIntersectionGrad(dg1, dg2, k, k2, k3); }
        return opChamferIntersectionGrad(dg1, dg2, k);
      }
      else if (bt == SDF_BLEND_TYPE_ROUND) {
        if (has_smooth) { return opSmoothRoundIntersectionGrad(dg1, dg2, k, k2, k3); }
        return opRoundIntersectionGrad(dg1, dg2, k);
      }
    }
    return opIntersectionGrad(dg1, dg2);
  }
  else if (op == SDF_CSG_OP_PUSH) {
    float4 sub;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        sub = opSmoothSubtractionGrad(dg2, dg1, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        if (has_smooth) { sub = opSmoothChamferSubtractionGrad(dg2, dg1, k, k2, k3); }
        else { sub = opChamferSubtractionGrad(dg2, dg1, k); }
      }
      else {
        if (has_smooth) { sub = opSmoothRoundSubtractionGrad(dg2, dg1, k, k2, k3); }
        else { sub = opRoundSubtractionGrad(dg2, dg1, k); }
      }
    }
    else {
      sub = opSubtractionGrad(dg1, dg2);
    }
    return opUnionGrad(sub, dg2);
  }
  else if (op == SDF_CSG_OP_AVOID) {
    float4 carved;
    if (k > 0.0f && bt > 0) {
      if (bt == SDF_BLEND_TYPE_SMOOTH) {
        carved = opSmoothSubtractionGrad(dg1, dg2, k);
      }
      else if (bt == SDF_BLEND_TYPE_CHAMFER) {
        if (has_smooth) { carved = opSmoothChamferSubtractionGrad(dg1, dg2, k, k2, k3); }
        else { carved = opChamferSubtractionGrad(dg1, dg2, k); }
      }
      else {
        if (has_smooth) { carved = opSmoothRoundSubtractionGrad(dg1, dg2, k, k2, k3); }
        else { carved = opRoundSubtractionGrad(dg1, dg2, k); }
      }
    }
    else {
      float4 neg1 = float4(-dg1.x, -dg1.yzw);
      carved = opIntersectionGrad(dg2, neg1);
    }
    return opUnionGrad(dg1, carved);
  }
  else if (op == SDF_CSG_OP_SHELL) {
    float sd = (shell_op == SDF_SHELL_OP_SUBTRACTION) ? -shell_dist : shell_dist;
    float h = abs(sd);

    if (shell_mode == SDF_SHELL_MODE_PUSH) {
      /* abs() gradient: use safe sign to avoid flip instability at d2≈0 */
      float s_abs = abs(dg2.x);
      float safe_s = dg2.x / max(abs(dg2.x), 0.001f);
      float3 s_grad = safe_s * dg2.yzw;
      float4 dg_shell = float4(s_abs - h, s_grad);
      float4 sub;
      if (k > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { sub = opSmoothSubtractionGrad(dg_shell, dg1, k); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { sub = opChamferSubtractionGrad(dg_shell, dg1, k); }
        else { sub = opRoundSubtractionGrad(dg_shell, dg1, k); }
      }
      else {
        sub = opSubtractionGrad(dg1, dg_shell);
      }
      return opUnionGrad(sub, dg_shell);
    }

    float lk_top = min(shell_k_top, h);
    float lk_bot = min(shell_k_bot, h);
    float4 dg_shell;

    if (sd < 0.0f) {
      float4 d_sub;
      if (shell_k_bot > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { d_sub = opSmoothSubtractionGrad(dg2, dg1, shell_k_bot); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { d_sub = opChamferSubtractionGrad(dg2, dg1, shell_k_bot); }
        else if (bt == SDF_BLEND_TYPE_ROUND) { d_sub = opRoundSubtractionGrad(dg2, dg1, shell_k_bot); }
        else { d_sub = opSubtractionGrad(dg1, dg2); }
      }
      else { d_sub = opSubtractionGrad(dg1, dg2); }
      float4 dg_lim = float4(dg1.x + h, dg1.yzw);
      if (lk_top > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { dg_shell = opSmoothUnionGrad(d_sub, dg_lim, lk_top); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { dg_shell = opChamferUnionGrad(d_sub, dg_lim, lk_top); }
        else if (bt == SDF_BLEND_TYPE_ROUND) { dg_shell = opRoundUnionGrad(d_sub, dg_lim, lk_top); }
        else { dg_shell = opUnionGrad(d_sub, dg_lim); }
      }
      else { dg_shell = opUnionGrad(d_sub, dg_lim); }
    }
    else {
      float4 d_union;
      if (shell_k_top > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { d_union = opSmoothUnionGrad(dg1, dg2, shell_k_top); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { d_union = opChamferUnionGrad(dg1, dg2, shell_k_top); }
        else if (bt == SDF_BLEND_TYPE_ROUND) { d_union = opRoundUnionGrad(dg1, dg2, shell_k_top); }
        else { d_union = opUnionGrad(dg1, dg2); }
      }
      else { d_union = opUnionGrad(dg1, dg2); }
      float4 dg_lim = float4(dg1.x - h, dg1.yzw);
      if (lk_bot > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { dg_shell = opSmoothIntersectionGrad(d_union, dg_lim, lk_bot); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { dg_shell = opChamferIntersectionGrad(d_union, dg_lim, lk_bot); }
        else if (bt == SDF_BLEND_TYPE_ROUND) { dg_shell = opRoundIntersectionGrad(d_union, dg_lim, lk_bot); }
        else { dg_shell = opIntersectionGrad(d_union, dg_lim); }
      }
      else { dg_shell = opIntersectionGrad(d_union, dg_lim); }
    }

    if (shell_mode == SDF_SHELL_MODE_AVOID) {
      float4 carved;
      if (k > 0.0f && bt > 0) {
        if (bt == SDF_BLEND_TYPE_SMOOTH) { carved = opSmoothSubtractionGrad(dg1, dg_shell, k); }
        else if (bt == SDF_BLEND_TYPE_CHAMFER) { carved = opChamferSubtractionGrad(dg1, dg_shell, k); }
        else { carved = opRoundSubtractionGrad(dg1, dg_shell, k); }
      }
      else {
        float4 neg1 = float4(-dg1.x, -dg1.yzw);
        carved = opIntersectionGrad(dg_shell, neg1);
      }
      return opUnionGrad(dg1, carved);
    }
    return dg_shell;
  }

  return dg1;
}

/* ---- Master primitive gradient evaluator ---- */

float4 evalPrimitiveGrad(float3 local_pos, SDFObjectGPU obj, float ray_epsilon)
{
  float actual_dist = evalPrimitiveOnly(obj, local_pos);

  float3 size = obj.sdf_size.xyz;
  float min_dim = min(size.x, min(size.y, size.z));
  float grad_bevel = max(obj.bevel, min(0.005f, min_dim * 0.5f));
  float4 dg;

  if (obj.sdf_type == 1) {
    float3 r = max(size - float3(grad_bevel), float3(0.001f));
    if (abs(r.x - r.y) < 0.0001f && abs(r.x - r.z) < 0.0001f) {
      dg = sdgSphere(local_pos, r.x);
    }
    else {
      dg = sdgEllipsoid(local_pos, r);
    }
  }
  else if (obj.sdf_type == 2) {
    float3 cyl_size = max(size - float3(grad_bevel), float3(0.001f));
    dg = sdgCylinder(local_pos, cyl_size);
  }
  else if (obj.sdf_type == 3) {
    float cone_r = max(size.x - grad_bevel, 0.001f);
    float cone_h = max(size.y - grad_bevel, 0.001f);
    dg = sdgCone(local_pos, cone_r, cone_h);
  }
  else if (obj.sdf_type == 4) {
    float3 cap_size = max(size - float3(grad_bevel), float3(0.001f));
    dg = sdgCapsule(local_pos, cap_size);
  }
  else if (obj.sdf_type == 5) {
    float major = max(size.x - grad_bevel, 0.001f);
    float minor = max(size.y - grad_bevel, 0.001f);
    if (obj.box_modes.w != 0) {
      dg = sdgCappedTorus(local_pos, obj.box_corners.xy, major, minor);
    }
    else {
      dg = sdgTorus(local_pos, float2(major, minor));
    }
  }
  else if (obj.sdf_type == 6 || obj.sdf_type == 7) {
    float eps = 0.0005f;
    float dx = evalPrimitiveOnly(obj, local_pos + float3(eps, 0.0f, 0.0f));
    float dy = evalPrimitiveOnly(obj, local_pos + float3(0.0f, eps, 0.0f));
    float dz = evalPrimitiveOnly(obj, local_pos + float3(0.0f, 0.0f, eps));
    float3 g = float3(dx - actual_dist, dy - actual_dist, dz - actual_dist) / eps;
    float gl = max(length(g), 1e-8f);
    return float4(actual_dist, g / gl);
  }
  else {
    bool hasAdvanced = (obj.box_corners.x + obj.box_corners.y + obj.box_corners.z +
                        obj.box_corners.w + obj.box_edges.x + obj.box_edges.y +
                        obj.box_edges.z + obj.box_edges.w) > 0.001f;
    if (hasAdvanced) {
      float eps = 0.0005f;
      float dx = evalPrimitiveOnly(obj, local_pos + float3(eps, 0.0f, 0.0f));
      float dy = evalPrimitiveOnly(obj, local_pos + float3(0.0f, eps, 0.0f));
      float dz = evalPrimitiveOnly(obj, local_pos + float3(0.0f, 0.0f, eps));
      float3 g = float3(dx - actual_dist, dy - actual_dist, dz - actual_dist) / eps;
      float gl = max(length(g), 1e-8f);
      return float4(actual_dist, g / gl);
    }
    float3 box_size = max(size - float3(grad_bevel), float3(0.001f));
    dg = sdgBox(local_pos, box_size);
  }

  /* Return exact distance with beveled gradient direction. */
  return float4(actual_dist, dg.yzw);
}

/* ---- Distance modifier gradient ---- */

float4 applyDistanceModifiersGrad(float4 dg, int mod_start, int mod_count)
{
  for (int i = mod_start; i < mod_start + mod_count; i++) {
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;

    if (mtype == SDF_MOD_HOLLOW || mtype == SDF_MOD_ONION) {
      float safe_s = dg.x / max(abs(dg.x), 0.001f);
      dg = float4(abs(dg.x) - smod.params.x, safe_s * dg.yzw);
    }
    else if (mtype == SDF_MOD_ROUND) {
      dg.x -= smod.params.x;
    }
  }
  return dg;
}

/* ---- Domain modifier inverse Jacobian on gradient ---- */

/* Walk the modifier stack and apply the inverse Jacobian of each domain
 * modifier to the gradient. Must be called with the ORIGINAL (pre-modifier)
 * local-space position so we know which mirrors were triggered. */
float3 invertDomainModifiersGrad(float3 grad, float3 orig_p,
                                 int mod_start, int mod_count,
                                 float4x4 inv_mat)
{
  /* Pass 1: walk forward (high-to-low, same as applyDomainModifiers)
   * to record the position at each modifier's input. */
  float3 positions[8];
  int n = min(mod_count, 8);
  {
    float3 p = orig_p;
    for (int i = mod_start + mod_count - 1; i >= mod_start; i--) {
      int slot = i - mod_start;
      if (slot < 8) { positions[slot] = p; }
      SDFModifierGPU smod = sdf_modifiers[i];
      int mtype = smod.header.x;
      int mflags = smod.header.y;
      if (mtype == SDF_MOD_MIRROR) {
        float offset = smod.params.x;
        float3 origin = smod.params.yzw;
        float blend = smod.params2.x;
        int blend_type = smod.header.z;
        float k = (blend_type > 0) ? blend : 0.0f;
        if ((mflags & SDF_MOD_MIRROR_X) != 0) {
          float3 raw_N = float3(inv_mat[0]);
          float nl = length(raw_N);
          float3 N = (nl > 1e-6f) ? raw_N / nl : float3(1, 0, 0);
          float d = dot(p - origin, N);
          p -= (d - sabs(d, k)) * N;
          p -= offset * abs(N);
        }
        if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
          float3 raw_N = float3(inv_mat[1]);
          float nl = length(raw_N);
          float3 N = (nl > 1e-6f) ? raw_N / nl : float3(0, 1, 0);
          float d = dot(p - origin, N);
          p -= (d - sabs(d, k)) * N;
          p -= offset * abs(N);
        }
        if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
          float3 raw_N = float3(inv_mat[2]);
          float nl = length(raw_N);
          float3 N = (nl > 1e-6f) ? raw_N / nl : float3(0, 0, 1);
          float d = dot(p - origin, N);
          p -= (d - sabs(d, k)) * N;
          p -= offset * abs(N);
        }
      }
      else if (mtype == SDF_MOD_TWIST) {
        float tk = smod.params.x;
        int axis = int(smod.params.y);
        float drive;
        if (axis == 1) { drive = p.y; }
        else if (axis == 2) { drive = p.x; }
        else { drive = p.z; }
        float angle = tk * drive;
        float c = cos(angle), s = sin(angle);
        if (axis == 1) { p = float3(c*p.x - s*p.z, p.y, s*p.x + c*p.z); }
        else if (axis == 2) { p = float3(p.x, c*p.y - s*p.z, s*p.y + c*p.z); }
        else { p = float3(c*p.x - s*p.y, s*p.x + c*p.y, p.z); }
      }
      else if (mtype == SDF_MOD_ARRAY) {
        float cnt = smod.params.x;
        float arr_blend = smod.params2.x;
        int arr_bt = smod.header.z;
        float abk = (arr_bt > 0 && arr_blend > 0.001f) ? arr_blend : 0.0f;
        if (mflags == SDF_MOD_ARRAY_LINEAR) {
          float3 aoff = smod.params.yzw;
          float sp = length(aoff);
          if (sp > 0.0001f && cnt > 0.5f) {
            float3 adir = aoff / sp;
            float nt = dot(p, adir) / sp;
            float aid = round(nt);
            aid = clamp(aid, 0.0f, max(0.0f, cnt - 1.0f));
            float lt = nt - aid;
            if (abk > 0.001f) {
              float kh = abk / (sp * 2.0f);
              float alt = abs(lt);
              float db = 0.5f - alt;
              float sd = sabs(db, kh);
              lt = sign(lt + 1e-8f) * (0.5f - sd);
            }
            p = p - adir * (aid + lt) * sp;
          }
        }
        else if (mflags == SDF_MOD_ARRAY_RADIAL) {
          float arad = smod.params.y;
          if (cnt > 0.5f) {
            float aa = (2.0f * SDF_PI) / cnt;
            float na = atan(p.y, p.x) / aa;
            float aid = round(na);
            float la = na - aid;
            if (abk > 0.001f) {
              float kh = abk / (aa * 2.0f);
              float ala = abs(la);
              float db = 0.5f - ala;
              float sd = sabs(db, kh);
              la = sign(la + 1e-8f) * (0.5f - sd);
            }
            float fa = la * aa;
            float r = length(p.xy);
            p.x = r * cos(fa) - arad;
            p.y = r * sin(fa);
          }
        }
      }
      else if (mtype == SDF_MOD_BEND) {
        float bk = smod.params.x;
        int axis = int(smod.params.y);
        float3 origin = float3(smod.params.z, smod.params.w, smod.params2.x);
        p -= origin;
        if (abs(bk) > 0.0001f) {
          float drive, curve;
          if (axis == 1) {
            drive = p.y; curve = p.z;
            float a = bk * drive; float c = cos(a); float s = sin(a);
            p.y = c * drive - s * curve;
            p.z = s * drive + c * curve;
          }
          else if (axis == 2) {
            drive = p.z; curve = p.x;
            float a = bk * drive; float c = cos(a); float s = sin(a);
            p.z = c * drive - s * curve;
            p.x = s * drive + c * curve;
          }
          else {
            drive = p.x; curve = p.y;
            float a = bk * drive; float c = cos(a); float s = sin(a);
            p.x = c * drive - s * curve;
            p.y = s * drive + c * curve;
          }
        }
        p += origin;
      }
      else if (mtype == SDF_MOD_ELONGATE) {
        float3 h = smod.params.xyz;
        p = p - clamp(p, -h, h);
      }
    }
  }

  /* Pass 2: walk low-to-high applying J^T at each step.
   * Chain rule: ∇_p0 = J_high^T * J_low^T * ∇_out → apply J_low^T first. */
  for (int i = mod_start; i < mod_start + mod_count; i++) {
    int slot = i - mod_start;
    float3 pp = (slot < 8) ? positions[slot] : orig_p;
    SDFModifierGPU smod = sdf_modifiers[i];
    int mtype = smod.header.x;
    int mflags = smod.header.y;

    if (mtype == SDF_MOD_MIRROR) {
      float3 origin = smod.params.yzw;
      float blend = smod.params2.x;
      int blend_type = smod.header.z;
      float k = (blend_type > 0) ? blend : 0.0f;
      if ((mflags & SDF_MOD_MIRROR_X) != 0) {
        float3 raw_N = float3(inv_mat[0]);
        float nl = length(raw_N);
        float3 N = (nl > 1e-6f) ? raw_N / nl : float3(1, 0, 0);
        float d = dot(pp - origin, N);
        float h = (k > 0.0001f) ? clamp(0.5f + 0.5f * d / k, 0.0f, 1.0f) : (d < 0.0f ? 0.0f : 1.0f);
        grad = grad - 2.0f * (1.0f - h) * dot(grad, N) * N;
      }
      if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
        float3 raw_N = float3(inv_mat[1]);
        float nl = length(raw_N);
        float3 N = (nl > 1e-6f) ? raw_N / nl : float3(0, 1, 0);
        float d = dot(pp - origin, N);
        float h = (k > 0.0001f) ? clamp(0.5f + 0.5f * d / k, 0.0f, 1.0f) : (d < 0.0f ? 0.0f : 1.0f);
        grad = grad - 2.0f * (1.0f - h) * dot(grad, N) * N;
      }
      if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
        float3 raw_N = float3(inv_mat[2]);
        float nl = length(raw_N);
        float3 N = (nl > 1e-6f) ? raw_N / nl : float3(0, 0, 1);
        float d = dot(pp - origin, N);
        float h = (k > 0.0001f) ? clamp(0.5f + 0.5f * d / k, 0.0f, 1.0f) : (d < 0.0f ? 0.0f : 1.0f);
        grad = grad - 2.0f * (1.0f - h) * dot(grad, N) * N;
      }
    }
    else if (mtype == SDF_MOD_ARRAY) {
      float cnt = smod.params.x;
      float arr_blend = smod.params2.x;
      int arr_bt = smod.header.z;
      float bk = (arr_bt > 0 && arr_blend > 0.001f) ? arr_blend : 0.0f;
      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        /* sabs-boundary Jacobian: J = I - (1 - sabs'(d_bnd, k)) * dir ⊗ dir
         * sabs' ∈ [0,1] → (1 - sabs') ∈ [0,1] → well-behaved. */
        float3 aoff = smod.params.yzw;
        float sp = length(aoff);
        if (sp > 0.0001f && cnt > 0.5f && bk > 0.001f) {
          float3 dir = aoff / sp;
          float nt = dot(pp, dir) / sp;
          float aid = round(nt);
          float lt = nt - aid;
          float kh = bk / (sp * 2.0f);
          float abs_lt = abs(lt);
          float d_bnd = 0.5f - abs_lt;
          float h_sabs = (kh > 0.0001f) ? clamp(0.5f + 0.5f * d_bnd / kh, 0.0f, 1.0f) : 1.0f;
          float sabs_deriv = 2.0f * h_sabs - 1.0f;
          float fold_deriv = sabs_deriv;
          float g_dir = dot(grad, dir);
          grad = grad - (1.0f - fold_deriv) * g_dir * dir;
        }
      }
      else if (mflags == SDF_MOD_ARRAY_RADIAL) {
        if (cnt > 0.5f) {
          float aa = (2.0f * SDF_PI) / cnt;
          float na = atan(pp.y, pp.x) / aa;
          float aid = round(na);
          float la = na - aid;
          if (bk > 0.001f) {
            float kh = bk / (aa * 2.0f);
            float abs_la = abs(la);
            float d_bnd = 0.5f - abs_la;
            float h_sabs = (kh > 0.0001f) ? clamp(0.5f + 0.5f * d_bnd / kh, 0.0f, 1.0f) : 1.0f;
            float sabs_deriv = 2.0f * h_sabs - 1.0f;
            la = sign(la + 1e-8f) * (0.5f - sabs(d_bnd, kh));
          }
          float cell_angle = (aid + la) * aa;
          float ca = cos(cell_angle), sa = sin(cell_angle);
          float gx = grad.x, gy = grad.y;
          grad.x = ca * gx - sa * gy;
          grad.y = sa * gx + ca * gy;
          float3 rot = smod.params2.yzw;
          if (abs(rot.z) > 0.0001f) {
            float cz = cos(-rot.z), sz = sin(-rot.z);
            gx = grad.x; gy = grad.y;
            grad.x = cz * gx - sz * gy;
            grad.y = sz * gx + cz * gy;
          }
          if (abs(rot.y) > 0.0001f) {
            float cy = cos(-rot.y), sy = sin(-rot.y);
            gx = grad.x; float gz = grad.z;
            grad.x = cy * gx + sy * gz;
            grad.z = -sy * gx + cy * gz;
          }
          if (abs(rot.x) > 0.0001f) {
            float cx = cos(-rot.x), sx = sin(-rot.x);
            gy = grad.y; float gz = grad.z;
            grad.y = cx * gy - sx * gz;
            grad.z = sx * gy + cx * gz;
          }
        }
      }
    }
    else if (mtype == SDF_MOD_TWIST) {
      float tk = smod.params.x;
      int axis = int(smod.params.y);
      float drive;
      if (axis == 1) { drive = pp.y; }
      else if (axis == 2) { drive = pp.x; }
      else { drive = pp.z; }
      float fc = cos(tk * drive), fs = sin(tk * drive);
      float pa, pb;
      if (axis == 1) { pa = fc*pp.x - fs*pp.z; pb = fs*pp.x + fc*pp.z; }
      else if (axis == 2) { pa = fc*pp.y - fs*pp.z; pb = fs*pp.y + fc*pp.z; }
      else { pa = fc*pp.x - fs*pp.y; pb = fs*pp.x + fc*pp.y; }
      float ga, gb;
      if (axis == 1) { ga = grad.x; gb = grad.z; }
      else if (axis == 2) { ga = grad.y; gb = grad.z; }
      else { ga = grad.x; gb = grad.y; }
      if (axis == 1) {
        grad.x = fc * ga + fs * gb;
        grad.z = -fs * ga + fc * gb;
      } else if (axis == 2) {
        grad.y = fc * ga + fs * gb;
        grad.z = -fs * ga + fc * gb;
      } else {
        grad.x = fc * ga + fs * gb;
        grad.y = -fs * ga + fc * gb;
      }
      float drive_correction = tk * (-pb * ga + pa * gb);
      if (axis == 1) { grad.y += drive_correction; }
      else if (axis == 2) { grad.x += drive_correction; }
      else { grad.z += drive_correction; }
    }
    else if (mtype == SDF_MOD_BEND) {
      float bk = smod.params.x;
      int axis = int(smod.params.y);
      float3 origin = float3(smod.params.z, smod.params.w, smod.params2.x);
      float3 bp = pp - origin;
      if (abs(bk) > 0.0001f) {
        if (axis == 1) {
          float a = -bk * bp.y;
          float ca = cos(a), sa = sin(a);
          grad = float3(grad.x, ca * grad.y - sa * grad.z, sa * grad.y + ca * grad.z);
        }
        else if (axis == 2) {
          float a = -bk * bp.z;
          float ca = cos(a), sa = sin(a);
          grad = float3(ca * grad.x + sa * grad.z, grad.y, -sa * grad.x + ca * grad.z);
        }
        else {
          float a = -bk * bp.x;
          float ca = cos(a), sa = sin(a);
          grad = float3(ca * grad.x - sa * grad.y, sa * grad.x + ca * grad.y, grad.z);
        }
      }
    }
  }
  return grad;
}
