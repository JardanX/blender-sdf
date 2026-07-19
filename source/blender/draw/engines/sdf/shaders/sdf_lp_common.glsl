/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning: shared evaluation library.
 *
 * Port of the reference "Lipschitz Pruning" engine (Barbier et al., EG 2025)
 * to the Blender SDF draw engine. A CSG tree is serialized in post-order into
 * three flat arrays (lp_nodes / lp_prims / lp_binary_ops); evaluation is a
 * stack machine over those arrays. Per-cell "active node" lists record the
 * pruned sub-tree that is equivalent to the full tree inside that cell.
 *
 * Active node packing (uint): bit 31 = negated subtree, bits 30..0 = node index.
 * Binary op packing (uint): bit 0 = sign s (+1 for union, -1 otherwise),
 * bits 2..1 = SDF_LP_OP_*, bits 31..3 = float bits of the blend radius k
 * (low 3 mantissa bits truncated).
 *
 * Expects the including shader to declare (via its CREATE_INFO):
 *   lp_prims[], lp_nodes[], lp_binary_ops[], lp_active_nodes[]
 */

#define SDF_LP_STACK_DEPTH 128

/* ------------------------------------------------------------------ */
/** \name Binary op decoding / evaluation
 * \{ */

float lp_op_blend_factor(uint op)
{
  return uintBitsToFloat(op & ~7u);
}

/* Sign of the min() form: +1 for union, -1 for subtract/intersect. */
float lp_op_sign(uint op)
{
  return -1.0f + 2.0f * float(op & 1u);
}

float lp_kernel(float x, float k)
{
  if (k == 0.0f) {
    return 0.0f;
  }
  float m = max(0.0f, k - x);
  return m * m * 0.25f / k;
}

/* Common form for union/subtract/intersect (optionally smooth). */
float lp_binary_op_eval(uint op, float a, float b)
{
  float s = lp_op_sign(op);
  float k = lp_op_blend_factor(op);
  return s * (min(s * a, s * b) - lp_kernel(abs(a - b), k));
}

/* Smooth-min blend returning (distance, color mix factor). */
float2 lp_smin_blend(float a, float b, float k)
{
  float h = max(k - abs(a - b), 0.0f) / k;
  float m = h * h * 0.5f;
  float s = m * k * 0.5f;
  return (a < b) ? float2(a - s, m) : float2(b - s, 1.0f - m);
}

/** \} */

/* ------------------------------------------------------------------ */
/** \name Primitive evaluation (basic analytic shapes)
 *
 * Mirrors the classic engine (sdf_lib.glsl evalPrimitiveOnly) for the basic
 * shapes: box, sphere, cylinder, cone. Scale is applied as a coordinate
 * transform and the result multiplied by the min axis scale, which keeps the
 * field 1-Lipschitz (conservatively) as required by the pruning bound.
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

float lp_eval_prim(float3 p, SDFLpPrimitive prim)
{
  float3 d4 = p - prim.position.xyz;
  float3 lp = float3(dot(prim.m_row0, float4(d4, 1.0f)),
                     dot(prim.m_row1, float4(d4, 1.0f)),
                     dot(prim.m_row2, float4(d4, 1.0f)));
  lp /= prim.scale.xyz;

  float3 r = prim.size.xyz;
  float dist;
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
  else { /* SDF_GPU_TYPE_BOX and fallback */
    dist = lp_sd_box(lp, r);
  }

  return (dist - prim.size.w) * prim.scale.w;
}

/** \} */

/* ------------------------------------------------------------------ */
/** \name Active node list evaluation (post-order stack machine)
 * \{ */

uint lp_active_node_index(uint n)
{
  return n & ~SDF_LP_SIGN_BIT;
}

float lp_active_node_sign(uint n)
{
  return ((n & SDF_LP_SIGN_BIT) == 0u) ? 1.0f : -1.0f;
}

/* Evaluate `count` nodes of the active list starting at `base`. */
float lp_list_eval(float3 p, int count, int base)
{
  float stack[SDF_LP_STACK_DEPTH];
  int stack_idx = 0;

  for (int i = 0; i < count; i++) {
    uint active_node = lp_active_nodes[base + i];
    SDFLpNode node = lp_nodes[lp_active_node_index(active_node)];

    float d;
    if (node.type == SDF_LP_NODETYPE_BINARY) {
      float left_val = stack[stack_idx - 2];
      float right_val = stack[stack_idx - 1];
      stack_idx -= 2;
      d = lp_binary_op_eval(lp_binary_ops[node.idx_in_type], left_val, right_val);
    }
    else {
      d = lp_eval_prim(p, lp_prims[node.idx_in_type]);
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
  int stack_idx = 0;

  for (int i = 0; i < count; i++) {
    uint active_node = lp_active_nodes[base + i];
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
      float s = lp_op_sign(op);
      float k = lp_op_blend_factor(op);
      float2 v = s * lp_smin_blend(s * left_val, s * right_val, k);
      d = v.x;
      albedo = mix(left_col, right_col, v.y);
      obj_id = (v.y < 0.5f) ? left_id : right_id;
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
