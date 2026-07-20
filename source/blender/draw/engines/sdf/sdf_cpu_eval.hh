/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector.hh"
#include "sdf_shader_shared.hh"

namespace blender::sdf_cpu {

using blender::float2;
using blender::float3;
using blender::float4;
using blender::float4x4;
using blender::int4;

/* SDF modifier type IDs (must match sdf_lib.glsl). */
#define SDF_MOD_MIRROR 0
#define SDF_MOD_TWIST 1
#define SDF_MOD_BEND 2
#define SDF_MOD_ELONGATE 3
#define SDF_MOD_SOLIDIFY 4
#define SDF_MOD_ROUND 5
#define SDF_MOD_ONION 6
#define SDF_MOD_ARRAY 8
#define SDF_MOD_DISPLACE 9
#define SDF_MOD_MIRROR_X 1
#define SDF_MOD_MIRROR_Y 2
#define SDF_MOD_MIRROR_Z 4
#define SDF_MOD_ARRAY_LINEAR 0
#define SDF_MOD_ARRAY_RADIAL 1

/* Primitives */

inline float sdBox(float3 p, float3 b)
{
  float3 q = math::abs(p) - b;
  return math::length(math::max(q, float3(0.0f))) +
         std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
}

inline float sdSphere(float3 p, float r)
{
  return math::length(p) - r;
}

inline float sdCylinder(float3 p, float3 size)
{
  float2 d = float2(math::length(float2(p.x, p.y)) - size.x, fabsf(p.z) - size.z);
  return std::min(std::max(d.x, d.y), 0.0f) +
         math::length(math::max(float2(d.x, d.y), float2(0.0f)));
}

inline float sdConeFrustum(float3 p, float rb, float rt, float h)
{
  /* Mirror of GLSL sdConeFrustum: bottom radius rb at z=-h, top radius rt at z=+h. */
  float2 q = float2(math::length(float2(p.x, p.y)), p.z);
  float2 k1 = float2(rt, h);
  float2 k2 = float2(rt - rb, 2.0f * h);
  float2 ca = float2(q.x - std::min(q.x, (q.y < 0.0f) ? rb : rt), fabsf(q.y) - h);
  float2 cb = q - k1 + k2 * math::clamp(math::dot(k1 - q, k2) / math::dot(k2, k2), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * std::sqrt(std::min(math::dot(ca, ca), math::dot(cb, cb)));
}

inline float sdCone(float3 p, float r, float h)
{
  return sdConeFrustum(p, r, 0.0f, h);
}

inline float sdCapsule(float3 p, float3 size)
{
  float r = size.x;
  float h = std::max(size.y - r, 0.0f);
  p.z -= math::clamp(p.z, -h, h);
  return math::length(p) - r;
}

inline float sdTorus(float3 p, float2 t)
{
  float2 q = float2(math::length(float2(p.x, p.y)) - t.x, p.z);
  return math::length(q) - t.y;
}

/* Eval primitive (simplified, covers main types).
 * Note: the baked mesh volume fast path (SDF_LP_MESH_FLAG_BAKED, bake_*
 * fields on SDFObjectGPU) is GPU-only — this evaluator always stays on the
 * analytic path and ignores the flag. */

inline float evalPrimitive(const SDFObjectGPU &obj, float3 p)
{
  /* sdf_size.xyz = pre-subtracted BASE (size - bevel), sdf_size.w = effective bevel.
   * obj_scale.xyz applies object scale as a coordinate transform. */
  float3 size(obj.sdf_size);
  float bevel = obj.sdf_size.w;
  p = p / float3(obj.obj_scale);

  float dist;
  switch (obj.sdf_type) {
    case 1: { /* SPHERE */
      dist = sdSphere(p, size.x);
      break;
    }
    case 2: { /* CYLINDER */
      dist = sdCylinder(p, size);
      break;
    }
    case 3: { /* CONE / FRUSTUM */
      dist = sdConeFrustum(p, size.x, size.z, size.y);
      break;
    }
    case 4: { /* CAPSULE */
      dist = sdCapsule(p, size);
      break;
    }
    case 5: { /* TORUS */
      dist = sdTorus(p, float2(size.x, size.y));
      break;
    }
    default: { /* BOX */
      float3 bs = size - float3(bevel);
      bs = math::max(bs, float3(0.001f));
      dist = sdBox(p, bs);
      break;
    }
  }
  return dist - bevel;
}

/* Domain modifiers */

struct DomainResult {
  float3 p;
  float scale;
};

inline DomainResult applyDomainMods(float3 p,
                                    const SDFModifierGPU *mods,
                                    int mod_start,
                                    int mod_count,
                                    const float4x4 &inv_mat,
                                    bool skip_mirror = false)
{
  float scale = 1.0f;
  for (int i = mod_start + mod_count - 1; i >= mod_start; i--) {
    const SDFModifierGPU &m = mods[i];
    int mtype = m.header.x;
    int mflags = m.header.y;

    if (mtype == SDF_MOD_MIRROR && !skip_mirror) {
      float offset = m.params.x;
      float3 origin(m.params.y, m.params.z, m.params.w);
      float blend = m.params2.x;
      float bk = (m.header.z > 0 && blend > 0.001f) ? blend : 0.0f;
      auto sabs = [](float x, float k) -> float {
        if (k <= 0.0001f) { return fabsf(x); }
        float h = std::clamp(0.5f + 0.5f * x / k, 0.0f, 1.0f);
        return x * (2.0f * h - 1.0f) + k * h * (1.0f - h);
      };
      int sides = m.header.w;
      auto do_mirror = [&](int ax, int bit) {
        float3 N(inv_mat[ax][0], inv_mat[ax][1], inv_mat[ax][2]);
        float s = ((sides & bit) != 0) ? 1.0f : -1.0f;
        N *= s;
        float nl2 = std::max(math::dot(N, N), 1e-12f);
        float d = math::dot(p - origin, N) / nl2;
        float ad = sabs(d, bk);
        p -= (d - ad) * N;
        p -= offset * N;
      };
      if (mflags & SDF_MOD_MIRROR_X) { do_mirror(0, 1); }
      if (mflags & SDF_MOD_MIRROR_Y) { do_mirror(1, 2); }
      if (mflags & SDF_MOD_MIRROR_Z) { do_mirror(2, 4); }
    }
    else if (mtype == SDF_MOD_TWIST) {
      float k = m.params.x;
      int axis = int(m.params.y);
      float drive, r;
      if (axis == 1) { drive = p.y; r = math::length(float2(p.x, p.z)); }
      else if (axis == 2) { drive = p.x; r = math::length(float2(p.y, p.z)); }
      else { drive = p.z; r = math::length(float2(p.x, p.y)); }
      float angle = k * drive;
      float c = cosf(angle), s = sinf(angle);
      if (axis == 1) { p = float3(c * p.x - s * p.z, p.y, s * p.x + c * p.z); }
      else if (axis == 2) { p = float3(p.x, c * p.y - s * p.z, s * p.y + c * p.z); }
      else { p = float3(c * p.x - s * p.y, s * p.x + c * p.y, p.z); }
      scale *= 1.0f / std::min(1.0f + fabsf(k) * r, 2.0f);
    }
    else if (mtype == SDF_MOD_BEND) {
      float k = m.params.x;
      int axis = int(m.params.y);
      float3 origin(m.params.z, m.params.w, m.params2.x);
      p -= origin;
      if (fabsf(k) > 0.0001f) {
        float drive, curve, r;
        if (axis == 1) {
          drive = p.y; curve = p.z; r = math::length(float2(p.y, p.z));
          float a = k * drive, c = cosf(a), s = sinf(a);
          p.y = c * drive - s * curve;
          p.z = s * drive + c * curve;
        }
        else if (axis == 2) {
          drive = p.z; curve = p.x; r = math::length(float2(p.z, p.x));
          float a = k * drive, c = cosf(a), s = sinf(a);
          p.z = c * drive - s * curve;
          p.x = s * drive + c * curve;
        }
        else {
          drive = p.x; curve = p.y; r = math::length(float2(p.x, p.y));
          float a = k * drive, c = cosf(a), s = sinf(a);
          p.x = c * drive - s * curve;
          p.y = s * drive + c * curve;
        }
        scale *= 1.0f / (1.0f + fabsf(k) * r);
      }
      p += origin;
    }
    else if (mtype == SDF_MOD_ELONGATE) {
      float3 h(m.params.x, m.params.y, m.params.z);
      p = p - math::clamp(p, -h, h);
    }
    else if (mtype == SDF_MOD_ARRAY) {
      /* Mirror of GLSL SDF_MOD_ARRAY domain fold (always-smooth blend). */
      auto sabs = [](float x, float k) -> float {
        if (k <= 0.0001f) { return fabsf(x); }
        float hh = std::clamp(0.5f + 0.5f * x / k, 0.0f, 1.0f);
        return x * (2.0f * hh - 1.0f) + k * hh * (1.0f - hh);
      };
      auto fractf = [](float x) -> float { return x - std::floor(x); };
      float count = m.params.x;
      float blend = m.params2.x;
      float bk = (blend > 0.001f) ? blend : 0.0f;
      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        float3 offset(m.params.y, m.params.z, m.params.w);
        float spacing = math::length(offset);
        if (spacing > 0.0001f && count > 0.5f) {
          float3 dir = offset / spacing;
          float t = math::dot(p, dir);
          float norm_t = t / spacing;
          float id = std::clamp(std::round(norm_t), 0.0f, count - 1.0f);
          float local = norm_t - id;
          bool mirrored = false;
          if (count > 1.5f && fractf(id * 0.5f) > 0.25f) { local = -local; mirrored = true; }
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
        float radius = m.params.y;
        if (count > 1.5f) {
          float sector = (2.0f * float(M_PI)) / count;
          float angle = std::atan2(p.y, p.x);
          float norm_a = angle / sector;
          float id = std::round(norm_a);
          float local = norm_a - id;
          bool mir = fractf(fabsf(id) * 0.5f) > 0.25f;
          bool odd = fractf(count * 0.5f) > 0.25f;
          bool at_defect = odd && (fabsf(angle) > float(M_PI) - sector * 0.5f);
          if (at_defect) { local = fabsf(local); }
          else if (mir) { local = -local; }
          float arc = sector * std::max(radius, 0.0001f);
          float d_r = (0.5f - local) * arc;
          float pull_r = d_r - sabs(d_r, bk);
          float d_l = (0.5f + local) * arc;
          float pull_l = d_l - sabs(d_l, bk);
          local += (pull_r - pull_l) / arc;
          if (bk > 0.001f) { scale *= 0.5f; }
          float fold_a = local * sector;
          float r = math::length(float2(p.x, p.y));
          p.x = r * cosf(fold_a) - radius;
          p.y = r * sinf(fold_a);
        }
      }
    }
  }
  return {p, scale};
}

/* Distance modifiers */

inline float applyDistMods(float dist,
                           float3 p,
                           const SDFObjectGPU &obj,
                           const SDFModifierGPU *mods,
                           int mod_start,
                           int mod_count,
                           bool skip_shell = false)
{
  for (int i = mod_start; i < mod_start + mod_count; i++) {
    int mtype = mods[i].header.x;
    if ((mtype == SDF_MOD_SOLIDIFY || mtype == SDF_MOD_ONION) && skip_shell) {
      continue;
    }
    if (mtype == SDF_MOD_SOLIDIFY) {
      int mode = mods[i].header.y;
      float thickness = mods[i].params.x;
      float bevel = mods[i].params.z;

      if (mode == 0) {
        float d_inner = -(dist + thickness);
        if (bevel > 0.0001f) {
          float h = math::clamp(0.5f - 0.5f * (d_inner - dist) / bevel, 0.0f, 1.0f);
          dist = math::interpolate(d_inner, dist, h) + bevel * h * (1.0f - h);
        }
        else {
          dist = fmaxf(dist, d_inner);
        }
      }
      else {
        int axis = mods[i].header.z;
        float inner_scale = mods[i].params.y;
        float3 p_inner = p;
        p_inner[axis] *= inner_scale;
        float d_eval = evalPrimitive(obj, p_inner);
        float d_inner = -(d_eval + thickness);
        if (bevel > 0.0001f) {
          float h = math::clamp(0.5f - 0.5f * (d_inner - dist) / bevel, 0.0f, 1.0f);
          dist = math::interpolate(d_inner, dist, h) + bevel * h * (1.0f - h);
        }
        else {
          dist = fmaxf(dist, d_inner);
        }
      }
    }
    else if (mtype == SDF_MOD_ONION) {
      int layers = mods[i].header.y > 0 ? mods[i].header.y : 1;
      float cut_half = fmaxf(mods[i].params.x, 0.001f) * 0.5f;
      float min_ext = mods[i].params.y;
      float original_d = dist;
      if (layers > 1) {
        float spacing = min_ext / float(layers);
        float depth = fmaxf(-dist, 0.0f);
        float max_cut = float(layers - 1) * spacing;
        float nearest = math::clamp(
            floorf(depth / spacing + 0.5f) * spacing, spacing, max_cut);
        float cut_dist = fabsf(depth - nearest);
        float onion_d = cut_half - cut_dist;
        dist = fmaxf(original_d, onion_d);
      }
    }
    else if (mtype == SDF_MOD_ROUND) {
      dist -= mods[i].params.x;
    }
    else if (mtype == SDF_MOD_DISPLACE) {
      dist += mods[i].params.x;
    }
  }
  return dist;
}

/* Full per-object SDF eval in local space */

inline float evalObjectSDF(const SDFObjectGPU &obj,
                           const SDFModifierGPU *mods,
                           float3 local_pos,
                           bool skip_mirror = false,
                           bool skip_shell = false)
{
  DomainResult dm = applyDomainMods(
      local_pos, mods, obj.modifier_start, obj.modifier_count, obj.inverse_matrix, skip_mirror);
  /* evalPrimitive returns a BASE-space distance; convert to world via min(scale). */
  float d = evalPrimitive(obj, dm.p) * obj.obj_scale.w;
  return applyDistMods(d, dm.p, obj, mods, obj.modifier_start, obj.modifier_count, skip_shell);
}

}  // namespace blender::sdf_cpu
