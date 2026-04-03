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
#define SDF_MOD_HOLLOW 4
#define SDF_MOD_ROUND 5
#define SDF_MOD_ONION 6
#define SDF_MOD_MIRROR_X 1
#define SDF_MOD_MIRROR_Y 2
#define SDF_MOD_MIRROR_Z 4

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

inline float sdCone(float3 p, float r, float h)
{
  float2 q = float2(math::length(float2(p.x, p.y)), p.z + h);
  float2 e = float2(-r, 2.0f * h);
  float2 pq = q - float2(r, 0.0f);
  float t = math::clamp(math::dot(pq, e) / math::dot(e, e), 0.0f, 1.0f);
  float2 d1 = pq - e * t;
  float d1l = math::length(d1);
  float s = (e.x * pq.y - e.y * pq.x < 0.0f && q.y < 2.0f * h && q.y > 0.0f) ? -1.0f : 1.0f;
  return std::min(s * d1l, std::max(-q.y, std::max(q.y - 2.0f * h, q.x - r)));
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

/* Eval primitive (simplified, covers main types) */

inline float evalPrimitive(const SDFObjectGPU &obj, float3 p)
{
  float3 size(obj.sdf_size);
  float min_dim = std::min(size.x, std::min(size.y, size.z));
  float bevel = std::max(obj.bevel, std::min(0.005f, min_dim * 0.5f));

  float dist;
  switch (obj.sdf_type) {
    case 1: { /* SPHERE */
      float3 r = size - float3(bevel);
      r = math::max(r, float3(0.001f));
      dist = sdSphere(p, r.x);
      break;
    }
    case 2: { /* CYLINDER */
      float3 cs = size - float3(bevel);
      cs = math::max(cs, float3(0.001f));
      dist = sdCylinder(p, cs);
      break;
    }
    case 3: { /* CONE */
      float cr = std::max(size.x - bevel, 0.001f);
      float ch = std::max(size.y - bevel, 0.001f);
      dist = sdCone(p, cr, ch);
      break;
    }
    case 4: { /* CAPSULE */
      float3 cs = size - float3(bevel);
      cs = math::max(cs, float3(0.001f));
      dist = sdCapsule(p, cs);
      break;
    }
    case 5: { /* TORUS */
      float major = std::max(size.x - bevel, 0.001f);
      float minor = std::max(size.y - bevel, 0.001f);
      dist = sdTorus(p, float2(major, minor));
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
  }
  return {p, scale};
}

/* Distance modifiers */

inline float applyDistMods(float dist, const SDFModifierGPU *mods, int mod_start, int mod_count)
{
  for (int i = mod_start; i < mod_start + mod_count; i++) {
    int mtype = mods[i].header.x;
    if (mtype == SDF_MOD_HOLLOW || mtype == SDF_MOD_ONION) {
      dist = fabsf(dist) - mods[i].params.x;
    }
    else if (mtype == SDF_MOD_ROUND) {
      dist -= mods[i].params.x;
    }
  }
  return dist;
}

/* Full per-object SDF eval in local space */

inline float evalObjectSDF(const SDFObjectGPU &obj,
                           const SDFModifierGPU *mods,
                           float3 local_pos,
                           bool skip_mirror = false)
{
  DomainResult dm = applyDomainMods(
      local_pos, mods, obj.modifier_start, obj.modifier_count, obj.inverse_matrix, skip_mirror);
  float d = evalPrimitive(obj, dm.p);
  return applyDistMods(d, mods, obj.modifier_start, obj.modifier_count);
}

}  // namespace blender::sdf_cpu
