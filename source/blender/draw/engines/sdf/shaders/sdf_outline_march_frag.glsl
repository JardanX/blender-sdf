/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF outline fragment shader — per-object analytical sphere-march.
 *
 * Matches MathOPS approach: each selected object is sphere-marched
 * independently against the camera ray. The closest hit across all
 * selected objects wins. Non-selected objects are skipped entirely.
 *
 * Each selected object gets a unique packed outline ID so edge detection
 * finds boundaries between any pair of objects, producing outlines at
 * intersections between selected objects as well as at silhouette edges.
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_outline_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

/* SDF primitive types (must match eSDFType in DNA_sdf_types.h). */
#define SDF_TYPE_BOX 0
#define SDF_TYPE_SPHERE 1
#define SDF_TYPE_CYLINDER 2
#define SDF_TYPE_CONE 3
#define SDF_TYPE_CAPSULE 4
#define SDF_TYPE_TORUS 5
#define SDF_TYPE_NGON 6

#define MAX_MARCH_STEPS 96

/* ---- Local SDF primitives (not in sdf_lib.glsl) ---- */

float sdSphere(float3 p, float r)
{
  return length(p) - r;
}

float sdCapsule(float3 p, float3 size)
{
  float h = size.y;
  float r = size.x;
  p.z -= clamp(p.z, -h, h);
  return length(p) - r;
}

float sdTorus(float3 p, float2 t)
{
  float2 q = float2(length(p.xy) - t.x, p.z);
  return length(q) - t.y;
}

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

float sdCone(float3 p, float r, float h)
{
  float2 q = float2(length(p.xy), p.z);
  float2 k1 = float2(0.0f, h);
  float2 k2 = float2(-r, 2.0f * h);
  float2 ca = float2(q.x - min(q.x, (q.y < 0.0f) ? r : 0.0f), abs(q.y) - h);
  float2 cb = q - k1 + k2 * clamp(dot(k1 - q, k2) / dot(k2, k2), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * sqrt(min(dot(ca, ca), dot(cb, cb)));
}

/* ---- Evaluate a single SDF primitive with modifiers ---- */

/** Matches evalSDFPrimitive() in sdf_classify_comp.glsl. */
float evalOutlinePrimitive(float3 local_pos, SDFObjectGPU obj)
{
  /* Apply domain modifiers (warp sampling space). */
  if (obj.modifier_count > 0) {
    local_pos = applyDomainModifiers(local_pos, obj.modifier_start, obj.modifier_count);
  }

  float3 size = obj.sdf_size.xyz;
  float bevel = obj.bevel;
  float dist;

  if (obj.sdf_type == SDF_TYPE_SPHERE) {
    dist = sdSphere(local_pos, size.x - bevel);
  }
  else if (obj.sdf_type == SDF_TYPE_CYLINDER) {
    float3 cyl_size = max(size - float3(bevel), float3(0.001f));
    dist = sdCylinder(local_pos, cyl_size);
  }
  else if (obj.sdf_type == SDF_TYPE_CONE) {
    float cone_r = max(size.x - bevel, 0.001f);
    float cone_h = max(size.y - bevel, 0.001f);
    dist = sdCone(local_pos, cone_r, cone_h);
  }
  else if (obj.sdf_type == SDF_TYPE_CAPSULE) {
    float3 cap_size = max(size - float3(bevel), float3(0.001f));
    dist = sdCapsule(local_pos, cap_size);
  }
  else if (obj.sdf_type == SDF_TYPE_TORUS) {
    float major = max(size.x - bevel, 0.001f);
    float minor = max(size.y - bevel, 0.001f);
    if (obj.box_modes.w != 0) {
      dist = sdCappedTorus(local_pos, obj.box_corners.xy, major, minor);
    }
    else {
      dist = sdTorus(local_pos, float2(major, minor));
    }
  }
  else if (obj.sdf_type == SDF_TYPE_NGON) {
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
      dist = sdAdvancedNgon(
          local_pos, R, halfH, sides, corner, edgeTop, edgeBot, tapTop, tapBot, edgeMode, halfH,
          star);
    }
    else {
      float d2d = sdRegularPolygon2D(local_pos.xy, R, sides);
      float dz = abs(local_pos.z) - halfH;
      float2 dd = float2(d2d, dz);
      dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
    }
  }
  else {
    /* SDF_TYPE_BOX (default). */
    float4 corners = obj.box_corners;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    bool hasAdvanced = (corners.x + corners.y + corners.z + corners.w + edgeTop + edgeBot +
                        tapTop + tapBot) > 0.001f;
    if (hasAdvanced) {
      float3 box_size = max(size - float3(bevel), float3(0.001f));
      dist = sdAdvancedBox(local_pos, box_size, corners, edgeTop, edgeBot, tapTop, tapBot,
                           obj.box_modes.x, obj.box_modes.y, box_size.z);
    }
    else {
      float3 box_size = max(size - float3(bevel), float3(0.001f));
      dist = sdBox(local_pos, box_size);
    }
  }

  dist -= bevel;

  /* Apply distance modifiers. */
  if (obj.modifier_count > 0) {
    dist = applyDistanceModifiers(dist, obj.modifier_start, obj.modifier_count);
  }

  return dist;
}

void main()
{
  float2 uv = screen_uv;

  /* Reconstruct camera ray. */
  ViewMatrices vm = drw_view();
  float4x4 view_inv = vm.viewinv;
  float4x4 win_inv = vm.wininv;

  float4 ndc_near = float4(uv * 2.0f - 1.0f, -1.0f, 1.0f);
  float4 ndc_far = float4(uv * 2.0f - 1.0f, 1.0f, 1.0f);

  float4 world_near = view_inv * (win_inv * ndc_near);
  float4 world_far = view_inv * (win_inv * ndc_far);
  world_near.xyz /= world_near.w;
  world_far.xyz /= world_far.w;

  float3 ray_origin = world_near.xyz;
  float3 ray_dir = normalize(world_far.xyz - world_near.xyz);
  float3 inv_dir = 1.0f / ray_dir;

  /* Per-object independent sphere-march (matches MathOPS approach).
   * Only selected objects are marched. Closest hit wins. */
  float best_depth = 1.0f;
  uint best_outline_id = 0u;

  for (int i = 0; i < object_count; i++) {
    /* Skip non-selected objects entirely. */
    uint oid = outline_id_map_buf[i];
    if (oid == 0u) {
      continue;
    }

    /* Ray-AABB intersection test. */
    float3 bmin = sdf_objects[i].bbox_min.xyz;
    float3 bmax = sdf_objects[i].bbox_max.xyz;
    float3 t0 = (bmin - ray_origin) * inv_dir;
    float3 t1 = (bmax - ray_origin) * inv_dir;
    float3 t_lo = min(t0, t1);
    float3 t_hi = max(t0, t1);
    float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    float t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

    if (t_enter > t_exit || t_exit < 0.0f) {
      continue;
    }
    t_enter = max(t_enter, 0.0f);

    SDFObjectGPU obj = sdf_objects[i];

    /* Threshold: sub-voxel precision, capped at 0.2% of object extent. */
    float3 obj_extent = bmax - bmin;
    float thr = 0.002f * max(obj_extent.x, max(obj_extent.y, obj_extent.z));
    thr = clamp(thr, 1e-5f, voxel_size * 0.25f);
    float min_step = thr * 0.5f;

    /* Sphere-march this object's individual SDF. */
    float t = t_enter;
    bool hit = false;
    float3 hit_pos;

    for (int s = 0; s < MAX_MARCH_STEPS; s++) {
      float3 wp = ray_origin + ray_dir * t;
      float3 lp = (obj.inverse_matrix * float4(wp - obj.position.xyz, 1.0f)).xyz;
      float d = evalOutlinePrimitive(lp, obj);

      if (d < thr) {
        hit = true;
        hit_pos = wp;
        break;
      }

      t += max(d, min_step);
      if (t > t_exit) {
        break;
      }
    }

    if (hit) {
      float depth = drw_point_world_to_screen(hit_pos).z;
      if (depth < best_depth) {
        best_depth = depth;
        best_outline_id = oid;
      }
    }
  }

  if (best_outline_id == 0u) {
    discard;
    return;
  }

  out_object_id = best_outline_id;
  gl_FragDepth = best_depth - 1e-5f;
}
