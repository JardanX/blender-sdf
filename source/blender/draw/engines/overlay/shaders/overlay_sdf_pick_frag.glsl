/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF picking fragment shader.
 * Sphere-traces the analytical SDF primitive to produce exact depth for GPU picking.
 * Ray starts from the bounding-cube surface (world_pos) so no iterations are wasted
 * traversing empty space between camera and object.
 * On miss: discard — no false positives outside the SDF silhouette.
 */

#include "infos/overlay_sdf_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(overlay_sdf_pick_base)

#include "draw_model_lib.glsl"
#include "draw_view_lib.glsl"
#include "select_lib.glsl"

/* -------------------------------------------------------------------- */
/** \name SDF Evaluation Functions
 * \{ */

float sd_box(float3 p, float3 b)
{
  float3 q = abs(p) - b;
  return length(max(q, float3(0.0f))) + min(max(q.x, max(q.y, q.z)), 0.0f);
}

float sd_sphere(float3 p, float r)
{
  return length(p) - r;
}

float sd_capsule(float3 p, float3 size)
{
  /* Capsule along Y axis: half-height = size.y, radius = size.x. */
  float h = size.y;
  float r = size.x;
  p.y -= clamp(p.y, -h, h);
  return length(p) - r;
}

float sd_torus(float3 p, float2 t)
{
  /* t.x = major radius, t.y = minor radius. */
  float2 q = float2(length(p.xz) - t.x, p.y);
  return length(q) - t.y;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Dispatch
 * \{ */

float eval_sdf(float3 p)
{
  float d;
  if (sdf_type == 0) {
    /* SDF_TYPE_BOX */
    d = sd_box(p, sdf_size);
  }
  else if (sdf_type == 1) {
    /* SDF_TYPE_SPHERE */
    d = sd_sphere(p, sdf_size.x);
  }
  else if (sdf_type == 4) {
    /* SDF_TYPE_CAPSULE */
    d = sd_capsule(p, sdf_size);
  }
  else {
    /* SDF_TYPE_TORUS (5) */
    d = sd_torus(p, float2(sdf_size.x, sdf_size.y));
  }
  /* Apply bevel (round). */
  d -= sdf_bevel;
  return d;
}

/** \} */

void main()
{
  /* Reconstruct ray direction in world space. */
  float3 cam_pos = drw_view_position();
  bool is_persp = drw_view_is_perspective();
  float3 ray_dir_world;

  if (is_persp) {
    ray_dir_world = normalize(world_pos - cam_pos);
  }
  else {
    ray_dir_world = -drw_view_forward();
  }

  /* Start the ray from the bounding-cube surface (world_pos) instead of the
   * camera. This skips all empty space and puts us right at the SDF volume. */
  float3 ray_ori = drw_point_world_to_object(world_pos);
  float3 ray_dir = normalize(drw_normal_world_to_object(ray_dir_world));

  /* Sphere trace — 24 steps is plenty when starting from the bounding surface. */
  const int MAX_STEPS = 24;
  const float HIT_THRESHOLD = 0.005f;
  float max_t = length(float3(sdf_bound)) * 2.0f;

  /* Start slightly behind the bounding surface to catch SDF surfaces that
   * coincide exactly with the bounding cube face. */
  float t = -0.01f;

  for (int i = 0; i < MAX_STEPS; i++) {
    float3 p = ray_ori + ray_dir * t;
    float d = eval_sdf(p);
    if (d < HIT_THRESHOLD) {
      /* Hit: compute world position and write depth. */
      float3 hit_world = drw_point_object_to_world(p);
      float4 hit_clip = drw_point_world_to_homogenous(hit_world);
      float ndc_depth = hit_clip.z / hit_clip.w;
      gl_FragDepth = ndc_depth * 0.5f + 0.5f;
      select_id_output(select_id);
      return;
    }
    t += d;
    if (t > max_t) {
      break;
    }
  }

  discard;
}
