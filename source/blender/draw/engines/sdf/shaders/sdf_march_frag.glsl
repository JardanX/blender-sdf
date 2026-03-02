/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF ray-march fragment shader.
 * Fullscreen pass that ray-marches the 3D SDF atlas and writes color + depth.
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

/** Convert world position to atlas UV [0,1]^3. */
float3 world_to_atlas_uv(float3 world_pos)
{
  return (world_pos - atlas_origin) / atlas_extent;
}

/** Sample SDF distance from the atlas. */
float sample_atlas(float3 world_pos)
{
  float3 uv = world_to_atlas_uv(world_pos);
  if (any(lessThan(uv, float3(0.0f))) || any(greaterThan(uv, float3(1.0f)))) {
    return 1e10f;
  }
  return textureLod(sdf_atlas, uv, 0.0f).r;
}

/** Compute normal via central differences in world space. */
float3 calc_normal(float3 p)
{
  float eps = voxel_size * 0.5f;
  float dx = sample_atlas(p + float3(eps, 0.0f, 0.0f)) -
             sample_atlas(p - float3(eps, 0.0f, 0.0f));
  float dy = sample_atlas(p + float3(0.0f, eps, 0.0f)) -
             sample_atlas(p - float3(0.0f, eps, 0.0f));
  float dz = sample_atlas(p + float3(0.0f, 0.0f, eps)) -
             sample_atlas(p - float3(0.0f, 0.0f, eps));
  return normalize(float3(dx, dy, dz));
}

/** Blend-aware color lookup: mirrors bake shader logic to mix colors. */
float3 find_blended_color(float3 world_pos)
{
  float acc_dist = 1e10f;
  float3 acc_color = float3(0.0f);
  for (int i = 0; i < object_count; i++) {
    SDFObjectGPU obj = objects[i];
    float3 local_pos = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float3 size = obj.sdf_size.xyz - obj.bevel;
    size = max(size, float3(0.001f));
    float dist = sdBox(local_pos, size) - obj.bevel;

    float k = obj.blend;
    if (k > 0.0f && acc_dist < 1e9f) {
      float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
      acc_color = mix(acc_color, obj.color.rgb, h);
      acc_dist = mix(acc_dist, dist, h) - k * h * (1.0f - h);
    }
    else {
      if (dist < acc_dist) {
        acc_color = obj.color.rgb;
        acc_dist = dist;
      }
    }
  }
  return acc_color;
}

void main()
{
  float2 uv = screen_uv;

  /* Reconstruct ray from camera using draw_view matrices. */
  ViewMatrices vm = drw_view();
  float4x4 view_inv = vm.viewinv;
  float4x4 win_inv = vm.wininv;

  /* NDC position at near and far planes. */
  float4 ndc_near = float4(uv * 2.0f - 1.0f, -1.0f, 1.0f);
  float4 ndc_far = float4(uv * 2.0f - 1.0f, 1.0f, 1.0f);

  float4 world_near = view_inv * (win_inv * ndc_near);
  float4 world_far = view_inv * (win_inv * ndc_far);
  world_near.xyz /= world_near.w;
  world_far.xyz /= world_far.w;

  float3 ray_origin = world_near.xyz;
  float3 ray_dir = normalize(world_far.xyz - world_near.xyz);

  /* Clip ray to atlas volume. */
  float3 box_min = atlas_origin;
  float3 box_max = atlas_origin + atlas_extent;

  float3 inv_dir = 1.0f / ray_dir;
  float3 t0 = (box_min - ray_origin) * inv_dir;
  float3 t1 = (box_max - ray_origin) * inv_dir;
  float3 t_lo = min(t0, t1);
  float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  float t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

  /* No intersection with atlas volume. */
  if (t_enter > t_exit || t_exit < 0.0f) {
    discard;
    return;
  }

  t_enter = max(t_enter, 0.0f);

  /* NOTE: We don't read the depth texture here because the framebuffer's depth
   * attachment IS the same texture — reading it would be a texture feedback loop
   * (undefined behavior in OpenGL). A proper approach would copy depth to a
   * separate texture first. For now, we just march to the atlas exit. */
  float max_t = t_exit;

  /* Ray march. */
  float t = t_enter;
  float hit_dist = 1e10f;
  float threshold = voxel_size * 0.5f;

  const int MAX_STEPS = 128;
  for (int step = 0; step < MAX_STEPS; step++) {
    if (t > max_t) {
      break;
    }

    float3 p = ray_origin + ray_dir * t;
    float dist = sample_atlas(p);

    if (dist < threshold) {
      hit_dist = dist;
      break;
    }

    /* Sphere tracing step. Clamp to [min_step, max_step] range.
     * Max clamp is essential: voxels far from any object store 1e10,
     * which would overshoot the entire atlas in one step. */
    t += clamp(dist * 0.9f, voxel_size * 0.25f, voxel_size * 32.0f);
  }

  if (hit_dist >= threshold) {
    discard;
    return;
  }

  float3 hit_pos = ray_origin + ray_dir * t;

  /* Compute normal. */
  float3 normal = calc_normal(hit_pos);

  /* Simple directional lighting. */
  float3 light_dir = normalize(float3(0.5f, 0.7f, 1.0f));
  float ndl = max(dot(normal, light_dir), 0.0f);
  float ambient = 0.15f;
  float lighting = ambient + (1.0f - ambient) * ndl;

  /* Get blended object color. */
  float3 obj_color = find_blended_color(hit_pos);

  out_color = float4(obj_color * lighting, 1.0f);

  /* Write depth: transform hit point to clip space. */
  float4x4 view_mat = vm.viewmat;
  float4x4 win_mat = vm.winmat;
  float4 clip_pos = win_mat * (view_mat * float4(hit_pos, 1.0f));
  gl_FragDepth = (clip_pos.z / clip_pos.w) * 0.5f + 0.5f;
}
