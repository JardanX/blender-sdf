/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_march_comp)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define kFltMax 1e30f
#define kAabbTreeStackSize 64
#define kTileSize 8
#define kMaxBitfieldBits 1024
#define kBitfieldWords (kMaxBitfieldBits / 32)

shared float tile_heat[kTileSize * kTileSize];

/* Per-invocation BVH filter */
uint g_bvh_bits[kBitfieldWords];
int g_numNearShapes;

bool is_shape_near(int idx)
{
  if (idx >= kMaxBitfieldBits) {
    return true;
  }
  return (g_bvh_bits[idx >> 5] & (1u << (uint(idx) & 31u))) != 0u;
}

float evalPrimitive(float3 local_pos, SDFObjectGPU obj)
{
  SDFPrimitiveData prim_data;
  prim_data.sdf_type = obj.sdf_type;
  prim_data.size = obj.sdf_size.xyz;
  prim_data.bevel = obj.bevel;
  prim_data.box_corners = obj.box_corners;
  prim_data.box_edges = obj.box_edges;
  prim_data.box_modes = obj.box_modes;
  prim_data.modifier_start = obj.modifier_start;
  prim_data.modifier_count = obj.modifier_count;
  prim_data.inverse_matrix = obj.inverse_matrix;

  return evalObjectSDF(prim_data, local_pos);
}

float evalObjectDist(float3 world_pos, int idx)
{
  SDFObjectGPU obj = objects[idx];
  float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
  return evalPrimitive(lp, obj);
}

/* BVH helpers */

float3 find_ortho(float3 v)
{
  if (abs(v.x) >= 0.57735f) {
    return float3(v.y, -v.x, 0.0f);
  }
  return float3(0.0f, v.z, -v.y);
}

/* Scene evaluation with BVH bitfield filter */
float evalSceneBVH(float3 world_pos, out float3 out_color)
{
  float scene_dist = 1e10f;
  out_color = float3(0.5f);

  for (int g = 0; g < group_count; g++) {
    SDFGroupGPU grp = groups[g];
    float grp_dist = 1e10f;
    float3 grp_color = grp.color.rgb;
    bool grp_has_hit = false;

    for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
      if (!is_shape_near(m)) continue;

      SDFObjectGPU obj = objects[m];
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      float d = evalPrimitive(lp, obj);

      if (!grp_has_hit) {
        grp_dist = d;
        grp_color = obj.color.rgb;
        grp_has_hit = true;
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode);
        if (obj.csg_operation == 0 && d < prev) {
          float t = clamp(1.0f - (d - grp_dist) / max(obj.blend, 0.001f), 0.0f, 1.0f);
          grp_color = mix(grp_color, obj.color.rgb, t);
        }
      }
    }

    if (!grp_has_hit) continue;

    if (scene_dist >= 1e9f) {
      scene_dist = grp_dist;
      out_color = grp_color;
    }
    else {
      float prev = scene_dist;
      scene_dist = combineCSG(
          scene_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend,
          grp.shell_distance, grp.shell_mode);
      if (grp.csg_operation == 0 && grp_dist < prev) {
        float t = clamp(1.0f - (grp_dist - scene_dist) / max(grp.blend, 0.001f), 0.0f, 1.0f);
        out_color = mix(out_color, grp_color, t);
      }
    }
  }

  /* Ungrouped objects */
  for (int i = 0; i < object_count; i++) {
    if (!is_shape_near(i)) continue;

    SDFObjectGPU obj = objects[i];
    if (obj.group_id >= 0) continue;

    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);

    if (scene_dist >= 1e9f) {
      scene_dist = d;
      out_color = obj.color.rgb;
    }
    else {
      float prev = scene_dist;
      scene_dist = combineCSG(
          scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
          obj.shell_distance, obj.shell_mode);
      if (obj.csg_operation == 0 && d < prev) {
        float t = clamp(1.0f - (d - scene_dist) / max(obj.blend, 0.001f), 0.0f, 1.0f);
        out_color = mix(out_color, obj.color.rgb, t);
      }
    }
  }

  return scene_dist;
}

/* Full scene evaluation (no BVH filter) */
float evalSceneFull(float3 world_pos, out float3 out_color)
{
  float scene_dist = 1e10f;
  out_color = float3(0.5f);

  for (int g = 0; g < group_count; g++) {
    SDFGroupGPU grp = groups[g];
    float grp_dist = 1e10f;
    float3 grp_color = grp.color.rgb;
    bool grp_has_hit = false;

    for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
      SDFObjectGPU obj = objects[m];
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      float d = evalPrimitive(lp, obj);

      if (!grp_has_hit) {
        grp_dist = d;
        grp_color = obj.color.rgb;
        grp_has_hit = true;
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode);
        if (obj.csg_operation == 0 && d < prev) {
          float t = clamp(1.0f - (d - grp_dist) / max(obj.blend, 0.001f), 0.0f, 1.0f);
          grp_color = mix(grp_color, obj.color.rgb, t);
        }
      }
    }

    if (!grp_has_hit) continue;

    if (scene_dist >= 1e9f) {
      scene_dist = grp_dist;
      out_color = grp_color;
    }
    else {
      float prev = scene_dist;
      scene_dist = combineCSG(
          scene_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend,
          grp.shell_distance, grp.shell_mode);
      if (grp.csg_operation == 0 && grp_dist < prev) {
        float t = clamp(1.0f - (grp_dist - scene_dist) / max(grp.blend, 0.001f), 0.0f, 1.0f);
        out_color = mix(out_color, grp_color, t);
      }
    }
  }

  for (int i = 0; i < object_count; i++) {
    SDFObjectGPU obj = objects[i];
    if (obj.group_id >= 0) continue;

    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);

    if (scene_dist >= 1e9f) {
      scene_dist = d;
      out_color = obj.color.rgb;
    }
    else {
      float prev = scene_dist;
      scene_dist = combineCSG(
          scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
          obj.shell_distance, obj.shell_mode);
      if (obj.csg_operation == 0 && d < prev) {
        float t = clamp(1.0f - (d - scene_dist) / max(obj.blend, 0.001f), 0.0f, 1.0f);
        out_color = mix(out_color, obj.color.rgb, t);
      }
    }
  }
  return scene_dist;
}

float evalSceneDistFull(float3 world_pos)
{
  float3 dummy_color;
  return evalSceneFull(world_pos, dummy_color);
}

float evalSceneDistBVH(float3 world_pos)
{
  float3 dummy_color;
  return evalSceneBVH(world_pos, dummy_color);
}

float3 heat_color(float t)
{
  float3 cool = float3(0.0, 1.0, 0.0);
  float3 medium = float3(1.0, 1.0, 0.0);
  float3 hot = float3(1.0, 0.0, 0.0);
  return t < 0.5f ? mix(cool, medium, t / 0.5f) : mix(medium, hot, (t - 0.5f) / 0.5f);
}

void main()
{
  int2 pixel = ivec2(gl_GlobalInvocationID.xy);
  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float2 uv = (float2(pixel) + 0.5f) / float2(screen_size);

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

  float max_dist = 1000.0f;

  /* Scene AABB ray clipping (uniform, no per-pixel loop) */
  float3 s_min = scene_aabb_min;
  float3 s_max = scene_aabb_max;
  bool scene_aabb_valid = all(lessThan(s_min, s_max));
  float t_enter, t_exit;

  if (scene_aabb_valid) {
    float3 t0 = (s_min - ray_origin) * inv_dir;
    float3 t1 = (s_max - ray_origin) * inv_dir;
    float3 t_lo = min(t0, t1);
    float3 t_hi = max(t0, t1);
    t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

    if (t_enter > t_exit || t_exit < 0.0f) {
      if (debug_bvh_views == 0) {
        imageStore(out_color_img, pixel, float4(0.0));
        imageStore(out_depth_img, pixel, float4(0.0));
        return;
      }
      t_enter = 0.0f;
      t_exit = max_dist;
    }
    t_enter = max(t_enter, 0.0f);
    t_exit = min(t_exit, max_dist);
  }
  else {
    t_enter = 0.0f;
    t_exit = max_dist;
  }

  /* Clip ray endpoint for tighter BVH pruning */
  float3 ray_to = ray_origin + ray_dir * t_exit;

  /* BVH traversal */
  g_numNearShapes = 0;
  for (int w = 0; w < kBitfieldWords; w++) {
    g_bvh_bits[w] = 0u;
  }

  if (use_bvh != 0 && bvh_root >= 0) {
    float3 rayDirOrtho = normalize(find_ortho(ray_dir));
    float3 rayDirOrthoAbs = abs(rayDirOrtho);

    float3 rayBoundsMin = min(ray_origin, ray_to);
    float3 rayBoundsMax = max(ray_origin, ray_to);

    int stackTop = 0;
    int stack[kAabbTreeStackSize];
    stack[stackTop] = bvh_root;

    while (stackTop >= 0) {
      int index = stack[stackTop--];
      if (index < 0) continue;

      SdfAabbNodeGPU node = aabb_nodes[index];

      /* Loose AABB test */
      if (any(greaterThan(node.bounds_min.xyz, rayBoundsMax)) ||
          any(greaterThan(rayBoundsMin, node.bounds_max.xyz)))
      {
        continue;
      }

      /* Tight orthogonal separation test */
      float3 aabbCenter = 0.5f * (node.bounds_min.xyz + node.bounds_max.xyz);
      float3 aabbHalfExtents = 0.5f * (node.bounds_max.xyz - node.bounds_min.xyz);
      float separation = abs(dot(rayDirOrtho, ray_origin - aabbCenter)) -
                          dot(rayDirOrthoAbs, aabbHalfExtents);
      if (separation > 0.0f) continue;

      if (node.child_a < 0) {
        /* Leaf: precise slab test (handles camera inside AABB) */
        float3 lt1 = (node.bounds_min.xyz - ray_origin) * inv_dir;
        float3 lt2 = (node.bounds_max.xyz - ray_origin) * inv_dir;
        float ltMin = max(max(min(lt1.x, lt2.x), min(lt1.y, lt2.y)), min(lt1.z, lt2.z));
        float ltMax = min(min(max(lt1.x, lt2.x), max(lt1.y, lt2.y)), max(lt1.z, lt2.z));
        if (ltMax < 0.0f || ltMin > ltMax) continue;

        int idx = node.shape_index;
        if (idx >= 0 && idx < kMaxBitfieldBits) {
          uint word = uint(idx) >> 5u;
          uint bit = 1u << (uint(idx) & 31u);
          if ((g_bvh_bits[word] & bit) == 0u) {
            g_bvh_bits[word] |= bit;
            g_numNearShapes++;
          }
        }
      }
      else {
        if (stackTop + 2 < kAabbTreeStackSize) {
          stack[++stackTop] = node.child_a;
          stack[++stackTop] = node.child_b;
        }
      }
    }
  }
  else {
    /* BVH off: mark all objects as near */
    g_numNearShapes = object_count;
    for (int w = 0; w < kBitfieldWords; w++) {
      g_bvh_bits[w] = 0xFFFFFFFFu;
    }
  }

  /* Sphere tracing */
  float t = t_enter;
  bool hit = false;
  float3 hit_pos;
  float3 hit_color;
  int steps_taken = 0;

  for (int step = 0; step < sdf_max_steps; step++) {
    steps_taken++;
    float3 pos = ray_origin + ray_dir * t;
    float3 color;
    float d;
    if (use_bvh != 0) {
      d = evalSceneBVH(pos, color);
    }
    else {
      d = evalSceneFull(pos, color);
    }

    if (d < sdf_ray_epsilon) {
      hit = true;
      hit_pos = pos;
      hit_color = color;
      break;
    }

    t += max(d, sdf_ray_epsilon);
    if (t > t_exit) {
      break;
    }
  }

  /* Debug views */
  int local_idx = int(gl_LocalInvocationID.y * kTileSize + gl_LocalInvocationID.x);

  if (debug_bvh_views == 1) {
    float heat = float(use_bvh != 0 ? g_numNearShapes : object_count) / 16.0f;
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    return;
  }
  else if (debug_bvh_views == 2) {
    float heat = float(use_bvh != 0 ? g_numNearShapes : object_count) / 16.0f;
    tile_heat[local_idx] = heat;
    barrier();
    float max_heat = 0.0f;
    for (int i = 0; i < kTileSize * kTileSize; i++) {
      max_heat = max(max_heat, tile_heat[i]);
    }
    imageStore(out_color_img, pixel, float4(heat_color(max_heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    return;
  }
  else if (debug_bvh_views == 3) {
    float heat = float(steps_taken) / 64.0f;
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    return;
  }
  else if (debug_bvh_views == 4) {
    float heat = float(steps_taken) / 64.0f;
    tile_heat[local_idx] = heat;
    barrier();
    float max_heat = 0.0f;
    for (int i = 0; i < kTileSize * kTileSize; i++) {
      max_heat = max(max_heat, tile_heat[i]);
    }
    imageStore(out_color_img, pixel, float4(heat_color(max_heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    return;
  }

  if (!hit) {
    imageStore(out_color_img, pixel, float4(0.0));
    imageStore(out_depth_img, pixel, float4(0.0));
    return;
  }

  /* Normal computation */
  float eps = sdf_ray_epsilon * 2.0f;
  float3 normal;
  if (use_bvh != 0) {
    normal = normalize(float3(
        evalSceneDistBVH(hit_pos + float3(eps, 0.0f, 0.0f)) -
            evalSceneDistBVH(hit_pos - float3(eps, 0.0f, 0.0f)),
        evalSceneDistBVH(hit_pos + float3(0.0f, eps, 0.0f)) -
            evalSceneDistBVH(hit_pos - float3(0.0f, eps, 0.0f)),
        evalSceneDistBVH(hit_pos + float3(0.0f, 0.0f, eps)) -
            evalSceneDistBVH(hit_pos - float3(0.0f, 0.0f, eps))));
  }
  else {
    normal = normalize(float3(
        evalSceneDistFull(hit_pos + float3(eps, 0.0f, 0.0f)) -
            evalSceneDistFull(hit_pos - float3(eps, 0.0f, 0.0f)),
        evalSceneDistFull(hit_pos + float3(0.0f, eps, 0.0f)) -
            evalSceneDistFull(hit_pos - float3(0.0f, eps, 0.0f)),
        evalSceneDistFull(hit_pos + float3(0.0f, 0.0f, eps)) -
            evalSceneDistFull(hit_pos - float3(0.0f, 0.0f, eps))));
  }

  /* Shading */
  float3 obj_color = hit_color;
  float3 shaded_color;

  if (lighting_type == 0) {
    shaded_color = obj_color;
  }
  else if (lighting_type == 1) {
    float3 N = normalize((vm.viewmat * float4(normal, 0.0f)).xyz);
    float3 view_pos = (vm.viewmat * float4(hit_pos, 1.0f)).xyz;
    float3 I = drw_view_incident_vector(view_pos);

    float4 dirs[4] = float4[4](studio_light0, studio_light1, studio_light2, studio_light3);
    float4 cols[4] = float4[4](studio_color0, studio_color1, studio_color2, studio_color3);
    float4 specs[4] = float4[4](studio_spec0, studio_spec1, studio_spec2, studio_spec3);

    float3 diffuse_light = studio_ambient;
    float3 specular_light = studio_ambient;
    float roughness = 0.5f;

    for (int i = 0; i < 4; i++) {
      float NL = dot(dirs[i].xyz, N);
      float w = cols[i].w;
      float w1 = w + 1.0f;
      diffuse_light += cols[i].rgb * clamp((NL + w) / (w1 * w1), 0.0f, 1.0f);
    }

    float3 spec_col = float3(0.0f);
    if (use_specular != 0) {
      float3 R = -reflect(I, N);
      for (int i = 0; i < 4; i++) {
        float3 L = dirs[i].xyz;
        float w = cols[i].w;
        float3 H = normalize(L + I);
        float spec_angle = clamp(dot(H, N), 0.0f, 1.0f);
        float cNL = clamp(dot(L, N), 0.0f, 1.0f);

        float gloss = (1.0f - roughness) * (1.0f - w);
        float shininess = exp2(10.0f * gloss + 1.0f);
        float norm_factor = shininess * 0.125f + 1.0f;
        float spec = pow(spec_angle, shininess) * cNL * norm_factor;

        float wrap_NL = dot(L, R);
        float w_s = mix(w, 1.0f, roughness);
        float w_s1 = w_s + 1.0f;
        float spec_env = clamp((wrap_NL + w_s) / (w_s1 * w_s1), 0.0f, 1.0f);

        specular_light += specs[i].rgb * mix(spec, spec_env, w * w);
      }

      spec_col = float3(0.05f);
      float NV = clamp(dot(N, I), 0.0f, 1.0f);
      float fresnel = exp2(-8.35f * NV) * (1.0f - roughness);
      spec_col = mix(spec_col, float3(1.0f), fresnel);
    }

    specular_light *= spec_col;
    float spec_energy = dot(spec_col, float3(0.33333f));
    diffuse_light *= obj_color * (1.0f - spec_energy);
    shaded_color = diffuse_light + specular_light;
  }
  else {
    float3 view_normal = normalize((vm.viewmat * float4(normal, 0.0f)).xyz);
    float3 view_pos = (vm.viewmat * float4(hit_pos, 1.0f)).xyz;
    float3 I = drw_view_incident_vector(view_pos);

    float a = 1.0f / (1.0f + I.z);
    float b = -I.x * I.y * a;
    float3 b1 = float3(1.0f - I.x * I.x * a, b, -I.x);
    float3 b2 = float3(b, 1.0f - I.y * I.y * a, -I.y);
    float2 matcap_uv = float2(dot(b1, view_normal), dot(b2, view_normal));
    if (use_matcap_flip != 0) {
      matcap_uv.x = -matcap_uv.x;
    }
    matcap_uv = matcap_uv * 0.496f + 0.5f;

    float3 diffuse = textureLod(matcap_tx, float3(matcap_uv, 0.0f), 0.0f).rgb;
    float3 specular = textureLod(matcap_tx, float3(matcap_uv, 1.0f), 0.0f).rgb;

    shaded_color = diffuse * obj_color + specular * float(use_specular);
  }

  imageStore(out_color_img, pixel, float4(shaded_color, 1.0f));
  imageStore(out_depth_img, pixel, float4(drw_point_world_to_screen(hit_pos).z));
}
