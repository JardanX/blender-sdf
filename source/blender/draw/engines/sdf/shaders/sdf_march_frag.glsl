/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF ray-march fragment shader (sparse brick version).
 * Two-level DDA: brick-level traversal skips empty space, then
 * voxel-level DDA within active bricks finds exact surface hits.
 *
 * Based on "Ray Tracing of Signed Distance Function Grids"
 * (Hansson-Soderlund, Evans, Akenine-Moller 2022).
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

/** Max brick-level DDA steps. Non-cubic grids up to 128/axis → diagonal ~ 222. */
#define MAX_BRICK_STEPS 256
/** Max voxel-level DDA steps within one brick (8 * sqrt(3) ~ 14). */
#define MAX_VOXEL_STEPS 24

/**
 * Convert a grid-space voxel coordinate to compact atlas coordinate.
 * \param brick: brick coordinate in the grid.
 * \param local_voxel: voxel offset within the brick [0..7].
 * \param slot: compact atlas slot index for this brick.
 * \param bpa: bricks per axis in the compact atlas.
 * \return texel coordinate in the compact atlas.
 */
int3 gridToCompact(int3 brick, int3 local_voxel, int slot, int bpa)
{
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;
  /* +2 for the 2-voxel overlap padding border. */
  return slot_origin + local_voxel + int3(2);
}

/**
 * Fetch 8 corner SDF values for a voxel cell within a compact atlas brick.
 */
void fetchCornersCompact(int3 brick, int3 local_cell, int slot, int bpa, out float s[8])
{
  int3 base = gridToCompact(brick, local_cell, slot, bpa);
  s[0] = texelFetch(compact_atlas, base + int3(0, 0, 0), 0).r;
  s[1] = texelFetch(compact_atlas, base + int3(1, 0, 0), 0).r;
  s[2] = texelFetch(compact_atlas, base + int3(0, 1, 0), 0).r;
  s[3] = texelFetch(compact_atlas, base + int3(1, 1, 0), 0).r;
  s[4] = texelFetch(compact_atlas, base + int3(0, 0, 1), 0).r;
  s[5] = texelFetch(compact_atlas, base + int3(1, 0, 1), 0).r;
  s[6] = texelFetch(compact_atlas, base + int3(0, 1, 1), 0).r;
  s[7] = texelFetch(compact_atlas, base + int3(1, 1, 1), 0).r;
}

/**
 * Compute normal from trilinear gradient at the hit point.
 * Fetches 8 corner SDF values and computes the analytic gradient (Eq. 9-11).
 * Much cheaper than the 27-fetch dual voxel method while giving smooth results
 * for blended SDFs (the smooth union produces a smooth distance field).
 */
float3 computeNormalCompact(float3 grid_pos_in_brick, int3 brick, int slot, int bpa)
{
  int3 cell = int3(floor(grid_pos_in_brick));
  cell = clamp(cell, int3(0), int3(BRICK_SIZE - 1));

  float3 frac_pos = grid_pos_in_brick - float3(cell);
  frac_pos = clamp(frac_pos, float3(0.0f), float3(1.0f));

  float s[8];
  fetchCornersCompact(brick, cell, slot, bpa, s);

  float3 grad = trilinearGradient(s, frac_pos);
  float len = length(grad);
  return len > 1e-8f ? grad / len : float3(0.0f, 0.0f, 1.0f);
}

void main()
{
  float2 uv = screen_uv;

  /* ---- 1. Reconstruct camera ray ---- */
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

  /* ---- 2. Clip ray to total voxel volume ---- */
  int3 grid_res = grid_resolution;
  int3 total_res = grid_res * BRICK_SIZE;
  float3 grid_world_min = atlas_origin;
  float3 grid_world_max = atlas_origin + float3(total_res) * voxel_size;

  float3 inv_dir = 1.0f / ray_dir;
  float3 t0 = (grid_world_min - ray_origin) * inv_dir;
  float3 t1 = (grid_world_max - ray_origin) * inv_dir;
  float3 t_lo = min(t0, t1);
  float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  float t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

  if (t_enter > t_exit || t_exit < 0.0f) {
    discard;
    return;
  }

  t_enter = max(t_enter, 0.0f);

  /* ---- 3. Brick-level DDA setup ---- */
  float inv_voxel = 1.0f / voxel_size;
  float brick_world_size = float(BRICK_SIZE) * voxel_size;
  float inv_brick = 1.0f / brick_world_size;

  /* Brick-space ray: B = (ray_origin - atlas_origin) / brick_world_size */
  float3 B = (ray_origin - atlas_origin) * inv_brick;
  float3 BD = ray_dir * inv_brick;

  float3 B_enter = B + t_enter * BD;
  int3 brick_cell = int3(floor(B_enter));
  brick_cell = clamp(brick_cell, int3(0), grid_res - int3(1));

  int3 brick_step;
  brick_step.x = BD.x > 0.0f ? 1 : (BD.x < 0.0f ? -1 : 0);
  brick_step.y = BD.y > 0.0f ? 1 : (BD.y < 0.0f ? -1 : 0);
  brick_step.z = BD.z > 0.0f ? 1 : (BD.z < 0.0f ? -1 : 0);

  float3 brick_tDelta = float3(BD.x != 0.0f ? abs(1.0f / BD.x) : 1e30f,
                                BD.y != 0.0f ? abs(1.0f / BD.y) : 1e30f,
                                BD.z != 0.0f ? abs(1.0f / BD.z) : 1e30f);

  float3 brick_boundary;
  brick_boundary.x = BD.x > 0.0f ? float(brick_cell.x + 1) : float(brick_cell.x);
  brick_boundary.y = BD.y > 0.0f ? float(brick_cell.y + 1) : float(brick_cell.y);
  brick_boundary.z = BD.z > 0.0f ? float(brick_cell.z + 1) : float(brick_cell.z);

  float3 brick_tMax;
  brick_tMax.x = BD.x != 0.0f ? (brick_boundary.x - B.x) / BD.x : 1e30f;
  brick_tMax.y = BD.y != 0.0f ? (brick_boundary.y - B.y) / BD.y : 1e30f;
  brick_tMax.z = BD.z != 0.0f ? (brick_boundary.z - B.z) / BD.z : 1e30f;

  /* ---- 4. Two-level DDA traversal ---- */
  float hit_t = -1.0f;
  int3 hit_brick = int3(0);
  int3 hit_local_cell = int3(0);
  int hit_slot = -1;

  /* Debug occupancy tracking. */
  int bricks_traversed = 0;
  int active_bricks_hit = 0;

  float t_current = t_enter;

  for (int bstep = 0; bstep < MAX_BRICK_STEPS; bstep++) {
    if (any(lessThan(brick_cell, int3(0))) || any(greaterThanEqual(brick_cell, grid_res))) {
      break;
    }

    bricks_traversed++;

    /* Brick exit t. */
    float t_brick_exit = min(min(brick_tMax.x, brick_tMax.y), brick_tMax.z);
    t_brick_exit = min(t_brick_exit, t_exit);

    /* Read indirection. */
    int slot = texelFetch(indirection_tx, brick_cell, 0).r;

    if (slot == -2) {
      /* Fully inside the SDF: surface was already passed. Stop marching. */
      break;
    }

    if (slot >= 0) {
      /* Active brick: enter voxel-level DDA. */
      active_bricks_hit++;

      /* Voxel-space ray within this brick.
       * V = (ray_origin - brick_world_origin) / voxel_size
       * brick_world_origin = atlas_origin + brick_cell * BRICK_SIZE * voxel_size */
      float3 brick_origin = atlas_origin + float3(brick_cell * BRICK_SIZE) * voxel_size;
      float3 V = (ray_origin - brick_origin) * inv_voxel;
      float3 VD = ray_dir * inv_voxel;

      float3 V_enter = V + t_current * VD;
      int3 vcell = int3(floor(V_enter));
      vcell = clamp(vcell, int3(0), int3(BRICK_SIZE - 1));

      int3 vstep;
      vstep.x = VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0);
      vstep.y = VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0);
      vstep.z = VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0);

      float3 vtDelta = float3(VD.x != 0.0f ? abs(1.0f / VD.x) : 1e30f,
                               VD.y != 0.0f ? abs(1.0f / VD.y) : 1e30f,
                               VD.z != 0.0f ? abs(1.0f / VD.z) : 1e30f);

      float3 vbound;
      vbound.x = VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x);
      vbound.y = VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y);
      vbound.z = VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z);

      float3 vtMax;
      vtMax.x = VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f;
      vtMax.y = VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f;
      vtMax.z = VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f;

      float vt_current = t_current;
      bool voxel_hit = false;

      for (int vstep_i = 0; vstep_i < MAX_VOXEL_STEPS; vstep_i++) {
        if (any(lessThan(vcell, int3(0))) || any(greaterThan(vcell, int3(BRICK_SIZE - 1)))) {
          break;
        }

        float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
        vt_cell_exit = min(vt_cell_exit, t_brick_exit);

        /* Fetch 8 corners from compact atlas. */
        float s[8];
        fetchCornersCompact(brick_cell, vcell, slot, bricks_per_axis, s);

        float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                         min(min(s[4], s[5]), min(s[6], s[7])));
        float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                         max(max(s[4], s[5]), max(s[6], s[7])));

        if (smin <= 0.0f) {
          if (smax < 0.0f) {
            /* All negative: immediate hit. */
            hit_t = vt_current;
            hit_brick = brick_cell;
            hit_local_cell = vcell;
            hit_slot = slot;
            voxel_hit = true;
            break;
          }

          /* Mixed signs: cubic solve. */
          float T_max = vt_cell_exit - vt_current;
          if (T_max > 1e-8f) {
            float k[8];
            computeTrilinearCoeffs(s, k);

            float3 o_local = V + vt_current * VD - float3(vcell);
            o_local = clamp(o_local, float3(0.0f), float3(1.0f));
            float3 d_scaled = VD * T_max;

            float c[4];
            computeCubicCoeffs(k, o_local, d_scaled, c);

            if (c[0] <= 0.0f) {
              hit_t = vt_current;
              hit_brick = brick_cell;
              hit_local_cell = vcell;
              hit_slot = slot;
              voxel_hit = true;
              break;
            }

            float u_hit = solveCubicMarmittNR(c, 1.0f);
            if (u_hit >= 0.0f) {
              hit_t = vt_current + u_hit * T_max;
              hit_brick = brick_cell;
              hit_local_cell = vcell;
              hit_slot = slot;
              voxel_hit = true;
              break;
            }
          }
        }

        /* Advance voxel DDA. */
        if (vtMax.x < vtMax.y) {
          if (vtMax.x < vtMax.z) {
            vt_current = vtMax.x;
            vcell.x += vstep.x;
            vtMax.x += vtDelta.x;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }
        else {
          if (vtMax.y < vtMax.z) {
            vt_current = vtMax.y;
            vcell.y += vstep.y;
            vtMax.y += vtDelta.y;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }

        if (vt_current >= t_brick_exit) {
          break;
        }
      }

      if (voxel_hit) {
        break;
      }
    }

    /* ---- Advance brick-level DDA ---- */
    if (brick_tMax.x < brick_tMax.y) {
      if (brick_tMax.x < brick_tMax.z) {
        t_current = brick_tMax.x;
        brick_cell.x += brick_step.x;
        brick_tMax.x += brick_tDelta.x;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }
    else {
      if (brick_tMax.y < brick_tMax.z) {
        t_current = brick_tMax.y;
        brick_cell.y += brick_step.y;
        brick_tMax.y += brick_tDelta.y;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }

    if (t_current > t_exit) {
      break;
    }
  }

  /* ---- 5. No hit: discard fragment ---- */
  if (hit_t < 0.0f) {
    discard;
    return;
  }

  float3 hit_pos = ray_origin + ray_dir * hit_t;

  /* ---- 5b. Debug: Object ID visualization ---- */
  if (debug_mode == 1) {
    /* Read object ID from atlas at hit position. */
    float3 brick_origin_dbg = atlas_origin + float3(hit_brick * BRICK_SIZE) * voxel_size;
    float3 local_pos_dbg = (hit_pos - brick_origin_dbg) * inv_voxel;

    int bpa_dbg = bricks_per_axis;
    int3 slot_block_dbg = int3(hit_slot % bpa_dbg,
                               (hit_slot / bpa_dbg) % bpa_dbg,
                               hit_slot / (bpa_dbg * bpa_dbg));
    int3 atlas_coord_dbg = slot_block_dbg * BRICK_STORAGE + int3(floor(local_pos_dbg)) + int3(2);
    int obj_id = texelFetch(object_id_tx, atlas_coord_dbg, 0).r;

    /* Golden-ratio hue hash for distinct per-object colors. */
    float3 id_color;
    if (obj_id < 0) {
      id_color = float3(0.15f); /* Dark gray for unclaimed voxels. */
    }
    else {
      float hue = fract(float(obj_id) * 0.618033988749895f + 0.1f);
      /* Simplified HSV→RGB (S=0.75, V=0.9). */
      float3 k = fract(float3(hue) + float3(0.0f, 0.6667f, 0.3333f)) * 6.0f;
      id_color = 0.9f * (1.0f - 0.75f * max(1.0f - abs(k - 3.0f), float3(0.0f)));
    }

    /* Simple lambertian with fixed light for depth cues. */
    float3 brick_origin_n = atlas_origin + float3(hit_brick * BRICK_SIZE) * voxel_size;
    float3 hit_in_brick_n = (hit_pos - brick_origin_n) * inv_voxel;
    float3 N = computeNormalCompact(
        hit_in_brick_n, hit_brick, hit_slot, bricks_per_axis);
    float NdotL = max(dot(normalize(N), normalize(float3(0.5f, 0.7f, 1.0f))), 0.0f);
    float shade = 0.35f + 0.65f * NdotL;

    out_color = float4(id_color * shade, 1.0f);
    gl_FragDepth = drw_point_world_to_screen(hit_pos).z;
    return;
  }

  /* ---- 6. Compute normal ---- */
  /* Grid position within the brick (0..BRICK_SIZE). */
  float3 brick_origin = atlas_origin + float3(hit_brick * BRICK_SIZE) * voxel_size;
  float3 hit_in_brick = (hit_pos - brick_origin) * inv_voxel;
  float3 normal = computeNormalCompact(hit_in_brick, hit_brick, hit_slot, bricks_per_axis);

  /* ---- 7. Read blended color ---- */
  /* Trilinear interpolate color from the compact atlas. */
  float3 local_pos = (hit_pos - brick_origin) * inv_voxel;

  /* Compute compact atlas UV for hardware trilinear. */
  int bpa = bricks_per_axis;
  int3 slot_block = int3(hit_slot % bpa, (hit_slot / bpa) % bpa, hit_slot / (bpa * bpa));
  float3 atlas_pos = float3(slot_block * BRICK_STORAGE) + local_pos + float3(2.0f);

  /* Normalize to [0,1] for texture() sampling. */
  int3 compact_size = int3(textureSize(compact_atlas, 0));
  float3 atlas_uv = atlas_pos / float3(compact_size);
  float3 obj_color = textureLod(compact_atlas, atlas_uv, 0.0f).gba;

  /* ---- 8. Shading ---- */
  float3 shaded_color;
  if (lighting_type == 0) {
    shaded_color = obj_color;
  }
  else if (lighting_type == 1) {
    /* STUDIO lighting. */
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
    /* MATCAP lighting. */
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

  out_color = float4(shaded_color, 1.0f);

  /* ---- 9. Write depth ---- */
  gl_FragDepth = drw_point_world_to_screen(hit_pos).z;
}
