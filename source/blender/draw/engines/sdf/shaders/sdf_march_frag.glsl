/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF ray-march fragment shader.
 * Fullscreen pass that uses DDA traversal + analytic cubic intersection
 * to find exact surface hits in the baked 3D SDF atlas.
 *
 * Based on "Ray Tracing of Signed Distance Function Grids"
 * (Hansson-Soderlund, Evans, Akenine-Moller 2022).
 */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_march)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

/** Maximum DDA traversal steps (256 * sqrt(3) ~ 443, rounded up). */
#define MAX_DDA_STEPS 512

/**
 * Fetch 8 corner SDF values for a voxel cell.
 * Corner ordering: s[0]=s000, s[1]=s100, s[2]=s010, s[3]=s110,
 *                  s[4]=s001, s[5]=s101, s[6]=s011, s[7]=s111
 */
void fetchCorners(int3 cell, out float s[8])
{
  s[0] = texelFetch(sdf_atlas, cell + int3(0, 0, 0), 0).r;
  s[1] = texelFetch(sdf_atlas, cell + int3(1, 0, 0), 0).r;
  s[2] = texelFetch(sdf_atlas, cell + int3(0, 1, 0), 0).r;
  s[3] = texelFetch(sdf_atlas, cell + int3(1, 1, 0), 0).r;
  s[4] = texelFetch(sdf_atlas, cell + int3(0, 0, 1), 0).r;
  s[5] = texelFetch(sdf_atlas, cell + int3(1, 0, 1), 0).r;
  s[6] = texelFetch(sdf_atlas, cell + int3(0, 1, 1), 0).r;
  s[7] = texelFetch(sdf_atlas, cell + int3(1, 1, 1), 0).r;
}

/**
 * Compute C0-continuous normal using the dual voxel method.
 * (Section 3.2, Hansson-Soderlund et al. 2022, Eq. 12)
 *
 * A dual voxel (shifted by half a voxel) overlaps 2x2x2 primal voxels.
 * Computes analytic gradient normal in each primal voxel at the hit point,
 * normalizes each, then trilinearly interpolates the 8 results.
 * Eliminates C0 discontinuity at primal voxel boundaries.
 *
 * Cost: 27 texelFetch + 8 gradient evaluations + 7 mix operations.
 *
 * \param grid_pos: hit point in grid space (sample i,j,k at position i,j,k).
 * \param res: atlas resolution.
 * \return normalized normal in grid space (== world space after normalization).
 */
float3 computeDualVoxelNormal(float3 grid_pos, int3 res)
{
  /* Dual voxel base: shift grid by -0.5 then floor. */
  int3 base = int3(floor(grid_pos - float3(0.5f)));
  base = clamp(base, int3(0), res - int3(3));

  /* Interpolation weights within the dual voxel [0,1]^3. */
  float3 uvw = clamp(grid_pos - float3(0.5f) - float3(base), float3(0.0f), float3(1.0f));

  /* Fetch 3x3x3 = 27 neighborhood samples. Index: v[z*9 + y*3 + x]. */
  float v[27];
  v[ 0] = texelFetch(sdf_atlas, base + int3(0, 0, 0), 0).r;
  v[ 1] = texelFetch(sdf_atlas, base + int3(1, 0, 0), 0).r;
  v[ 2] = texelFetch(sdf_atlas, base + int3(2, 0, 0), 0).r;
  v[ 3] = texelFetch(sdf_atlas, base + int3(0, 1, 0), 0).r;
  v[ 4] = texelFetch(sdf_atlas, base + int3(1, 1, 0), 0).r;
  v[ 5] = texelFetch(sdf_atlas, base + int3(2, 1, 0), 0).r;
  v[ 6] = texelFetch(sdf_atlas, base + int3(0, 2, 0), 0).r;
  v[ 7] = texelFetch(sdf_atlas, base + int3(1, 2, 0), 0).r;
  v[ 8] = texelFetch(sdf_atlas, base + int3(2, 2, 0), 0).r;
  v[ 9] = texelFetch(sdf_atlas, base + int3(0, 0, 1), 0).r;
  v[10] = texelFetch(sdf_atlas, base + int3(1, 0, 1), 0).r;
  v[11] = texelFetch(sdf_atlas, base + int3(2, 0, 1), 0).r;
  v[12] = texelFetch(sdf_atlas, base + int3(0, 1, 1), 0).r;
  v[13] = texelFetch(sdf_atlas, base + int3(1, 1, 1), 0).r;
  v[14] = texelFetch(sdf_atlas, base + int3(2, 1, 1), 0).r;
  v[15] = texelFetch(sdf_atlas, base + int3(0, 2, 1), 0).r;
  v[16] = texelFetch(sdf_atlas, base + int3(1, 2, 1), 0).r;
  v[17] = texelFetch(sdf_atlas, base + int3(2, 2, 1), 0).r;
  v[18] = texelFetch(sdf_atlas, base + int3(0, 0, 2), 0).r;
  v[19] = texelFetch(sdf_atlas, base + int3(1, 0, 2), 0).r;
  v[20] = texelFetch(sdf_atlas, base + int3(2, 0, 2), 0).r;
  v[21] = texelFetch(sdf_atlas, base + int3(0, 1, 2), 0).r;
  v[22] = texelFetch(sdf_atlas, base + int3(1, 1, 2), 0).r;
  v[23] = texelFetch(sdf_atlas, base + int3(2, 1, 2), 0).r;
  v[24] = texelFetch(sdf_atlas, base + int3(0, 2, 2), 0).r;
  v[25] = texelFetch(sdf_atlas, base + int3(1, 2, 2), 0).r;
  v[26] = texelFetch(sdf_atlas, base + int3(2, 2, 2), 0).r;

  /* For each of 8 overlapping primal voxels: extract corners, compute
   * analytic gradient at hit point, normalize. */
  float3 normals[8];
  for (int dz = 0; dz < 2; dz++) {
    for (int dy = 0; dy < 2; dy++) {
      for (int dx = 0; dx < 2; dx++) {
        int o = dz * 9 + dy * 3 + dx;
        float corners[8];
        corners[0] = v[o];
        corners[1] = v[o + 1];
        corners[2] = v[o + 3];
        corners[3] = v[o + 4];
        corners[4] = v[o + 9];
        corners[5] = v[o + 10];
        corners[6] = v[o + 12];
        corners[7] = v[o + 13];

        /* Hit point in this primal voxel's local space (may be outside [0,1],
         * but the trilinear gradient polynomial is valid everywhere). */
        float3 local = grid_pos - float3(base + int3(dx, dy, dz));

        float3 grad = trilinearGradient(corners, local);
        float len = length(grad);
        normals[dz * 4 + dy * 2 + dx] = len > 1e-8f ? grad / len
                                                       : float3(0.0f, 0.0f, 1.0f);
      }
    }
  }

  /* Trilinear interpolation of 8 normalized normals (Eq. 12). */
  float3 n00 = mix(normals[0], normals[1], uvw.x);
  float3 n10 = mix(normals[2], normals[3], uvw.x);
  float3 n01 = mix(normals[4], normals[5], uvw.x);
  float3 n11 = mix(normals[6], normals[7], uvw.x);
  float3 n0 = mix(n00, n10, uvw.y);
  float3 n1 = mix(n01, n11, uvw.y);
  return normalize(mix(n0, n1, uvw.z));
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

  /* ---- 2. Clip ray to grid interpolation bounds ---- */
  /* Grid coordinates [0, res-1] map to world positions offset by half a voxel
   * from atlas_origin. The trilinear grid only covers between sample centers. */
  int3 res = atlas_resolution;
  float3 grid_world_min = atlas_origin + float3(0.5f) * voxel_size;
  float3 grid_world_max = atlas_origin + (float3(res) - float3(0.5f)) * voxel_size;

  float3 inv_dir = 1.0f / ray_dir;
  float3 t0 = (grid_world_min - ray_origin) * inv_dir;
  float3 t1 = (grid_world_max - ray_origin) * inv_dir;
  float3 t_lo = min(t0, t1);
  float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  float t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

  /* No intersection with grid volume. */
  if (t_enter > t_exit || t_exit < 0.0f) {
    discard;
    return;
  }

  t_enter = max(t_enter, 0.0f);

  /* ---- 3. Transform ray to grid space ---- */
  /* Grid space: sample (i,j,k) is at position (i,j,k).
   * grid_pos = (world_pos - atlas_origin) / voxel_size - 0.5
   * The parametric t is shared between world and grid space. */
  float inv_voxel = 1.0f / voxel_size;
  float3 G = (ray_origin - atlas_origin) * inv_voxel - float3(0.5f); /* grid ray origin */
  float3 D = ray_dir * inv_voxel;                                     /* grid ray direction */

  /* ---- 4. DDA setup (Amanatides & Woo) ---- */
  float3 P_enter = G + t_enter * D; /* grid position at entry */

  /* Starting cell (clamped to valid range [0, res-2]). */
  int3 cell = int3(floor(P_enter));
  cell = clamp(cell, int3(0), res - int3(2));

  /* Step direction per axis. */
  int3 step_dir;
  step_dir.x = D.x > 0.0f ? 1 : (D.x < 0.0f ? -1 : 0);
  step_dir.y = D.y > 0.0f ? 1 : (D.y < 0.0f ? -1 : 0);
  step_dir.z = D.z > 0.0f ? 1 : (D.z < 0.0f ? -1 : 0);

  /* t-delta: world-t to traverse one cell width per axis. */
  float3 tDelta = float3(D.x != 0.0f ? abs(1.0f / D.x) : 1e30f,
                          D.y != 0.0f ? abs(1.0f / D.y) : 1e30f,
                          D.z != 0.0f ? abs(1.0f / D.z) : 1e30f);

  /* tMax: world-t at which the ray crosses the next cell boundary per axis.
   * Computed from grid ray origin (G) and the next boundary in step direction. */
  float3 boundary;
  boundary.x = D.x > 0.0f ? float(cell.x + 1) : float(cell.x);
  boundary.y = D.y > 0.0f ? float(cell.y + 1) : float(cell.y);
  boundary.z = D.z > 0.0f ? float(cell.z + 1) : float(cell.z);

  float3 tMax;
  tMax.x = D.x != 0.0f ? (boundary.x - G.x) / D.x : 1e30f;
  tMax.y = D.y != 0.0f ? (boundary.y - G.y) / D.y : 1e30f;
  tMax.z = D.z != 0.0f ? (boundary.z - G.z) / D.z : 1e30f;

  /* ---- 5. Hybrid sphere-trace / DDA traversal ---- */
  /* Strategy: use a cheap textureLod probe at the current position.
   * When far from the surface (probe > 2 voxels), jump ahead like sphere
   * tracing — O(1) texture reads vs O(cells) with pure DDA.
   * When near the surface, fall through to the full 8-corner fetch + cubic
   * solve for exact analytic intersection. This gives the best of both:
   * sphere tracing's logarithmic convergence in empty space + the paper's
   * exact intersection at the surface. */
  float t_current = t_enter;
  float hit_t = -1.0f;
  int3 hit_cell = int3(0);
  float s[8]; /* corner values — persists across iterations, valid at hit */
  float3 inv_extent = 1.0f / atlas_extent;

  for (int step = 0; step < MAX_DDA_STEPS; step++) {
    /* Bounds check: cell must be in [0, res-2]^3. */
    if (any(lessThan(cell, int3(0))) || any(greaterThan(cell, res - int3(2)))) {
      break;
    }

    /* --- Empty-space skip: 1 textureLod probe vs 8 texelFetch --- */
    float3 probe_world = ray_origin + ray_dir * t_current;
    float3 probe_uv = (probe_world - atlas_origin) * inv_extent;
    float probe_dist = textureLod(sdf_atlas, probe_uv, 0.0f).r;

    if (probe_dist > voxel_size * 2.0f) {
      /* Far from surface: sphere-tracing jump. */
      t_current += probe_dist * 0.9f;
      if (t_current > t_exit) {
        break;
      }
      /* Recompute DDA state at new position. */
      float3 new_pos = G + t_current * D;
      cell = clamp(int3(floor(new_pos)), int3(0), res - int3(2));
      boundary.x = D.x > 0.0f ? float(cell.x + 1) : float(cell.x);
      boundary.y = D.y > 0.0f ? float(cell.y + 1) : float(cell.y);
      boundary.z = D.z > 0.0f ? float(cell.z + 1) : float(cell.z);
      tMax.x = D.x != 0.0f ? (boundary.x - G.x) / D.x : 1e30f;
      tMax.y = D.y != 0.0f ? (boundary.y - G.y) / D.y : 1e30f;
      tMax.z = D.z != 0.0f ? (boundary.z - G.z) / D.z : 1e30f;
      continue;
    }

    /* Exit t for this cell (smallest tMax across all axes, capped at atlas exit). */
    float t_cell_exit = min(min(tMax.x, tMax.y), tMax.z);
    t_cell_exit = min(t_cell_exit, t_exit);

    /* (a) Fetch 8 corners via texelFetch. */
    fetchCorners(cell, s);

    /* (b) Quick reject: find min/max of all corners. */
    float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                     min(min(s[4], s[5]), min(s[6], s[7])));
    float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                     max(max(s[4], s[5]), max(s[6], s[7])));

    /* All positive: no surface in this cell. */
    if (smin > 0.0f) {
      /* Skip — fall through to DDA advance. */
    }
    else {
      /* (c) All negative: entire cell is inside surface -> immediate hit. */
      if (smax < 0.0f) {
        hit_t = t_current;
        hit_cell = cell;
        break;
      }

      /* (d) Mixed signs: compute trilinear coefficients and solve cubic. */
      float T_max = t_cell_exit - t_current;

      /* Skip zero-width cells (ray barely grazes a cell corner). */
      if (T_max > 1e-8f) {
        float k[8];
        computeTrilinearCoeffs(s, k);

        /* Ray origin in cell-local [0,1]^3 coordinates at t_current. */
        float3 o_local = G + t_current * D - float3(cell);
        /* Clamp to [0,1] to prevent numerical drift at cell boundaries. */
        o_local = clamp(o_local, float3(0.0f), float3(1.0f));

        /* CRITICAL: Scale direction by T_max so the cubic parameter u is in
         * [0,1] instead of [0, T_max]. With raw D (~60 for a 256^3 grid),
         * coefficients span 5+ orders of magnitude and roots are ~0.005,
         * which is catastrophically ill-conditioned for float32.
         * With d_scaled (~1), all coefficients are O(0.1) and u ~ 0.5. */
        float3 d_scaled = D * T_max;

        float c[4];
        computeCubicCoeffs(k, o_local, d_scaled, c);

        /* Check if entry point is already inside the surface. */
        if (c[0] <= 0.0f) {
          hit_t = t_current;
          hit_cell = cell;
          break;
        }

        /* Solve cubic for smallest root u in [0, 1].
         * Marmitt+NR is more robust than Vieta's on GPU: no transcendental
         * functions, no discriminant-near-zero edge cases, uniform branches. */
        float u_hit = solveCubicMarmittNR(c, 1.0f);

        if (u_hit >= 0.0f) {
          hit_t = t_current + u_hit * T_max;
          hit_cell = cell;
          break;
        }
      }
    }

    /* ---- Advance DDA to next voxel ---- */
    if (tMax.x < tMax.y) {
      if (tMax.x < tMax.z) {
        t_current = tMax.x;
        cell.x += step_dir.x;
        tMax.x += tDelta.x;
      }
      else {
        t_current = tMax.z;
        cell.z += step_dir.z;
        tMax.z += tDelta.z;
      }
    }
    else {
      if (tMax.y < tMax.z) {
        t_current = tMax.y;
        cell.y += step_dir.y;
        tMax.y += tDelta.y;
      }
      else {
        t_current = tMax.z;
        cell.z += step_dir.z;
        tMax.z += tDelta.z;
      }
    }

    if (t_current > t_exit) {
      break;
    }
  }

  /* ---- 6. No hit: discard fragment ---- */
  if (hit_t < 0.0f) {
    discard;
    return;
  }

  float3 hit_pos = ray_origin + ray_dir * hit_t;

  /* ---- 7. Compute normal via dual voxel method ---- */
  /* Dual voxel interpolation gives C0-continuous normals across voxel
   * boundaries, unlike the single-voxel analytic gradient which has
   * C0 discontinuities. (Section 3.2, Hansson-Soderlund et al. 2022) */
  float3 hit_grid = G + hit_t * D;
  float3 normal = computeDualVoxelNormal(hit_grid, res);

  /* ---- 8. Shading ---- */
  /* Read blended color from atlas (baked alongside distance).
   * Atlas layout: .r = signed distance, .gba = RGB color.
   * One textureLod gives hardware-trilinear-interpolated color — O(1). */
  float3 hit_uv = (hit_pos - atlas_origin) * inv_extent;
  float3 obj_color = textureLod(sdf_atlas, hit_uv, 0.0f).gba;

  float3 shaded_color;
  if (lighting_type == 0) {
    /* FLAT: output object color directly, no lighting. */
    shaded_color = obj_color;
  }
  else if (lighting_type == 1) {
    /* STUDIO: 4 directional lights + ambient + specular.
     * Matches workbench_world_light_lib.glsl: get_world_lighting().
     * Studio light directions are in view space (camera-relative),
     * so transform the world-space normal to view space. */
    float3 N = normalize((vm.viewmat * float4(normal, 0.0f)).xyz);
    float3 view_pos = (vm.viewmat * float4(hit_pos, 1.0f)).xyz;
    float3 I = drw_view_incident_vector(view_pos);

    float4 dirs[4] = float4[4](studio_light0, studio_light1, studio_light2, studio_light3);
    float4 cols[4] = float4[4](studio_color0, studio_color1, studio_color2, studio_color3);
    float4 specs[4] = float4[4](studio_spec0, studio_spec1, studio_spec2, studio_spec3);

    float3 diffuse_light = studio_ambient;
    float3 specular_light = studio_ambient;
    float roughness = 0.5f;

    /* Wrapped diffuse: (NL + w) / (1+w)^2, matching Workbench. */
    for (int i = 0; i < 4; i++) {
      float NL = dot(dirs[i].xyz, N);
      float w = cols[i].w;
      float w1 = w + 1.0f;
      diffuse_light += cols[i].rgb * clamp((NL + w) / (w1 * w1), 0.0f, 1.0f);
    }

    float3 spec_col = float3(0.0f);
    if (use_specular != 0) {
      /* Blinn-Phong specular, matching Workbench's blinn_specular(). */
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

        /* Env specular (simulates environment reflection). */
        float wrap_NL = dot(L, R);
        float w_s = mix(w, 1.0f, roughness);
        float w_s1 = w_s + 1.0f;
        float spec_env = clamp((wrap_NL + w_s) / (w_s1 * w_s1), 0.0f, 1.0f);

        specular_light += specs[i].rgb * mix(spec, spec_env, w * w);
      }

      /* BRDF fresnel approximation. */
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
    /* MATCAP: compute matcap UV from view-space normal.
     * Matches workbench_matcap_lib.glsl: matcap_uv_compute(I, N, flipped).
     * I = incident vector (surface-to-camera), handles ortho vs perspective. */
    float3 view_normal = normalize((vm.viewmat * float4(normal, 0.0f)).xyz);
    float3 view_pos = (vm.viewmat * float4(hit_pos, 1.0f)).xyz;
    float3 I = drw_view_incident_vector(view_pos);

    /* Build orthonormal basis in screen space. */
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

  /* ---- 9. Write depth using Blender's canonical world-to-screen transform ---- */
  gl_FragDepth = drw_point_world_to_screen(hit_pos).z;
}
