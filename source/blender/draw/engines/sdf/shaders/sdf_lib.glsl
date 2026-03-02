/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF primitives library.
 * Cube-only for the initial implementation.
 */

#pragma once

#include "gpu_shader_compat.hh"

/**
 * Signed distance to an axis-aligned box centered at origin.
 * \param p: sample point in local space.
 * \param b: box half-extents.
 * \return signed distance (negative inside).
 */
float sdBox(float3 p, float3 b)
{
  float3 q = abs(p) - b;
  return length(max(q, float3(0.0f))) + min(max(q.x, max(q.y, q.z)), 0.0f);
}

/**
 * Smooth union (Inigo Quilez pattern).
 * Blends two SDF distances with smooth radius k.
 * \param d1: first distance.
 * \param d2: second distance.
 * \param k: blend radius (0 = hard union).
 * \return blended distance.
 */
float opSmoothUnion(float d1, float d2, float k)
{
  float h = clamp(0.5f + 0.5f * (d2 - d1) / k, 0.0f, 1.0f);
  return mix(d2, d1, h) - k * h * (1.0f - h);
}
