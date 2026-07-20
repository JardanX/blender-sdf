/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

/* clang-format off */

/* -------------------------------------------------------------------- */
/** \name SDFGroup Struct
 * \{ */

#define _DNA_DEFAULT_SDFGroup \
  { \
    .csg_operation = 0, /* SDF_CSG_UNION */ \
    .blend_type = 1, /* SDF_BLEND_SMOOTH */ \
    .blend = 0.1f, \
    .shell_distance = 0.2f, \
    .shell_mode = 0, \
    .shell_op = 0, \
    .shell_blend_top = 0.1f, \
    .shell_blend_bottom = 0.1f, \
    .chamfer_k2 = 0.001f, \
    .chamfer_k3 = 0.001f, \
    .chamfer_k4 = 0.01f, \
    .chamfer_k5 = 0.01f, \
    .flip_blend = 0, \
    .flip_blend_end = 0, \
    .color = {1.0f, 1.0f, 1.0f, 1.0f}, \
    .totmember = 0, \
    .group_order = 0, \
    .modifiers = {NULL, NULL}, \
    .totmodifier = 0, \
  }

/** \} */

/* clang-format on */
