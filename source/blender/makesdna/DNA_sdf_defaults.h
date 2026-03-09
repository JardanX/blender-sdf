/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

/* clang-format off */

/* -------------------------------------------------------------------- */
/** \name SDF Struct
 * \{ */

#define _DNA_DEFAULT_SDF \
  { \
    .sdf_type = 0, \
    .size = {1.0f, 1.0f, 1.0f}, \
    .bevel = 0.0f, \
    .color = {0.8f, 0.8f, 0.8f, 1.0f}, \
    .blend = 0.0f, \
    .blend_type = 0, \
    .csg_operation = 0, \
    .shell_distance = 0.0f, \
  }

/** \} */

/* clang-format on */
