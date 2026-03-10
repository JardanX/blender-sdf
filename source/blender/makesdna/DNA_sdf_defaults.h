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
    .blend = 0.1f, \
    .blend_type = 1, /* SDF_BLEND_SMOOTH */ \
    .csg_operation = 0, \
    .shell_distance = 0.2f, \
    .box_corners = {0.0f, 0.0f, 0.0f, 0.0f}, \
    .box_edge_top = 0.0f, \
    .box_edge_bottom = 0.0f, \
    .box_taper = 0.0f, \
    .box_corner_mode = 0, \
    .box_edge_mode = 0, \
    .ngon_sides = 6, \
    .ngon_corner = 0.0f, \
    .ngon_edge_top = 0.0f, \
    .ngon_edge_bottom = 0.0f, \
    .ngon_taper = 0.0f, \
    .ngon_edge_mode = 0, \
  }

/** \} */

/* clang-format on */
