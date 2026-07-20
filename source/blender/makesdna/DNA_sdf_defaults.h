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
    .clearance = 0.0f, \
    .color_blend = 0.1f, \
    .color_blend_type = 0, \
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
    .ngon_star = 0.0f, \
    .totpolygon = 0, \
    .polygon_edge_top = 0.0f, \
    .polygon_edge_bottom = 0.0f, \
    .polygon_taper = 0.0f, \
    .polygon_edge_mode = 0, \
    .polygon_is_line = 0, \
    .polygon_line_thickness = 0.1f, \
    .torus_angle = ((float)M_PI * 2.0f), \
    .cylinder_edge_top = 0.0f, \
    .cylinder_edge_bottom = 0.0f, \
    .cylinder_taper = 0.0f, \
    .cylinder_edge_mode = 0, \
    .cone_edge_top = 0.0f, \
    .cone_edge_bottom = 0.0f, \
    .text_len = 0, \
    .text_len_char32 = 0, \
    .text_pos = 0, \
    .text_selstart = 0, \
    .text_selend = 0, \
    .text_size = 1.0f, \
    .text_spacing = 1.0f, \
    .text_linedist = 1.0f, \
    .text_shear = 0.0f, \
    .text_xof = 0.0f, \
    .text_yof = 0.0f, \
    .text_thickness = 0.0f, \
    .text_corner = 0.0f, \
    .text_align_x = 1, /* CU_ALIGN_X_MIDDLE */ \
    .text_align_y = 2, /* CU_ALIGN_Y_CENTER */ \
    .sdf_index = 0, \
  }

/** \} */

/* clang-format on */
