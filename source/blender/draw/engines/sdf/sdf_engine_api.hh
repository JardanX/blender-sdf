/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector_types.hh"

namespace blender {

struct Object;

namespace draw::sdf {

bool sdf_object_at_pixel(int2 pixel,
                         int2 viewport_size,
                         const void *viewport_key,
                         const Object **r_object,
                         float *r_depth);

}  // namespace draw::sdf
}  // namespace blender
