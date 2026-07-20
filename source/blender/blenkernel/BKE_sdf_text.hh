/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 * \brief Text support for the SDF text primitive (#SDF_TYPE_TEXT).
 *
 * The text is stored on the #SDF data-block (mirroring the font subset of
 * #Curve) so the analytic SDF can reuse Blender's text layout and the
 * #EditFont editing machinery (TAB to type, like a regular text object).
 */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

namespace blender {

struct Curve;
struct Object;
struct SDF;

/** One closed glyph-outline loop: per-edge start points with optional
 * quadratic Bezier control points. Edge `i` goes from `points[i]` to
 * `points[(i + 1) % size]` (straight) or describes a quadratic arc with
 * control `ctrls[i]` (when `is_arc[i]`). */
struct SDFTextContour {
  Vector<float2> points;
  Vector<float2> ctrls;
  Vector<char> is_arc;
  /** Per vertex: true for original font on-curve points (corner-roundable);
   * false for points inserted by curve fitting (always smooth). */
  Vector<char> is_knot;
};

/** Flattened 2D glyph-outline contours of an SDF text object, in object-local
 * space (font units scaled by #SDF::text_size, before object scale). */
struct SDFTextContours {
  /** Closed loops; the winding distinguishes outer contours from holes. */
  Vector<SDFTextContour> contours;
  float2 bounds_min;
  float2 bounds_max;
};

/**
 * Tessellate the SDF text into closed 2D contours (glyph outlines as straight
 * and exact quadratic-arc edges). Uses the live edit-mode text buffer when the
 * object is being edited. Returns the runtime-cached result (rebuilt only when
 * the text state changes), or nullptr when there is no text to render.
 */
const SDFTextContours *BKE_sdf_text_get_contours(Object *ob);

/** Bounds of the 2D text layout (without the extrusion depth/bevels). */
bool BKE_sdf_text_bounds(Object *ob, float2 &r_min, float2 &r_max);

/** Replace the UTF-8 text (also resets per-character info). */
void BKE_sdf_text_set(SDF *sdf, const char *str);

/** SDF-only variant (builds a temporary object wrapper internally). */
bool BKE_sdf_text_bounds_sdf(const SDF *sdf, float2 &r_min, float2 &r_max);

/**
 * Runtime edit-mode text store: a #Curve (never linked in #Main) that mirrors
 * the SDF text state so #EditFont operators can edit it unchanged.
 * Owned by the original (non-evaluated) SDF's runtime; evaluated copies share
 * the pointer (non-owning), like #Curve::editfont.
 */
Curve *BKE_sdf_text_edit_curve_get(const SDF *sdf);

/** Create the runtime edit curve from the persistent SDF text state. */
Curve *BKE_sdf_text_edit_curve_enter(SDF *sdf);

/** Copy the edited text from the runtime edit curve into the persistent SDF
 * text state (does not free the edit curve). */
void BKE_sdf_text_edit_curve_store(SDF *sdf);

/** Store the edited text back into the SDF data-block and free the edit curve. */
void BKE_sdf_text_edit_curve_exit(SDF *sdf);

/** Free the edit curve without storing (ID free path). */
void BKE_sdf_text_edit_curve_free(SDF *sdf);

}  // namespace blender
