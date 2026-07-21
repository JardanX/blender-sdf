/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * Text layout/tessellation and edit-mode text store for the SDF text
 * primitive (#SDF_TYPE_TEXT). The glyph outlines come from the regular
 * Blender text layout (#BKE_vfont_to_curve_nubase), so fonts, spacing,
 * alignment and edit-mode typing behave exactly like a text object.
 */

#include "MEM_guardedalloc.h"

#include "BLI_listbase.h"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_string_utf8.h"

#include "DNA_curve_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"
#include "DNA_vfont_types.h"

#include "BKE_curve.hh"
#include "BKE_lib_id.hh"
#include "BKE_sdf.hh"
#include "BKE_sdf_text.hh"
#include "BKE_vfont.hh"

namespace blender {

/* -------------------------------------------------------------------- */
/** \name Curve <-> SDF text state
 * \{ */

/** Fill a (non-owned) Curve with the persistent SDF text state.
 * Pointers are shared, nothing is freed through the curve. */
static void sdf_text_curve_from_dna(Curve *cu, const SDF *sdf)
{
  /* Not a real Main ID, but keep the type code valid for id_cast checks. */
  cu->id.name[0] = 'C';
  cu->id.name[1] = 'U';
  cu->ob_type = OB_FONT;
  cu->str = sdf->text;
  cu->len = sdf->text_len;
  cu->len_char32 = sdf->text_len_char32;
  cu->strinfo = sdf->text_strinfo;
  cu->vfont = sdf->text_font;
  cu->fsize = sdf->text_size;
  cu->spacing = sdf->text_spacing;
  cu->linedist = sdf->text_linedist;
  cu->shear = sdf->text_shear;
  cu->xof = sdf->text_xof;
  cu->yof = sdf->text_yof;
  cu->spacemode = sdf->text_align_x;
  cu->align_y = sdf->text_align_y;
  cu->pos = sdf->text_pos;
  cu->selstart = sdf->text_selstart;
  cu->selend = sdf->text_selend;
  cu->curinfo = sdf->text_curinfo;
}

static void sdf_text_curve_store_dna(SDF *sdf, const Curve *cu)
{
  if (sdf->text) {
    MEM_delete(sdf->text);
  }
  sdf->text = cu->str ? MEM_new_array_uninitialized<char>(cu->len + 1, "sdf text") : nullptr;
  if (sdf->text) {
    memcpy(sdf->text, cu->str, cu->len + 1);
  }
  sdf->text_len = cu->len;
  sdf->text_len_char32 = cu->len_char32;

  if (sdf->text_strinfo) {
    MEM_delete(sdf->text_strinfo);
  }
  sdf->text_strinfo = cu->strinfo ?
                          MEM_new_array<CharInfo>(cu->len_char32 + 1, "sdf text strinfo") :
                          nullptr;
  if (sdf->text_strinfo) {
    memcpy(sdf->text_strinfo, cu->strinfo, (cu->len_char32 + 1) * sizeof(CharInfo));
  }

  sdf->text_pos = cu->pos;
  sdf->text_selstart = cu->selstart;
  sdf->text_selend = cu->selend;
  sdf->text_curinfo = cu->curinfo;

  sdf->text_size = cu->fsize;
  sdf->text_spacing = cu->spacing;
  sdf->text_linedist = cu->linedist;
  sdf->text_shear = cu->shear;
  sdf->text_xof = cu->xof;
  sdf->text_yof = cu->yof;
  sdf->text_align_x = cu->spacemode;
  sdf->text_align_y = cu->align_y;

  if (sdf->text_font != cu->vfont) {
    if (cu->vfont) {
      id_us_plus(&cu->vfont->id);
    }
    if (sdf->text_font) {
      id_us_min(&sdf->text_font->id);
    }
    sdf->text_font = cu->vfont;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Glyph outline tessellation
 * \{ */

static void contour_add_edge(SDFTextContour &ct,
                             const float2 &start,
                             const float2 &ctrl,
                             const bool is_arc,
                             const bool knot)
{
  ct.points.append(start);
  ct.ctrls.append(ctrl);
  ct.is_arc.append(char(is_arc));
  ct.is_knot.append(char(knot));
}

/** Exact quadratic control of a degree-elevated cubic (TrueType conics are
 * converted to cubics this way, see blf_glyph_curves.cc), or false when the
 * cubic is a true cubic curve. */
static bool cubic_as_quadratic(const float2 &p0,
                               const float2 &p1,
                               const float2 &p2,
                               const float2 &p3,
                               const float elev_tol_sq,
                               float2 &r_ctrl)
{
  const float2 q1 = (p1 * 3.0f - p0) * 0.5f;
  const float2 q2 = (p2 * 3.0f - p3) * 0.5f;
  if (math::distance_squared(q1, q2) > elev_tol_sq) {
    return false;
  }
  r_ctrl = (q1 + q2) * 0.5f;
  return true;
}

static float2 quadratic_eval(const float2 &p0, const float2 &q, const float2 &p3, const float t)
{
  const float mt = 1.0f - t;
  return p0 * (mt * mt) + q * (2.0f * mt * t) + p3 * (t * t);
}

static float2 cubic_eval(const float2 &p0,
                         const float2 &p1,
                         const float2 &p2,
                         const float2 &p3,
                         const float t)
{
  const float mt = 1.0f - t;
  return p0 * (mt * mt * mt) + p1 * (3.0f * mt * mt * t) + p2 * (3.0f * mt * t * t) +
         p3 * (t * t * t);
}

/** Emit a cubic segment as quadratic arc edges (endpoint-tangent fit with
 * midpoint error check, split recursively when the fit is poor). Straight and
 * degree-elevated segments collapse to a single exact edge. */
static void tessellate_cubic(const float2 &p0,
                             const float2 &p1,
                             const float2 &p2,
                             const float2 &p3,
                             const float tol_sq,
                             const float elev_tol_sq,
                             const int depth,
                             SDFTextContour &ct,
                             const bool knot)
{
  const float2 chord = p3 - p0;
  const float chord_len_sq = math::length_squared(chord);

  /* Degenerate / straight segment: single line edge. */
  if (chord_len_sq < 1e-14f) {
    return;
  }
  {
    const float2 n(-chord.y, chord.x);
    const float d1 = math::dot(n, p1 - p0);
    const float d2 = math::dot(n, p2 - p0);
    const float flat_sq = math::max(d1 * d1, d2 * d2) / chord_len_sq;
    if (flat_sq <= tol_sq * 0.01f) {
      contour_add_edge(ct, p0, p0, false, knot);
      return;
    }
  }

  float2 q;
  if (cubic_as_quadratic(p0, p1, p2, p3, elev_tol_sq, q)) {
    contour_add_edge(ct, p0, q, true, knot);
    return;
  }

  /* Endpoint-tangent fit: the quadratic control lies on both endpoint
   * tangent lines. */
  if (depth < 6) {
    const float2 ta = p1 - p0;
    const float2 tb = p2 - p3;
    const float denom = ta.x * tb.y - ta.y * tb.x;
    if (fabsf(denom) > 1e-12f) {
      const float2 d = p3 - p0;
      const float s = (d.x * tb.y - d.y * tb.x) / denom;
      const float2 qf = p0 + ta * s;
      float err_sq = 0.0f;
      for (const float t : {0.25f, 0.5f, 0.75f}) {
        err_sq = math::max(err_sq,
                           math::distance_squared(cubic_eval(p0, p1, p2, p3, t),
                                                  quadratic_eval(p0, qf, p3, t)));
      }
      if (err_sq <= tol_sq) {
        contour_add_edge(ct, p0, qf, true, knot);
        return;
      }
    }
  }

  if (depth >= 6) {
    /* Should not happen for font curves; keep it simple and correct. */
    contour_add_edge(ct, p0, p0, false, knot);
    return;
  }

  /* Split at t = 0.5 and fit both halves. */
  const float2 p01 = (p0 + p1) * 0.5f;
  const float2 p12 = (p1 + p2) * 0.5f;
  const float2 p23 = (p2 + p3) * 0.5f;
  const float2 p012 = (p01 + p12) * 0.5f;
  const float2 p123 = (p12 + p23) * 0.5f;
  const float2 p0123 = (p012 + p123) * 0.5f;
  tessellate_cubic(p0, p01, p012, p0123, tol_sq, elev_tol_sq, depth + 1, ct, knot);
  tessellate_cubic(p0123, p123, p23, p3, tol_sq, elev_tol_sq, depth + 1, ct, false);
}

static void contour_update_bounds(const SDFTextContour &ct,
                                  float2 &r_min,
                                  float2 &r_max)
{
  const int n = int(ct.points.size());
  for (int i = 0; i < n; i++) {
    r_min = math::min(r_min, ct.points[i]);
    r_max = math::max(r_max, ct.points[i]);
    if (ct.is_arc[i]) {
      /* Exact quadratic extrema per axis. */
      const float2 &p0 = ct.points[i];
      const float2 &q = ct.ctrls[i];
      const float2 &p3 = ct.points[(i + 1) % n];
      for (int axis = 0; axis < 2; axis++) {
        const float denom = p0[axis] - 2.0f * q[axis] + p3[axis];
        if (fabsf(denom) > 1e-12f) {
          const float t = (p0[axis] - q[axis]) / denom;
          if (t > 0.0f && t < 1.0f) {
            const float2 p = quadratic_eval(p0, q, p3, t);
            r_min = math::min(r_min, p);
            r_max = math::max(r_max, p);
          }
        }
      }
    }
  }
}

static uint64_t sdf_text_state_hash(const SDF *sdf, const Curve *cu_edit)
{
  uint64_t h = 0xcbf29ce484222325ULL;
  auto mix = [&h](const void *data, size_t len) {
    const unsigned char *p = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < len; i++) {
      h ^= p[i];
      h *= 0x100000001b3ULL;
    }
  };
  if (cu_edit && cu_edit->editfont) {
    const EditFont *ef = cu_edit->editfont;
    mix(ef->textbuf, size_t(ef->len) * sizeof(char32_t));
    mix(ef->textbufinfo, size_t(ef->len) * sizeof(CharInfo));
    mix(&cu_edit->fsize, sizeof(float));
    mix(&cu_edit->spacing, sizeof(float));
    mix(&cu_edit->linedist, sizeof(float));
    mix(&cu_edit->shear, sizeof(float));
    mix(&cu_edit->xof, sizeof(float));
    mix(&cu_edit->yof, sizeof(float));
    mix(&cu_edit->spacemode, sizeof(char));
    mix(&cu_edit->align_y, sizeof(char));
  }
  else {
    if (sdf->text) {
      mix(sdf->text, size_t(sdf->text_len) + 1);
    }
    if (sdf->text_strinfo) {
      mix(sdf->text_strinfo, size_t(sdf->text_len_char32 + 1) * sizeof(CharInfo));
    }
    mix(&sdf->text_size, sizeof(float));
    mix(&sdf->text_spacing, sizeof(float));
    mix(&sdf->text_linedist, sizeof(float));
    mix(&sdf->text_shear, sizeof(float));
    mix(&sdf->text_xof, sizeof(float));
    mix(&sdf->text_yof, sizeof(float));
    mix(&sdf->text_align_x, sizeof(char));
    mix(&sdf->text_align_y, sizeof(char));
  }
  const VFont *font = cu_edit ? cu_edit->vfont : sdf->text_font;
  mix(&font, sizeof(font));
  return h;
}

/** \} */

const SDFTextContours *BKE_sdf_text_get_contours(Object *ob)
{
  const SDF *sdf = id_cast<const SDF *>(ob->data);
  if (sdf->sdf_type != SDF_TYPE_TEXT || sdf->runtime == nullptr) {
    return nullptr;
  }

  /* The runtime edit curve holds the live text while editing (edit-mode
   * typing only lands in the DNA state on edit-mode exit). */
  Curve *cu_edit = sdf->runtime->text_curve;

  bke::SDFRuntime *runtime = sdf->runtime;
  const uint64_t hash = sdf_text_state_hash(sdf, cu_edit);
  if (runtime->text_cache && runtime->text_cache_hash == hash) {
    return runtime->text_cache;
  }

  Curve cu_tmp = {};
  TextBox *tb = nullptr;
  CharInfo *strinfo_tmp = nullptr;
  Curve *cu = cu_edit;
  if (cu == nullptr) {
    if (sdf->text == nullptr || sdf->text[0] == '\0') {
      delete runtime->text_cache;
      runtime->text_cache = nullptr;
      runtime->text_cache_hash = hash;
      return nullptr;
    }
    cu = &cu_tmp;
    sdf_text_curve_from_dna(cu, sdf);

    /* #vfont_to_curve requires text boxes and per-char info to be non-null. */
    tb = MEM_new_array<TextBox>(MAXTEXTBOX, "sdf text tb");
    cu_tmp.tb = tb;
    cu_tmp.totbox = 1;
    cu_tmp.actbox = 1;
    if (cu_tmp.strinfo == nullptr) {
      strinfo_tmp = MEM_new_array<CharInfo>(cu_tmp.len_char32 + 1, "sdf text strinfo");
      cu_tmp.strinfo = strinfo_tmp;
    }
  }

  Object ob_tmp = dna::shallow_copy(*ob);
  ob_tmp.data = reinterpret_cast<ID *>(cu);

  ListBaseT<Nurb> nubase = {nullptr, nullptr};
  const bool ok = BKE_vfont_to_curve_nubase(&ob_tmp, FO_EDIT, &nubase);

  if (tb) {
    MEM_delete(tb);
  }
  if (strinfo_tmp) {
    MEM_delete(strinfo_tmp);
  }
  if (!ok) {
    BKE_nurbList_free(&nubase);
    return nullptr;
  }

  const float fsize = cu->fsize;
  /* Fit tolerance: quadratic arcs are exact curves, this only bounds the
   * approximation error for true cubic (PostScript) outlines. */
  const float tol_sq = math::max(0.002f * fsize, 1e-6f) * math::max(0.002f * fsize, 1e-6f);
  const float elev_tol_sq = math::max(1e-5f * fsize, 1e-9f) * math::max(1e-5f * fsize, 1e-9f);

  SDFTextContours *result = new SDFTextContours();
  result->bounds_min = float2(FLT_MAX);
  result->bounds_max = float2(-FLT_MAX);

  for (Nurb *nu = static_cast<Nurb *>(nubase.first); nu; nu = nu->next) {
    if (nu->type != CU_BEZIER || nu->pntsu < 3 || nu->bezt == nullptr) {
      continue;
    }
    SDFTextContour ct;
    ct.points.reserve(nu->pntsu);
    for (int i = 0; i < nu->pntsu; i++) {
      const BezTriple &b0 = nu->bezt[i];
      const BezTriple &b1 = nu->bezt[(i + 1) % nu->pntsu];
      tessellate_cubic(float2(b0.vec[1]),
                       float2(b0.vec[2]),
                       float2(b1.vec[0]),
                       float2(b1.vec[1]),
                       tol_sq,
                       elev_tol_sq,
                       0,
                       ct,
                       true);
    }
    /* Drop zero-length edges (duplicated on-curve points). */
    for (int i = 0; i < ct.points.size(); i++) {
      const float2 &a = ct.points[i];
      const float2 &b = ct.points[(i + 1) % ct.points.size()];
      if (math::distance_squared(a, b) < 1e-14f) {
        ct.points.remove(i);
        ct.ctrls.remove(i);
        ct.is_arc.remove(i);
        ct.is_knot.remove(i);
        i--;
      }
    }
    if (ct.points.size() >= 3) {
      contour_update_bounds(ct, result->bounds_min, result->bounds_max);
      result->contours.append(std::move(ct));
    }
  }

  BKE_nurbList_free(&nubase);

  if (result->contours.is_empty()) {
    delete result;
    delete runtime->text_cache;
    runtime->text_cache = nullptr;
    runtime->text_cache_hash = hash;
    return nullptr;
  }

  delete runtime->text_cache;
  runtime->text_cache = result;
  runtime->text_cache_hash = hash;

  /* Debug helper: SDF_TEXT_DUMP=1 dumps the tessellated contours. */
  if (getenv("SDF_TEXT_DUMP")) {
    if (FILE *f = fopen("/tmp/sdf_text_contours.txt", "w")) {
      fprintf(f, "bounds %f %f %f %f\n",
              result->bounds_min.x,
              result->bounds_min.y,
              result->bounds_max.x,
              result->bounds_max.y);
      for (const SDFTextContour &ct : result->contours) {
        fprintf(f, "contour %d\n", int(ct.points.size()));
        for (int i = 0; i < ct.points.size(); i++) {
          fprintf(f,
                  "edge %.9g %.9g %.9g %.9g %d %d\n",
                  ct.points[i].x,
                  ct.points[i].y,
                  ct.ctrls[i].x,
                  ct.ctrls[i].y,
                  int(ct.is_arc[i]),
                  int(ct.is_knot[i]));
        }
      }
      fclose(f);
    }
  }
  return result;
}

bool BKE_sdf_text_bounds(Object *ob, float2 &r_min, float2 &r_max)
{
  const SDFTextContours *contours = BKE_sdf_text_get_contours(ob);
  if (contours == nullptr) {
    return false;
  }
  r_min = contours->bounds_min;
  r_max = contours->bounds_max;
  return true;
}

bool BKE_sdf_text_bounds_sdf(const SDF *sdf, float2 &r_min, float2 &r_max)
{
  if (sdf->sdf_type != SDF_TYPE_TEXT) {
    return false;
  }
  /* Read-only layout through a temporary object wrapper (not in edit-mode,
   * so no EditFont state is touched). */
  Object ob_tmp = {};
  ob_tmp.type = OB_SDF;
  ob_tmp.data = reinterpret_cast<ID *>(const_cast<SDF *>(sdf));
  return BKE_sdf_text_bounds(&ob_tmp, r_min, r_max);
}

/* -------------------------------------------------------------------- */
/** \name Edit-mode text store
 * \{ */

void BKE_sdf_text_set(SDF *sdf, const char *str)
{
  size_t len_bytes;
  const size_t len_char32 = BLI_strlen_utf8_ex(str, &len_bytes);

  if (sdf->text) {
    MEM_delete(sdf->text);
  }
  sdf->text_len = int(len_bytes);
  sdf->text_len_char32 = int(len_char32);
  sdf->text = MEM_new_array_uninitialized<char>(len_bytes + 1, "sdf text");
  memcpy(sdf->text, str, len_bytes + 1);

  if (sdf->text_strinfo) {
    MEM_delete(sdf->text_strinfo);
  }
  sdf->text_strinfo = MEM_new_array<CharInfo>(len_char32 + 1, "sdf text strinfo");
}


Curve *BKE_sdf_text_edit_curve_get(const SDF *sdf)
{
  return sdf->runtime ? sdf->runtime->text_curve : nullptr;
}

Curve *BKE_sdf_text_edit_curve_enter(SDF *sdf)
{
  if (sdf->runtime->text_curve) {
    return sdf->runtime->text_curve;
  }

  Curve *cu = MEM_new<Curve>("sdf text edit curve");
  sdf_text_curve_from_dna(cu, sdf);

  /* Own copies of the text data (edit-mode re-allocates these). */
  cu->str = MEM_new_array_uninitialized<char>((sdf->text_len + 4), "str");
  if (sdf->text) {
    memcpy(cu->str, sdf->text, sdf->text_len + 1);
  }
  else {
    cu->str[0] = '\0';
  }
  cu->strinfo = MEM_new_array<CharInfo>((cu->len_char32 + 4), "strinfo");
  if (sdf->text_strinfo) {
    memcpy(cu->strinfo, sdf->text_strinfo, cu->len_char32 * sizeof(CharInfo));
  }
  cu->tb = MEM_new_array<TextBox>(MAXTEXTBOX, "textbox");
  cu->totbox = 1;
  cu->actbox = 1;

  if (cu->vfont == nullptr) {
    cu->vfont = BKE_vfont_builtin_ensure();
  }
  if (cu->vfont) {
    id_us_plus(&cu->vfont->id);
  }

  sdf->runtime->text_curve = cu;
  sdf->runtime->text_curve_owned = true;
  return cu;
}

void BKE_sdf_text_edit_curve_store(SDF *sdf)
{
  const Curve *cu = sdf->runtime->text_curve;
  if (cu == nullptr) {
    return;
  }
  sdf_text_curve_store_dna(sdf, cu);
}

void BKE_sdf_text_edit_curve_exit(SDF *sdf)
{
  if (sdf->runtime->text_curve == nullptr) {
    return;
  }
  BLI_assert(sdf->runtime->text_curve_owned);

  BKE_sdf_text_edit_curve_store(sdf);

  BKE_sdf_text_edit_curve_free(sdf);
}

void BKE_sdf_text_edit_curve_free(SDF *sdf)
{
  Curve *cu = sdf->runtime->text_curve;
  if (cu == nullptr || !sdf->runtime->text_curve_owned) {
    sdf->runtime->text_curve = nullptr;
    return;
  }

  BKE_curve_editfont_free(cu);
  MEM_SAFE_DELETE(cu->str);
  MEM_SAFE_DELETE(cu->strinfo);
  MEM_SAFE_DELETE(cu->tb);
  if (cu->vfont) {
    id_us_min(&cu->vfont->id);
  }
  MEM_delete(cu);

  sdf->runtime->text_curve = nullptr;
  sdf->runtime->text_curve_owned = false;
}

/** \} */

}  // namespace blender
