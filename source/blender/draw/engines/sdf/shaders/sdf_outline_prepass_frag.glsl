/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/sdf_shader_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(sdf_outline_prepass)

void main()
{
  float2 sdf_uv = screen_uv * uv_scale;

  float sdf_depth = texture(sdf_depth_tx, sdf_uv).r;
  if (sdf_depth <= 0.0f || sdf_depth >= 1.0f) {
    discard;
    return;
  }

  float obj_id_f = texture(sdf_gbuf_color_tx, sdf_uv).a;
  int obj_id = int(obj_id_f + 0.5f);
  if (obj_id < 0 || obj_id >= outline_id_count) {
    discard;
    return;
  }

  /* Stabilize boundary pixels between adjacent SDF objects. At a coincident
   * silhouette (selected SDF butted against an unselected one, two SDFs
   * sharing an edge in 3D) the per-pixel CSG winner in sdf_trace_comp flips
   * frame to frame due to FP tie in the < / > comparisons, making the
   * prepass color_id flicker between orange (1/3) and black (2).
   *
   * Sample the 4-connected neighbors' g-buffer obj_id; any neighbor with a
   * different obj_id but an equal sdf_depth (within eps) is on the same
   * surface — i.e. a coincident silhouette. Among those candidates determin-
   * istically pick the lowest obj_id as this pixel's winner. The boundary
   * line resolves to a consistent location regardless of trace FP noise, so
   * the detect shader sees a stable edge and renders the selected and
   * always-on outlines side-by-side (Blender multi-object outline look).
   *
   * eps is sized to float32 compute convergence noise; distinct surfaces at
   * typical scene scale differ by >> 1e-4 in clip-space depth. */
  float2 gbuf_texel = 1.0f / float2(textureSize(sdf_gbuf_color_tx, 0));
  const float coincident_eps = 1.0e-4f;
  float2 offsets[4] = {
      float2(-1.0f, 0.0f), float2(1.0f, 0.0f),
      float2(0.0f, -1.0f), float2(0.0f, 1.0f),
  };
  int stable_obj_id = obj_id;
  for (int i = 0; i < 4; i++) {
    float2 nuv = sdf_uv + offsets[i] * gbuf_texel;
    float nd = texture(sdf_depth_tx, nuv).r;
    if (nd <= 0.0f || nd >= 1.0f) { continue; }
    if (abs(nd - sdf_depth) > coincident_eps) { continue; }
    int nid = int(texture(sdf_gbuf_color_tx, nuv).a + 0.5f);
    if (nid < 0 || nid >= outline_id_count) { continue; }
    if (nid < stable_obj_id) { stable_obj_id = nid; }
  }
  obj_id = stable_obj_id;

  uint oid = outline_ids[obj_id];
  if (oid == 0u) {
    discard;
    return;
  }

  /* Discard if SDF is behind the scene surface. */
  float scene_d = texture(scene_depth_tx, screen_uv).r;
  float eps = max(3.0f / 8388608.0f, scene_d * 2.0e-6f);
  if (scene_d > 0.0f && scene_d < 1.0f && sdf_depth > scene_d + eps) {
    discard;
    return;
  }

  out_object_id = oid;

  /* Selection outlines pitch depth closer, always-on unselected pitch further,
   * so depth-tested against any coincident mesh outline drawn earlier in the
   * prepass, the selection deterministically wins. Constant 1e-6 offset only;
   * no screen-space or camera-dependent bias. Detect-shader occlusion epsilon
   * (~8e-6) dwarfs this so occlusion tests are unaffected. */
  uint color_id = oid >> 14u;
  if (color_id == 1u || color_id == 3u) {
    gl_FragDepth = max(sdf_depth - 1.0e-6f, 0.0f);
  }
  else {
    gl_FragDepth = min(sdf_depth + 1.0e-6f, 1.0f);
  }
}
