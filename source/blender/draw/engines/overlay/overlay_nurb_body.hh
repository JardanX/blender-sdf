/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "DNA_mesh_types.h"
#include "DNA_nurb_body_types.h"
#include "DNA_object_types.h"

#include "BLI_math_base.h"
#include "BLI_math_matrix.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector.h"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "BKE_mesh.hh"
#include "BKE_nurb_body.hh"
#include "BKE_object.hh"

#include "GPU_immediate.hh"
#include "GPU_matrix.hh"
#include "GPU_state.hh"

#include "overlay_base.hh"

namespace blender::draw::overlay {

class NurbBodies : Overlay {
 private:
  struct Entry {
    const Object *object;
    bool object_mode;
    bool selected;
  };

  Vector<Entry> entries_;

  static bool edge_index_in_mask(const uint64_t mask, const int edge_index)
  {
    return edge_index >= 0 && edge_index < 64 && (mask & (uint64_t(1) << uint(edge_index)));
  }

  static void draw_lines(const Vector<float3> &lines, const float4 &color, const float width)
  {
    if (lines.is_empty()) {
      return;
    }

    GPU_line_width(width);
    uint pos_attr = GPU_vertformat_attr_add_legacy(
        immVertexFormat(), "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
    immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
    immUniformColor4fv(&color.x);
    immBegin(GPU_PRIM_LINES, lines.size());
    for (const float3 &co : lines) {
      immVertex3fv(pos_attr, co);
    }
    immEnd();
    immUnbindProgram();
  }

  static void append_mesh_silhouette_lines(const Object &object,
                                           const Mesh &mesh,
                                           const float3 &view_forward,
                                           Vector<float3> &r_lines)
  {
    const Span<float3> positions = mesh.vert_positions();
    const Span<int2> edges = mesh.edges();
    const Span<int> corner_edges = mesh.corner_edges();
    const OffsetIndices<int> faces = mesh.faces();
    const Span<float3> face_normals = mesh.face_normals();

    if (positions.is_empty() || edges.is_empty() || faces.is_empty() || corner_edges.is_empty()) {
      return;
    }

    Vector<int2> edge_faces(edges.size(), int2(-1, -1));
    Vector<int8_t> face_side(faces.size(), 0);

    for (const int face_i : faces.index_range()) {
      float normal[3] = {face_normals[face_i].x, face_normals[face_i].y, face_normals[face_i].z};
      mul_mat3_m4_v3(object.object_to_world().ptr(), normal);
      normalize_v3(normal);
      face_side[face_i] = dot_v3v3(normal, view_forward) >= 0.0f ? int8_t(1) : int8_t(-1);

      for (const int corner_i : faces[face_i]) {
        const int edge_i = corner_edges[corner_i];
        if (!edges.index_range().contains(edge_i)) {
          continue;
        }
        if (edge_faces[edge_i].x == -1) {
          edge_faces[edge_i].x = face_i;
        }
        else if (edge_faces[edge_i].y == -1) {
          edge_faces[edge_i].y = face_i;
        }
      }
    }

    for (const int edge_i : edges.index_range()) {
      const int face_a = edge_faces[edge_i].x;
      const int face_b = edge_faces[edge_i].y;
      const bool boundary_edge = face_a == -1 || face_b == -1;
      const bool silhouette_edge = !boundary_edge && face_side[face_a] != face_side[face_b];
      if (!boundary_edge && !silhouette_edge) {
        continue;
      }

      const int2 edge = edges[edge_i];
      if (!positions.index_range().contains(edge.x) || !positions.index_range().contains(edge.y)) {
        continue;
      }

      float world[3];
      mul_v3_m4v3(world, object.object_to_world().ptr(), positions[edge.x]);
      r_lines.append(float3(world[0], world[1], world[2]));
      mul_v3_m4v3(world, object.object_to_world().ptr(), positions[edge.y]);
      r_lines.append(float3(world[0], world[1], world[2]));
    }
  }

 public:
  void begin_sync(Resources & /*res*/, const State &state) final
  {
    enabled_ = state.is_space_v3d() && !state.hide_overlays;
    entries_.clear();
  }

  void object_sync(Manager & /*manager*/,
                   const ObjectRef &ob_ref,
                   Resources & /*res*/,
                   const State &state) final
  {
    if (!enabled_ || ob_ref.object->type != OB_NURB_BODY || ob_ref.object->data == nullptr) {
      return;
    }

    entries_.append({ob_ref.object,
                     state.object_mode == OB_MODE_OBJECT,
                     (ob_ref.object->base_flag & BASE_SELECTED) != 0});
  }

  void draw_line(Framebuffer &framebuffer, Manager & /*manager*/, View &view) final
  {
    if (!enabled_ || entries_.is_empty()) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_line_smooth(true);
    GPU_matrix_push_projection();
    GPU_polygon_offset(1.0f, 2.0f);

    Vector<float3> silhouette_lines;
    Vector<float3> selected_silhouette_lines;
    Vector<float3> top_hovered_lines;
    Vector<float3> top_selected_lines;

    for (const Entry &entry : entries_) {
      const NurbBody *body = reinterpret_cast<const NurbBody *>(entry.object->data);
      Vector<NurbBodyEdgePolyline> polylines;
      Vector<float3> object_silhouette_lines;
      Vector<float3> normal_lines;
      Vector<float3> hovered_lines;
      Vector<float3> selected_lines;

      if (entry.object_mode) {
        if (const Mesh *mesh = BKE_object_get_evaluated_mesh_no_subsurf_unchecked(entry.object)) {
          append_mesh_silhouette_lines(
              *entry.object, *mesh, view.forward(), object_silhouette_lines);
          if (entry.selected) {
            selected_silhouette_lines.extend(object_silhouette_lines);
          }
          else {
            silhouette_lines.extend(object_silhouette_lines);
          }
        }
      }

      BKE_nurb_body_boolean_edge_polylines(entry.object, polylines, 64);
      for (const NurbBodyEdgePolyline &polyline : polylines) {
        if ((polyline.flag & NURB_BODY_EDGE_POLYLINE_SURFACE) == 0) {
          continue;
        }
        const NurbBodyBooleanOp *op = polyline.op;
        const bool body_edge = (polyline.flag & NURB_BODY_EDGE_POLYLINE_BODY) != 0;
        const bool selected = body_edge ? edge_index_in_mask(body->selected_edges,
                                                             polyline.edge_index) :
                              (op != nullptr && polyline.edge_index >= 0 &&
                               (op->flag & NURB_BODY_BOOLEAN_OP_SELECTED) != 0 &&
                               edge_index_in_mask(op->selected_edges, polyline.edge_index));
        const bool hovered = body_edge ? (body->hovered_edge == polyline.edge_index) :
                             (op != nullptr && polyline.edge_index >= 0 &&
                              (op->flag & NURB_BODY_BOOLEAN_OP_HOVERED) != 0 &&
                              op->hovered_edge == polyline.edge_index);
        if (polyline.points.size() < 2) {
          continue;
        }

        Vector<float3> lines;
        for (int i = 1; i < polyline.points.size(); i++) {
          float world[3];
          mul_v3_m4v3(world, entry.object->object_to_world().ptr(), polyline.points[i - 1]);
          lines.append(float3(world[0], world[1], world[2]));
          mul_v3_m4v3(world, entry.object->object_to_world().ptr(), polyline.points[i]);
          lines.append(float3(world[0], world[1], world[2]));
        }
        if (lines.is_empty()) {
          continue;
        }

        if (selected) {
          selected_lines.extend(lines);
          top_selected_lines.extend(lines);
        }
        else if (hovered) {
          hovered_lines.extend(lines);
          top_hovered_lines.extend(lines);
        }
        else {
          normal_lines.extend(lines);
        }
      }

      const float line_width = 1.65f;
      draw_lines(normal_lines, float4(0.0f, 0.0f, 0.0f, 0.9f), line_width);
      if (!hovered_lines.is_empty()) {
        draw_lines(hovered_lines, float4(1.0f, 1.0f, 1.0f, 1.0f), line_width);
      }
      if (!selected_lines.is_empty()) {
        draw_lines(selected_lines, float4(1.0f, 0.62f, 0.0f, 1.0f), line_width);
      }
    }

    GPU_matrix_pop_projection();
    GPU_depth_test(GPU_DEPTH_NONE);
    draw_lines(silhouette_lines, float4(0.0f, 0.0f, 0.0f, 1.0f), 2.35f);
    draw_lines(selected_silhouette_lines, float4(1.0f, 0.62f, 0.0f, 1.0f), 2.35f);
    draw_lines(top_hovered_lines, float4(1.0f, 1.0f, 1.0f, 1.0f), 1.65f);
    draw_lines(top_selected_lines, float4(1.0f, 0.62f, 0.0f, 1.0f), 1.65f);
    GPU_line_smooth(false);
    GPU_blend(GPU_BLEND_NONE);
    GPU_depth_mask(true);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
  }
};

}  // namespace blender::draw::overlay
