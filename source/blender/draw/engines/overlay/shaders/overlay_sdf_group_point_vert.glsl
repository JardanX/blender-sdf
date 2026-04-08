/* SDF group point: world-space sized, always on top, smooth AA. */

#include "infos/overlay_extra_infos.hh"

VERTEX_SHADER_CREATE_INFO(overlay_sdf_group_point_base)
VERTEX_SHADER_CREATE_INFO(draw_modelmat)

#include "draw_model_lib.glsl"
#include "draw_view_clipping_lib.glsl"
#include "draw_view_lib.glsl"
#include "select_lib.glsl"

void main()
{
  select_id_set(in_select_buf[gl_VertexID]);

  float3 world_pos = drw_point_object_to_world(data_buf[gl_VertexID].pos_.xyz);
  gl_Position = drw_point_world_to_homogenous(world_pos);

  /* World radius to pixel size via perspective W */
  float world_radius = sdf_point_size;
  float pixel_radius = (world_radius * drw_view().winmat[1][1]) / gl_Position.w;
  /* winmat[1][1] maps to NDC [-1,1], half-viewport converts to pixels.
   * We approximate half-viewport as 1/theme.sizes.pixel (DPI-corrected). */
  pixel_radius = abs(pixel_radius) / (2.0f * theme.sizes.pixel);
  pixel_radius = clamp(pixel_radius, 4.0f, 64.0f);

  gl_PointSize = pixel_radius * 2.0f;

  float radius = pixel_radius;
  float outline_width = theme.sizes.pixel;
  radii[0] = radius;
  radii[1] = radius - 1.0f;
  radii[2] = radius - outline_width;
  radii[3] = radius - outline_width - 1.0f;
  radii /= gl_PointSize;

  fill_color = data_buf[gl_VertexID].color_;
  outline_color = theme.colors.outline;

#ifdef SELECT_ENABLE
  gl_PointSize = max(gl_PointSize, 6.0f);
#endif

  /* On top */
  gl_Position.z = 0.0f;

  view_clipping_distances(world_pos);
}
