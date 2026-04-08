/* SDF group point: smooth filled circle with outline. */

#include "infos/overlay_extra_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(overlay_sdf_group_point_base)

void main()
{
  float2 uv = gl_PointCoord * 2.0f - 1.0f;
  float dist = length(uv);

  float aa = fwidth(dist);
  float fill_alpha = 1.0f - smoothstep(1.0f - aa * 2.0f, 1.0f, dist);

  float outline_inner = 0.7f;
  float outline_mix = smoothstep(outline_inner - aa, outline_inner + aa, dist);

  float3 fill = fill_color.rgb;
  float3 outline = outline_color.rgb;
  float3 color = mix(fill, outline, outline_mix);

  frag_color = float4(color, fill_alpha * fill_color.a);

  if (fill_alpha < 0.01f) {
    discard;
  }
}
