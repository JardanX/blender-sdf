/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF FXAA library — self-contained FXAA 3.11 implementation.
 *
 * Ported from the MathOPS addon's postprocess/fxaa.glsl which uses
 * sRGB-based luma (LinearToSRGB + weighted green/red) for better
 * perceptual edge detection on SDF ray-march output.
 *
 * Quality preset 29: 12 search steps, low dither.
 */

#pragma once

#ifndef FXAA_QUALITY_PRESET
#  define FXAA_QUALITY_PRESET 29
#endif

#if (FXAA_QUALITY_PRESET == 29)
#  define FXAA_QUALITY_PS 12
#  define FXAA_QUALITY_P0 1.0
#  define FXAA_QUALITY_P1 1.5
#  define FXAA_QUALITY_P2 2.0
#  define FXAA_QUALITY_P3 2.0
#  define FXAA_QUALITY_P4 2.0
#  define FXAA_QUALITY_P5 2.0
#  define FXAA_QUALITY_P6 2.0
#  define FXAA_QUALITY_P7 2.0
#  define FXAA_QUALITY_P8 2.0
#  define FXAA_QUALITY_P9 4.0
#  define FXAA_QUALITY_P10 12.0
#  define FXAA_QUALITY_P11 12.0
#endif

#define FxaaSat(x) clamp(x, 0.0, 1.0)
#define FxaaTexTop(t, p) textureLod(t, p, 0.0)
#define FxaaTexOff(t, p, o, r) textureLod(t, p + (float2(o) * r), 0.0)

/* ---- Color space helpers ---- */

float3 sdf_fxaa_linear_to_srgb(float3 linear)
{
  float3 higher = float3(1.055) * pow(linear, float3(1.0 / 2.4)) - float3(0.055);
  float3 lower = linear * float3(12.92);
  return mix(lower, higher, step(float3(0.0031308), linear));
}

float3 sdf_fxaa_srgb_to_linear(float3 srgb)
{
  float3 higher = pow((srgb + float3(0.055)) / float3(1.055), float3(2.4));
  float3 lower = srgb / float3(12.92);
  return mix(lower, higher, step(float3(0.04045), srgb));
}

/* Perceptual luma via sRGB conversion (matches MathOPS addon). */
float FxaaLuma(float3 rgb)
{
  float3 srgb = sdf_fxaa_linear_to_srgb(clamp(rgb, 0.0, 1.0));
  return srgb.g * (0.587 / 0.299) + srgb.r;
}

float FxaaLuma(float4 rgba)
{
  return FxaaLuma(rgba.rgb);
}

/* ---- Main FXAA pixel shader ---- */

float4 FxaaPixelShader(float2 pos,
                     sampler2D tex,
                     float2 fxaaQualityRcpFrame,
                     float fxaaQualitySubpix,
                     float fxaaQualityEdgeThreshold,
                     float fxaaQualityEdgeThresholdMin)
{
  float2 posM;
  posM.x = pos.x;
  posM.y = pos.y;
  float4 rgbyM = FxaaTexTop(tex, posM);
  float lumaM = FxaaLuma(rgbyM);
  float lumaS = FxaaLuma(FxaaTexOff(tex, posM, int2(0, 1), fxaaQualityRcpFrame.xy));
  float lumaE = FxaaLuma(FxaaTexOff(tex, posM, int2(1, 0), fxaaQualityRcpFrame.xy));
  float lumaN = FxaaLuma(FxaaTexOff(tex, posM, int2(0, -1), fxaaQualityRcpFrame.xy));
  float lumaW = FxaaLuma(FxaaTexOff(tex, posM, int2(-1, 0), fxaaQualityRcpFrame.xy));

  float maxSM = max(lumaS, lumaM);
  float minSM = min(lumaS, lumaM);
  float maxESM = max(lumaE, maxSM);
  float minESM = min(lumaE, minSM);
  float maxWN = max(lumaN, lumaW);
  float minWN = min(lumaN, lumaW);
  float rangeMax = max(maxWN, maxESM);
  float rangeMin = min(minWN, minESM);
  float rangeMaxScaled = rangeMax * fxaaQualityEdgeThreshold;
  float range = rangeMax - rangeMin;
  float rangeMaxClamped = max(fxaaQualityEdgeThresholdMin, rangeMaxScaled);
  bool earlyExit = range < rangeMaxClamped;

  if (earlyExit) {
    return rgbyM;
  }

  float lumaNW = FxaaLuma(FxaaTexOff(tex, posM, int2(-1, -1), fxaaQualityRcpFrame.xy));
  float lumaSE = FxaaLuma(FxaaTexOff(tex, posM, int2(1, 1), fxaaQualityRcpFrame.xy));
  float lumaNE = FxaaLuma(FxaaTexOff(tex, posM, int2(1, -1), fxaaQualityRcpFrame.xy));
  float lumaSW = FxaaLuma(FxaaTexOff(tex, posM, int2(-1, 1), fxaaQualityRcpFrame.xy));

  float lumaNS = lumaN + lumaS;
  float lumaWE = lumaW + lumaE;
  float subpixRcpRange = 1.0 / range;
  float subpixNSWE = lumaNS + lumaWE;
  float edgeHorz1 = (-2.0 * lumaM) + lumaNS;
  float edgeVert1 = (-2.0 * lumaM) + lumaWE;

  float lumaNESE = lumaNE + lumaSE;
  float lumaNWNE = lumaNW + lumaNE;
  float edgeHorz2 = (-2.0 * lumaE) + lumaNESE;
  float edgeVert2 = (-2.0 * lumaN) + lumaNWNE;

  float lumaNWSW = lumaNW + lumaSW;
  float lumaSWSE = lumaSW + lumaSE;
  float edgeHorz4 = (abs(edgeHorz1) * 2.0) + abs(edgeHorz2);
  float edgeVert4 = (abs(edgeVert1) * 2.0) + abs(edgeVert2);
  float edgeHorz3 = (-2.0 * lumaW) + lumaNWSW;
  float edgeVert3 = (-2.0 * lumaS) + lumaSWSE;
  float edgeHorz = abs(edgeHorz3) + edgeHorz4;
  float edgeVert = abs(edgeVert3) + edgeVert4;

  float subpixNWSWNESE = lumaNWSW + lumaNESE;
  float lengthSign = fxaaQualityRcpFrame.x;
  bool horzSpan = edgeHorz >= edgeVert;
  float subpixA = subpixNSWE * 2.0 + subpixNWSWNESE;
  if (!horzSpan) {
    lumaN = lumaW;
  }
  if (!horzSpan) {
    lumaS = lumaE;
  }
  if (horzSpan) {
    lengthSign = fxaaQualityRcpFrame.y;
  }
  float subpixB = (subpixA * (1.0 / 12.0)) - lumaM;

  float gradientN = lumaN - lumaM;
  float gradientS = lumaS - lumaM;
  float lumaNN = lumaN + lumaM;
  float lumaSS = lumaS + lumaM;
  bool pairN = abs(gradientN) >= abs(gradientS);
  float gradient = max(abs(gradientN), abs(gradientS));
  if (pairN) {
    lengthSign = -lengthSign;
  }
  float subpixC = FxaaSat(abs(subpixB) * subpixRcpRange);

  float2 posB;
  posB.x = posM.x;
  posB.y = posM.y;
  float2 offNP;
  offNP.x = (!horzSpan) ? 0.0 : fxaaQualityRcpFrame.x;
  offNP.y = (horzSpan) ? 0.0 : fxaaQualityRcpFrame.y;
  if (!horzSpan) {
    posB.x += lengthSign * 0.5;
  }
  if (horzSpan) {
    posB.y += lengthSign * 0.5;
  }
  float2 posN;
  posN.x = posB.x - offNP.x * FXAA_QUALITY_P0;
  posN.y = posB.y - offNP.y * FXAA_QUALITY_P0;
  float2 posP;
  posP.x = posB.x + offNP.x * FXAA_QUALITY_P0;
  posP.y = posB.y + offNP.y * FXAA_QUALITY_P0;
  float subpixD = ((-2.0) * subpixC) + 3.0;
  float lumaEndN = FxaaLuma(FxaaTexTop(tex, posN));
  float subpixE = subpixC * subpixC;
  float lumaEndP = FxaaLuma(FxaaTexTop(tex, posP));

  if (!pairN) {
    lumaNN = lumaSS;
  }
  float gradientScaled = gradient * 1.0 / 4.0;
  float lumaMM = lumaM - lumaNN * 0.5;
  float subpixF = subpixD * subpixE;
  bool lumaMLTZero = lumaMM < 0.0;

  lumaEndN -= lumaNN * 0.5;
  lumaEndP -= lumaNN * 0.5;
  bool doneN = abs(lumaEndN) >= gradientScaled;
  bool doneP = abs(lumaEndP) >= gradientScaled;
  if (!doneN) {
    posN.x -= offNP.x * FXAA_QUALITY_P1;
  }
  if (!doneN) {
    posN.y -= offNP.y * FXAA_QUALITY_P1;
  }
  bool doneNP = (!doneN) || (!doneP);
  if (!doneP) {
    posP.x += offNP.x * FXAA_QUALITY_P1;
  }
  if (!doneP) {
    posP.y += offNP.y * FXAA_QUALITY_P1;
  }

  if (doneNP) {
    if (!doneN) {
      lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
    }
    if (!doneP) {
      lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
    }
    if (!doneN) {
      lumaEndN = lumaEndN - lumaNN * 0.5;
    }
    if (!doneP) {
      lumaEndP = lumaEndP - lumaNN * 0.5;
    }
    doneN = abs(lumaEndN) >= gradientScaled;
    doneP = abs(lumaEndP) >= gradientScaled;
    if (!doneN) {
      posN.x -= offNP.x * FXAA_QUALITY_P2;
    }
    if (!doneN) {
      posN.y -= offNP.y * FXAA_QUALITY_P2;
    }
    doneNP = (!doneN) || (!doneP);
    if (!doneP) {
      posP.x += offNP.x * FXAA_QUALITY_P2;
    }
    if (!doneP) {
      posP.y += offNP.y * FXAA_QUALITY_P2;
    }
#if (FXAA_QUALITY_PS > 3)
    if (doneNP) {
      if (!doneN) {
        lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
      }
      if (!doneP) {
        lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
      }
      if (!doneN) {
        lumaEndN = lumaEndN - lumaNN * 0.5;
      }
      if (!doneP) {
        lumaEndP = lumaEndP - lumaNN * 0.5;
      }
      doneN = abs(lumaEndN) >= gradientScaled;
      doneP = abs(lumaEndP) >= gradientScaled;
      if (!doneN) {
        posN.x -= offNP.x * FXAA_QUALITY_P3;
      }
      if (!doneN) {
        posN.y -= offNP.y * FXAA_QUALITY_P3;
      }
      doneNP = (!doneN) || (!doneP);
      if (!doneP) {
        posP.x += offNP.x * FXAA_QUALITY_P3;
      }
      if (!doneP) {
        posP.y += offNP.y * FXAA_QUALITY_P3;
      }
#  if (FXAA_QUALITY_PS > 4)
      if (doneNP) {
        if (!doneN) {
          lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
        }
        if (!doneP) {
          lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
        }
        if (!doneN) {
          lumaEndN = lumaEndN - lumaNN * 0.5;
        }
        if (!doneP) {
          lumaEndP = lumaEndP - lumaNN * 0.5;
        }
        doneN = abs(lumaEndN) >= gradientScaled;
        doneP = abs(lumaEndP) >= gradientScaled;
        if (!doneN) {
          posN.x -= offNP.x * FXAA_QUALITY_P4;
        }
        if (!doneN) {
          posN.y -= offNP.y * FXAA_QUALITY_P4;
        }
        doneNP = (!doneN) || (!doneP);
        if (!doneP) {
          posP.x += offNP.x * FXAA_QUALITY_P4;
        }
        if (!doneP) {
          posP.y += offNP.y * FXAA_QUALITY_P4;
        }
#    if (FXAA_QUALITY_PS > 5)
        if (doneNP) {
          if (!doneN) {
            lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
          }
          if (!doneP) {
            lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
          }
          if (!doneN) {
            lumaEndN = lumaEndN - lumaNN * 0.5;
          }
          if (!doneP) {
            lumaEndP = lumaEndP - lumaNN * 0.5;
          }
          doneN = abs(lumaEndN) >= gradientScaled;
          doneP = abs(lumaEndP) >= gradientScaled;
          if (!doneN) {
            posN.x -= offNP.x * FXAA_QUALITY_P5;
          }
          if (!doneN) {
            posN.y -= offNP.y * FXAA_QUALITY_P5;
          }
          doneNP = (!doneN) || (!doneP);
          if (!doneP) {
            posP.x += offNP.x * FXAA_QUALITY_P5;
          }
          if (!doneP) {
            posP.y += offNP.y * FXAA_QUALITY_P5;
          }
#      if (FXAA_QUALITY_PS > 6)
          if (doneNP) {
            if (!doneN) {
              lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
            }
            if (!doneP) {
              lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
            }
            if (!doneN) {
              lumaEndN = lumaEndN - lumaNN * 0.5;
            }
            if (!doneP) {
              lumaEndP = lumaEndP - lumaNN * 0.5;
            }
            doneN = abs(lumaEndN) >= gradientScaled;
            doneP = abs(lumaEndP) >= gradientScaled;
            if (!doneN) {
              posN.x -= offNP.x * FXAA_QUALITY_P6;
            }
            if (!doneN) {
              posN.y -= offNP.y * FXAA_QUALITY_P6;
            }
            doneNP = (!doneN) || (!doneP);
            if (!doneP) {
              posP.x += offNP.x * FXAA_QUALITY_P6;
            }
            if (!doneP) {
              posP.y += offNP.y * FXAA_QUALITY_P6;
            }
#        if (FXAA_QUALITY_PS > 7)
            if (doneNP) {
              if (!doneN) {
                lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
              }
              if (!doneP) {
                lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
              }
              if (!doneN) {
                lumaEndN = lumaEndN - lumaNN * 0.5;
              }
              if (!doneP) {
                lumaEndP = lumaEndP - lumaNN * 0.5;
              }
              doneN = abs(lumaEndN) >= gradientScaled;
              doneP = abs(lumaEndP) >= gradientScaled;
              if (!doneN) {
                posN.x -= offNP.x * FXAA_QUALITY_P7;
              }
              if (!doneN) {
                posN.y -= offNP.y * FXAA_QUALITY_P7;
              }
              doneNP = (!doneN) || (!doneP);
              if (!doneP) {
                posP.x += offNP.x * FXAA_QUALITY_P7;
              }
              if (!doneP) {
                posP.y += offNP.y * FXAA_QUALITY_P7;
              }
#          if (FXAA_QUALITY_PS > 8)
              if (doneNP) {
                if (!doneN) {
                  lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
                }
                if (!doneP) {
                  lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
                }
                if (!doneN) {
                  lumaEndN = lumaEndN - lumaNN * 0.5;
                }
                if (!doneP) {
                  lumaEndP = lumaEndP - lumaNN * 0.5;
                }
                doneN = abs(lumaEndN) >= gradientScaled;
                doneP = abs(lumaEndP) >= gradientScaled;
                if (!doneN) {
                  posN.x -= offNP.x * FXAA_QUALITY_P8;
                }
                if (!doneN) {
                  posN.y -= offNP.y * FXAA_QUALITY_P8;
                }
                doneNP = (!doneN) || (!doneP);
                if (!doneP) {
                  posP.x += offNP.x * FXAA_QUALITY_P8;
                }
                if (!doneP) {
                  posP.y += offNP.y * FXAA_QUALITY_P8;
                }
#            if (FXAA_QUALITY_PS > 9)
                if (doneNP) {
                  if (!doneN) {
                    lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
                  }
                  if (!doneP) {
                    lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
                  }
                  if (!doneN) {
                    lumaEndN = lumaEndN - lumaNN * 0.5;
                  }
                  if (!doneP) {
                    lumaEndP = lumaEndP - lumaNN * 0.5;
                  }
                  doneN = abs(lumaEndN) >= gradientScaled;
                  doneP = abs(lumaEndP) >= gradientScaled;
                  if (!doneN) {
                    posN.x -= offNP.x * FXAA_QUALITY_P9;
                  }
                  if (!doneN) {
                    posN.y -= offNP.y * FXAA_QUALITY_P9;
                  }
                  doneNP = (!doneN) || (!doneP);
                  if (!doneP) {
                    posP.x += offNP.x * FXAA_QUALITY_P9;
                  }
                  if (!doneP) {
                    posP.y += offNP.y * FXAA_QUALITY_P9;
                  }
#              if (FXAA_QUALITY_PS > 10)
                  if (doneNP) {
                    if (!doneN) {
                      lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
                    }
                    if (!doneP) {
                      lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
                    }
                    if (!doneN) {
                      lumaEndN = lumaEndN - lumaNN * 0.5;
                    }
                    if (!doneP) {
                      lumaEndP = lumaEndP - lumaNN * 0.5;
                    }
                    doneN = abs(lumaEndN) >= gradientScaled;
                    doneP = abs(lumaEndP) >= gradientScaled;
                    if (!doneN) {
                      posN.x -= offNP.x * FXAA_QUALITY_P10;
                    }
                    if (!doneN) {
                      posN.y -= offNP.y * FXAA_QUALITY_P10;
                    }
                    doneNP = (!doneN) || (!doneP);
                    if (!doneP) {
                      posP.x += offNP.x * FXAA_QUALITY_P10;
                    }
                    if (!doneP) {
                      posP.y += offNP.y * FXAA_QUALITY_P10;
                    }
#                if (FXAA_QUALITY_PS > 11)
                    if (doneNP) {
                      if (!doneN) {
                        lumaEndN = FxaaLuma(FxaaTexTop(tex, posN.xy));
                      }
                      if (!doneP) {
                        lumaEndP = FxaaLuma(FxaaTexTop(tex, posP.xy));
                      }
                      if (!doneN) {
                        lumaEndN = lumaEndN - lumaNN * 0.5;
                      }
                      if (!doneP) {
                        lumaEndP = lumaEndP - lumaNN * 0.5;
                      }
                      doneN = abs(lumaEndN) >= gradientScaled;
                      doneP = abs(lumaEndP) >= gradientScaled;
                      if (!doneN) {
                        posN.x -= offNP.x * FXAA_QUALITY_P11;
                      }
                      if (!doneN) {
                        posN.y -= offNP.y * FXAA_QUALITY_P11;
                      }
                      doneNP = (!doneN) || (!doneP);
                      if (!doneP) {
                        posP.x += offNP.x * FXAA_QUALITY_P11;
                      }
                      if (!doneP) {
                        posP.y += offNP.y * FXAA_QUALITY_P11;
                      }
                    }
#                endif
                  }
#              endif
                }
#            endif
              }
#          endif
            }
#        endif
          }
#      endif
        }
#    endif
      }
#  endif
    }
#endif
  }

  float dstN = posM.x - posN.x;
  float dstP = posP.x - posM.x;
  if (!horzSpan) {
    dstN = posM.y - posN.y;
  }
  if (!horzSpan) {
    dstP = posP.y - posM.y;
  }

  bool goodSpanN = (lumaEndN < 0.0) != lumaMLTZero;
  float spanLength = (dstP + dstN);
  bool goodSpanP = (lumaEndP < 0.0) != lumaMLTZero;
  float spanLengthRcp = 1.0 / spanLength;

  bool directionN = dstN < dstP;
  float dst = min(dstN, dstP);
  bool goodSpan = directionN ? goodSpanN : goodSpanP;
  float subpixG = subpixF * subpixF;
  float pixelOffset = (dst * (-spanLengthRcp)) + 0.5;
  float subpixH = subpixG * fxaaQualitySubpix;

  float pixelOffsetGood = goodSpan ? pixelOffset : 0.0;
  float pixelOffsetSubpix = max(pixelOffsetGood, subpixH);
  if (!horzSpan) {
    posM.x += pixelOffsetSubpix * lengthSign;
  }
  if (horzSpan) {
    posM.y += pixelOffsetSubpix * lengthSign;
  }
  return FxaaTexTop(tex, posM);
}
