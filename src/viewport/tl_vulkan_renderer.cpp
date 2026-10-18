#include "opentl/viewport/tl_vulkan_renderer.h"

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <cstdlib>

#if defined(OPENTL_HAS_VULKAN)
  #include <vulkan/vulkan.h>
#endif

namespace {

// Catppuccin Mocha roles used by the renderer (copied so the pointers stay stable).
const float kBackground[4] = { TL_CatppuccinMochaTheme::kCrust[0], TL_CatppuccinMochaTheme::kCrust[1],
                               TL_CatppuccinMochaTheme::kCrust[2], 1.0f };
const float kGrid[4] = { TL_CatppuccinMochaTheme::kSurface0[0], TL_CatppuccinMochaTheme::kSurface0[1],
                         TL_CatppuccinMochaTheme::kSurface0[2], 1.0f };
const float kGridMajor[4] = { TL_CatppuccinMochaTheme::kSurface1[0], TL_CatppuccinMochaTheme::kSurface1[1],
                              TL_CatppuccinMochaTheme::kSurface1[2], 1.0f };
const float kAxisX[4] = { TL_CatppuccinMochaTheme::kRed[0], TL_CatppuccinMochaTheme::kRed[1],
                          TL_CatppuccinMochaTheme::kRed[2], 1.0f };
const float kAxisY[4] = { TL_CatppuccinMochaTheme::kGreen[0], TL_CatppuccinMochaTheme::kGreen[1],
                          TL_CatppuccinMochaTheme::kGreen[2], 1.0f };
const float kAxisZ[4] = { TL_CatppuccinMochaTheme::kBlue[0], TL_CatppuccinMochaTheme::kBlue[1],
                          TL_CatppuccinMochaTheme::kBlue[2], 1.0f };
const float kBorder[4] = { TL_CatppuccinMochaTheme::kSurface2[0], TL_CatppuccinMochaTheme::kSurface2[1],
                           TL_CatppuccinMochaTheme::kSurface2[2], 1.0f };
const float kLabel[4] = { TL_CatppuccinMochaTheme::kLavender[0], TL_CatppuccinMochaTheme::kLavender[1],
                          TL_CatppuccinMochaTheme::kLavender[2], 1.0f };
const float kOrigin[4] = { TL_CatppuccinMochaTheme::kYellow[0], TL_CatppuccinMochaTheme::kYellow[1],
                           TL_CatppuccinMochaTheme::kYellow[2], 1.0f };
const float kGeometryFallback[4] = { TL_CatppuccinMochaTheme::kSapphire[0],
                                     TL_CatppuccinMochaTheme::kSapphire[1],
                                     TL_CatppuccinMochaTheme::kSapphire[2], 1.0f };
// Silhouette + crease overlay: Text (#cdd6f4), the lightest neutral in the palette, so
// it reads as a drawing line rather than as another material.
const float kFeatureEdge[4] = { TL_CatppuccinMochaTheme::kText[0], TL_CatppuccinMochaTheme::kText[1],
                                TL_CatppuccinMochaTheme::kText[2], TL_CatppuccinMochaTheme::kText[3] };

const float kMeshColor[4] = { TL_CatppuccinMochaTheme::kMauve[0], TL_CatppuccinMochaTheme::kMauve[1],
                              TL_CatppuccinMochaTheme::kMauve[2], 1.0f };

// ============================================================================
// Shaders (GLSL 450 -> SPIR-V via glslc, embedded as uint32_t[]).
//
// mesh.vert -- Gouraud Lambert + Blinn-Phong, pixel-space XY + normalised depth.
//   #version 450
//   layout(location = 0) in vec3 inPos;      // pixels + depth
//   layout(location = 1) in vec3 inNormal;
//   layout(location = 2) in vec4 inColor;
//   layout(push_constant) uniform PC { vec2 canvasSize; } pc;
//   layout(location = 0) out vec4 fragColor;
//   void main() {
//     vec3 n = normalize(inNormal);
//     vec3 l = normalize(vec3(0.38, -0.52, 0.76));
//     float diffuse = max(dot(n, l), 0.0);
//     vec3 h = normalize(l + vec3(0.0, 0.0, 1.0));
//     float specular = pow(max(dot(n, h), 0.0), 28.0) * 0.22;
//     vec3 c = inColor.rgb * (0.44 + 0.72 * diffuse) + vec3(specular);
//     vec2 ndc = (inPos.xy / pc.canvasSize) * 2.0 - 1.0;
//     gl_Position = vec4(ndc, inPos.z, 1.0);
//     fragColor = vec4(clamp(c, 0.0, 1.0), inColor.a);
//   }
//
// line.vert -- unlit, pixel-space XY + depth.
// ============================================================================

#if defined(OPENTL_HAS_VULKAN)

const uint32_t kMeshVertSpv[] = {
  0x07230203u, 0x00010000u, 0x000d000bu, 0x00000067u, 0x00000000u, 0x00020011u, 0x00000001u,
  0x0006000bu, 0x00000001u, 0x4c534c47u, 0x6474732eu, 0x3035342eu, 0x00000000u, 0x0003000eu,
  0x00000000u, 0x00000001u, 0x000a000fu, 0x00000000u, 0x00000004u, 0x6e69616du, 0x00000000u,
  0x0000000bu, 0x0000002cu, 0x0000003bu, 0x00000050u, 0x0000005bu, 0x00030003u, 0x00000002u,
  0x000001c2u, 0x000a0004u, 0x475f4c47u, 0x4c474f4fu, 0x70635f45u, 0x74735f70u, 0x5f656c79u,
  0x656e696cu, 0x7269645fu, 0x69746365u, 0x00006576u, 0x00080004u, 0x475f4c47u, 0x4c474f4fu,
  0x6e695f45u, 0x64756c63u, 0x69645f65u, 0x74636572u, 0x00657669u, 0x00040005u, 0x00000004u,
  0x6e69616du, 0x00000000u, 0x00030005u, 0x00000009u, 0x0000006eu, 0x00050005u, 0x0000000bu,
  0x6f4e6e69u, 0x6c616d72u, 0x00000000u, 0x00030005u, 0x0000000eu, 0x0000006cu, 0x00040005u,
  0x00000014u, 0x66666964u, 0x00657375u, 0x00030005u, 0x0000001au, 0x00000068u, 0x00050005u,
  0x00000020u, 0x63657073u, 0x72616c75u, 0x00000000u, 0x00030005u, 0x00000029u, 0x00000063u,
  0x00040005u, 0x0000002cu, 0x6f436e69u, 0x00726f6cu, 0x00030005u, 0x0000003au, 0x0063646eu,
  0x00040005u, 0x0000003bu, 0x6f506e69u, 0x00000073u, 0x00060005u, 0x0000003eu, 0x68737550u,
  0x736e6f43u, 0x746e6174u, 0x00000073u, 0x00060006u, 0x0000003eu, 0x00000000u, 0x766e6163u,
  0x69537361u, 0x0000657au, 0x00030005u, 0x00000040u, 0x00006370u, 0x00060005u, 0x0000004eu,
  0x505f6c67u, 0x65567265u, 0x78657472u, 0x00000000u, 0x00060006u, 0x0000004eu, 0x00000000u,
  0x505f6c67u, 0x7469736fu, 0x006e6f69u, 0x00070006u, 0x0000004eu, 0x00000001u, 0x505f6c67u,
  0x746e696fu, 0x657a6953u, 0x00000000u, 0x00070006u, 0x0000004eu, 0x00000002u, 0x435f6c67u,
  0x4470696cu, 0x61747369u, 0x0065636eu, 0x00070006u, 0x0000004eu, 0x00000003u, 0x435f6c67u,
  0x446c6c75u, 0x61747369u, 0x0065636eu, 0x00030005u, 0x00000050u, 0x00000000u, 0x00050005u,
  0x0000005bu, 0x67617266u, 0x6f6c6f43u, 0x00000072u, 0x00040047u, 0x0000000bu, 0x0000001eu,
  0x00000001u, 0x00040047u, 0x0000002cu, 0x0000001eu, 0x00000002u, 0x00040047u, 0x0000003bu,
  0x0000001eu, 0x00000000u, 0x00030047u, 0x0000003eu, 0x00000002u, 0x00050048u, 0x0000003eu,
  0x00000000u, 0x00000023u, 0x00000000u, 0x00030047u, 0x0000004eu, 0x00000002u, 0x00050048u,
  0x0000004eu, 0x00000000u, 0x0000000bu, 0x00000000u, 0x00050048u, 0x0000004eu, 0x00000001u,
  0x0000000bu, 0x00000001u, 0x00050048u, 0x0000004eu, 0x00000002u, 0x0000000bu, 0x00000003u,
  0x00050048u, 0x0000004eu, 0x00000003u, 0x0000000bu, 0x00000004u, 0x00040047u, 0x0000005bu,
  0x0000001eu, 0x00000000u, 0x00020013u, 0x00000002u, 0x00030021u, 0x00000003u, 0x00000002u,
  0x00030016u, 0x00000006u, 0x00000020u, 0x00040017u, 0x00000007u, 0x00000006u, 0x00000003u,
  0x00040020u, 0x00000008u, 0x00000007u, 0x00000007u, 0x00040020u, 0x0000000au, 0x00000001u,
  0x00000007u, 0x0004003bu, 0x0000000au, 0x0000000bu, 0x00000001u, 0x0004002bu, 0x00000006u,
  0x0000000fu, 0x3ec34db7u, 0x0004002bu, 0x00000006u, 0x00000010u, 0xbf05a0f6u, 0x0004002bu,
  0x00000006u, 0x00000011u, 0x3f434db7u, 0x0006002cu, 0x00000007u, 0x00000012u, 0x0000000fu,
  0x00000010u, 0x00000011u, 0x00040020u, 0x00000013u, 0x00000007u, 0x00000006u, 0x0004002bu,
  0x00000006u, 0x00000018u, 0x00000000u, 0x0004002bu, 0x00000006u, 0x0000001cu, 0x3f800000u,
  0x0006002cu, 0x00000007u, 0x0000001du, 0x00000018u, 0x00000018u, 0x0000001cu, 0x0004002bu,
  0x00000006u, 0x00000025u, 0x41e00000u, 0x0004002bu, 0x00000006u, 0x00000027u, 0x3e6147aeu,
  0x00040017u, 0x0000002au, 0x00000006u, 0x00000004u, 0x00040020u, 0x0000002bu, 0x00000001u,
  0x0000002au, 0x0004003bu, 0x0000002bu, 0x0000002cu, 0x00000001u, 0x0004002bu, 0x00000006u,
  0x0000002fu, 0x3ee147aeu, 0x0004002bu, 0x00000006u, 0x00000030u, 0x3f3851ecu, 0x00040017u,
  0x00000038u, 0x00000006u, 0x00000002u, 0x00040020u, 0x00000039u, 0x00000007u, 0x00000038u,
  0x0004003bu, 0x0000000au, 0x0000003bu, 0x00000001u, 0x0003001eu, 0x0000003eu, 0x00000038u,
  0x00040020u, 0x0000003fu, 0x00000009u, 0x0000003eu, 0x0004003bu, 0x0000003fu, 0x00000040u,
  0x00000009u, 0x00040015u, 0x00000041u, 0x00000020u, 0x00000001u, 0x0004002bu, 0x00000041u,
  0x00000042u, 0x00000000u, 0x00040020u, 0x00000043u, 0x00000009u, 0x00000038u, 0x0004002bu,
  0x00000006u, 0x00000047u, 0x40000000u, 0x00040015u, 0x0000004bu, 0x00000020u, 0x00000000u,
  0x0004002bu, 0x0000004bu, 0x0000004cu, 0x00000001u, 0x0004001cu, 0x0000004du, 0x00000006u,
  0x0000004cu, 0x0006001eu, 0x0000004eu, 0x0000002au, 0x00000006u, 0x0000004du, 0x0000004du,
  0x00040020u, 0x0000004fu, 0x00000003u, 0x0000004eu, 0x0004003bu, 0x0000004fu, 0x00000050u,
  0x00000003u, 0x0004002bu, 0x0000004bu, 0x00000052u, 0x00000002u, 0x00040020u, 0x00000053u,
  0x00000001u, 0x00000006u, 0x00040020u, 0x00000059u, 0x00000003u, 0x0000002au, 0x0004003bu,
  0x00000059u, 0x0000005bu, 0x00000003u, 0x0004002bu, 0x0000004bu, 0x00000060u, 0x00000003u,
  0x00050036u, 0x00000002u, 0x00000004u, 0x00000000u, 0x00000003u, 0x000200f8u, 0x00000005u,
  0x0004003bu, 0x00000008u, 0x00000009u, 0x00000007u, 0x0004003bu, 0x00000008u, 0x0000000eu,
  0x00000007u, 0x0004003bu, 0x00000013u, 0x00000014u, 0x00000007u, 0x0004003bu, 0x00000008u,
  0x0000001au, 0x00000007u, 0x0004003bu, 0x00000013u, 0x00000020u, 0x00000007u, 0x0004003bu,
  0x00000008u, 0x00000029u, 0x00000007u, 0x0004003bu, 0x00000039u, 0x0000003au, 0x00000007u,
  0x0004003du, 0x00000007u, 0x0000000cu, 0x0000000bu, 0x0006000cu, 0x00000007u, 0x0000000du,
  0x00000001u, 0x00000045u, 0x0000000cu, 0x0003003eu, 0x00000009u, 0x0000000du, 0x0003003eu,
  0x0000000eu, 0x00000012u, 0x0004003du, 0x00000007u, 0x00000015u, 0x00000009u, 0x0004003du,
  0x00000007u, 0x00000016u, 0x0000000eu, 0x00050094u, 0x00000006u, 0x00000017u, 0x00000015u,
  0x00000016u, 0x0007000cu, 0x00000006u, 0x00000019u, 0x00000001u, 0x00000028u, 0x00000017u,
  0x00000018u, 0x0003003eu, 0x00000014u, 0x00000019u, 0x0004003du, 0x00000007u, 0x0000001bu,
  0x0000000eu, 0x00050081u, 0x00000007u, 0x0000001eu, 0x0000001bu, 0x0000001du, 0x0006000cu,
  0x00000007u, 0x0000001fu, 0x00000001u, 0x00000045u, 0x0000001eu, 0x0003003eu, 0x0000001au,
  0x0000001fu, 0x0004003du, 0x00000007u, 0x00000021u, 0x00000009u, 0x0004003du, 0x00000007u,
  0x00000022u, 0x0000001au, 0x00050094u, 0x00000006u, 0x00000023u, 0x00000021u, 0x00000022u,
  0x0007000cu, 0x00000006u, 0x00000024u, 0x00000001u, 0x00000028u, 0x00000023u, 0x00000018u,
  0x0007000cu, 0x00000006u, 0x00000026u, 0x00000001u, 0x0000001au, 0x00000024u, 0x00000025u,
  0x00050085u, 0x00000006u, 0x00000028u, 0x00000026u, 0x00000027u, 0x0003003eu, 0x00000020u,
  0x00000028u, 0x0004003du, 0x0000002au, 0x0000002du, 0x0000002cu, 0x0008004fu, 0x00000007u,
  0x0000002eu, 0x0000002du, 0x0000002du, 0x00000000u, 0x00000001u, 0x00000002u, 0x0004003du,
  0x00000006u, 0x00000031u, 0x00000014u, 0x00050085u, 0x00000006u, 0x00000032u, 0x00000030u,
  0x00000031u, 0x00050081u, 0x00000006u, 0x00000033u, 0x0000002fu, 0x00000032u, 0x0005008eu,
  0x00000007u, 0x00000034u, 0x0000002eu, 0x00000033u, 0x0004003du, 0x00000006u, 0x00000035u,
  0x00000020u, 0x00060050u, 0x00000007u, 0x00000036u, 0x00000035u, 0x00000035u, 0x00000035u,
  0x00050081u, 0x00000007u, 0x00000037u, 0x00000034u, 0x00000036u, 0x0003003eu, 0x00000029u,
  0x00000037u, 0x0004003du, 0x00000007u, 0x0000003cu, 0x0000003bu, 0x0007004fu, 0x00000038u,
  0x0000003du, 0x0000003cu, 0x0000003cu, 0x00000000u, 0x00000001u, 0x00050041u, 0x00000043u,
  0x00000044u, 0x00000040u, 0x00000042u, 0x0004003du, 0x00000038u, 0x00000045u, 0x00000044u,
  0x00050088u, 0x00000038u, 0x00000046u, 0x0000003du, 0x00000045u, 0x0005008eu, 0x00000038u,
  0x00000048u, 0x00000046u, 0x00000047u, 0x00050050u, 0x00000038u, 0x00000049u, 0x0000001cu,
  0x0000001cu, 0x00050083u, 0x00000038u, 0x0000004au, 0x00000048u, 0x00000049u, 0x0003003eu,
  0x0000003au, 0x0000004au, 0x0004003du, 0x00000038u, 0x00000051u, 0x0000003au, 0x00050041u,
  0x00000053u, 0x00000054u, 0x0000003bu, 0x00000052u, 0x0004003du, 0x00000006u, 0x00000055u,
  0x00000054u, 0x00050051u, 0x00000006u, 0x00000056u, 0x00000051u, 0x00000000u, 0x00050051u,
  0x00000006u, 0x00000057u, 0x00000051u, 0x00000001u, 0x00070050u, 0x0000002au, 0x00000058u,
  0x00000056u, 0x00000057u, 0x00000055u, 0x0000001cu, 0x00050041u, 0x00000059u, 0x0000005au,
  0x00000050u, 0x00000042u, 0x0003003eu, 0x0000005au, 0x00000058u, 0x0004003du, 0x00000007u,
  0x0000005cu, 0x00000029u, 0x00060050u, 0x00000007u, 0x0000005du, 0x00000018u, 0x00000018u,
  0x00000018u, 0x00060050u, 0x00000007u, 0x0000005eu, 0x0000001cu, 0x0000001cu, 0x0000001cu,
  0x0008000cu, 0x00000007u, 0x0000005fu, 0x00000001u, 0x0000002bu, 0x0000005cu, 0x0000005du,
  0x0000005eu, 0x00050041u, 0x00000053u, 0x00000061u, 0x0000002cu, 0x00000060u, 0x0004003du,
  0x00000006u, 0x00000062u, 0x00000061u, 0x00050051u, 0x00000006u, 0x00000063u, 0x0000005fu,
  0x00000000u, 0x00050051u, 0x00000006u, 0x00000064u, 0x0000005fu, 0x00000001u, 0x00050051u,
  0x00000006u, 0x00000065u, 0x0000005fu, 0x00000002u, 0x00070050u, 0x0000002au, 0x00000066u,
  0x00000063u, 0x00000064u, 0x00000065u, 0x00000062u, 0x0003003eu, 0x0000005bu, 0x00000066u,
  0x000100fdu, 0x00010038u
};

const uint32_t kLineVertSpv[] = {
  0x07230203u, 0x00010000u, 0x000d000bu, 0x00000032u, 0x00000000u, 0x00020011u, 0x00000001u,
  0x0006000bu, 0x00000001u, 0x4c534c47u, 0x6474732eu, 0x3035342eu, 0x00000000u, 0x0003000eu,
  0x00000000u, 0x00000001u, 0x0009000fu, 0x00000000u, 0x00000004u, 0x6e69616du, 0x00000000u,
  0x0000000cu, 0x00000023u, 0x0000002eu, 0x00000030u, 0x00030003u, 0x00000002u, 0x000001c2u,
  0x000a0004u, 0x475f4c47u, 0x4c474f4fu, 0x70635f45u, 0x74735f70u, 0x5f656c79u, 0x656e696cu,
  0x7269645fu, 0x69746365u, 0x00006576u, 0x00080004u, 0x475f4c47u, 0x4c474f4fu, 0x6e695f45u,
  0x64756c63u, 0x69645f65u, 0x74636572u, 0x00657669u, 0x00040005u, 0x00000004u, 0x6e69616du,
  0x00000000u, 0x00030005u, 0x00000009u, 0x0063646eu, 0x00040005u, 0x0000000cu, 0x6f506e69u,
  0x00000073u, 0x00060005u, 0x0000000fu, 0x68737550u, 0x736e6f43u, 0x746e6174u, 0x00000073u,
  0x00060006u, 0x0000000fu, 0x00000000u, 0x766e6163u, 0x69537361u, 0x0000657au, 0x00030005u,
  0x00000011u, 0x00006370u, 0x00060005u, 0x00000021u, 0x505f6c67u, 0x65567265u, 0x78657472u,
  0x00000000u, 0x00060006u, 0x00000021u, 0x00000000u, 0x505f6c67u, 0x7469736fu, 0x006e6f69u,
  0x00070006u, 0x00000021u, 0x00000001u, 0x505f6c67u, 0x746e696fu, 0x657a6953u, 0x00000000u,
  0x00070006u, 0x00000021u, 0x00000002u, 0x435f6c67u, 0x4470696cu, 0x61747369u, 0x0065636eu,
  0x00070006u, 0x00000021u, 0x00000003u, 0x435f6c67u, 0x446c6c75u, 0x61747369u, 0x0065636eu,
  0x00030005u, 0x00000023u, 0x00000000u, 0x00050005u, 0x0000002eu, 0x67617266u, 0x6f6c6f43u,
  0x00000072u, 0x00040005u, 0x00000030u, 0x6f436e69u, 0x00726f6cu, 0x00040047u, 0x0000000cu,
  0x0000001eu, 0x00000000u, 0x00030047u, 0x0000000fu, 0x00000002u, 0x00050048u, 0x0000000fu,
  0x00000000u, 0x00000023u, 0x00000000u, 0x00030047u, 0x00000021u, 0x00000002u, 0x00050048u,
  0x00000021u, 0x00000000u, 0x0000000bu, 0x00000000u, 0x00050048u, 0x00000021u, 0x00000001u,
  0x0000000bu, 0x00000001u, 0x00050048u, 0x00000021u, 0x00000002u, 0x0000000bu, 0x00000003u,
  0x00050048u, 0x00000021u, 0x00000003u, 0x0000000bu, 0x00000004u, 0x00040047u, 0x0000002eu,
  0x0000001eu, 0x00000000u, 0x00040047u, 0x00000030u, 0x0000001eu, 0x00000001u, 0x00020013u,
  0x00000002u, 0x00030021u, 0x00000003u, 0x00000002u, 0x00030016u, 0x00000006u, 0x00000020u,
  0x00040017u, 0x00000007u, 0x00000006u, 0x00000002u, 0x00040020u, 0x00000008u, 0x00000007u,
  0x00000007u, 0x00040017u, 0x0000000au, 0x00000006u, 0x00000003u, 0x00040020u, 0x0000000bu,
  0x00000001u, 0x0000000au, 0x0004003bu, 0x0000000bu, 0x0000000cu, 0x00000001u, 0x0003001eu,
  0x0000000fu, 0x00000007u, 0x00040020u, 0x00000010u, 0x00000009u, 0x0000000fu, 0x0004003bu,
  0x00000010u, 0x00000011u, 0x00000009u, 0x00040015u, 0x00000012u, 0x00000020u, 0x00000001u,
  0x0004002bu, 0x00000012u, 0x00000013u, 0x00000000u, 0x00040020u, 0x00000014u, 0x00000009u,
  0x00000007u, 0x0004002bu, 0x00000006u, 0x00000018u, 0x40000000u, 0x0004002bu, 0x00000006u,
  0x0000001au, 0x3f800000u, 0x00040017u, 0x0000001du, 0x00000006u, 0x00000004u, 0x00040015u,
  0x0000001eu, 0x00000020u, 0x00000000u, 0x0004002bu, 0x0000001eu, 0x0000001fu, 0x00000001u,
  0x0004001cu, 0x00000020u, 0x00000006u, 0x0000001fu, 0x0006001eu, 0x00000021u, 0x0000001du,
  0x00000006u, 0x00000020u, 0x00000020u, 0x00040020u, 0x00000022u, 0x00000003u, 0x00000021u,
  0x0004003bu, 0x00000022u, 0x00000023u, 0x00000003u, 0x0004002bu, 0x0000001eu, 0x00000025u,
  0x00000002u, 0x00040020u, 0x00000026u, 0x00000001u, 0x00000006u, 0x00040020u, 0x0000002cu,
  0x00000003u, 0x0000001du, 0x0004003bu, 0x0000002cu, 0x0000002eu, 0x00000003u, 0x00040020u,
  0x0000002fu, 0x00000001u, 0x0000001du, 0x0004003bu, 0x0000002fu, 0x00000030u, 0x00000001u,
  0x00050036u, 0x00000002u, 0x00000004u, 0x00000000u, 0x00000003u, 0x000200f8u, 0x00000005u,
  0x0004003bu, 0x00000008u, 0x00000009u, 0x00000007u, 0x0004003du, 0x0000000au, 0x0000000du,
  0x0000000cu, 0x0007004fu, 0x00000007u, 0x0000000eu, 0x0000000du, 0x0000000du, 0x00000000u,
  0x00000001u, 0x00050041u, 0x00000014u, 0x00000015u, 0x00000011u, 0x00000013u, 0x0004003du,
  0x00000007u, 0x00000016u, 0x00000015u, 0x00050088u, 0x00000007u, 0x00000017u, 0x0000000eu,
  0x00000016u, 0x0005008eu, 0x00000007u, 0x00000019u, 0x00000017u, 0x00000018u, 0x00050050u,
  0x00000007u, 0x0000001bu, 0x0000001au, 0x0000001au, 0x00050083u, 0x00000007u, 0x0000001cu,
  0x00000019u, 0x0000001bu, 0x0003003eu, 0x00000009u, 0x0000001cu, 0x0004003du, 0x00000007u,
  0x00000024u, 0x00000009u, 0x00050041u, 0x00000026u, 0x00000027u, 0x0000000cu, 0x00000025u,
  0x0004003du, 0x00000006u, 0x00000028u, 0x00000027u, 0x00050051u, 0x00000006u, 0x00000029u,
  0x00000024u, 0x00000000u, 0x00050051u, 0x00000006u, 0x0000002au, 0x00000024u, 0x00000001u,
  0x00070050u, 0x0000001du, 0x0000002bu, 0x00000029u, 0x0000002au, 0x00000028u, 0x0000001au,
  0x00050041u, 0x0000002cu, 0x0000002du, 0x00000023u, 0x00000013u, 0x0003003eu, 0x0000002du,
  0x0000002bu, 0x0004003du, 0x0000001du, 0x00000031u, 0x00000030u, 0x0003003eu, 0x0000002eu,
  0x00000031u, 0x000100fdu, 0x00010038u
};

const uint32_t kPlainFragSpv[] = {
  0x07230203u, 0x00010000u, 0x000d000bu, 0x0000000du, 0x00000000u, 0x00020011u, 0x00000001u,
  0x0006000bu, 0x00000001u, 0x4c534c47u, 0x6474732eu, 0x3035342eu, 0x00000000u, 0x0003000eu,
  0x00000000u, 0x00000001u, 0x0007000fu, 0x00000004u, 0x00000004u, 0x6e69616du, 0x00000000u,
  0x00000009u, 0x0000000bu, 0x00030010u, 0x00000004u, 0x00000007u, 0x00030003u, 0x00000002u,
  0x000001c2u, 0x000a0004u, 0x475f4c47u, 0x4c474f4fu, 0x70635f45u, 0x74735f70u, 0x5f656c79u,
  0x656e696cu, 0x7269645fu, 0x69746365u, 0x00006576u, 0x00080004u, 0x475f4c47u, 0x4c474f4fu,
  0x6e695f45u, 0x64756c63u, 0x69645f65u, 0x74636572u, 0x00657669u, 0x00040005u, 0x00000004u,
  0x6e69616du, 0x00000000u, 0x00050005u, 0x00000009u, 0x4374756fu, 0x726f6c6fu, 0x00000000u,
  0x00050005u, 0x0000000bu, 0x67617266u, 0x6f6c6f43u, 0x00000072u, 0x00040047u, 0x00000009u,
  0x0000001eu, 0x00000000u, 0x00040047u, 0x0000000bu, 0x0000001eu, 0x00000000u, 0x00020013u,
  0x00000002u, 0x00030021u, 0x00000003u, 0x00000002u, 0x00030016u, 0x00000006u, 0x00000020u,
  0x00040017u, 0x00000007u, 0x00000006u, 0x00000004u, 0x00040020u, 0x00000008u, 0x00000003u,
  0x00000007u, 0x0004003bu, 0x00000008u, 0x00000009u, 0x00000003u, 0x00040020u, 0x0000000au,
  0x00000001u, 0x00000007u, 0x0004003bu, 0x0000000au, 0x0000000bu, 0x00000001u, 0x00050036u,
  0x00000002u, 0x00000004u, 0x00000000u, 0x00000003u, 0x000200f8u, 0x00000005u, 0x0004003du,
  0x00000007u, 0x0000000cu, 0x0000000bu, 0x0003003eu, 0x00000009u, 0x0000000cu, 0x000100fdu,
  0x00010038u
};

struct LineVertex
{
  float pos[3];
  float color[4];
};

struct MeshVertex
{
  float pos[3];
  float normal[3];
  float color[4];
};

struct ViewportRect
{
  float left = 0.0f;
  float top = 0.0f;
  float right = 0.0f;
  float bottom = 0.0f;
};

/** Camera-space depth window for one viewport, used to normalise the 0..1 depth. */
struct DepthWindow
{
  double near_d = 0.0;
  double far_d = 1.0;
  double span = 1.0;
};

// ---- 5x7 bitmap font (uppercase, digits, a few separators) -------------------

struct Glyph
{
  char c;
  uint8_t rows[7];
};

const Glyph kGlyphs[] = {
  { 'A', { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
  { 'B', { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E } },
  { 'C', { 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E } },
  { 'D', { 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E } },
  { 'E', { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F } },
  { 'F', { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 } },
  { 'G', { 0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F } },
  { 'H', { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
  { 'I', { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E } },
  { 'J', { 0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C } },
  { 'K', { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 } },
  { 'L', { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F } },
  { 'M', { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 } },
  { 'N', { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 } },
  { 'O', { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E } },
  { 'P', { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 } },
  { 'Q', { 0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D } },
  { 'R', { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 } },
  { 'S', { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E } },
  { 'T', { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 } },
  { 'U', { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E } },
  { 'V', { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04 } },
  { 'W', { 0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11 } },
  { 'X', { 0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11 } },
  { 'Y', { 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04 } },
  { 'Z', { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F } },
  { '0', { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E } },
  { '1', { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E } },
  { '2', { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F } },
  { '3', { 0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E } },
  { '4', { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 } },
  { '5', { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E } },
  { '6', { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E } },
  { '7', { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 } },
  { '8', { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E } },
  { '9', { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C } },
  { '.', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C } },
  { '-', { 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 } },
  { ':', { 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00 } },
  { ' ', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
};

const uint8_t* FindGlyph(char c)
{
  const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  for (const Glyph& g : kGlyphs)
  {
    if (g.c == upper)
      return g.rows;
  }
  return nullptr;
}

} // anonymous namespace

namespace {

void EmitSegment(std::vector<LineVertex>& out, const ON_2dPoint& a, const ON_2dPoint& b,
                 double depth_a, double depth_b, const float* color)
{
  out.push_back({ { static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(depth_a) },
                  { color[0], color[1], color[2], color[3] } });
  out.push_back({ { static_cast<float>(b.x), static_cast<float>(b.y), static_cast<float>(depth_b) },
                  { color[0], color[1], color[2], color[3] } });
}

bool ClipSegment(const ON_2dPoint& a, const ON_2dPoint& b, const ViewportRect& r,
                 ON_2dPoint& a_out, ON_2dPoint& b_out, double& t0_out, double& t1_out)
{
  double t0 = 0.0;
  double t1 = 1.0;
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;

  const double p[4] = { -dx, dx, -dy, dy };
  const double q[4] = { a.x - r.left, r.right - a.x, a.y - r.top, r.bottom - a.y };

  for (int i = 0; i < 4; ++i)
  {
    if (std::fabs(p[i]) < 1e-12)
    {
      if (q[i] < 0.0)
        return false;
      continue;
    }
    const double t = q[i] / p[i];
    if (p[i] < 0.0)
    {
      if (t > t1)
        return false;
      if (t > t0)
        t0 = t;
    }
    else
    {
      if (t < t0)
        return false;
      if (t < t1)
        t1 = t;
    }
  }

  a_out = ON_2dPoint(a.x + t0 * dx, a.y + t0 * dy);
  b_out = ON_2dPoint(a.x + t1 * dx, a.y + t1 * dy);
  t0_out = t0;
  t1_out = t1;
  return true;
}

/** Camera-space depth of a world point for one viewport, normalised to 0..1. */
double DepthOf(const TL_Viewport& vp, const ON_3dPoint& world, const DepthWindow& window)
{
  const ON_3dPoint origin = (TL_PROJECTION_PERSPECTIVE == vp.m_projection) ? vp.CameraLocation()
                                                                          : vp.m_target;
  const double d = ON_DotProduct(world - origin, vp.m_camera_dir);
  if (!ON_IsValid(d) || window.span <= 1e-12)
    return 0.5;
  return std::clamp((d - window.near_d) / window.span, 0.0, 1.0);
}

/**
 * `bias_towards_eye` pulls the segment a hair closer than the shaded surfaces so a
 * wireframe drawn on top of a mesh wins the depth test.  Grid, axes and HUD pass
 * false: they must be occluded by solids like any other background geometry.
 */
void EmitWorldPolyline(const TL_Viewport& vp, const ViewportRect& rect, const DepthWindow& window,
                       const std::vector<ON_3dPoint>& world_points, const float* color,
                       std::vector<LineVertex>& out, bool bias_towards_eye = false)
{
  if (world_points.size() < 2)
    return;

  bool have_prev = false;
  ON_2dPoint prev = ON_2dPoint::Origin;
  double prev_depth = 0.0;
  for (const ON_3dPoint& p : world_points)
  {
    ON_2dPoint screen = ON_2dPoint::Origin;
    if (!vp.WorldToScreen(p, screen))
    {
      have_prev = false;
      continue;
    }
    const double depth = DepthOf(vp, p, window);
    if (have_prev)
    {
      ON_2dPoint ca = ON_2dPoint::Origin;
      ON_2dPoint cb = ON_2dPoint::Origin;
      double ta = 0.0;
      double tb = 0.0;
      if (ClipSegment(prev, screen, rect, ca, cb, ta, tb))
      {
        double da = prev_depth + (depth - prev_depth) * ta;
        double db = prev_depth + (depth - prev_depth) * tb;
        if (bias_towards_eye)
        {
          da = std::max(0.0, da - 2e-4);
          db = std::max(0.0, db - 2e-4);
        }
        EmitSegment(out, ca, cb, da, db, color);
      }
    }
    prev = screen;
    prev_depth = depth;
    have_prev = true;
  }
}

void EmitLabel(std::vector<LineVertex>& out, const char* text, double x, double y, double pixel,
               const float* color)
{
  double cursor = x;
  for (const char* p = text; nullptr != p && *p != '\0'; ++p)
  {
    const uint8_t* rows = FindGlyph(*p);
    if (nullptr != rows)
    {
      for (int row = 0; row < 7; ++row)
      {
        for (int col = 0; col < 5; ++col)
        {
          if (0 == (rows[row] & (1u << (4 - col))))
            continue;
          const double px = cursor + col * pixel;
          const double py = y + row * pixel;
          EmitSegment(out, ON_2dPoint(px, py), ON_2dPoint(px + pixel, py), 0.0, 0.0, color);
        }
      }
    }
    cursor += 6.0 * pixel;
  }
}

ON_Plane GridPlaneForView(int standard_view)
{
  switch (standard_view)
  {
    case TL_VIEW_FRONT:
      return ON_Plane(ON_3dPoint::Origin, ON_3dVector(1.0, 0.0, 0.0),
                      ON_3dVector(0.0, 0.0, 1.0));
    case TL_VIEW_RIGHT:
      return ON_Plane(ON_3dPoint::Origin, ON_3dVector(0.0, 1.0, 0.0),
                      ON_3dVector(0.0, 0.0, 1.0));
    case TL_VIEW_TOP:
    case TL_VIEW_PERSPECTIVE:
    default:
      return ON_Plane::World_xy;
  }
}

int GridHalfExtent(const TL_Viewport& vp)
{
  if (TL_PROJECTION_PERSPECTIVE == vp.m_projection)
    return 10;
  const double scale = (vp.m_scale > 1e-9) ? vp.m_scale : 1.0;
  const double hx = 0.5 * static_cast<double>(vp.m_width) / scale;
  const double hy = 0.5 * static_cast<double>(vp.m_height) / scale;
  const double half = std::max(hx, hy) + 1.0;
  if (!ON_IsValid(half))
    return 10;
  return std::clamp(static_cast<int>(std::ceil(half)), 10, 200);
}

void EmitGrid(const TL_Viewport& vp, const ViewportRect& rect, const DepthWindow& window,
              std::vector<LineVertex>& out)
{
  const int half = GridHalfExtent(vp);
  const ON_Plane cplane = GridPlaneForView(vp.m_standard_view);

  for (int i = -half; i <= half; ++i)
  {
    if (0 == i)
      continue;
    const bool major = (0 == (i % 5));
    std::vector<ON_3dPoint> u_line;
    u_line.push_back(cplane.PointAt(i, -half));
    u_line.push_back(cplane.PointAt(i, half));
    EmitWorldPolyline(vp, rect, window, u_line, major ? kGridMajor : kGrid, out);

    std::vector<ON_3dPoint> v_line;
    v_line.push_back(cplane.PointAt(-half, i));
    v_line.push_back(cplane.PointAt(half, i));
    EmitWorldPolyline(vp, rect, window, v_line, major ? kGridMajor : kGrid, out);
  }

  const double axis = static_cast<double>(half);
  std::vector<ON_3dPoint> x_axis = { ON_3dPoint(-axis, 0.0, 0.0), ON_3dPoint(axis, 0.0, 0.0) };
  EmitWorldPolyline(vp, rect, window, x_axis, kAxisX, out);
  std::vector<ON_3dPoint> y_axis = { ON_3dPoint(0.0, -axis, 0.0), ON_3dPoint(0.0, axis, 0.0) };
  EmitWorldPolyline(vp, rect, window, y_axis, kAxisY, out);
  std::vector<ON_3dPoint> z_axis = { ON_3dPoint(0.0, 0.0, -axis), ON_3dPoint(0.0, 0.0, axis) };
  EmitWorldPolyline(vp, rect, window, z_axis, kAxisZ, out);
}

std::vector<ON_3dPoint> CurvePolyline(const opentl::mvp::SceneObject& obj)
{
  using opentl::mvp::ObjectKind;
  std::vector<ON_3dPoint> points;

  switch (obj.kind)
  {
    case ObjectKind::Line:
      points.push_back(obj.line_val.from);
      points.push_back(obj.line_val.to);
      break;
    case ObjectKind::Polyline:
      for (int i = 0; i < obj.polyline_val.Count(); ++i)
        points.push_back(obj.polyline_val[i]);
      break;
    case ObjectKind::Circle:
    {
      const int n = 160;
      for (int i = 0; i <= n; ++i)
      {
        const double angle = 2.0 * ON_PI * static_cast<double>(i) / static_cast<double>(n);
        points.push_back(obj.circle_val.PointAt(angle));
      }
      break;
    }
    case ObjectKind::Arc:
    {
      const int n = 96;
      const ON_Interval dom = obj.arc_val.Domain();
      for (int i = 0; i <= n; ++i)
      {
        const double t = dom.ParameterAt(static_cast<double>(i) / static_cast<double>(n));
        points.push_back(obj.arc_val.PointAt(t));
      }
      break;
    }
    case ObjectKind::NurbsCurve:
    {
      const int n = 128;
      const ON_Interval dom = obj.nurbs_val.Domain();
      if (dom.IsIncreasing())
      {
        for (int i = 0; i <= n; ++i)
        {
          const double t = dom.ParameterAt(static_cast<double>(i) / static_cast<double>(n));
          points.push_back(obj.nurbs_val.PointAt(t));
        }
      }
      break;
    }
    case ObjectKind::Plane:
    {
      const double s = 10.0;
      const ON_Plane& pl = obj.plane_val;
      points.push_back(pl.PointAt(-s, -s));
      points.push_back(pl.PointAt(s, -s));
      points.push_back(pl.PointAt(s, s));
      points.push_back(pl.PointAt(-s, s));
      points.push_back(pl.PointAt(-s, -s));
      break;
    }
    case ObjectKind::Point:
    default:
      break;
  }
  return points;
}

/** Triangulate an ON_Mesh into flat-shaded vertices (triangle list). */
void AppendMeshTriangles(const ON_Mesh& mesh, const TL_Viewport& vp, const ViewportRect& rect,
                         const DepthWindow& window, const float* color,
                         std::vector<MeshVertex>& out, std::size_t& triangle_count)
{
  const int face_count = mesh.m_F.Count();
  for (int fi = 0; fi < face_count; ++fi)
  {
    const ON_MeshFace& face = mesh.m_F[fi];
    const bool quad = face.IsQuad();
    const int indices[2][3] = {
      { face.vi[0], face.vi[1], face.vi[2] },
      { face.vi[0], face.vi[2], face.vi[3] },
    };
    const int tri_count = quad ? 2 : 1;

    for (int t = 0; t < tri_count; ++t)
    {
      ON_3dPoint p[3];
      ON_2dPoint s[3];
      double d[3];
      bool visible = true;
      for (int k = 0; k < 3; ++k)
      {
        const int vi = indices[t][k];
        if (vi < 0 || vi >= mesh.VertexCount())
        {
          visible = false;
          break;
        }
        p[k] = mesh.m_V[vi];
        if (!vp.WorldToScreen(p[k], s[k]))
          visible = false;
        // Reject triangles that fall completely outside the viewport rectangle.
        if (s[k].x < rect.left - 2.0 * vp.m_width || s[k].x > rect.right + 2.0 * vp.m_width ||
            s[k].y < rect.top - 2.0 * vp.m_height || s[k].y > rect.bottom + 2.0 * vp.m_height)
          visible = false;
        d[k] = DepthOf(vp, p[k], window);
      }
      if (!visible)
        continue;

      ON_3dVector face_normal = ON_CrossProduct(p[1] - p[0], p[2] - p[0]);
      if (!face_normal.Unitize())
        continue;

      const bool have_vertex_normals = (mesh.m_N.Count() == mesh.VertexCount());
      // Decide the facing once for the whole triangle: flipping per vertex makes two
      // corners disagree and shows up as a diagonal shading seam across the quad.
      const bool flip = (ON_DotProduct(face_normal, vp.m_camera_dir) > 0.0);

      for (int k = 0; k < 3; ++k)
      {
        const int vi = indices[t][k];
        ON_3dVector normal = face_normal;
        if (have_vertex_normals)
        {
          const ON_3dVector& candidate = mesh.m_N[vi];
          if (candidate.x != 0.0f || candidate.y != 0.0f || candidate.z != 0.0f)
          {
            ON_3dVector smooth = candidate;
            if (smooth.Unitize())
              normal = smooth;
          }
        }

        // Keep the lighting two-sided so the inside of an open surface is not black.
        if (flip)
          normal = -normal;

        out.push_back({ { static_cast<float>(s[k].x), static_cast<float>(s[k].y),
                          static_cast<float>(d[k]) },
                        { static_cast<float>(normal.x), static_cast<float>(normal.y),
                          static_cast<float>(normal.z) },
                        { color[0], color[1], color[2], color[3] } });
      }
      ++triangle_count;
    }
  }
}

// ---- Vulkan helpers --------------------------------------------------------

uint32_t FindMemoryType(VkPhysicalDevice physical, uint32_t type_bits,
                        VkMemoryPropertyFlags properties)
{
  VkPhysicalDeviceMemoryProperties mem_props{};
  vkGetPhysicalDeviceMemoryProperties(physical, &mem_props);
  for (uint32_t i = 0; i < mem_props.memoryTypeCount; ++i)
  {
    if ((type_bits & (1u << i)) &&
        (mem_props.memoryTypes[i].propertyFlags & properties) == properties)
      return i;
  }
  return UINT32_MAX;
}

bool FormatSupportsDepth(VkPhysicalDevice physical, VkFormat format)
{
  VkFormatProperties props{};
  vkGetPhysicalDeviceFormatProperties(physical, format, &props);
  return (props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0;
}

VkSampleCountFlagBits PickSampleCount(VkPhysicalDevice physical, int requested)
{
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(physical, &props);
  const VkSampleCountFlags counts = props.limits.framebufferColorSampleCounts &
                                    props.limits.framebufferDepthSampleCounts;
  if (requested >= 8 && (counts & VK_SAMPLE_COUNT_8_BIT))
    return VK_SAMPLE_COUNT_8_BIT;
  if (requested >= 4 && (counts & VK_SAMPLE_COUNT_4_BIT))
    return VK_SAMPLE_COUNT_4_BIT;
  if (requested >= 2 && (counts & VK_SAMPLE_COUNT_2_BIT))
    return VK_SAMPLE_COUNT_2_BIT;
  return VK_SAMPLE_COUNT_1_BIT;
}

#endif // OPENTL_HAS_VULKAN

} // anonymous namespace

// ============================================================================
// Renderer
// ============================================================================

bool TL_VulkanFourViewRenderer::RenderScene(
  const opentl::mvp::SceneDocument& scene,
  const TL_FourViewLayout& layout,
  TL_VulkanRenderResult& out_result,
  const std::vector<const ON_Mesh*>& extra_meshes,
  const TL_VulkanRenderOptions& options
) const
{
  out_result = TL_VulkanRenderResult();
  const int width = layout.CanvasWidth();
  const int height = layout.CanvasHeight();
  out_result.width = width;
  out_result.height = height;
  out_result.theme_name = "Catppuccin Mocha";

  // ---- world bounds (scene + extra meshes) for depth normalisation ---------
  ON_BoundingBox bounds = scene.GetSceneBoundingBox();
  for (const ON_Mesh* mesh : extra_meshes)
  {
    if (nullptr == mesh)
      continue;
    ON_BoundingBox mesh_box;
    if (mesh->GetBoundingBox(mesh_box) && mesh_box.IsValid())
      bounds.Union(mesh_box);
  }
  if (!bounds.IsValid())
  {
    bounds.Set(ON_3dPoint(-1.0, -1.0, -1.0));
    bounds.Set(ON_3dPoint(1.0, 1.0, 1.0), true);
  }

  // ---- geometry -----------------------------------------------------------
  std::vector<LineVertex> lines;
  std::vector<MeshVertex> triangles;
  lines.reserve(1 << 16);
  triangles.reserve(1 << 14);
  std::size_t triangle_count = 0;

  for (int vi = 0; vi < layout.Count(); ++vi)
  {
    const TL_Viewport& vp = layout.View(vi);

    ViewportRect rect;
    rect.left = static_cast<float>(vp.m_left) + 0.5f;
    rect.top = static_cast<float>(vp.m_top) + 0.5f;
    rect.right = static_cast<float>(vp.m_left + vp.m_width) - 0.5f;
    rect.bottom = static_cast<float>(vp.m_top + vp.m_height) - 0.5f;

    // Depth window from the projected bounds of the scene.
    DepthWindow window;
    {
      ON_3dPoint corners[8];
      // ON_BoundingBox::GetCorners returns bool: using it as a count made the depth
      // window collapse to a single corner, which flattened every depth value and
      // turned the depth test into painter's order.
      const int n = bounds.GetCorners(corners) ? 8 : 0;
      const ON_3dPoint origin = (TL_PROJECTION_PERSPECTIVE == vp.m_projection)
        ? vp.CameraLocation() : vp.m_target;
      double dmin = ON_DBL_MAX;
      double dmax = -ON_DBL_MAX;
      for (int c = 0; c < n; ++c)
      {
        const double d = ON_DotProduct(corners[c] - origin, vp.m_camera_dir);
        if (!ON_IsValid(d))
          continue;
        dmin = std::min(dmin, d);
        dmax = std::max(dmax, d);
      }
      if (!(dmax > dmin))
      {
        dmin = -1.0;
        dmax = 1.0;
      }
      const double pad = std::max(1e-6, 0.05 * (dmax - dmin));
      window.near_d = dmin - pad;
      window.far_d = dmax + pad;
      window.span = window.far_d - window.near_d;
      if (std::getenv("OPENTL_RENDER_DEBUG") != nullptr)
      {
        std::printf("[dbg] view=%d corners=%d d=[%.4f, %.4f] window=[%.4f, %.4f] span=%.4f\n",
                    vi, n, dmin, dmax, window.near_d, window.far_d, window.span);
      }
    }

    EmitGrid(vp, rect, window, lines);

    // Shaded meshes first (they write depth), wireframe afterwards.
    if (options.enable_shaded_meshes)
    {
      for (const ON_Mesh* mesh : extra_meshes)
      {
        if (nullptr == mesh)
          continue;
        AppendMeshTriangles(*mesh, vp, rect, window, kMeshColor, triangles, triangle_count);
      }
    }

    // Feature edges (silhouette + creases) sit on top of the shading, like a technical
    // illustration: without them a smooth-shaded solid has no readable outline.
    if (options.enable_feature_edges)
    {
      const double crease_radians =
        options.crease_angle_degrees * ON_PI / 180.0;
      for (const ON_Mesh* mesh : extra_meshes)
      {
        if (nullptr == mesh)
          continue;
        ON_SimpleArray<ON_Line> feature_lines;
        TL_ExtractMeshFeatureEdges(*mesh, vp.m_camera_dir, crease_radians, feature_lines, nullptr,
                                   true);
        for (int k = 0; k < feature_lines.Count(); ++k)
        {
          const std::vector<ON_3dPoint> segment = { feature_lines[k].from, feature_lines[k].to };
          EmitWorldPolyline(vp, rect, window, segment, kFeatureEdge, lines,
                            /*bias_towards_eye=*/true);
        }
        out_result.feature_edge_count += feature_lines.Count();
      }
    }

    for (const auto& obj : scene.Objects())
    {
      const auto* layer = scene.GetLayer(obj.layer_index);
      if (nullptr != layer && !layer->visible)
        continue;

      const float* color = kGeometryFallback;
      float layer_color[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
      bool use_layer_color = false;
      if (nullptr != layer)
      {
        const float r = static_cast<float>(layer->color_r) / 255.0f;
        const float g = static_cast<float>(layer->color_g) / 255.0f;
        const float b = static_cast<float>(layer->color_b) / 255.0f;
        if (r + g + b > 0.05f)
        {
          layer_color[0] = r;
          layer_color[1] = g;
          layer_color[2] = b;
          layer_color[3] = 1.0f;
          use_layer_color = true;
        }
      }
      if (use_layer_color)
        color = layer_color;

      if (opentl::mvp::ObjectKind::Point == obj.kind)
      {
        ON_2dPoint s = ON_2dPoint::Origin;
        if (vp.WorldToScreen(obj.point_val, s) &&
            s.x >= rect.left && s.x <= rect.right && s.y >= rect.top && s.y <= rect.bottom)
        {
          const double depth = std::max(0.0, DepthOf(vp, obj.point_val, window) - 2e-4);
          EmitSegment(lines, ON_2dPoint(s.x - 4.0, s.y), ON_2dPoint(s.x + 4.0, s.y), depth, depth,
                      color);
          EmitSegment(lines, ON_2dPoint(s.x, s.y - 4.0), ON_2dPoint(s.x, s.y + 4.0), depth, depth,
                      color);
        }
        continue;
      }

      const std::vector<ON_3dPoint> polyline = CurvePolyline(obj);
      EmitWorldPolyline(vp, rect, window, polyline, color, lines, /*bias_towards_eye=*/true);
    }

    // Viewport panel background: a thin inset border pair in Surface2 + Lavender accent
    // strip along the top edge gives each panel the Catppuccin "window" look.
    EmitSegment(lines, ON_2dPoint(rect.left, rect.top), ON_2dPoint(rect.right, rect.top), 0.0, 0.0,
                kBorder);
    EmitSegment(lines, ON_2dPoint(rect.right, rect.top), ON_2dPoint(rect.right, rect.bottom), 0.0,
                0.0, kBorder);
    EmitSegment(lines, ON_2dPoint(rect.right, rect.bottom), ON_2dPoint(rect.left, rect.bottom), 0.0,
                0.0, kBorder);
    EmitSegment(lines, ON_2dPoint(rect.left, rect.bottom), ON_2dPoint(rect.left, rect.top), 0.0, 0.0,
                kBorder);

    // 2px accent strip under the title row.
    EmitSegment(lines, ON_2dPoint(rect.left + 1.0, rect.top + 1.0),
                ON_2dPoint(rect.right - 1.0, rect.top + 1.0), 0.0, 0.0, kLabel);

    std::string label = vp.m_name;
    for (char& ch : label)
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    EmitLabel(lines, label.c_str(), rect.left + 10.0, rect.top + 10.0, 2.0, kLabel);

    // Origin marker lives in world space, so it must be depth-tested like geometry
    // (borders and labels stay screen-space on purpose).
    ON_2dPoint origin_screen = ON_2dPoint::Origin;
    const ON_3dPoint world_origin(0.0, 0.0, 0.0);
    if (vp.WorldToScreen(world_origin, origin_screen) &&
        origin_screen.x >= rect.left && origin_screen.x <= rect.right &&
        origin_screen.y >= rect.top && origin_screen.y <= rect.bottom)
    {
      const double origin_depth =
        std::max(0.0, DepthOf(vp, world_origin, window) - 2e-4);
      EmitSegment(lines, ON_2dPoint(origin_screen.x - 5.0, origin_screen.y),
                  ON_2dPoint(origin_screen.x + 5.0, origin_screen.y), origin_depth, origin_depth,
                  kOrigin);
      EmitSegment(lines, ON_2dPoint(origin_screen.x, origin_screen.y - 5.0),
                  ON_2dPoint(origin_screen.x, origin_screen.y + 5.0), origin_depth, origin_depth,
                  kOrigin);
    }
  }

  out_result.vertex_count = lines.size() / 2;
  out_result.triangle_count = triangle_count;

#if !defined(OPENTL_HAS_VULKAN)
  out_result.error = "OpenTL was built without Vulkan support (find_package(Vulkan) failed)";
  return false;
#else
  // ---- instance -----------------------------------------------------------
  VkApplicationInfo app_info{};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = "OpenTL FourView";
  app_info.applicationVersion = VK_MAKE_VERSION(0, 4, 0);
  app_info.pEngineName = "OpenTL";
  app_info.engineVersion = VK_MAKE_VERSION(0, 4, 0);
  app_info.apiVersion = VK_API_VERSION_1_2;

  std::vector<const char*> instance_extensions;
  instance_extensions.push_back("VK_KHR_surface");
#if defined(__APPLE__)
  instance_extensions.push_back("VK_KHR_portability_enumeration");
  instance_extensions.push_back("VK_KHR_get_physical_device_properties2");
#endif

  VkInstanceCreateInfo instance_info{};
  instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instance_info.pApplicationInfo = &app_info;
  instance_info.enabledExtensionCount = static_cast<uint32_t>(instance_extensions.size());
  instance_info.ppEnabledExtensionNames = instance_extensions.data();
#if defined(__APPLE__)
  instance_info.flags = VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif

  VkInstance instance = VK_NULL_HANDLE;
  if (VK_SUCCESS != vkCreateInstance(&instance_info, nullptr, &instance))
  {
    out_result.error = "vkCreateInstance failed (is a Vulkan ICD installed? e.g. MoltenVK)";
    return false;
  }

  uint32_t device_count = 0;
  vkEnumeratePhysicalDevices(instance, &device_count, nullptr);
  if (0 == device_count)
  {
    vkDestroyInstance(instance, nullptr);
    out_result.error = "no Vulkan physical device found";
    return false;
  }
  std::vector<VkPhysicalDevice> devices(device_count);
  vkEnumeratePhysicalDevices(instance, &device_count, devices.data());
  VkPhysicalDevice physical = devices[0];

  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(physical, &props);
  out_result.device_name = props.deviceName;
  out_result.api_version = props.apiVersion;
  out_result.backend_name = std::string("Vulkan ") +
    std::to_string(VK_VERSION_MAJOR(props.apiVersion)) + "." +
    std::to_string(VK_VERSION_MINOR(props.apiVersion)) + "." +
    std::to_string(VK_VERSION_PATCH(props.apiVersion));

  uint32_t queue_family_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &queue_family_count, nullptr);
  std::vector<VkQueueFamilyProperties> queue_families(queue_family_count);
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &queue_family_count, queue_families.data());

  uint32_t queue_family = UINT32_MAX;
  for (uint32_t i = 0; i < queue_family_count; ++i)
  {
    if (queue_families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)
    {
      queue_family = i;
      break;
    }
  }
  if (UINT32_MAX == queue_family)
  {
    vkDestroyInstance(instance, nullptr);
    out_result.error = "no graphics queue family";
    return false;
  }

  std::vector<const char*> device_extensions;
  uint32_t ext_count = 0;
  vkEnumerateDeviceExtensionProperties(physical, nullptr, &ext_count, nullptr);
  std::vector<VkExtensionProperties> available(ext_count);
  vkEnumerateDeviceExtensionProperties(physical, nullptr, &ext_count, available.data());
  for (const auto& e : available)
  {
    if (0 == std::strcmp(e.extensionName, "VK_KHR_portability_subset"))
      device_extensions.push_back("VK_KHR_portability_subset");
  }

  const float priority = 1.0f;
  VkDeviceQueueCreateInfo queue_info{};
  queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  queue_info.queueFamilyIndex = queue_family;
  queue_info.queueCount = 1;
  queue_info.pQueuePriorities = &priority;

  VkPhysicalDeviceFeatures features{};
  features.wideLines = VK_FALSE;       // keep the feature set conservative
  features.fillModeNonSolid = VK_FALSE;

  VkDeviceCreateInfo device_info{};
  device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  device_info.queueCreateInfoCount = 1;
  device_info.pQueueCreateInfos = &queue_info;
  device_info.pEnabledFeatures = &features;
  device_info.enabledExtensionCount = static_cast<uint32_t>(device_extensions.size());
  device_info.ppEnabledExtensionNames = device_extensions.empty() ? nullptr : device_extensions.data();

  VkDevice device = VK_NULL_HANDLE;
  if (VK_SUCCESS != vkCreateDevice(physical, &device_info, nullptr, &device))
  {
    vkDestroyInstance(instance, nullptr);
    out_result.error = "vkCreateDevice failed";
    return false;
  }
  VkQueue queue = VK_NULL_HANDLE;
  vkGetDeviceQueue(device, queue_family, 0, &queue);

  // ---- formats / MSAA -----------------------------------------------------
  const VkSampleCountFlagBits samples = PickSampleCount(physical, options.sample_count);
  const int sample_int = static_cast<int>(samples);
  out_result.msaa_samples = sample_int;

  VkFormat depth_format = VK_FORMAT_UNDEFINED;
  if (options.enable_depth)
  {
    const VkFormat candidates[] = { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D24_UNORM_S8_UINT,
                                    VK_FORMAT_D16_UNORM };
    for (VkFormat candidate : candidates)
    {
      if (FormatSupportsDepth(physical, candidate))
      {
        depth_format = candidate;
        break;
      }
    }
  }
  out_result.depth_enabled = (VK_FORMAT_UNDEFINED != depth_format);

  // ---- images -------------------------------------------------------------
  struct ImageBundle
  {
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
  };

  VkFormat color_format = VK_FORMAT_R8G8B8A8_UNORM;

  auto create_image = [&](uint32_t w, uint32_t h, VkFormat format, VkImageUsageFlags usage,
                          VkSampleCountFlagBits sample_count, bool depth) -> ImageBundle {
    ImageBundle bundle;
    VkImageCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    info.imageType = VK_IMAGE_TYPE_2D;
    info.format = format;
    info.extent = { w, h, 1 };
    info.mipLevels = 1;
    info.arrayLayers = 1;
    info.samples = sample_count;
    info.tiling = VK_IMAGE_TILING_OPTIMAL;
    info.usage = usage;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (VK_SUCCESS != vkCreateImage(device, &info, nullptr, &bundle.image))
      return bundle;

    VkMemoryRequirements req{};
    vkGetImageMemoryRequirements(device, bundle.image, &req);
    const uint32_t type = FindMemoryType(physical, req.memoryTypeBits,
                                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (UINT32_MAX == type)
      return bundle;

    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = type;
    if (VK_SUCCESS != vkAllocateMemory(device, &alloc, nullptr, &bundle.memory) ||
        VK_SUCCESS != vkBindImageMemory(device, bundle.image, bundle.memory, 0))
      return bundle;

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = bundle.image;
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = format;
    view_info.subresourceRange = { depth ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT,
                                   0, 1, 0, 1 };
    vkCreateImageView(device, &view_info, nullptr, &bundle.view);
    return bundle;
  };

  const bool multisampled = samples != VK_SAMPLE_COUNT_1_BIT;
  ImageBundle color_ms = {};
  ImageBundle resolve = {};
  ImageBundle depth = {};

  resolve = create_image(static_cast<uint32_t>(width), static_cast<uint32_t>(height), color_format,
                         VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                         multisampled ? VK_SAMPLE_COUNT_1_BIT : samples, false);
  if (multisampled)
  {
    color_ms = create_image(static_cast<uint32_t>(width), static_cast<uint32_t>(height),
                            color_format, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, samples, false);
  }
  if (out_result.depth_enabled)
  {
    depth = create_image(static_cast<uint32_t>(width), static_cast<uint32_t>(height), depth_format,
                         VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, samples, true);
    if (VK_NULL_HANDLE == depth.view)
    {
      out_result.depth_enabled = false;
      depth_format = VK_FORMAT_UNDEFINED;
    }
  }

  if (VK_NULL_HANDLE == resolve.view || (multisampled && VK_NULL_HANDLE == color_ms.view))
  {
    out_result.error = "offscreen image allocation failed";
    vkDestroyDevice(device, nullptr);
    vkDestroyInstance(instance, nullptr);
    return false;
  }

  // ---- render pass (MSAA + depth + resolve, or the 1-sample variants) -----
  VkAttachmentDescription attachments[3]{};
  uint32_t attachment_count = 0;
  uint32_t depth_attachment_index = UINT32_MAX;

  // 0: multisample (or single-sample) colour
  attachments[attachment_count].format = color_format;
  attachments[attachment_count].samples = samples;
  attachments[attachment_count].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachments[attachment_count].storeOp = multisampled ? VK_ATTACHMENT_STORE_OP_DONT_CARE
                                                       : VK_ATTACHMENT_STORE_OP_STORE;
  attachments[attachment_count].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  attachments[attachment_count].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[attachment_count].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  attachments[attachment_count].finalLayout = multisampled ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
                                                           : VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
  const uint32_t color_index = attachment_count++;

  if (out_result.depth_enabled)
  {
    attachments[attachment_count].format = depth_format;
    attachments[attachment_count].samples = samples;
    attachments[attachment_count].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments[attachment_count].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[attachment_count].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[attachment_count].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[attachment_count].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[attachment_count].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depth_attachment_index = attachment_count++;
  }

  uint32_t resolve_index = UINT32_MAX;
  if (multisampled)
  {
    attachments[attachment_count].format = color_format;
    attachments[attachment_count].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[attachment_count].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[attachment_count].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[attachment_count].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[attachment_count].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[attachment_count].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[attachment_count].finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    resolve_index = attachment_count++;
  }

  VkAttachmentReference color_ref{ color_index, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
  VkAttachmentReference resolve_ref{ resolve_index, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
  VkAttachmentReference depth_ref{ depth_attachment_index,
                                  VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };

  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &color_ref;
  subpass.pResolveAttachments = multisampled ? &resolve_ref : nullptr;
  subpass.pDepthStencilAttachment = out_result.depth_enabled ? &depth_ref : nullptr;

  VkSubpassDependency dependency{};
  dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
  dependency.dstSubpass = 0;
  dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                            VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                            VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                             VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

  VkRenderPassCreateInfo rp_info{};
  rp_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  rp_info.attachmentCount = attachment_count;
  rp_info.pAttachments = attachments;
  rp_info.subpassCount = 1;
  rp_info.pSubpasses = &subpass;
  rp_info.dependencyCount = 1;
  rp_info.pDependencies = &dependency;
  VkRenderPass render_pass = VK_NULL_HANDLE;
  if (VK_SUCCESS != vkCreateRenderPass(device, &rp_info, nullptr, &render_pass))
  {
    out_result.error = "vkCreateRenderPass failed";
    vkDestroyDevice(device, nullptr);
    vkDestroyInstance(instance, nullptr);
    return false;
  }

  VkImageView framebuffer_views[3];
  uint32_t framebuffer_view_count = 0;
  framebuffer_views[framebuffer_view_count++] = multisampled ? color_ms.view : resolve.view;
  if (out_result.depth_enabled)
    framebuffer_views[framebuffer_view_count++] = depth.view;
  if (multisampled)
    framebuffer_views[framebuffer_view_count++] = resolve.view;

  VkFramebufferCreateInfo fb_info{};
  fb_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
  fb_info.renderPass = render_pass;
  fb_info.attachmentCount = framebuffer_view_count;
  fb_info.pAttachments = framebuffer_views;
  fb_info.width = static_cast<uint32_t>(width);
  fb_info.height = static_cast<uint32_t>(height);
  fb_info.layers = 1;
  VkFramebuffer framebuffer = VK_NULL_HANDLE;
  vkCreateFramebuffer(device, &fb_info, nullptr, &framebuffer);

  // ---- shader modules -----------------------------------------------------
  auto make_module = [&](const uint32_t* code, size_t size) -> VkShaderModule {
    VkShaderModuleCreateInfo sm{};
    sm.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    sm.codeSize = size;
    sm.pCode = code;
    VkShaderModule module = VK_NULL_HANDLE;
    vkCreateShaderModule(device, &sm, nullptr, &module);
    return module;
  };

  VkShaderModule mesh_vert = make_module(kMeshVertSpv, sizeof(kMeshVertSpv));
  VkShaderModule line_vert = make_module(kLineVertSpv, sizeof(kLineVertSpv));
  VkShaderModule frag = make_module(kPlainFragSpv, sizeof(kPlainFragSpv));

  VkPushConstantRange push_range{};
  push_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  push_range.offset = 0;
  push_range.size = 2 * sizeof(float);

  VkPipelineLayoutCreateInfo layout_info{};
  layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  layout_info.pushConstantRangeCount = 1;
  layout_info.pPushConstantRanges = &push_range;
  VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
  vkCreatePipelineLayout(device, &layout_info, nullptr, &pipeline_layout);

  VkViewport vk_viewport{};
  vk_viewport.x = 0.0f;
  vk_viewport.y = 0.0f;
  vk_viewport.width = static_cast<float>(width);
  vk_viewport.height = static_cast<float>(height);
  vk_viewport.minDepth = 0.0f;
  vk_viewport.maxDepth = 1.0f;

  VkRect2D scissor{};
  scissor.offset = { 0, 0 };
  scissor.extent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height) };

  VkPipelineViewportStateCreateInfo viewport_state{};
  viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewport_state.viewportCount = 1;
  viewport_state.pViewports = &vk_viewport;
  viewport_state.scissorCount = 1;
  viewport_state.pScissors = &scissor;

  VkPipelineMultisampleStateCreateInfo multisample{};
  multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample.rasterizationSamples = samples;

  VkPipelineDepthStencilStateCreateInfo depth_state{};
  depth_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depth_state.depthTestEnable = out_result.depth_enabled ? VK_TRUE : VK_FALSE;
  depth_state.depthWriteEnable = VK_TRUE;
  depth_state.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
  depth_state.depthBoundsTestEnable = VK_FALSE;
  depth_state.stencilTestEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState blend_attachment{};
  blend_attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
    VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  blend_attachment.blendEnable = VK_FALSE;

  VkPipelineColorBlendStateCreateInfo blend{};
  blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  blend.attachmentCount = 1;
  blend.pAttachments = &blend_attachment;

  // ---- pipelines ----------------------------------------------------------
  // Mesh pipeline: TRIANGLE_LIST, pos(3)+normal(3)+color(4)
  VkPipeline mesh_pipeline = VK_NULL_HANDLE;
  {
    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = mesh_vert;
    stages[0].pName = "main";
    stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = frag;
    stages[1].pName = "main";

    VkVertexInputBindingDescription binding{ 0, sizeof(MeshVertex),
                                             VK_VERTEX_INPUT_RATE_VERTEX };
    VkVertexInputAttributeDescription attributes[3]{};
    attributes[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(MeshVertex, pos) };
    attributes[1] = { 1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(MeshVertex, normal) };
    attributes[2] = { 2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(MeshVertex, color) };

    VkPipelineVertexInputStateCreateInfo vertex_input{};
    vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertex_input.vertexBindingDescriptionCount = 1;
    vertex_input.pVertexBindingDescriptions = &binding;
    vertex_input.vertexAttributeDescriptionCount = 3;
    vertex_input.pVertexAttributeDescriptions = attributes;

    VkPipelineInputAssemblyStateCreateInfo input_assembly{};
    input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineRasterizationStateCreateInfo raster{};
    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = options.cull_backfaces ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    raster.lineWidth = 1.0f;

    VkGraphicsPipelineCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    info.stageCount = 2;
    info.pStages = stages;
    info.pVertexInputState = &vertex_input;
    info.pInputAssemblyState = &input_assembly;
    info.pViewportState = &viewport_state;
    info.pRasterizationState = &raster;
    info.pMultisampleState = &multisample;
    info.pDepthStencilState = &depth_state;
    info.pColorBlendState = &blend;
    info.layout = pipeline_layout;
    info.renderPass = render_pass;
    info.subpass = 0;
    vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &mesh_pipeline);
  }

  // Line pipeline: LINE_LIST, pos(3)+color(4)
  VkPipeline line_pipeline = VK_NULL_HANDLE;
  {
    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = line_vert;
    stages[0].pName = "main";
    stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = frag;
    stages[1].pName = "main";

    VkVertexInputBindingDescription binding{ 0, sizeof(LineVertex),
                                             VK_VERTEX_INPUT_RATE_VERTEX };
    VkVertexInputAttributeDescription attributes[2]{};
    attributes[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(LineVertex, pos) };
    attributes[1] = { 1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(LineVertex, color) };

    VkPipelineVertexInputStateCreateInfo vertex_input{};
    vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertex_input.vertexBindingDescriptionCount = 1;
    vertex_input.pVertexBindingDescriptions = &binding;
    vertex_input.vertexAttributeDescriptionCount = 2;
    vertex_input.pVertexAttributeDescriptions = attributes;

    VkPipelineInputAssemblyStateCreateInfo input_assembly{};
    input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

    VkPipelineRasterizationStateCreateInfo raster{};
    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = options.cull_backfaces ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    raster.lineWidth = 1.0f;

    VkGraphicsPipelineCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    info.stageCount = 2;
    info.pStages = stages;
    info.pVertexInputState = &vertex_input;
    info.pInputAssemblyState = &input_assembly;
    info.pViewportState = &viewport_state;
    info.pRasterizationState = &raster;
    info.pMultisampleState = &multisample;
    info.pDepthStencilState = &depth_state;
    info.pColorBlendState = &blend;
    info.layout = pipeline_layout;
    info.renderPass = render_pass;
    info.subpass = 0;
    vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &line_pipeline);
  }

  if (VK_NULL_HANDLE == mesh_pipeline || VK_NULL_HANDLE == line_pipeline)
  {
    out_result.error = "vkCreateGraphicsPipelines failed";
    vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
    vkDestroyShaderModule(device, mesh_vert, nullptr);
    vkDestroyShaderModule(device, line_vert, nullptr);
    vkDestroyShaderModule(device, frag, nullptr);
    vkDestroyFramebuffer(device, framebuffer, nullptr);
    vkDestroyRenderPass(device, render_pass, nullptr);
    vkDestroyDevice(device, nullptr);
    vkDestroyInstance(instance, nullptr);
    return false;
  }

  // ---- vertex buffers -----------------------------------------------------
  struct BufferBundle
  {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    void* mapped = nullptr;
  };

  auto create_host_buffer = [&](VkDeviceSize size, VkBufferUsageFlags usage) -> BufferBundle {
    BufferBundle bundle;
    VkBufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    info.size = std::max<VkDeviceSize>(size, 16);
    info.usage = usage;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (VK_SUCCESS != vkCreateBuffer(device, &info, nullptr, &bundle.buffer))
      return bundle;
    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, bundle.buffer, &req);
    const uint32_t type = FindMemoryType(physical, req.memoryTypeBits,
                                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (UINT32_MAX == type)
      return bundle;
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = type;
    vkAllocateMemory(device, &alloc, nullptr, &bundle.memory);
    vkBindBufferMemory(device, bundle.buffer, bundle.memory, 0);
    vkMapMemory(device, bundle.memory, 0, VK_WHOLE_SIZE, 0, &bundle.mapped);
    return bundle;
  };

  BufferBundle line_buffer = create_host_buffer(
    static_cast<VkDeviceSize>(lines.size()) * sizeof(LineVertex),
    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
  BufferBundle mesh_buffer = create_host_buffer(
    static_cast<VkDeviceSize>(triangles.size()) * sizeof(MeshVertex),
    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);

  if (nullptr != line_buffer.mapped && !lines.empty())
    std::memcpy(line_buffer.mapped, lines.data(), lines.size() * sizeof(LineVertex));
  if (nullptr != mesh_buffer.mapped && !triangles.empty())
    std::memcpy(mesh_buffer.mapped, triangles.data(), triangles.size() * sizeof(MeshVertex));

  BufferBundle readback = create_host_buffer(
    static_cast<VkDeviceSize>(width) * height * 4, VK_BUFFER_USAGE_TRANSFER_DST_BIT);

  // ---- command buffer -----------------------------------------------------
  VkCommandPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pool_info.queueFamilyIndex = queue_family;
  VkCommandPool command_pool = VK_NULL_HANDLE;
  vkCreateCommandPool(device, &pool_info, nullptr, &command_pool);

  VkCommandBufferAllocateInfo cmd_alloc{};
  cmd_alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  cmd_alloc.commandPool = command_pool;
  cmd_alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  cmd_alloc.commandBufferCount = 1;
  VkCommandBuffer command_buffer = VK_NULL_HANDLE;
  vkAllocateCommandBuffers(device, &cmd_alloc, &command_buffer);

  VkCommandBufferBeginInfo begin_info{};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  bool ok = VK_SUCCESS == vkBeginCommandBuffer(command_buffer, &begin_info);

  VkClearValue clears[3]{};
  clears[0].color.float32[0] = kBackground[0];
  clears[0].color.float32[1] = kBackground[1];
  clears[0].color.float32[2] = kBackground[2];
  clears[0].color.float32[3] = 1.0f;
  uint32_t clear_count = 1;
  if (out_result.depth_enabled)
  {
    clears[clear_count].depthStencil.depth = 1.0f;
    clears[clear_count].depthStencil.stencil = 0;
    ++clear_count;
  }
  if (multisampled)
  {
    clears[clear_count].color = clears[0].color;
    ++clear_count;
  }

  VkRenderPassBeginInfo rp_begin{};
  rp_begin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  rp_begin.renderPass = render_pass;
  rp_begin.framebuffer = framebuffer;
  rp_begin.renderArea.offset = { 0, 0 };
  rp_begin.renderArea.extent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height) };
  rp_begin.clearValueCount = clear_count;
  rp_begin.pClearValues = clears;

  if (ok)
  {
    vkCmdBeginRenderPass(command_buffer, &rp_begin, VK_SUBPASS_CONTENTS_INLINE);
    const float canvas[2] = { static_cast<float>(width), static_cast<float>(height) };

    if (!triangles.empty())
    {
      vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, mesh_pipeline);
      VkDeviceSize offset = 0;
      vkCmdBindVertexBuffers(command_buffer, 0, 1, &mesh_buffer.buffer, &offset);
      vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 0,
                         sizeof(canvas), canvas);
      vkCmdDraw(command_buffer, static_cast<uint32_t>(triangles.size()), 1, 0, 0);
    }

    if (!lines.empty())
    {
      vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, line_pipeline);
      VkDeviceSize offset = 0;
      vkCmdBindVertexBuffers(command_buffer, 0, 1, &line_buffer.buffer, &offset);
      vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 0,
                         sizeof(canvas), canvas);
      vkCmdDraw(command_buffer, static_cast<uint32_t>(lines.size()), 1, 0, 0);
    }

    vkCmdEndRenderPass(command_buffer);

    VkBufferImageCopy region{};
    region.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
    region.imageExtent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
    vkCmdCopyImageToBuffer(command_buffer, resolve.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           readback.buffer, 1, &region);
    ok = ok && VK_SUCCESS == vkEndCommandBuffer(command_buffer);
  }

  if (ok)
  {
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &command_buffer;
    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    VkFence fence = VK_NULL_HANDLE;
    vkCreateFence(device, &fence_info, nullptr, &fence);
    ok = VK_SUCCESS == vkQueueSubmit(queue, 1, &submit, fence) &&
         VK_SUCCESS == vkWaitForFences(device, 1, &fence, VK_TRUE,
                                       std::numeric_limits<uint64_t>::max());
    vkDestroyFence(device, fence, nullptr);
  }

  if (ok && nullptr != readback.mapped)
  {
    out_result.rgba.assign(static_cast<size_t>(width) * height * 4, 0);
    std::memcpy(out_result.rgba.data(), readback.mapped,
                static_cast<size_t>(width) * height * 4);
    out_result.success = true;
  }
  else if (ok)
  {
    out_result.error = "readback buffer was not mapped";
  }
  else
  {
    out_result.error = "command buffer execution failed";
  }

  // ---- teardown -----------------------------------------------------------
  vkDeviceWaitIdle(device);
  vkDestroyCommandPool(device, command_pool, nullptr);

  auto destroy_buffer = [&](BufferBundle& bundle) {
    if (nullptr != bundle.mapped)
      vkUnmapMemory(device, bundle.memory);
    if (VK_NULL_HANDLE != bundle.buffer)
      vkDestroyBuffer(device, bundle.buffer, nullptr);
    if (VK_NULL_HANDLE != bundle.memory)
      vkFreeMemory(device, bundle.memory, nullptr);
  };
  destroy_buffer(readback);
  destroy_buffer(mesh_buffer);
  destroy_buffer(line_buffer);

  vkDestroyPipeline(device, mesh_pipeline, nullptr);
  vkDestroyPipeline(device, line_pipeline, nullptr);
  vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
  vkDestroyShaderModule(device, mesh_vert, nullptr);
  vkDestroyShaderModule(device, line_vert, nullptr);
  vkDestroyShaderModule(device, frag, nullptr);
  vkDestroyFramebuffer(device, framebuffer, nullptr);
  vkDestroyRenderPass(device, render_pass, nullptr);

  auto destroy_image = [&](ImageBundle& bundle) {
    if (VK_NULL_HANDLE != bundle.view)
      vkDestroyImageView(device, bundle.view, nullptr);
    if (VK_NULL_HANDLE != bundle.image)
      vkDestroyImage(device, bundle.image, nullptr);
    if (VK_NULL_HANDLE != bundle.memory)
      vkFreeMemory(device, bundle.memory, nullptr);
  };
  destroy_image(depth);
  destroy_image(color_ms);
  destroy_image(resolve);

  vkDestroyDevice(device, nullptr);
  vkDestroyInstance(instance, nullptr);
  return out_result.success;
#endif
}

// ============================================================================
// PNG writer (RGBA8, filter type 0, zlib deflate)
// ============================================================================

namespace {

void PutU32(std::vector<uint8_t>& out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(value & 0xFF));
}

void PutChunk(std::vector<uint8_t>& png, const char type[4], const std::vector<uint8_t>& data)
{
  PutU32(png, static_cast<uint32_t>(data.size()));
  const size_t type_offset = png.size();
  for (int i = 0; i < 4; ++i)
    png.push_back(static_cast<uint8_t>(type[i]));
  png.insert(png.end(), data.begin(), data.end());

  uLong crc = crc32(0L, Z_NULL, 0);
  crc = crc32(crc, png.data() + type_offset, static_cast<uInt>(4 + data.size()));
  PutU32(png, static_cast<uint32_t>(crc));
}

} // anonymous namespace

bool TL_VulkanFourViewRenderer::SavePng(
  const std::string& path,
  int width,
  int height,
  const std::vector<uint8_t>& rgba,
  std::string* error_out
)
{
  if (width <= 0 || height <= 0 || rgba.size() < static_cast<size_t>(width) * height * 4)
  {
    if (nullptr != error_out)
      *error_out = "invalid RGBA buffer";
    return false;
  }

  std::vector<uint8_t> raw;
  raw.reserve(static_cast<size_t>(height) * (static_cast<size_t>(width) * 4 + 1));
  for (int y = 0; y < height; ++y)
  {
    raw.push_back(0); // filter: none
    const uint8_t* row = rgba.data() + static_cast<size_t>(y) * width * 4;
    raw.insert(raw.end(), row, row + static_cast<size_t>(width) * 4);
  }

  uLongf compressed_size = compressBound(static_cast<uLong>(raw.size()));
  std::vector<uint8_t> compressed(compressed_size);
  const int zr = compress2(compressed.data(), &compressed_size, raw.data(),
                           static_cast<uLong>(raw.size()), 9);
  if (Z_OK != zr)
  {
    if (nullptr != error_out)
      *error_out = "zlib compress2 failed";
    return false;
  }
  compressed.resize(compressed_size);

  std::vector<uint8_t> png;
  png.reserve(compressed.size() + 128);
  const uint8_t signature[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
  png.insert(png.end(), signature, signature + 8);

  std::vector<uint8_t> ihdr;
  PutU32(ihdr, static_cast<uint32_t>(width));
  PutU32(ihdr, static_cast<uint32_t>(height));
  ihdr.push_back(8);  // bit depth
  ihdr.push_back(6);  // color type: RGBA
  ihdr.push_back(0);
  ihdr.push_back(0);
  ihdr.push_back(0);
  PutChunk(png, "IHDR", ihdr);
  PutChunk(png, "IDAT", compressed);
  PutChunk(png, "IEND", std::vector<uint8_t>());

  std::ofstream file(path.c_str(), std::ios::binary | std::ios::trunc);
  if (!file)
  {
    if (nullptr != error_out)
      *error_out = "cannot open output file";
    return false;
  }
  file.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
  return file.good();
}
