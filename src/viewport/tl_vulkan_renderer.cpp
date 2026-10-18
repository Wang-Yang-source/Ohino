#include "opentl/viewport/tl_vulkan_renderer.h"

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>

#if defined(OPENTL_HAS_VULKAN)
  #include <vulkan/vulkan.h>
#endif

namespace {

// ============================================================================
// Shaders (GLSL 450 -> SPIR-V, compiled with glslc; see tools/README or
// docs/PROGRESS.md §11 for the exact source)
//
//   #version 450
//   layout(location = 0) in vec2 inPos;      // pixels, canvas top-left origin
//   layout(location = 1) in vec4 inColor;
//   layout(push_constant) uniform PC { vec2 canvasSize; } pc;
//   layout(location = 0) out vec4 fragColor;
//   void main() {
//     vec2 ndc = (inPos / pc.canvasSize) * 2.0 - 1.0;
//     gl_Position = vec4(ndc, 0.0, 1.0);
//     fragColor = inColor;
//   }
// ============================================================================

#if defined(OPENTL_HAS_VULKAN)

const uint32_t kVertSpv[] = {
  0x07230203u, 0x00010000u, 0x000d000bu, 0x0000002du, 0x00000000u, 0x00020011u, 0x00000001u,
  0x0006000bu, 0x00000001u, 0x4c534c47u, 0x6474732eu, 0x3035342eu, 0x00000000u, 0x0003000eu,
  0x00000000u, 0x00000001u, 0x0009000fu, 0x00000000u, 0x00000004u, 0x6e69616du, 0x00000000u,
  0x0000000bu, 0x00000021u, 0x00000029u, 0x0000002bu, 0x00030003u, 0x00000002u, 0x000001c2u,
  0x000a0004u, 0x475f4c47u, 0x4c474f4fu, 0x70635f45u, 0x74735f70u, 0x5f656c79u, 0x656e696cu,
  0x7269645fu, 0x69746365u, 0x00006576u, 0x00080004u, 0x475f4c47u, 0x4c474f4fu, 0x6e695f45u,
  0x64756c63u, 0x69645f65u, 0x74636572u, 0x00657669u, 0x00040005u, 0x00000004u, 0x6e69616du,
  0x00000000u, 0x00030005u, 0x00000009u, 0x0063646eu, 0x00040005u, 0x0000000bu, 0x6f506e69u,
  0x00000073u, 0x00060005u, 0x0000000du, 0x68737550u, 0x736e6f43u, 0x746e6174u, 0x00000073u,
  0x00060006u, 0x0000000du, 0x00000000u, 0x766e6163u, 0x69537361u, 0x0000657au, 0x00030005u,
  0x0000000fu, 0x00006370u, 0x00060005u, 0x0000001fu, 0x505f6c67u, 0x65567265u, 0x78657472u,
  0x00000000u, 0x00060006u, 0x0000001fu, 0x00000000u, 0x505f6c67u, 0x7469736fu, 0x006e6f69u,
  0x00070006u, 0x0000001fu, 0x00000001u, 0x505f6c67u, 0x746e696fu, 0x657a6953u, 0x00000000u,
  0x00070006u, 0x0000001fu, 0x00000002u, 0x435f6c67u, 0x4470696cu, 0x61747369u, 0x0065636eu,
  0x00070006u, 0x0000001fu, 0x00000003u, 0x435f6c67u, 0x446c6c75u, 0x61747369u, 0x0065636eu,
  0x00030005u, 0x00000021u, 0x00000000u, 0x00050005u, 0x00000029u, 0x67617266u, 0x6f6c6f43u,
  0x00000072u, 0x00040005u, 0x0000002bu, 0x6f436e69u, 0x00726f6cu, 0x00040047u, 0x0000000bu,
  0x0000001eu, 0x00000000u, 0x00030047u, 0x0000000du, 0x00000002u, 0x00050048u, 0x0000000du,
  0x00000000u, 0x00000023u, 0x00000000u, 0x00030047u, 0x0000001fu, 0x00000002u, 0x00050048u,
  0x0000001fu, 0x00000000u, 0x0000000bu, 0x00000000u, 0x00050048u, 0x0000001fu, 0x00000001u,
  0x0000000bu, 0x00000001u, 0x00050048u, 0x0000001fu, 0x00000002u, 0x0000000bu, 0x00000003u,
  0x00050048u, 0x0000001fu, 0x00000003u, 0x0000000bu, 0x00000004u, 0x00040047u, 0x00000029u,
  0x0000001eu, 0x00000000u, 0x00040047u, 0x0000002bu, 0x0000001eu, 0x00000001u, 0x00020013u,
  0x00000002u, 0x00030021u, 0x00000003u, 0x00000002u, 0x00030016u, 0x00000006u, 0x00000020u,
  0x00040017u, 0x00000007u, 0x00000006u, 0x00000002u, 0x00040020u, 0x00000008u, 0x00000007u,
  0x00000007u, 0x00040020u, 0x0000000au, 0x00000001u, 0x00000007u, 0x0004003bu, 0x0000000au,
  0x0000000bu, 0x00000001u, 0x0003001eu, 0x0000000du, 0x00000007u, 0x00040020u, 0x0000000eu,
  0x00000009u, 0x0000000du, 0x0004003bu, 0x0000000eu, 0x0000000fu, 0x00000009u, 0x00040015u,
  0x00000010u, 0x00000020u, 0x00000001u, 0x0004002bu, 0x00000010u, 0x00000011u, 0x00000000u,
  0x00040020u, 0x00000012u, 0x00000009u, 0x00000007u, 0x0004002bu, 0x00000006u, 0x00000016u,
  0x40000000u, 0x0004002bu, 0x00000006u, 0x00000018u, 0x3f800000u, 0x00040017u, 0x0000001bu,
  0x00000006u, 0x00000004u, 0x00040015u, 0x0000001cu, 0x00000020u, 0x00000000u, 0x0004002bu,
  0x0000001cu, 0x0000001du, 0x00000001u, 0x0004001cu, 0x0000001eu, 0x00000006u, 0x0000001du,
  0x0006001eu, 0x0000001fu, 0x0000001bu, 0x00000006u, 0x0000001eu, 0x0000001eu, 0x00040020u,
  0x00000020u, 0x00000003u, 0x0000001fu, 0x0004003bu, 0x00000020u, 0x00000021u, 0x00000003u,
  0x0004002bu, 0x00000006u, 0x00000023u, 0x00000000u, 0x00040020u, 0x00000027u, 0x00000003u,
  0x0000001bu, 0x0004003bu, 0x00000027u, 0x00000029u, 0x00000003u, 0x00040020u, 0x0000002au,
  0x00000001u, 0x0000001bu, 0x0004003bu, 0x0000002au, 0x0000002bu, 0x00000001u, 0x00050036u,
  0x00000002u, 0x00000004u, 0x00000000u, 0x00000003u, 0x000200f8u, 0x00000005u, 0x0004003bu,
  0x00000008u, 0x00000009u, 0x00000007u, 0x0004003du, 0x00000007u, 0x0000000cu, 0x0000000bu,
  0x00050041u, 0x00000012u, 0x00000013u, 0x0000000fu, 0x00000011u, 0x0004003du, 0x00000007u,
  0x00000014u, 0x00000013u, 0x00050088u, 0x00000007u, 0x00000015u, 0x0000000cu, 0x00000014u,
  0x0005008eu, 0x00000007u, 0x00000017u, 0x00000015u, 0x00000016u, 0x00050050u, 0x00000007u,
  0x00000019u, 0x00000018u, 0x00000018u, 0x00050083u, 0x00000007u, 0x0000001au, 0x00000017u,
  0x00000019u, 0x0003003eu, 0x00000009u, 0x0000001au, 0x0004003du, 0x00000007u, 0x00000022u,
  0x00000009u, 0x00050051u, 0x00000006u, 0x00000024u, 0x00000022u, 0x00000000u, 0x00050051u,
  0x00000006u, 0x00000025u, 0x00000022u, 0x00000001u, 0x00070050u, 0x0000001bu, 0x00000026u,
  0x00000024u, 0x00000025u, 0x00000023u, 0x00000018u, 0x00050041u, 0x00000027u, 0x00000028u,
  0x00000021u, 0x00000011u, 0x0003003eu, 0x00000028u, 0x00000026u, 0x0004003du, 0x0000001bu,
  0x0000002cu, 0x0000002bu, 0x0003003eu, 0x00000029u, 0x0000002cu, 0x000100fdu, 0x00010038u
};

// #version 450
// layout(location = 0) in vec4 fragColor;
// layout(location = 0) out vec4 outColor;
// void main() { outColor = fragColor; }
const uint32_t kFragSpv[] = {
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

struct Vertex
{
  float pos[2];
  float color[4];
};

struct ViewportRect
{
  float left = 0.0f;
  float top = 0.0f;
  float right = 0.0f;
  float bottom = 0.0f;
};

// ---- 5x7 bitmap font, uppercase A-Z, digits and a few separators --------------

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

const float kBackground[4] = { 0.078f, 0.086f, 0.102f, 1.0f };
const float kGrid[4] = { 0.20f, 0.22f, 0.26f, 1.0f };
const float kGridMajor[4] = { 0.28f, 0.31f, 0.36f, 1.0f };
const float kAxisX[4] = { 0.85f, 0.25f, 0.28f, 1.0f };
const float kAxisY[4] = { 0.30f, 0.75f, 0.35f, 1.0f };
const float kAxisZ[4] = { 0.30f, 0.50f, 0.90f, 1.0f };
const float kBorder[4] = { 0.45f, 0.48f, 0.55f, 1.0f };
const float kLabel[4] = { 0.85f, 0.88f, 0.92f, 1.0f };
const float kCursor[4] = { 0.95f, 0.80f, 0.25f, 1.0f };

void EmitSegment(std::vector<Vertex>& out, const ON_2dPoint& a, const ON_2dPoint& b,
                 const float color[4])
{
  out.push_back({ { static_cast<float>(a.x), static_cast<float>(a.y) },
                  { color[0], color[1], color[2], color[3] } });
  out.push_back({ { static_cast<float>(b.x), static_cast<float>(b.y) },
                  { color[0], color[1], color[2], color[3] } });
}

/** Liang-Barsky clip of a screen-space segment against the viewport rectangle. */
bool ClipSegment(const ON_2dPoint& a, const ON_2dPoint& b, const ViewportRect& r,
                 ON_2dPoint& a_out, ON_2dPoint& b_out)
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
  return true;
}

void EmitWorldPolyline(const TL_Viewport& vp, const ViewportRect& rect,
                       const std::vector<ON_3dPoint>& world_points, const float color[4],
                       std::vector<Vertex>& out)
{
  if (world_points.size() < 2)
    return;

  bool have_prev = false;
  ON_2dPoint prev = ON_2dPoint::Origin;
  for (const ON_3dPoint& p : world_points)
  {
    ON_2dPoint screen = ON_2dPoint::Origin;
    if (!vp.WorldToScreen(p, screen))
    {
      have_prev = false;
      continue;
    }
    if (have_prev)
    {
      ON_2dPoint ca = ON_2dPoint::Origin;
      ON_2dPoint cb = ON_2dPoint::Origin;
      if (ClipSegment(prev, screen, rect, ca, cb))
        EmitSegment(out, ca, cb, color);
    }
    prev = screen;
    have_prev = true;
  }
}

void EmitLabel(std::vector<Vertex>& out, const char* text, double x, double y, double pixel,
               const float color[4])
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
          EmitSegment(out, ON_2dPoint(px, py), ON_2dPoint(px + pixel, py), color);
        }
      }
    }
    cursor += 6.0 * pixel;
  }
}

/**
 * Rhino gives each standard view its own construction plane: Top uses world XY,
 * Front uses XZ and Right uses YZ.  Drawing the XY grid in every viewport would
 * collapse Front/Right to a single line during the orthographic views.
 */
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

/**
 * Grid half-extent in construction-plane units: cover the visible area of a parallel
 * viewport (plus one cell of margin) instead of a fixed +-10, which clipped the grid in
 * views where a flat scene zooms in tight.  Perspective keeps a fixed extent because its
 * visible footprint varies with depth.
 */
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

/** Construction-plane grid plus the three world axes for one viewport. */
void EmitGrid(const TL_Viewport& vp, const ViewportRect& rect, std::vector<Vertex>& out)
{
  const int half = GridHalfExtent(vp);
  const ON_Plane cplane = GridPlaneForView(vp.m_standard_view);

  for (int i = -half; i <= half; ++i)
  {
    const bool major = (0 == (i % 5));
    std::vector<ON_3dPoint> u_line;
    u_line.push_back(cplane.PointAt(i, -half));
    u_line.push_back(cplane.PointAt(i, half));
    if (0 != i)
      EmitWorldPolyline(vp, rect, u_line, major ? kGridMajor : kGrid, out);

    std::vector<ON_3dPoint> v_line;
    v_line.push_back(cplane.PointAt(-half, i));
    v_line.push_back(cplane.PointAt(half, i));
    if (0 != i)
      EmitWorldPolyline(vp, rect, v_line, major ? kGridMajor : kGrid, out);
  }

  // World axes span the same extent so they always reach the viewport edge.
  const double axis = static_cast<double>(half);
  std::vector<ON_3dPoint> x_axis;
  x_axis.push_back(ON_3dPoint(-axis, 0.0, 0.0));
  x_axis.push_back(ON_3dPoint(axis, 0.0, 0.0));
  EmitWorldPolyline(vp, rect, x_axis, kAxisX, out);

  std::vector<ON_3dPoint> y_axis;
  y_axis.push_back(ON_3dPoint(0.0, -axis, 0.0));
  y_axis.push_back(ON_3dPoint(0.0, axis, 0.0));
  EmitWorldPolyline(vp, rect, y_axis, kAxisY, out);

  std::vector<ON_3dPoint> z_axis;
  z_axis.push_back(ON_3dPoint(0.0, 0.0, -axis));
  z_axis.push_back(ON_3dPoint(0.0, 0.0, axis));
  EmitWorldPolyline(vp, rect, z_axis, kAxisZ, out);
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
      const int n = 120;
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
      const int n = 96;
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

// ---- Vulkan helpers --------------------------------------------------------

bool Fail(std::string& error, const char* message)
{
  error = message;
  return false;
}

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

#endif // OPENTL_HAS_VULKAN

} // anonymous namespace

// ============================================================================
// Renderer
// ============================================================================

bool TL_VulkanFourViewRenderer::RenderScene(
  const opentl::mvp::SceneDocument& scene,
  const TL_FourViewLayout& layout,
  TL_VulkanRenderResult& out_result
) const
{
  out_result = TL_VulkanRenderResult();
  const int width = layout.CanvasWidth();
  const int height = layout.CanvasHeight();
  out_result.width = width;
  out_result.height = height;

  // ---- geometry (viewport independent of the graphics API) ----------------
  std::vector<Vertex> vertices;
  vertices.reserve(1 << 16);

  for (int vi = 0; vi < layout.Count(); ++vi)
  {
    const TL_Viewport& vp = layout.View(vi);
    ViewportRect rect;
    rect.left = static_cast<float>(vp.m_left) + 0.5f;
    rect.top = static_cast<float>(vp.m_top) + 0.5f;
    rect.right = static_cast<float>(vp.m_left + vp.m_width) - 0.5f;
    rect.bottom = static_cast<float>(vp.m_top + vp.m_height) - 0.5f;

    EmitGrid(vp, rect, vertices);

    for (const auto& obj : scene.Objects())
    {
      const auto* layer = scene.GetLayer(obj.layer_index);
      if (nullptr != layer && !layer->visible)
        continue;

      float color[4] = { 0.85f, 0.87f, 0.90f, 1.0f };
      if (nullptr != layer)
      {
        color[0] = static_cast<float>(layer->color_r) / 255.0f;
        color[1] = static_cast<float>(layer->color_g) / 255.0f;
        color[2] = static_cast<float>(layer->color_b) / 255.0f;
        if (color[0] + color[1] + color[2] < 0.05f)
        {
          color[0] = 0.85f;
          color[1] = 0.87f;
          color[2] = 0.90f;
        }
      }

      if (opentl::mvp::ObjectKind::Point == obj.kind)
      {
        ON_2dPoint s = ON_2dPoint::Origin;
        if (vp.WorldToScreen(obj.point_val, s) &&
            s.x >= rect.left && s.x <= rect.right && s.y >= rect.top && s.y <= rect.bottom)
        {
          EmitSegment(vertices, ON_2dPoint(s.x - 4.0, s.y), ON_2dPoint(s.x + 4.0, s.y), color);
          EmitSegment(vertices, ON_2dPoint(s.x, s.y - 4.0), ON_2dPoint(s.x, s.y + 4.0), color);
        }
        continue;
      }

      const std::vector<ON_3dPoint> polyline = CurvePolyline(obj);
      EmitWorldPolyline(vp, rect, polyline, color, vertices);
    }

    // Border
    EmitSegment(vertices, ON_2dPoint(rect.left, rect.top), ON_2dPoint(rect.right, rect.top), kBorder);
    EmitSegment(vertices, ON_2dPoint(rect.right, rect.top), ON_2dPoint(rect.right, rect.bottom), kBorder);
    EmitSegment(vertices, ON_2dPoint(rect.right, rect.bottom), ON_2dPoint(rect.left, rect.bottom), kBorder);
    EmitSegment(vertices, ON_2dPoint(rect.left, rect.bottom), ON_2dPoint(rect.left, rect.top), kBorder);

    // Label (uppercase 5x7 bitmap font), inside the top-left corner.
    std::string label = vp.m_name;
    for (char& ch : label)
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    EmitLabel(vertices, label.c_str(), rect.left + 10.0, rect.top + 10.0, 2.0, kLabel);

    // Small orientation cross at the viewport centre.
    const double cx = 0.5 * (rect.left + rect.right);
    const double cy = 0.5 * (rect.top + rect.bottom);
    ON_2dPoint cursor_screen = ON_2dPoint::Origin;
    if (vp.WorldToScreen(ON_3dPoint(0.0, 0.0, 0.0), cursor_screen) &&
        cursor_screen.x >= rect.left && cursor_screen.x <= rect.right &&
        cursor_screen.y >= rect.top && cursor_screen.y <= rect.bottom)
    {
      EmitSegment(vertices, ON_2dPoint(cursor_screen.x - 5.0, cursor_screen.y),
                  ON_2dPoint(cursor_screen.x + 5.0, cursor_screen.y), kCursor);
      EmitSegment(vertices, ON_2dPoint(cursor_screen.x, cursor_screen.y - 5.0),
                  ON_2dPoint(cursor_screen.x, cursor_screen.y + 5.0), kCursor);
    }
    (void)cx;
    (void)cy;
  }

  out_result.vertex_count = vertices.size() / 2;

#if !defined(OPENTL_HAS_VULKAN)
  out_result.error = "OpenTL was built without Vulkan support (find_package(Vulkan) failed)";
  return false;
#else
  // ---- Vulkan instance ----------------------------------------------------
  std::string error;

  VkApplicationInfo app_info{};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = "OpenTL FourView";
  app_info.applicationVersion = VK_MAKE_VERSION(0, 3, 0);
  app_info.pEngineName = "OpenTL";
  app_info.engineVersion = VK_MAKE_VERSION(0, 3, 0);
  app_info.apiVersion = VK_API_VERSION_1_2;

  std::vector<const char*> instance_extensions;
  instance_extensions.push_back("VK_KHR_surface"); // harmless; portability requires it
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
  out_result.backend_name = std::string("Vulkan ") + std::to_string(VK_VERSION_MAJOR(props.apiVersion)) +
    "." + std::to_string(VK_VERSION_MINOR(props.apiVersion)) + "." +
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

  // device extensions: portability subset is mandatory on MoltenVK
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

  VkDeviceCreateInfo device_info{};
  device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  device_info.queueCreateInfoCount = 1;
  device_info.pQueueCreateInfos = &queue_info;
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

  // ---- offscreen color target --------------------------------------------
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
  image_info.extent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  VkImage color_image = VK_NULL_HANDLE;
  if (VK_SUCCESS != vkCreateImage(device, &image_info, nullptr, &color_image))
  {
    vkDestroyDevice(device, nullptr);
    vkDestroyInstance(instance, nullptr);
    out_result.error = "vkCreateImage failed";
    return false;
  }
  VkMemoryRequirements image_req{};
  vkGetImageMemoryRequirements(device, color_image, &image_req);
  const uint32_t image_type = FindMemoryType(physical, image_req.memoryTypeBits,
                                             VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  VkMemoryAllocateInfo image_alloc{};
  image_alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  image_alloc.allocationSize = image_req.size;
  image_alloc.memoryTypeIndex = image_type;
  VkDeviceMemory image_memory = VK_NULL_HANDLE;
  if (UINT32_MAX == image_type ||
      VK_SUCCESS != vkAllocateMemory(device, &image_alloc, nullptr, &image_memory) ||
      VK_SUCCESS != vkBindImageMemory(device, color_image, image_memory, 0))
  {
    vkDestroyImage(device, color_image, nullptr);
    vkDestroyDevice(device, nullptr);
    vkDestroyInstance(instance, nullptr);
    out_result.error = "color image allocation failed";
    return false;
  }

  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = color_image;
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = VK_FORMAT_R8G8B8A8_UNORM;
  view_info.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
  VkImageView color_view = VK_NULL_HANDLE;
  vkCreateImageView(device, &view_info, nullptr, &color_view);

  // ---- render pass --------------------------------------------------------
  VkAttachmentDescription color_attachment{};
  color_attachment.format = VK_FORMAT_R8G8B8A8_UNORM;
  color_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
  color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  color_attachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

  VkAttachmentReference color_ref{};
  color_ref.attachment = 0;
  color_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &color_ref;

  VkSubpassDependency dependency{};
  dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
  dependency.dstSubpass = 0;
  dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

  VkRenderPassCreateInfo rp_info{};
  rp_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  rp_info.attachmentCount = 1;
  rp_info.pAttachments = &color_attachment;
  rp_info.subpassCount = 1;
  rp_info.pSubpasses = &subpass;
  rp_info.dependencyCount = 1;
  rp_info.pDependencies = &dependency;
  VkRenderPass render_pass = VK_NULL_HANDLE;
  vkCreateRenderPass(device, &rp_info, nullptr, &render_pass);

  VkImageView attachments[] = { color_view };
  VkFramebufferCreateInfo fb_info{};
  fb_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
  fb_info.renderPass = render_pass;
  fb_info.attachmentCount = 1;
  fb_info.pAttachments = attachments;
  fb_info.width = static_cast<uint32_t>(width);
  fb_info.height = static_cast<uint32_t>(height);
  fb_info.layers = 1;
  VkFramebuffer framebuffer = VK_NULL_HANDLE;
  vkCreateFramebuffer(device, &fb_info, nullptr, &framebuffer);

  // ---- shader modules -----------------------------------------------------
  VkShaderModule vert_module = VK_NULL_HANDLE;
  VkShaderModule frag_module = VK_NULL_HANDLE;
  {
    VkShaderModuleCreateInfo sm{};
    sm.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    sm.codeSize = sizeof(kVertSpv);
    sm.pCode = kVertSpv;
    vkCreateShaderModule(device, &sm, nullptr, &vert_module);
    sm.codeSize = sizeof(kFragSpv);
    sm.pCode = kFragSpv;
    vkCreateShaderModule(device, &sm, nullptr, &frag_module);
  }

  VkPipelineShaderStageCreateInfo stages[2]{};
  stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  stages[0].module = vert_module;
  stages[0].pName = "main";
  stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  stages[1].module = frag_module;
  stages[1].pName = "main";

  VkVertexInputBindingDescription binding{};
  binding.binding = 0;
  binding.stride = sizeof(Vertex);
  binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  VkVertexInputAttributeDescription attributes[2]{};
  attributes[0].location = 0;
  attributes[0].binding = 0;
  attributes[0].format = VK_FORMAT_R32G32_SFLOAT;
  attributes[0].offset = offsetof(Vertex, pos);
  attributes[1].location = 1;
  attributes[1].binding = 0;
  attributes[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
  attributes[1].offset = offsetof(Vertex, color);

  VkPipelineVertexInputStateCreateInfo vertex_input{};
  vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  vertex_input.vertexBindingDescriptionCount = 1;
  vertex_input.pVertexBindingDescriptions = &binding;
  vertex_input.vertexAttributeDescriptionCount = 2;
  vertex_input.pVertexAttributeDescriptions = attributes;

  VkPipelineInputAssemblyStateCreateInfo input_assembly{};
  input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

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

  VkPipelineRasterizationStateCreateInfo raster{};
  raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  raster.polygonMode = VK_POLYGON_MODE_FILL;
  raster.cullMode = VK_CULL_MODE_NONE;
  raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  raster.lineWidth = 1.0f;

  VkPipelineMultisampleStateCreateInfo multisample{};
  multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineColorBlendAttachmentState blend_attachment{};
  blend_attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
    VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  blend_attachment.blendEnable = VK_FALSE;

  VkPipelineColorBlendStateCreateInfo blend{};
  blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  blend.attachmentCount = 1;
  blend.pAttachments = &blend_attachment;

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

  VkGraphicsPipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipeline_info.stageCount = 2;
  pipeline_info.pStages = stages;
  pipeline_info.pVertexInputState = &vertex_input;
  pipeline_info.pInputAssemblyState = &input_assembly;
  pipeline_info.pViewportState = &viewport_state;
  pipeline_info.pRasterizationState = &raster;
  pipeline_info.pMultisampleState = &multisample;
  pipeline_info.pColorBlendState = &blend;
  pipeline_info.layout = pipeline_layout;
  pipeline_info.renderPass = render_pass;
  pipeline_info.subpass = 0;

  VkPipeline pipeline = VK_NULL_HANDLE;
  const VkResult pipeline_result =
    vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &pipeline);
  if (VK_SUCCESS != pipeline_result)
  {
    vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
    vkDestroyShaderModule(device, vert_module, nullptr);
    vkDestroyShaderModule(device, frag_module, nullptr);
    vkDestroyFramebuffer(device, framebuffer, nullptr);
    vkDestroyRenderPass(device, render_pass, nullptr);
    vkDestroyImageView(device, color_view, nullptr);
    vkDestroyImage(device, color_image, nullptr);
    vkFreeMemory(device, image_memory, nullptr);
    vkDestroyDevice(device, nullptr);
    vkDestroyInstance(instance, nullptr);
    out_result.error = "vkCreateGraphicsPipelines failed";
    return false;
  }

  // ---- vertex buffer (host visible) --------------------------------------
  VkBuffer vertex_buffer = VK_NULL_HANDLE;
  VkDeviceMemory vertex_memory = VK_NULL_HANDLE;
  void* vertex_mapped = nullptr;
  const VkDeviceSize vertex_bytes = std::max<VkDeviceSize>(
    static_cast<VkDeviceSize>(vertices.size()) * sizeof(Vertex), sizeof(Vertex));
  {
    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = vertex_bytes;
    buffer_info.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    vkCreateBuffer(device, &buffer_info, nullptr, &vertex_buffer);
    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, vertex_buffer, &req);
    const uint32_t type = FindMemoryType(physical, req.memoryTypeBits,
                                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                           VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = type;
    vkAllocateMemory(device, &alloc, nullptr, &vertex_memory);
    vkBindBufferMemory(device, vertex_buffer, vertex_memory, 0);
    vkMapMemory(device, vertex_memory, 0, VK_WHOLE_SIZE, 0, &vertex_mapped);
    if (nullptr != vertex_mapped && !vertices.empty())
      std::memcpy(vertex_mapped, vertices.data(), vertices.size() * sizeof(Vertex));
  }

  // ---- readback buffer ----------------------------------------------------
  VkBuffer readback_buffer = VK_NULL_HANDLE;
  VkDeviceMemory readback_memory = VK_NULL_HANDLE;
  void* readback_mapped = nullptr;
  const VkDeviceSize pixel_bytes = static_cast<VkDeviceSize>(width) * height * 4;
  {
    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = pixel_bytes;
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    vkCreateBuffer(device, &buffer_info, nullptr, &readback_buffer);
    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, readback_buffer, &req);
    const uint32_t type = FindMemoryType(physical, req.memoryTypeBits,
                                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                           VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = type;
    vkAllocateMemory(device, &alloc, nullptr, &readback_memory);
    vkBindBufferMemory(device, readback_buffer, readback_memory, 0);
    vkMapMemory(device, readback_memory, 0, VK_WHOLE_SIZE, 0, &readback_mapped);
  }

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
  bool ok = true;
  ok = ok && VK_SUCCESS == vkBeginCommandBuffer(command_buffer, &begin_info);

  VkClearValue clear{};
  clear.color.float32[0] = kBackground[0];
  clear.color.float32[1] = kBackground[1];
  clear.color.float32[2] = kBackground[2];
  clear.color.float32[3] = kBackground[3];

  VkRenderPassBeginInfo rp_begin{};
  rp_begin.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  rp_begin.renderPass = render_pass;
  rp_begin.framebuffer = framebuffer;
  rp_begin.renderArea.offset = { 0, 0 };
  rp_begin.renderArea.extent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height) };
  rp_begin.clearValueCount = 1;
  rp_begin.pClearValues = &clear;

  if (ok)
  {
    vkCmdBeginRenderPass(command_buffer, &rp_begin, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(command_buffer, 0, 1, &vertex_buffer, &offset);
    const float canvas[2] = { static_cast<float>(width), static_cast<float>(height) };
    vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 0,
                       sizeof(canvas), canvas);
    if (!vertices.empty())
      vkCmdDraw(command_buffer, static_cast<uint32_t>(vertices.size()), 1, 0, 0);
    vkCmdEndRenderPass(command_buffer);

    VkBufferImageCopy region{};
    region.bufferOffset = 0;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
    region.imageOffset = { 0, 0, 0 };
    region.imageExtent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
    vkCmdCopyImageToBuffer(command_buffer, color_image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           readback_buffer, 1, &region);
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

  if (ok && nullptr != readback_mapped)
  {
    out_result.rgba.assign(static_cast<size_t>(width) * height * 4, 0);
    std::memcpy(out_result.rgba.data(), readback_mapped, static_cast<size_t>(pixel_bytes));
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
  if (nullptr != readback_mapped)
    vkUnmapMemory(device, readback_memory);
  vkDestroyBuffer(device, readback_buffer, nullptr);
  vkFreeMemory(device, readback_memory, nullptr);
  if (nullptr != vertex_mapped)
    vkUnmapMemory(device, vertex_memory);
  vkDestroyBuffer(device, vertex_buffer, nullptr);
  vkFreeMemory(device, vertex_memory, nullptr);
  vkDestroyPipeline(device, pipeline, nullptr);
  vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
  vkDestroyShaderModule(device, vert_module, nullptr);
  vkDestroyShaderModule(device, frag_module, nullptr);
  vkDestroyFramebuffer(device, framebuffer, nullptr);
  vkDestroyRenderPass(device, render_pass, nullptr);
  vkDestroyImageView(device, color_view, nullptr);
  vkDestroyImage(device, color_image, nullptr);
  vkFreeMemory(device, image_memory, nullptr);
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
  ihdr.push_back(0);  // compression
  ihdr.push_back(0);  // filter
  ihdr.push_back(0);  // interlace
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
