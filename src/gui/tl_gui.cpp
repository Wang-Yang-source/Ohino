/**
 * @file tl_gui.cpp
 * @brief Ohino Studio workbench layout + screen-space draw list builder.
 */

#include "opentl/gui/tl_gui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

// Catppuccin Mocha
const float kCrust[4]     = { 0.067f, 0.067f, 0.106f, 1.0f };
const float kMantle[4]    = { 0.094f, 0.094f, 0.145f, 1.0f };
const float kBase[4]      = { 0.118f, 0.118f, 0.180f, 1.0f };
const float kSurface0[4]  = { 0.192f, 0.196f, 0.267f, 1.0f };
const float kSurface1[4]  = { 0.271f, 0.278f, 0.353f, 1.0f };
const float kSurface2[4]  = { 0.345f, 0.357f, 0.439f, 1.0f };
const float kOverlay0[4]  = { 0.424f, 0.439f, 0.525f, 1.0f };
const float kSubtext0[4]  = { 0.651f, 0.678f, 0.784f, 1.0f };
const float kText[4]      = { 0.804f, 0.839f, 0.957f, 1.0f };
const float kLavender[4]  = { 0.706f, 0.745f, 0.996f, 1.0f };
const float kMauve[4]     = { 0.796f, 0.651f, 0.969f, 1.0f };
const float kTeal[4]      = { 0.580f, 0.886f, 0.835f, 1.0f };
const float kPeach[4]     = { 0.980f, 0.702f, 0.529f, 1.0f };
const float kYellow[4]    = { 0.976f, 0.886f, 0.686f, 1.0f };
const float kSapphire[4]  = { 0.455f, 0.780f, 0.925f, 1.0f };
const float kGreen[4]     = { 0.651f, 0.890f, 0.631f, 1.0f };
const float kRed[4]       = { 0.953f, 0.545f, 0.659f, 1.0f };

std::string U(const std::string& s) {
  std::string out = s;
  for (char& c : out) c = (char)std::toupper((unsigned char)c);
  return out;
}

void SetFill(TL_GuiFill& f, float x, float y, float w, float h,
             const float* color, const float* border = nullptr, float border_px = 1.0f) {
  f.x = x; f.y = y; f.w = w; f.h = h;
  for (int i = 0; i < 4; ++i) { f.rgba[i] = color[i]; f.border_rgba[i] = border ? border[i] : color[i]; }
  f.border_px = border ? border_px : 0.0f;
}

void AddFill(TL_GuiOverlay& ov, float x, float y, float w, float h,
             const float* color, const float* border = nullptr, float border_px = 1.0f) {
  if (w <= 0.0f || h <= 0.0f) return;
  TL_GuiFill f;
  SetFill(f, x, y, w, h, color, border, border_px);
  ov.fills.push_back(f);
}

void AddLine(TL_GuiOverlay& ov, float x0, float y0, float x1, float y1,
             const float* color, float px = 1.0f) {
  TL_GuiLine l;
  l.x0 = x0; l.y0 = y0; l.x1 = x1; l.y1 = y1; l.px = px;
  for (int i = 0; i < 4; ++i) l.rgba[i] = color[i];
  ov.lines.push_back(l);
}

void AddText(TL_GuiOverlay& ov, float x, float y, float pixel, const std::string& text,
             const float* color) {
  if (text.empty()) return;
  TL_GuiText t;
  t.x = x; t.y = y; t.pixel = pixel; t.text = U(text);
  for (int i = 0; i < 4; ++i) t.rgba[i] = color[i];
  ov.texts.push_back(t);
}

float TextWidth(const std::string& s, float pixel) { return (float)s.size() * 6.0f * pixel; }

std::vector<std::string> SplitLines(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == '\n') { out.push_back(cur); cur.clear(); }
    else if (c != '\r') cur += c;
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

std::string Truncate(const std::string& s, float max_px, float pixel) {
  const size_t max_chars = (size_t)std::max(1.0f, max_px / (6.0f * pixel));
  if (s.size() <= max_chars) return s;
  if (max_chars <= 2) return s.substr(0, max_chars);
  return s.substr(0, max_chars - 2) + "..";
}

const float* AccentForNode(const TL_GhNode& n) {
  if (n.is_slider) return kPeach;
  if (n.is_panel) return kYellow;
  if (n.is_group) return kSurface1;
  if (n.is_scribble) return kOverlay0;
  const std::string& nm = n.name;
  if (nm == "Point" || nm == "Construct Point" || nm == "Deconstruct") return kSapphire;
  if (nm == "Line" || nm == "Line SDL" || nm == "Nurbs Curve" || nm == "Interpolate data" ||
      nm == "Divide Curve" || nm == "Curve Closest Point" || nm == "Offset Curve" ||
      nm == "Project" || nm == "Polyline" || nm == "Arc SED" || nm == "Circle")
    return kTeal;
  if (nm == "Surface" || nm == "Loft" || nm == "Sweep1" || nm == "SubD Box" || nm == "Brep | Line")
    return kMauve;
  if (nm == "Addition" || nm == "Subtraction" || nm == "Multiplication" || nm == "Division" ||
      nm == "Range" || nm == "Average" || nm == "Remap Numbers" || nm == "Bounds")
    return kGreen;
  return kLavender;
}

void DrawPorts(TL_GuiOverlay& ov, const TL_GhNode& n, float x, float y, float w, float h, float s) {
  const float r = std::max(2.0f, 3.2f * s);
  const int cols = std::max<int>(1, (int)n.inputs.size());
  for (int i = 0; i < (int)n.inputs.size(); ++i) {
    const float py = y + h * ((float)(i + 1) / (float)(cols + 1));
    AddFill(ov, x - r * 0.5f, py - r * 0.5f, r, r, kSurface2, kOverlay0, 1.0f);
  }
  const int ocols = std::max<int>(1, (int)n.outputs.size());
  for (int i = 0; i < (int)n.outputs.size(); ++i) {
    const float py = y + h * ((float)(i + 1) / (float)(ocols + 1));
    AddFill(ov, x + w - r * 0.5f, py - r * 0.5f, r, r, kSurface2, kOverlay0, 1.0f);
  }
}

}  // namespace

TL_GuiStudioRects TL_ComputeStudioRects(int width, int height) {
  TL_GuiStudioRects r;
  r.width = width;
  r.height = height;
  const float w = (float)width;
  const float h = (float)height;
  const float menu_h = 30.0f;
  const float cmd_h = 30.0f;
  const float tabs_h = 38.0f;
  const float status_h = 28.0f;
  const float dock_w = std::min(268.0f, w * 0.16f);
  const float gh_w = std::min(760.0f, w * 0.40f);
  const float params_h = std::min(196.0f, h * 0.20f);
  const float top = menu_h + cmd_h + tabs_h;

  SetFill(r.menu, 0.0f, 0.0f, w, menu_h, kBase, kSurface0);
  SetFill(r.command, 0.0f, menu_h, w, cmd_h, kMantle, kSurface0);
  SetFill(r.tabs, 0.0f, menu_h + cmd_h, w, tabs_h, kMantle, kSurface0);
  SetFill(r.status, 0.0f, h - status_h, w, status_h, kBase, kSurface0);
  SetFill(r.dock, 0.0f, top, dock_w, h - top - status_h, kMantle, kSurface0);
  SetFill(r.gh_canvas, w - gh_w, top, gh_w, h - top - status_h - params_h, kCrust, kSurface0);
  SetFill(r.gh_params, w - gh_w, h - status_h - params_h, gh_w, params_h, kBase, kSurface0);
  SetFill(r.viewports, dock_w, top, w - dock_w - gh_w, h - top - status_h, kCrust, kSurface0);
  return r;
}

void TL_ApplyStudioViewports(TL_FourViewLayout& layout, const TL_GuiStudioRects& rects) {
  layout.SetCanvasSize(rects.width, rects.height);
  const int vx = (int)rects.viewports.x;
  const int vy = (int)rects.viewports.y;
  const int vw = (int)rects.viewports.w;
  const int vh = (int)rects.viewports.h;
  const int half_w = vw / 2;
  const int half_h = vh / 2;
  layout.View(0).SetCanvasRect(vx, vy, half_w, half_h);
  layout.View(1).SetCanvasRect(vx + half_w, vy, vw - half_w, half_h);
  layout.View(2).SetCanvasRect(vx, vy + half_h, half_w, vh - half_h);
  layout.View(3).SetCanvasRect(vx + half_w, vy + half_h, vw - half_w, vh - half_h);
}

void TL_BuildStudioOverlay(const TL_GuiStudioRects& rects, const TL_GhDocument* gh,
                           const TL_GuiStudioInfo& info, TL_GuiOverlay& ov) {
  ov.fills.clear();
  ov.lines.clear();
  ov.texts.clear();
  ov.enabled = true;

  const float w = (float)rects.width;
  const float h = (float)rects.height;

  ov.fills.push_back(rects.menu);
  ov.fills.push_back(rects.command);
  ov.fills.push_back(rects.tabs);
  ov.fills.push_back(rects.dock);
  ov.fills.push_back(rects.status);
  ov.fills.push_back(rects.gh_canvas);
  ov.fills.push_back(rects.gh_params);

  // ---- menu bar -----------------------------------------------------------
  AddFill(ov, 12.0f, 7.0f, 16.0f, 16.0f, kMauve, kLavender, 1.0f);
  AddText(ov, 36.0f, 11.0f, 1.6f, info.title, kText);
  const char* menus[] = {"FILE", "EDIT", "VIEW", "CURVE", "SURFACE", "SOLID", "SUBD", "TRANSFORM", "TOOLS", "HELP"};
  float mx = w * 0.32f;
  for (const char* m : menus) {
    AddText(ov, mx, 11.0f, 1.3f, m, kSubtext0);
    mx += TextWidth(m, 1.3f) + 18.0f;
  }
  AddFill(ov, w - 74.0f, 6.0f, 34.0f, 18.0f, kSurface0, kPeach, 1.0f);
  AddText(ov, w - 68.0f, 11.0f, 1.3f, "WIP", kPeach);
  AddText(ov, w - 178.0f, 11.0f, 1.3f, info.device, kOverlay0);

  // ---- command bar --------------------------------------------------------
  AddText(ov, 14.0f, rects.command.y + 10.0f, 1.4f, info.command, kLavender);
  AddFill(ov, w * 0.60f, rects.command.y + 8.0f, 14.0f, 14.0f, kGreen, kGreen, 0.0f);
  AddText(ov, w * 0.60f + 22.0f, rects.command.y + 10.0f, 1.3f, "SOLVED", kGreen);

  // ---- tool palette tabs --------------------------------------------------
  const char* tabs[] = {"STANDARD", "CPLANES", "SET VIEW", "SELECT", "CURVE", "SURFACE", "SOLID", "SUBD", "GRASSHOPPER"};
  float tx = 12.0f;
  for (int i = 0; i < 9; ++i) {
    const bool active = (i == 8);
    const float tw = TextWidth(tabs[i], 1.3f) + 22.0f;
    AddFill(ov, tx, rects.tabs.y + 5.0f, tw, rects.tabs.h - 10.0f,
            active ? kSurface0 : kMantle, active ? kLavender : kSurface0, 1.0f);
    AddText(ov, tx + 11.0f, rects.tabs.y + 14.0f, 1.3f, tabs[i], active ? kLavender : kSubtext0);
    tx += tw + 6.0f;
  }

  // ---- left dock: layers + properties ------------------------------------
  const float dx = rects.dock.x + 12.0f;
  float dy = rects.dock.y + 12.0f;
  AddText(ov, dx, dy, 1.5f, "LAYERS", kText);
  char buf[128];
  std::snprintf(buf, sizeof(buf), "%zu", info.layers.size());
  AddText(ov, rects.dock.x + rects.dock.w - 34.0f, dy, 1.3f, buf, kOverlay0);
  dy += 22.0f;
  const float* dot_colors[] = {kLavender, kRed, kMauve, kYellow, kTeal, kSapphire, kPeach};
  for (size_t i = 0; i < info.layers.size(); ++i) {
    const bool active = (info.layers[i] == info.active_layer);
    if (active) AddFill(ov, rects.dock.x + 6.0f, dy - 4.0f, rects.dock.w - 12.0f, 18.0f, kSurface0, kSurface1, 1.0f);
    AddFill(ov, dx, dy, 9.0f, 9.0f, dot_colors[i % 7], kSurface2, 1.0f);
    AddText(ov, dx + 18.0f, dy, 1.3f, Truncate(info.layers[i], rects.dock.w - 50.0f, 1.3f),
            active ? kText : kSubtext0);
    dy += 22.0f;
  }

  dy += 16.0f;
  AddLine(ov, dx, dy, rects.dock.x + rects.dock.w - 12.0f, dy, kSurface0, 1.0f);
  dy += 12.0f;
  AddText(ov, dx, dy, 1.5f, "PROPERTIES", kText);
  dy += 24.0f;
  char prop[96];
  std::snprintf(prop, sizeof(prop), "OBJECTS %zu", info.object_count);
  AddText(ov, dx, dy, 1.3f, prop, kSubtext0); dy += 20.0f;
  std::snprintf(prop, sizeof(prop), "MESHES %zu", info.mesh_count);
  AddText(ov, dx, dy, 1.3f, prop, kSubtext0); dy += 20.0f;
  std::snprintf(prop, sizeof(prop), "GH NODES %zu", gh ? gh->Nodes().size() : (size_t)0);
  AddText(ov, dx, dy, 1.3f, prop, kSubtext0); dy += 20.0f;
  std::snprintf(prop, sizeof(prop), "SLIDERS %zu", gh ? gh->SliderCount() : (size_t)0);
  AddText(ov, dx, dy, 1.3f, prop, kSubtext0); dy += 20.0f;
  std::snprintf(prop, sizeof(prop), "WIRES %zu", gh ? gh->Wires().size() : (size_t)0);
  AddText(ov, dx, dy, 1.3f, prop, kSubtext0); dy += 26.0f;

  AddLine(ov, dx, dy, rects.dock.x + rects.dock.w - 12.0f, dy, kSurface0, 1.0f);
  dy += 12.0f;
  AddText(ov, dx, dy, 1.3f, "OSNAP", kOverlay0); dy += 18.0f;
  AddText(ov, dx, dy, 1.2f, "END MID CEN INT PERP", kSubtext0); dy += 16.0f;
  AddText(ov, dx, dy, 1.3f, "GUMBALL", kOverlay0); dy += 18.0f;
  AddFill(ov, dx, dy, 52.0f, 16.0f, kSurface0, kTeal, 1.0f);
  AddText(ov, dx + 8.0f, dy + 4.0f, 1.2f, "ACTIVE", kTeal);

  // ---- Grasshopper canvas ------------------------------------------------
  const TL_GuiFill& canvas = rects.gh_canvas;
  AddFill(ov, canvas.x, canvas.y, canvas.w, 26.0f, kSurface0, kSurface1, 1.0f);
  AddFill(ov, canvas.x + 8.0f, canvas.y + 8.0f, 10.0f, 10.0f, kGreen, kGreen, 0.0f);
  AddText(ov, canvas.x + 26.0f, canvas.y + 8.0f, 1.4f, "GRASSHOPPER", kText);
  if (gh) {
    std::snprintf(buf, sizeof(buf), "%zu NODES  %zu WIRES  %zu SLIDERS",
                  gh->Nodes().size(), gh->Wires().size(), gh->SliderCount());
    AddText(ov, canvas.x + canvas.w - TextWidth(buf, 1.2f) - 12.0f, canvas.y + 9.0f, 1.2f, buf, kOverlay0);
  }

  const float cx0 = canvas.x + 14.0f;
  const float cy0 = canvas.y + 36.0f;
  const float cw = canvas.w - 28.0f;
  const float ch = canvas.h - 48.0f;
  AddFill(ov, cx0, cy0, cw, ch, kMantle, kSurface0, 1.0f);

  if (gh && !gh->Nodes().empty()) {
    TL_GhRectF cb = gh->CanvasBounds();
    {
      float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
      bool any = false;
      for (const auto& n : gh->Nodes()) {
        if (n.is_group || n.is_scribble) continue;
        x0 = std::min(x0, n.bounds.x); y0 = std::min(y0, n.bounds.y);
        x1 = std::max(x1, n.bounds.x + n.bounds.w); y1 = std::max(y1, n.bounds.y + n.bounds.h);
        any = true;
      }
      if (any && x1 > x0) { cb.x = x0; cb.y = y0; cb.w = x1 - x0; cb.h = y1 - y0; }
    }
    float s = std::min(cw / cb.w, ch / cb.h) * 0.92f;
    s = std::max(0.22f, std::min(0.62f, s));
    const float ox = cx0 + (cw - cb.w * s) * 0.5f - cb.x * s;
    const float oy = cy0 + (ch - cb.h * s) * 0.5f - cb.y * s;
    auto mx = [&](float x) { return ox + x * s; };
    auto my = [&](float y) { return oy + y * s; };

    // background dot grid
    for (float gx = cx0 + 8.0f; gx < cx0 + cw; gx += 24.0f)
      for (float gy = cy0 + 8.0f; gy < cy0 + ch; gy += 24.0f)
        AddFill(ov, gx, gy, 1.4f, 1.4f, kSurface0, kSurface0, 0.0f);

    // groups first (behind wires and nodes)
    for (const auto& n : gh->Nodes()) {
      if (!n.is_group) continue;
      const float gx = mx(n.bounds.x) - 6.0f * s;
      const float gy = my(n.bounds.y) - 14.0f * s;
      const float gw = n.bounds.w * s + 12.0f * s;
      const float gh_h = n.bounds.h * s + 20.0f * s;
      AddFill(ov, gx, gy, gw, gh_h, kSurface0, kLavender, 1.0f);
      AddText(ov, gx + 5.0f, gy + 4.0f, 1.1f, Truncate(n.nickname, gw - 10.0f, 1.1f), kSubtext0);
    }

    // wires (cubic bezier with horizontal tangents)
    for (const auto& wire : gh->Wires()) {
      const float x0 = mx(wire.x0);
      const float y0 = my(wire.y0);
      const float x1 = mx(wire.x1);
      const float y1 = my(wire.y1);
      const float dx = std::max(24.0f * s, std::fabs(x1 - x0) * 0.45f);
      float px = x0, py = y0;
      for (int k = 1; k <= 10; ++k) {
        const float t = (float)k / 10.0f;
        const float it = 1.0f - t;
        const float bx = it * it * it * x0 + 3 * it * it * t * (x0 + dx) + 3 * it * t * t * (x1 - dx) + t * t * t * x1;
        const float by = it * it * it * y0 + 3 * it * it * t * y0 + 3 * it * t * t * y1 + t * t * t * y1;
        AddLine(ov, px, py, bx, by, kSurface2, 1.4f);
        px = bx;
        py = by;
      }
    }

    // nodes
    for (const auto& n : gh->Nodes()) {
      if (n.is_group) continue;
      const float nx = mx(n.bounds.x);
      const float ny = my(n.bounds.y);
      const float nw = std::max(10.0f, n.bounds.w * s);
      const float nh = std::max(8.0f, n.bounds.h * s);
      if (nx + nw < cx0 || nx > cx0 + cw || ny + nh < cy0 || ny > cy0 + ch) continue;

      const float* accent = AccentForNode(n);

      if (n.is_panel) {
        AddFill(ov, nx, ny, nw, nh, kBase, kYellow, 1.0f);
        const auto lines = SplitLines(n.panel_text);
        float ty = ny + 4.0f;
        for (size_t i = 0; i < lines.size() && i < 6; ++i) {
          AddText(ov, nx + 4.0f, ty, 1.0f, Truncate(lines[i], nw - 8.0f, 1.0f), kSubtext0);
          ty += 9.0f;
        }
        continue;
      }
      if (n.is_scribble) {
        const auto sl = SplitLines(n.panel_text);
        const std::string label = sl.empty() ? n.nickname : sl.front();
        AddText(ov, nx, ny, 1.1f, Truncate(label, std::max(60.0f, nw), 1.1f), kOverlay0);
        continue;
      }

      AddFill(ov, nx, ny, nw, nh, kBase, kSurface2, 1.0f);
      AddFill(ov, nx + 1.0f, ny + 1.0f, nw - 2.0f, std::max(2.0f, 4.0f * s), accent, accent, 0.0f);
      if (nw >= 30.0f) {
        const float tp = std::max(0.85f, std::min(1.25f, nw / 46.0f));
        AddText(ov, nx + 4.0f, ny + std::max(6.0f, 7.0f * s), tp,
                Truncate(n.nickname.empty() ? n.name : n.nickname, nw - 8.0f, tp), kText);
      }

      if (n.is_slider) {
        const float tx0 = nx + 6.0f;
        const float tx1 = nx + nw - 6.0f;
        const float ty = ny + nh - std::max(7.0f, 9.0f * s);
        AddLine(ov, tx0, ty, tx1, ty, kSurface2, 2.0f);
        const double span = std::max(1e-9, n.slider.max_val - n.slider.min_val);
        const float frac = (float)std::max(0.0, std::min(1.0, (n.slider.value - n.slider.min_val) / span));
        AddLine(ov, tx0, ty, tx0 + (tx1 - tx0) * frac, ty, kPeach, 2.0f);
        AddFill(ov, tx0 + (tx1 - tx0) * frac - 2.0f, ty - 3.0f, 4.0f, 7.0f, kText, kPeach, 1.0f);
        char val[32];
        std::snprintf(val, sizeof(val), "%.4g", n.slider.value);
        const float vp = 0.95f;
        if (nw >= 34.0f)
          AddText(ov, nx + nw - TextWidth(val, vp) - 4.0f, ny + 3.0f, vp, val, kYellow);
      } else {
        DrawPorts(ov, n, nx, ny, nw, nh, s);
        if (nw > 54.0f && nh > 26.0f) {
          char cbuf[32];
          std::snprintf(cbuf, sizeof(cbuf), "%zux%zu", n.inputs.size(), n.outputs.size());
          AddText(ov, nx + nw - TextWidth(cbuf, 0.9f) - 4.0f, ny + nh - 10.0f, 0.9f, cbuf, kOverlay0);
        }
      }
    }
    AddText(ov, cx0 + 8.0f, cy0 + ch - 14.0f, 1.0f, "SCROLL TO ZOOM   MMB PAN   LMB DRAG SLIDER", kOverlay0);
  } else {
    AddText(ov, cx0 + 16.0f, cy0 + 16.0f, 1.3f, "NO GRASSHOPPER DEFINITION LOADED", kOverlay0);
  }

  // ---- Grasshopper slider inspector --------------------------------------
  const TL_GuiFill& params = rects.gh_params;
  AddFill(ov, params.x, params.y, params.w, 24.0f, kSurface0, kSurface1, 1.0f);
  AddText(ov, params.x + 10.0f, params.y + 7.0f, 1.3f, "SOLVED PARAMETERS", kText);
  float py = params.y + 34.0f;
  int shown = 0;
  if (gh) {
    for (const auto& n : gh->Nodes()) {
      if (!n.is_slider || shown >= 5) continue;
      const float bar_x = params.x + 150.0f;
      const float bar_w = params.w - 250.0f;
      AddText(ov, params.x + 12.0f, py, 1.15f,
              Truncate(n.nickname.empty() ? n.name : n.nickname, 132.0f, 1.15f), kSubtext0);
      AddLine(ov, bar_x, py + 4.0f, bar_x + bar_w, py + 4.0f, kSurface2, 2.0f);
      const double span = std::max(1e-9, n.slider.max_val - n.slider.min_val);
      const float frac = (float)std::max(0.0, std::min(1.0, (n.slider.value - n.slider.min_val) / span));
      AddLine(ov, bar_x, py + 4.0f, bar_x + bar_w * frac, py + 4.0f, kPeach, 2.0f);
      AddFill(ov, bar_x + bar_w * frac - 2.5f, py, 5.0f, 9.0f, kText, kPeach, 1.0f);
      char val[48];
      std::snprintf(val, sizeof(val), "%.4g", n.slider.value);
      AddText(ov, params.x + params.w - 78.0f, py, 1.15f, val, kYellow);
      py += 26.0f;
      ++shown;
    }
    if (shown == 0) {
      AddText(ov, params.x + 12.0f, py, 1.2f, "THIS DEFINITION HAS NO SLIDERS", kOverlay0);
    }
  }

  // ---- status bar --------------------------------------------------------
  AddFill(ov, 0.0f, rects.status.y + 1.0f, w, 1.0f, kSurface1, kSurface1, 0.0f);
  AddText(ov, 14.0f, rects.status.y + 9.0f, 1.25f, info.status, kSubtext0);
  AddFill(ov, w - 96.0f, rects.status.y + 7.0f, 8.0f, 8.0f, kGreen, kGreen, 0.0f);
  AddText(ov, w - 82.0f, rects.status.y + 9.0f, 1.2f, "READY", kGreen);
}
