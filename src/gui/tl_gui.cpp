/**
 * @file tl_gui.cpp
 * @brief Ohino Studio workbench layout, screen-space draw list builder, and SDL3 interactive window.
 */

#include "opentl/gui/tl_gui.h"

#if defined(OPENTL_HAS_SDL3)
#include <SDL3/SDL.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unordered_map>

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
const float kBlue[4]      = { 0.537f, 0.706f, 0.980f, 1.0f };

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
  if (nm == "Point" || nm == "Construct Point" || nm == "Deconstruct" || nm == "Poisson Distribution")
    return kSapphire;
  if (nm == "Line" || nm == "Line SDL" || nm == "Nurbs Curve" || nm == "Interpolate data" ||
      nm == "Divide Curve" || nm == "Curve Closest Point" || nm == "Offset Curve" ||
      nm == "Project" || nm == "Polyline" || nm == "Arc SED" || nm == "Circle" || nm == "Circle Radius")
    return kTeal;
  if (nm == "Surface" || nm == "Loft" || nm == "Sweep1" || nm == "Sweep2" ||
      nm == "SubD Box" || nm == "Brep | Line")
    return kMauve;
  if (nm == "Addition" || nm == "Subtraction" || nm == "Multiplication" || nm == "Division" ||
      nm == "Range" || nm == "Average" || nm == "Remap Numbers" || nm == "Bounds")
    return kGreen;
  return kLavender;
}

void DrawPorts(TL_GuiOverlay& ov, const TL_GhNode& n, float x, float y, float w, float h, float s) {
  const float r = std::max(2.5f, 3.6f * s);
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

void DrawViewportGumballAndHud(TL_GuiOverlay& ov, const TL_GuiStudioInfo& info) {
  if (!info.layout) return;
  const char* modes[4] = {"WIREFRAME", "SHADED", "SHADED", "SHADED"};
  for (int vi = 0; vi < info.layout->Count(); ++vi) {
    const TL_Viewport& vp = info.layout->View(vi);
    const float vx = (float)vp.m_left;
    const float vy = (float)vp.m_top;
    const float vw = (float)vp.m_width;
    const float vh = (float)vp.m_height;

    // Rhino 9 viewport display mode pill next to title
    const float name_w = TextWidth(vp.m_name, 2.0f);
    const float pill_x = vx + 16.0f + name_w;
    const float pill_y = vy + 7.0f;
    const float pill_w = TextWidth(modes[vi & 3], 1.1f) + 14.0f;
    AddFill(ov, pill_x, pill_y, pill_w, 16.0f, kSurface0, kSurface2, 1.0f);
    AddText(ov, pill_x + 7.0f, pill_y + 4.0f, 1.1f, modes[vi & 3], kSubtext0);

    if (!info.show_gumball) continue;
    ON_2dPoint c2d;
    if (!vp.WorldToScreen(info.gumball_origin, c2d)) continue;
    if (c2d.x < vx + 24.0 || c2d.x > vx + vw - 24.0 ||
        c2d.y < vy + 24.0 || c2d.y > vy + vh - 24.0) continue;

    const double R = std::max(1.0, info.gumball_radius);
    const ON_3dPoint axes[3] = {
      info.gumball_origin + ON_3dVector(R, 0.0, 0.0),
      info.gumball_origin + ON_3dVector(0.0, R, 0.0),
      info.gumball_origin + ON_3dVector(0.0, 0.0, R)
    };
    const float* acols[3] = { kRed, kGreen, kBlue };

    for (int a = 0; a < 3; ++a) {
      ON_2dPoint p2d;
      if (!vp.WorldToScreen(axes[a], p2d)) continue;
      double dx = p2d.x - c2d.x;
      double dy = p2d.y - c2d.y;
      const double len = std::hypot(dx, dy);
      if (len < 6.0) continue;
      // Normalize screen length of Gumball arm to 42 px so it stays crisp at any zoom
      const double ux = dx / len;
      const double uy = dy / len;
      const float ex = (float)(c2d.x + ux * 42.0);
      const float ey = (float)(c2d.y + uy * 42.0);
      AddLine(ov, (float)c2d.x, (float)c2d.y, ex, ey, acols[a], 2.2f);
      // Arrowhead wings
      const float nx = (float)(-uy);
      const float ny = (float)(ux);
      AddLine(ov, ex, ey, ex - (float)ux * 7.0f + nx * 4.0f, ey - (float)uy * 7.0f + ny * 4.0f, acols[a], 2.0f);
      AddLine(ov, ex, ey, ex - (float)ux * 7.0f - nx * 4.0f, ey - (float)uy * 7.0f - ny * 4.0f, acols[a], 2.0f);
      // Scale handle square at 70% arm length
      const float sx = (float)(c2d.x + ux * 28.0);
      const float sy = (float)(c2d.y + uy * 28.0);
      AddFill(ov, sx - 2.5f, sy - 2.5f, 5.0f, 5.0f, acols[a], kBase, 1.0f);
    }
    // Center pivot + OSnap CEN badge
    AddFill(ov, (float)c2d.x - 3.5f, (float)c2d.y - 3.5f, 7.0f, 7.0f, kText, kLavender, 1.0f);
    AddFill(ov, (float)c2d.x + 8.0f, (float)c2d.y - 16.0f, 26.0f, 12.0f, kSurface0, kYellow, 1.0f);
    AddText(ov, (float)c2d.x + 11.0f, (float)c2d.y - 13.0f, 0.95f, "CEN", kYellow);
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

  ov.fills.push_back(rects.menu);
  ov.fills.push_back(rects.command);
  ov.fills.push_back(rects.tabs);
  ov.fills.push_back(rects.dock);
  ov.fills.push_back(rects.status);
  ov.fills.push_back(rects.gh_canvas);
  ov.fills.push_back(rects.gh_params);

  // ---- 3D Viewport HUD pills + Gumball + OSnap markers --------------------
  DrawViewportGumballAndHud(ov, info);

  // ---- menu bar -----------------------------------------------------------
  AddFill(ov, 12.0f, 7.0f, 16.0f, 16.0f, kMauve, kLavender, 1.0f);
  AddText(ov, 36.0f, 11.0f, 1.6f, info.title, kText);
  const char* menus[] = {"FILE", "EDIT", "VIEW", "CURVE", "SURFACE", "SOLID", "SUBD", "TRANSFORM", "TOOLS", "HELP"};
  float mx_menu = w * 0.32f;
  for (const char* m : menus) {
    AddText(ov, mx_menu, 11.0f, 1.3f, m, kSubtext0);
    mx_menu += TextWidth(m, 1.3f) + 18.0f;
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
    std::unordered_map<std::string, const TL_GhNode*> guid_map;
    for (const auto& n : gh->Nodes()) {
      if (!n.instance_guid.empty()) guid_map[n.instance_guid] = &n;
    }

    TL_GhRectF cb = gh->CanvasBounds();
    {
      float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
      bool any = false;
      for (const auto& n : gh->Nodes()) {
        if (n.is_group || n.is_scribble) continue;
        const float w_eff = n.is_slider ? std::min(n.bounds.w, 170.0f) : n.bounds.w;
        x0 = std::min(x0, n.bounds.x); y0 = std::min(y0, n.bounds.y);
        x1 = std::max(x1, n.bounds.x + w_eff); y1 = std::max(y1, n.bounds.y + n.bounds.h);
        any = true;
      }
      if (any && x1 > x0) {
        cb.x = x0; cb.y = y0;
        cb.w = std::max(100.0f, x1 - x0);
        cb.h = std::max(80.0f, y1 - y0);
      }
    }

    // Separate horizontal and vertical scale so wide definitions fill vertical canvas height
    const float sx = std::max(0.32f, std::min(0.75f, (cw - 36.0f) / cb.w));
    const float sy = std::max(sx, std::min(sx * 1.65f, (ch - 64.0f) / cb.h));
    const float ox = cx0 + (cw - cb.w * sx) * 0.5f - cb.x * sx;
    const float oy = cy0 + (ch - cb.h * sy) * 0.52f - cb.y * sy;
    auto mx = [&](float x) { return ox + x * sx; };
    auto my = [&](float y) { return oy + y * sy; };

    // background dot grid
    for (float gx = cx0 + 8.0f; gx < cx0 + cw; gx += 24.0f)
      for (float gy = cy0 + 8.0f; gy < cy0 + ch; gy += 24.0f)
        AddFill(ov, gx, gy, 1.4f, 1.4f, kSurface0, kSurface0, 0.0f);

    // groups first (computed dynamically from member GUIDs when bounds are 0)
    for (const auto& n : gh->Nodes()) {
      if (!n.is_group) continue;
      float gx0 = 1e9f, gy0 = 1e9f, gx1 = -1e9f, gy1 = -1e9f;
      for (const std::string& mg : n.group_member_guids) {
        auto it = guid_map.find(mg);
        if (it == guid_map.end() || it->second->is_group || it->second->is_scribble) continue;
        const TL_GhNode& m = *it->second;
        const float mw = m.is_slider ? std::min(m.bounds.w, 165.0f) : m.bounds.w;
        gx0 = std::min(gx0, m.bounds.x);
        gy0 = std::min(gy0, m.bounds.y);
        gx1 = std::max(gx1, m.bounds.x + mw);
        gy1 = std::max(gy1, m.bounds.y + m.bounds.h);
      }
      if (gx1 <= gx0 && n.bounds.w > 1.0f) {
        gx0 = n.bounds.x; gy0 = n.bounds.y;
        gx1 = n.bounds.x + n.bounds.w; gy1 = n.bounds.y + n.bounds.h;
      }
      if (gx1 <= gx0) continue;
      const float rx = std::max(cx0 + 4.0f, mx(gx0) - 8.0f);
      const float ry = std::max(cy0 + 4.0f, my(gy0) - 18.0f);
      const float rw = std::min(cx0 + cw - 4.0f - rx, (gx1 - gx0) * sx + 16.0f);
      const float rh = std::min(cy0 + ch - 20.0f - ry, (gy1 - gy0) * sy + 26.0f);
      if (rw <= 8.0f || rh <= 8.0f) continue;
      AddFill(ov, rx, ry, rw, rh, kSurface0, kSurface2, 1.0f);
      if (!n.nickname.empty()) {
        AddText(ov, rx + 5.0f, ry + 4.0f, 1.0f, Truncate(n.nickname, rw - 10.0f, 1.0f), kSubtext0);
      }
    }

    // wires (cubic bezier with horizontal tangents)
    for (const auto& wire : gh->Wires()) {
      const float x0 = std::clamp(mx(wire.x0), cx0 + 4.0f, cx0 + cw - 4.0f);
      const float y0 = std::clamp(my(wire.y0), cy0 + 4.0f, cy0 + ch - 18.0f);
      const float x1 = std::clamp(mx(wire.x1), cx0 + 4.0f, cx0 + cw - 4.0f);
      const float y1 = std::clamp(my(wire.y1), cy0 + 4.0f, cy0 + ch - 18.0f);
      const float dx_w = std::max(18.0f, std::fabs(x1 - x0) * 0.45f);
      float px = x0, py = y0;
      for (int k = 1; k <= 10; ++k) {
        const float t = (float)k / 10.0f;
        const float it = 1.0f - t;
        const float bx = it * it * it * x0 + 3 * it * it * t * (x0 + dx_w) + 3 * it * t * t * (x1 - dx_w) + t * t * t * x1;
        const float by = it * it * it * y0 + 3 * it * it * t * y0 + 3 * it * t * t * y1 + t * t * t * y1;
        AddLine(ov, px, py, bx, by, kLavender, 1.4f);
        px = bx;
        py = by;
      }
    }

    // nodes
    for (const auto& n : gh->Nodes()) {
      if (n.is_group || n.is_scribble) continue;
      float nx = mx(n.bounds.x);
      float ny = my(n.bounds.y);
      float nw = n.is_slider ? std::min(112.0f, std::max(78.0f, n.bounds.w * sx * 0.65f))
                             : std::max(38.0f, n.bounds.w * sx);
      float nh = n.is_slider ? 20.0f : std::max(22.0f, n.bounds.h * sy);
      if (nx + nw > cx0 + cw - 6.0f) nw = std::max(28.0f, cx0 + cw - 6.0f - nx);
      if (nx < cx0 + 4.0f || nx >= cx0 + cw - 12.0f || ny < cy0 + 4.0f || ny + nh > cy0 + ch - 18.0f)
        continue;

      const float* accent = AccentForNode(n);

      if (n.is_panel) {
        AddFill(ov, nx, ny, nw, nh, kBase, kYellow, 1.0f);
        const auto lines = SplitLines(n.panel_text);
        float ty = ny + 4.0f;
        for (size_t i = 0; i < lines.size() && i < 5; ++i) {
          AddText(ov, nx + 4.0f, ty, 0.95f, Truncate(lines[i], nw - 8.0f, 0.95f), kSubtext0);
          ty += 9.0f;
        }
        continue;
      }

      if (n.is_slider) {
        // Dedicated pill container for canvas sliders so text never collides with wires/nodes
        AddFill(ov, nx, ny, nw, nh, kBase, kPeach, 1.0f);
        char val[32];
        std::snprintf(val, sizeof(val), "%.4g", n.slider.value);
        const float vp = 0.95f;
        const float val_w = TextWidth(val, vp);
        const std::string lbl = n.nickname.empty() ? "SLIDER" : n.nickname;
        AddText(ov, nx + 4.0f, ny + 3.0f, vp, Truncate(lbl, nw - val_w - 14.0f, vp), kText);
        AddText(ov, nx + nw - val_w - 4.0f, ny + 3.0f, vp, val, kYellow);

        const float tx0 = nx + 5.0f;
        const float tx1 = nx + nw - 5.0f;
        const float ty = ny + nh - 5.0f;
        AddLine(ov, tx0, ty, tx1, ty, kSurface2, 2.0f);
        const double span = std::max(1e-9, n.slider.max_val - n.slider.min_val);
        const float frac = (float)std::max(0.0, std::min(1.0, (n.slider.value - n.slider.min_val) / span));
        AddLine(ov, tx0, ty, tx0 + (tx1 - tx0) * frac, ty, kPeach, 2.0f);
        AddFill(ov, tx0 + (tx1 - tx0) * frac - 2.0f, ty - 2.5f, 4.0f, 5.0f, kText, kPeach, 1.0f);
        continue;
      }

      AddFill(ov, nx, ny, nw, nh, kBase, kSurface2, 1.0f);
      AddFill(ov, nx + 1.0f, ny + 1.0f, nw - 2.0f, 3.5f, accent, accent, 0.0f);
      const std::string title = n.nickname.empty() ? n.name : n.nickname;
      const float tp = 1.05f;
      AddText(ov, nx + 5.0f, ny + 7.0f, tp, Truncate(title, nw - 10.0f, tp), kText);
      DrawPorts(ov, n, nx, ny, nw, nh, sx);
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

// ============================================================================
// Interactive SDL3 Desktop Window (`--gui`)
// ============================================================================

bool TL_RunInteractiveStudioWindow(
  opentl::mvp::SceneDocument& base_scene,
  const std::vector<const ON_Mesh*>& base_meshes,
  const std::vector<const float*>& base_mesh_colors,
  TL_GhDocument* gh_doc,
  int max_frames,
  std::string* error_out)
{
#if !defined(OPENTL_HAS_SDL3)
  (void)base_scene; (void)base_meshes; (void)base_mesh_colors; (void)gh_doc; (void)max_frames;
  if (error_out) *error_out = "OpenTL was built without SDL3 support";
  return false;
#else
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    if (error_out) *error_out = std::string("SDL_Init failed: ") + SDL_GetError();
    return false;
  }

  const int win_w = 1920;
  const int win_h = 1080;
  SDL_Window* window = SDL_CreateWindow("Ohino Studio 9 WIP (SDL3 + Vulkan 1.2)", win_w, win_h, SDL_WINDOW_RESIZABLE);
  if (!window) {
    if (error_out) *error_out = std::string("SDL_CreateWindow failed: ") + SDL_GetError();
    SDL_Quit();
    return false;
  }
  SDL_Renderer* sdl_renderer = SDL_CreateRenderer(window, nullptr);
  if (!sdl_renderer) {
    if (error_out) *error_out = std::string("SDL_CreateRenderer failed: ") + SDL_GetError();
    SDL_DestroyWindow(window);
    SDL_Quit();
    return false;
  }
  SDL_Texture* texture = SDL_CreateTexture(
    sdl_renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, win_w, win_h);

  TL_GuiStudioRects studio_rects = TL_ComputeStudioRects(win_w, win_h);
  TL_FourViewLayout layout = TL_FourViewLayout::Default(win_w, win_h);
  TL_ApplyStudioViewports(layout, studio_rects);

  ON_BoundingBox scene_bbox = base_scene.GetSceneBoundingBox();
  for (const ON_Mesh* m : base_meshes) {
    ON_BoundingBox mb;
    if (m && m->GetBoundingBox(mb) && mb.IsValid()) scene_bbox.Union(mb);
  }
  if (!scene_bbox.IsValid())
    scene_bbox = ON_BoundingBox(ON_3dPoint(-15, -15, -5), ON_3dPoint(15, 15, 15));
  layout.ZoomToFitAll(scene_bbox, 0.92);
  TL_ApplyStudioViewports(layout, studio_rects);

  TL_VulkanFourViewRenderer vk_renderer;
  bool dirty = true;
  bool running = true;
  int frames = 0;
  int active_view = -1;
  bool dragging_view = false;
  bool orbit_mode = false;
  int active_slider = -1;
  float last_mx = 0.0f, last_my = 0.0f;

  auto find_viewport_at = [&](float x, float y) -> int {
    for (int i = 0; i < 4; ++i) {
      const TL_Viewport& vp = layout.View(i);
      if (x >= vp.m_left && x <= vp.m_left + vp.m_width &&
          y >= vp.m_top && y <= vp.m_top + vp.m_height)
        return i;
    }
    return -1;
  };

  auto update_slider_from_mouse = [&](int slider_idx, float mx_pos) {
    if (!gh_doc) return;
    const float bar_x = studio_rects.gh_params.x + 150.0f;
    const float bar_w = studio_rects.gh_params.w - 250.0f;
    const double t = std::clamp((double)(mx_pos - bar_x) / (double)bar_w, 0.0, 1.0);
    size_t ord = 0;
    for (const auto& n : gh_doc->Nodes()) {
      if (!n.is_slider) continue;
      if ((int)ord == slider_idx) {
        const double v = n.slider.min_val + t * (n.slider.max_val - n.slider.min_val);
        gh_doc->SetSliderValueByOrdinal(ord, v);
        dirty = true;
        break;
      }
      ++ord;
    }
  };

  while (running) {
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
      if (ev.type == SDL_EVENT_QUIT) {
        running = false;
      } else if (ev.type == SDL_EVENT_KEY_DOWN) {
        if (ev.key.key == SDLK_ESCAPE) running = false;
        else if (ev.key.key == SDLK_F) {
          layout.ZoomToFitAll(scene_bbox, 0.92);
          TL_ApplyStudioViewports(layout, studio_rects);
          dirty = true;
        }
      } else if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        const float mx_e = ev.button.x;
        const float my_e = ev.button.y;
        last_mx = mx_e; last_my = my_e;
        // Check Solved Parameters slider bars
        if (gh_doc && mx_e >= studio_rects.gh_params.x &&
            mx_e <= studio_rects.gh_params.x + studio_rects.gh_params.w &&
            my_e >= studio_rects.gh_params.y + 28.0f) {
          const int idx = (int)((my_e - (studio_rects.gh_params.y + 28.0f)) / 26.0f);
          if (idx >= 0 && idx < (int)gh_doc->SliderCount()) {
            active_slider = idx;
            update_slider_from_mouse(active_slider, mx_e);
          }
        } else {
          const int vi = find_viewport_at(mx_e, my_e);
          if (vi >= 0) {
            active_view = vi;
            dragging_view = true;
            orbit_mode = (ev.button.button == SDL_BUTTON_RIGHT) ||
                         (layout.View(vi).m_projection == TL_PROJECTION_PERSPECTIVE &&
                          ev.button.button == SDL_BUTTON_LEFT);
          }
        }
      } else if (ev.type == SDL_EVENT_MOUSE_BUTTON_UP) {
        dragging_view = false;
        active_slider = -1;
      } else if (ev.type == SDL_EVENT_MOUSE_MOTION) {
        const float mx_e = ev.motion.x;
        const float my_e = ev.motion.y;
        if (active_slider >= 0) {
          update_slider_from_mouse(active_slider, mx_e);
        } else if (dragging_view && active_view >= 0) {
          const float dx = mx_e - last_mx;
          const float dy = my_e - last_my;
          TL_Viewport& vp = layout.View(active_view);
          if (orbit_mode && vp.m_projection == TL_PROJECTION_PERSPECTIVE) {
            const double yaw = -dx * 0.008;
            const double pitch = -dy * 0.006;
            ON_Xform rot_z;
            rot_z.Rotation(yaw, ON_3dVector::ZAxis, ON_3dPoint::Origin);
            vp.m_camera_dir = rot_z * vp.m_camera_dir;
            ON_3dVector right = vp.ScreenRight();
            ON_Xform rot_r;
            rot_r.Rotation(pitch, right, ON_3dPoint::Origin);
            vp.m_camera_dir = rot_r * vp.m_camera_dir;
            vp.m_camera_dir.Unitize();
          } else {
            vp.Pan(dx, dy);
          }
          dirty = true;
        }
        last_mx = mx_e; last_my = my_e;
      } else if (ev.type == SDL_EVENT_MOUSE_WHEEL) {
        const int vi = find_viewport_at(last_mx, last_my);
        if (vi >= 0) {
          const double factor = (ev.wheel.y > 0) ? 1.12 : (1.0 / 1.12);
          ON_2dPoint anchor(last_mx, last_my);
          layout.View(vi).Zoom(factor, &anchor);
          dirty = true;
        }
      }
    }

    if (dirty) {
      opentl::mvp::SceneDocument frame_scene = base_scene;
      std::vector<const ON_Mesh*> frame_meshes = base_meshes;
      std::vector<const float*> frame_colors = base_mesh_colors;
      std::vector<ON_Mesh*> gh_baked_meshes;
      if (gh_doc) {
        gh_doc->BakeToScene(frame_scene, gh_baked_meshes, &frame_colors);
        for (ON_Mesh* m : gh_baked_meshes) frame_meshes.push_back(m);
      }

      TL_GuiStudioInfo info;
      info.title = "OHINO STUDIO 9 WIP";
      info.command = "COMMAND: _GRASSHOPPER _INTERACTIVE";
      info.object_count = frame_scene.Objects().size();
      info.mesh_count = frame_meshes.size();
      info.layout = &layout;
      info.gumball_origin = scene_bbox.Center();
      info.gumball_radius = std::max(4.0, scene_bbox.Diagonal().Length() * 0.12);

      TL_VulkanRenderOptions opts;
      opts.sample_count = 4;
      opts.enable_depth = true;
      opts.enable_shaded_meshes = true;
      opts.mesh_colors = frame_colors;
      TL_BuildStudioOverlay(studio_rects, gh_doc, info, opts.gui);

      TL_VulkanRenderResult res;
      if (vk_renderer.RenderScene(frame_scene, layout, res, frame_meshes, opts) &&
          !res.rgba.empty()) {
        SDL_UpdateTexture(texture, nullptr, res.rgba.data(), win_w * 4);
      }
      for (ON_Mesh* m : gh_baked_meshes) delete m;
      dirty = false;
    }

    SDL_RenderClear(sdl_renderer);
    SDL_RenderTexture(sdl_renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(sdl_renderer);
    ++frames;
    if (max_frames > 0 && frames >= max_frames) {
      running = false;
    } else {
      SDL_Delay(16);
    }
  }

  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(sdl_renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return true;
#endif
}
