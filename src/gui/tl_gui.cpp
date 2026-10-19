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

float TextWidthOf(const std::string& s, float pixel) { return (float)s.size() * 6.0f * pixel; }
std::string TruncateStr(const std::string& s, float max_px, float pixel) {
  return Truncate(s, max_px, pixel);
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

void TL_ApplyStudioViewports(TL_FourViewLayout& layout, const TL_GuiStudioRects& rects,
                             int maximized_view) {
  layout.SetCanvasSize(rects.width, rects.height);
  const int vx = (int)rects.viewports.x;
  const int vy = (int)rects.viewports.y;
  const int vw = (int)rects.viewports.w;
  const int vh = (int)rects.viewports.h;
  if (maximized_view >= 0 && maximized_view < 4) {
    for (int i = 0; i < 4; ++i) {
      if (i == maximized_view) layout.View(i).SetCanvasRect(vx, vy, vw, vh);
      else layout.View(i).SetCanvasRect(vx, vy, 0, 0);
    }
    return;
  }
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

  // 10-tab Grasshopper Component Battery Palette (Params / Maths / Sets / Vector / Curve / Surface / Mesh / Intersect / Transform / Display)
  const auto& cats = TL_GhComponentCatalog::Categories();
  const int active_cat = std::clamp(info.active_gh_category, 0, (int)cats.size() - 1);
  float cat_x = canvas.x + 8.0f;
  const float cat_y = canvas.y + 28.0f;
  for (int ci = 0; ci < (int)cats.size(); ++ci) {
    const bool is_act = (ci == active_cat);
    const std::string cshort = cats[(size_t)ci].substr(0, 6);
    const float cw_tab = TextWidth(cshort, 0.95f) + 12.0f;
    AddFill(ov, cat_x, cat_y, cw_tab, 16.0f, is_act ? kSurface1 : kMantle, is_act ? kTeal : kSurface0, 1.0f);
    AddText(ov, cat_x + 6.0f, cat_y + 4.0f, 0.95f, cshort, is_act ? kTeal : kSubtext0);
    cat_x += cw_tab + 3.0f;
  }

  // Battery icons row for the active category
  const float pal_y = canvas.y + 46.0f;
  AddFill(ov, canvas.x + 8.0f, pal_y, canvas.w - 16.0f, 24.0f, kBase, kSurface0, 1.0f);
  const auto cat_specs = TL_GhComponentCatalog::ByCategory(cats[(size_t)active_cat]);
  float bx_pal = canvas.x + 12.0f;
  for (size_t bi = 0; bi < cat_specs.size() && bi < 10; ++bi) {
    const std::string bnick = Truncate(cat_specs[bi]->nickname, 56.0f, 0.95f);
    const float bw_chip = std::max(44.0f, TextWidth(bnick, 0.95f) + 12.0f);
    if (bx_pal + bw_chip > canvas.x + canvas.w - 12.0f) break;
    AddFill(ov, bx_pal, pal_y + 3.0f, bw_chip, 18.0f, kSurface0, kLavender, 1.0f);
    AddText(ov, bx_pal + 6.0f, pal_y + 8.0f, 0.95f, bnick, kText);
    bx_pal += bw_chip + 5.0f;
  }

  const float cx0 = canvas.x + 14.0f;
  const float cy0 = canvas.y + 74.0f;
  const float cw = canvas.w - 28.0f;
  const float ch = canvas.h - 86.0f;
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

    // Separate horizontal and vertical scale + interactive trackpad zoom/pan
    const float zoom = std::clamp(info.gh_zoom, 0.25f, 2.50f);
    const float sx = std::max(0.32f, std::min(0.75f, (cw - 36.0f) / cb.w)) * zoom;
    const float sy = std::max(sx, std::min(sx * 1.65f, (ch - 64.0f) / cb.h * zoom));
    const float ox = cx0 + (cw - cb.w * sx) * 0.5f - cb.x * sx + info.gh_pan_x;
    const float oy = cy0 + (ch - cb.h * sy) * 0.52f - cb.y * sy + info.gh_pan_y;
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
// Trackpad + mouse interaction state machine
// ============================================================================

namespace {

int StudioViewportAt(const TL_FourViewLayout& layout, float x, float y) {
  for (int i = 0; i < 4; ++i) {
    const TL_Viewport& vp = layout.View(i);
    if (vp.m_width <= 0 || vp.m_height <= 0) continue;
    if (x >= vp.m_left && x <= vp.m_left + vp.m_width &&
        y >= vp.m_top && y <= vp.m_top + vp.m_height)
      return i;
  }
  return -1;
}

bool InFill(const TL_GuiFill& f, float x, float y) {
  return x >= f.x && x <= f.x + f.w && y >= f.y && y <= f.y + f.h;
}

int GhCanvasPortAt(const TL_GhDocument& gh, const TL_GuiStudioRects& rects,
                   float x, float y, bool want_output, int* out_node, int* out_port) {
  if (!out_node || !out_port) return 0;
  const TL_GuiFill& canvas = rects.gh_canvas;
  const float cx0 = canvas.x + 14.0f;
  const float cy0 = canvas.y + 74.0f;
  const float cw = canvas.w - 28.0f;
  const float ch = canvas.h - 86.0f;
  const TL_GhRectF cb = gh.CanvasBounds();
  float sx = std::max(0.32f, std::min(0.75f, (cw - 36.0f) / std::max(1.0f, cb.w)));
  float sy = std::max(sx, std::min(sx * 1.65f, (ch - 64.0f) / std::max(1.0f, cb.h)));
  const float ox = cx0 + (cw - cb.w * sx) * 0.5f - cb.x * sx;
  const float oy = cy0 + (ch - cb.h * sy) * 0.52f - cb.y * sy;
  for (int i = (int)gh.Nodes().size() - 1; i >= 0; --i) {
    const TL_GhNode& n = gh.Nodes()[(size_t)i];
    if (n.is_group || n.is_scribble) continue;
    const float nx = ox + n.bounds.x * sx;
    const float ny = oy + n.bounds.y * sy;
    const float nw = n.is_slider ? std::min(112.0f, std::max(78.0f, n.bounds.w * sx * 0.65f))
                                 : std::max(38.0f, n.bounds.w * sx);
    const float nh = n.is_slider ? 20.0f : std::max(22.0f, n.bounds.h * sy);
    if (want_output) {
      if (n.is_slider || n.outputs.empty()) {
        if (std::hypot(x - (nx + nw), y - (ny + nh * 0.5f)) <= 10.0f) {
          *out_node = i; *out_port = -1; return 1;
        }
      }
      const int ocols = std::max<int>(1, (int)n.outputs.size());
      for (int p_i = 0; p_i < (int)n.outputs.size(); ++p_i) {
        const float py = ny + nh * ((float)(p_i + 1) / (float)(ocols + 1));
        if (std::hypot(x - (nx + nw), y - py) <= 10.0f) {
          *out_node = i; *out_port = p_i; return 1;
        }
      }
    } else {
      const int icols = std::max<int>(1, (int)n.inputs.size());
      for (int p_i = 0; p_i < (int)n.inputs.size(); ++p_i) {
        const float py = ny + nh * ((float)(p_i + 1) / (float)(icols + 1));
        if (std::hypot(x - nx, y - py) <= 10.0f) {
          *out_node = i; *out_port = p_i; return 1;
        }
      }
    }
  }
  return 0;
}

int GhCanvasNodeAt(const TL_GhDocument& gh, const TL_GuiStudioRects& rects,
                   float x, float y, bool want_slider, int* out_node) {
  if (!out_node) return 0;
  const TL_GuiFill& canvas = rects.gh_canvas;
  const float cx0 = canvas.x + 14.0f;
  const float cy0 = canvas.y + 74.0f;
  const float cw = canvas.w - 28.0f;
  const float ch = canvas.h - 86.0f;
  const TL_GhRectF cb = gh.CanvasBounds();
  float sx = std::max(0.32f, std::min(0.75f, (cw - 36.0f) / std::max(1.0f, cb.w)));
  float sy = std::max(sx, std::min(sx * 1.65f, (ch - 64.0f) / std::max(1.0f, cb.h)));
  const float ox = cx0 + (cw - cb.w * sx) * 0.5f - cb.x * sx;
  const float oy = cy0 + (ch - cb.h * sy) * 0.52f - cb.y * sy;
  for (int i = (int)gh.Nodes().size() - 1; i >= 0; --i) {
    const TL_GhNode& n = gh.Nodes()[(size_t)i];
    if (n.is_group || n.is_scribble) continue;
    if (n.is_slider != want_slider) continue;
    const float nx = ox + n.bounds.x * sx;
    const float ny = oy + n.bounds.y * sy;
    const float nw = std::max(38.0f, n.bounds.w * sx);
    const float nh = std::max(22.0f, n.bounds.h * sy);
    if (x >= nx && x <= nx + nw && y >= ny && y <= ny + nh) { *out_node = i; return 1; }
  }
  return 0;
}

}  // namespace

bool TL_ProcessStudioInputEvent(const TL_StudioInputEvent& ev, const TL_GuiStudioRects& rects,
                                TL_FourViewLayout& layout, const ON_BoundingBox& scene_bbox,
                                TL_GhDocument* gh_doc, TL_StudioInputState& state) {
  const int hovered = StudioViewportAt(layout, ev.x, ev.y);
  const bool in_viewports = InFill(rects.viewports, ev.x, ev.y);
  const bool in_gh_canvas = InFill(rects.gh_canvas, ev.x, ev.y);
  const bool in_gh_params = InFill(rects.gh_params, ev.x, ev.y);

  auto refresh_status = [&]() {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "X %.3f  Y %.3f  Z %.3f   | MILLIMETERS   | %s   | OSNAP   | GUMBALL   | GH ZOOM %.2FX",
                  state.cursor_world.x, state.cursor_world.y, state.cursor_world.z,
                  state.maximized_view >= 0 ? "MAXIMIZED" : "4-VIEW", state.gh_zoom);
    state.status_line = buf;
  };

  switch (ev.kind) {
    case TL_StudioEventKind::KeyDown: {
      const int k = std::toupper(ev.key_code);
      if (k == 'F') {
        layout.ZoomToFitAll(scene_bbox, 0.92);
        TL_ApplyStudioViewports(layout, rects, state.maximized_view);
        return true;
      }
      if (k >= '1' && k <= '4') {
        state.maximized_view = ((state.maximized_view == k - '1') ? -1 : (k - '1'));
        TL_ApplyStudioViewports(layout, rects, state.maximized_view);
        refresh_status();
        return true;
      }
      if (k == '0' || ev.key_code == 27) {
        state.maximized_view = -1;
        TL_ApplyStudioViewports(layout, rects, state.maximized_view);
        refresh_status();
        return true;
      }
      return false;
    }
    case TL_StudioEventKind::MouseDown: {
      state.last_mx = ev.x;
      state.last_my = ev.y;

      // (a) Solved Parameters panel: grab a slider track
      if (in_gh_params && gh_doc && ev.button == 1) {
        const int idx = (int)((ev.y - (rects.gh_params.y + 28.0f)) / 26.0f);
        if (idx >= 0 && idx < (int)gh_doc->SliderCount()) {
          const float bar_x = rects.gh_params.x + 150.0f;
          const float bar_w = rects.gh_params.w - 250.0f;
          double t = std::clamp((double)(ev.x - bar_x) / (double)bar_w, 0.0, 1.0);
          size_t ord = 0;
          for (const auto& n : gh_doc->Nodes()) {
            if (!n.is_slider) continue;
            if ((int)ord == idx) {
              gh_doc->SetSliderValueByOrdinal(ord, n.slider.min_val + t * (n.slider.max_val - n.slider.min_val));
              break;
            }
            ++ord;
          }
          state.active_slider = idx;
          return true;
        }
      }

      // (b) Grasshopper canvas: category tabs, battery palette, sliders, nodes, pan
      if (in_gh_canvas && ev.button == 1) {
        const TL_GuiFill& canvas = rects.gh_canvas;
        const float cat_y = canvas.y + 28.0f;
        if (ev.y >= cat_y && ev.y <= cat_y + 16.0f) {
          float cat_x = canvas.x + 8.0f;
          const auto& cats = TL_GhComponentCatalog::Categories();
          for (int ci = 0; ci < (int)cats.size(); ++ci) {
            const std::string cshort = cats[(size_t)ci].substr(0, 6);
            const float w_tab = TextWidthOf(cshort, 0.95f) + 12.0f;
            if (ev.x >= cat_x && ev.x <= cat_x + w_tab) {
              state.active_gh_category = ci;
              return true;
            }
            cat_x += w_tab + 3.0f;
          }
        }
        const float pal_y = canvas.y + 46.0f;
        if (ev.y >= pal_y && ev.y <= pal_y + 24.0f && gh_doc) {
          float bx = canvas.x + 12.0f;
          const auto specs = TL_GhComponentCatalog::ByCategory(
            TL_GhComponentCatalog::Categories()[(size_t)state.active_gh_category]);
          for (size_t bi = 0; bi < specs.size() && bi < 10; ++bi) {
            const std::string bnick = TruncateStr(specs[bi]->nickname, 56.0f, 0.95f);
            const float bw = std::max(44.0f, TextWidthOf(bnick, 0.95f) + 12.0f);
            if (bx + bw > canvas.x + canvas.w - 12.0f) break;
            if (ev.x >= bx && ev.x <= bx + bw) {
              gh_doc->AddComponent(specs[bi]->name, 40.0f + 180.0f * (float)(gh_doc->Nodes().size() % 5),
                                   60.0f + 80.0f * (float)(gh_doc->Nodes().size() % 4));
              return true;
            }
            bx += bw + 5.0f;
          }
        }
        if (gh_doc) {
          int w_node = -1, w_port = -1;
          if (GhCanvasPortAt(*gh_doc, rects, ev.x, ev.y, true, &w_node, &w_port)) {
            state.wiring_from_node = w_node;
            state.wiring_from_output = w_port;
            state.wiring_cursor_x = ev.x;
            state.wiring_cursor_y = ev.y;
            return true;
          }
          int slider_node = -1;
          if (GhCanvasNodeAt(*gh_doc, rects, ev.x, ev.y, true, &slider_node)) {
            state.dragging_canvas_slider = slider_node;
            return true;
          }
          int any_node = -1;
          if (GhCanvasNodeAt(*gh_doc, rects, ev.x, ev.y, false, &any_node)) {
            state.dragging_canvas_node = any_node;
            return true;
          }
        }
      }
      if (in_gh_canvas && (ev.button == 2 || ev.button == 3)) {
        state.panning_gh_canvas = true;
        return true;
      }

      // (c) 3D viewports: Gumball axis, double-click maximize, orbit/pan/zoom drag
      if (hovered >= 0 && in_viewports) {
        state.active_view = hovered;
        if (ev.clicks >= 2) {
          const TL_Viewport& vp = layout.View(hovered);
          if (ev.y <= vp.m_top + 26.0f) {
            state.maximized_view = (state.maximized_view == hovered) ? -1 : hovered;
            TL_ApplyStudioViewports(layout, rects, state.maximized_view);
            refresh_status();
            return true;
          }
        }
        if (ev.button == 1 && state.dragging_gumball_axis < 0) {
          ON_2dPoint c2d;
          if (layout.View(hovered).WorldToScreen(state.gumball_origin, c2d)) {
            const double R = std::max(1.0, state.gumball_radius);
            const ON_3dPoint axes[3] = {
              state.gumball_origin + ON_3dVector(R, 0, 0),
              state.gumball_origin + ON_3dVector(0, R, 0),
              state.gumball_origin + ON_3dVector(0, 0, R)};
            for (int a = 0; a < 3; ++a) {
              ON_2dPoint p2d;
              if (!layout.View(hovered).WorldToScreen(axes[a], p2d)) continue;
              double dx = p2d.x - c2d.x, dy = p2d.y - c2d.y;
              double len = std::hypot(dx, dy);
              if (len < 6.0) continue;
              const double ux = dx / len, uy = dy / len;
              const double ex = c2d.x + ux * 42.0, ey = c2d.y + uy * 42.0;
              const double dist = std::hypot(ev.x - ex, ev.y - ey);
              const double proj = (ev.x - c2d.x) * ux + (ev.y - c2d.y) * uy;
              if (dist < 14.0 && proj > 8.0) {
                state.dragging_gumball_axis = a;
                return true;
              }
            }
          }
        }
        state.dragging_view = true;
        const bool is_persp = (layout.View(hovered).m_projection == TL_PROJECTION_PERSPECTIVE);
        state.zoom_drag_mode = (ev.button == 3 && (ev.mod_ctrl || ev.mod_cmd));
        if (state.zoom_drag_mode) state.orbit_mode = false;
        else if (ev.button == 2 || (ev.button == 3 && ev.mod_shift)) state.orbit_mode = false;
        else if (ev.button == 3) state.orbit_mode = is_persp;
        else if (ev.button == 1) state.orbit_mode = is_persp && ev.mod_alt;
        return true;
      }
      return false;
    }
    case TL_StudioEventKind::MouseUp: {
      bool connected = false;
      if (state.wiring_from_node >= 0 && gh_doc) {
        int dst_node = -1, dst_port = -1;
        if (GhCanvasPortAt(*gh_doc, rects, ev.x, ev.y, false, &dst_node, &dst_port)) {
          connected = gh_doc->Connect(state.wiring_from_node, state.wiring_from_output, dst_node, dst_port);
        }
      }
      state.wiring_from_node = -1;
      state.wiring_from_output = -1;
      state.dragging_view = false;
      state.active_slider = -1;
      state.dragging_canvas_slider = -1;
      state.dragging_canvas_node = -1;
      state.panning_gh_canvas = false;
      state.dragging_gumball_axis = -1;
      return connected;
    }
    case TL_StudioEventKind::MouseMove: {
      const float dx = ev.x - state.last_mx;
      const float dy = ev.y - state.last_my;
      state.last_mx = ev.x;
      state.last_my = ev.y;
      bool dirty = false;

      if (state.wiring_from_node >= 0) {
        state.wiring_cursor_x = ev.x;
        state.wiring_cursor_y = ev.y;
        return true;
      }
      if (state.active_slider >= 0 && gh_doc) {
        const float bar_x = rects.gh_params.x + 150.0f;
        const float bar_w = rects.gh_params.w - 250.0f;
        const double t = std::clamp((double)(ev.x - bar_x) / (double)bar_w, 0.0, 1.0);
        size_t ord = 0;
        for (const auto& n : gh_doc->Nodes()) {
          if (!n.is_slider) continue;
          if ((int)ord == state.active_slider) {
            gh_doc->SetSliderValueByOrdinal(ord, n.slider.min_val + t * (n.slider.max_val - n.slider.min_val));
            break;
          }
          ++ord;
        }
        return true;
      }
      if (state.dragging_canvas_slider >= 0 && gh_doc) {
        auto& nodes = gh_doc->NodesMutable();
        if (state.dragging_canvas_slider < (int)nodes.size()) {
          TL_GhNode& n = nodes[(size_t)state.dragging_canvas_slider];
          double t = std::clamp((double)(ev.x - (rects.gh_canvas.x + 40.0f)) /
                                  std::max(40.0, (double)rects.gh_canvas.w - 80.0), 0.0, 1.0);
          gh_doc->SetSliderValue(n.nickname, n.slider.min_val + t * (n.slider.max_val - n.slider.min_val));
          return true;
        }
      }
      if (state.dragging_canvas_node >= 0 && gh_doc) {
        auto& nodes = gh_doc->NodesMutable();
        if (state.dragging_canvas_node < (int)nodes.size()) {
          TL_GhNode& n = nodes[(size_t)state.dragging_canvas_node];
          gh_doc->MoveNode(state.dragging_canvas_node, n.bounds.x + dx, n.bounds.y + dy);
          return true;
        }
      }
      if (state.panning_gh_canvas) {
        state.gh_pan_x += dx;
        state.gh_pan_y += dy;
        return true;
      }
      if (state.dragging_gumball_axis >= 0 && state.active_view >= 0) {
        const TL_Viewport& vp = layout.View(state.active_view);
        ON_2dPoint c2d;
        if (vp.WorldToScreen(state.gumball_origin, c2d)) {
          const double R = std::max(1.0, state.gumball_radius);
          ON_3dPoint tip = state.gumball_origin;
          if (state.dragging_gumball_axis == 1) tip += ON_3dVector(0, R, 0);
          else if (state.dragging_gumball_axis == 2) tip += ON_3dVector(0, 0, R);
          else tip += ON_3dVector(R, 0, 0);
          ON_2dPoint t2d;
          if (vp.WorldToScreen(tip, t2d)) {
            const double sx = t2d.x - c2d.x, sy = t2d.y - c2d.y;
            const double slen2 = sx * sx + sy * sy;
            if (slen2 > 1e-6) {
              const double along = ((double)ev.dx * sx + (double)ev.dy * sy) / slen2 * R;
              ON_3dPoint offs = ON_3dPoint::Origin;
              if (state.dragging_gumball_axis == 1) offs = ON_3dVector(0, along, 0);
              else if (state.dragging_gumball_axis == 2) offs = ON_3dVector(0, 0, along);
              else offs = ON_3dVector(along, 0, 0);
              state.gumball_origin += offs;
              refresh_status();
            }
          }
        }
        return true;
      }
      if (state.dragging_view && state.active_view >= 0) {
        TL_Viewport& vp = layout.View(state.active_view);
        if (state.zoom_drag_mode) {
          vp.Zoom(dy > 0 ? 1.0 / (1.0 + std::fabs(dy) * 0.01) : 1.0 + std::fabs(dy) * 0.01, nullptr);
        } else if (state.orbit_mode && vp.m_projection == TL_PROJECTION_PERSPECTIVE) {
          ON_Xform rz, rr;
          rz.Rotation(-dx * 0.008, ON_3dVector::ZAxis, ON_3dPoint::Origin);
          vp.m_camera_dir = rz * vp.m_camera_dir;
          rr.Rotation(-dy * 0.006, vp.ScreenRight(), ON_3dPoint::Origin);
          vp.m_camera_dir = rr * vp.m_camera_dir;
          vp.m_camera_dir.Unitize();
        } else {
          vp.Pan(dx, dy);
        }
        dirty = true;
      }
      if (hovered >= 0) {
        ON_3dPoint w = ON_3dPoint::Origin;
        if (layout.View(hovered).ScreenToPlane(ON_2dPoint(ev.x, ev.y), ON_Plane::World_xy, w))
          state.cursor_world = w;
        refresh_status();
      }
      return dirty;
    }
    case TL_StudioEventKind::MouseWheel: {
      const float scroll = (std::fabs(ev.wheel_y) > 1e-6f) ? ev.wheel_y : -ev.wheel_x;
      if (in_gh_canvas) {
        if (ev.mod_ctrl || ev.mod_cmd || !ev.is_trackpad_scroll) {
          state.gh_zoom = std::clamp(state.gh_zoom * (1.0f + scroll * 0.10f), 0.25f, 2.50f);
        } else {
          state.gh_pan_x += ev.wheel_x * 12.0f;
          state.gh_pan_y += ev.wheel_y * 12.0f;
        }
        refresh_status();
        return true;
      }
      if (hovered >= 0) {
        TL_Viewport& vp = layout.View(hovered);
        const bool want_zoom = ev.mod_ctrl || ev.mod_cmd || !ev.is_trackpad_scroll;
        if (want_zoom) {
          ON_2dPoint anchor(ev.x, ev.y);
          vp.Zoom(1.0 + scroll * 0.12, &anchor);
        } else if (ev.mod_alt) {
          ON_Xform rz;
          rz.Rotation(-ev.wheel_x * 0.05, ON_3dVector::ZAxis, ON_3dPoint::Origin);
          vp.m_camera_dir = rz * vp.m_camera_dir;
          vp.m_camera_dir.Unitize();
        } else {
          vp.Pan(ev.wheel_x * 14.0f, ev.wheel_y * 14.0f);
        }
        return true;
      }
      return false;
    }
    case TL_StudioEventKind::TrackpadPinch: {
      const float s = (std::fabs(ev.pinch_scale) < 1e-4f) ? 1.0f : ev.pinch_scale;
      if (in_gh_canvas) {
        state.gh_zoom = std::clamp(state.gh_zoom * s, 0.25f, 2.50f);
        refresh_status();
        return true;
      }
      if (hovered >= 0) {
        ON_2dPoint anchor(ev.x, ev.y);
        layout.View(hovered).Zoom(s, &anchor);
        return true;
      }
      return false;
    }
  }
  return false;
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
  const int win_w = 1920, win_h = 1080;
  SDL_Window* window = SDL_CreateWindow("Ohino Studio 9 WIP (SDL3 + Vulkan 1.2)", win_w, win_h,
                                        SDL_WINDOW_RESIZABLE);
  SDL_Renderer* rend = window ? SDL_CreateRenderer(window, nullptr) : nullptr;
  SDL_Texture* tex = rend ? SDL_CreateTexture(rend, SDL_PIXELFORMAT_RGBA32,
                                              SDL_TEXTUREACCESS_STREAMING, win_w, win_h) : nullptr;
  if (!window || !rend || !tex) {
    if (error_out) *error_out = std::string("SDL window creation failed: ") + SDL_GetError();
    if (tex) SDL_DestroyTexture(tex);
    if (rend) SDL_DestroyRenderer(rend);
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
    return false;
  }

  const TL_GuiStudioRects rects = TL_ComputeStudioRects(win_w, win_h);
  TL_FourViewLayout layout = TL_FourViewLayout::Default(win_w, win_h);
  TL_ApplyStudioViewports(layout, rects, -1);

  ON_BoundingBox scene_bbox = base_scene.GetSceneBoundingBox();
  for (const ON_Mesh* m : base_meshes) {
    ON_BoundingBox mb;
    if (m && m->GetBoundingBox(mb) && mb.IsValid()) scene_bbox.Union(mb);
  }
  if (!scene_bbox.IsValid())
    scene_bbox = ON_BoundingBox(ON_3dPoint(-15, -15, -5), ON_3dPoint(15, 15, 15));
  layout.ZoomToFitAll(scene_bbox, 0.92);
  TL_ApplyStudioViewports(layout, rects, -1);

  TL_StudioInputState state;
  state.gumball_origin = scene_bbox.Center();
  state.gumball_radius = std::max(4.0, scene_bbox.Diagonal().Length() * 0.12);

  TL_VulkanFourViewRenderer vk;
  bool dirty = true, running = true;
  int frames = 0;

  auto pump = [&](const TL_StudioInputEvent& ev) {
    if (TL_ProcessStudioInputEvent(ev, rects, layout, scene_bbox, gh_doc, state)) dirty = true;
  };

  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      const bool shift = (e.key.mod & SDL_KMOD_SHIFT) != 0;
      const bool ctrl = (e.key.mod & SDL_KMOD_CTRL) != 0;
      const bool alt = (e.key.mod & SDL_KMOD_ALT) != 0;
      const bool gui = (e.key.mod & SDL_KMOD_GUI) != 0;
      TL_StudioInputEvent ev;
      switch (e.type) {
        case SDL_EVENT_QUIT: running = false; break;
        case SDL_EVENT_KEY_DOWN:
          if (e.key.key == SDLK_ESCAPE) { running = false; break; }
          ev.kind = TL_StudioEventKind::KeyDown;
          ev.key_code = (int)e.key.key;
          pump(ev);
          break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_UP:
          ev.kind = (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) ? TL_StudioEventKind::MouseDown
                                                           : TL_StudioEventKind::MouseUp;
          ev.x = e.button.x; ev.y = e.button.y;
          ev.button = (e.button.button == SDL_BUTTON_LEFT) ? 1
                    : (e.button.button == SDL_BUTTON_MIDDLE ? 2 : 3);
          ev.clicks = e.button.clicks;
          ev.mod_shift = shift; ev.mod_ctrl = ctrl; ev.mod_alt = alt; ev.mod_cmd = gui;
          pump(ev);
          break;
        case SDL_EVENT_MOUSE_MOTION: {
          ev.kind = TL_StudioEventKind::MouseMove;
          ev.x = e.motion.x; ev.y = e.motion.y;
          ev.dx = e.motion.xrel; ev.dy = e.motion.yrel;
          pump(ev);
          break;
        }
        case SDL_EVENT_MOUSE_WHEEL: {
          ev.kind = TL_StudioEventKind::MouseWheel;
          ev.x = e.wheel.mouse_x; ev.y = e.wheel.mouse_y;
          const float fy = (e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED) ? -e.wheel.y : e.wheel.y;
          const float fx = (e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED) ? -e.wheel.x : e.wheel.x;
          ev.wheel_x = fx; ev.wheel_y = fy;
          // Trackpad two-finger scroll arrives as continuous sub-tick deltas.
          ev.is_trackpad_scroll = (e.wheel.integer_y == 0 && std::fabs(fy) < 3.0f);
          ev.mod_shift = shift; ev.mod_ctrl = ctrl; ev.mod_alt = alt; ev.mod_cmd = gui;
          pump(ev);
          break;
        }
        case SDL_EVENT_PINCH_UPDATE:
          ev.kind = TL_StudioEventKind::TrackpadPinch;
          ev.pinch_scale = e.pinch.scale;
          ev.x = state.last_mx; ev.y = state.last_my;
          pump(ev);
          break;
        default: break;
      }
    }

    if (dirty) {
      opentl::mvp::SceneDocument frame_scene = base_scene;
      std::vector<const ON_Mesh*> frame_meshes = base_meshes;
      std::vector<const float*> frame_colors = base_mesh_colors;
      std::vector<ON_Mesh*> baked;
      if (gh_doc) {
        gh_doc->BakeToScene(frame_scene, baked, &frame_colors);
        for (ON_Mesh* m : baked) frame_meshes.push_back(m);
      }
      TL_GuiStudioInfo info;
      info.command = "COMMAND: _GRASSHOPPER _INTERACTIVE";
      info.object_count = frame_scene.Objects().size();
      info.mesh_count = frame_meshes.size();
      info.layout = &layout;
      info.gumball_origin = state.gumball_origin;
      info.gumball_radius = state.gumball_radius;
      info.active_gumball_axis = state.dragging_gumball_axis;
      info.active_gh_category = state.active_gh_category;
      info.gh_zoom = state.gh_zoom;
      info.gh_pan_x = state.gh_pan_x;
      info.gh_pan_y = state.gh_pan_y;
      info.maximized_view = state.maximized_view;
      info.status = state.status_line;

      TL_VulkanRenderOptions opts;
      opts.sample_count = 4;
      opts.enable_depth = true;
      opts.enable_shaded_meshes = true;
      opts.mesh_colors = frame_colors;
      TL_BuildStudioOverlay(rects, gh_doc, info, opts.gui);

      TL_VulkanRenderResult res;
      if (vk.RenderScene(frame_scene, layout, res, frame_meshes, opts) && !res.rgba.empty())
        SDL_UpdateTexture(tex, nullptr, res.rgba.data(), win_w * 4);
      for (ON_Mesh* m : baked) delete m;
      dirty = false;
    }

    SDL_RenderClear(rend);
    SDL_RenderTexture(rend, tex, nullptr, nullptr);
    SDL_RenderPresent(rend);
    ++frames;
    if (max_frames > 0 && frames >= max_frames) running = false;
    else SDL_Delay(16);
  }

  SDL_DestroyTexture(tex);
  SDL_DestroyRenderer(rend);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return true;
#endif
}
