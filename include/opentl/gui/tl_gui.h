#pragma once
/**
 * @file tl_gui.h
 * @brief Ohino Studio GUI layout: Rhino 9 WIP style workbench chrome, Grasshopper 10-tab
 *        battery palette, Trackpad/Mouse interaction state machine, and SDL3 window.
 */

#include "../tl_export.h"
#include "../viewport/tl_viewport.h"
#include "../viewport/tl_vulkan_renderer.h"
#include "../gh/tl_gh.h"

#include <string>
#include <vector>

/** Screen regions of the studio workbench, in pixels from the top-left. */
struct TL_CLASS TL_GuiStudioRects
{
  int width = 1920;
  int height = 1080;

  TL_GuiFill menu;       ///< application menu bar
  TL_GuiFill command;    ///< command prompt strip
  TL_GuiFill tabs;       ///< tool palette tabs
  TL_GuiFill dock;       ///< layers / inspector dock (left)
  TL_GuiFill viewports;  ///< 2x2 view area (centre-left)
  TL_GuiFill gh_canvas;  ///< Grasshopper node canvas + 10-tab battery palette (right)
  TL_GuiFill gh_params;  ///< Grasshopper slider inspector (bottom right)
  TL_GuiFill status;     ///< status bar
};

struct TL_CLASS TL_GuiStudioInfo
{
  std::string title = "OHINO STUDIO 9 WIP";
  std::string command = "COMMAND: _GRASSHOPPER _OPEN";
  std::string status = "X 0.000  Y 0.000  Z 0.000"
                       "   | MILLIMETERS   | GRID SNAP   | OSNAP   | GUMBALL   | VULKAN 1.2";
  std::string device = "APPLE M4";
  std::string gh_title = "GRASSHOPPER";
  std::string active_layer = "DEFAULT";
  std::vector<std::string> layers = {"DEFAULT", "CHASSIS", "DISPLAY", "KEYCAPS", "GH_PREVIEW"};
  size_t object_count = 0;
  size_t mesh_count = 0;

  // Optional 3D Gumball + OSnap projection context across the four viewports
  const TL_FourViewLayout* layout = nullptr;
  bool show_gumball = true;
  ON_3dPoint gumball_origin = ON_3dPoint::Origin;
  double gumball_radius = 12.0;
  int active_gumball_axis = -1; ///< -1 none, 0 X, 1 Y, 2 Z

  // Grasshopper interactive state
  int active_gh_category = 4;   ///< Default tab index: 4 = "Curve"
  float gh_zoom = 1.0f;         ///< Interactive canvas zoom multiplier
  float gh_pan_x = 0.0f;        ///< Interactive canvas pan X offset (px)
  float gh_pan_y = 0.0f;        ///< Interactive canvas pan Y offset (px)
  int maximized_view = -1;      ///< -1 = 2x2 four-view, 0..3 = maximized single viewport
};

enum class TL_StudioEventKind : int
{
  MouseDown     = 1,
  MouseUp       = 2,
  MouseMove     = 3,
  MouseWheel    = 4, ///< Mouse wheel or macOS trackpad 2-finger scroll
  TrackpadPinch = 5, ///< macOS trackpad 2-finger pinch-to-zoom gesture
  KeyDown       = 6
};

struct TL_CLASS TL_StudioInputEvent
{
  TL_StudioEventKind kind = TL_StudioEventKind::MouseMove;
  float x = 0.0f;
  float y = 0.0f;
  float dx = 0.0f;
  float dy = 0.0f;
  float wheel_x = 0.0f;
  float wheel_y = 0.0f;
  float pinch_scale = 1.0f;        ///< >1 zoom in, <1 zoom out
  bool is_trackpad_scroll = false; ///< True for continuous two-finger trackpad scroll
  int button = 1;                  ///< 1 = Left, 2 = Middle, 3 = Right
  int clicks = 1;                  ///< 2 = Double-click
  int key_code = 0;                ///< ASCII uppercase ('F', 'R', '1'..'4') or 27 for ESC
  bool mod_shift = false;
  bool mod_ctrl = false;
  bool mod_alt = false;
  bool mod_cmd = false;
};

struct TL_CLASS TL_StudioInputState
{
  int active_view = -1;
  int maximized_view = -1;         ///< -1 = 2x2 grid, 0..3 = single maximized viewport
  bool dragging_view = false;
  bool orbit_mode = false;
  bool zoom_drag_mode = false;     ///< Ctrl/Cmd + RMB vertical drag zoom
  int dragging_gumball_axis = -1;  ///< 0 = X, 1 = Y, 2 = Z
  int active_slider = -1;          ///< Solved Parameters panel slider index
  int dragging_canvas_slider = -1; ///< Node index of slider dragged directly on GH canvas
  int dragging_canvas_node = -1;   ///< Node index being repositioned on GH canvas
  bool panning_gh_canvas = false;
  int active_gh_category = 4;      ///< 0..9 index into TL_GhComponentCatalog::Categories()
  float gh_zoom = 1.0f;
  float gh_pan_x = 0.0f;
  float gh_pan_y = 0.0f;
  float last_mx = 0.0f;
  float last_my = 0.0f;
  ON_3dPoint gumball_origin = ON_3dPoint::Origin;
  double gumball_radius = 12.0;
  ON_3dPoint cursor_world = ON_3dPoint::Origin;
  std::string status_line =
    "X 0.000  Y 0.000  Z 0.000   | MILLIMETERS   | GRID SNAP   | OSNAP   | GUMBALL   | VULKAN 1.2";
};

/** Compute the Rhino 9 workbench regions for a canvas of `width` x `height`. */
TL_FUNCTION TL_GuiStudioRects TL_ComputeStudioRects(int width, int height);

/** Restrict the four viewports to the workbench's centre-left region (or maximize `maximized_view`). */
TL_FUNCTION void TL_ApplyStudioViewports(
  TL_FourViewLayout& layout,
  const TL_GuiStudioRects& rects,
  int maximized_view = -1);

/**
 * Process a single trackpad or mouse input event against the studio layout,
 * four-view cameras, 3D Gumball, and Grasshopper document. Returns true when
 * the frame needs to be re-rendered.
 */
TL_FUNCTION bool TL_ProcessStudioInputEvent(
  const TL_StudioInputEvent& ev,
  const TL_GuiStudioRects& rects,
  TL_FourViewLayout& layout,
  const ON_BoundingBox& scene_bbox,
  TL_GhDocument* gh_doc,
  TL_StudioInputState& state);

/**
 * Build the complete Rhino 9 style chrome draw list: menu, command bar, tool tabs,
 * layer dock, viewport caption bars, 3D Gumball & OSnap markers, Grasshopper 10-tab
 * battery palette, Grasshopper canvas (groups, wires, components, sliders, panels,
 * ports) and the status bar.
 */
TL_FUNCTION void TL_BuildStudioOverlay(
  const TL_GuiStudioRects& rects,
  const TL_GhDocument* gh,
  const TL_GuiStudioInfo& info,
  TL_GuiOverlay& out_overlay);

/**
 * Launch an interactive SDL3 desktop window presenting the Vulkan 4x MSAA studio
 * workbench (`--gui`).
 */
TL_FUNCTION bool TL_RunInteractiveStudioWindow(
  opentl::mvp::SceneDocument& base_scene,
  const std::vector<const ON_Mesh*>& base_meshes,
  const std::vector<const float*>& base_mesh_colors,
  TL_GhDocument* gh_doc = nullptr,
  int max_frames = 0,
  std::string* error_out = nullptr);
