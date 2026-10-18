#pragma once
/**
 * @file tl_gui.h
 * @brief Ohino Studio GUI layout: Rhino 9 WIP style workbench chrome.
 *
 * The GUI is emitted as a backend-neutral screen-space draw list (TL_GuiOverlay)
 * consumed by the Vulkan renderer, so the same layout drives both the headless
 * 1920x1080 framebuffer render (`--render-gui`) and an interactive SDL3 window.
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
  TL_GuiFill gh_canvas;  ///< Grasshopper node canvas (right)
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
};

/** Compute the Rhino 9 workbench regions for a canvas of `width` x `height`. */
TL_FUNCTION TL_GuiStudioRects TL_ComputeStudioRects(int width, int height);

/** Restrict the four viewports to the workbench's centre-left region. */
TL_FUNCTION void TL_ApplyStudioViewports(TL_FourViewLayout& layout, const TL_GuiStudioRects& rects);

/**
 * Build the complete Rhino 9 style chrome draw list: menu, command bar, tool tabs,
 * layer dock, viewport caption bars, Grasshopper canvas (groups, wires, components,
 * sliders, panels, ports) and the status bar.
 */
TL_FUNCTION void TL_BuildStudioOverlay(
  const TL_GuiStudioRects& rects,
  const TL_GhDocument* gh,
  const TL_GuiStudioInfo& info,
  TL_GuiOverlay& out_overlay);
