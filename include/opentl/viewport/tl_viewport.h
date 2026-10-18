#pragma once
/**
 * @file tl_viewport.h
 * @brief OpenTL Four-View viewport system (Rhino's Top / Perspective / Front / Right)
 *
 * Rhino's model space is presented through four standard viewports laid out in a
 * 2x2 canvas grid:
 *
 *   +-----------------------+-----------------------+
 *   | Top      (parallel,   | Perspective (lens)    |
 *   |           +Z down)    |                       |
 *   +-----------------------+-----------------------+
 *   | Front    (parallel,   | Right    (parallel,   |
 *   |           -Y up)      |           +X toward)  |
 *   +-----------------------+-----------------------+
 *
 * This subsystem is the *kernel* half of that UI: camera frames, world<->screen
 * projection, screen->world rays (for picking), pan / zoom / zoom-to-fit, and a
 * deterministic JSON persistence format so a layout survives a save/load round-trip.
 *
 * Binary provenance is *not* claimed here: `tl.dll` has no viewport exports — the
 * viewports live in RhinoCore's UI layer, which is out of MVP scope.  This module is
 * an independent implementation of the documented Rhino camera semantics.
 */

#include "../tl_export.h"

#include <string>
#include <vector>

enum TL_ProjectionType : int
{
  TL_PROJECTION_PARALLEL = 0,   ///< orthographic / parallel projection
  TL_PROJECTION_PERSPECTIVE = 1 ///< lens (perspective) projection
};

enum TL_StandardView : int
{
  TL_VIEW_TOP = 0,          ///< looking down -Z, up +Y, screen x = +X, screen y = +Y
  TL_VIEW_PERSPECTIVE = 1,  ///< lens view from the south-east-up octant
  TL_VIEW_FRONT = 2,        ///< looking along +Y, up +Z, screen x = +X, screen y = +Z
  TL_VIEW_RIGHT = 3         ///< looking along -X, up +Z, screen x = +Y, screen y = +Z
};

/**
 * One viewport: a rectangular slice of the canvas plus a camera frame.
 *
 * Screen coordinates are pixels measured from the *canvas* top-left corner with
 * y increasing downward, matching window/GUI conventions.
 */
class TL_CLASS TL_Viewport
{
public:
  TL_Viewport();
  TL_Viewport(const std::string& name, int standard_view);

  // ---- identification -----------------------------------------------------
  std::string m_name;
  int m_standard_view = TL_VIEW_TOP;
  int m_projection = TL_PROJECTION_PARALLEL;

  // ---- camera frame -------------------------------------------------------
  ON_3dPoint m_target = ON_3dPoint::Origin;      ///< point the camera looks at
  ON_3dVector m_camera_dir = ON_3dVector(0.0, 0.0, -1.0); ///< unit view direction
  ON_3dVector m_up = ON_3dVector(0.0, 1.0, 0.0); ///< unit up hint
  double m_target_distance = 10.0;               ///< eye = target - dir*distance (lens only)
  double m_lens_mm = 50.0;                       ///< 35mm-equivalent focal length
  double m_scale = 20.0;                         ///< parallel: pixels per world unit

  // ---- canvas rectangle ---------------------------------------------------
  int m_left = 0;
  int m_top = 0;
  int m_width = 800;
  int m_height = 600;

  // ---- frame queries ------------------------------------------------------
  ON_3dVector ScreenRight() const;   ///< unit vector for +screen-x
  ON_3dVector ScreenUp() const;      ///< unit vector for +screen-y (up)
  ON_3dPoint CameraLocation() const; ///< eye point (parallel views sit on the target plane)
  double VerticalFieldOfView() const;///< radians, derived from m_lens_mm (36mm frame)
  bool IsValid() const;

  // ---- projection ---------------------------------------------------------
  bool WorldToScreen(const ON_3dPoint& world, ON_2dPoint& screen) const;
  bool ScreenToRay(const ON_2dPoint& screen, ON_3dPoint& ray_from, ON_3dVector& ray_dir) const;
  /** Intersect the screen ray with the camera plane through the target. */
  bool ScreenToCameraPlane(const ON_2dPoint& screen, ON_3dPoint& world) const;
  /** Intersect the screen ray with a construction plane; false when parallel/skew. */
  bool ScreenToPlane(const ON_2dPoint& screen, const ON_Plane& plane, ON_3dPoint& world) const;

  // ---- navigation ---------------------------------------------------------
  void Pan(double dx_pixels, double dy_pixels);
  void Zoom(double factor, const ON_2dPoint* anchor_screen = nullptr);
  void ZoomToFit(const ON_BoundingBox& world_bbox, double margin = 0.9);
  void SetStandardView(int standard_view);
  void SetCanvasRect(int left, int top, int width, int height);

  // ---- persistence --------------------------------------------------------
  std::string ToJson() const;
};

/**
 * The four-view layout: Top / Perspective / Front / Right in a 2x2 grid.
 */
class TL_CLASS TL_FourViewLayout
{
public:
  TL_FourViewLayout(); ///< Rhino default: 1600x900 canvas, viewports already framed

  static TL_FourViewLayout Default(int canvas_width = 1600, int canvas_height = 900);

  int Count() const { return 4; }
  TL_Viewport& View(int index);
  const TL_Viewport& View(int index) const;
  int IndexOfName(const std::string& name) const;

  int CanvasWidth() const { return m_canvas_width; }
  int CanvasHeight() const { return m_canvas_height; }
  void SetCanvasSize(int width, int height); ///< keeps the 2x2 split proportional

  bool WorldToScreen(int view_index, const ON_3dPoint& world, ON_2dPoint& screen) const;
  bool ScreenToRay(int view_index, const ON_2dPoint& screen, ON_3dPoint& from, ON_3dVector& dir) const;

  /** Index of the point closest to `screen` within `pixel_tol`, or -1. */
  int PickPoint(int view_index, const ON_2dPoint& screen,
                const std::vector<ON_3dPoint>& world_points, double pixel_tol = 6.0) const;

  void ZoomToFitAll(const ON_BoundingBox& world_bbox, double margin = 0.9);

  /** Deterministic JSON (schema `opentl.fourview`, version 1). */
  std::string ToJson() const;
  bool FromJson(const std::string& json);
  bool SaveJson(const std::string& path) const;
  bool LoadJson(const std::string& path);

private:
  int m_canvas_width = 1600;
  int m_canvas_height = 900;
  TL_Viewport m_views[4];
};
