#pragma once
/**
 * @file tl_intersect.h
 * @brief OpenTL Curve/Surface/Brep/Line/Plane Intersection subsystem (intersect/)
 *
 * Corresponds to tl.dll intersection exports:
 *   - TL_IntersectCurveLine
 *   - TL_IntersectCurves
 *   - TL_CurveSurfaceIntersect
 *   - TL_SurfaceSurfaceIntersect
 *   - TL_IntersectBrepPlane
 */

#include "../tl_export.h"

struct TL_CurveCurveIntersectionEvent
{
  double param_a = 0.0;
  double param_b = 0.0;
  ON_3dPoint point_a = ON_3dPoint::Origin;
  ON_3dPoint point_b = ON_3dPoint::Origin;
  bool is_overlap = false;
};

/**
 * Intersect an ON_NurbsCurve (or general ON_Curve) with a 3D line segment or infinite line
 * using subdivision + Newton-Raphson refinement on distance to line.
 */
TL_FUNCTION int TL_IntersectCurveLine(
  const ON_Curve& curve,
  const ON_Line& line,
  double tolerance,
  ON_SimpleArray<TL_CurveCurveIntersectionEvent>& events
);

/**
 * Intersect a curve with a plane `plane` within `tolerance`.
 */
TL_FUNCTION int TL_IntersectCurvePlane(
  const ON_Curve& curve,
  const ON_Plane& plane,
  double tolerance,
  ON_SimpleArray<double>& curve_params,
  ON_SimpleArray<ON_3dPoint>& intersection_points
);
