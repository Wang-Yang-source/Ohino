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

// ============================================================================
// Scalar line/plane primitive
// tl.dll!?TL_3DX_LineXPlane@@YAH... (RVA 0x305d0, 378 bytes, 0 IAT calls)
// ============================================================================

enum TL_LinePlaneStatus : int
{
  TL_LINEXPLANE_COPLANAR = -2, ///< line lies in the plane (infinitely many solutions)
  TL_LINEXPLANE_PARALLEL = -1, ///< parallel and not coplanar (no solution)
  TL_LINEXPLANE_AT_ZERO = 0,   ///< unique solution at parameter exactly 0
  TL_LINEXPLANE_UNIQUE = 1     ///< unique solution at a non-zero parameter
};

/**
 * Exact reimplementation of `tl.dll!TL_3DX_LineXPlane`
 * (`int TL_3DX_LineXPlane(const ON_3dPoint& line_from, const ON_3dVector& dir,
 *                        const ON_3dPoint& plane_pt, const ON_3dVector& plane_normal,
 *                        double* t_out)`, RVA 0x305d0, zero imports):
 *
 *   den = dir . n                    num = (plane_pt - line_from) . n
 *   den == 0 : *t = 0 ; return (num == 0) ? -2 : -1
 *   num == 0 : *t = 0 ; return 0
 *   otherwise: return 1, with saturating division --
 *     |den| >= 1 : |num| <= |den|*DBL_MIN  -> t = 0     else t = num/den
 *     |den| <  1 : |num| >  |den|*DBL_MAX  -> t = +-DBL_MAX (sign of num/den)
 *                                             else t = num/den
 *
 * The saturation is what keeps "almost parallel" configurations from producing inf.
 */
TL_FUNCTION int TL_3DX_LineXPlane(
  const ON_3dPoint& line_from,
  const ON_3dVector& dir,
  const ON_3dPoint& plane_pt,
  const ON_3dVector& plane_normal,
  double* t_out
);

// ============================================================================
// Line distance C-ABI entry points
// rhcommon_c!ON_Line_DistanceToPoint (RVA 0xd0210, 127 B)
// rhcommon_c!ON_Line_DistanceToLine  (RVA 0xd0290,  38 B)
// ============================================================================

/**
 * `bool maximum` selects ON_Line::MinimumDistanceTo / MaximumDistanceTo.
 * A null input returns the -1.0 sentinel (never 0.0), matching the binary.
 */
TL_FUNCTION double TL_LineDistanceToPoint(const ON_Line* line, ON_3dPoint test_point, bool maximum);

/** Same contract as TL_LineDistanceToPoint, line-vs-line variant. */
TL_FUNCTION double TL_LineDistanceToLine(const ON_Line* line_a, const ON_Line* line_b, bool maximum);

/**
 * Second, independent curve x line strategy ("subdivision" solver).
 *
 * The shipped `TL_IntersectCurveLine` uses dense sampling + golden-section + damped
 * Newton.  This variant solves the same problem with a recursive bounding-box
 * subdivision whose pruning relies only on the 1-Lipschitz property of the
 * point-to-line distance, and refines each surviving leaf with the same Newton
 * polish.  Two independent algorithms over the same inputs is what lets the test
 * suite cross-check (对拍) the primary solver instead of trusting it blindly.
 *
 * NOTE: `rhcommon_c!ON_Intersect_CurveLine` forwards to opennurbs'
 * `ON_Curve::IntersectCurve`; the opennurbs revision vendored in `external/` predates
 * `ON_X_EVENT` and has no curve-curve intersection, so an instruction-level replay of
 * that path is not possible in this tree.  This is the documented substitute.
 */
TL_FUNCTION int TL_IntersectCurveLineBySubdivision(
  const ON_Curve& curve,
  const ON_Line& line,
  double tolerance,
  ON_SimpleArray<TL_CurveCurveIntersectionEvent>& events
);
