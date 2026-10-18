#pragma once
/**
 * @file tl_closest_point.h
 * @brief OpenTL Curve & Surface Closest-Point Newton-Raphson Solver subsystem (closest_point/)
 *
 * Corresponds to tl.dll closest-point exports:
 *   - TL_CurveClosestPoint / ON_Curve::GetClosestPoint
 *   - TL_SurfaceClosestPoint / ON_Surface::GetClosestPoint
 */

#include "../tl_export.h"

/**
 * Find the parameter `t` on `curve` closest to `test_point` using coarse multi-span sampling
 * followed by damped Newton-Raphson minimization of:
 *   f(t) = (C(t) - P) . C'(t) = 0
 *   f'(t) = C'(t) . C'(t) + (C(t) - P) . C''(t)
 */
TL_FUNCTION bool TL_GetCurveClosestPoint(
  const ON_Curve& curve,
  const ON_3dPoint& test_point,
  double* t_out,
  double maximum_distance = 0.0,
  const ON_Interval* sub_domain = nullptr
);

/**
 * Find the parameters `(u, v)` on `surface` closest to `test_point` using 2D grid seeding
 * followed by 2x2 Gauss-Newton / Newton-Raphson iteration on first and second partial derivatives.
 */
TL_FUNCTION bool TL_GetSurfaceClosestPoint(
  const ON_Surface& surface,
  const ON_3dPoint& test_point,
  double* u_out,
  double* v_out,
  double maximum_distance = 0.0
);

/**
 * `tl.dll!TL_GetClosestPoint(const ON_Line&, const ON_Line&, double*, double*)`
 * (RVA 0x214670, 2016 bytes) -- 4-argument overload.
 *
 * Two distinct branches (both verified in the disassembly):
 *   A) `ON_Intersect(lineA, lineB, &ta, &tb)` succeeds -> use the analytic parameters
 *      when both lie in [0,1]; otherwise clamp the offending end to the segment
 *      boundary and re-project the other side.
 *   B) analytic solve fails (parallel or skew) -> project all four segment endpoints
 *      onto the opposite line, keep the pair with the smallest separation, then do one
 *      refinement pass.
 *
 * IMPORTANT CONTRACT: the `true` return means "a usable parameter pair was produced",
 * NOT "the lines intersect". Intersection-ness must be decided by the caller with a
 * distance threshold. This differs from `rhcommon_c!ON_Intersect_LineLine`, whose bool
 * means "did intersect" -- the two must never be conflated.
 */
TL_FUNCTION bool TL_GetClosestPoint(
  const ON_Line& line_a,
  const ON_Line& line_b,
  double* t_a,
  double* t_b
);

/**
 * `tl.dll!TL_GetClosestPoint(const ON_Line&, const ON_Line&, double*, double*, double,
 *                            const ON_Interval*, const ON_Interval*)`
 * (RVA 0x214540, 299 bytes) -- 7-argument overload.
 *
 * The binary tail-calls the `ON_BezierCurve x ON_BezierCurve` closest-point solver after
 * lifting both lines to degree-1 Bezier curves (`dim=3, rational=false, order=2`), i.e.
 * a line closest-point query is the degree-1 special case of the Bezier solver.
 * This implementation performs the same lift and delegates to the 4-argument overload
 * (数学等价, not instruction-identical), then applies the caller's sub-domains and the
 * `maximum_distance` gate (0 or non-finite = no gate; returns false when the achieved
 * distance exceeds it).
 */
TL_FUNCTION bool TL_GetClosestPoint(
  const ON_Line& line_a,
  const ON_Line& line_b,
  double* t_a,
  double* t_b,
  double maximum_distance,
  const ON_Interval* domain_a,
  const ON_Interval* domain_b
);
