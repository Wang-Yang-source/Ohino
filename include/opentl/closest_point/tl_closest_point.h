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
