#pragma once
/**
 * @file tl_fitting_loft_sweep.h
 * @brief OpenTL Curve Deformable Approximation, Fitting, Knot Removal & Lofting (fitting_loft_sweep/)
 *
 * Binary Provenance:
 *   - tl.dll!?TL_DeformableArc2@@YAPEAVON_NurbsCurve@@AEBVON_Arc@@HHPEAV1@@Z      (RVA 0x37b220 .. 0x37b785)
 *   - tl.dll!?TL_DeformableCircle@@YAPEAVON_NurbsCurve@@AEBVON_Circle@@HHPEAV1@@Z (RVA 0x37b790 .. 0x37b7e5)
 *   - tl.dll!?RemoveKnots@TL_NurbsSurface@@QEAA_NHHHN@Z                            (RVA 0x384d90)
 */

#include "../tl_export.h"

/**
 * Reverse-engineered from `tl.dll!?TL_DeformableArc2@@YAPEAVON_NurbsCurve@@AEBVON_Arc@@HHPEAV1@@Z`
 * (`RVA 0x37b220 .. 0x37b785`).
 *
 * Constructs a non-rational (`is_rat = 0`, 3D) deformable `ON_NurbsCurve` of specified
 * `degree` (>= 1) and `cv_count` (>= degree + 1) that smoothly approximates `arc`.
 *
 * Disassembly verification (`0x18037b220`):
 * 1. Validates `degree >= 1`, `cv_count > degree`, and `arc.IsValid()`.
 * 2. Computes `order = degree + 1`; when `arc.IsCircle()` it tail-calls
 *    `TL_DeformableCircle` (0x18037b291) instead of running the fitting path.
 * 3. Allocates `ON_NurbsCurve::New()` (if `result == nullptr`) and calls
 *    `curve->Create(3, false, order, cv_count)` + `MakeClampedUniformKnotVector(1.0)`,
 *    leaving the clamped knot domain `[0, SpanCount()]` in place while sampling.
 * 4. Samples CVs with Greville abscissae: `angle = arc.AngleRadians() * greville[i] /
 *    arc.SpanCount()`, then `ON_Plane::PointAt(R*cos, R*sin)` (0x18037b320..0x18037b3b1).
 * 5. Solves the interpolating CVs and pins the first/last CV to `arc.StartPoint()` /
 *    `arc.EndPoint()`.
 * 6. Finally re-parameterises the finished curve to **arc length**:
 *    `SetDomain(0, arc.Length())` (0x18037b69f). The radian domain is never the
 *    result's domain, and `TL_DeformableCircle` uses `[0, 2*pi]` instead — the two
 *    parameter domains are NOT interchangeable.
 */
TL_FUNCTION ON_NurbsCurve* TL_DeformableArc2(
  const ON_Arc& arc,
  int degree,
  int cv_count,
  ON_NurbsCurve* result = nullptr
);

/**
 * Reverse-engineered from `tl.dll!?TL_DeformableCircle@@YAPEAVON_NurbsCurve@@AEBVON_Circle@@HHPEAV1@@Z`
 * (`RVA 0x37b790 .. 0x37b7e5`).
 *
 * Constructs a periodic non-rational 3D `ON_NurbsCurve` of `degree` and `cv_count`
 * (with `cv_count + degree` total CVs in opennurbs periodic representation) whose
 * limit curve approximates `circle` to high accuracy.
 */
TL_FUNCTION ON_NurbsCurve* TL_DeformableCircle(
  const ON_Circle& circle,
  int degree,
  int cv_count,
  ON_NurbsCurve* result = nullptr
);

/**
 * Create a ruled/linear or cubic lofted ON_NurbsSurface through an array of compatible ON_Curves.
 */
TL_FUNCTION ON_NurbsSurface* TL_LoftCurves(
  const ON_SimpleArray<const ON_Curve*>& sections,
  ON_NurbsSurface* result = nullptr
);

/**
 * One-rail sweep: transport `profile` along `rail` and skin the resulting sections.
 *
 * - The profile is interpreted in its own XY plane (`p.x` / `p.y`), which is exact for
 *   planar profiles such as circles and rectangles drawn in the world XY plane.
 * - Frames are built by **parallel transport** (the previous up vector is projected onto
 *   the plane perpendicular to the current tangent), so the profile does not spin or
 *   flip along the rail.  Degenerate tangents fall back to the last valid frame.
 * - `station_count` is clamped to [2, 256]; the sections are tessellated polyline curves
 *   and skinned by `TL_LoftCurves`, which interpolates (not linear-skins) in U.
 *
 * NOT implemented (still in README §9.1): two-rail sweep, rails with twist/scale laws,
 * G1/G2 corner blending.  This is an approximation, not the original product's sweep.
 */
TL_FUNCTION ON_NurbsSurface* TL_Sweep1Rail(
  const ON_Curve& rail,
  const ON_Curve& profile,
  int station_count = 16,
  ON_NurbsSurface* result = nullptr
);

/**
 * Two-rail sweep: transport and proportionally scale `profile` between `rail1` and `rail2`,
 * then skin the resulting station curves via `TL_LoftCurves`.
 */
TL_FUNCTION ON_NurbsSurface* TL_Sweep2Rails(
  const ON_Curve& rail1,
  const ON_Curve& rail2,
  const ON_Curve& profile,
  int station_count = 24,
  ON_NurbsSurface* result = nullptr
);

/**
 * Offset a curve in a reference plane by `distance` using normal sampling + cubic NURBS
 * Greville interpolation.
 */
TL_FUNCTION ON_NurbsCurve* TL_OffsetCurve(
  const ON_Curve& curve,
  const ON_Plane& plane,
  double distance,
  int sample_count = 48,
  ON_NurbsCurve* result = nullptr
);

/**
 * Create a rational NURBS surface of revolution (`ON_RevSurface::GetNurbForm`) by revolving
 * `profile` around `axis_line` through angle interval `[start_angle_radians, end_angle_radians]`.
 */
TL_FUNCTION ON_NurbsSurface* TL_RevolveCurve(
  const ON_Curve& profile,
  const ON_Line& axis_line,
  double start_angle_radians = 0.0,
  double end_angle_radians = 2.0 * ON_PI,
  ON_NurbsSurface* result = nullptr
);

/**
 * Construct a Coons / bilinearly blended Gordon-Coons NURBS patch from 2, 3, or 4 boundary
 * curves (`EdgeSurface`). When 4 curves are provided in loop order `(south, east, north, west)`
 * or unordered, endpoints are matched and blended via transfinite Coons interpolation:
 *   S(u,v) = (1-v)*C_south(u) + v*C_north(u) + (1-u)*C_west(v) + u*C_east(v) - BilinearCorners(u,v).
 */
TL_FUNCTION ON_NurbsSurface* TL_CreateEdgeSurface(
  const ON_SimpleArray<const ON_Curve*>& boundary_curves,
  ON_NurbsSurface* result = nullptr
);

/**
 * Compute exact tangent-continuous circular fillet arc `out_arc` of `radius` between two
 * coplanar curves `curve_a` and `curve_b`, plus the trimmed copy of each curve (`out_trimmed_a`,
 * `out_trimmed_b`). Returns true when a valid fillet arc of `radius` exists.
 */
TL_FUNCTION bool TL_FilletCurves(
  const ON_Curve& curve_a,
  const ON_Curve& curve_b,
  double radius,
  ON_Arc& out_arc,
  ON_NurbsCurve* out_trimmed_a = nullptr,
  ON_NurbsCurve* out_trimmed_b = nullptr
);
