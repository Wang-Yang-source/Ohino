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
