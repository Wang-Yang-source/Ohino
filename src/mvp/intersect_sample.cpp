#include "opentl/mvp/rhino_mvp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace opentl::mvp {
namespace {

/**
 * Parameters of the point on `line` closest to `test_point`, clamped to [0, 1].
 */
double ClampedLineParameter(const ON_Line& line, const ON_3dPoint& test_point)
{
  double s = 0.0;
  if (!line.ClosestPointTo(test_point, &s))
    return 0.0;
  if (!ON_IsValid(s))
    return 0.0;
  return std::clamp(s, 0.0, 1.0);
}

/**
 * 5-point Gauss-Legendre quadrature of the arc-length integrand |C'(t)| on [a, b].
 */
double QuadratureArcLength(const ON_Curve& curve, double a, double b)
{
  if (!(b > a))
    return 0.0;

  static const double gx[5] = {
    -0.9061798459386640, -0.5384693101056831, 0.0,
     0.5384693101056831,  0.9061798459386640
  };
  static const double gw[5] = {
    0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
    0.4786286704993665, 0.2369268850561891
  };

  const double half = 0.5 * (b - a);
  const double mid = 0.5 * (a + b);
  double sum = 0.0;
  for (int i = 0; i < 5; ++i)
  {
    ON_3dPoint pt = ON_3dPoint::Origin;
    ON_3dVector d1 = ON_3dVector::ZeroVector;
    if (!curve.Ev1Der(mid + half * gx[i], pt, d1))
      return 0.0;
    sum += gw[i] * d1.Length();
  }
  return half * sum;
}

/**
 * Cumulative arc-length table used for equal-arc-length sampling
 * (mirrors the behaviour of tl.dll!TL_ArcLengthPointsOnNurb).
 *
 * The table stores cumulative Gauss-Legendre quadrature values of |C'(t)| so the
 * reported arc lengths are not biased low by chord under-estimation.
 */
struct ArcLengthTable
{
  const ON_Curve* curve = nullptr;
  std::vector<double> t;
  std::vector<double> s;

  double Total() const { return s.empty() ? 0.0 : s.back(); }

  /**
   * Cumulative length at an arbitrary parameter.
   */
  double LengthAt(double param) const
  {
    if (t.size() < 2 || nullptr == curve)
      return 0.0;
    if (param <= t.front())
      return 0.0;
    if (param >= t.back())
      return s.back();

    const auto it = std::lower_bound(t.begin(), t.end(), param);
    const size_t hi = static_cast<size_t>(it - t.begin());
    if (hi == 0)
      return 0.0;
    const size_t lo = hi - 1;
    return s[lo] + QuadratureArcLength(*curve, t[lo], param);
  }

  /**
   * Inverse of LengthAt: parameter at which the cumulative length equals `target`.
   * Seeded from the chord table and polished with Newton iterations using the true speed.
   */
  double ParameterAtLength(const ON_Curve& crv, double target) const
  {
    if (t.size() < 2)
      return crv.Domain().Min();

    if (target <= 0.0)
      return t.front();
    if (target >= s.back())
      return t.back();

    const auto it = std::lower_bound(s.begin(), s.end(), target);
    size_t hi = static_cast<size_t>(it - s.begin());
    if (hi == 0)
      hi = 1;
    if (hi >= t.size())
      hi = t.size() - 1;
    const size_t lo = hi - 1;

    const double ds = s[hi] - s[lo];
    double param = t[lo];
    if (ds > 0.0)
      param = t[lo] + (target - s[lo]) / ds * (t[hi] - t[lo]);

    const double param_min = t.front();
    const double param_max = t.back();
    const double total = s.back();
    const double tol = 1e-13 * std::max(1.0, total);

    for (int iter = 0; iter < 16; ++iter)
    {
      ON_3dPoint pt = ON_3dPoint::Origin;
      ON_3dVector d1 = ON_3dVector::ZeroVector;
      if (!crv.Ev1Der(param, pt, d1))
        break;
      const double speed = d1.Length();
      const double err = LengthAt(param) - target;
      if (std::fabs(err) <= tol)
        break;
      if (!(speed > 1e-14))
        break;
      const double step = -err / speed;
      const double next = std::clamp(param + step, param_min, param_max);
      if (std::fabs(next - param) <= 1e-16 * std::max(1.0, std::fabs(param)))
      {
        param = next;
        break;
      }
      param = next;
    }

    return std::clamp(param, param_min, param_max);
  }
};

/**
 * Build a cumulative arc-length table by sampling `curve`.
 */
ArcLengthTable BuildArcLengthTable(const ON_Curve& curve, int density)
{
  ArcLengthTable table;
  table.curve = &curve;

  const ON_Interval domain = curve.Domain();
  const double t0 = domain.Min();
  const double t1 = domain.Max();

  const int span_count = std::max(1, curve.SpanCount());
  const int samples = std::max(density, span_count * 32);
  const int count = samples + 1;

  table.t.reserve(static_cast<size_t>(count));
  table.s.reserve(static_cast<size_t>(count));

  table.t.push_back(t0);
  table.s.push_back(0.0);

  double accum = 0.0;
  double prev_param = t0;
  for (int i = 1; i < count; ++i)
  {
    const double u = static_cast<double>(i) / static_cast<double>(count - 1);
    const double param = (1.0 - u) * t0 + u * t1;
    accum += QuadratureArcLength(curve, prev_param, param);
    table.t.push_back(param);
    table.s.push_back(accum);
    prev_param = param;
  }

  return table;
}

ON_3dVector SafeUnitTangent(const ON_Curve& curve, double param)
{
  ON_3dVector tangent = curve.TangentAt(param);
  if (!tangent.Unitize())
    return ON_3dVector::ZeroVector;
  return tangent;
}

} // anonymous namespace

// ============================================================================
// IntersectLineLine
// ============================================================================

TL_FUNCTION LineLineIntersection IntersectLineLine(
  const Line& line_a,
  const Line& line_b,
  double tolerance,
  bool finite_segments
)
{
  LineLineIntersection result;

  const ON_3dVector dir_a = line_a.Direction();
  const ON_3dVector dir_b = line_b.Direction();
  const double len_a = dir_a.Length();
  const double len_b = dir_b.Length();

  const bool degenerate_a = !(len_a > ON_ZERO_TOLERANCE);
  const bool degenerate_b = !(len_b > ON_ZERO_TOLERANCE);

  // Parallel test on unit tangents (cosine angle test, matching ON_IntersectLineLine docs).
  const double angle_tolerance_radians = 0.5 * ON_PI / 180.0;
  const double parallel_tolerance = std::cos(angle_tolerance_radians);
  if (!degenerate_a && !degenerate_b)
  {
    const double cosine = std::fabs(ON_DotProduct(dir_a / len_a, dir_b / len_b));
    result.is_parallel = (cosine >= parallel_tolerance);
  }

  double a = 0.0;
  double b = 0.0;
  const bool solved = ON_Intersect(line_a, line_b, &a, &b);

  if (!solved)
  {
    // Parallel (or degenerate) configuration: closest-point fallback.
    a = 0.0;
    b = ClampedLineParameter(line_b, line_a.PointAt(a));
    if (finite_segments)
    {
      a = ClampedLineParameter(line_a, line_b.PointAt(b));
      b = ClampedLineParameter(line_b, line_a.PointAt(a));
    }
    else
    {
      double s = 0.0;
      if (line_b.ClosestPointTo(line_a.PointAt(0.0), &s) && ON_IsValid(s))
        b = s;
      if (line_a.ClosestPointTo(line_b.PointAt(b), &s) && ON_IsValid(s))
        a = s;
    }
  }

  if (!ON_IsValid(a))
    a = 0.0;
  if (!ON_IsValid(b))
    b = 0.0;

  if (finite_segments)
  {
    a = std::clamp(a, 0.0, 1.0);
    b = std::clamp(b, 0.0, 1.0);
    // Recompute the nearest-point pair on the segment boundary.
    b = ClampedLineParameter(line_b, line_a.PointAt(a));
    a = ClampedLineParameter(line_a, line_b.PointAt(b));
  }

  result.param_a = a;
  result.param_b = b;
  result.point_a = line_a.PointAt(a);
  result.point_b = line_b.PointAt(b);
  result.distance = result.point_a.DistanceTo(result.point_b);

  if (tolerance > 0.0 && ON_IsValid(tolerance))
    result.intersects = (result.distance <= tolerance);
  else
    result.intersects = !result.is_parallel;

  return result;
}

// ============================================================================
// IntersectLinePlane
// ============================================================================

TL_FUNCTION LinePlaneIntersection IntersectLinePlane(
  const Line& line,
  const Plane& plane,
  bool finite_segment,
  double tolerance
)
{
  LinePlaneIntersection result;

  Plane local_plane = plane;
  if (!local_plane.plane_equation.IsSet())
    local_plane.UpdateEquation();

  const ON_3dVector normal = local_plane.Normal();
  const ON_3dVector direction = line.Direction();
  const double dir_len = direction.Length();
  const double normal_len = normal.Length();

  if (!(normal_len > 0.0) || !(dir_len > 0.0))
    return result;

  const double cosine = std::fabs(ON_DotProduct(direction / dir_len, normal / normal_len));
  result.is_parallel = (cosine <= ON_SQRT_EPSILON);

  double t = 0.0;
  if (!ON_Intersect(line, local_plane, &t) || !ON_IsValid(t))
  {
    // Degenerate / edge case: fall back to the analytic plane-distance solve.
    const double da = local_plane.plane_equation.ValueAt(line.from);
    const double db = local_plane.plane_equation.ValueAt(line.to);
    const double denom = da - db;
    if (std::fabs(denom) > 0.0 && ON_IsValid(denom))
      t = da / denom;
    else
      t = 0.5;
  }

  result.line_param = t;
  result.point = line.PointAt(t);

  double u = 0.0;
  double v = 0.0;
  if (local_plane.ClosestPointTo(result.point, &u, &v))
  {
    result.plane_u = u;
    result.plane_v = v;
  }

  if (result.is_parallel)
  {
    result.intersects = false;
    return result;
  }

  if (finite_segment)
  {
    const double domain_tolerance = (tolerance > 0.0 && ON_IsValid(tolerance)) ? tolerance : 0.0;
    result.intersects = (t >= -domain_tolerance && t <= 1.0 + domain_tolerance);
  }
  else
  {
    result.intersects = true;
  }

  return result;
}

// ============================================================================
// SampleCurveByCount
// ============================================================================

TL_FUNCTION std::vector<CurveSamplePoint> SampleCurveByCount(
  const ON_Curve& curve,
  int sample_count,
  bool by_arc_length
)
{
  std::vector<CurveSamplePoint> samples;
  if (sample_count < 2)
    return samples;

  const ON_Interval domain = curve.Domain();
  if (!domain.IsIncreasing())
    return samples;

  const double t0 = domain.Min();
  const double t1 = domain.Max();
  const double span = t1 - t0;

  const int n = sample_count;
  samples.reserve(static_cast<size_t>(n));

  // The cumulative chord table always provides the reported arc_length values.
  const ArcLengthTable table = BuildArcLengthTable(curve, std::max(256, n * 8));
  const double total_length = table.Total();

  for (int i = 0; i < n; ++i)
  {
    const double fraction = static_cast<double>(i) / static_cast<double>(n - 1);

    double param = 0.0;
    if (by_arc_length && total_length > 0.0)
      param = table.ParameterAtLength(curve, fraction * total_length);
    else
      param = t0 + fraction * span;

    param = std::clamp(param, t0, t1);

    CurveSamplePoint sample;
    sample.t = param;
    sample.normalized_t = (span > 0.0) ? (param - t0) / span : 0.0;
    sample.point = curve.PointAt(param);
    sample.tangent = SafeUnitTangent(curve, param);

    if (i == 0)
      sample.arc_length = 0.0;
    else if (i == n - 1)
      sample.arc_length = total_length;
    else
      sample.arc_length = table.LengthAt(param);

    // Keep the reported cumulative length monotonically non-decreasing.
    if (i > 0 && sample.arc_length < samples.back().arc_length)
      sample.arc_length = samples.back().arc_length;

    samples.push_back(sample);
  }

  return samples;
}

// ============================================================================
// CreateClampedNurbsCurve
// ============================================================================

TL_FUNCTION bool CreateClampedNurbsCurve(
  const std::vector<Point3d>& control_points,
  int degree,
  NurbsCurve& out_curve
)
{
  if (degree < 1)
    return false;

  const size_t point_count = control_points.size();
  if (point_count < static_cast<size_t>(degree + 1))
    return false;

  for (const Point3d& pt : control_points)
  {
    if (!pt.IsValid())
      return false;
  }

  const int count = static_cast<int>(point_count);
  if (!out_curve.Create(3, false, degree + 1, count))
    return false;

  if (!out_curve.MakeClampedUniformKnotVector(1.0))
    return false;

  for (int i = 0; i < count; ++i)
  {
    if (!out_curve.SetCV(i, control_points[static_cast<size_t>(i)]))
      return false;
  }

  return true;
}

} // namespace opentl::mvp
