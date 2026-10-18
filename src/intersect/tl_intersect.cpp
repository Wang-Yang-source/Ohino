#include "opentl/intersect/tl_intersect.h"
#include "opentl/closest_point/tl_closest_point.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <vector>

namespace {

/**
 * Small helper bundle for "distance from curve to infinite line" queries.
 *
 * For a point P and an infinite line L(s) = from + s*dir the squared distance is
 *   D(t) = || perp(C(t) - L.from) ||^2 ,   perp(v) = v - (v.u) u ,  u = unit(dir)
 * and its first derivative is
 *   D'(t) = 2 * perp(C(t) - L.from) . C'(t)
 * which is exactly the orthogonality condition used by Newton-Raphson.
 */
struct CurveLineDistanceEvaluator
{
  const ON_Curve& curve;
  const ON_Line& line;
  ON_3dVector u = ON_3dVector::ZeroVector; // unit line direction
  bool is_valid = false;

  CurveLineDistanceEvaluator(const ON_Curve& c, const ON_Line& l)
    : curve(c)
    , line(l)
  {
    ON_3dVector dir = l.Direction();
    const double len = dir.Length();
    if (len > 0.0 && ON_IsValid(len))
    {
      u = dir / len;
      is_valid = true;
    }
  }

  ON_3dVector Perp(const ON_3dPoint& pt) const
  {
    ON_3dVector v = pt - line.from;
    return v - (ON_DotProduct(v, u) * u);
  }

  double DistanceAt(double t) const
  {
    ON_3dPoint pt = ON_3dPoint::Origin;
    if (!curve.EvPoint(t, pt))
      return ON_DBL_MAX;
    return Perp(pt).Length();
  }

  bool NewtonStep(
    double t,
    double& f,
    double& fp,
    ON_3dPoint& pt
  ) const
  {
    ON_3dVector d1 = ON_3dVector::ZeroVector;
    ON_3dVector d2 = ON_3dVector::ZeroVector;
    if (!curve.Ev2Der(t, pt, d1, d2))
    {
      if (!curve.Ev1Der(t, pt, d1))
        return false;
      d2 = ON_3dVector::ZeroVector;
    }
    const ON_3dVector r = Perp(pt);
    f = ON_DotProduct(r, d1);
    const double d1_dot_u = ON_DotProduct(d1, u);
    fp = ON_DotProduct(d1, d1) - d1_dot_u * d1_dot_u + ON_DotProduct(r, d2);
    return true;
  }
};

/**
 * Golden-section minimization of the curve-to-line distance on [a, b].
 * Robust bracketing step used before Newton-Raphson polishing.
 */
double GoldenSectionMinimizeDistance(
  const CurveLineDistanceEvaluator& evaluator,
  double a,
  double b,
  double param_tol
)
{
  if (!(b > a))
    return a;

  const double phi = 0.6180339887498949;
  double c = b - phi * (b - a);
  double d = a + phi * (b - a);
  double fc = evaluator.DistanceAt(c);
  double fd = evaluator.DistanceAt(d);

  int guard = 0;
  while ((b - a) > param_tol && guard++ < 200)
  {
    if (fc < fd)
    {
      b = d;
      d = c;
      fd = fc;
      c = b - phi * (b - a);
      fc = evaluator.DistanceAt(c);
    }
    else
    {
      a = c;
      c = d;
      fc = fd;
      d = a + phi * (b - a);
      fd = evaluator.DistanceAt(d);
    }
  }

  return (fc < fd) ? c : d;
}

/**
 * Damped Newton-Raphson polish for the local minimum of the curve-to-line distance.
 */
void RefineCurveLineMinimum(
  const CurveLineDistanceEvaluator& evaluator,
  double a,
  double b,
  double t_seed,
  double& best_t,
  double& best_dist
)
{
  double t = std::clamp(t_seed, a, b);
  double dist = evaluator.DistanceAt(t);
  if (dist < best_dist)
  {
    best_dist = dist;
    best_t = t;
  }

  const double span = b - a;
  if (span <= 0.0)
    return;

  const double p_tol = std::max(1e-14, span * 1e-13);
  for (int iter = 0; iter < 32; ++iter)
  {
    double f = 0.0;
    double fp = 0.0;
    ON_3dPoint pt = ON_3dPoint::Origin;
    if (!evaluator.NewtonStep(t, f, fp, pt))
      break;

    const double curr = evaluator.Perp(pt).Length();
    if (curr < best_dist)
    {
      best_dist = curr;
      best_t = t;
    }

    // Non-convex region: fall back to Gauss-Newton denominator.
    ON_3dVector d1 = ON_3dVector::ZeroVector;
    ON_3dPoint pt1 = ON_3dPoint::Origin;
    double gauss_den = (fp > 0.0 && ON_IsValid(fp)) ? fp : 0.0;
    if (gauss_den <= 0.0 && evaluator.curve.Ev1Der(t, pt1, d1))
    {
      const double d1u = ON_DotProduct(d1, evaluator.u);
      gauss_den = std::max(0.0, ON_DotProduct(d1, d1) - d1u * d1u);
    }
    if (gauss_den <= 1e-30)
      break;

    if (std::fabs(f) <= 1e-15 * std::sqrt(gauss_den) * std::max(1.0, curr))
      break;

    double step = -f / gauss_den;
    step = std::clamp(step, -0.5 * span, 0.5 * span);
    if (std::fabs(step) <= p_tol)
      break;

    double alpha = 1.0;
    bool improved = false;
    for (int ls = 0; ls < 12; ++ls)
    {
      const double t_cand = std::clamp(t + alpha * step, a, b);
      const double d_cand = evaluator.DistanceAt(t_cand);
      if (d_cand < curr)
      {
        t = t_cand;
        if (d_cand < best_dist)
        {
          best_dist = d_cand;
          best_t = t_cand;
        }
        improved = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!improved)
      break;
  }
}

/**
 * Build a strictly increasing list of evaluation parameters covering the curve domain,
 * densified inside every smooth (C-infinity) span.
 */
std::vector<double> BuildSampleParameters(const ON_Curve& curve, int min_per_span)
{
  const ON_Interval domain = curve.Domain();
  const double t_min = domain.Min();
  const double t_max = domain.Max();

  const int span_count = std::max(1, curve.SpanCount());
  std::vector<double> spans(static_cast<size_t>(span_count + 1), 0.0);
  if (!curve.GetSpanVector(spans.data()))
  {
    spans.resize(2);
    spans[0] = t_min;
    spans[1] = t_max;
  }
  spans.front() = t_min;
  spans.back() = t_max;

  std::vector<double> ts;
  ts.reserve(static_cast<size_t>(span_count + 1) * static_cast<size_t>(min_per_span) + 2);

  for (size_t i = 0; i + 1 < spans.size(); ++i)
  {
    const double a = std::max(spans[i], t_min);
    const double b = std::min(spans[i + 1], t_max);
    if (!(b > a))
      continue;
    for (int k = 0; k < min_per_span; ++k)
    {
      const double u = static_cast<double>(k) / static_cast<double>(min_per_span);
      ts.push_back((1.0 - u) * a + u * b);
    }
  }
  ts.push_back(t_max);

  std::sort(ts.begin(), ts.end());
  ts.erase(std::unique(ts.begin(), ts.end()), ts.end());
  return ts;
}

struct SortedCandidate
{
  double t = 0.0;
  double dist = 0.0;
  bool overlap = false;
};

/**
 * Merge candidates that describe the same intersection locus
 * (close in parameter space *and* close in 3D space, so genuine
 * self-intersections at distinct parameters are preserved).
 */
std::vector<SortedCandidate> MergeCandidates(
  const std::vector<SortedCandidate>& raw,
  const ON_Curve& curve,
  double point_tol,
  double param_tol
)
{
  std::vector<SortedCandidate> sorted = raw;
  std::sort(sorted.begin(), sorted.end(),
    [](const SortedCandidate& a, const SortedCandidate& b) { return a.t < b.t; });

  std::vector<SortedCandidate> merged;
  merged.reserve(sorted.size());
  for (const SortedCandidate& cand : sorted)
  {
    bool duplicate = false;
    for (SortedCandidate& kept : merged)
    {
      if (std::fabs(kept.t - cand.t) > param_tol)
        continue;
      const ON_3dPoint p0 = curve.PointAt(kept.t);
      const ON_3dPoint p1 = curve.PointAt(cand.t);
      if (p0.DistanceTo(p1) <= point_tol)
      {
        // Keep the closer / overlapping representative
        if (cand.dist < kept.dist)
        {
          kept.dist = cand.dist;
          kept.t = cand.t;
        }
        kept.overlap = kept.overlap || cand.overlap;
        duplicate = true;
        break;
      }
    }
    if (!duplicate)
      merged.push_back(cand);
  }
  return merged;
}

} // anonymous namespace

TL_FUNCTION int TL_IntersectCurveLine(
  const ON_Curve& curve,
  const ON_Line& line,
  double tolerance,
  ON_SimpleArray<TL_CurveCurveIntersectionEvent>& events
)
{
  events.SetCount(0);

  const ON_Interval domain = curve.Domain();
  if (!domain.IsIncreasing())
    return 0;

  const double tol = (tolerance > 0.0 && ON_IsValid(tolerance)) ? tolerance : ON_ZERO_TOLERANCE;
  const double t_min = domain.Min();
  const double t_max = domain.Max();
  const double domain_len = t_max - t_min;

  const CurveLineDistanceEvaluator evaluator(curve, line);
  if (!evaluator.is_valid)
  {
    // Degenerate (point) line: treat as closest-point query with distance filter.
    const ON_3dPoint target = line.from;
    const std::vector<double> ts = BuildSampleParameters(curve, std::max(24, (curve.Degree() + 1) * 8));
    double best_t = t_min;
    double best_d = ON_DBL_MAX;
    for (double t : ts)
    {
      const ON_3dPoint p = curve.PointAt(t);
      const double d = p.DistanceTo(target);
      if (d < best_d)
      {
        best_d = d;
        best_t = t;
      }
    }
    if (best_d > tol)
      return 0;
    TL_CurveCurveIntersectionEvent event;
    event.param_a = best_t;
    event.param_b = 0.0;
    event.point_a = curve.PointAt(best_t);
    event.point_b = target;
    event.is_overlap = false;
    events.Append(event);
    return 1;
  }

  // Cheap global rejection: curve bounding box versus line.
  ON_BoundingBox curve_bbox;
  if (curve.GetBoundingBox(curve_bbox, false) && curve_bbox.IsValid())
  {
    if (!ON_Intersect(curve_bbox, line, tol, nullptr))
      return 0;
  }

  const int per_span = std::max(24, (curve.Degree() + 1) * 10);
  const std::vector<double> ts = BuildSampleParameters(curve, per_span);
  if (ts.size() < 2)
    return 0;

  std::vector<double> ds(ts.size(), ON_DBL_MAX);
  for (size_t i = 0; i < ts.size(); ++i)
    ds[i] = evaluator.DistanceAt(ts[i]);

  const double param_tol = std::max(1e-13, domain_len * 1e-11);
  std::vector<SortedCandidate> raw;
  raw.reserve(16);

  const double bbox_diag = curve_bbox.IsValid() ? curve_bbox.Diagonal().Length() : 1.0;

  // ---- (a) collinear overlap runs ----
  // A run of consecutive samples that all lie within tolerance of the line, whose
  // mean distance is essentially zero, means the curve is (locally) contained in the
  // line.  The two run endpoints are reported as overlapping events so the caller can
  // build the overlapping interval.  A shallow tangency produces a short run with a
  // clearly non-zero mean distance and is therefore not mistaken for an overlap.
  std::vector<char> in_overlap(ts.size(), 0);
  {
    size_t i = 0;
    const double overlap_mean_tol = 0.25 * tol;
    const double overlap_arc_tol = std::max(50.0 * tol, 1e-4 * bbox_diag);
    while (i < ts.size())
    {
      if (ds[i] > tol)
      {
        ++i;
        continue;
      }
      size_t j = i;
      double sum = ds[i];
      while (j + 1 < ts.size() && ds[j + 1] <= tol)
      {
        ++j;
        sum += ds[j];
      }
      const size_t steps = j - i;
      const double mean = sum / static_cast<double>(steps + 1);
      const double arc = curve.PointAt(ts[j]).DistanceTo(curve.PointAt(ts[i]));
      if (steps >= 3 && mean <= overlap_mean_tol && arc > overlap_arc_tol)
      {
        for (size_t k = i; k <= j; ++k)
          in_overlap[k] = 1;
        raw.push_back({ ts[i], 0.0, true });
        raw.push_back({ ts[j], 0.0, true });
      }
      i = j + 1;
    }
  }

  // ---- (b) local minima of the sampled distance function ----
  for (size_t i = 0; i < ts.size(); ++i)
  {
    if (in_overlap[i])
      continue;

    const bool left_ok = (i == 0) || (ds[i] <= ds[i - 1]);
    const bool right_ok = (i + 1 == ts.size()) || (ds[i] <= ds[i + 1]);
    if (!left_ok || !right_ok)
      continue;

    const double a = (i == 0) ? t_min : ts[i - 1];
    const double b = (i + 1 == ts.size()) ? t_max : ts[i + 1];

    double best_t = ts[i];
    double best_d = ds[i];
    if (b > a)
    {
      const double t_gs = GoldenSectionMinimizeDistance(evaluator, a, b, std::max(1e-15, (b - a) * 1e-12));
      const double d_gs = evaluator.DistanceAt(t_gs);
      if (d_gs < best_d)
      {
        best_d = d_gs;
        best_t = t_gs;
      }
      RefineCurveLineMinimum(evaluator, a, b, best_t, best_t, best_d);
    }

    if (best_d <= tol)
      raw.push_back({ best_t, best_d, false });
  }

  if (raw.empty())
    return 0;

  const std::vector<SortedCandidate> merged = MergeCandidates(raw, curve, 2.0 * tol, param_tol);

  // Closed curves evaluate to the same 3D point at both ends of the domain.
  // Without this fold the seam crossing is reported twice (once at t_min and
  // once at t_max); fold the seam parameter and suppress the duplicate event.
  const bool closed = curve.IsClosed();
  const double seam_tol = std::max(param_tol, domain_len * 1e-9);
  std::vector<double> emitted_params;

  for (const SortedCandidate& cand : merged)
  {
    double t = std::clamp(cand.t, t_min, t_max);
    if (closed)
    {
      if (std::fabs(t - t_max) <= seam_tol)
        t = t_min;
      bool duplicate = false;
      for (size_t k = 0; k < emitted_params.size(); ++k)
      {
        if (std::fabs(emitted_params[k] - t) <= seam_tol)
        {
          duplicate = true;
          break;
        }
      }
      if (duplicate)
        continue;
      emitted_params.push_back(t);
    }
    const ON_3dPoint pa = curve.PointAt(t);
    double s = 0.0;
    if (!line.ClosestPointTo(pa, &s))
      s = 0.0;

    TL_CurveCurveIntersectionEvent event;
    event.param_a = t;
    event.param_b = s;
    event.point_a = pa;
    event.point_b = line.PointAt(s);
    event.is_overlap = cand.overlap;
    events.Append(event);
  }

  return events.Count();
}

namespace {

struct CurvePlaneSignedDistance
{
  const ON_Curve& curve;
  ON_PlaneEquation equation;
  ON_3dVector normal = ON_3dVector::ZeroVector; // possibly non-unit
  double normal_len = 1.0;

  CurvePlaneSignedDistance(const ON_Curve& c, const ON_Plane& plane)
    : curve(c)
  {
    ON_Plane local = plane;
    if (!local.plane_equation.IsSet())
    {
      local.UpdateEquation();
    }
    equation = local.plane_equation;
    normal = ON_3dVector(equation.x, equation.y, equation.z);
    normal_len = normal.Length();
    if (!(normal_len > 0.0) || !ON_IsValid(normal_len))
    {
      normal_len = 1.0;
      normal = local.zaxis;
      if (!normal.IsZero())
        normal_len = normal.Length();
    }
  }

  // Signed distance (scaled to world units when the equation is not unitized).
  double ValueAt(double t) const
  {
    ON_3dPoint pt = ON_3dPoint::Origin;
    if (!curve.EvPoint(t, pt))
      return ON_DBL_MAX;
    return equation.ValueAt(pt) / normal_len;
  }

  double ScaledValueAt(double t) const
  {
    ON_3dPoint pt = ON_3dPoint::Origin;
    if (!curve.EvPoint(t, pt))
      return ON_DBL_MAX;
    return equation.ValueAt(pt);
  }

  // g(t) and g'(t) = n . C'(t)
  bool DerivativeAt(double t, double& g, double& gp) const
  {
    ON_3dPoint pt = ON_3dPoint::Origin;
    ON_3dVector d1 = ON_3dVector::ZeroVector;
    if (!curve.Ev1Der(t, pt, d1))
      return false;
    g = equation.ValueAt(pt);
    gp = ON_DotProduct(normal, d1);
    return true;
  }

  // g(t), g'(t) = n . C'(t) and g''(t) = n . C''(t) (used to polish tangency extrema).
  bool SecondDerivativeAt(double t, double& g, double& gp, double& gpp) const
  {
    ON_3dPoint pt = ON_3dPoint::Origin;
    ON_3dVector d1 = ON_3dVector::ZeroVector;
    ON_3dVector d2 = ON_3dVector::ZeroVector;
    if (!curve.Ev2Der(t, pt, d1, d2))
    {
      gpp = 0.0;
      return DerivativeAt(t, g, gp);
    }
    g = equation.ValueAt(pt);
    gp = ON_DotProduct(normal, d1);
    gpp = ON_DotProduct(normal, d2);
    return true;
  }
};

/**
 * Safeguarded Newton / bisection root finder for a scalar function with an
 * analytic derivative on a bracket with a sign change.
 */
double SolveBracketedRoot(
  const CurvePlaneSignedDistance& functor,
  double a,
  double b,
  double param_tol
)
{
  double fa = functor.ScaledValueAt(a);
  double fb = functor.ScaledValueAt(b);
  if (!(fa == fa) || !(fb == fb))
    return 0.5 * (a + b);

  double t = 0.5 * (a + b);
  for (int iter = 0; iter < 80; ++iter)
  {
    double g = 0.0;
    double gp = 0.0;
    if (!functor.DerivativeAt(t, g, gp))
      break;

    if (std::fabs(g) <= 1e-15 * std::max(1.0, std::fabs(functor.normal_len)))
      break;

    // Maintain the bracket.
    if ((fa < 0.0) != (g < 0.0))
    {
      b = t;
      fb = g;
    }
    else
    {
      a = t;
      fa = g;
    }

    double t_next = 0.0;
    if (std::fabs(gp) > 1e-300)
    {
      t_next = t - g / gp;
      const double lo = std::min(a, b);
      const double hi = std::max(a, b);
      if (!(t_next > lo && t_next < hi))
        t_next = 0.5 * (a + b);
    }
    else
    {
      t_next = 0.5 * (a + b);
    }

    if (std::fabs(t_next - t) <= param_tol)
    {
      t = t_next;
      break;
    }
    t = t_next;
  }

  // Pick the bracketing endpoint with the smaller residual as a guard.
  const double rt = functor.ScaledValueAt(t);
  const double ra = functor.ScaledValueAt(a);
  const double rb = functor.ScaledValueAt(b);
  if (std::fabs(ra) < std::fabs(rt))
    return a;
  if (std::fabs(rb) < std::fabs(rt))
    return b;
  return t;
}

/**
 * Newton polish of an extremum of g(t), i.e. a root of g'(t) = n.C'(t) = 0.
 * Used to place tangency contacts accurately (golden-section alone only brackets them).
 */
double PolishPlaneExtremum(
  const CurvePlaneSignedDistance& functor,
  double lo,
  double hi,
  double t_seed
)
{
  if (!(hi > lo))
    return t_seed;

  double t = std::clamp(t_seed, lo, hi);
  for (int iter = 0; iter < 32; ++iter)
  {
    double g = 0.0;
    double gp = 0.0;
    double gpp = 0.0;
    if (!functor.SecondDerivativeAt(t, g, gp, gpp))
      break;
    if (std::fabs(gpp) <= 1e-300)
      break;

    const double step = -gp / gpp;
    const double t_next = std::clamp(t + step, lo, hi);
    if (std::fabs(t_next - t) <= 1e-15 * std::max(1.0, std::fabs(t)))
    {
      t = t_next;
      break;
    }
    t = t_next;
    if (std::fabs(gp) <= 1e-15 * std::fabs(gpp))
      break;
  }
  return t;
}

} // anonymous namespace

TL_FUNCTION int TL_IntersectCurvePlane(
  const ON_Curve& curve,
  const ON_Plane& plane,
  double tolerance,
  ON_SimpleArray<double>& curve_params,
  ON_SimpleArray<ON_3dPoint>& intersection_points
)
{
  curve_params.SetCount(0);
  intersection_points.SetCount(0);

  const ON_Interval domain = curve.Domain();
  if (!domain.IsIncreasing())
    return 0;

  const CurvePlaneSignedDistance functor(curve, plane);
  if (!functor.equation.IsValid() || functor.normal.IsZero())
    return 0;

  const double tol = (tolerance > 0.0 && ON_IsValid(tolerance)) ? tolerance : ON_ZERO_TOLERANCE;
  const double t_min = domain.Min();
  const double t_max = domain.Max();
  const double domain_len = t_max - t_min;
  const double param_tol = std::max(1e-13, domain_len * 1e-11);

  const int per_span = std::max(24, (curve.Degree() + 1) * 10);
  const std::vector<double> ts = BuildSampleParameters(curve, per_span);
  if (ts.size() < 2)
    return 0;

  std::vector<double> gs(ts.size(), 0.0);
  for (size_t i = 0; i < ts.size(); ++i)
    gs[i] = functor.ValueAt(ts[i]);

  std::vector<double> hits;
  hits.reserve(16);

  auto accept = [&](double t) {
    const double g = functor.ValueAt(t);
    if (std::fabs(g) > tol)
      return;
    if (t < t_min - param_tol || t > t_max + param_tol)
      return;
    hits.push_back(std::clamp(t, t_min, t_max));
  };

  // ---- sign changes across every sampled span ----
  for (size_t i = 0; i + 1 < ts.size(); ++i)
  {
    const double a = ts[i];
    const double b = ts[i + 1];
    const double ga = gs[i];
    const double gb = gs[i + 1];

    if (std::fabs(ga) <= tol)
    {
      accept(a);
      continue;
    }
    if ((ga < 0.0) != (gb < 0.0))
    {
      const double root = SolveBracketedRoot(functor, a, b, std::max(1e-15, (b - a) * 1e-13));
      accept(root);
    }
  }

  // ---- tangencies: local extrema of g(t) that touch (not cross) the plane ----
  for (size_t i = 0; i < ts.size(); ++i)
  {
    const bool left_ok = (i == 0) || (std::fabs(gs[i]) <= std::fabs(gs[i - 1]));
    const bool right_ok = (i + 1 == ts.size()) || (std::fabs(gs[i]) <= std::fabs(gs[i + 1]));
    if (!left_ok || !right_ok)
      continue;
    if ((i == 0 || i + 1 == ts.size()) && std::fabs(gs[i]) > tol)
      continue;

    const double a = (i == 0) ? t_min : ts[i - 1];
    const double b = (i + 1 == ts.size()) ? t_max : ts[i + 1];
    if (!(b > a))
      continue;

    // Cheap Lipschitz prune: an extremum cannot dip below tolerance when the sampled value
    // is larger than the largest observed slope times the bracket width.
    double max_slope = 0.0;
    if (i > 0 && ts[i] > ts[i - 1])
      max_slope = std::max(max_slope, std::fabs(gs[i] - gs[i - 1]) / (ts[i] - ts[i - 1]));
    if (i + 1 < ts.size() && ts[i + 1] > ts[i])
      max_slope = std::max(max_slope, std::fabs(gs[i + 1] - gs[i]) / (ts[i + 1] - ts[i]));
    if (std::fabs(gs[i]) - max_slope * (b - a) > tol)
      continue;

    // Golden-section minimization of |g| inside the bracketing interval.
    const double phi = 0.6180339887498949;
    double lo = a;
    double hi = b;
    double c = hi - phi * (hi - lo);
    double d = lo + phi * (hi - lo);
    double fc = std::fabs(functor.ValueAt(c));
    double fd = std::fabs(functor.ValueAt(d));
    int guard = 0;
    while ((hi - lo) > std::max(1e-15, (b - a) * 1e-12) && guard++ < 100)
    {
      if (fc < fd)
      {
        hi = d;
        d = c;
        fd = fc;
        c = hi - phi * (hi - lo);
        fc = std::fabs(functor.ValueAt(c));
      }
      else
      {
        lo = c;
        c = d;
        fc = fd;
        d = lo + phi * (hi - lo);
        fd = std::fabs(functor.ValueAt(d));
      }
    }
    const double t_ext = (fc < fd) ? c : d;
    const double t_polished = PolishPlaneExtremum(functor, lo, hi, t_ext);
    accept(t_polished);
  }

  // ---- domain endpoints ----
  accept(t_min);
  accept(t_max);

  if (hits.empty())
    return 0;

  std::sort(hits.begin(), hits.end());

  // Deduplicate by parameter and by resulting point.
  std::vector<double> unique;
  unique.reserve(hits.size());
  const double point_tol = std::max(10.0 * tol, 1e-12);
  for (double t : hits)
  {
    bool duplicate = false;
    for (double kept : unique)
    {
      if (std::fabs(kept - t) <= 4.0 * param_tol)
      {
        duplicate = true;
        break;
      }
      if (curve.PointAt(kept).DistanceTo(curve.PointAt(t)) <= point_tol)
      {
        duplicate = true;
        break;
      }
    }
    if (!duplicate)
      unique.push_back(t);
  }

  for (double t : unique)
  {
    curve_params.Append(t);
    intersection_points.Append(curve.PointAt(t));
  }

  return curve_params.Count();
}

// ============================================================================
// TL_3DX_LineXPlane  (tl.dll RVA 0x305d0, 378 bytes, zero imports)
// ============================================================================

TL_FUNCTION int TL_3DX_LineXPlane(
  const ON_3dPoint& line_from,
  const ON_3dVector& dir,
  const ON_3dPoint& plane_pt,
  const ON_3dVector& plane_normal,
  double* t_out
)
{
  const double den = ON_DotProduct(dir, plane_normal);
  const double num = ON_DotProduct(plane_pt - line_from, plane_normal);

  if (0.0 == den)
  {
    if (nullptr != t_out)
      *t_out = 0.0;
    // num == 0 -> the line lies in the plane (coplanar, infinitely many solutions);
    // otherwise it is parallel and not coplanar (no solution).
    return (0.0 == num) ? TL_LINEXPLANE_COPLANAR : TL_LINEXPLANE_PARALLEL;
  }

  if (0.0 == num)
  {
    if (nullptr != t_out)
      *t_out = 0.0;
    return TL_LINEXPLANE_AT_ZERO;
  }

  const double abs_den = std::fabs(den);
  const double abs_num = std::fabs(num);
  double t = 0.0;

  if (abs_den >= 1.0)
  {
    // 0x180030715 branch: DBL_MIN guards a numerator that is numerically zero.
    t = (abs_num <= abs_den * ON_DBL_MIN) ? 0.0 : (num / den);
  }
  else
  {
    // 0x1800306af branch: saturate instead of producing +-inf.
    if (abs_num > abs_den * ON_DBL_MAX)
    {
      const bool same_sign = (num > 0.0) == (den > 0.0);
      t = same_sign ? ON_DBL_MAX : -ON_DBL_MAX;
    }
    else
    {
      t = num / den;
    }
  }

  if (nullptr != t_out)
    *t_out = t;
  return TL_LINEXPLANE_UNIQUE;
}

// ============================================================================
// Line distance C-ABI entry points
// ============================================================================

TL_FUNCTION double TL_LineDistanceToPoint(const ON_Line* line, ON_3dPoint test_point, bool maximum)
{
  if (nullptr == line)
    return -1.0; // rhcommon_c preloads xmm0 with the -1.0 constant
  return maximum ? line->MaximumDistanceTo(test_point) : line->MinimumDistanceTo(test_point);
}

TL_FUNCTION double TL_LineDistanceToLine(const ON_Line* line_a, const ON_Line* line_b, bool maximum)
{
  if (nullptr == line_a || nullptr == line_b)
    return -1.0;
  return maximum ? line_a->MaximumDistanceTo(*line_b) : line_a->MinimumDistanceTo(*line_b);
}

// ============================================================================
// TL_IntersectCurveLineBySubdivision -- independent second strategy
// ============================================================================

namespace {

/**
 * Adaptive subdivision with Lipschitz pruning.
 *
 * The distance from a point on the curve to the (infinite) line is 1-Lipschitz in
 * space, so for any sub-interval [t_i, t_i+1]:
 *
 *     min_{t in [t_i, t_i+1]} D(t)  >=  min(D(t_i), D(t_i+1)) - |C(t_i) - C(t_i+1)|
 *
 * That lower bound is rigorous (no heuristic constant), which is what makes this an
 * independent check on the sampling + golden-section + Newton primary solver.  Every
 * sub-interval whose bound dips below the tolerance gets a Newton polish; the rest are
 * provably clear.
 */
void CollectHitsByLipschitzSubdivision(
  const CurveLineDistanceEvaluator& evaluator,
  const ON_Curve& curve,
  double tol,
  int samples_per_span,
  std::vector<double>& hits
)
{
  const ON_Interval domain = curve.Domain();
  const double t_min = domain.Min();
  const double t_max = domain.Max();
  const int spans = std::max(1, curve.SpanCount());
  const int samples = std::max(64, spans * samples_per_span);

  double prev_t = t_min;
  double prev_d = evaluator.DistanceAt(t_min);
  ON_3dPoint prev_pt = curve.PointAt(t_min);

  for (int i = 1; i <= samples; ++i)
  {
    const double t = (i == samples)
      ? t_max
      : t_min + (t_max - t_min) * (static_cast<double>(i) / static_cast<double>(samples));
    const double d = evaluator.DistanceAt(t);
    const ON_3dPoint pt = curve.PointAt(t);

    const double chord = prev_pt.DistanceTo(pt);
    const double lower_bound = std::min(prev_d, d) - chord;

    if (lower_bound <= tol)
    {
      double best_t = 0.5 * (prev_t + t);
      double best_d = evaluator.DistanceAt(best_t);
      RefineCurveLineMinimum(evaluator, prev_t, t, best_t, best_t, best_d);
      if (best_d <= tol)
        hits.push_back(std::clamp(best_t, prev_t, t));
    }

    prev_t = t;
    prev_d = d;
    prev_pt = pt;
  }
}

} // anonymous namespace

TL_FUNCTION int TL_IntersectCurveLineBySubdivision(
  const ON_Curve& curve,
  const ON_Line& line,
  double tolerance,
  ON_SimpleArray<TL_CurveCurveIntersectionEvent>& events
)
{
  events.SetCount(0);

  const ON_Interval domain = curve.Domain();
  if (!domain.IsIncreasing())
    return 0;

  const double tol = (tolerance > 0.0 && ON_IsValid(tolerance)) ? tolerance : ON_ZERO_TOLERANCE;
  const double t_min = domain.Min();
  const double t_max = domain.Max();

  const CurveLineDistanceEvaluator evaluator(curve, line);
  if (!evaluator.is_valid)
  {
    // Degenerate (point) line: fall back to the primary solver so both strategies
    // agree on the documented degenerate contract.
    return TL_IntersectCurveLine(curve, line, tolerance, events);
  }

  ON_BoundingBox curve_bbox;
  if (curve.GetBoundingBox(curve_bbox, false) && curve_bbox.IsValid())
  {
    if (!ON_Intersect(curve_bbox, line, tol, nullptr))
      return 0;
  }

  std::vector<double> hits;
  hits.reserve(8);
  CollectHitsByLipschitzSubdivision(evaluator, curve, tol, 64, hits);

  if (hits.empty())
    return 0;

  std::vector<SortedCandidate> raw;
  raw.reserve(hits.size());
  for (double t : hits)
    raw.push_back({ t, evaluator.DistanceAt(t), false });

  const double diag = curve_bbox.IsValid() ? curve_bbox.Diagonal().Length() : 1.0;
  const double param_tol = std::max(1e-13, domain.Length() * 1e-9);
  // A grazing contact keeps the curve within `tol` of the line over a whole parameter
  // window, so adjacent refinements report the two ends of that window.  Those ends are
  // far apart in parameter but nearly coincident in space, which is why this merge is
  // driven by 3D proximity rather than by the parameter gap.
  const double point_tol = std::max(2.0 * tol, 1e-6 * diag);

  std::vector<SortedCandidate> merged;
  merged.reserve(raw.size());
  for (const SortedCandidate& cand : raw)
  {
    bool absorbed = false;
    for (SortedCandidate& kept : merged)
    {
      if (std::fabs(kept.t - cand.t) <= param_tol ||
          curve.PointAt(kept.t).DistanceTo(curve.PointAt(cand.t)) <= point_tol)
      {
        if (cand.dist < kept.dist)
        {
          kept.dist = cand.dist;
          kept.t = cand.t;
        }
        absorbed = true;
        break;
      }
    }
    if (!absorbed)
      merged.push_back(cand);
  }

  const bool closed = curve.IsClosed();
  const double seam_tol = std::max(param_tol, domain.Length() * 1e-9);
  std::vector<double> emitted;

  for (const SortedCandidate& cand : merged)
  {
    double t = std::clamp(cand.t, t_min, t_max);
    if (closed)
    {
      if (std::fabs(t - t_max) <= seam_tol)
        t = t_min;
      bool duplicate = false;
      for (double kept : emitted)
      {
        if (std::fabs(kept - t) <= seam_tol)
        {
          duplicate = true;
          break;
        }
      }
      if (duplicate)
        continue;
      emitted.push_back(t);
    }

    const ON_3dPoint pa = curve.PointAt(t);
    double s = 0.0;
    if (!line.ClosestPointTo(pa, &s))
      s = 0.0;

    TL_CurveCurveIntersectionEvent event;
    event.param_a = t;
    event.param_b = s;
    event.point_a = pa;
    event.point_b = line.PointAt(s);
    event.is_overlap = false;
    events.Append(event);
  }

  return events.Count();
}

// ============================================================================
// Curve x Curve intersection -- primary solver and independent grid solver
// ============================================================================

namespace {

/** Evaluate a curve point plus first and second derivatives; false when outside the domain. */
bool EvalCurveDTK(const ON_Curve& curve, double t, ON_3dPoint& p, ON_3dVector& d1, ON_3dVector& d2)
{
  if (!ON_IsValid(t))
    return false;
  const ON_Interval dom = curve.Domain();
  if (!dom.IsIncreasing())
    return false;
  if (t < dom.Min() - 1e-12 * dom.Length() || t > dom.Max() + 1e-12 * dom.Length())
    return false;

  ON_3dPoint unused_point = ON_3dPoint::Origin;
  ON_3dVector unused_first = ON_3dVector::ZeroVector;
  if (!curve.EvPoint(t, p) || !curve.Ev1Der(t, unused_point, d1))
    return false;
  if (!curve.Ev2Der(t, unused_point, unused_first, d2))
    d2 = ON_3dVector::ZeroVector;
  return true;
}

double CurveCurveGap(const ON_Curve& curve_a, const ON_Curve& curve_b, double ta, double tb)
{
  ON_3dPoint pa = ON_3dPoint::Origin;
  ON_3dPoint pb = ON_3dPoint::Origin;
  if (!curve_a.EvPoint(ta, pa) || !curve_b.EvPoint(tb, pb))
    return ON_DBL_MAX;
  return pa.DistanceTo(pb);
}

/**
 * Local 2-parameter minimisation of |C_a(ta) - C_b(tb)|^2.
 *
 * Full Gauss-Newton with the second-order curvature correction is used while the 2x2
 * Hessian stays well conditioned; near a tangency (parallel tangents) it degenerates and
 * the iteration falls back to alternating 1D projection steps, which still converge
 * because each sub-problem is a strict 1D minimum.
 */
bool RefineCurveCurvePair(const ON_Curve& curve_a, const ON_Curve& curve_b,
                          const ON_Interval& span_a, const ON_Interval& span_b,
                          double& ta, double& tb, double& dist)
{
  const double lo_a = span_a.Min();
  const double hi_a = span_a.Max();
  const double lo_b = span_b.Min();
  const double hi_b = span_b.Max();
  if (!(hi_a > lo_a) || !(hi_b > lo_b))
    return false;

  ta = std::clamp(ta, lo_a, hi_a);
  tb = std::clamp(tb, lo_b, hi_b);

  double best_dist = ON_DBL_MAX;
  double best_ta = ta;
  double best_tb = tb;

  for (int iter = 0; iter < 64; ++iter)
  {
    ON_3dPoint pa, pb;
    ON_3dVector da, dda, db, ddb;
    if (!EvalCurveDTK(curve_a, ta, pa, da, dda) || !EvalCurveDTK(curve_b, tb, pb, db, ddb))
      break;

    const ON_3dVector r = pa - pb;
    const double d = r.Length();
    if (d < best_dist)
    {
      best_dist = d;
      best_ta = ta;
      best_tb = tb;
    }
    if (d <= 1e-15)
      break;

    const double g0 = ON_DotProduct(r, da);
    const double g1 = -ON_DotProduct(r, db);
    if (std::fabs(g0) < 1e-16 && std::fabs(g1) < 1e-16)
      break;

    const double aa = ON_DotProduct(da, da);
    const double bb = ON_DotProduct(db, db);
    const double ab = -ON_DotProduct(da, db);
    const double h00 = aa + std::max(0.0, ON_DotProduct(r, dda));
    const double h11 = bb + std::max(0.0, -ON_DotProduct(r, ddb));
    const double h01 = ab;
    const double det = h00 * h11 - h01 * h01;

    double step_a = 0.0;
    double step_b = 0.0;
    bool used_newton = false;

    if (det > 1e-18 * std::max(1.0, h00 * h11) && aa > 0.0 && bb > 0.0)
    {
      step_a = -(h11 * g0 - h01 * g1) / det;
      step_b = -(h00 * g1 - h01 * g0) / det;
      used_newton = true;
    }
    else
    {
      // Tangency: alternate 1D projection steps.
      if (aa > 0.0)
        step_a = -g0 / aa;
      if (bb > 0.0)
        step_b = -g1 / bb;
    }

    const double max_a = hi_a - lo_a;
    const double max_b = hi_b - lo_b;
    step_a = std::clamp(step_a, -0.5 * max_a, 0.5 * max_a);
    step_b = std::clamp(step_b, -0.5 * max_b, 0.5 * max_b);

    // Armijo backtracking on the gap.
    double alpha = 1.0;
    bool improved = false;
    double next_ta = ta;
    double next_tb = tb;
    for (int ls = 0; ls < 24; ++ls)
    {
      const double cand_a = std::clamp(ta + alpha * step_a, lo_a, hi_a);
      const double cand_b = std::clamp(tb + alpha * step_b, lo_b, hi_b);
      const double cand_d = CurveCurveGap(curve_a, curve_b, cand_a, cand_b);
      if (cand_d < d - 1e-18)
      {
        next_ta = cand_a;
        next_tb = cand_b;
        improved = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!improved)
      break;

    const double move = std::fabs(next_ta - ta) + std::fabs(next_tb - tb);
    ta = next_ta;
    tb = next_tb;
    if (move <= 1e-14 * (1.0 + std::fabs(ta) + std::fabs(tb)))
      break;
    if (used_newton && move <= 1e-12)
      break;
  }

  // Final alternating 1D polish: guarantees the reported pair really is a local minimum.
  for (int sweep = 0; sweep < 8; ++sweep)
  {
    double t = best_ta;
    if (TL_GetCurveClosestPoint(curve_a, curve_b.PointAt(best_tb), &t, 1e-13, &span_a))
      best_ta = std::clamp(t, lo_a, hi_a);
    t = best_tb;
    if (TL_GetCurveClosestPoint(curve_b, curve_a.PointAt(best_ta), &t, 1e-13, &span_b))
      best_tb = std::clamp(t, lo_b, hi_b);
  }

  const double polished = CurveCurveGap(curve_a, curve_b, best_ta, best_tb);
  if (polished < best_dist)
    best_dist = polished;
  ta = best_ta;
  tb = best_tb;
  dist = best_dist;
  return ON_IsValid(best_dist);
}

/** Merge events that describe the same 3D location (parameter gaps are unreliable). */
void MergeCurveCurveEvents(std::vector<TL_CurveCurveIntersectionEvent>& events,
                           double param_span, double point_tol)
{
  std::sort(events.begin(), events.end(),
            [](const TL_CurveCurveIntersectionEvent& a, const TL_CurveCurveIntersectionEvent& b) {
              return a.param_a < b.param_a;
            });

  std::vector<TL_CurveCurveIntersectionEvent> merged;
  merged.reserve(events.size());
  const double param_tol = std::max(1e-13, param_span * 1e-9);

  for (const auto& cand : events)
  {
    bool absorbed = false;
    for (auto& kept : merged)
    {
      if (std::fabs(kept.param_a - cand.param_a) <= param_tol ||
          kept.point_a.DistanceTo(cand.point_a) <= point_tol)
      {
        if (cand.point_a.DistanceTo(cand.point_b) < kept.point_a.DistanceTo(kept.point_b))
          kept = cand;
        absorbed = true;
        break;
      }
    }
    if (!absorbed)
      merged.push_back(cand);
  }

  events.swap(merged);
}

} // anonymous namespace

TL_FUNCTION int TL_IntersectCurves(
  const ON_Curve& curve_a,
  const ON_Curve& curve_b,
  double tolerance,
  ON_SimpleArray<TL_CurveCurveIntersectionEvent>& events
)
{
  events.SetCount(0);

  const ON_Interval dom_a = curve_a.Domain();
  const ON_Interval dom_b = curve_b.Domain();
  if (!dom_a.IsIncreasing() || !dom_b.IsIncreasing())
    return 0;

  const double tol = (tolerance > 0.0 && ON_IsValid(tolerance)) ? tolerance : ON_ZERO_TOLERANCE;

  ON_BoundingBox box_a;
  ON_BoundingBox box_b;
  if (curve_a.GetBoundingBox(box_a, false) && curve_b.GetBoundingBox(box_b, false))
  {
    if (box_a.IsValid() && box_b.IsValid() && box_a.MinimumDistanceTo(box_b) > tol)
      return 0;
  }

  // ---- initial subdivision ---------------------------------------------------
  const int grid_a = std::max(16, curve_a.SpanCount() * 8);
  const int grid_b = std::max(16, curve_b.SpanCount() * 8);
  const std::vector<double> ta_grid = BuildSampleParameters(curve_a, grid_a);
  const std::vector<double> tb_grid = BuildSampleParameters(curve_b, grid_b);
  if (ta_grid.size() < 2 || tb_grid.size() < 2)
    return 0;

  // Per-span bounding boxes, inflated by the sampled chord sag so a curved span cannot
  // slip past the box test.
  auto span_boxes = [](const ON_Curve& curve, const std::vector<double>& ts,
                       std::vector<ON_BoundingBox>& boxes) {
    boxes.clear();
    boxes.reserve(ts.size() - 1);
    for (size_t i = 0; i + 1 < ts.size(); ++i)
    {
      ON_BoundingBox box;
      ON_3dPoint p0 = ON_3dPoint::Origin;
      ON_3dPoint p1 = ON_3dPoint::Origin;
      if (!curve.EvPoint(ts[i], p0) || !curve.EvPoint(ts[i + 1], p1))
        return false;
      box.Set(p0);
      box.Set(p1, true);
      ON_3dPoint mid = ON_3dPoint::Origin;
      if (curve.EvPoint(0.5 * (ts[i] + ts[i + 1]), mid))
        box.Set(mid, true);
      box.m_min -= ON_3dVector(1e-12, 1e-12, 1e-12);
      box.m_max += ON_3dVector(1e-12, 1e-12, 1e-12);
      boxes.push_back(box);
    }
    return true;
  };

  std::vector<ON_BoundingBox> boxes_a;
  std::vector<ON_BoundingBox> boxes_b;
  if (!span_boxes(curve_a, ta_grid, boxes_a) || !span_boxes(curve_b, tb_grid, boxes_b))
    return 0;

  std::vector<TL_CurveCurveIntersectionEvent> raw;
  raw.reserve(16);

  auto try_pair = [&](size_t ia, size_t ib) {
    const ON_Interval sa(ta_grid[ia], ta_grid[ia + 1]);
    const ON_Interval sb(tb_grid[ib], tb_grid[ib + 1]);
    double ta = 0.5 * (sa.Min() + sa.Max());
    double tb = 0.5 * (sb.Min() + sb.Max());
    double dist = ON_DBL_MAX;
    if (!RefineCurveCurvePair(curve_a, curve_b, sa, sb, ta, tb, dist))
      return;
    if (!(dist <= tol))
      return;

    TL_CurveCurveIntersectionEvent ev;
    ev.param_a = ta;
    ev.param_b = tb;
    ev.point_a = curve_a.PointAt(ta);
    ev.point_b = curve_b.PointAt(tb);
    ev.is_overlap = false;
    raw.push_back(ev);
  };

  // ---- branch and bound over span pairs -------------------------------------
  struct Task
  {
    int depth = 0;
    size_t ia = 0;
    size_t ib = 0;
    ON_Interval sa;
    ON_Interval sb;
  };

  std::vector<Task> stack;
  stack.reserve(256);
  for (size_t ia = 0; ia + 1 < ta_grid.size(); ++ia)
  {
    for (size_t ib = 0; ib + 1 < tb_grid.size(); ++ib)
    {
      if (boxes_a[ia].MinimumDistanceTo(boxes_b[ib]) <= tol)
        stack.push_back({ 0, ia, ib, ON_Interval(ta_grid[ia], ta_grid[ia + 1]),
                          ON_Interval(tb_grid[ib], tb_grid[ib + 1]) });
    }
  }

  const int max_depth = 6;
  while (!stack.empty())
  {
    const Task task = stack.back();
    stack.pop_back();

    if (task.depth >= max_depth)
    {
      const size_t ia = std::min(task.ia, boxes_a.size() - 1);
      const size_t ib = std::min(task.ib, boxes_b.size() - 1);
      try_pair(ia, ib);
      continue;
    }

    // Split the wider normalised interval, keep both children when their boxes are close.
    const double width_a = (task.sa.Max() - task.sa.Min()) / std::max(1e-30, dom_a.Length());
    const double width_b = (task.sb.Max() - task.sb.Min()) / std::max(1e-30, dom_b.Length());
    const bool split_a = (width_a >= width_b);

    ON_Interval first_a = task.sa;
    ON_Interval second_a = task.sa;
    ON_Interval first_b = task.sb;
    ON_Interval second_b = task.sb;
    if (split_a)
    {
      const double mid = 0.5 * (task.sa.Min() + task.sa.Max());
      first_a = ON_Interval(task.sa.Min(), mid);
      second_a = ON_Interval(mid, task.sa.Max());
    }
    else
    {
      const double mid = 0.5 * (task.sb.Min() + task.sb.Max());
      first_b = ON_Interval(task.sb.Min(), mid);
      second_b = ON_Interval(mid, task.sb.Max());
    }

    auto child_box = [&](const ON_Curve& curve, const ON_Interval& span) {
      ON_BoundingBox box;
      ON_3dPoint p = ON_3dPoint::Origin;
      for (int k = 0; k <= 2; ++k)
      {
        const double t = span.ParameterAt(0.5 * k);
        if (curve.EvPoint(t, p))
          box.Set(p, box.IsValid());
      }
      return box;
    };

    const ON_BoundingBox other_a = split_a ? child_box(curve_a, first_a) : boxes_a[task.ia];
    const ON_BoundingBox other_b = split_a ? boxes_b[task.ib] : child_box(curve_b, first_b);

    if (split_a)
    {
      const ON_BoundingBox ba = child_box(curve_a, first_a);
      if (ba.MinimumDistanceTo(other_b) <= tol)
        stack.push_back({ task.depth + 1, task.ia, task.ib, first_a, task.sb });
      const ON_BoundingBox bb = child_box(curve_a, second_a);
      if (bb.MinimumDistanceTo(other_b) <= tol)
        stack.push_back({ task.depth + 1, task.ia, task.ib, second_a, task.sb });
    }
    else
    {
      const ON_BoundingBox ba = child_box(curve_b, first_b);
      if (other_a.MinimumDistanceTo(ba) <= tol)
        stack.push_back({ task.depth + 1, task.ia, task.ib, task.sa, first_b });
      const ON_BoundingBox bb = child_box(curve_b, second_b);
      if (other_a.MinimumDistanceTo(bb) <= tol)
        stack.push_back({ task.depth + 1, task.ia, task.ib, task.sa, second_b });
    }
  }

  if (raw.empty())
    return 0;

  const double diag = box_a.IsValid() ? box_a.Diagonal().Length() : 1.0;
  MergeCurveCurveEvents(raw, dom_a.Length(), std::max(2.0 * tol, 1e-6 * diag));

  // Closed curves report the seam twice; fold t_max onto t_min.
  const bool closed_a = curve_a.IsClosed();
  const double seam_tol = std::max(1e-13, dom_a.Length() * 1e-9);
  std::vector<TL_CurveCurveIntersectionEvent> emitted;
  emitted.reserve(raw.size());
  for (auto ev : raw)
  {
    if (closed_a && std::fabs(ev.param_a - dom_a.Max()) <= seam_tol)
    {
      ev.param_a = dom_a.Min();
      ev.point_a = curve_a.PointAt(dom_a.Min());
    }
    bool duplicate = false;
    for (const auto& kept : emitted)
    {
      if (kept.point_a.DistanceTo(ev.point_a) <= std::max(2.0 * tol, 1e-6 * diag))
      {
        duplicate = true;
        break;
      }
    }
    if (!duplicate)
      emitted.push_back(ev);
  }

  for (const auto& ev : emitted)
    events.Append(ev);
  return events.Count();
}

TL_FUNCTION int TL_IntersectCurvesByGrid(
  const ON_Curve& curve_a,
  const ON_Curve& curve_b,
  double tolerance,
  ON_SimpleArray<TL_CurveCurveIntersectionEvent>& events
)
{
  events.SetCount(0);

  const ON_Interval dom_a = curve_a.Domain();
  const ON_Interval dom_b = curve_b.Domain();
  if (!dom_a.IsIncreasing() || !dom_b.IsIncreasing())
    return 0;

  const double tol = (tolerance > 0.0 && ON_IsValid(tolerance)) ? tolerance : ON_ZERO_TOLERANCE;

  ON_BoundingBox box_a;
  ON_BoundingBox box_b;
  if (curve_a.GetBoundingBox(box_a, false) && curve_b.GetBoundingBox(box_b, false))
  {
    if (box_a.IsValid() && box_b.IsValid() && box_a.MinimumDistanceTo(box_b) > tol)
      return 0;
  }

  // Fixed grid, no recursion: this is deliberately a different algorithm.
  const int count_a = std::max(32, curve_a.SpanCount() * 12);
  const int count_b = std::max(32, curve_b.SpanCount() * 12);
  const std::vector<double> ts = BuildSampleParameters(curve_a, count_a);
  const std::vector<double> ss = BuildSampleParameters(curve_b, count_b);
  if (ts.size() < 2 || ss.size() < 2)
    return 0;

  std::vector<ON_3dPoint> pa(ts.size());
  std::vector<ON_3dPoint> pb(ss.size());
  for (size_t i = 0; i < ts.size(); ++i)
    if (!curve_a.EvPoint(ts[i], pa[i]))
      return 0;
  for (size_t j = 0; j < ss.size(); ++j)
    if (!curve_b.EvPoint(ss[j], pb[j]))
      return 0;

  // Chord lengths give the rigorous triangle-inequality slack for every cell.
  std::vector<double> cha(ts.size() - 1, 0.0);
  for (size_t i = 0; i + 1 < ts.size(); ++i)
    cha[i] = pa[i].DistanceTo(pa[i + 1]);
  std::vector<double> chb(ss.size() - 1, 0.0);
  for (size_t j = 0; j + 1 < ss.size(); ++j)
    chb[j] = pb[j].DistanceTo(pb[j + 1]);

  std::vector<TL_CurveCurveIntersectionEvent> raw;
  raw.reserve(16);

  for (size_t i = 0; i + 1 < ts.size(); ++i)
  {
    for (size_t j = 0; j + 1 < ss.size(); ++j)
    {
      const double corner_min = std::min(std::min(pa[i].DistanceTo(pb[j]), pa[i].DistanceTo(pb[j + 1])),
                                         std::min(pa[i + 1].DistanceTo(pb[j]),
                                                  pa[i + 1].DistanceTo(pb[j + 1])));
      if (corner_min - cha[i] - chb[j] > tol)
        continue;   // provably clear

      const ON_Interval sa(ts[i], ts[i + 1]);
      const ON_Interval sb(ss[j], ss[j + 1]);
      double ta = 0.5 * (sa.Min() + sa.Max());
      double tb = 0.5 * (sb.Min() + sb.Max());
      double dist = ON_DBL_MAX;
      if (!RefineCurveCurvePair(curve_a, curve_b, sa, sb, ta, tb, dist))
        continue;
      if (!(dist <= tol))
        continue;

      TL_CurveCurveIntersectionEvent ev;
      ev.param_a = ta;
      ev.param_b = tb;
      ev.point_a = curve_a.PointAt(ta);
      ev.point_b = curve_b.PointAt(tb);
      ev.is_overlap = false;
      raw.push_back(ev);
    }
  }

  if (raw.empty())
    return 0;

  const double diag = box_a.IsValid() ? box_a.Diagonal().Length() : 1.0;
  MergeCurveCurveEvents(raw, dom_a.Length(), std::max(2.0 * tol, 1e-6 * diag));

  for (const auto& ev : raw)
    events.Append(ev);
  return events.Count();
}

// ============================================================================
// Surface x Surface intersection -- seed discovery + predictor/corrector marching
// ============================================================================

namespace {

constexpr int kSSXMaxCorrectorSteps = 24;

/** One point of an SSX trace: the 3D point plus its parameters on both surfaces. */
struct SSXNode
{
  double ua = 0.0;
  double va = 0.0;
  double ub = 0.0;
  double vb = 0.0;
  ON_3dPoint point = ON_3dPoint::Origin;
};

bool EvalSurfaceFrame(const ON_Surface& surface, double u, double v, ON_3dPoint& point,
                      ON_3dVector& du, ON_3dVector& dv)
{
  if (!ON_IsValid(u) || !ON_IsValid(v))
    return false;
  if (!surface.EvPoint(u, v, point))
    return false;
  if (!surface.Ev1Der(u, v, point, du, dv))
    return false;
  return true;
}

/** Clamp to the surface domain, or wrap when that direction is periodic. */
double ClampOrWrap(const ON_Surface& surface, int dir, double t)
{
  const ON_Interval dom = surface.Domain(dir);
  if (!dom.IsIncreasing())
    return t;
  if (surface.IsClosed(dir))
  {
    const double length = dom.Length();
    double wrapped = std::fmod(t - dom.Min(), length);
    if (wrapped < 0.0)
      wrapped += length;
    return dom.Min() + wrapped;
  }
  return std::clamp(t, dom.Min(), dom.Max());
}

/**
 * Minimum-norm 4D Gauss-Newton corrector.
 *
 * Solves the under-determined 3x4 system  J dx = -r  with  dx = -J^T (J J^T + lam I)^-1 r.
 * The regularisation lam is scaled by the trace so that near-tangential configurations
 * (where J J^T becomes singular) degrade gracefully instead of blowing up.
 */
bool CorrectSSXNode(const ON_Surface& surface_a, const ON_Surface& surface_b, SSXNode& node,
                    double tol)
{
  const double target = std::max(tol, 1e-14);

  for (int iter = 0; iter < kSSXMaxCorrectorSteps; ++iter)
  {
    ON_3dPoint pa, pb;
    ON_3dVector au, av, bu, bv;
    if (!EvalSurfaceFrame(surface_a, node.ua, node.va, pa, au, av))
      return false;
    if (!EvalSurfaceFrame(surface_b, node.ub, node.vb, pb, bu, bv))
      return false;

    const ON_3dVector r = pa - pb;
    const double gap = r.Length();
    if (gap <= target)
    {
      node.point = 0.5 * (pa + pb);
      return true;
    }

    // J = [au, av, -bu, -bv]  (3x4)
    const ON_3dVector cols[4] = { au, av, -bu, -bv };

    // M = J J^T  (3x3)
    double m[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
    for (int i = 0; i < 4; ++i)
    {
      for (int a = 0; a < 3; ++a)
      {
        for (int b = 0; b < 3; ++b)
          m[a][b] += cols[i][a] * cols[i][b];
      }
    }

    const double trace = m[0][0] + m[1][1] + m[2][2];
    const double lam = 1e-12 * std::max(trace, 1e-30);
    m[0][0] += lam;
    m[1][1] += lam;
    m[2][2] += lam;

    // Solve M y = r  (Cramer, 3x3)
    const double det =
      m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
      m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
      m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (!ON_IsValid(det) || std::fabs(det) <= 1e-300)
      return false;

    double y[3] = { 0.0, 0.0, 0.0 };
    for (int c = 0; c < 3; ++c)
    {
      double mv[3][3];
      for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
          mv[a][b] = (b == c) ? r[a] : m[a][b];
      y[c] = (mv[0][0] * (mv[1][1] * mv[2][2] - mv[1][2] * mv[2][1]) -
              mv[0][1] * (mv[1][0] * mv[2][2] - mv[1][2] * mv[2][0]) +
              mv[0][2] * (mv[1][0] * mv[2][1] - mv[1][1] * mv[2][0])) / det;
    }

    double dx[4] = { 0.0, 0.0, 0.0, 0.0 };
    for (int i = 0; i < 4; ++i)
      dx[i] = -(cols[i].x * y[0] + cols[i].y * y[1] + cols[i].z * y[2]);

    // Armijo backtracking.
    double alpha = 1.0;
    bool improved = false;
    SSXNode candidate = node;
    for (int ls = 0; ls < 16; ++ls)
    {
      candidate.ua = ClampOrWrap(surface_a, 0, node.ua + alpha * dx[0]);
      candidate.va = ClampOrWrap(surface_a, 1, node.va + alpha * dx[1]);
      candidate.ub = ClampOrWrap(surface_b, 0, node.ub + alpha * dx[2]);
      candidate.vb = ClampOrWrap(surface_b, 1, node.vb + alpha * dx[3]);

      ON_3dPoint ca, cb;
      ON_3dVector du, dv;
      if (EvalSurfaceFrame(surface_a, candidate.ua, candidate.va, ca, du, dv) &&
          EvalSurfaceFrame(surface_b, candidate.ub, candidate.vb, cb, du, dv))
      {
        if ((ca - cb).Length() < gap)
        {
          improved = true;
          break;
        }
      }
      alpha *= 0.5;
    }
    if (!improved)
      return gap <= std::max(target, 1e-9);

    node = candidate;
  }

  ON_3dPoint pa, pb;
  ON_3dVector du, dv;
  if (EvalSurfaceFrame(surface_a, node.ua, node.va, pa, du, dv) &&
      EvalSurfaceFrame(surface_b, node.ub, node.vb, pb, du, dv))
  {
    node.point = 0.5 * (pa + pb);
    return (pa - pb).Length() <= std::max(target, 1e-9);
  }
  return false;
}

/**
 * Local 2D Gauss-Newton orthogonal projection of a 3D point onto `surface` starting from
 * `(u_seed, v_seed)`.  Unlike a global grid search, this stays in the caller's parameter
 * basin and converges to machine precision in a few steps.
 */
bool ProjectPointToSurfaceLocal(const ON_Surface& surface, const ON_3dPoint& target,
                                double u_seed, double v_seed, double& u_out, double& v_out,
                                ON_3dPoint* closest_pt = nullptr)
{
  double u = ClampOrWrap(surface, 0, u_seed);
  double v = ClampOrWrap(surface, 1, v_seed);

  for (int iter = 0; iter < 16; ++iter)
  {
    ON_3dPoint p;
    ON_3dVector su, sv;
    if (!EvalSurfaceFrame(surface, u, v, p, su, sv))
      break;

    const ON_3dVector r = p - target;
    const double g0 = ON_DotProduct(r, su);
    const double g1 = ON_DotProduct(r, sv);
    if (std::fabs(g0) <= 1e-15 && std::fabs(g1) <= 1e-15)
      break;

    const double h00 = ON_DotProduct(su, su);
    const double h01 = ON_DotProduct(su, sv);
    const double h11 = ON_DotProduct(sv, sv);
    const double det = h00 * h11 - h01 * h01;
    if (!(det > 1e-24 * std::max(1.0, h00 * h11)))
      break;

    const double du = -(h11 * g0 - h01 * g1) / det;
    const double dv = -(h00 * g1 - h01 * g0) / det;

    const double cur_d = r.Length();
    double alpha = 1.0;
    bool improved = false;
    for (int ls = 0; ls < 10; ++ls)
    {
      const double cu = ClampOrWrap(surface, 0, u + alpha * du);
      const double cv = ClampOrWrap(surface, 1, v + alpha * dv);
      ON_3dPoint cp;
      if (surface.EvPoint(cu, cv, cp) && cp.DistanceTo(target) < cur_d)
      {
        u = cu;
        v = cv;
        improved = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!improved)
      break;
  }

  // Polish across knot spans where the first derivative jumps.
  ON_3dPoint best_p = ON_3dPoint::Origin;
  surface.EvPoint(u, v, best_p);
  double best_d = best_p.DistanceTo(target);
  if (best_d > 1e-10)
  {
    double du_step = 1e-3 * surface.Domain(0).Length();
    double dv_step = 1e-3 * surface.Domain(1).Length();
    for (int r = 0; r < 10 && best_d > 1e-12; ++r)
    {
      for (int dir = 0; dir < 2; ++dir)
      {
        for (int sign : { -1, 1 })
        {
          const double cu = ClampOrWrap(surface, 0, u + (0 == dir ? sign * du_step : 0.0));
          const double cv = ClampOrWrap(surface, 1, v + (1 == dir ? sign * dv_step : 0.0));
          ON_3dPoint cp;
          if (surface.EvPoint(cu, cv, cp))
          {
            const double cd = cp.DistanceTo(target);
            if (cd < best_d)
            {
              best_d = cd;
              u = cu;
              v = cv;
              best_p = cp;
            }
          }
        }
      }
      du_step *= 0.5;
      dv_step *= 0.5;
    }
  }

  u_out = u;
  v_out = v;
  if (nullptr != closest_pt)
    *closest_pt = best_p;
  return true;
}

/**
 * Second-order corrector for tangential contact (where N_a x N_b == 0 makes the 3x4
 * Jacobian rank-deficient along the shared normal).
 *
 * Minimises g(u_a, v_a) = dist(S_a(u_a, v_a), S_b) using coordinate search + parabolic /
 * golden-section line search in the parameter plane of A while keeping (u_b, v_b)
 * orthogonally projected via `ProjectPointToSurfaceLocal`.
 */
bool CorrectSSXTangentNode(const ON_Surface& surface_a, const ON_Surface& surface_b,
                           SSXNode& node, double tol)
{
  const double target = std::max(tol, 1e-12);
  const ON_Interval da0 = surface_a.Domain(0);
  const ON_Interval da1 = surface_a.Domain(1);

  auto eval_gap = [&](double ua, double va, double& ub_io, double& vb_io, ON_3dPoint& mid_out) {
    ua = ClampOrWrap(surface_a, 0, ua);
    va = ClampOrWrap(surface_a, 1, va);
    ON_3dPoint pa = ON_3dPoint::Origin;
    if (!surface_a.EvPoint(ua, va, pa))
      return ON_DBL_MAX;
    ON_3dPoint pb = ON_3dPoint::Origin;
    ProjectPointToSurfaceLocal(surface_b, pa, ub_io, vb_io, ub_io, vb_io, &pb);
    mid_out = 0.5 * (pa + pb);
    return pa.DistanceTo(pb);
  };

  double ub = node.ub;
  double vb = node.vb;
  ON_3dPoint mid = node.point;
  double best_gap = eval_gap(node.ua, node.va, ub, vb, mid);
  if (best_gap <= target)
  {
    node.ub = ub;
    node.vb = vb;
    node.point = mid;
    return true;
  }

  double step_u = 0.04 * da0.Length();
  double step_v = 0.04 * da1.Length();
  const double phi = 0.6180339887498949;

  for (int round = 0; round < 18 && best_gap > target; ++round)
  {
    // 1D golden-section minimization along u_a, then along v_a.
    for (int dir = 0; dir < 2; ++dir)
    {
      const double center = (0 == dir) ? node.ua : node.va;
      const double half = (0 == dir) ? step_u : step_v;
      double lo = center - half;
      double hi = center + half;
      double c = hi - phi * (hi - lo);
      double d = lo + phi * (hi - lo);

      auto f_1d = [&](double t) {
        double u_try = (0 == dir) ? t : node.ua;
        double v_try = (0 == dir) ? node.va : t;
        double ub_tmp = ub, vb_tmp = vb;
        ON_3dPoint m_tmp;
        return eval_gap(u_try, v_try, ub_tmp, vb_tmp, m_tmp);
      };

      double fc = f_1d(c);
      double fd = f_1d(d);
      for (int k = 0; k < 16; ++k)
      {
        if (fc < fd)
        {
          hi = d; d = c; fd = fc;
          c = hi - phi * (hi - lo);
          fc = f_1d(c);
        }
        else
        {
          lo = c; c = d; fc = fd;
          d = lo + phi * (hi - lo);
          fd = f_1d(d);
        }
      }
      const double best_t = (fc < fd) ? c : d;
      const double cand_u = ClampOrWrap(surface_a, 0, (0 == dir) ? best_t : node.ua);
      const double cand_v = ClampOrWrap(surface_a, 1, (0 == dir) ? node.va : best_t);
      double ub_cand = ub, vb_cand = vb;
      ON_3dPoint mid_cand;
      const double g_cand = eval_gap(cand_u, cand_v, ub_cand, vb_cand, mid_cand);
      if (g_cand < best_gap)
      {
        best_gap = g_cand;
        node.ua = cand_u;
        node.va = cand_v;
        ub = ub_cand;
        vb = vb_cand;
        mid = mid_cand;
      }
    }
    step_u *= 0.5;
    step_v *= 0.5;
  }

  node.ub = ub;
  node.vb = vb;
  node.point = mid;
  return best_gap <= std::max(target, 1e-7);
}

/**
 * 3D intersection direction of two surfaces at a point (unit); returns false when the
 * surfaces are tangential (`|N_a x N_b| / (|N_a||N_b|) <= 1e-4`).
 */
bool SSXTangent(const ON_Surface& surface_a, const ON_Surface& surface_b, const SSXNode& node,
                ON_3dVector& tangent)
{
  ON_3dPoint p;
  ON_3dVector au, av, bu, bv;
  if (!EvalSurfaceFrame(surface_a, node.ua, node.va, p, au, av))
    return false;
  if (!EvalSurfaceFrame(surface_b, node.ub, node.vb, p, bu, bv))
    return false;

  ON_3dVector na = ON_CrossProduct(au, av);
  ON_3dVector nb = ON_CrossProduct(bu, bv);
  if (!na.Unitize() || !nb.Unitize())
    return false;

  tangent = ON_CrossProduct(na, nb);
  const double len = tangent.Length();
  if (!(len > 1e-4))
    return false;
  tangent = tangent / len;
  return true;
}

/**
 * Discover a 1D tangential contact direction (e.g. a cylinder resting on a plane along
 * a ruling line) when `SSXTangent` reports parallel normals.  Samples directions in the
 * shared tangent plane and returns the unit direction along which the gap stays zero.
 */
bool DiscoverTangentialCurveDirection(const ON_Surface& surface_a, const ON_Surface& surface_b,
                                      const SSXNode& seed, double probe_step, double tol,
                                      ON_3dVector& tangent_out)
{
  ON_3dPoint p;
  ON_3dVector au, av;
  if (!EvalSurfaceFrame(surface_a, seed.ua, seed.va, p, au, av))
    return false;
  ON_3dVector e1 = au;
  if (!e1.Unitize())
    return false;
  ON_3dVector n = ON_CrossProduct(au, av);
  if (!n.Unitize())
    return false;
  ON_3dVector e2 = ON_CrossProduct(n, e1);
  if (!e2.Unitize())
    return false;

  double best_gap = ON_DBL_MAX;
  ON_3dVector best_dir = ON_3dVector::ZeroVector;
  const int kAngles = 36;
  for (int i = 0; i < kAngles; ++i)
  {
    const double theta = (ON_PI * static_cast<double>(i)) / static_cast<double>(kAngles);
    const ON_3dVector dir = std::cos(theta) * e1 + std::sin(theta) * e2;
    const ON_3dPoint target = seed.point + probe_step * dir;
    double ua = seed.ua, va = seed.va, ub = seed.ub, vb = seed.vb;
    ON_3dPoint pa, pb;
    ProjectPointToSurfaceLocal(surface_a, target, ua, va, ua, va, &pa);
    ProjectPointToSurfaceLocal(surface_b, pa, ub, vb, ub, vb, &pb);
    const double gap = pa.DistanceTo(pb);
    if (gap < best_gap && pa.DistanceTo(seed.point) >= 0.25 * probe_step)
    {
      best_gap = gap;
      best_dir = dir;
    }
  }

  if (best_gap <= std::max(tol * 25.0, 1e-5) && !best_dir.IsZero())
  {
    tangent_out = best_dir;
    return true;
  }
  return false;
}

/**
 * Convert a 3D step into parameter steps on one surface by solving the first
 * fundamental form  [[E,F],[F,G]] [du,dv]^T = [dP.Su, dP.Sv]^T.
 */
bool ParamStepForDelta(const ON_Surface& surface, double u, double v, const ON_3dVector& delta,
                       double& du_out, double& dv_out)
{
  ON_3dPoint p;
  ON_3dVector su, sv;
  if (!EvalSurfaceFrame(surface, u, v, p, su, sv))
    return false;

  const double e = ON_DotProduct(su, su);
  const double f = ON_DotProduct(su, sv);
  const double g = ON_DotProduct(sv, sv);
  const double rhs_u = ON_DotProduct(delta, su);
  const double rhs_v = ON_DotProduct(delta, sv);

  const double det = e * g - f * f;
  if (!(std::fabs(det) > 1e-20))
    return false;

  du_out = (g * rhs_u - f * rhs_v) / det;
  dv_out = (e * rhs_v - f * rhs_u) / det;
  return ON_IsValid(du_out) && ON_IsValid(dv_out);
}

/** March from `start` in one direction until the domain ends or the loop closes. */
std::vector<SSXNode> MarchSSX(const ON_Surface& surface_a, const ON_Surface& surface_b,
                              const SSXNode& start, const ON_3dVector& direction, double step,
                              double tol, bool allow_close, bool& closed_out)
{
  closed_out = false;
  std::vector<SSXNode> trace;
  trace.push_back(start);
  const char* stop_reason = "max_steps";

  SSXNode current = start;
  const int max_steps = 4000;
  double current_step = step;
  // Follow the trace by continuity, not by a fixed world direction: a periodic surface
  // flips the sign of its partial derivative across the seam, which flips N_a x N_b and
  // would send a fixed-direction march back down the branch it just walked.
  ON_3dVector last_advance = direction;

  for (int i = 0; i < max_steps; ++i)
  {
    SSXNode next = current;
    ON_3dVector tangent;
    if (!SSXTangent(surface_a, surface_b, current, tangent))
    {
      stop_reason = "tangent";
      break;
    }

    ON_3dVector aligned = tangent;
    if (ON_DotProduct(aligned, last_advance) < 0.0)
      aligned = -aligned;
    const ON_3dVector delta = current_step * aligned;
    double dua = 0.0, dva = 0.0, dub = 0.0, dvb = 0.0;
    if (!ParamStepForDelta(surface_a, current.ua, current.va, delta, dua, dva))
    {
      stop_reason = "paramstep_a";
      break;
    }
    if (!ParamStepForDelta(surface_b, current.ub, current.vb, delta, dub, dvb))
    {
      stop_reason = "paramstep_b";
      break;
    }

    const double next_ua = current.ua + dua;
    const double next_va = current.va + dva;
    const double next_ub = current.ub + dub;
    const double next_vb = current.vb + dvb;

    // Leave the domain: the branch ends here.
    auto outside = [](const ON_Surface& s, int dir, double t) {
      const ON_Interval dom = s.Domain(dir);
      if (s.IsClosed(dir))
        return false;
      return t < dom.Min() || t > dom.Max();
    };
    if (outside(surface_a, 0, next_ua) || outside(surface_a, 1, next_va) ||
        outside(surface_b, 0, next_ub) || outside(surface_b, 1, next_vb))
    {
      stop_reason = "domain_exit";
      break;
    }

    next.ua = ClampOrWrap(surface_a, 0, next_ua);
    next.va = ClampOrWrap(surface_a, 1, next_va);
    next.ub = ClampOrWrap(surface_b, 0, next_ub);
    next.vb = ClampOrWrap(surface_b, 1, next_vb);

    if (!CorrectSSXNode(surface_a, surface_b, next, tol))
    {
      stop_reason = "corrector";
      break;
    }

    // Stop when the corrector stood still (tangential contact / fold).
    if (next.point.DistanceTo(current.point) <= 1e-3 * step)
    {
      stop_reason = "stall";
      break;
    }

    // Keep the arc-length spacing honest: if the corrector moved the point further or
    // less than requested, rescale the next prediction instead of letting the error
    // accumulate (accumulated drift is what made the loop miss its own start).
    const double advanced = next.point.DistanceTo(current.point);
    if (advanced > 1e-12)
    {
      current_step = std::clamp(current_step * (step / advanced), 0.5 * step, 1.5 * step);
      last_advance = (next.point - current.point) / advanced;
    }

    trace.push_back(next);

    if (allow_close && trace.size() > 24)
    {
      // Return-to-start closes the loop; touching an older interior sample means the
      // trace reached a part it already covered, which also ends the march.
      size_t touch = trace.size();
      for (size_t k = 0; k + 12 < trace.size(); ++k)
      {
        if (trace[k].point.DistanceTo(next.point) <= 2.0 * step)
        {
          touch = k;
          break;
        }
      }
      if (0 == touch)
      {
        closed_out = true;
        stop_reason = "closed";
        trace.pop_back();          // the closing sample duplicates trace[0]
        break;
      }
      if (touch < trace.size())
      {
        stop_reason = "self_touch";
        trace.resize(touch + 1);
        break;
      }
    }

    current = next;
  }

  if (std::getenv("OPENTL_SSX_DEBUG") != nullptr)
  {
    std::printf("[ssx] march steps=%zu stop=%s dist_to_start=%.6f step=%.6f closed=%d\n",
                trace.size() - 1, stop_reason,
                trace.back().point.DistanceTo(start.point), step, static_cast<int>(closed_out));
  }

  return trace;
}

/**
 * Locate the knot span containing `t` (Piegl & Tiller A2.1).  `knots` holds
 * `n_cv + degree + 1` values; the returned span is clamped to `[degree, n_cv - 1]`.
 */
bool FindKnotSpan(const std::vector<double>& knots, int n_cv, int degree, double t, int& span)
{
  const int last = n_cv - 1;
  if (last < degree)
    return false;
  if (t <= knots[static_cast<size_t>(degree)])
  {
    span = degree;
    return true;
  }
  if (t >= knots[static_cast<size_t>(last + 1)])
  {
    span = last;
    return true;
  }

  int low = degree;
  int high = last + 1;
  int mid = (low + high) / 2;
  while (t < knots[static_cast<size_t>(mid)] || t >= knots[static_cast<size_t>(mid + 1)])
  {
    if (t < knots[static_cast<size_t>(mid)])
      high = mid;
    else
      low = mid;
    mid = (low + high) / 2;
    if (high - low <= 1)
    {
      mid = low;
      break;
    }
  }
  span = std::clamp(mid, degree, last);
  return true;
}

/**
 * Non-zero B-spline basis values at `t` (Piegl & Tiller A2.2), written to
 * `basis[0..degree]` for control points `span - degree .. span`.
 */
void BasisFunctions(const double* knots, int degree, int span, double t, double* basis)
{
  double left[16] = { 0.0 };
  double right[16] = { 0.0 };
  basis[0] = 1.0;

  for (int j = 1; j <= degree; ++j)
  {
    left[j] = t - knots[span + 1 - j];
    right[j] = knots[span + j] - t;
    double saved = 0.0;
    for (int r = 0; r < j; ++r)
    {
      const double denominator = right[r + 1] + left[j - r];
      const double temp = (std::fabs(denominator) > 1e-300) ? basis[r] / denominator : 0.0;
      basis[r] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    basis[j] = saved;
  }
}

/** Dense Gaussian elimination with partial pivoting; destroys `a`. */
bool SolveDense(std::vector<double>& a, std::vector<double>& b, int n)
{
  for (int col = 0; col < n; ++col)
  {
    int pivot = col;
    double best = std::fabs(a[static_cast<size_t>(col) * n + col]);
    for (int row = col + 1; row < n; ++row)
    {
      const double v = std::fabs(a[static_cast<size_t>(row) * n + col]);
      if (v > best)
      {
        best = v;
        pivot = row;
      }
    }
    if (!(best > 1e-300))
      return false;

    if (pivot != col)
    {
      for (int k = 0; k < n; ++k)
        std::swap(a[static_cast<size_t>(pivot) * n + k], a[static_cast<size_t>(col) * n + k]);
      std::swap(b[static_cast<size_t>(pivot)], b[static_cast<size_t>(col)]);
    }

    const double diag = a[static_cast<size_t>(col) * n + col];
    for (int row = col + 1; row < n; ++row)
    {
      const double factor = a[static_cast<size_t>(row) * n + col] / diag;
      if (0.0 == factor)
        continue;
      for (int k = col; k < n; ++k)
        a[static_cast<size_t>(row) * n + k] -= factor * a[static_cast<size_t>(col) * n + k];
      b[static_cast<size_t>(row)] -= factor * b[static_cast<size_t>(col)];
    }
  }

  for (int row = n - 1; row >= 0; --row)
  {
    double sum = b[static_cast<size_t>(row)];
    for (int k = row + 1; k < n; ++k)
      sum -= a[static_cast<size_t>(row) * n + k] * b[static_cast<size_t>(k)];
    const double diag = a[static_cast<size_t>(row) * n + row];
    if (!(std::fabs(diag) > 1e-300))
      return false;
    b[static_cast<size_t>(row)] = sum / diag;
  }
  return true;
}

/**
 * Interpolate an ordered point list with a clamped cubic NURBS curve.
 *
 * Chord-length parameters + the Piegl & Tiller (eq. 9.8) knot averaging rule, then a
 * dense solve of the (n x n) collocation system.  Degree drops for short traces.
 */
ON_NurbsCurve* FitTraceToNurbs(const std::vector<ON_3dPoint>& points)
{
  const int n = static_cast<int>(points.size());
  if (n < 2)
    return nullptr;

  const int degree = (n >= 4) ? 3 : (n - 1);
  if (degree < 1 || degree + 1 > 16)
    return nullptr;

  std::vector<double> chord(static_cast<size_t>(n), 0.0);
  for (int i = 1; i < n; ++i)
    chord[static_cast<size_t>(i)] =
      chord[static_cast<size_t>(i - 1)] +
      points[static_cast<size_t>(i)].DistanceTo(points[static_cast<size_t>(i - 1)]);
  const double total = chord[static_cast<size_t>(n - 1)];
  if (!(total > 1e-12))
    return nullptr;

  for (int i = 0; i < n; ++i)
    chord[static_cast<size_t>(i)] /= total;   // normalised [0,1]

  // Clamped knot vector: p+1 zeros, averaged interior knots, p+1 ones.
  std::vector<double> knots(static_cast<size_t>(n + degree + 1), 0.0);
  for (int i = 0; i < n + degree + 1; ++i)
    knots[static_cast<size_t>(i)] = (i > n) ? 1.0 : 0.0;
  for (int j = 1; j <= n - degree - 1; ++j)
  {
    double sum = 0.0;
    for (int k = j; k <= j + degree - 1; ++k)
      sum += chord[static_cast<size_t>(k)];
    knots[static_cast<size_t>(j + degree)] = sum / degree;
  }
  for (int i = 0; i <= degree; ++i)
  {
    knots[static_cast<size_t>(i)] = 0.0;
    knots[static_cast<size_t>(n + i)] = 1.0;
  }
  for (int i = 1; i < n + degree + 1; ++i)
    if (knots[static_cast<size_t>(i)] < knots[static_cast<size_t>(i - 1)])
      knots[static_cast<size_t>(i)] = knots[static_cast<size_t>(i - 1)];

  // ON_NurbsCurve stores KnotCount() == CVCount + Order - 2 knots: the duplicated
  // clamped end knots are implicit, so only indices 1 .. n+degree-1 are settable.
  ON_NurbsCurve shaped(3, false, degree + 1, n);
  if (shaped.KnotCount() != n + degree - 1)
    return nullptr;
  if (!shaped.ReserveKnotCapacity(n + degree - 1))
    return nullptr;
  for (int i = 1; i < n + degree; ++i)
  {
    if (!shaped.SetKnot(i - 1, knots[static_cast<size_t>(i)]))
      return nullptr;
  }

  std::vector<double> matrix(static_cast<size_t>(n) * n, 0.0);
  std::vector<double> rhs_x(static_cast<size_t>(n), 0.0);
  std::vector<double> rhs_y(static_cast<size_t>(n), 0.0);
  std::vector<double> rhs_z(static_cast<size_t>(n), 0.0);

  for (int i = 0; i < n; ++i)
  {
    int span = 0;
    if (!FindKnotSpan(knots, n, degree, chord[static_cast<size_t>(i)], span))
      return nullptr;
    double basis[16] = { 0.0 };
    BasisFunctions(knots.data(), degree, span, chord[static_cast<size_t>(i)], basis);
    for (int k = 0; k <= degree; ++k)
    {
      const int column = span - degree + k;
      if (column < 0 || column >= n)
        continue;
      matrix[static_cast<size_t>(i) * n + column] = basis[k];
    }
    rhs_x[static_cast<size_t>(i)] = points[static_cast<size_t>(i)].x;
    rhs_y[static_cast<size_t>(i)] = points[static_cast<size_t>(i)].y;
    rhs_z[static_cast<size_t>(i)] = points[static_cast<size_t>(i)].z;
  }

  std::vector<double> ax = matrix, ay = matrix, az = matrix;
  if (!SolveDense(ax, rhs_x, n) || !SolveDense(ay, rhs_y, n) || !SolveDense(az, rhs_z, n))
    return nullptr;

  for (int i = 0; i < n; ++i)
  {
    if (!shaped.SetCV(i, ON_3dPoint(rhs_x[static_cast<size_t>(i)], rhs_y[static_cast<size_t>(i)],
                                    rhs_z[static_cast<size_t>(i)])))
      return nullptr;
  }

  if (!shaped.IsValid())
    return nullptr;
  ON_NurbsCurve* result = new ON_NurbsCurve(shaped);
  result->SetDomain(0.0, total);   // arc-length-ish parameter, like TL_DeformableArc2
  return result;
}

/**
 * Interpolate a closed ring of `n` points (`points[0]` .. `points[n-1]`, without
 * duplicating `points[0]` at the end) as a true C^2 periodic cubic `ON_NurbsCurve`
 * (`IsPeriodic() == true`, `IsClosed() == true`).
 *
 * At the uniform knot sites of `CreatePeriodicUniformNurbs(3, 4, n, ...)`, the cubic
 * B-spline basis values are `(1/6, 4/6, 1/6)` on adjacent control points `(i-1, i, i+1) mod n`,
 * so solving that circulant system produces control points whose limit curve passes
 * through every sample and joins with C^2 continuity across the seam.
 */
ON_NurbsCurve* FitPeriodicTraceToNurbs(const std::vector<ON_3dPoint>& raw_ring_points)
{
  const int n = static_cast<int>(raw_ring_points.size());
  if (n < 4)
    return FitTraceToNurbs(raw_ring_points);

  // The closing step of a marched loop has whatever residual length remained when the
  // trace re-entered the start threshold, whereas a uniform periodic B-spline places its
  // knots at equal parameter intervals.  Build a clamped chord-length cubic through the
  // closed loop (with `points[0]` appended) and sample it at `n` equal parameter steps so
  // the periodic knots sit at uniform arc-length stations.
  std::vector<ON_3dPoint> closed_chain = raw_ring_points;
  if (closed_chain.back().DistanceTo(closed_chain.front()) > 1e-12)
    closed_chain.push_back(closed_chain.front());

  std::vector<ON_3dPoint> ring_points = raw_ring_points;
  double perimeter = 0.0;
  for (int i = 0; i < n; ++i)
    perimeter += raw_ring_points[static_cast<size_t>(i)].DistanceTo(
      raw_ring_points[static_cast<size_t>((i + 1) % n)]);
  if (!(perimeter > 1e-12))
    return nullptr;

  if (ON_NurbsCurve* clamped = FitTraceToNurbs(closed_chain))
  {
    const ON_Interval dom = clamped->Domain();
    for (int i = 0; i < n; ++i)
    {
      const double t = dom.ParameterAt(static_cast<double>(i) / static_cast<double>(n));
      ring_points[static_cast<size_t>(i)] = clamped->PointAt(t);
    }
    delete clamped;
  }

  std::vector<double> matrix(static_cast<size_t>(n) * n, 0.0);
  std::vector<double> rhs_x(static_cast<size_t>(n), 0.0);
  std::vector<double> rhs_y(static_cast<size_t>(n), 0.0);
  std::vector<double> rhs_z(static_cast<size_t>(n), 0.0);

  for (int i = 0; i < n; ++i)
  {
    const int prev = (i + n - 1) % n;
    const int next = (i + 1) % n;
    matrix[static_cast<size_t>(i) * n + prev] = 1.0 / 6.0;
    matrix[static_cast<size_t>(i) * n + i]    = 4.0 / 6.0;
    matrix[static_cast<size_t>(i) * n + next] = 1.0 / 6.0;
    rhs_x[static_cast<size_t>(i)] = ring_points[static_cast<size_t>(i)].x;
    rhs_y[static_cast<size_t>(i)] = ring_points[static_cast<size_t>(i)].y;
    rhs_z[static_cast<size_t>(i)] = ring_points[static_cast<size_t>(i)].z;
  }

  std::vector<double> ax = matrix, ay = matrix, az = matrix;
  if (!SolveDense(ax, rhs_x, n) || !SolveDense(ay, rhs_y, n) || !SolveDense(az, rhs_z, n))
    return FitTraceToNurbs(ring_points);

  // `CreatePeriodicUniformNurbs` evaluates `t = Domain().Min()` at `(CV[0] + 4*CV[1] + CV[2])/6`,
  // i.e. centred on `CV[1]`.  Rotate the solved CVs by -1 so `Domain().Min()` sits at `ring_points[0]`.
  std::vector<ON_3dPoint> cvs(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i)
  {
    const int src = (i + n - 1) % n;
    cvs[static_cast<size_t>(i)] = ON_3dPoint(rhs_x[static_cast<size_t>(src)],
                                             rhs_y[static_cast<size_t>(src)],
                                             rhs_z[static_cast<size_t>(src)]);
  }

  ON_NurbsCurve* curve = ON_NurbsCurve::New();
  if (!curve->CreatePeriodicUniformNurbs(3, 4, n, cvs.data(), 1.0) || !curve->IsValid())
  {
    delete curve;
    return FitTraceToNurbs(ring_points);
  }
  curve->SetDomain(0.0, perimeter);
  return curve;
}

/**
 * March along a 1D tangential contact locus (where both surfaces share a tangent plane).
 */
std::vector<SSXNode> MarchSSXTangent(const ON_Surface& surface_a, const ON_Surface& surface_b,
                                     const SSXNode& start, const ON_3dVector& direction,
                                     double step, double tol)
{
  std::vector<SSXNode> trace;
  trace.push_back(start);
  SSXNode current = start;
  ON_3dVector last_dir = direction;

  auto outside = [](const ON_Surface& s, int dir, double t) {
    const ON_Interval dom = s.Domain(dir);
    if (s.IsClosed(dir))
      return false;
    return t < dom.Min() || t > dom.Max();
  };

  for (int i = 0; i < 1000; ++i)
  {
    double dua = 0.0, dva = 0.0, dub = 0.0, dvb = 0.0;
    if (!ParamStepForDelta(surface_a, current.ua, current.va, step * last_dir, dua, dva))
      break;
    if (!ParamStepForDelta(surface_b, current.ub, current.vb, step * last_dir, dub, dvb))
      break;

    const double next_ua = current.ua + dua;
    const double next_va = current.va + dva;
    const double next_ub = current.ub + dub;
    const double next_vb = current.vb + dvb;
    if (outside(surface_a, 0, next_ua) || outside(surface_a, 1, next_va) ||
        outside(surface_b, 0, next_ub) || outside(surface_b, 1, next_vb))
      break;

    SSXNode next;
    next.ua = ClampOrWrap(surface_a, 0, next_ua);
    next.va = ClampOrWrap(surface_a, 1, next_va);
    next.ub = ClampOrWrap(surface_b, 0, next_ub);
    next.vb = ClampOrWrap(surface_b, 1, next_vb);

    if (!CorrectSSXTangentNode(surface_a, surface_b, next, tol))
      break;

    const double adv = next.point.DistanceTo(current.point);
    if (adv <= 0.1 * step)
      break;

    last_dir = (next.point - current.point) / adv;
    trace.push_back(next);
    current = next;
  }
  return trace;
}

} // anonymous namespace

TL_FUNCTION void TL_FreeSSXBranches(std::vector<TL_SurfaceSurfaceIntersectionBranch>& branches)
{
  for (auto& branch : branches)
  {
    delete branch.curve_3d;
    branch.curve_3d = nullptr;
  }
  branches.clear();
}

TL_FUNCTION int TL_SurfaceSurfaceIntersect(
  const ON_Surface& surface_a,
  const ON_Surface& surface_b,
  double tolerance,
  std::vector<TL_SurfaceSurfaceIntersectionBranch>& branches)
{
  TL_FreeSSXBranches(branches);

  const double tol = (tolerance > 0.0 && ON_IsValid(tolerance)) ? tolerance : 1e-6;

  ON_BoundingBox box_a;
  ON_BoundingBox box_b;
  if (!surface_a.GetBoundingBox(box_a) || !surface_b.GetBoundingBox(box_b))
    return 0;
  if (!box_a.IsValid() || !box_b.IsValid())
    return 0;
  if (box_a.MinimumDistanceTo(box_b) > std::max(tol, 1e-9))
    return 0;

  // An intersection curve is bounded by the *smaller* surface (its bbox intersection),
  // not the larger one: sizing `step` from `max(diag_a, diag_b)` made a huge reference
  // plane inflate the marching step 3x on a small tube.
  const double diag = std::min(box_a.Diagonal().Length(), box_b.Diagonal().Length());
  const double seed_step = std::max(tol * 10.0, 1e-9);

  // ---- seed discovery: sample both parameter grids and correct close pairs -----
  const int grid_u = 24;
  const int grid_v = 24;
  struct Sample
  {
    double u = 0.0;
    double v = 0.0;
    ON_3dPoint point = ON_3dPoint::Origin;
  };

  std::vector<Sample> samples_a;
  std::vector<Sample> samples_b;
  const ON_Interval da0 = surface_a.Domain(0);
  const ON_Interval da1 = surface_a.Domain(1);
  const ON_Interval db0 = surface_b.Domain(0);
  const ON_Interval db1 = surface_b.Domain(1);
  if (!da0.IsIncreasing() || !da1.IsIncreasing() || !db0.IsIncreasing() || !db1.IsIncreasing())
    return 0;

  for (int i = 0; i <= grid_u; ++i)
  {
    for (int j = 0; j <= grid_v; ++j)
    {
      Sample s;
      s.u = da0.ParameterAt(static_cast<double>(i) / grid_u);
      s.v = da1.ParameterAt(static_cast<double>(j) / grid_v);
      if (surface_a.EvPoint(s.u, s.v, s.point))
        samples_a.push_back(s);
    }
  }
  for (int i = 0; i <= grid_u; ++i)
  {
    for (int j = 0; j <= grid_v; ++j)
    {
      Sample s;
      s.u = db0.ParameterAt(static_cast<double>(i) / grid_u);
      s.v = db1.ParameterAt(static_cast<double>(j) / grid_v);
      if (surface_b.EvPoint(s.u, s.v, s.point))
        samples_b.push_back(s);
    }
  }
  if (samples_a.empty() || samples_b.empty())
    return 0;

  const double cell = std::max(1e-9, diag / static_cast<double>(grid_u));
  const double seed_radius = 2.5 * cell;

  std::vector<SSXNode> seeds;

  auto add_seed = [&](double ua, double va, double ub, double vb) {
    SSXNode node;
    node.ua = ua;
    node.va = va;
    node.ub = ub;
    node.vb = vb;
    if (!CorrectSSXNode(surface_a, surface_b, node, tol))
    {
      // Transverse corrector stalled: try the second-order tangential corrector so
      // grazing contacts (cylinder tangent to plane, kissing spheres) still seed.
      if (!CorrectSSXTangentNode(surface_a, surface_b, node, tol))
        return;
    }
    for (const SSXNode& kept : seeds)
    {
      if (kept.point.DistanceTo(node.point) <= seed_step)
        return;
    }
    seeds.push_back(node);
  };

  // Sampling both parameter grids independently only finds seeds when the intersection
  // happens to fall near a sample of both surfaces (a plane's grid is far too coarse for
  // that).  Instead project every sample of A onto B (and vice versa) and keep the pairs
  // whose gap is within one cell: the closest-point projection lands on the trace
  // whenever the sample is within a cell of it.
  for (const Sample& sa : samples_a)
  {
    double ub = 0.0;
    double vb = 0.0;
    if (!TL_GetSurfaceClosestPoint(surface_b, sa.point, &ub, &vb))
      continue;
    if (sa.point.DistanceTo(surface_b.PointAt(ub, vb)) > seed_radius)
      continue;
    add_seed(sa.u, sa.v, ub, vb);
  }
  for (const Sample& sb : samples_b)
  {
    double ua = 0.0;
    double va = 0.0;
    if (!TL_GetSurfaceClosestPoint(surface_a, sb.point, &ua, &va))
      continue;
    if (sb.point.DistanceTo(surface_a.PointAt(ua, va)) > seed_radius)
      continue;
    add_seed(ua, va, sb.u, sb.v);
  }

  if (std::getenv("OPENTL_SSX_DEBUG") != nullptr)
  {
    std::printf("[ssx] samples a=%zu b=%zu cell=%.4f seeds=%zu tol=%.3e\n", samples_a.size(),
                samples_b.size(), cell, seeds.size(), tol);
  }

  if (seeds.empty())
    return 0;

  // ---- march every seed that is not already covered by a trace ------------------
  const double step = std::max(diag / 160.0, 1e-7);
  std::vector<char> consumed(seeds.size(), 0);

  auto mark_consumed = [&](const std::vector<SSXNode>& trace) {
    for (size_t s = 0; s < seeds.size(); ++s)
    {
      if (consumed[s])
        continue;
      for (const SSXNode& node : trace)
      {
        if (node.point.DistanceTo(seeds[s].point) <= 1.75 * step)
        {
          consumed[s] = 1;
          break;
        }
      }
    }
  };

  for (size_t s = 0; s < seeds.size(); ++s)
  {
    if (consumed[s])
      continue;

    ON_3dVector tangent;
    bool is_tangent_branch = false;
    bool closed_forward = false;
    std::vector<SSXNode> trace;

    if (SSXTangent(surface_a, surface_b, seeds[s], tangent))
    {
      std::vector<SSXNode> forward = MarchSSX(surface_a, surface_b, seeds[s], tangent, step, tol,
                                              true, closed_forward);
      if (closed_forward)
      {
        trace = forward;
      }
      else
      {
        bool closed_backward = false;
        std::vector<SSXNode> backward = MarchSSX(surface_a, surface_b, seeds[s], -tangent, step, tol,
                                                 false, closed_backward);
        trace.reserve(backward.size() + forward.size());
        for (size_t i = backward.size(); i-- > 1;)
          trace.push_back(backward[i]);
        trace.insert(trace.end(), forward.begin(), forward.end());
      }
    }
    else
    {
      // Tangential contact: either a 1D grazing curve (cylinder tangent to plane) or an
      // isolated kissing point (sphere tangent to plane).
      is_tangent_branch = true;
      if (DiscoverTangentialCurveDirection(surface_a, surface_b, seeds[s], step, tol, tangent))
      {
        std::vector<SSXNode> forward = MarchSSXTangent(surface_a, surface_b, seeds[s], tangent, step, tol);
        std::vector<SSXNode> backward = MarchSSXTangent(surface_a, surface_b, seeds[s], -tangent, step, tol);
        trace.reserve(backward.size() + forward.size());
        for (size_t i = backward.size(); i-- > 1;)
          trace.push_back(backward[i]);
        trace.insert(trace.end(), forward.begin(), forward.end());
      }
      else
      {
        trace.push_back(seeds[s]);
      }
    }

    if (trace.empty() || (!is_tangent_branch && trace.size() < 2))
    {
      consumed[s] = 1;
      continue;
    }

    mark_consumed(trace);

    TL_SurfaceSurfaceIntersectionBranch branch;
    branch.is_closed = closed_forward;
    branch.is_tangent = is_tangent_branch;
    branch.points_3d.Reserve(static_cast<int>(trace.size()));
    branch.params_a.Reserve(static_cast<int>(trace.size()));
    branch.params_b.Reserve(static_cast<int>(trace.size()));
    for (const SSXNode& node : trace)
    {
      branch.points_3d.Append(node.point);
      branch.params_a.Append(ON_2dPoint(node.ua, node.va));
      branch.params_b.Append(ON_2dPoint(node.ub, node.vb));
    }

    std::vector<ON_3dPoint> fit_points(trace.size());
    for (size_t i = 0; i < trace.size(); ++i)
      fit_points[i] = trace[i].point;

    if (branch.is_closed && fit_points.size() >= 4)
      branch.curve_3d = FitPeriodicTraceToNurbs(fit_points);
    else if (fit_points.size() >= 2)
      branch.curve_3d = FitTraceToNurbs(fit_points);

    // Measure the achieved accuracy against both surfaces using local 2D Newton
    // projection seeded from the branch's own (u,v) trace, avoiding coarse-grid basin hops.
    double max_error = 0.0;
    if (nullptr != branch.curve_3d && !trace.empty())
    {
      const ON_Interval dom = branch.curve_3d->Domain();
      const int samples = 32;
      for (int k = 0; k <= samples; ++k)
      {
        const double frac = static_cast<double>(k) / static_cast<double>(samples);
        const ON_3dPoint p = branch.curve_3d->PointAt(dom.ParameterAt(frac));
        const size_t idx = std::min(trace.size() - 1,
                                    static_cast<size_t>(std::llround(frac * (trace.size() - 1))));
        double ua = trace[idx].ua, va = trace[idx].va;
        double ub = trace[idx].ub, vb = trace[idx].vb;
        ON_3dPoint pa, pb;
        if (ProjectPointToSurfaceLocal(surface_a, p, ua, va, ua, va, &pa))
          max_error = std::max(max_error, p.DistanceTo(pa));
        if (ProjectPointToSurfaceLocal(surface_b, p, ub, vb, ub, vb, &pb))
          max_error = std::max(max_error, p.DistanceTo(pb));
      }
    }
    else if (trace.size() == 1)
    {
      max_error = surface_a.PointAt(trace[0].ua, trace[0].va)
                    .DistanceTo(surface_b.PointAt(trace[0].ub, trace[0].vb));
    }
    branch.max_error = max_error;

    branches.push_back(branch);
  }

  return static_cast<int>(branches.size());
}
