#include "opentl/intersect/tl_intersect.h"

#include <algorithm>
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
