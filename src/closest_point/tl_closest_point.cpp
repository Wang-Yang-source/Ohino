#include "opentl/closest_point/tl_closest_point.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

/**
 * Perform damped Newton-Raphson refinement on a 1D curve parameter interval [t_min, t_max]
 * starting from `t_init`, minimizing D(t) = 0.5 * ||C(t) - P||^2:
 *   f(t)  = (C(t) - P) . C'(t)
 *   f'(t) = C'(t) . C'(t) + (C(t) - P) . C''(t)
 */
void RefineCurveClosestPointNewton(
  const ON_Curve& curve,
  const ON_3dPoint& test_point,
  double t_min,
  double t_max,
  double t_init,
  double& best_t,
  double& best_dist_sq
)
{
  double t = std::clamp(t_init, t_min, t_max);
  ON_3dPoint pt = ON_3dPoint::Origin;
  ON_3dVector d1 = ON_3dVector::ZeroVector;
  ON_3dVector d2 = ON_3dVector::ZeroVector;

  if (!curve.EvPoint(t, pt))
    return;

  double curr_dist_sq = (pt - test_point).LengthSquared();
  if (curr_dist_sq < best_dist_sq)
  {
    best_dist_sq = curr_dist_sq;
    best_t = t;
  }

  const double domain_len = t_max - t_min;
  if (domain_len <= 0.0)
    return;

  const double param_tol = std::max(1e-13, domain_len * 1e-12);
  const int max_iters = 24;

  for (int iter = 0; iter < max_iters; ++iter)
  {
    if (!curve.Ev2Der(t, pt, d1, d2))
    {
      if (!curve.Ev1Der(t, pt, d1))
        break;
      d2 = ON_3dVector::ZeroVector;
    }

    const ON_3dVector diff = pt - test_point;
    curr_dist_sq = diff.LengthSquared();
    if (curr_dist_sq < best_dist_sq)
    {
      best_dist_sq = curr_dist_sq;
      best_t = t;
    }

    const double f = ON_DotProduct(diff, d1);
    const double d1_len_sq = d1.LengthSquared();

    // Check orthogonality convergence
    if (d1_len_sq <= 1e-30)
      break;
    if (std::fabs(f) <= 1e-13 * std::sqrt(d1_len_sq) * std::max(1.0, std::sqrt(curr_dist_sq)))
      break;

    // If at boundary and gradient pushes further outside the interval, stop
    if (t <= t_min + param_tol && f > 0.0)
      break;
    if (t >= t_max - param_tol && f < 0.0)
      break;

    double fp = d1_len_sq + ON_DotProduct(diff, d2);
    // Guard against non-convex / negative curvature regions by falling back to Gauss-Newton denominator
    if (fp <= 1e-12 * d1_len_sq)
      fp = d1_len_sq;

    double step = -f / fp;
    // Clamp maximum single step to half the span length
    const double max_step = 0.5 * domain_len;
    step = std::clamp(step, -max_step, max_step);

    if (std::fabs(step) <= param_tol)
    {
      double t_final = std::clamp(t + step, t_min, t_max);
      ON_3dPoint pt_final;
      if (curve.EvPoint(t_final, pt_final))
      {
        double d_sq = (pt_final - test_point).LengthSquared();
        if (d_sq < best_dist_sq)
        {
          best_dist_sq = d_sq;
          best_t = t_final;
        }
      }
      break;
    }

    // Damped backtracking line search ensuring strictly decreasing distance squared
    double alpha = 1.0;
    bool improved = false;
    for (int ls = 0; ls < 10; ++ls)
    {
      double t_cand = std::clamp(t + alpha * step, t_min, t_max);
      ON_3dPoint pt_cand;
      if (curve.EvPoint(t_cand, pt_cand))
      {
        double cand_dist_sq = (pt_cand - test_point).LengthSquared();
        if (cand_dist_sq < curr_dist_sq)
        {
          t = t_cand;
          curr_dist_sq = cand_dist_sq;
          if (cand_dist_sq < best_dist_sq)
          {
            best_dist_sq = cand_dist_sq;
            best_t = t_cand;
          }
          improved = true;
          break;
        }
      }
      alpha *= 0.5;
    }

    if (!improved)
      break;
  }
}

} // anonymous namespace

TL_FUNCTION bool TL_GetCurveClosestPoint(
  const ON_Curve& curve,
  const ON_3dPoint& test_point,
  double* t_out,
  double maximum_distance,
  const ON_Interval* sub_domain
)
{
  if (!test_point.IsValid())
    return false;

  const ON_Interval full_domain = curve.Domain();
  if (!full_domain.IsIncreasing())
    return false;

  ON_Interval active_domain = full_domain;
  if (nullptr != sub_domain)
  {
    if (!sub_domain->IsIncreasing())
      return false;
    if (!active_domain.Intersection(*sub_domain))
      return false;
  }

  const double dom_min = active_domain.Min();
  const double dom_max = active_domain.Max();

  double best_t = dom_min;
  double best_dist_sq = std::numeric_limits<double>::infinity();

  auto update_candidate = [&](double t_cand) {
    t_cand = std::clamp(t_cand, dom_min, dom_max);
    ON_3dPoint p = curve.PointAt(t_cand);
    if (p.IsValid())
    {
      double d2 = (p - test_point).LengthSquared();
      if (d2 < best_dist_sq)
      {
        best_dist_sq = d2;
        best_t = t_cand;
      }
    }
  };

  // Evaluate active domain endpoints
  update_candidate(dom_min);
  update_candidate(dom_max);

  // Fast exact path for ON_LineCurve
  if (const ON_LineCurve* line_crv = ON_LineCurve::Cast(&curve))
  {
    double line_s = 0.0;
    if (line_crv->m_line.ClosestPointTo(test_point, &line_s))
    {
      line_s = std::clamp(line_s, 0.0, 1.0);
      double t_mapped = full_domain.ParameterAt(line_s);
      update_candidate(t_mapped);
    }
  }

  // Fast exact path for ON_ArcCurve
  if (const ON_ArcCurve* arc_crv = ON_ArcCurve::Cast(&curve))
  {
    double arc_rad = 0.0;
    if (arc_crv->m_arc.ClosestPointTo(test_point, &arc_rad))
    {
      ON_Interval arc_rad_dom = arc_crv->m_arc.DomainRadians();
      if (arc_rad_dom.IsIncreasing())
      {
        double norm_s = arc_rad_dom.NormalizedParameterAt(arc_rad);
        norm_s = std::clamp(norm_s, 0.0, 1.0);
        double t_mapped = full_domain.ParameterAt(norm_s);
        update_candidate(t_mapped);
      }
    }
  }

  // Fast exact path for ON_PolylineCurve or polyline-like curves
  ON_SimpleArray<ON_3dPoint> pline_pts;
  ON_SimpleArray<double> pline_t;
  if (curve.IsPolyline(&pline_pts, &pline_t) >= 2 && pline_pts.Count() == pline_t.Count())
  {
    const int n = pline_pts.Count();
    for (int i = 0; i < n - 1; ++i)
    {
      double t0 = pline_t[i];
      double t1 = pline_t[i + 1];
      if (t1 < dom_min || t0 > dom_max)
        continue;

      ON_Line seg(pline_pts[i], pline_pts[i + 1]);
      double s = 0.0;
      if (seg.ClosestPointTo(test_point, &s))
      {
        s = std::clamp(s, 0.0, 1.0);
        double tc = (1.0 - s) * t0 + s * t1;
        update_candidate(tc);
      }
      update_candidate(t0);
      update_candidate(t1);
    }
  }

  // Collect smooth spans and perform multi-span sampling + damped Newton-Raphson
  const int span_count = std::max(1, curve.SpanCount());
  std::vector<double> spans(static_cast<size_t>(span_count + 1));
  bool has_spans = curve.GetSpanVector(spans.data());
  if (!has_spans)
  {
    spans.resize(2);
    spans[0] = dom_min;
    spans[1] = dom_max;
  }

  const int degree = std::max(1, curve.Degree());
  const int samples_per_span = std::max(12, (degree + 1) * 6);

  struct SpanSeed
  {
    double t = 0.0;
    double span_min = 0.0;
    double span_max = 0.0;
    double dist_sq = std::numeric_limits<double>::infinity();
  };
  std::vector<SpanSeed> seeds;
  seeds.reserve(static_cast<size_t>(spans.size()));

  for (size_t si = 0; si + 1 < spans.size(); ++si)
  {
    double s0 = std::max(spans[si], dom_min);
    double s1 = std::min(spans[si + 1], dom_max);
    if (s1 <= s0)
      continue;

    update_candidate(s0);
    update_candidate(s1);

    SpanSeed best_span_seed;
    best_span_seed.span_min = s0;
    best_span_seed.span_max = s1;
    best_span_seed.t = 0.5 * (s0 + s1);

    double prev_t = s0;
    ON_3dPoint prev_pt = curve.PointAt(s0);

    for (int k = 0; k <= samples_per_span; ++k)
    {
      double u = static_cast<double>(k) / static_cast<double>(samples_per_span);
      double tc = (1.0 - u) * s0 + u * s1;
      ON_3dPoint pc = curve.PointAt(tc);
      if (!pc.IsValid())
        continue;

      double d2 = (pc - test_point).LengthSquared();
      if (d2 < best_dist_sq)
      {
        best_dist_sq = d2;
        best_t = tc;
      }
      if (d2 < best_span_seed.dist_sq)
      {
        best_span_seed.dist_sq = d2;
        best_span_seed.t = tc;
      }

      // Also project onto the chord between adjacent samples for better initial guess
      if (k > 0 && prev_pt.IsValid())
      {
        ON_3dVector seg_v = pc - prev_pt;
        double seg_len_sq = seg_v.LengthSquared();
        if (seg_len_sq > 1e-24)
        {
          double frac = ON_DotProduct(test_point - prev_pt, seg_v) / seg_len_sq;
          if (frac > 0.0 && frac < 1.0)
          {
            double t_proj = (1.0 - frac) * prev_t + frac * tc;
            ON_3dPoint p_proj = curve.PointAt(t_proj);
            if (p_proj.IsValid())
            {
              double proj_d2 = (p_proj - test_point).LengthSquared();
              if (proj_d2 < best_dist_sq)
              {
                best_dist_sq = proj_d2;
                best_t = t_proj;
              }
              if (proj_d2 < best_span_seed.dist_sq)
              {
                best_span_seed.dist_sq = proj_d2;
                best_span_seed.t = t_proj;
              }
            }
          }
        }
      }

      prev_t = tc;
      prev_pt = pc;
    }

    if (std::isfinite(best_span_seed.dist_sq))
      seeds.push_back(best_span_seed);
  }

  // Refine each span's best candidate, plus global refinement around best_t
  for (const SpanSeed& seed : seeds)
  {
    RefineCurveClosestPointNewton(
      curve, test_point, seed.span_min, seed.span_max, seed.t, best_t, best_dist_sq);
  }

  RefineCurveClosestPointNewton(
    curve, test_point, dom_min, dom_max, best_t, best_t, best_dist_sq);

  if (!std::isfinite(best_dist_sq))
    return false;

  if (maximum_distance > 0.0 && ON_IsValid(maximum_distance))
  {
    if (best_dist_sq > maximum_distance * maximum_distance)
      return false;
  }

  if (nullptr != t_out)
    *t_out = std::clamp(best_t, dom_min, dom_max);

  return true;
}

namespace {

/**
 * Refine (u, v) on `surface` using 2x2 damped Newton-Raphson / Gauss-Newton iteration:
 *   Objective: E(u, v) = 0.5 * ||S(u, v) - P||^2
 *   Gradient:  g_u = r . S_u,  g_v = r . S_v   where r = S(u, v) - P
 *   Hessian:   H_uu = S_u . S_u + r . S_uu
 *              H_uv = S_u . S_v + r . S_uv
 *              H_vv = S_v . S_v + r . S_vv
 */
void RefineSurfaceClosestPointNewton(
  const ON_Surface& surface,
  const ON_3dPoint& test_point,
  const ON_Interval& u_dom,
  const ON_Interval& v_dom,
  double u_init,
  double v_init,
  double& best_u,
  double& best_v,
  double& best_dist_sq
)
{
  const double u_min = u_dom.Min();
  const double u_max = u_dom.Max();
  const double v_min = v_dom.Min();
  const double v_max = v_dom.Max();

  double u = std::clamp(u_init, u_min, u_max);
  double v = std::clamp(v_init, v_min, v_max);

  ON_3dPoint pt = ON_3dPoint::Origin;
  ON_3dVector du = ON_3dVector::ZeroVector;
  ON_3dVector dv = ON_3dVector::ZeroVector;
  ON_3dVector duu = ON_3dVector::ZeroVector;
  ON_3dVector duv = ON_3dVector::ZeroVector;
  ON_3dVector dvv = ON_3dVector::ZeroVector;

  if (!surface.EvPoint(u, v, pt))
    return;

  double curr_dist_sq = (pt - test_point).LengthSquared();
  if (curr_dist_sq < best_dist_sq)
  {
    best_dist_sq = curr_dist_sq;
    best_u = u;
    best_v = v;
  }

  const double u_span = u_max - u_min;
  const double v_span = v_max - v_min;
  const double u_tol = std::max(1e-13, u_span * 1e-12);
  const double v_tol = std::max(1e-13, v_span * 1e-12);
  const int max_iters = 28;

  for (int iter = 0; iter < max_iters; ++iter)
  {
    if (!surface.Ev2Der(u, v, pt, du, dv, duu, duv, dvv))
    {
      if (!surface.Ev1Der(u, v, pt, du, dv))
        break;
      duu = ON_3dVector::ZeroVector;
      duv = ON_3dVector::ZeroVector;
      dvv = ON_3dVector::ZeroVector;
    }

    const ON_3dVector r = pt - test_point;
    curr_dist_sq = r.LengthSquared();
    if (curr_dist_sq < best_dist_sq)
    {
      best_dist_sq = curr_dist_sq;
      best_u = u;
      best_v = v;
    }

    double g_u = ON_DotProduct(r, du);
    double g_v = ON_DotProduct(r, dv);

    // Project gradient if on domain boundary and pointing outward
    const bool clamp_u = (u <= u_min + u_tol && g_u > 0.0) || (u >= u_max - u_tol && g_u < 0.0);
    const bool clamp_v = (v <= v_min + v_tol && g_v > 0.0) || (v >= v_max - v_tol && g_v < 0.0);
    if (clamp_u)
      g_u = 0.0;
    if (clamp_v)
      g_v = 0.0;

    if (std::fabs(g_u) <= 1e-13 && std::fabs(g_v) <= 1e-13)
      break;

    // First fundamental form (Gauss-Newton matrix J^T J)
    const double j_uu = ON_DotProduct(du, du);
    const double j_uv = ON_DotProduct(du, dv);
    const double j_vv = ON_DotProduct(dv, dv);

    // Full Newton Hessian
    double h_uu = j_uu + ON_DotProduct(r, duu);
    double h_uv = j_uv + ON_DotProduct(r, duv);
    double h_vv = j_vv + ON_DotProduct(r, dvv);

    if (clamp_u)
    {
      h_uu = 1.0;
      h_uv = 0.0;
    }
    if (clamp_v)
    {
      h_vv = 1.0;
      h_uv = 0.0;
    }

    // Check positive definiteness of full Hessian; fallback to Gauss-Newton + Levenberg damping if needed
    double det = h_uu * h_vv - h_uv * h_uv;
    if (h_uu <= 1e-14 || h_vv <= 1e-14 || det <= 1e-12 * (j_uu * j_vv + 1e-20))
    {
      const double lambda = 1e-6 * (j_uu + j_vv + 1.0);
      h_uu = clamp_u ? 1.0 : (j_uu + lambda);
      h_uv = (clamp_u || clamp_v) ? 0.0 : j_uv;
      h_vv = clamp_v ? 1.0 : (j_vv + lambda);
      det = h_uu * h_vv - h_uv * h_uv;
    }

    if (std::fabs(det) <= 1e-28)
      break;

    double step_u = -(h_vv * g_u - h_uv * g_v) / det;
    double step_v = -(-h_uv * g_u + h_uu * g_v) / det;

    step_u = std::clamp(step_u, -0.5 * u_span, 0.5 * u_span);
    step_v = std::clamp(step_v, -0.5 * v_span, 0.5 * v_span);

    if (std::fabs(step_u) <= u_tol && std::fabs(step_v) <= v_tol)
    {
      double u_f = std::clamp(u + step_u, u_min, u_max);
      double v_f = std::clamp(v + step_v, v_min, v_max);
      ON_3dPoint pt_f;
      if (surface.EvPoint(u_f, v_f, pt_f))
      {
        double d2 = (pt_f - test_point).LengthSquared();
        if (d2 < best_dist_sq)
        {
          best_dist_sq = d2;
          best_u = u_f;
          best_v = v_f;
        }
      }
      break;
    }

    // Backtracking line search
    double alpha = 1.0;
    bool improved = false;
    for (int ls = 0; ls < 10; ++ls)
    {
      double u_cand = std::clamp(u + alpha * step_u, u_min, u_max);
      double v_cand = std::clamp(v + alpha * step_v, v_min, v_max);
      ON_3dPoint pt_cand;
      if (surface.EvPoint(u_cand, v_cand, pt_cand))
      {
        double cand_dist_sq = (pt_cand - test_point).LengthSquared();
        if (cand_dist_sq < curr_dist_sq)
        {
          u = u_cand;
          v = v_cand;
          curr_dist_sq = cand_dist_sq;
          if (cand_dist_sq < best_dist_sq)
          {
            best_dist_sq = cand_dist_sq;
            best_u = u_cand;
            best_v = v_cand;
          }
          improved = true;
          break;
        }
      }
      alpha *= 0.5;
    }

    if (!improved)
      break;
  }
}

} // anonymous namespace

TL_FUNCTION bool TL_GetSurfaceClosestPoint(
  const ON_Surface& surface,
  const ON_3dPoint& test_point,
  double* u_out,
  double* v_out,
  double maximum_distance
)
{
  if (!test_point.IsValid())
    return false;

  const ON_Interval u_dom = surface.Domain(0);
  const ON_Interval v_dom = surface.Domain(1);
  if (!u_dom.IsIncreasing() || !v_dom.IsIncreasing())
    return false;

  const int span_u = std::max(1, surface.SpanCount(0));
  const int span_v = std::max(1, surface.SpanCount(1));
  const int deg_u = std::max(1, surface.Degree(0));
  const int deg_v = std::max(1, surface.Degree(1));

  std::vector<double> knots_u(static_cast<size_t>(span_u + 1));
  std::vector<double> knots_v(static_cast<size_t>(span_v + 1));
  if (!surface.GetSpanVector(0, knots_u.data()))
  {
    knots_u = { u_dom.Min(), u_dom.Max() };
  }
  if (!surface.GetSpanVector(1, knots_v.data()))
  {
    knots_v = { v_dom.Min(), v_dom.Max() };
  }

  const int sub_u = std::max(6, (deg_u + 1) * 3);
  const int sub_v = std::max(6, (deg_v + 1) * 3);

  std::vector<double> sample_u;
  std::vector<double> sample_v;
  sample_u.reserve(knots_u.size() * static_cast<size_t>(sub_u) + 1);
  sample_v.reserve(knots_v.size() * static_cast<size_t>(sub_v) + 1);

  for (size_t i = 0; i + 1 < knots_u.size(); ++i)
  {
    double a = knots_u[i];
    double b = knots_u[i + 1];
    if (b <= a)
      continue;
    for (int k = 0; k < sub_u; ++k)
    {
      sample_u.push_back(a + (b - a) * (static_cast<double>(k) / static_cast<double>(sub_u)));
    }
  }
  sample_u.push_back(u_dom.Max());

  for (size_t j = 0; j + 1 < knots_v.size(); ++j)
  {
    double a = knots_v[j];
    double b = knots_v[j + 1];
    if (b <= a)
      continue;
    for (int k = 0; k < sub_v; ++k)
    {
      sample_v.push_back(a + (b - a) * (static_cast<double>(k) / static_cast<double>(sub_v)));
    }
  }
  sample_v.push_back(v_dom.Max());

  struct GridCandidate
  {
    double u = 0.0;
    double v = 0.0;
    double dist_sq = std::numeric_limits<double>::infinity();
  };

  double best_u = u_dom.Mid();
  double best_v = v_dom.Mid();
  double best_dist_sq = std::numeric_limits<double>::infinity();

  std::vector<GridCandidate> candidates;
  candidates.reserve(sample_u.size() * sample_v.size());

  for (double u : sample_u)
  {
    for (double v : sample_v)
    {
      ON_3dPoint pt;
      if (surface.EvPoint(u, v, pt) && pt.IsValid())
      {
        double d2 = (pt - test_point).LengthSquared();
        if (d2 < best_dist_sq)
        {
          best_dist_sq = d2;
          best_u = u;
          best_v = v;
        }
        candidates.push_back({ u, v, d2 });
      }
    }
  }

  if (candidates.empty() || !std::isfinite(best_dist_sq))
    return false;

  // Sort top-K grid seeds and refine each with 2x2 Gauss-Newton / Newton-Raphson
  const size_t top_k = std::min<size_t>(candidates.size(), 8);
  std::partial_sort(
    candidates.begin(),
    candidates.begin() + static_cast<std::ptrdiff_t>(top_k),
    candidates.end(),
    [](const GridCandidate& a, const GridCandidate& b) { return a.dist_sq < b.dist_sq; });

  for (size_t i = 0; i < top_k; ++i)
  {
    RefineSurfaceClosestPointNewton(
      surface,
      test_point,
      u_dom,
      v_dom,
      candidates[i].u,
      candidates[i].v,
      best_u,
      best_v,
      best_dist_sq);
  }

  // Final polish around best_u, best_v
  RefineSurfaceClosestPointNewton(
    surface, test_point, u_dom, v_dom, best_u, best_v, best_u, best_v, best_dist_sq);

  if (maximum_distance > 0.0 && ON_IsValid(maximum_distance))
  {
    if (best_dist_sq > maximum_distance * maximum_distance)
      return false;
  }

  if (nullptr != u_out)
    *u_out = std::clamp(best_u, u_dom.Min(), u_dom.Max());
  if (nullptr != v_out)
    *v_out = std::clamp(best_v, v_dom.Min(), v_dom.Max());

  return true;
}
