#include "opentl/fitting_loft_sweep/tl_fitting_loft_sweep.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

bool SolveLinearSystem3d(
  int n,
  std::vector<double>& A,
  std::vector<ON_3dPoint>& rhs,
  std::vector<ON_3dPoint>& sol
)
{
  if (n <= 0 || static_cast<int>(A.size()) != n * n || static_cast<int>(rhs.size()) != n)
    return false;

  sol = rhs;

  for (int col = 0; col < n; ++col)
  {
    int pivot = col;
    double max_val = std::fabs(A[col * n + col]);
    for (int row = col + 1; row < n; ++row)
    {
      const double v = std::fabs(A[row * n + col]);
      if (v > max_val)
      {
        max_val = v;
        pivot = row;
      }
    }

    if (max_val <= 1e-14)
      return false;

    if (pivot != col)
    {
      for (int j = col; j < n; ++j)
        std::swap(A[col * n + j], A[pivot * n + j]);
      std::swap(sol[col], sol[pivot]);
    }

    const double diag = A[col * n + col];
    for (int row = col + 1; row < n; ++row)
    {
      const double factor = A[row * n + col] / diag;
      if (std::fabs(factor) <= 1e-18)
        continue;
      A[row * n + col] = 0.0;
      for (int j = col + 1; j < n; ++j)
        A[row * n + j] -= factor * A[col * n + j];
      sol[row].x -= factor * sol[col].x;
      sol[row].y -= factor * sol[col].y;
      sol[row].z -= factor * sol[col].z;
    }
  }

  for (int row = n - 1; row >= 0; --row)
  {
    ON_3dPoint acc = sol[row];
    for (int j = row + 1; j < n; ++j)
    {
      const double a_ij = A[row * n + j];
      acc.x -= a_ij * sol[j].x;
      acc.y -= a_ij * sol[j].y;
      acc.z -= a_ij * sol[j].z;
    }
    const double diag = A[row * n + row];
    sol[row].x = acc.x / diag;
    sol[row].y = acc.y / diag;
    sol[row].z = acc.z / diag;
  }

  return true;
}

void EvaluateClampedCurveBasisRow(
  const ON_NurbsCurve& curve,
  double t,
  std::vector<double>& row
)
{
  const int n = curve.CVCount();
  row.assign(static_cast<size_t>(n), 0.0);

  const int order = curve.Order();
  const double* knots = curve.Knot();
  if (nullptr == knots || order < 2 || n < order)
    return;

  const int span = ON_NurbsSpanIndex(order, n, knots, t, 0, 0);
  if (span < 0 || span > n - order)
    return;

  std::vector<double> N(static_cast<size_t>(order * order), 0.0);
  if (ON_EvaluateNurbsBasis(order, knots + span, t, N.data()))
  {
    for (int j = 0; j < order; ++j)
    {
      const int cvi = span + j;
      if (cvi >= 0 && cvi < n)
        row[static_cast<size_t>(cvi)] = N[static_cast<size_t>(j)];
    }
  }
}

} // anonymous namespace

TL_FUNCTION ON_NurbsCurve* TL_DeformableArc2(
  const ON_Arc& arc,
  int degree,
  int cv_count,
  ON_NurbsCurve* result
)
{
  if (degree < 1 || cv_count <= degree || !arc.IsValid())
    return nullptr;

  const int order = degree + 1;
  ON_NurbsCurve* curve = (nullptr != result) ? result : ON_NurbsCurve::New();
  if (nullptr == curve)
    return nullptr;

  if (!curve->Create(3, false, order, cv_count))
  {
    if (nullptr == result)
      delete curve;
    return nullptr;
  }

  if (!curve->MakeClampedUniformKnotVector(1.0))
  {
    if (nullptr == result)
      delete curve;
    return nullptr;
  }

  // tl.dll 0x18037b307 samples CVs on the *original* clamped knot domain
  // [0, SpanCount()]; the arc-length domain is only assigned at the end
  // (0x18037b69f `SetDomain(0, ON_Arc::Length())`).  Sampling in radians here
  // (the previous implementation) produced the wrong parameterisation.
  const double angle_span = arc.AngleRadians();
  const int span_count = std::max(1, curve->SpanCount());

  std::vector<double> grev(static_cast<size_t>(cv_count), 0.0);
  if (!curve->GetGrevilleAbcissae(grev.data()))
  {
    const ON_Interval cdom = curve->Domain();
    for (int i = 0; i < cv_count; ++i)
    {
      grev[static_cast<size_t>(i)] = cdom.ParameterAt(
        static_cast<double>(i) / static_cast<double>(cv_count - 1)
      );
    }
  }

  const double radius = arc.Radius();
  const ON_Plane& plane = arc.Plane();

  std::vector<ON_3dPoint> target_pts(static_cast<size_t>(cv_count));
  for (int i = 0; i < cv_count; ++i)
  {
    // angle = AngleRadians() * greville / SpanCount()   (0x18037b320..0x18037b3b1)
    const double angle = angle_span * grev[static_cast<size_t>(i)] /
      static_cast<double>(span_count);
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    target_pts[static_cast<size_t>(i)] = plane.PointAt(x, y);
  }
  target_pts.front() = arc.StartPoint();
  target_pts.back() = arc.EndPoint();

  std::vector<double> A(static_cast<size_t>(cv_count * cv_count), 0.0);
  std::vector<double> row;
  for (int i = 0; i < cv_count; ++i)
  {
    EvaluateClampedCurveBasisRow(*curve, grev[static_cast<size_t>(i)], row);
    for (int j = 0; j < cv_count; ++j)
      A[static_cast<size_t>(i * cv_count + j)] = row[static_cast<size_t>(j)];
  }

  std::vector<ON_3dPoint> cvs;
  if (SolveLinearSystem3d(cv_count, A, target_pts, cvs))
  {
    cvs.front() = arc.StartPoint();
    cvs.back() = arc.EndPoint();
    for (int i = 0; i < cv_count; ++i)
      curve->SetCV(i, cvs[static_cast<size_t>(i)]);
  }
  else
  {
    for (int i = 0; i < cv_count; ++i)
      curve->SetCV(i, target_pts[static_cast<size_t>(i)]);
  }

  // tl.dll 0x18037b69f: the finished curve is re-parameterised to arc length,
  // i.e. domain = [0, ON_Arc::Length()] (NOT the radian domain).
  const double arc_length = arc.Length();
  if (ON_IsValid(arc_length) && arc_length > 0.0)
    curve->SetDomain(0.0, arc_length);

  return curve;
}

TL_FUNCTION ON_NurbsCurve* TL_DeformableCircle(
  const ON_Circle& circle,
  int degree,
  int cv_count,
  ON_NurbsCurve* result
)
{
  if (degree < 1 || cv_count < std::max(3, degree) || !circle.IsValid())
    return nullptr;

  const int order = degree + 1;
  ON_NurbsCurve* curve = (nullptr != result) ? result : ON_NurbsCurve::New();
  if (nullptr == curve)
    return nullptr;

  if (degree == 1)
  {
    std::vector<ON_3dPoint> pts(static_cast<size_t>(cv_count + 1));
    for (int i = 0; i < cv_count; ++i)
    {
      const double angle = (2.0 * ON_PI * static_cast<double>(i)) / static_cast<double>(cv_count);
      pts[static_cast<size_t>(i)] = circle.PointAt(angle);
    }
    pts[static_cast<size_t>(cv_count)] = pts[0];
    if (!curve->CreateClampedUniformNurbs(3, 2, cv_count + 1, pts.data(), 1.0))
    {
      if (nullptr == result)
        delete curve;
      return nullptr;
    }
    curve->SetDomain(0.0, 2.0 * ON_PI);
    return curve;
  }

  // Compute radial dilation factor so periodic B-spline limit curve interpolates the circle
  ON_NurbsCurve unit_test;
  std::vector<ON_3dPoint> unit_pts(static_cast<size_t>(cv_count));
  for (int i = 0; i < cv_count; ++i)
  {
    const double angle = (2.0 * ON_PI * static_cast<double>(i)) / static_cast<double>(cv_count);
    unit_pts[static_cast<size_t>(i)] = ON_3dPoint(std::cos(angle), std::sin(angle), 0.0);
  }

  double scale = 1.0;
  // A periodic uniform B-spline does not start at control point 0: its domain
  // start sits between control points. Recover that angular offset from the
  // unit control polygon so the emitted curve starts exactly at circle.PointAt(0).
  double angle_offset = 0.0;
  if (unit_test.CreatePeriodicUniformNurbs(3, order, cv_count, unit_pts.data(), 1.0))
  {
    const ON_Interval dom = unit_test.Domain();
    ON_3dPoint p0 = unit_test.PointAt(dom.Min());
    const double r_eval = std::hypot(p0.x, p0.y);
    if (r_eval > 1e-12)
      scale = 1.0 / r_eval;
    angle_offset = std::atan2(p0.y, p0.x);
  }

  const double r_cv = circle.Radius() * scale;
  const ON_Plane& plane = circle.Plane();
  std::vector<ON_3dPoint> cv_pts(static_cast<size_t>(cv_count));
  for (int i = 0; i < cv_count; ++i)
  {
    const double angle = (2.0 * ON_PI * static_cast<double>(i)) / static_cast<double>(cv_count) - angle_offset;
    cv_pts[static_cast<size_t>(i)] = plane.PointAt(r_cv * std::cos(angle), r_cv * std::sin(angle));
  }

  if (!curve->CreatePeriodicUniformNurbs(3, order, cv_count, cv_pts.data(), 1.0))
  {
    if (nullptr == result)
      delete curve;
    return nullptr;
  }

  curve->SetDomain(0.0, 2.0 * ON_PI);
  return curve;
}

TL_FUNCTION ON_NurbsSurface* TL_LoftCurves(
  const ON_SimpleArray<const ON_Curve*>& sections,
  ON_NurbsSurface* result
)
{
  std::vector<const ON_Curve*> valid_sections;
  valid_sections.reserve(static_cast<size_t>(sections.Count()));
  for (int i = 0; i < sections.Count(); ++i)
  {
    if (nullptr != sections[i] && sections[i]->IsValid())
      valid_sections.push_back(sections[i]);
  }

  const int section_count = static_cast<int>(valid_sections.size());
  if (section_count < 2)
    return nullptr;

  ON_NurbsSurface* srf = (nullptr != result) ? result : ON_NurbsSurface::New();
  if (nullptr == srf)
    return nullptr;

  // Fast exact path for 2 sections when CreateRuledSurface succeeds
  if (section_count == 2)
  {
    if (srf->CreateRuledSurface(*valid_sections[0], *valid_sections[1]) > 0 && srf->IsValid())
      return srf;
  }

  // Convert each section to a compatible clamped uniform non-rational 3D ON_NurbsCurve
  int max_degree_u = 1;
  int max_cv_u = 4;
  for (const ON_Curve* c : valid_sections)
  {
    max_degree_u = std::max(max_degree_u, std::clamp(c->Degree(), 1, 3));
    ON_NurbsCurve tmp;
    if (c->GetNurbForm(tmp) > 0)
      max_cv_u = std::max(max_cv_u, tmp.CVCount());
  }

  const int deg_u = max_degree_u;
  const int order_u = deg_u + 1;
  const int cv_count_u = std::max(order_u, std::min(64, std::max(max_cv_u, 12)));

  const int deg_v = std::min(3, section_count - 1);
  const int order_v = deg_v + 1;
  const int cv_count_v = section_count;

  if (!srf->Create(3, false, order_u, order_v, cv_count_u, cv_count_v))
  {
    if (nullptr == result)
      delete srf;
    return nullptr;
  }

  srf->MakeClampedUniformKnotVector(0, 1.0);
  srf->MakeClampedUniformKnotVector(1, 1.0);
  srf->SetDomain(0, 0.0, 1.0);
  srf->SetDomain(1, 0.0, 1.0);

  // Build 1D basis curves along U and V to solve for interpolating control net
  ON_NurbsCurve basis_u(3, false, order_u, cv_count_u);
  basis_u.MakeClampedUniformKnotVector(1.0);
  basis_u.SetDomain(0.0, 1.0);

  ON_NurbsCurve basis_v(3, false, order_v, cv_count_v);
  basis_v.MakeClampedUniformKnotVector(1.0);
  basis_v.SetDomain(0.0, 1.0);

  std::vector<double> grev_u(static_cast<size_t>(cv_count_u), 0.0);
  std::vector<double> grev_v(static_cast<size_t>(cv_count_v), 0.0);
  if (!basis_u.GetGrevilleAbcissae(grev_u.data()))
  {
    for (int i = 0; i < cv_count_u; ++i)
      grev_u[static_cast<size_t>(i)] = static_cast<double>(i) / static_cast<double>(cv_count_u - 1);
  }
  if (!basis_v.GetGrevilleAbcissae(grev_v.data()))
  {
    for (int j = 0; j < cv_count_v; ++j)
      grev_v[static_cast<size_t>(j)] = static_cast<double>(j) / static_cast<double>(cv_count_v - 1);
  }

  std::vector<double> Au_base(static_cast<size_t>(cv_count_u * cv_count_u), 0.0);
  std::vector<double> row;
  for (int i = 0; i < cv_count_u; ++i)
  {
    EvaluateClampedCurveBasisRow(basis_u, grev_u[static_cast<size_t>(i)], row);
    for (int j = 0; j < cv_count_u; ++j)
      Au_base[static_cast<size_t>(i * cv_count_u + j)] = row[static_cast<size_t>(j)];
  }

  // Step 1: Fit each section curve in U at Greville abscissae grev_u
  std::vector<std::vector<ON_3dPoint>> section_cvs(
    static_cast<size_t>(cv_count_v),
    std::vector<ON_3dPoint>(static_cast<size_t>(cv_count_u), ON_3dPoint::Origin)
  );

  for (int iv = 0; iv < cv_count_v; ++iv)
  {
    const ON_Curve* sec = valid_sections[static_cast<size_t>(iv)];
    const ON_Interval sec_dom = sec->Domain();
    std::vector<ON_3dPoint> samples_u(static_cast<size_t>(cv_count_u));
    for (int iu = 0; iu < cv_count_u; ++iu)
    {
      const double u_norm = std::clamp(grev_u[static_cast<size_t>(iu)], 0.0, 1.0);
      const double t = sec_dom.ParameterAt(u_norm);
      samples_u[static_cast<size_t>(iu)] = sec->PointAt(t);
    }
    samples_u.front() = sec->PointAtStart();
    samples_u.back() = sec->PointAtEnd();

    std::vector<double> Au = Au_base;
    if (!SolveLinearSystem3d(cv_count_u, Au, samples_u, section_cvs[static_cast<size_t>(iv)]))
    {
      section_cvs[static_cast<size_t>(iv)] = samples_u;
    }
  }

  // Step 2: Interpolate across sections in V for each U control column
  std::vector<double> Av_base(static_cast<size_t>(cv_count_v * cv_count_v), 0.0);
  for (int i = 0; i < cv_count_v; ++i)
  {
    EvaluateClampedCurveBasisRow(basis_v, grev_v[static_cast<size_t>(i)], row);
    for (int j = 0; j < cv_count_v; ++j)
      Av_base[static_cast<size_t>(i * cv_count_v + j)] = row[static_cast<size_t>(j)];
  }

  for (int iu = 0; iu < cv_count_u; ++iu)
  {
    std::vector<ON_3dPoint> col_rhs(static_cast<size_t>(cv_count_v));
    for (int iv = 0; iv < cv_count_v; ++iv)
      col_rhs[static_cast<size_t>(iv)] = section_cvs[static_cast<size_t>(iv)][static_cast<size_t>(iu)];

    std::vector<double> Av = Av_base;
    std::vector<ON_3dPoint> col_sol;
    if (!SolveLinearSystem3d(cv_count_v, Av, col_rhs, col_sol))
      col_sol = col_rhs;

    for (int iv = 0; iv < cv_count_v; ++iv)
      srf->SetCV(iu, iv, col_sol[static_cast<size_t>(iv)]);
  }

  return srf;
}

// ============================================================================
// TL_Sweep1Rail -- parallel-transport sweep (approximation, see header)
// ============================================================================

TL_FUNCTION ON_NurbsSurface* TL_Sweep1Rail(
  const ON_Curve& rail,
  const ON_Curve& profile,
  int station_count,
  ON_NurbsSurface* result
)
{
  const ON_Interval rail_dom = rail.Domain();
  const ON_Interval prof_dom = profile.Domain();
  if (!rail_dom.IsIncreasing() || !prof_dom.IsIncreasing())
    return nullptr;

  const int stations = std::clamp(station_count, 2, 256);
  // Profile fidelity is bounded by the loft's U control count (<= 64 CVs), so a
  // finer tessellation than that buys nothing: for R = 2 the residual radial error
  // is ~ R*(1-cos(pi/64)) ~ 2.4e-3.
  constexpr int kProfileSamples = 64;

  std::vector<ON_2dPoint> profile_points(static_cast<size_t>(kProfileSamples) + 1);
  for (int k = 0; k <= kProfileSamples; ++k)
  {
    const double s = static_cast<double>(k) / static_cast<double>(kProfileSamples);
    const ON_3dPoint p = profile.PointAt(prof_dom.ParameterAt(s));
    profile_points[static_cast<size_t>(k)] = ON_2dPoint(p.x, p.y);
  }
  // Closed profile: make the seam exact.
  if (profile_points.front().DistanceTo(profile_points.back()) <= 1e-9)
    profile_points.back() = profile_points.front();

  ON_SimpleArray<ON_Curve*> owned;
  ON_SimpleArray<const ON_Curve*> sections;

  ON_3dVector previous_up(1.0, 0.0, 0.0);
  bool have_frame = false;

  for (int i = 0; i < stations; ++i)
  {
    const double s = static_cast<double>(i) / static_cast<double>(stations - 1);
    const double t = rail_dom.ParameterAt(s);
    const ON_3dPoint origin = rail.PointAt(t);

    ON_3dPoint frame_origin = ON_3dPoint::Origin;
    ON_3dVector tangent = ON_3dVector::ZeroVector;
    if (!rail.Ev1Der(t, frame_origin, tangent) || !tangent.Unitize())
    {
      if (i == 0)
        return nullptr;
      tangent = ON_3dVector(0.0, 0.0, 1.0);
    }

    // Parallel transport: remove the tangential component of the previous up vector.
    ON_3dVector up = previous_up - tangent * ON_DotProduct(previous_up, tangent);
    if (!up.Unitize())
    {
      // The previous up became parallel to the tangent: pick any perpendicular.
      const ON_3dVector fallback = (std::fabs(tangent.z) < 0.9) ? ON_3dVector(0.0, 0.0, 1.0)
                                                                : ON_3dVector(1.0, 0.0, 0.0);
      up = ON_CrossProduct(tangent, fallback);
      if (!up.Unitize())
        return nullptr;
      if (have_frame)
        up = previous_up;
    }
    previous_up = up;
    have_frame = true;

    const ON_3dVector across = ON_CrossProduct(tangent, up);

    ON_SimpleArray<ON_3dPoint> points(static_cast<int>(profile_points.size()));
    for (const ON_2dPoint& p : profile_points)
      points.Append(origin + up * p.x + across * p.y);

    ON_PolylineCurve* section = new ON_PolylineCurve(points);
    if (nullptr == section)
      break;
    owned.Append(section);
    sections.Append(section);
  }

  ON_NurbsSurface* surface = nullptr;
  if (sections.Count() >= 2)
    surface = TL_LoftCurves(sections, result);

  for (int i = 0; i < owned.Count(); ++i)
    delete owned[i];

  return surface;
}

// ============================================================================
// TL_Sweep2Rails -- two-rail proportional sweep + loft skinning
// ============================================================================

TL_FUNCTION ON_NurbsSurface* TL_Sweep2Rails(
  const ON_Curve& rail1,
  const ON_Curve& rail2,
  const ON_Curve& profile,
  int station_count,
  ON_NurbsSurface* result
)
{
  const ON_Interval dom1 = rail1.Domain();
  const ON_Interval dom2 = rail2.Domain();
  const ON_Interval pdom = profile.Domain();
  if (!dom1.IsIncreasing() || !dom2.IsIncreasing() || !pdom.IsIncreasing())
    return nullptr;

  const int stations = std::clamp(station_count, 2, 256);
  constexpr int kProfileSamples = 64;

  const ON_3dPoint r1_0 = rail1.PointAtStart();
  const ON_3dPoint r2_0 = rail2.PointAtStart();
  const double base_span = std::max(1e-6, (r2_0 - r1_0).Length());

  ON_3dVector x0 = r2_0 - r1_0;
  if (!x0.Unitize())
    x0 = ON_3dVector::XAxis;
  ON_3dPoint dummy = ON_3dPoint::Origin;
  ON_3dVector t0 = ON_3dVector::ZeroVector;
  if (!rail1.Ev1Der(dom1.Min(), dummy, t0) || !t0.Unitize())
    t0 = ON_3dVector::YAxis;
  ON_3dVector y0 = ON_CrossProduct(t0, x0);
  if (!y0.Unitize())
    y0 = ON_3dVector::ZAxis;

  // Record profile coordinates relative to the initial two-rail frame at station 0.
  std::vector<ON_2dPoint> local_uv(static_cast<size_t>(kProfileSamples) + 1);
  for (int k = 0; k <= kProfileSamples; ++k)
  {
    const double s = static_cast<double>(k) / static_cast<double>(kProfileSamples);
    const ON_3dPoint p = profile.PointAt(pdom.ParameterAt(s));
    const ON_3dVector rel = p - r1_0;
    // If profile lies in XY plane near origin rather than attached to rail1, use p.x/p.y directly.
    if ((profile.PointAtStart() - r1_0).Length() > base_span * 2.5 &&
        std::fabs(p.z) < 1e-6)
    {
      local_uv[static_cast<size_t>(k)] = ON_2dPoint(p.x / base_span, p.y / base_span);
    }
    else
    {
      local_uv[static_cast<size_t>(k)] = ON_2dPoint(
        ON_DotProduct(rel, x0) / base_span,
        ON_DotProduct(rel, y0) / base_span
      );
    }
  }

  ON_SimpleArray<ON_Curve*> owned;
  ON_SimpleArray<const ON_Curve*> sections;
  ON_3dVector prev_y = y0;

  for (int i = 0; i < stations; ++i)
  {
    const double s = static_cast<double>(i) / static_cast<double>(stations - 1);
    const double u1 = dom1.ParameterAt(s);
    const double u2 = dom2.ParameterAt(s);
    const ON_3dPoint p1 = rail1.PointAt(u1);
    const ON_3dPoint p2 = rail2.PointAt(u2);
    const ON_3dVector span_vec = p2 - p1;
    const double span = std::max(1e-6, span_vec.Length());
    ON_3dVector x_axis = span_vec / span;

    ON_3dPoint ev1 = ON_3dPoint::Origin, ev2 = ON_3dPoint::Origin;
    ON_3dVector tan1 = ON_3dVector::ZeroVector, tan2 = ON_3dVector::ZeroVector;
    rail1.Ev1Der(u1, ev1, tan1);
    rail2.Ev1Der(u2, ev2, tan2);
    ON_3dVector tangent = tan1 + tan2;
    if (!tangent.Unitize())
      tangent = t0;

    ON_3dVector y_axis = ON_CrossProduct(tangent, x_axis);
    if (!y_axis.Unitize())
      y_axis = prev_y;
    if (ON_DotProduct(y_axis, prev_y) < 0.0)
      y_axis = -y_axis;
    prev_y = y_axis;

    ON_SimpleArray<ON_3dPoint> pts(static_cast<int>(local_uv.size()));
    for (const ON_2dPoint& uv : local_uv)
      pts.Append(p1 + x_axis * (uv.x * span) + y_axis * (uv.y * span));

    ON_PolylineCurve* sec = new ON_PolylineCurve(pts);
    owned.Append(sec);
    sections.Append(sec);
  }

  ON_NurbsSurface* srf = nullptr;
  if (sections.Count() >= 2)
    srf = TL_LoftCurves(sections, result);
  for (int i = 0; i < owned.Count(); ++i)
    delete owned[i];
  return srf;
}

// ============================================================================
// TL_OffsetCurve -- planar normal offset + cubic Greville NURBS interpolation
// ============================================================================

TL_FUNCTION ON_NurbsCurve* TL_OffsetCurve(
  const ON_Curve& curve,
  const ON_Plane& plane,
  double distance,
  int sample_count,
  ON_NurbsCurve* result
)
{
  const ON_Interval dom = curve.Domain();
  if (!dom.IsIncreasing())
    return nullptr;

  ON_3dVector plane_n = plane.zaxis;
  if (!plane_n.Unitize())
    plane_n = ON_3dVector::ZAxis;

  const int cv_count = std::clamp(sample_count, 4, 256);
  const int deg = std::clamp(curve.Degree(), 1, 3);
  const int order = deg + 1;

  ON_NurbsCurve* out = (nullptr != result) ? result : ON_NurbsCurve::New();
  if (!out->Create(3, false, order, cv_count))
  {
    if (nullptr == result)
      delete out;
    return nullptr;
  }
  out->MakeClampedUniformKnotVector(1.0);
  out->SetDomain(dom.Min(), dom.Max());

  std::vector<double> grev(static_cast<size_t>(cv_count), 0.0);
  if (!out->GetGrevilleAbcissae(grev.data()))
  {
    for (int i = 0; i < cv_count; ++i)
      grev[static_cast<size_t>(i)] = dom.ParameterAt(static_cast<double>(i) / static_cast<double>(cv_count - 1));
  }

  std::vector<ON_3dPoint> rhs(static_cast<size_t>(cv_count), ON_3dPoint::Origin);
  ON_3dVector last_n(1.0, 0.0, 0.0);
  for (int i = 0; i < cv_count; ++i)
  {
    const double t = std::clamp(grev[static_cast<size_t>(i)], dom.Min(), dom.Max());
    ON_3dPoint p = ON_3dPoint::Origin;
    ON_3dVector tan = ON_3dVector::ZeroVector;
    if (!curve.Ev1Der(t, p, tan) || !tan.Unitize())
    {
      p = curve.PointAt(t);
      rhs[static_cast<size_t>(i)] = p + last_n * distance;
      continue;
    }
    ON_3dVector n = ON_CrossProduct(tan, plane_n);
    if (!n.Unitize())
      n = last_n;
    last_n = n;
    rhs[static_cast<size_t>(i)] = p + n * distance;
  }

  // Normalize basis evaluation to [0, 1]
  ON_NurbsCurve basis(3, false, order, cv_count);
  basis.MakeClampedUniformKnotVector(1.0);
  basis.SetDomain(0.0, 1.0);
  std::vector<double> grev01(static_cast<size_t>(cv_count), 0.0);
  if (!basis.GetGrevilleAbcissae(grev01.data()))
  {
    for (int i = 0; i < cv_count; ++i)
      grev01[static_cast<size_t>(i)] = static_cast<double>(i) / static_cast<double>(cv_count - 1);
  }
  std::vector<double> A(static_cast<size_t>(cv_count * cv_count), 0.0);
  std::vector<double> row;
  for (int i = 0; i < cv_count; ++i)
  {
    EvaluateClampedCurveBasisRow(basis, grev01[static_cast<size_t>(i)], row);
    for (int j = 0; j < cv_count; ++j)
      A[static_cast<size_t>(i * cv_count + j)] = row[static_cast<size_t>(j)];
  }

  std::vector<ON_3dPoint> cvs;
  if (!SolveLinearSystem3d(cv_count, A, rhs, cvs))
    cvs = rhs;
  for (int i = 0; i < cv_count; ++i)
    out->SetCV(i, cvs[static_cast<size_t>(i)]);

  return out;
}
