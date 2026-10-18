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

  const ON_Interval arc_dom = arc.DomainRadians();
  if (arc_dom.IsIncreasing())
    curve->SetDomain(arc_dom.Min(), arc_dom.Max());

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

  const ON_Interval cdom = curve->Domain();
  const double radius = arc.Radius();
  const ON_Plane& plane = arc.Plane();

  std::vector<ON_3dPoint> target_pts(static_cast<size_t>(cv_count));
  for (int i = 0; i < cv_count; ++i)
  {
    double norm_s = cdom.NormalizedParameterAt(grev[static_cast<size_t>(i)]);
    norm_s = std::clamp(norm_s, 0.0, 1.0);
    const double angle = arc_dom.ParameterAt(norm_s);
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
