#include "opentl/tl_promote.h"
#include <cmath>
#include <algorithm>

/* ============================================================================
 * TL_Brep::Promote (Reverse-engineered from tl.dll RVA 0x2b2180 & 0x2b21b0)
 *
 * Disassembly of tl.dll!?Promote@TL_Brep@@SAPEBV1@PEBVON_Object@@@Z (0x1802b2180):
 *   40 53                 push    rbx
 *   48 83 ec 20           sub     rsp, 0x20
 *   48 8b d9              mov     rbx, rcx
 *   48 85 c9              test    rcx, rcx
 *   74 17                 je      0x1802b21a5
 *   ff 15 29 6e 4a 00     call    qword ptr [rip + 0x4a6e29] ; opennurbs.dll!ON_Brep::Cast(const ON_Object*)
 *   48 85 c0              test    rax, rax
 *   75 09                 jne     0x1802b21a5
 *   48 8b d3              mov     rdx, rbx
 *   8d 48 03              lea     ecx, [rax + 0x3]
 *   e8 2c 27 0b 00        call    0x1803648d0                ; TL_Error
 *   48 83 c4 20           add     rsp, 0x20
 *   5b                    pop     rbx
 *   c3                    ret
 * ============================================================================ */

const TL_Brep* TL_Brep::Promote(const ON_Object* obj)
{
  if (nullptr == obj)
    return nullptr;
  const ON_Brep* brep = ON_Brep::Cast(obj);
  return static_cast<const TL_Brep*>(brep);
}

TL_Brep* TL_Brep::Promote(ON_Object* obj)
{
  if (nullptr == obj)
    return nullptr;
  ON_Brep* brep = ON_Brep::Cast(obj);
  return static_cast<TL_Brep*>(brep);
}

/* ============================================================================
 * TL_NurbsCurve::Promote & TL_NurbsSurface::Promote
 * (Reverse-engineered from tl.dll RVA 0x2b20c0 & 0x2b21e0)
 * ============================================================================ */

const TL_NurbsCurve* TL_NurbsCurve::Promote(const ON_Object* obj)
{
  if (nullptr == obj)
    return nullptr;
  const ON_NurbsCurve* crv = ON_NurbsCurve::Cast(obj);
  return static_cast<const TL_NurbsCurve*>(crv);
}

TL_NurbsCurve* TL_NurbsCurve::Promote(ON_Object* obj)
{
  if (nullptr == obj)
    return nullptr;
  ON_NurbsCurve* crv = ON_NurbsCurve::Cast(obj);
  return static_cast<TL_NurbsCurve*>(crv);
}

const TL_NurbsSurface* TL_NurbsSurface::Promote(const ON_Object* obj)
{
  if (nullptr == obj)
    return nullptr;
  const ON_NurbsSurface* srf = ON_NurbsSurface::Cast(obj);
  return static_cast<const TL_NurbsSurface*>(srf);
}

TL_NurbsSurface* TL_NurbsSurface::Promote(ON_Object* obj)
{
  if (nullptr == obj)
    return nullptr;
  ON_NurbsSurface* srf = ON_NurbsSurface::Cast(obj);
  return static_cast<TL_NurbsSurface*>(srf);
}

/* ============================================================================
 * Helper: 2D Winding Number / Boundary Containment against an ON_BrepLoop
 * Used by TL_Brep::PointIsOnFace (tl.dll RVA 0x1fcf30 .. 0x1fd350)
 * ============================================================================ */
namespace {

/**
 * Evaluate a single ON_BrepLoop in 2D parameter space against `uv`:
 * Returns:
 *   2 if `uv` lies within `(u_tol, v_tol)` of any 2D trim curve in the loop,
 *   1 if `uv` is strictly inside the closed 2D loop (non-zero winding / odd ray crossings),
 *   0 if `uv` is outside the closed 2D loop.
 */
int EvaluateLoopContainment2d(
  const ON_Brep& brep,
  const ON_BrepLoop& loop,
  const ON_2dPoint& uv,
  double u_tol,
  double v_tol
)
{
  // Quick bounding-box check expanded by (u_tol, v_tol), matching 0x1801fcfb3 .. 0x1801fd013
  ON_BoundingBox loop_bbox = loop.m_pbox;
  if (loop_bbox.IsValid())
  {
    if (uv.x < loop_bbox.m_min.x - u_tol || uv.x > loop_bbox.m_max.x + u_tol ||
        uv.y < loop_bbox.m_min.y - v_tol || uv.y > loop_bbox.m_max.y + v_tol)
    {
      return 0;
    }
  }

  const int trim_count = loop.m_ti.Count();
  if (trim_count <= 0)
    return 0;

  int winding_number = 0;
  const double tol_u = (u_tol > 0.0) ? u_tol : 1e-8;
  const double tol_v = (v_tol > 0.0) ? v_tol : 1e-8;

  for (int lti = 0; lti < trim_count; ++lti)
  {
    const int ti = loop.m_ti[lti];
    if (ti < 0 || ti >= brep.m_T.Count())
      continue;
    const ON_BrepTrim& trim = brep.m_T[ti];
    if (trim.m_type == ON_BrepTrim::singular)
    {
      // Singular trim is a line segment in 2D parameter space
      ON_Interval dom = trim.Domain();
      ON_3dPoint p0 = trim.PointAt(dom.Min());
      ON_3dPoint p1 = trim.PointAt(dom.Max());
      // Check boundary proximity
      ON_Line seg(ON_3dPoint(p0.x, p0.y, 0.0), ON_3dPoint(p1.x, p1.y, 0.0));
      double seg_t = 0.0;
      if (seg.ClosestPointTo(ON_3dPoint(uv.x, uv.y, 0.0), &seg_t))
      {
        seg_t = std::clamp(seg_t, 0.0, 1.0);
        ON_3dPoint cp = seg.PointAt(seg_t);
        if (std::fabs(cp.x - uv.x) <= tol_u && std::fabs(cp.y - uv.y) <= tol_v)
          return 2;
      }
      // Winding number update for singular segment
      if ((p0.y <= uv.y && p1.y > uv.y) || (p0.y > uv.y && p1.y <= uv.y))
      {
        double vt = (uv.y - p0.y) / (p1.y - p0.y);
        double x_int = p0.x + vt * (p1.x - p0.x);
        if (std::fabs(x_int - uv.x) <= tol_u)
          return 2;
        if (uv.x < x_int)
          winding_number += (p0.y <= uv.y) ? 1 : -1;
      }
      continue;
    }

    const ON_Curve* c2 = trim.TrimCurveOf();
    if (nullptr == c2)
      continue;

    ON_Interval dom = trim.Domain();
    const int span_count = std::max(1, c2->SpanCount());
    const int samples = std::max(32, span_count * 16);

    ON_3dPoint prev = c2->PointAt(dom.Min());
    if (std::fabs(prev.x - uv.x) <= tol_u && std::fabs(prev.y - uv.y) <= tol_v)
      return 2;

    for (int s = 1; s <= samples; ++s)
    {
      double t = dom.ParameterAt(static_cast<double>(s) / static_cast<double>(samples));
      ON_3dPoint curr = c2->PointAt(t);

      // Segment bounding box & boundary tolerance test
      double min_x = std::min(prev.x, curr.x) - tol_u;
      double max_x = std::max(prev.x, curr.x) + tol_u;
      double min_y = std::min(prev.y, curr.y) - tol_v;
      double max_y = std::max(prev.y, curr.y) + tol_v;
      if (uv.x >= min_x && uv.x <= max_x && uv.y >= min_y && uv.y <= max_y)
      {
        double dx = curr.x - prev.x;
        double dy = curr.y - prev.y;
        double len2 = dx * dx + dy * dy;
        double proj = 0.0;
        if (len2 > 1e-24)
        {
          proj = ((uv.x - prev.x) * dx + (uv.y - prev.y) * dy) / len2;
          proj = std::clamp(proj, 0.0, 1.0);
        }
        double cx = prev.x + proj * dx;
        double cy = prev.y + proj * dy;
        if (std::fabs(cx - uv.x) <= tol_u && std::fabs(cy - uv.y) <= tol_v)
          return 2;
      }

      // Horizontal ray crossing (+X ray from uv)
      if ((prev.y <= uv.y && curr.y > uv.y) || (prev.y > uv.y && curr.y <= uv.y))
      {
        double vt = (uv.y - prev.y) / (curr.y - prev.y);
        double x_int = prev.x + vt * (curr.x - prev.x);
        if (std::fabs(x_int - uv.x) <= tol_u)
          return 2;
        if (uv.x < x_int)
          winding_number += (prev.y <= uv.y) ? 1 : -1;
      }

      prev = curr;
    }
  }

  return (winding_number != 0) ? 1 : 0;
}

/**
 * Tolerance normalisation of `TL_Brep::PointIsOnFace`
 * (tl.dll 0x1801fd02c..0x1801fd0c8):
 *   lower bound  = sqrt(DBL_EPSILON) ~ 1.4901e-08   (const @0x18065f9d0)
 *   upper bound  = 0.01 * face domain length        (const @0x18065fb88)
 * The binary passes u_tol / v_tol by pointer into its helper, which may fix them
 * in place; this is the equivalent value semantics.
 */
double ClampFaceTolerance(const ON_BrepFace& face, int dir, double tol)
{
  const double floor_tol = std::sqrt(ON_EPSILON);
  double t = (tol > floor_tol) ? tol : floor_tol;

  double span = 0.0;
  const ON_Surface* srf = face.SurfaceOf();
  if (nullptr != srf)
  {
    const ON_Interval dom = srf->Domain(dir);
    if (dom.IsIncreasing())
      span = dom.Length();
  }
  const double cap = 0.01 * std::fabs(span);
  if (cap > floor_tol && t > cap)
    t = cap;
  return t;
}

/**
 * Iso-parameter decomposition used by `TL_Brep::PointIsOnFace`
 * (tl.dll 0x1801fd1bb / 0x1801fd24b -> GetIsoIntervalsHelper + 2D verdict,
 *  medium confidence on the helper's exact arithmetic).
 *
 * For dir == 0 the iso line is `v = uv.y` and the varying coordinate is u;
 * for dir == 1 the iso line is `u = uv.x` and the varying coordinate is v.
 * The trim curves are sampled and each sign change is bisected, giving the
 * crossing coordinates.  Returns
 *   2 - uv lies within `tol` of a crossing (on the face boundary),
 *   1 - uv lies strictly inside an even/odd inside interval,
 *   0 - outside (or no crossing found; the loop test still has the last word).
 */
int IsoContainment(
  const ON_Brep& brep,
  const ON_BrepFace& face,
  int dir,
  const ON_2dPoint& uv,
  double tol
)
{
  const int fixed = 1 - dir;            // axis of the fixed coordinate
  const double fixed_value = (0 == dir) ? uv.y : uv.x;
  const double test_value = (0 == dir) ? uv.x : uv.y;

  std::vector<double> crossings;
  const int loop_count = face.m_li.Count();
  for (int fli = 0; fli < loop_count; ++fli)
  {
    const int li = face.m_li[fli];
    if (li < 0 || li >= brep.m_L.Count())
      continue;
    const ON_BrepLoop& loop = brep.m_L[li];
    for (int ti = 0; ti < loop.m_ti.Count(); ++ti)
    {
      const int trim_index = loop.m_ti[ti];
      if (trim_index < 0 || trim_index >= brep.m_T.Count())
        continue;
      const ON_BrepTrim& trim = brep.m_T[trim_index];
      const ON_Interval dom = trim.Domain();
      if (!dom.IsIncreasing())
        continue;

      const int samples = 64;
      ON_2dPoint prev(0.0, 0.0);
      bool have_prev = false;
      for (int s = 0; s <= samples; ++s)
      {
        const double t = dom.ParameterAt(static_cast<double>(s) / static_cast<double>(samples));
        const ON_3dPoint p = trim.PointAt(t);
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
        {
          have_prev = false;
          continue;
        }
        const ON_2dPoint curr(p.x, p.y);
        if (have_prev)
        {
          const double f0 = (fixed == 0 ? prev.x : prev.y) - fixed_value;
          const double f1 = (fixed == 0 ? curr.x : curr.y) - fixed_value;
          if ((f0 < 0.0) != (f1 < 0.0))
          {
            // Linear interpolation is sufficient at 64 samples per trim.
            const double w = f0 / (f0 - f1);
            const double x0 = (dir == 0 ? prev.x : prev.y);
            const double x1 = (dir == 0 ? curr.x : curr.y);
            crossings.push_back(x0 + w * (x1 - x0));
          }
          else if (std::fabs(f1) <= 1e-12)
          {
            crossings.push_back((dir == 0 ? curr.x : curr.y));
          }
        }
        prev = curr;
        have_prev = true;
      }
    }
  }

  if (crossings.empty())
    return 0;

  std::sort(crossings.begin(), crossings.end());
  for (double c : crossings)
  {
    if (std::fabs(c - test_value) <= tol)
      return 2;
  }

  // Even/odd rule: the interval between crossing 2k and 2k+1 is inside.
  int inside_intervals = 0;
  for (size_t k = 0; k + 1 < crossings.size(); k += 2)
  {
    if (test_value > crossings[k] && test_value < crossings[k + 1])
      ++inside_intervals;
  }
  return (inside_intervals % 2 != 0) ? 1 : 0;
}

} // anonymous namespace

/* ============================================================================
 * TL_Brep::PointIsOnFace (Reverse-engineered from tl.dll RVA 0x1fcf30 .. 0x1fd36a)
 *
 * Disassembly structure (1083 bytes / 206 instructions):
 *   0x1801fcf7a / 0x1801fcf8f: `face_index < 0 || >= m_F.Count()` -> **-1**
 *                (`mov eax,0xffffffff` @0x1801fd331). NOT 0.
 *   0x1801fcfad..0x1801fd0c8: u_tol / v_tol are passed by pointer to the internal
 *                helper and clamped to [sqrt(DBL_EPSILON), 0.01 * face domain length].
 *   0x1801fd0f1..0x1801fd154: OuterLoop().m_pbox expanded by (u_tol, v_tol) as a cheap
 *                early-out -> 0 when uv is outside.
 *   0x1801fd1bb / 0x1801fd24b: `GetIsoIntervalsHelper(dir=0|1)` + 2D verdict.
 *                A 2 (on boundary) returns immediately; a 1 (inside) short-circuits;
 *                only 0 falls through to the loop containment below.
 *   0x1801fd022..: loop containment (outer loop must be inside/on, inner loops
 *                must not strictly contain uv) -> 2 or 0.
 * ============================================================================ */

int TL_Brep::PointIsOnFace(
  int face_index,
  const ON_2dPoint& uv,
  double u_tol,
  double v_tol
) const
{
  // tl.dll 0x1801fcf7a / 0x1801fcf8f -> 0x1801fd331 `mov eax,0xffffffff`.
  // An out-of-range face index reports -1 (invalid), NOT 0 (outside).  Callers
  // in the boolean pipeline must distinguish the two.
  if (face_index < 0 || face_index >= m_F.Count())
    return -1;

  const ON_BrepFace& face = m_F[face_index];
  const int loop_count = face.m_li.Count();

  // tl.dll 0x1801fd02c..0x1801fd0c8: tolerances are clamped into
  // [sqrt(DBL_EPSILON) , 0.01 * face domain length] per direction.
  u_tol = ClampFaceTolerance(face, 0, u_tol);
  v_tol = ClampFaceTolerance(face, 1, v_tol);

  // If the face has no trimming loops, check against the underlying surface domain
  if (loop_count <= 0)
  {
    const ON_Surface* srf = face.SurfaceOf();
    if (nullptr == srf)
      return 0;
    ON_Interval u_dom = srf->Domain(0);
    ON_Interval v_dom = srf->Domain(1);
    if (uv.x < u_dom.Min() - u_tol || uv.x > u_dom.Max() + u_tol ||
        uv.y < v_dom.Min() - v_tol || uv.y > v_dom.Max() + v_tol)
      return 0;
    if (std::fabs(uv.x - u_dom.Min()) <= u_tol || std::fabs(uv.x - u_dom.Max()) <= u_tol ||
        std::fabs(uv.y - v_dom.Min()) <= v_tol || std::fabs(uv.y - v_dom.Max()) <= v_tol)
      return 2;
    return 1;
  }

  // Face 2D parameter-space bounding box check (0x1801fcf95 .. 0x1801fd013).
  // The openNURBS build in external/ has no cached ON_BrepFace::m_pbox member,
  // so the box is derived deterministically from the trimming curves' 2D
  // evaluation (5 samples per trim). This is only an early-out optimisation;
  // the authoritative containment test is the loop evaluation below.
  {
    bool have_pbox = false;
    ON_2dPoint pmin(0.0, 0.0);
    ON_2dPoint pmax(0.0, 0.0);
    for (int fli = 0; fli < loop_count; ++fli)
    {
      const int li = face.m_li[fli];
      if (li < 0 || li >= m_L.Count())
        continue;
      const ON_BrepLoop& loop = m_L[li];
      for (int ti = 0; ti < loop.m_ti.Count(); ++ti)
      {
        const int trim_index = loop.m_ti[ti];
        if (trim_index < 0 || trim_index >= m_T.Count())
          continue;
        const ON_BrepTrim& trim = m_T[trim_index];
        const ON_Interval dom = trim.Domain();
        for (int s = 0; s <= 4; ++s)
        {
          // ON_BrepTrim is an ON_CurveProxy over the 2D trim curve; the proxy
          // evaluates to an ON_3dPoint with z == 0.
          const ON_3dPoint evaluated = trim.PointAt(dom.ParameterAt(0.25 * static_cast<double>(s)));
          ON_2dPoint p(evaluated.x, evaluated.y);
          if (!std::isfinite(p.x) || !std::isfinite(p.y))
            continue;
          if (!have_pbox)
          {
            pmin = p;
            pmax = p;
            have_pbox = true;
          }
          else
          {
            if (p.x < pmin.x) pmin.x = p.x;
            if (p.y < pmin.y) pmin.y = p.y;
            if (p.x > pmax.x) pmax.x = p.x;
            if (p.y > pmax.y) pmax.y = p.y;
          }
        }
      }
    }
    if (have_pbox)
    {
      if (uv.x < pmin.x - u_tol || uv.x > pmax.x + u_tol ||
          uv.y < pmin.y - v_tol || uv.y > pmax.y + v_tol)
      {
        return 0;
      }
    }
  }

  // tl.dll 0x1801fd1bb / 0x1801fd24b: iso-parameter decomposition runs FIRST.
  // A boundary hit (2) returns immediately; an interior hit (1) short-circuits the
  // loop test; only an outside verdict (0) falls through to the loop containment.
  {
    const int iso_u = IsoContainment(*this, face, 0, uv, u_tol);
    if (2 == iso_u)
      return 2;
    const int iso_v = IsoContainment(*this, face, 1, uv, v_tol);
    if (2 == iso_v)
      return 2;
    if (1 == iso_u || 1 == iso_v)
      return 1;
  }

  int result = 1;
  for (int fli = 0; fli < loop_count; ++fli)
  {
    const int li = face.m_li[fli];
    if (li < 0 || li >= m_L.Count())
      continue;
    const ON_BrepLoop& loop = m_L[li];
    const int loop_rel = EvaluateLoopContainment2d(*this, loop, uv, u_tol, v_tol);

    if (loop.m_type == ON_BrepLoop::outer)
    {
      if (loop_rel == 0)
        return 0; // Outside outer loop
      if (loop_rel == 2)
        result = 2; // On outer boundary
    }
    else if (loop.m_type == ON_BrepLoop::inner)
    {
      if (loop_rel == 1)
        return 0; // Strictly inside an inner hole -> outside face
      if (loop_rel == 2)
        result = 2; // On inner hole boundary
    }
  }

  return result;
}

/* ============================================================================
 * TL_Brep::IsPointOnFace (Reverse-engineered from tl.dll RVA 0x1fd520)
 *
 * Disassembly of tl.dll!?IsPointOnFace@TL_Brep@@QEBAHHNNN@Z (0x1801fd520):
 *   48 83 ec 48                 sub     rsp, 0x48
 *   f2 0f 10 44 24 70           movsd   xmm0, qword ptr [rsp + 0x70] ; tol
 *   f2 0f 11 54 24 30           movsd   qword ptr [rsp + 0x30], xmm2 ; uv.x = u
 *   f2 0f 11 5c 24 38           movsd   qword ptr [rsp + 0x38], xmm3 ; uv.y = v
 *   4c 8d 44 24 30              lea     r8, [rsp + 0x30]             ; &uv
 *   f2 0f 11 44 24 20           movsd   qword ptr [rsp + 0x20], xmm0 ; v_tol = tol
 *   0f 28 d8                    movaps  xmm3, xmm0                   ; u_tol = tol
 *   e8 ef f9 ff ff              call    0x1801fcf30                  ; TL_Brep::PointIsOnFace
 *   85 c0                       test    eax, eax
 *   0f 95 c0                    setne   al
 *   48 83 c4 48                 add     rsp, 0x48
 *   c3                          ret
 * ============================================================================ */

bool TL_Brep::IsPointOnFace(
  int face_index,
  double u,
  double v,
  double tol
) const
{
  const ON_2dPoint uv(u, v);
  return PointIsOnFace(face_index, uv, tol, tol) != 0;
}

/* ============================================================================
 * TL_NurbsCurve::RemoveKnots & TL_NurbsSurface::RemoveKnots
 * (Reverse-engineered from tl.dll RVA 0x384d90)
 *
 * Removes knots in `[index0, index1)` by refitting the span / applying Boehm knot
 * removal when geometric deviation is within `tolerance`.
 * ============================================================================ */

bool TL_NurbsCurve::RemoveKnots(int index0, int index1, double tolerance)
{
  if (!IsValid() || m_order < 2)
    return false;
  const int knot_count = KnotCount();
  if (index0 < m_order - 1 || index1 > knot_count - m_order + 1 || index0 >= index1)
    return false;

  const int remove_count = index1 - index0;
  const int new_cv_count = m_cv_count - remove_count;
  if (new_cv_count < m_order)
    return false;

  // Sample original curve to verify tolerance and fit reduced knot vector
  const ON_Interval dom = Domain();
  const int num_samples = std::max(32, m_cv_count * 8);
  ON_3dPointArray sample_pts(num_samples);
  ON_SimpleArray<double> sample_t(num_samples);
  for (int i = 0; i < num_samples; ++i)
  {
    double t = dom.ParameterAt(static_cast<double>(i) / static_cast<double>(num_samples - 1));
    sample_t.Append(t);
    sample_pts.Append(PointAt(t));
  }

  ON_SimpleArray<double> kept_knots(knot_count - remove_count);
  for (int i = 0; i < knot_count; ++i)
  {
    if (i < index0 || i >= index1)
      kept_knots.Append(m_knot[i]);
  }

  ON_NurbsCurve candidate;
  if (!candidate.Create(m_dim, m_is_rat != 0, m_order, new_cv_count))
    return false;
  for (int i = 0; i < kept_knots.Count(); ++i)
    candidate.SetKnot(i, kept_knots[i]);

  // Set Greville CVs from original curve evaluation
  ON_SimpleArray<double> grev(new_cv_count);
  grev.SetCount(new_cv_count);
  if (!candidate.GetGrevilleAbcissae(grev.Array()))
    return false;

  for (int i = 0; i < new_cv_count; ++i)
  {
    ON_3dPoint pt = PointAt(grev[i]);
    candidate.SetCV(i, pt);
  }
  candidate.SetCV(0, PointAtStart());
  candidate.SetCV(new_cv_count - 1, PointAtEnd());

  if (tolerance > 0.0 && ON_IsValid(tolerance))
  {
    for (int i = 0; i < num_samples; ++i)
    {
      if (candidate.PointAt(sample_t[i]).DistanceTo(sample_pts[i]) > tolerance)
        return false;
    }
  }

  *static_cast<ON_NurbsCurve*>(this) = candidate;
  return true;
}

bool TL_NurbsSurface::RemoveKnots(int dir, int index0, int index1, double tolerance)
{
  if (dir < 0 || dir > 1 || !IsValid())
    return false;
  const int order = m_order[dir];
  const int knot_count = KnotCount(dir);
  if (index0 < order - 1 || index1 > knot_count - order + 1 || index0 >= index1)
    return false;

  const int remove_count = index1 - index0;
  const int new_cv_count = m_cv_count[dir] - remove_count;
  if (new_cv_count < order)
    return false;

  ON_SimpleArray<double> kept_knots(knot_count - remove_count);
  for (int i = 0; i < knot_count; ++i)
  {
    if (i < index0 || i >= index1)
      kept_knots.Append(m_knot[dir][i]);
  }

  const int cv_count_u = (dir == 0) ? new_cv_count : m_cv_count[0];
  const int cv_count_v = (dir == 1) ? new_cv_count : m_cv_count[1];

  ON_NurbsSurface candidate;
  if (!candidate.Create(m_dim, m_is_rat != 0, m_order[0], m_order[1], cv_count_u, cv_count_v))
    return false;

  for (int i = 0; i < candidate.KnotCount(1 - dir); ++i)
    candidate.SetKnot(1 - dir, i, m_knot[1 - dir][i]);
  for (int i = 0; i < kept_knots.Count(); ++i)
    candidate.SetKnot(dir, i, kept_knots[i]);

  ON_SimpleArray<double> grev_u(cv_count_u);
  ON_SimpleArray<double> grev_v(cv_count_v);
  grev_u.SetCount(cv_count_u);
  grev_v.SetCount(cv_count_v);
  candidate.GetGrevilleAbcissae(0, grev_u.Array());
  candidate.GetGrevilleAbcissae(1, grev_v.Array());

  for (int iu = 0; iu < cv_count_u; ++iu)
  {
    for (int iv = 0; iv < cv_count_v; ++iv)
    {
      ON_3dPoint pt = PointAt(grev_u[iu], grev_v[iv]);
      candidate.SetCV(iu, iv, pt);
    }
  }

  if (tolerance > 0.0 && ON_IsValid(tolerance))
  {
    ON_Interval dom_u = Domain(0);
    ON_Interval dom_v = Domain(1);
    for (int iu = 0; iu <= 8; ++iu)
    {
      double u = dom_u.ParameterAt(iu / 8.0);
      for (int iv = 0; iv <= 8; ++iv)
      {
        double v = dom_v.ParameterAt(iv / 8.0);
        if (candidate.PointAt(u, v).DistanceTo(PointAt(u, v)) > tolerance)
          return false;
      }
    }
  }

  *static_cast<ON_NurbsSurface*>(this) = candidate;
  return true;
}
