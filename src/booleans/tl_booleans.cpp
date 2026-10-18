#include "opentl/booleans/tl_booleans.h"

#include <algorithm>
#include <cmath>
#include <vector>

TL_FUNCTION TL_PointFaceRelation TL_ClassifyPointOnBrepFace(
  const ON_Brep& brep,
  int face_index,
  const ON_2dPoint& uv,
  double u_tol,
  double v_tol
)
{
  const TL_Brep* tl_brep = TL_Brep::Promote(&brep);
  if (nullptr == tl_brep)
    return TL_PointFaceRelation::Outside;

  const int rel = tl_brep->PointIsOnFace(face_index, uv, u_tol, v_tol);
  switch (rel)
  {
  case 1:
    return TL_PointFaceRelation::Inside;
  case 2:
    return TL_PointFaceRelation::Boundary;
  default:
    return TL_PointFaceRelation::Outside;
  }
}

namespace {

struct Tri3d
{
  ON_3dPoint a = ON_3dPoint::Origin;
  ON_3dPoint b = ON_3dPoint::Origin;
  ON_3dPoint c = ON_3dPoint::Origin;
  ON_3dVector normal = ON_3dVector::ZeroVector;
  ON_Plane plane;
  ON_BoundingBox bbox;
  bool valid = false;
};

double SignedTetVolume6(const ON_3dPoint& a, const ON_3dPoint& b, const ON_3dPoint& c)
{
  return a.x * (b.y * c.z - b.z * c.y) +
         a.y * (b.z * c.x - b.x * c.z) +
         a.z * (b.x * c.y - b.y * c.x);
}

/** Exact squared distance from point `p` to triangle `(a, b, c)`. */
double PointTriangleDistanceSq(const ON_3dPoint& p, const ON_3dPoint& a,
                               const ON_3dPoint& b, const ON_3dPoint& c)
{
  const ON_3dVector ab = b - a;
  const ON_3dVector ac = c - a;
  const ON_3dVector ap = p - a;
  const double d1 = ON_DotProduct(ab, ap);
  const double d2 = ON_DotProduct(ac, ap);
  if (d1 <= 0.0 && d2 <= 0.0)
    return ap.LengthSquared();

  const ON_3dVector bp = p - b;
  const double d3 = ON_DotProduct(ab, bp);
  const double d4 = ON_DotProduct(ac, bp);
  if (d3 >= 0.0 && d4 <= d3)
    return bp.LengthSquared();

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
  {
    const double v = (d1 - d3 > 1e-30) ? (d1 / (d1 - d3)) : 0.0;
    return (p - (a + v * ab)).LengthSquared();
  }

  const ON_3dVector cp = p - c;
  const double d5 = ON_DotProduct(ab, cp);
  const double d6 = ON_DotProduct(ac, cp);
  if (d6 >= 0.0 && d5 <= d6)
    return cp.LengthSquared();

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
  {
    const double w = (d2 - d6 > 1e-30) ? (d2 / (d2 - d6)) : 0.0;
    return (p - (a + w * ac)).LengthSquared();
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
  {
    const double denom = (d4 - d3) + (d5 - d6);
    const double w = (denom > 1e-30) ? ((d4 - d3) / denom) : 0.0;
    return (p - (b + w * (c - b))).LengthSquared();
  }

  const double sum = va + vb + vc;
  if (sum <= 1e-30)
    return ap.LengthSquared();
  const double inv = 1.0 / sum;
  const double v = vb * inv;
  const double w = vc * inv;
  return (p - (a + v * ab + w * ac)).LengthSquared();
}

/** van Oosterom & Strackee (1983) signed solid angle of triangle `(a,b,c)` seen from `p`. */
double TriangleSolidAngle(const ON_3dPoint& p, const ON_3dPoint& a,
                          const ON_3dPoint& b, const ON_3dPoint& c)
{
  const ON_3dVector r1 = a - p;
  const ON_3dVector r2 = b - p;
  const ON_3dVector r3 = c - p;
  const double l1 = r1.Length();
  const double l2 = r2.Length();
  const double l3 = r3.Length();
  if (l1 <= 1e-15 || l2 <= 1e-15 || l3 <= 1e-15)
    return 0.0;

  const double num = ON_DotProduct(r1, ON_CrossProduct(r2, r3));
  const double den = l1 * l2 * l3 +
                     ON_DotProduct(r1, r2) * l3 +
                     ON_DotProduct(r2, r3) * l1 +
                     ON_DotProduct(r3, r1) * l2;
  return 2.0 * std::atan2(num, den);
}

void ExtractMeshTriangles(const ON_Mesh& mesh, std::vector<Tri3d>& out_tris)
{
  out_tris.clear();
  out_tris.reserve(static_cast<size_t>(mesh.m_F.Count()) * 2);
  const int v_count = mesh.VertexCount();

  auto push_tri = [&](int i0, int i1, int i2) {
    if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= v_count || i1 >= v_count || i2 >= v_count)
      return;
    if (i0 == i1 || i1 == i2 || i2 == i0)
      return;
    Tri3d t;
    t.a = mesh.m_V[i0];
    t.b = mesh.m_V[i1];
    t.c = mesh.m_V[i2];
    t.normal = ON_CrossProduct(t.b - t.a, t.c - t.a);
    if (!t.normal.Unitize())
      return;
    t.plane = ON_Plane(t.a, t.normal);
    t.bbox.Set(t.a, false);
    t.bbox.Set(t.b, true);
    t.bbox.Set(t.c, true);
    t.valid = true;
    out_tris.push_back(t);
  };

  for (int fi = 0; fi < mesh.m_F.Count(); ++fi)
  {
    const ON_MeshFace& f = mesh.m_F[fi];
    push_tri(f.vi[0], f.vi[1], f.vi[2]);
    if (f.IsQuad())
      push_tri(f.vi[0], f.vi[2], f.vi[3]);
  }
}

/**
 * Sutherland-Hodgman clip of a convex polygon against a half-space `sign * plane.ValueAt(p) >= 0`.
 */
std::vector<ON_3dPoint> ClipPolygonByHalfSpace(const std::vector<ON_3dPoint>& poly,
                                               const ON_Plane& plane, double sign, double eps)
{
  std::vector<ON_3dPoint> out;
  const size_t n = poly.size();
  if (n < 3)
    return out;

  std::vector<double> d(n, 0.0);
  bool all_in = true;
  bool all_out = true;
  for (size_t i = 0; i < n; ++i)
  {
    d[i] = sign * plane.DistanceTo(poly[i]);
    if (d[i] < -eps)
      all_in = false;
    if (d[i] > eps)
      all_out = false;
  }
  if (all_in)
    return poly;
  if (all_out)
    return out;

  out.reserve(n + 2);
  for (size_t i = 0; i < n; ++i)
  {
    const size_t j = (i + 1) % n;
    const ON_3dPoint& pi = poly[i];
    const ON_3dPoint& pj = poly[j];
    const double di = d[i];
    const double dj = d[j];
    const bool in_i = (di >= -eps);
    const bool in_j = (dj >= -eps);

    if (in_i)
      out.push_back(pi);

    if ((di > eps && dj < -eps) || (di < -eps && dj > eps))
    {
      const double t = std::fabs(di) / (std::fabs(di) + std::fabs(dj));
      const ON_3dPoint q = (1.0 - t) * pi + t * pj;
      if (out.empty() || out.back().DistanceTo(q) > 1e-11)
        out.push_back(q);
    }
  }

  if (out.size() >= 2 && out.front().DistanceTo(out.back()) <= 1e-11)
    out.pop_back();
  return out;
}

int FindOrAppendVertex(ON_Mesh& mesh, const ON_3dPoint& p, int weld_from_vi, double weld_tol)
{
  const int v_count = mesh.VertexCount();
  if (weld_from_vi >= 0 && weld_from_vi < v_count)
  {
    const double tol_sq = weld_tol * weld_tol;
    for (int i = weld_from_vi; i < v_count; ++i)
    {
      if ((ON_3dPoint(mesh.m_V[i]) - p).LengthSquared() <= tol_sq)
        return i;
    }
  }
  mesh.SetVertex(v_count, p);
  return v_count;
}

void AppendTriangleToMesh(ON_Mesh& mesh, const ON_3dPoint& a, const ON_3dPoint& b,
                          const ON_3dPoint& c, int weld_from_vi = -1, double weld_tol = 1e-7)
{
  if (ON_CrossProduct(b - a, c - a).LengthSquared() <= 1e-24)
    return;
  const int v0 = FindOrAppendVertex(mesh, a, weld_from_vi, weld_tol);
  const int v1 = FindOrAppendVertex(mesh, b, weld_from_vi, weld_tol);
  const int v2 = FindOrAppendVertex(mesh, c, weld_from_vi, weld_tol);
  if (v0 == v1 || v1 == v2 || v2 == v0)
    return;
  mesh.SetTriangle(mesh.m_F.Count(), v0, v1, v2);
}

void AppendPolygonFanToMesh(ON_Mesh& mesh, const std::vector<ON_3dPoint>& poly, bool reverse,
                            int weld_from_vi = -1, double weld_tol = 1e-7)
{
  if (poly.size() < 3)
    return;
  for (size_t i = 1; i + 1 < poly.size(); ++i)
  {
    if (reverse)
      AppendTriangleToMesh(mesh, poly[0], poly[i + 1], poly[i], weld_from_vi, weld_tol);
    else
      AppendTriangleToMesh(mesh, poly[0], poly[i], poly[i + 1], weld_from_vi, weld_tol);
  }
}

/**
 * Chain directed 3D segments `(from, to)` into closed loops by endpoint proximity.
 */
std::vector<std::vector<ON_3dPoint>> ChainDirectedSegments(
  const std::vector<std::pair<ON_3dPoint, ON_3dPoint>>& segments,
  double weld_tol)
{
  std::vector<std::vector<ON_3dPoint>> loops;
  const size_t m = segments.size();
  if (0 == m)
    return loops;

  std::vector<char> used(m, 0);
  for (size_t seed = 0; seed < m; ++seed)
  {
    if (used[seed])
      continue;
    used[seed] = 1;

    std::vector<ON_3dPoint> chain;
    chain.push_back(segments[seed].first);
    ON_3dPoint current = segments[seed].second;
    chain.push_back(current);

    for (size_t guard = 0; guard < m; ++guard)
    {
      if (chain.size() >= 3 && current.DistanceTo(chain.front()) <= weld_tol)
      {
        chain.pop_back();
        loops.push_back(chain);
        break;
      }

      int best = -1;
      double best_d = weld_tol;
      for (size_t k = 0; k < m; ++k)
      {
        if (used[k])
          continue;
        const double d = current.DistanceTo(segments[k].first);
        if (d <= best_d)
        {
          best_d = d;
          best = static_cast<int>(k);
        }
      }
      if (best < 0)
      {
        // Also try reversed segments when two triangles met at a slight angle.
        for (size_t k = 0; k < m; ++k)
        {
          if (used[k])
            continue;
          const double d = current.DistanceTo(segments[k].second);
          if (d <= best_d)
          {
            best_d = d;
            best = static_cast<int>(k);
          }
        }
        if (best >= 0)
        {
          used[static_cast<size_t>(best)] = 1;
          current = segments[static_cast<size_t>(best)].first;
          chain.push_back(current);
          continue;
        }
        break;
      }

      used[static_cast<size_t>(best)] = 1;
      current = segments[static_cast<size_t>(best)].second;
      chain.push_back(current);
    }
  }
  return loops;
}

/**
 * Compute the 3D segment where triangle `t` intersects `plane`; false when `t` does not
 * straddle `plane`.
 */
bool TrianglePlaneSegment(const ON_3dPoint& a, const ON_3dPoint& b, const ON_3dPoint& c,
                          const ON_Plane& plane, double eps, ON_3dPoint& p0, ON_3dPoint& p1)
{
  const ON_3dPoint v[3] = { a, b, c };
  const double d[3] = { plane.DistanceTo(a), plane.DistanceTo(b), plane.DistanceTo(c) };
  if ((d[0] >= -eps && d[1] >= -eps && d[2] >= -eps) ||
      (d[0] <=  eps && d[1] <=  eps && d[2] <=  eps))
    return false;

  ON_3dPoint pts[4];
  int count = 0;
  for (int i = 0; i < 3; ++i)
  {
    const int j = (i + 1) % 3;
    if (std::fabs(d[i]) <= eps)
      pts[count++] = v[i];
    if ((d[i] > eps && d[j] < -eps) || (d[i] < -eps && d[j] > eps))
    {
      const double t = std::fabs(d[i]) / (std::fabs(d[i]) + std::fabs(d[j]));
      pts[count++] = (1.0 - t) * v[i] + t * v[j];
    }
  }
  if (count < 2)
    return false;

  p0 = pts[0];
  p1 = pts[1];
  for (int k = 1; k < count; ++k)
  {
    if (pts[k].DistanceTo(p0) > p1.DistanceTo(p0))
      p1 = pts[k];
  }
  return p0.DistanceTo(p1) > eps;
}

bool PolygonStraddlesAndHitsTriangle(const std::vector<ON_3dPoint>& poly, const Tri3d& tri,
                                     double eps)
{
  if (poly.size() < 3)
    return false;

  ON_BoundingBox pbox;
  pbox.Set(poly[0], false);
  bool has_pos = false;
  bool has_neg = false;
  for (const ON_3dPoint& p : poly)
  {
    pbox.Set(p, true);
    const double d = tri.plane.DistanceTo(p);
    if (d > eps)
      has_pos = true;
    else if (d < -eps)
      has_neg = true;
  }
  if (!has_pos || !has_neg)
    return false;
  if (pbox.MinimumDistanceTo(tri.bbox) > eps)
    return false;

  const ON_3dVector n_poly = ON_CrossProduct(poly[1] - poly[0], poly[2] - poly[0]);
  ON_3dVector n_unit = n_poly;
  if (!n_unit.Unitize())
    return false;
  const ON_Plane poly_plane(poly[0], n_unit);

  ON_3dPoint b0, b1;
  if (!TrianglePlaneSegment(tri.a, tri.b, tri.c, poly_plane, eps, b0, b1))
    return false;

  // Clip the poly by tri.plane to obtain the crossing segment `a0..a1`.
  ON_3dPoint a0 = ON_3dPoint::Origin;
  ON_3dPoint a1 = ON_3dPoint::Origin;
  bool have_a0 = false;
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i)
  {
    const size_t j = (i + 1) % n;
    const double di = tri.plane.DistanceTo(poly[i]);
    const double dj = tri.plane.DistanceTo(poly[j]);
    if (std::fabs(di) <= eps)
    {
      if (!have_a0) { a0 = poly[i]; have_a0 = true; }
      else if (poly[i].DistanceTo(a0) > a1.DistanceTo(a0)) a1 = poly[i];
    }
    if ((di > eps && dj < -eps) || (di < -eps && dj > eps))
    {
      const double t = std::fabs(di) / (std::fabs(di) + std::fabs(dj));
      const ON_3dPoint q = (1.0 - t) * poly[i] + t * poly[j];
      if (!have_a0) { a0 = q; a1 = q; have_a0 = true; }
      else if (q.DistanceTo(a0) > a1.DistanceTo(a0)) a1 = q;
    }
  }
  if (!have_a0 || a0.DistanceTo(a1) <= eps)
    return false;

  ON_3dVector dir = a1 - a0;
  if (!dir.Unitize())
    return false;
  const double sa0 = 0.0;
  const double sa1 = ON_DotProduct(a1 - a0, dir);
  const double sb0 = ON_DotProduct(b0 - a0, dir);
  const double sb1 = ON_DotProduct(b1 - a0, dir);
  const double lo = std::max(std::min(sa0, sa1), std::min(sb0, sb1));
  const double hi = std::min(std::max(sa0, sa1), std::max(sb0, sb1));
  return hi >= lo - 10.0 * eps;
}

} // anonymous namespace

TL_FUNCTION double TL_MeshSignedVolume(const ON_Mesh& mesh)
{
  double v6 = 0.0;
  const int v_count = mesh.VertexCount();
  for (int fi = 0; fi < mesh.m_F.Count(); ++fi)
  {
    const ON_MeshFace& f = mesh.m_F[fi];
    if (f.vi[0] < 0 || f.vi[1] < 0 || f.vi[2] < 0 ||
        f.vi[0] >= v_count || f.vi[1] >= v_count || f.vi[2] >= v_count)
      continue;
    const ON_3dPoint& a = mesh.m_V[f.vi[0]];
    const ON_3dPoint& b = mesh.m_V[f.vi[1]];
    const ON_3dPoint& c = mesh.m_V[f.vi[2]];
    v6 += SignedTetVolume6(a, b, c);
    if (f.IsQuad() && f.vi[3] >= 0 && f.vi[3] < v_count)
      v6 += SignedTetVolume6(a, c, mesh.m_V[f.vi[3]]);
  }
  return v6 / 6.0;
}

TL_FUNCTION double TL_MeshWindingNumber(const ON_Mesh& mesh, const ON_3dPoint& point)
{
  double total_omega = 0.0;
  const int v_count = mesh.VertexCount();
  for (int fi = 0; fi < mesh.m_F.Count(); ++fi)
  {
    const ON_MeshFace& f = mesh.m_F[fi];
    if (f.vi[0] < 0 || f.vi[1] < 0 || f.vi[2] < 0 ||
        f.vi[0] >= v_count || f.vi[1] >= v_count || f.vi[2] >= v_count)
      continue;
    const ON_3dPoint& a = mesh.m_V[f.vi[0]];
    const ON_3dPoint& b = mesh.m_V[f.vi[1]];
    const ON_3dPoint& c = mesh.m_V[f.vi[2]];
    total_omega += TriangleSolidAngle(point, a, b, c);
    if (f.IsQuad() && f.vi[3] >= 0 && f.vi[3] < v_count)
      total_omega += TriangleSolidAngle(point, a, c, mesh.m_V[f.vi[3]]);
  }
  return total_omega / (4.0 * ON_PI);
}

TL_FUNCTION TL_PointFaceRelation TL_MeshContainsPoint(
  const ON_Mesh& mesh,
  const ON_3dPoint& point,
  double tolerance
)
{
  if (mesh.m_F.Count() <= 0 || mesh.VertexCount() <= 0)
    return TL_PointFaceRelation::Outside;

  const double tol = std::max(tolerance, 1e-12);
  const double tol_sq = tol * tol;
  const int v_count = mesh.VertexCount();

  for (int fi = 0; fi < mesh.m_F.Count(); ++fi)
  {
    const ON_MeshFace& f = mesh.m_F[fi];
    if (f.vi[0] < 0 || f.vi[1] < 0 || f.vi[2] < 0 ||
        f.vi[0] >= v_count || f.vi[1] >= v_count || f.vi[2] >= v_count)
      continue;
    const ON_3dPoint& a = mesh.m_V[f.vi[0]];
    const ON_3dPoint& b = mesh.m_V[f.vi[1]];
    const ON_3dPoint& c = mesh.m_V[f.vi[2]];
    if (PointTriangleDistanceSq(point, a, b, c) <= tol_sq)
      return TL_PointFaceRelation::Boundary;
    if (f.IsQuad() && f.vi[3] >= 0 && f.vi[3] < v_count)
    {
      if (PointTriangleDistanceSq(point, a, c, mesh.m_V[f.vi[3]]) <= tol_sq)
        return TL_PointFaceRelation::Boundary;
    }
  }

  const double w = std::fabs(TL_MeshWindingNumber(mesh, point));
  return (w >= 0.5) ? TL_PointFaceRelation::Inside : TL_PointFaceRelation::Outside;
}

TL_FUNCTION int TL_SliceMeshWithPlane(
  const ON_Mesh& mesh,
  const ON_Plane& plane,
  bool cap_section,
  ON_Mesh* out_positive,
  ON_Mesh* out_negative,
  std::vector<std::vector<ON_3dPoint>>* out_section_loops
)
{
  if (nullptr != out_positive)
    out_positive->Destroy();
  if (nullptr != out_negative)
    out_negative->Destroy();
  if (nullptr != out_section_loops)
    out_section_loops->clear();

  if (!plane.IsValid() || mesh.m_F.Count() <= 0)
    return 0;

  std::vector<Tri3d> tris;
  ExtractMeshTriangles(mesh, tris);
  if (tris.empty())
    return 0;

  ON_BoundingBox bbox;
  mesh.GetBoundingBox(bbox);
  const double diag = bbox.IsValid() ? std::max(1.0, bbox.Diagonal().Length()) : 1.0;
  const double eps = 1e-10 * diag;
  const double weld_tol = 1e-5 * diag;
  const ON_3dVector plane_n = plane.Normal();

  std::vector<std::pair<ON_3dPoint, ON_3dPoint>> section_segments;

  for (const Tri3d& tri : tris)
  {
    const std::vector<ON_3dPoint> base = { tri.a, tri.b, tri.c };
    if (nullptr != out_positive)
    {
      const std::vector<ON_3dPoint> pos = ClipPolygonByHalfSpace(base, plane, +1.0, eps);
      AppendPolygonFanToMesh(*out_positive, pos, false, /*weld_from_vi=*/0, weld_tol);
    }
    if (nullptr != out_negative)
    {
      const std::vector<ON_3dPoint> neg = ClipPolygonByHalfSpace(base, plane, -1.0, eps);
      AppendPolygonFanToMesh(*out_negative, neg, false, /*weld_from_vi=*/0, weld_tol);
    }

    ON_3dPoint q0, q1;
    if (TrianglePlaneSegment(tri.a, tri.b, tri.c, plane, eps, q0, q1))
    {
      // Orient `(q0 -> q1)` along `tri.normal x plane_n` so a loop seen from `+plane_n`
      // runs counter-clockwise (right-hand rule around `+plane_n`, cap normal `+plane_n`
      // on the negative half and `-plane_n` on the positive half).
      const ON_3dVector dir = ON_CrossProduct(tri.normal, plane_n);
      if (ON_DotProduct(q1 - q0, dir) >= 0.0)
        section_segments.push_back({ q0, q1 });
      else
        section_segments.push_back({ q1, q0 });
    }
  }

  std::vector<std::vector<ON_3dPoint>> loops = ChainDirectedSegments(section_segments, weld_tol);

  if (cap_section)
  {
    for (const std::vector<ON_3dPoint>& loop : loops)
    {
      if (loop.size() < 3)
        continue;
      ON_3dPoint centroid = ON_3dPoint::Origin;
      for (const ON_3dPoint& p : loop)
        centroid += p;
      centroid = centroid / static_cast<double>(loop.size());

      // Verify loop normal via Newell's method against `+plane_n`.
      ON_3dVector newell = ON_3dVector::ZeroVector;
      for (size_t i = 0; i < loop.size(); ++i)
      {
        const ON_3dPoint& p = loop[i];
        const ON_3dPoint& q = loop[(i + 1) % loop.size()];
        newell.x += (p.y - q.y) * (p.z + q.z);
        newell.y += (p.z - q.z) * (p.x + q.x);
        newell.z += (p.x - q.x) * (p.y + q.y);
      }
      const bool aligned_with_plus_n = (ON_DotProduct(newell, plane_n) >= 0.0);
      const int neg_cap_start = (nullptr != out_negative) ? out_negative->VertexCount() : 0;
      const int pos_cap_start = (nullptr != out_positive) ? out_positive->VertexCount() : 0;

      for (size_t i = 0; i < loop.size(); ++i)
      {
        const ON_3dPoint& a = loop[i];
        const ON_3dPoint& b = loop[(i + 1) % loop.size()];
        // Negative half-space needs outward cap normal `+plane_n`;
        // Positive half-space needs outward cap normal `-plane_n`.
        // Welding starts at `neg_cap_start` / `pos_cap_start` so internal fan spokes are
        // welded (0 deg dihedral -> no spurious feature edges) while the outer rim stays
        // split from the smooth shell (hard crease around the cut face).
        if (nullptr != out_negative)
        {
          if (aligned_with_plus_n)
            AppendTriangleToMesh(*out_negative, centroid, a, b, neg_cap_start, weld_tol);
          else
            AppendTriangleToMesh(*out_negative, centroid, b, a, neg_cap_start, weld_tol);
        }
        if (nullptr != out_positive)
        {
          if (aligned_with_plus_n)
            AppendTriangleToMesh(*out_positive, centroid, b, a, pos_cap_start, weld_tol);
          else
            AppendTriangleToMesh(*out_positive, centroid, a, b, pos_cap_start, weld_tol);
        }
      }
    }
  }

  if (nullptr != out_positive && out_positive->m_F.Count() > 0)
  {
    out_positive->ComputeFaceNormals();
    out_positive->ComputeVertexNormals();
  }
  if (nullptr != out_negative && out_negative->m_F.Count() > 0)
  {
    out_negative->ComputeFaceNormals();
    out_negative->ComputeVertexNormals();
  }
  if (nullptr != out_section_loops)
    *out_section_loops = loops;

  return static_cast<int>(loops.size());
}

TL_FUNCTION bool TL_MeshBoolean(
  const ON_Mesh& mesh_a,
  const ON_Mesh& mesh_b,
  TL_BooleanOp op,
  ON_Mesh& out_mesh,
  double tolerance
)
{
  out_mesh.Destroy();

  std::vector<Tri3d> tris_a;
  std::vector<Tri3d> tris_b;
  ExtractMeshTriangles(mesh_a, tris_a);
  ExtractMeshTriangles(mesh_b, tris_b);
  if (tris_a.empty() || tris_b.empty())
    return false;

  const double eps = std::max(tolerance, 1e-10);

  auto process_side = [&](const std::vector<Tri3d>& source_tris,
                          const std::vector<Tri3d>& cutter_tris,
                          const ON_Mesh& cutter_mesh,
                          bool keep_inside,
                          bool flip_orientation) {
    for (const Tri3d& src : source_tris)
    {
      // Collect candidate cutter triangles whose bounding boxes overlap `src.bbox`.
      std::vector<size_t> candidates;
      for (size_t j = 0; j < cutter_tris.size(); ++j)
      {
        if (src.bbox.MinimumDistanceTo(cutter_tris[j].bbox) <= eps)
          candidates.push_back(j);
      }

      std::vector<std::vector<ON_3dPoint>> fragments;
      fragments.push_back({ src.a, src.b, src.c });

      for (size_t idx : candidates)
      {
        const Tri3d& cut = cutter_tris[idx];
        std::vector<std::vector<ON_3dPoint>> next_fragments;
        next_fragments.reserve(fragments.size() * 2);

        for (const std::vector<ON_3dPoint>& poly : fragments)
        {
          if (!PolygonStraddlesAndHitsTriangle(poly, cut, eps))
          {
            next_fragments.push_back(poly);
            continue;
          }
          std::vector<ON_3dPoint> pos = ClipPolygonByHalfSpace(poly, cut.plane, +1.0, eps);
          std::vector<ON_3dPoint> neg = ClipPolygonByHalfSpace(poly, cut.plane, -1.0, eps);
          if (pos.size() >= 3)
            next_fragments.push_back(std::move(pos));
          if (neg.size() >= 3)
            next_fragments.push_back(std::move(neg));
        }
        fragments.swap(next_fragments);
      }

      for (const std::vector<ON_3dPoint>& poly : fragments)
      {
        if (poly.size() < 3)
          continue;
        for (size_t k = 1; k + 1 < poly.size(); ++k)
        {
          const ON_3dPoint& p0 = poly[0];
          const ON_3dPoint& p1 = poly[k];
          const ON_3dPoint& p2 = poly[k + 1];
          if (ON_CrossProduct(p1 - p0, p2 - p0).LengthSquared() <= 1e-22)
            continue;
          const ON_3dPoint centroid = (p0 + p1 + p2) / 3.0;
          // Perturb slightly along the triangle's own normal when coplanar with the cutter.
          const double w = std::fabs(TL_MeshWindingNumber(cutter_mesh, centroid));
          const bool is_inside = (w >= 0.5);
          if (is_inside == keep_inside)
          {
            if (flip_orientation)
              AppendTriangleToMesh(out_mesh, p0, p2, p1);
            else
              AppendTriangleToMesh(out_mesh, p0, p1, p2);
          }
        }
      }
    }
  };

  switch (op)
  {
    case TL_BooleanOp::Union:
      process_side(tris_a, tris_b, mesh_b, /*keep_inside=*/false, /*flip=*/false);
      process_side(tris_b, tris_a, mesh_a, /*keep_inside=*/false, /*flip=*/false);
      break;
    case TL_BooleanOp::Intersection:
      process_side(tris_a, tris_b, mesh_b, /*keep_inside=*/true, /*flip=*/false);
      process_side(tris_b, tris_a, mesh_a, /*keep_inside=*/true, /*flip=*/false);
      break;
    case TL_BooleanOp::Difference:
      process_side(tris_a, tris_b, mesh_b, /*keep_inside=*/false, /*flip=*/false);
      process_side(tris_b, tris_a, mesh_a, /*keep_inside=*/true, /*flip=*/true);
      break;
  }

  if (out_mesh.m_F.Count() > 0)
  {
    out_mesh.ComputeFaceNormals();
    out_mesh.ComputeVertexNormals();
  }
  return true;
}
