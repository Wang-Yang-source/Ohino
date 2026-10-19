#include "opentl/subd/tl_subd.h"

#include <algorithm>
#include <cmath>

TL_FUNCTION ON_SubD* TL_CreateSubDBox(
  const ON_BoundingBox& bbox,
  unsigned int subdivision_levels,
  ON_SubD* result
)
{
  if (!bbox.IsValid())
    return nullptr;

  // CRITICAL: `ON_SubD::CreateSubDBox` expects a right-handed quad-prism corner layout:
  //   xdir = corners[1] - corners[0], ydir = corners[3] - corners[0], zdir = corners[4] - corners[0].
  // `ON_BoundingBox::GetCorners` instead enumerates `(i,j,k)` with `z` in the inner loop
  // (`corners[1] = (min,min,max)`, `corners[3] = (min,max,max)`), which fed a left-handed
  // sheared basis `(+Z, +Y+Z, +X)` into `CreateSubDBox` and produced an inside-out skewed box!
  const ON_3dPoint& mn = bbox.m_min;
  const ON_3dPoint& mx = bbox.m_max;
  const ON_3dPoint corners[8] = {
    ON_3dPoint(mn.x, mn.y, mn.z), // 0: bottom (-x, -y, -z)
    ON_3dPoint(mx.x, mn.y, mn.z), // 1: bottom (+x, -y, -z) -> xdir = +X
    ON_3dPoint(mx.x, mx.y, mn.z), // 2: bottom (+x, +y, -z)
    ON_3dPoint(mn.x, mx.y, mn.z), // 3: bottom (-x, +y, -z) -> ydir = +Y
    ON_3dPoint(mn.x, mn.y, mx.z), // 4: top    (-x, -y, +z) -> zdir = +Z
    ON_3dPoint(mx.x, mn.y, mx.z), // 5: top    (+x, -y, +z)
    ON_3dPoint(mx.x, mx.y, mx.z), // 6: top    (+x, +y, +z)
    ON_3dPoint(mn.x, mx.y, mx.z)  // 7: top    (-x, +y, +z)
  };

  if (nullptr != result)
    result->Clear();

  ON_SubD* subd = ON_SubD::CreateSubDBox(
    corners,
    ON_SubDEdgeTag::Smooth,
    1,
    1,
    1,
    result
  );
  if (nullptr == subd)
    return nullptr;

  subd->UpdateAllTagsAndSectorCoefficients(false);
  if (subdivision_levels > 0)
  {
    subd->GlobalSubdivide(subdivision_levels);
  }
  return subd;
}

TL_FUNCTION ON_Mesh* TL_MeshFromSubD(
  const ON_SubD& subd,
  unsigned int extra_subdivision_levels,
  ON_Mesh* result
)
{
  const ON_SubD* source = &subd;
  ON_SubD subdivided_copy;
  if (extra_subdivision_levels > 0)
  {
    subdivided_copy = subd;
    subdivided_copy.UpdateAllTagsAndSectorCoefficients(false);
    subdivided_copy.GlobalSubdivide(extra_subdivision_levels);
    source = &subdivided_copy;
  }

  ON_Mesh* mesh = source->GetControlNetMesh(result, ON_SubDGetControlNetMeshPriority::Geometry);
  if (nullptr == mesh)
    return nullptr;

  if (mesh->m_F.Count() <= 0 || mesh->VertexCount() <= 0)
  {
    if (mesh != result)
      delete mesh;
    return nullptr;
  }

  mesh->ComputeFaceNormals();
  mesh->ComputeVertexNormals();
  return mesh;
}

TL_FUNCTION ON_SubD* TL_CreateSubDFromMesh(
  const ON_Mesh& control_net_mesh,
  unsigned int subdivision_levels,
  bool crease_naked_edges,
  ON_SubD* result
)
{
  if (control_net_mesh.VertexCount() <= 0 || control_net_mesh.m_F.Count() <= 0)
    return nullptr;

  ON_SubDFromMeshParameters params;
  params.SetInteriorCreaseOption(ON_SubDFromMeshParameters::InteriorCreaseOption::None);
  params.SetConvexCornerOption(
    crease_naked_edges ? ON_SubDFromMeshParameters::ConvexCornerOption::AtMeshCorner
                       : ON_SubDFromMeshParameters::ConvexCornerOption::None);

  if (nullptr != result)
    result->Clear();

  ON_SubD* subd = ON_SubD::CreateFromMesh(&control_net_mesh, &params, result);
  if (nullptr == subd)
    return nullptr;

  subd->UpdateAllTagsAndSectorCoefficients(false);
  if (subdivision_levels > 0)
    subd->GlobalSubdivide(subdivision_levels);
  return subd;
}

TL_FUNCTION ON_SubD* TL_CreateSubDCylinder(
  const ON_Circle& base_circle,
  double height,
  unsigned int around_faces,
  unsigned int vertical_faces,
  unsigned int subdivision_levels,
  ON_SubD* result
)
{
  if (!base_circle.IsValid() || std::fabs(height) <= 1e-9)
    return nullptr;

  const int n_around = std::clamp(static_cast<int>(around_faces), 4, 64);
  const int n_vert   = std::clamp(static_cast<int>(vertical_faces), 1, 32);
  const double R = base_circle.Radius();
  // Compensate control-net shrinkage of cubic Catmull-Clark circle so limit radius ~= R
  const double r_scale = 1.0 / std::cos(ON_PI / static_cast<double>(n_around));
  const double r_cv = R * (1.0 + 0.35 * (r_scale - 1.0));
  const ON_Plane& pl = base_circle.plane;
  const ON_3dVector z_vec = pl.zaxis * height;

  ON_Mesh cage;
  // Ring vertices: (n_vert + 1) rings of `n_around` vertices
  for (int iv = 0; iv <= n_vert; ++iv)
  {
    const double t = static_cast<double>(iv) / static_cast<double>(n_vert);
    const ON_3dPoint ring_o = pl.origin + z_vec * t;
    for (int ia = 0; ia < n_around; ++ia)
    {
      const double ang = 2.0 * ON_PI * static_cast<double>(ia) / static_cast<double>(n_around);
      const ON_3dPoint pt = ring_o + pl.xaxis * (r_cv * std::cos(ang)) + pl.yaxis * (r_cv * std::sin(ang));
      cage.m_V.Append(ON_3fPoint(pt));
    }
  }
  const int bot_center = cage.m_V.Count();
  cage.m_V.Append(ON_3fPoint(pl.origin));
  const int top_center = cage.m_V.Count();
  cage.m_V.Append(ON_3fPoint(pl.origin + z_vec));

  // Side quad faces (outward normal)
  for (int iv = 0; iv < n_vert; ++iv)
  {
    const int r0 = iv * n_around;
    const int r1 = (iv + 1) * n_around;
    for (int ia = 0; ia < n_around; ++ia)
    {
      const int ia_next = (ia + 1) % n_around;
      ON_MeshFace q;
      if (height >= 0.0)
      {
        q.vi[0] = r0 + ia; q.vi[1] = r0 + ia_next;
        q.vi[2] = r1 + ia_next; q.vi[3] = r1 + ia;
      }
      else
      {
        q.vi[0] = r0 + ia; q.vi[1] = r1 + ia;
        q.vi[2] = r1 + ia_next; q.vi[3] = r0 + ia_next;
      }
      cage.m_F.Append(q);
    }
  }
  // Bottom cap triangles (-z outward) and top cap triangles (+z outward)
  const int top_ring = n_vert * n_around;
  for (int ia = 0; ia < n_around; ++ia)
  {
    const int ia_next = (ia + 1) % n_around;
    ON_MeshFace fb, ft;
    if (height >= 0.0)
    {
      fb.vi[0] = bot_center; fb.vi[1] = ia_next; fb.vi[2] = ia; fb.vi[3] = ia;
      ft.vi[0] = top_center; ft.vi[1] = top_ring + ia; ft.vi[2] = top_ring + ia_next; ft.vi[3] = top_ring + ia_next;
    }
    else
    {
      fb.vi[0] = bot_center; fb.vi[1] = ia; fb.vi[2] = ia_next; fb.vi[3] = ia_next;
      ft.vi[0] = top_center; ft.vi[1] = top_ring + ia_next; ft.vi[2] = top_ring + ia; ft.vi[3] = top_ring + ia;
    }
    cage.m_F.Append(fb);
    cage.m_F.Append(ft);
  }

  ON_SubD* subd = TL_CreateSubDFromMesh(cage, subdivision_levels, false, result);
  if (nullptr == subd)
    return nullptr;

  // Catmull-Clark shrinks the control cage; measure the limit radius and rescale the cage
  // radially (axial scale is preserved so `height` stays exact).
  {
    ON_Mesh* probe = TL_MeshFromSubD(*subd, 0, nullptr);
    if (nullptr != probe)
    {
      double r_measured = 0.0;
      for (int i = 0; i < probe->VertexCount(); ++i)
      {
        const ON_3dPoint p(probe->m_V[i]);
        ON_3dVector rel = p - pl.origin;
        rel -= pl.zaxis * ON_DotProduct(rel, pl.zaxis);
        r_measured = std::max(r_measured, rel.Length());
      }
      delete probe;
      if (r_measured > 1e-9)
      {
        const double k = R / r_measured;
        ON_SubDVertexIterator vit(*subd);
        for (ON_SubDVertex* v = const_cast<ON_SubDVertex*>(vit.FirstVertex());
             nullptr != v;
             v = const_cast<ON_SubDVertex*>(vit.NextVertex()))
        {
          const ON_3dPoint p(v->m_P[0], v->m_P[1], v->m_P[2]);
          ON_3dVector rel = p - pl.origin;
          const double axial = ON_DotProduct(rel, pl.zaxis);
          ON_3dVector radial = rel - pl.zaxis * axial;
          const ON_3dPoint q = pl.origin + pl.zaxis * axial + radial * k;
          v->m_P[0] = q.x; v->m_P[1] = q.y; v->m_P[2] = q.z;
        }
        subd->UpdateAllTagsAndSectorCoefficients(false);
        subd->ClearEvaluationCache();
      }
    }
  }
  return subd;
}
TL_FUNCTION ON_SubD* TL_CreateSubDSphere(
  const ON_3dPoint& center,
  double radius,
  unsigned int subdivision_levels,
  ON_SubD* result
)
{
  if (radius <= 1e-9)
    return nullptr;

  // Start from a cube control net inscribed/scaled so Catmull-Clark limit surface has radius `radius`
  const double a = radius * 0.96;
  const ON_BoundingBox bb(center - ON_3dVector(a, a, a), center + ON_3dVector(a, a, a));
  ON_SubD* subd = TL_CreateSubDBox(bb, std::max(1u, subdivision_levels), result);
  if (nullptr == subd)
    return nullptr;

  // Spherize all control-net vertices onto the exact sphere of `radius` around `center`
  ON_SubDVertexIterator vit(*subd);
  for (ON_SubDVertex* v = const_cast<ON_SubDVertex*>(vit.FirstVertex());
       nullptr != v;
       v = const_cast<ON_SubDVertex*>(vit.NextVertex()))
  {
    ON_3dVector dir(v->m_P[0] - center.x, v->m_P[1] - center.y, v->m_P[2] - center.z);
    if (dir.Unitize())
    {
      v->m_P[0] = center.x + dir.x * radius;
      v->m_P[1] = center.y + dir.y * radius;
      v->m_P[2] = center.z + dir.z * radius;
    }
  }
  subd->UpdateAllTagsAndSectorCoefficients(false);
  subd->ClearEvaluationCache();
  return subd;
}
