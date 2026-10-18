#include "opentl/meshing/tl_meshing.h"

#include <algorithm>
#include <cmath>

namespace {

/**
 * True when `face` references mesh vertex `v` on any of its corners
 * (3 for a triangle, 4 for a quad).
 */
bool FaceHasVertex(const ON_MeshFace& face, int v)
{
  const int slots = face.IsQuad() ? 4 : 3;
  for (int k = 0; k < slots; ++k)
  {
    if (face.vi[k] == v)
      return true;
  }
  return false;
}

/**
 * Dual-loop match of one topological vertex against the two incident faces
 * (tl.dll 0x18033be84..0x18033bf6c).  The value recorded is the **mesh vertex
 * index** that was found in each face; each hit overwrites the recorded value,
 * so the last matching mesh vertex wins -- exactly as the binary does.
 */
void MatchTopologicalVertex(
  const ON_MeshTopologyVertex& topv,
  const ON_MeshFace& face_a,
  const ON_MeshFace& face_b,
  int& hit_a,
  int& hit_b
)
{
  hit_a = -1;
  hit_b = -1;
  for (int k = 0; k < topv.m_v_count; ++k)
  {
    const int v = topv.m_vi[k];
    if (FaceHasVertex(face_a, v))
      hit_a = v;
    if (FaceHasVertex(face_b, v))
      hit_b = v;
  }
}

} // anonymous namespace

TL_FUNCTION bool TL_IsMeshEdgeUnwelded(int edge_index, const ON_Mesh* mesh)
{
  if (nullptr == mesh)
    return false;

  const ON_MeshTopology& top = mesh->Topology();
  if (edge_index < 0 || edge_index >= top.m_tope.Count())
    return false;

  const ON_MeshTopologyEdge& tope = top.m_tope[edge_index];

  // tl.dll 0x18033bdc1: `cmp dword ptr [rax+0x8], 2` / `jne false`.
  // The binary requires EXACTLY two incident faces; non-manifold edges
  // (3+ faces) are not "unwelded" and return false.  The previous
  // `m_topf_count <= 1` reading was wrong.
  if (2 != tope.m_topf_count)
    return false;

  const int topv0 = tope.m_topvi[0];
  const int topv1 = tope.m_topvi[1];
  const int topv_count = top.m_topv.Count();
  if (topv0 < 0 || topv0 >= topv_count || topv1 < 0 || topv1 >= topv_count)
    return false;

  const int fi0 = tope.m_topfi[0];
  const int fi1 = tope.m_topfi[1];
  if (fi0 < 0 || fi1 < 0 || fi0 >= mesh->m_F.Count() || fi1 >= mesh->m_F.Count())
    return false;

  const ON_MeshFace& face0 = mesh->m_F[fi0];
  const ON_MeshFace& face1 = mesh->m_F[fi1];

  // The binary has NO m_v_count fast path: it always runs both dual loops.
  int a0 = -1;
  int a1 = -1;
  int b0 = -1;
  int b1 = -1;
  MatchTopologicalVertex(top.m_topv[topv0], face0, face1, a0, a1);
  MatchTopologicalVertex(top.m_topv[topv1], face0, face1, b0, b1);

  // tl.dll 0x18033bf73..0x18033bf85: four inequality tests; any equality -> false.
  if (a0 == a1)
    return false;
  if (a0 == b1)
    return false;
  if (b0 == a1)
    return false;
  if (b0 == b1)
    return false;
  return true;
}

TL_FUNCTION ON_Mesh* TL_MeshSurfaceGrid(
  const ON_Surface& surface,
  int u_divisions,
  int v_divisions,
  ON_Mesh* result_mesh
)
{
  const int u_divs = std::max(1, u_divisions);
  const int v_divs = std::max(1, v_divisions);

  const ON_Interval u_dom = surface.Domain(0);
  const ON_Interval v_dom = surface.Domain(1);
  if (!u_dom.IsIncreasing() || !v_dom.IsIncreasing())
    return nullptr;

  const int u_verts = u_divs + 1;
  const int v_verts = v_divs + 1;
  const int total_verts = u_verts * v_verts;
  const int total_faces = u_divs * v_divs;

  ON_Mesh* mesh = (nullptr != result_mesh)
    ? result_mesh
    : new ON_Mesh(total_faces, total_verts, true, true);

  if (nullptr != result_mesh)
  {
    mesh->Destroy();
    mesh->ReserveVertexCapacity(static_cast<size_t>(total_verts));
  }

  mesh->m_dV.SetCapacity(total_verts);
  mesh->m_dV.SetCount(total_verts);
  mesh->m_V.SetCapacity(total_verts);
  mesh->m_V.SetCount(total_verts);
  mesh->m_N.SetCapacity(total_verts);
  mesh->m_N.SetCount(total_verts);
  mesh->m_S.SetCapacity(total_verts);
  mesh->m_S.SetCount(total_verts);
  mesh->m_T.SetCapacity(total_verts);
  mesh->m_T.SetCount(total_verts);
  mesh->m_F.SetCapacity(total_faces);
  mesh->m_F.SetCount(total_faces);

  mesh->m_srf_domain[0] = u_dom;
  mesh->m_srf_domain[1] = v_dom;

  for (int iv = 0; iv < v_verts; ++iv)
  {
    const double norm_v = static_cast<double>(iv) / static_cast<double>(v_divs);
    const double v = v_dom.ParameterAt(norm_v);
    for (int iu = 0; iu < u_verts; ++iu)
    {
      const double norm_u = static_cast<double>(iu) / static_cast<double>(u_divs);
      const double u = u_dom.ParameterAt(norm_u);
      const int vi = iv * u_verts + iu;

      ON_3dPoint pt = ON_3dPoint::Origin;
      ON_3dVector nrm = ON_3dVector::ZAxis;
      if (!surface.EvNormal(u, v, pt, nrm))
      {
        pt = surface.PointAt(u, v);
        nrm = surface.NormalAt(u, v);
        if (!nrm.Unitize())
          nrm = ON_3dVector::ZAxis;
      }

      mesh->m_dV[vi] = pt;
      mesh->m_V[vi] = ON_3fPoint(pt);
      mesh->m_N[vi] = ON_3fVector(nrm);
      mesh->m_S[vi] = ON_2dPoint(u, v);
      mesh->m_T[vi] = ON_2fPoint(static_cast<float>(norm_u), static_cast<float>(norm_v));
    }
  }

  int fi = 0;
  for (int iv = 0; iv < v_divs; ++iv)
  {
    for (int iu = 0; iu < u_divs; ++iu)
    {
      const int v00 = iv * u_verts + iu;
      const int v10 = iv * u_verts + (iu + 1);
      const int v11 = (iv + 1) * u_verts + (iu + 1);
      const int v01 = (iv + 1) * u_verts + iu;
      mesh->SetQuad(fi++, v00, v10, v11, v01);
    }
  }

  mesh->ComputeFaceNormals();
  mesh->Compact();
  return mesh;
}
