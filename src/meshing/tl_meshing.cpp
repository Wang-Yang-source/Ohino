#include "opentl/meshing/tl_meshing.h"

#include <algorithm>
#include <cmath>

TL_FUNCTION bool TL_IsMeshEdgeUnwelded(int edge_index, const ON_Mesh* mesh)
{
  if (nullptr == mesh)
    return false;

  const ON_MeshTopology& top = mesh->Topology();
  if (edge_index < 0 || edge_index >= top.m_tope.Count())
    return false;

  const ON_MeshTopologyEdge& tope = top.m_tope[edge_index];
  if (tope.m_topf_count <= 1)
    return false;

  const int topv0 = tope.m_topvi[0];
  const int topv1 = tope.m_topvi[1];
  const int topv_count = top.m_topv.Count();
  if (topv0 < 0 || topv0 >= topv_count || topv1 < 0 || topv1 >= topv_count)
    return false;

  // Fast path: if both topological vertices reference at most 1 mesh vertex,
  // all incident faces must share the exact same mesh vertex indices.
  if (top.m_topv[topv0].m_v_count <= 1 && top.m_topv[topv1].m_v_count <= 1)
    return false;

  const int face_count = mesh->m_F.Count();
  const int topf_count = top.m_topf.Count();
  const int vertex_count = mesh->VertexCount();
  const int map_count = top.m_topv_map.Count();

  int first_v0 = -1;
  int first_v1 = -1;
  bool have_first_pair = false;

  // Slow path: inspect each incident face in tope.m_topfi[k]
  for (int k = 0; k < tope.m_topf_count; ++k)
  {
    const int fi = tope.m_topfi[k];
    if (fi < 0 || fi >= face_count || fi >= topf_count)
      continue;

    const ON_MeshTopologyFace& topf = top.m_topf[fi];
    const ON_MeshFace& mf = mesh->m_F[fi];
    const int side_count = topf.IsTriangle() ? 3 : 4;

    for (int side = 0; side < side_count; ++side)
    {
      if (topf.m_topei[side] != edge_index)
        continue;

      const int v_start = mf.vi[(side + side_count - 1) % side_count];
      const int v_end = mf.vi[side];
      if (v_start < 0 || v_start >= vertex_count || v_end < 0 || v_end >= vertex_count)
        continue;

      // Orient the pair along the topological edge direction using m_reve[side]
      int v0 = (topf.m_reve[side] != 0) ? v_end : v_start;
      int v1 = (topf.m_reve[side] != 0) ? v_start : v_end;

      if (map_count == vertex_count)
      {
        if (top.m_topv_map[v0] == topv1 && top.m_topv_map[v1] == topv0)
          std::swap(v0, v1);
      }

      if (!have_first_pair)
      {
        first_v0 = v0;
        first_v1 = v1;
        have_first_pair = true;
      }
      else
      {
        const bool same_ordered = (v0 == first_v0 && v1 == first_v1);
        const bool same_reversed = (v0 == first_v1 && v1 == first_v0);
        if (!same_ordered && !same_reversed)
          return true;
      }
    }
  }

  return false;
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
