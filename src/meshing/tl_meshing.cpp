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

// ============================================================================
// Mesh boundary detection & capping
// ============================================================================

TL_FUNCTION int TL_CountMeshBoundaryEdges(const ON_Mesh& mesh)
{
  if (mesh.m_F.Count() <= 0)
    return 0;

  const ON_MeshTopology& top = mesh.Topology();
  int count = 0;
  for (int i = 0; i < top.m_tope.Count(); ++i)
  {
    if (1 == top.m_tope[i].m_topf_count)
      ++count;
  }
  return count;
}

namespace {

/** Representative mesh vertex of a topological vertex (first of its welded cluster). */
int RepresentativeVertex(const ON_MeshTopology& top, int topv_index)
{
  if (topv_index < 0 || topv_index >= top.m_topv.Count())
    return -1;
  const ON_MeshTopologyVertex& topv = top.m_topv[topv_index];
  if (topv.m_v_count <= 0 || nullptr == topv.m_vi)
    return -1;
  return topv.m_vi[0];
}

/** Normal of the single face adjacent to a boundary edge. */
bool BoundaryFaceNormal(const ON_Mesh& mesh, int face_index, ON_3dVector& normal)
{
  if (face_index < 0 || face_index >= mesh.m_F.Count())
    return false;
  const ON_MeshFace& face = mesh.m_F[face_index];
  if (face.vi[0] < 0 || face.vi[1] < 0 || face.vi[2] < 0)
    return false;
  if (face.vi[0] >= mesh.VertexCount() || face.vi[1] >= mesh.VertexCount() ||
      face.vi[2] >= mesh.VertexCount())
    return false;

  normal = ON_CrossProduct(mesh.m_V[face.vi[1]] - mesh.m_V[face.vi[0]],
                           mesh.m_V[face.vi[2]] - mesh.m_V[face.vi[0]]);
  return normal.Unitize();
}

/** Newell normal of an ordered 3D loop (robust for non-planar rims). */
ON_3dVector LoopNormal(const std::vector<ON_3dPoint>& points)
{
  ON_3dVector normal = ON_3dVector::ZeroVector;
  const size_t n = points.size();
  for (size_t i = 0; i < n; ++i)
  {
    const ON_3dPoint& a = points[i];
    const ON_3dPoint& b = points[(i + 1) % n];
    normal.x += (a.y - b.y) * (a.z + b.z);
    normal.y += (a.z - b.z) * (a.x + b.x);
    normal.z += (a.x - b.x) * (a.y + b.y);
  }
  normal.Unitize();
  return normal;
}

} // anonymous namespace

TL_FUNCTION int TL_CapMeshOpenBoundaryLoops(ON_Mesh& mesh, int* out_triangles,
                                            int* out_open_chains, bool hard_crease)
{
  if (nullptr != out_triangles)
    *out_triangles = 0;
  if (nullptr != out_open_chains)
    *out_open_chains = 0;

  if (mesh.m_F.Count() <= 0)
    return 0;

  const ON_MeshTopology& top = mesh.Topology();

  // ---- boundary edges, indexed by their unordered topological vertex pair ------
  std::vector<int> boundary_edges;
  for (int i = 0; i < top.m_tope.Count(); ++i)
  {
    if (1 == top.m_tope[i].m_topf_count)
      boundary_edges.push_back(i);
  }
  if (boundary_edges.empty())
    return 0;

  auto pair_key = [](int a, int b) -> uint64_t {
    const uint32_t lo = static_cast<uint32_t>(std::min(a, b));
    const uint32_t hi = static_cast<uint32_t>(std::max(a, b));
    return (static_cast<uint64_t>(hi) << 32) | lo;
  };

  std::vector<std::pair<uint64_t, int>> boundary_index;
  boundary_index.reserve(boundary_edges.size());
  for (int edge_index : boundary_edges)
  {
    const ON_MeshTopologyEdge& edge = top.m_tope[edge_index];
    boundary_index.push_back({ pair_key(edge.m_topvi[0], edge.m_topvi[1]), edge_index });
  }
  std::sort(boundary_index.begin(), boundary_index.end());

  auto find_boundary_edge = [&](int a, int b) -> int {
    const uint64_t key = pair_key(a, b);
    const auto it = std::lower_bound(boundary_index.begin(), boundary_index.end(),
                                     std::make_pair(key, -1));
    if (it != boundary_index.end() && it->first == key)
      return it->second;
    return -1;
  };

  // ---- directed boundary edges taken from the face winding --------------------
  // Walking these (rather than the undirected topology pairs) is what makes the cap
  // orientation correct: the rim direction is the surface's own direction, so the cap
  // must be wound the other way round to close the shell consistently.
  const int topv_count = top.m_topv.Count();
  const bool have_topv_map = (top.m_topv_map.Count() == mesh.VertexCount());
  if (!have_topv_map)
    return 0;

  std::vector<std::vector<std::pair<int, int>>> outgoing(static_cast<size_t>(topv_count));
  for (int fi = 0; fi < mesh.m_F.Count(); ++fi)
  {
    const ON_MeshFace& face = mesh.m_F[fi];
    const int slots = face.IsQuad() ? 4 : 3;
    for (int k = 0; k < slots; ++k)
    {
      const int v0 = face.vi[k];
      const int v1 = face.vi[(k + 1) % slots];
      if (v0 < 0 || v1 < 0 || v0 >= mesh.VertexCount() || v1 >= mesh.VertexCount())
        continue;
      const int t0 = top.m_topv_map[v0];
      const int t1 = top.m_topv_map[v1];
      if (t0 < 0 || t1 < 0 || t0 >= topv_count || t1 >= topv_count || t0 == t1)
        continue;
      const int edge_index = find_boundary_edge(t0, t1);
      if (edge_index < 0)
        continue;
      outgoing[static_cast<size_t>(t0)].push_back({ t1, edge_index });
    }
  }

  // ---- chain the directed edges into closed loops ------------------------------
  std::vector<char> edge_used(static_cast<size_t>(top.m_tope.Count()), 0);
  std::vector<std::vector<int>> loops;
  int open_chains = 0;

  for (int seed_edge : boundary_edges)
  {
    if (edge_used[static_cast<size_t>(seed_edge)])
      continue;

    const ON_MeshTopologyEdge& seed = top.m_tope[seed_edge];
    int from = seed.m_topvi[0];
    int to = seed.m_topvi[1];

    // Prefer the direction that actually exists in the face winding.
    if (outgoing[static_cast<size_t>(from)].empty() ||
        std::none_of(outgoing[static_cast<size_t>(from)].begin(),
                     outgoing[static_cast<size_t>(from)].end(),
                     [&](const std::pair<int, int>& e) { return e.second == seed_edge; }))
    {
      std::swap(from, to);
    }

    std::vector<int> loop;
    loop.push_back(from);
    int current = from;
    int next = to;
    int current_edge = seed_edge;
    bool closed = false;

    for (size_t guard = 0; guard <= boundary_edges.size() + 1; ++guard)
    {
      edge_used[static_cast<size_t>(current_edge)] = 1;
      loop.push_back(next);
      current = next;
      if (current == loop.front())
      {
        loop.pop_back();  // do not duplicate the seam vertex
        closed = true;
        break;
      }

      int follow_edge = -1;
      int follow_vertex = -1;
      for (const auto& link : outgoing[static_cast<size_t>(current)])
      {
        if (edge_used[static_cast<size_t>(link.second)])
          continue;
        follow_edge = link.second;
        follow_vertex = link.first;
        break;
      }
      if (follow_edge < 0)
        break;
      current_edge = follow_edge;
      next = follow_vertex;
    }

    if (closed && loop.size() >= 3)
      loops.push_back(loop);
    else
      ++open_chains;
  }

  // ---- fan each loop ---------------------------------------------------------
  int capped = 0;
  int triangles_added = 0;

  for (const std::vector<int>& loop : loops)
  {
    std::vector<int> vertices;
    std::vector<ON_3dPoint> points;
    vertices.reserve(loop.size());
    points.reserve(loop.size());
    bool valid = true;
    for (int topv : loop)
    {
      const int vi = RepresentativeVertex(top, topv);
      if (vi < 0 || vi >= mesh.VertexCount())
      {
        valid = false;
        break;
      }
      vertices.push_back(vi);
      points.push_back(mesh.m_V[vi]);
    }
    if (!valid || vertices.size() < 3)
      continue;

    const ON_3dVector normal = LoopNormal(points);
    if (normal.IsZero())
      continue;  // degenerate rim

    ON_3dPoint centroid = ON_3dPoint::Origin;
    for (const ON_3dPoint& p : points)
      centroid += p;
    centroid = centroid / static_cast<double>(points.size());

    // Hard crease: give the cap its own vertices so the rim shades as a 90 degree edge
    // instead of averaging the wall normal into the cap's plane normal.
    std::vector<int> cap_vertices;
    cap_vertices.reserve(vertices.size());
    if (hard_crease)
    {
      for (const ON_3dPoint& p : points)
      {
        const int copy = mesh.VertexCount();
        if (!mesh.SetVertex(copy, p))
        {
          cap_vertices.clear();
          break;
        }
        cap_vertices.push_back(copy);
      }
      if (cap_vertices.size() != vertices.size())
        continue;
    }
    else
    {
      cap_vertices = vertices;
    }

    const int centre_index = mesh.VertexCount();
    if (!mesh.SetVertex(centre_index, centroid))
      continue;

    // The loop runs in the surface's own direction, so the cap is wound the other way.
    const size_t n = cap_vertices.size();
    const int face_start = mesh.m_F.Count();
    int added = 0;
    for (size_t i = 0; i < n; ++i)
    {
      const int a = cap_vertices[i];
      const int b = cap_vertices[(i + 1) % n];
      if (a == b || a == centre_index || b == centre_index)
        continue;
      if (mesh.SetTriangle(face_start + added, centre_index, b, a))
        ++added;
    }
    if (added > 0)
    {
      triangles_added += added;
      ++capped;
    }
  }

  if (triangles_added > 0)
  {
    // ON_Mesh caches the topology queried above; adding faces invalidates it.
    mesh.DestroyTopology();
    mesh.ComputeFaceNormals();
  }

  if (nullptr != out_triangles)
    *out_triangles = triangles_added;
  if (nullptr != out_open_chains)
    *out_open_chains = open_chains;
  return capped;
}
