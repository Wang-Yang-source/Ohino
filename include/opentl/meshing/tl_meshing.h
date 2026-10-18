#pragma once
/**
 * @file tl_meshing.h
 * @brief OpenTL Meshing & Mesh Topology subsystem (meshing/)
 *
 * Binary Provenance:
 *   - tl.dll!?TL_IsMeshEdgeUnwelded@@YA_NHPEBVON_Mesh@@@Z (RVA 0x33bd70 .. 0x33bfb0)
 *   - rhcommon_c.dll!TLC_IsMeshEdgeUnwelded (RVA 0x1555b0)
 *   - tl.dll!TL_MeshSurface / TL_MeshBrep
 */

#include "../tl_export.h"

/**
 * Exact reverse-engineered reimplementation of:
 *   `bool TL_IsMeshEdgeUnwelded(int edge_index, const ON_Mesh* mesh)`
 *   from `tl.dll` at RVA `0x33bd70`.
 *
 * Disassembly logic verified against `tl.dll` (`0x18033bd70 .. 0x18033bfb0`):
 * 1. If `mesh == nullptr` (`test rdx, rdx`), returns `false`.
 * 2. Calls `ON_Mesh::Topology()` (`call qword ptr [rip + 0x41daeb]`).
 * 3. Validates `0 <= edge_index < top.m_tope.Count()` (`[rax + 0x38]`).
 * 4. Reads `const ON_MeshTopologyEdge& tope = top.m_tope[edge_index]` (`sizeof == 24`).
 * 5. If `tope.m_topf_count <= 1`, returns `false` (interior manifold shared edge is required
 *    for an unwelded crease check between faces).
 * 6. Resolves the two topological vertices `topvi[0]`, `topvi[1]` in `top.m_topv`:
 *    - Fast path: if both `topv[0].m_v_count <= 1` and `topv[1].m_v_count <= 1`, each
 *      topological vertex has only 1 mesh vertex, so the edge cannot be unwelded -> returns `false`.
 * 7. Slow path (`0x18033be23 .. 0x18033bfa0`): iterates over the incident faces in `tope.m_topfi[k]`,
 *    locates which side `side` of face `F[fi]` corresponds to `edge_index` (watching `m_reve[side]`),
 *    extracts the actual mesh vertex pair `(v0, v1)` from `F.vi[]`, and checks whether all incident
 *    faces share the identical mesh vertex indices `(v0, v1)` or `(v1, v0)`. If any incident face
 *    uses distinct mesh vertex indices along this topological edge, returns `true` (unwelded).
 */
TL_FUNCTION bool TL_IsMeshEdgeUnwelded(int edge_index, const ON_Mesh* mesh);

/**
 * Generate a regular parameter-grid ON_Mesh from an ON_Surface.
 */
TL_FUNCTION ON_Mesh* TL_MeshSurfaceGrid(
  const ON_Surface& surface,
  int u_divisions,
  int v_divisions,
  ON_Mesh* result_mesh = nullptr
);
