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
 *   from `tl.dll` at RVA `0x33bd70` (569 bytes / 186 instructions).
 *
 * Disassembly verified against `tl.dll` (`0x18033bd70 .. 0x18033bfa8`):
 * 1. `mesh == nullptr` -> `false`.
 * 2. `ON_Mesh::Topology()`; `edge_index < 0 || >= top.m_tope.Count()` -> `false`.
 * 3. `tope.m_topf_count` must be **exactly 2** (`cmp dword ptr [rax+0x8], 2` / `jne false`).
 *    Non-manifold edges (3+ incident faces) return `false`; they are not "unwelded".
 *    (The earlier `<= 1` reading was wrong.)
 * 4. The two topological vertices are `top.m_topv[tope.m_topvi[0..1]]`; the two incident
 *    faces are `mesh->m_F[tope.m_topfi[0..1]]`.
 * 5. Two dual loops (`0x18033be84..0x18033bf6c`): for each topological vertex, walk its
 *    `m_vi[]` mesh vertices and record the **mesh vertex index** found in face 0
 *    (registers `r13`/`r15`) and in face 1 (`r12`/`ebp`). Each hit overwrites the
 *    register, so the last matching mesh vertex wins. There is **no** `m_v_count <= 1`
 *    fast path.
 * 6. Four inequality tests (`0x18033bf73..0x18033bf85`): if any of
 *    `a0 == a1`, `a0 == b1`, `b0 == a1`, `b0 == b1` holds (where `a0/a1` are the mesh
 *    vertices of topological vertex 0 in face 0 / face 1, `b0/b1` likewise for the
 *    other end), return `false`; all four indices pairwise distinct -> `true`.
 *
 * Semantics: an edge counts as unwelded only when **both** endpoints are split into
 * four distinct mesh vertices. A single shared endpoint returns `false`.
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

/**
 * Count topological edges used by exactly one face (the mesh boundary).
 * A welded, closed mesh returns 0; an open surface shell returns the rim length.
 */
TL_FUNCTION int TL_CountMeshBoundaryEdges(const ON_Mesh& mesh);

/**
 * Cap every closed boundary loop of an open mesh with a triangle fan.
 *
 * Procedure:
 *   1. `ON_Mesh::Topology()`; edges with `m_topf_count == 1` form the boundary.
 *   2. Boundary edges are chained through shared topological vertices into loops.
 *      Loops that do not close (open rims, non-manifold junctions) are skipped and
 *      reported through `out_open_chains`.
 *   3. Each loop is oriented so the cap faces the same way as the adjacent face at its
 *      first boundary edge (the cap must not turn the shell inside out).
 *   4. The loop centroid becomes one new vertex and the loop is fanned into
 *      `n` triangles.
 *
 * Returns the number of loops capped.  `out_triangles` (optional) receives the number
 * of triangles appended.  The mesh is recomputed (`ComputeFaceNormals`) before return.
 *
 * `hard_crease` (default true) duplicates the rim vertices for the cap instead of
 * reusing the shell's: shared rim vertices would average the cap's plane normal with the
 * wall's radial normal, shading a 90 degree rim as a soft dome.  The duplicates sit on
 * identical positions, so `ON_MeshTopology` still welds them and the mesh stays
 * watertight; only the vertex normals split.
 *
 * Reference-product note: `tl.dll` has no exported capping entry point that the static
 * evidence covers, so this is an independent implementation, not a replay.
 */
TL_FUNCTION int TL_CapMeshOpenBoundaryLoops(ON_Mesh& mesh, int* out_triangles = nullptr,
                                            int* out_open_chains = nullptr,
                                            bool hard_crease = true);
