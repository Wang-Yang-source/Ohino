#pragma once
/**
 * @file tl_booleans.h
 * @brief OpenTL Brep Face Containment, Solid Winding Number, Planar Slice & CSG Booleans (booleans/)
 *
 * Binary Provenance:
 *   - tl.dll!?PointIsOnFace@TL_Brep@@QEBAHHAEBVON_2dPoint@@NN@Z (RVA 0x1fcf30 .. 0x1fd350)
 *   - tl.dll!?IsPointOnFace@TL_Brep@@QEBAHHNNN@Z                (RVA 0x1fd520)
 *   - rhcommon_c.dll!TL_Brep_PointIsOnFace                      (RVA 0x153fe0)
 */

#include "../tl_export.h"
#include "../tl_promote.h"

#include <vector>

enum class TL_PointFaceRelation : int
{
  Outside  = 0,
  Inside   = 1,
  Boundary = 2
};

enum class TL_BooleanOp : int
{
  Union        = 0, ///< A U B  (keep A outside B, keep B outside A)
  Intersection = 1, ///< A n B  (keep A inside B,  keep B inside A)
  Difference   = 2  ///< A \ B  (keep A outside B, keep reversed B inside A)
};

/**
 * Standalone helper exposing TL_Brep::PointIsOnFace classification for boolean face splitting
 * and point-in-trimmed-face testing.
 */
TL_FUNCTION TL_PointFaceRelation TL_ClassifyPointOnBrepFace(
  const ON_Brep& brep,
  int face_index,
  const ON_2dPoint& uv,
  double u_tol = 1e-6,
  double v_tol = 1e-6
);

/**
 * Signed volume of a closed triangle/quad mesh via the divergence theorem:
 *   V = (1/6) sum_T a . (b x c)
 * Positive when faces are wound outward consistently.
 */
TL_FUNCTION double TL_MeshSignedVolume(const ON_Mesh& mesh);

/**
 * Generalized winding number omega(P) = (1 / 4pi) sum_T solid_angle(T, P) using the exact
 * van Oosterom & Strackee (1983) formula.  Equals ~1.0 strictly inside a closed outward-wound
 * mesh and ~0.0 outside, and is invariant to duplicate/unwelded hard-crease vertices.
 */
TL_FUNCTION double TL_MeshWindingNumber(const ON_Mesh& mesh, const ON_3dPoint& point);

/**
 * Classify a 3D point against a closed solid mesh (`Inside`, `Outside`, or `Boundary`
 * when within `tolerance` of any mesh triangle).
 */
TL_FUNCTION TL_PointFaceRelation TL_MeshContainsPoint(
  const ON_Mesh& mesh,
  const ON_3dPoint& point,
  double tolerance = 1e-6
);

/**
 * Slice a closed or open mesh with `plane`, producing the positive half-space mesh
 * (`plane.ValueAt(p) >= 0`), the negative half-space mesh (`<= 0`), and the ordered
 * planar section polylines.  When `cap_section` is true, closed section loops are capped
 * with hard-crease planar fans on both halves so a sliced solid stays watertight/closed
 * and obeys `V(pos) + V(neg) == V(mesh)`.
 *
 * Returns the number of section loops extracted.
 */
TL_FUNCTION int TL_SliceMeshWithPlane(
  const ON_Mesh& mesh,
  const ON_Plane& plane,
  bool cap_section,
  ON_Mesh* out_positive,
  ON_Mesh* out_negative,
  std::vector<std::vector<ON_3dPoint>>* out_section_loops = nullptr
);

/**
 * Exact polyhedral CSG boolean (`Union`, `Intersection`, `Difference`) between two closed
 * solid meshes using localized coplanar/intersecting triangle splitting + generalized
 * winding number classification.
 *
 * Returns true when `out_mesh` is non-empty (or valid empty result for disjoint intersection).
 */
TL_FUNCTION bool TL_MeshBoolean(
  const ON_Mesh& mesh_a,
  const ON_Mesh& mesh_b,
  TL_BooleanOp op,
  ON_Mesh& out_mesh,
  double tolerance = 1e-7
);

/**
 * Split a closed or open `ON_Brep` with `plane`, trimming its underlying surfaces in 2D
 * `(u, v)` parameter space (`ON_BrepLoop` / `ON_BrepTrim` / `ON_BrepEdge`) and optionally
 * adding a planar cap face along the section loop so both resulting `ON_Brep` solids stay
 * closed and watertight.
 *
 * Returns the number of planar section loops extracted.
 */
TL_FUNCTION int TL_SplitBrepWithPlane(
  const ON_Brep& brep,
  const ON_Plane& plane,
  bool cap_section,
  ON_Brep* out_positive,
  ON_Brep* out_negative,
  std::vector<std::vector<ON_3dPoint>>* out_section_loops = nullptr,
  double tolerance = 1e-6
);

/**
 * Exact parameter-space trimmed `ON_Brep` boolean (`Union`, `Intersection`, `Difference`)
 * between two closed `ON_Brep` solids:
 *   1. Pairwise `TL_SurfaceSurfaceIntersect` across overlapping face pairs;
 *   2. Parameter-space `(u, v)` face clipping & `ON_BrepLoop` / `ON_BrepTrim` / `ON_BrepEdge`
 *      reconstruction on every surviving face fragment;
 *   3. Generalized winding number classification (`TL_MeshContainsPoint`) of each trimmed
 *      Brep face fragment against the opposite solid, with `ON_BrepFace::m_bRev` flipped
 *      on interior cutter faces for `Difference`.
 *
 * Returns true when `out_brep` is valid and non-empty.
 */
TL_FUNCTION bool TL_BrepBoolean(
  const ON_Brep& brep_a,
  const ON_Brep& brep_b,
  TL_BooleanOp op,
  ON_Brep& out_brep,
  double tolerance = 1e-6
);
