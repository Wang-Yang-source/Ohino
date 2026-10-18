#pragma once
/**
 * @file rhcommon_c_bridge.h
 * @brief Reverse-engineered C ABI exports matching `rhcommon_c.dll` that bridge
 *        RhinoCommon C# (`UnsafeNativeMethods`) P/Invoke calls to `tl.dll` / `opennurbs.dll`.
 *
 * Binary Provenance (rhcommon_c.dll):
 *   - TL_Brep_PointIsOnFace          (RVA 0x153fe0) -> TL_Brep::Promote + TL_Brep::IsPointOnFace
 *   - TLC_IsMeshEdgeUnwelded         (RVA 0x1555b0) -> TL_IsMeshEdgeUnwelded
 *   - ON_NurbsCurve_RemoveKnots      (RVA 0x154d90) -> TL_NurbsCurve::Promote + TL_NurbsCurve::RemoveKnots
 *   - ON_Surface_RemoveKnots         (RVA 0x1552c0) -> TL_NurbsSurface::Promote + TL_NurbsSurface::RemoveKnots
 *   - ON_ArcCurve_ToNurbsCurve       (RVA 0x1533f0) -> TL_DeformableArc2
 */

#include "../tl_export.h"

/**
 * Exact C ABI match for `rhcommon_c.dll!TL_Brep_PointIsOnFace` (RVA 0x153fe0)
 * P/Invoked by `RhinoCommon.dll!UnsafeNativeMethods.TL_Brep_PointIsOnFace`.
 */
RH_C_FUNCTION bool TL_Brep_PointIsOnFace(
  const ON_Brep* pConstBrep,
  int face_index,
  double u,
  double v,
  double tol
);

/**
 * Exact C ABI match for `rhcommon_c.dll!TLC_IsMeshEdgeUnwelded` (RVA 0x1555b0)
 * P/Invoked by `RhinoCommon.dll!UnsafeNativeMethods.TLC_IsMeshEdgeUnwelded`.
 */
RH_C_FUNCTION bool TLC_IsMeshEdgeUnwelded(
  const ON_Mesh* pConstMesh,
  int edge_index
);

/**
 * Exact C ABI match for `rhcommon_c.dll!ON_NurbsCurve_RemoveKnots` (RVA 0x154d90)
 */
RH_C_FUNCTION bool ON_NurbsCurve_RemoveKnots(
  ON_NurbsCurve* pCurve,
  int index0,
  int index1,
  double tolerance
);

/**
 * Exact C ABI match for `rhcommon_c.dll!ON_Surface_RemoveKnots` (RVA 0x1552c0)
 */
RH_C_FUNCTION bool ON_Surface_RemoveKnots(
  ON_NurbsSurface* pSurface,
  int dir,
  int index0,
  int index1,
  double tolerance
);
