#include "opentl/c_api/rhcommon_c_bridge.h"
#include "opentl/tl_promote.h"
#include "opentl/meshing/tl_meshing.h"

RH_C_FUNCTION bool TL_Brep_PointIsOnFace(
  const ON_Brep* pConstBrep,
  int face_index,
  double u,
  double v,
  double tol
)
{
  const TL_Brep* tl_brep = TL_Brep::Promote(pConstBrep);
  if (nullptr == tl_brep)
    return false;
  return tl_brep->IsPointOnFace(face_index, u, v, tol);
}

RH_C_FUNCTION bool TLC_IsMeshEdgeUnwelded(
  const ON_Mesh* pConstMesh,
  int edge_index
)
{
  return TL_IsMeshEdgeUnwelded(edge_index, pConstMesh);
}

RH_C_FUNCTION bool ON_NurbsCurve_RemoveKnots(
  ON_NurbsCurve* pCurve,
  int index0,
  int index1,
  double tolerance
)
{
  TL_NurbsCurve* tl_curve = TL_NurbsCurve::Promote(pCurve);
  if (nullptr == tl_curve)
    return false;
  return tl_curve->RemoveKnots(index0, index1, tolerance);
}

RH_C_FUNCTION bool ON_Surface_RemoveKnots(
  ON_NurbsSurface* pSurface,
  int dir,
  int index0,
  int index1,
  double tolerance
)
{
  TL_NurbsSurface* tl_surface = TL_NurbsSurface::Promote(pSurface);
  if (nullptr == tl_surface)
    return false;
  return tl_surface->RemoveKnots(dir, index0, index1, tolerance);
}
