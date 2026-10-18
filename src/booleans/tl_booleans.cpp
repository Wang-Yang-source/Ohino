#include "opentl/booleans/tl_booleans.h"

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
