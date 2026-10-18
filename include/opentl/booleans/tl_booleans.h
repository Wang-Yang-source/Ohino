#pragma once
/**
 * @file tl_booleans.h
 * @brief OpenTL Brep Face Trimming, Point Containment & Boolean Classification subsystem (booleans/)
 *
 * Binary Provenance:
 *   - tl.dll!?PointIsOnFace@TL_Brep@@QEBAHHAEBVON_2dPoint@@NN@Z (RVA 0x1fcf30 .. 0x1fd350)
 *   - tl.dll!?IsPointOnFace@TL_Brep@@QEBAHHNNN@Z                (RVA 0x1fd520)
 *   - rhcommon_c.dll!TL_Brep_PointIsOnFace                      (RVA 0x153fe0)
 */

#include "../tl_export.h"
#include "../tl_promote.h"

enum class TL_PointFaceRelation : int
{
  Outside  = 0,
  Inside   = 1,
  Boundary = 2
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
