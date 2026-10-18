#pragma once
/**
 * @file tl_promote.h
 * @brief Reverse-engineered TL_Brep, TL_NurbsCurve, and TL_NurbsSurface promotion classes
 *        from Rhino 9 WIP tl.dll.
 *
 * Binary Provenance (tl.dll):
 *   - TL_Brep::Promote(const ON_Object*)         RVA 0x2b2180
 *   - TL_Brep::Promote(ON_Object*)               RVA 0x2b21b0
 *   - TL_NurbsSurface::Promote(const ON_Object*) RVA 0x2b21e0
 *   - TL_NurbsSurface::Promote(ON_Object*)       RVA 0x2b2210
 *   - TL_NurbsCurve::Promote(const ON_Object*)   RVA 0x2b20c0
 *
 * In tl.dll, TL_Brep, TL_NurbsCurve, and TL_NurbsSurface are zero-field C++ subclasses
 * of ON_Brep, ON_NurbsCurve, and ON_NurbsSurface respectively. `Promote` calls
 * `ON_Brep::Cast(obj)` (via IAT `0x180758fc8`) and returns the pointer directly cast as
 * `TL_Brep*` because `sizeof(TL_Brep) == sizeof(ON_Brep)` and no additional data members
 * exist on the wrapper class.
 */

#include "tl_export.h"

class TL_CLASS TL_Brep : public ON_Brep
{
public:
  static const TL_Brep* Promote(const ON_Object* obj);
  static TL_Brep* Promote(ON_Object* obj);

  /**
   * Reverse-engineered from tl.dll!?PointIsOnFace@TL_Brep@@QEBAHHAEBVON_2dPoint@@NN@Z (RVA 0x1fcf30)
   * Tests whether a 2D surface parameter (u, v) lies inside, outside, or on the boundary
   * of face `face_index`.
   *
   * @return 0 = Outside, 1 = Inside, 2 = On boundary (within u_tol, v_tol)
   */
  int PointIsOnFace(
    int face_index,
    const ON_2dPoint& uv,
    double u_tol,
    double v_tol
  ) const;

  /**
   * Reverse-engineered from tl.dll!?IsPointOnFace@TL_Brep@@QEBAHHNNN@Z (RVA 0x1fd520)
   * Called by rhcommon_c.dll!TL_Brep_PointIsOnFace (RVA 0x153fe0).
   */
  bool IsPointOnFace(
    int face_index,
    double u,
    double v,
    double tol
  ) const;
};

class TL_CLASS TL_NurbsCurve : public ON_NurbsCurve
{
public:
  static const TL_NurbsCurve* Promote(const ON_Object* obj);
  static TL_NurbsCurve* Promote(ON_Object* obj);

  /**
   * Reverse-engineered from tl.dll!?RemoveKnots@TL_NurbsCurve@@QEAA_NHHN@Z
   */
  bool RemoveKnots(int index0, int index1, double tolerance);
};

class TL_CLASS TL_NurbsSurface : public ON_NurbsSurface
{
public:
  static const TL_NurbsSurface* Promote(const ON_Object* obj);
  static TL_NurbsSurface* Promote(ON_Object* obj);

  /**
   * Reverse-engineered from tl.dll!?RemoveKnots@TL_NurbsSurface@@QEAA_NHHHN@Z (RVA 0x384d90)
   */
  bool RemoveKnots(int dir, int index0, int index1, double tolerance);
};
