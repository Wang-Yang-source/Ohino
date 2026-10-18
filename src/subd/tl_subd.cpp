#include "opentl/subd/tl_subd.h"

TL_FUNCTION ON_SubD* TL_CreateSubDBox(
  const ON_BoundingBox& bbox,
  unsigned int subdivision_levels,
  ON_SubD* result
)
{
  if (!bbox.IsValid())
    return nullptr;

  // CRITICAL: `ON_SubD::CreateSubDBox` expects a right-handed quad-prism corner layout:
  //   xdir = corners[1] - corners[0], ydir = corners[3] - corners[0], zdir = corners[4] - corners[0].
  // `ON_BoundingBox::GetCorners` instead enumerates `(i,j,k)` with `z` in the inner loop
  // (`corners[1] = (min,min,max)`, `corners[3] = (min,max,max)`), which fed a left-handed
  // sheared basis `(+Z, +Y+Z, +X)` into `CreateSubDBox` and produced an inside-out skewed box!
  const ON_3dPoint& mn = bbox.m_min;
  const ON_3dPoint& mx = bbox.m_max;
  const ON_3dPoint corners[8] = {
    ON_3dPoint(mn.x, mn.y, mn.z), // 0: bottom (-x, -y, -z)
    ON_3dPoint(mx.x, mn.y, mn.z), // 1: bottom (+x, -y, -z) -> xdir = +X
    ON_3dPoint(mx.x, mx.y, mn.z), // 2: bottom (+x, +y, -z)
    ON_3dPoint(mn.x, mx.y, mn.z), // 3: bottom (-x, +y, -z) -> ydir = +Y
    ON_3dPoint(mn.x, mn.y, mx.z), // 4: top    (-x, -y, +z) -> zdir = +Z
    ON_3dPoint(mx.x, mn.y, mx.z), // 5: top    (+x, -y, +z)
    ON_3dPoint(mx.x, mx.y, mx.z), // 6: top    (+x, +y, +z)
    ON_3dPoint(mn.x, mx.y, mx.z)  // 7: top    (-x, +y, +z)
  };

  if (nullptr != result)
    result->Clear();

  ON_SubD* subd = ON_SubD::CreateSubDBox(
    corners,
    ON_SubDEdgeTag::Smooth,
    1,
    1,
    1,
    result
  );
  if (nullptr == subd)
    return nullptr;

  subd->UpdateAllTagsAndSectorCoefficients(false);
  if (subdivision_levels > 0)
  {
    subd->GlobalSubdivide(subdivision_levels);
  }
  return subd;
}

TL_FUNCTION ON_Mesh* TL_MeshFromSubD(
  const ON_SubD& subd,
  unsigned int extra_subdivision_levels,
  ON_Mesh* result
)
{
  const ON_SubD* source = &subd;
  ON_SubD subdivided_copy;
  if (extra_subdivision_levels > 0)
  {
    subdivided_copy = subd;
    subdivided_copy.UpdateAllTagsAndSectorCoefficients(false);
    subdivided_copy.GlobalSubdivide(extra_subdivision_levels);
    source = &subdivided_copy;
  }

  ON_Mesh* mesh = source->GetControlNetMesh(result, ON_SubDGetControlNetMeshPriority::Geometry);
  if (nullptr == mesh)
    return nullptr;

  if (mesh->m_F.Count() <= 0 || mesh->VertexCount() <= 0)
  {
    if (mesh != result)
      delete mesh;
    return nullptr;
  }

  mesh->ComputeFaceNormals();
  mesh->ComputeVertexNormals();
  return mesh;
}
