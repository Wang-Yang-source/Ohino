#include "opentl/subd/tl_subd.h"

TL_FUNCTION ON_SubD* TL_CreateSubDBox(
  const ON_BoundingBox& bbox,
  unsigned int subdivision_levels,
  ON_SubD* result
)
{
  if (!bbox.IsValid())
    return nullptr;

  ON_3dPoint corners[8];
  if (!bbox.GetCorners(corners))
    return nullptr;

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
