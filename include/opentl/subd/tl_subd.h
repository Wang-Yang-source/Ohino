#pragma once
/**
 * @file tl_subd.h
 * @brief OpenTL Subdivision Surface (SubD) evaluation and conversion subsystem (subd/)
 */

#include "../tl_export.h"

/**
 * Create a Catmull-Clark ON_SubD box from an ON_BoundingBox / ON_Box and subdivide `level` steps.
 */
TL_FUNCTION ON_SubD* TL_CreateSubDBox(
  const ON_BoundingBox& bbox,
  unsigned int subdivision_levels = 1,
  ON_SubD* result = nullptr
);
