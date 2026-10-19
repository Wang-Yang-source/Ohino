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

/**
 * Convert an ON_SubD (optionally subdivided `extra_subdivision_levels` further steps)
 * into a smooth-shaded ON_Mesh with computed face and vertex normals.
 */
TL_FUNCTION ON_Mesh* TL_MeshFromSubD(
  const ON_SubD& subd,
  unsigned int extra_subdivision_levels = 0,
  ON_Mesh* result = nullptr
);

/**
 * Convert an arbitrary triangle/quad `ON_Mesh` control net into a Catmull-Clark `ON_SubD`
 * (`ON_SubD::CreateFromMesh`), optionally applying `subdivision_levels` global Catmull-Clark
 * refinement steps (`SubD From Mesh`).
 */
TL_FUNCTION ON_SubD* TL_CreateSubDFromMesh(
  const ON_Mesh& control_net_mesh,
  unsigned int subdivision_levels = 1,
  bool crease_naked_edges = false,
  ON_SubD* result = nullptr
);

/**
 * Create a smooth Catmull-Clark `ON_SubD` cylinder (`SubDCylinder`) from `base_circle` and `height`.
 */
TL_FUNCTION ON_SubD* TL_CreateSubDCylinder(
  const ON_Circle& base_circle,
  double height,
  unsigned int around_faces = 8,
  unsigned int vertical_faces = 2,
  unsigned int subdivision_levels = 1,
  ON_SubD* result = nullptr
);

/**
 * Create a smooth Catmull-Clark quad-sphere `ON_SubD` (`SubDSphere`) centered at `center` with `radius`.
 */
TL_FUNCTION ON_SubD* TL_CreateSubDSphere(
  const ON_3dPoint& center,
  double radius,
  unsigned int subdivision_levels = 2,
  ON_SubD* result = nullptr
);
