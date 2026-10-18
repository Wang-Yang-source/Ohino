#pragma once
/**
 * @file rhino_mvp.h
 * @brief OpenTL Rhino-like MVP Core API: Geometry Types, Scene Document,
 *        Transformations, Intersections, Curve Sampling, and Serialization (JSON + 3DM).
 *
 * Binary Provenance & Compatibility Evidence (saved in re-database/):
 *   - RhinoCommon.dll (SHA256: 7371baa598c5cdd83972eed48539d1e1e455b536899d556a0399f7879677ee89)
 *     REA Evidence IDs: ev_9cbc13d87953d809d4fd4b58d519236c1440f8a24ff86cf4cd6f392493be47fc,
 *                       ev_a0104bfe9cdcaa0ba40c811c92f5420ecbf14b12e183c2f73a0bb65c17c05c17,
 *                       ev_9c800b2a176a9e340d50209899faf78e96d40561b90a32ed8bad239f2e32b125,
 *                       ev_9a3df491bed33dacf94031349dae4d9caeea1c03b9541689ac254293c257952c
 *   - rhcommon_c.dll  (SHA256: d9006bceff5476a671439e7dc4ffacc666c2ca84f3c5e29657325bc688b229c1)
 *     REA Evidence ID:  ev_835ce2eb32a8991259c74c55247c52625e8f5338d4b8ac995e2e59948370c49b
 *     * ON_Intersect_LineLine  (RVA 0xcc020) -> opennurbs!ON_Intersect(ON_Line, ON_Line, double*, double*)
 *     * ON_Intersect_LinePlane (RVA 0xcc040) -> opennurbs!ON_Intersect(ON_Line, ON_Plane, double*)
 *     * ON_Intersect_CurveLine (RVA 0xcc8a0) -> curve-line intersection
 *     * ON_Curve_IntersectPlane (RVA 0xcc7b0) -> curve-plane intersection
 *   - tl.dll          (SHA256: 550a541018ac1b23c3a0dec53d3534fca4d212657dd7309e9b70c38775af3a7f)
 *     REA Evidence ID:  ev_8e0f70a9b13a407692a4fade183b51c80a5c4e58c0064aceed0348122824ae0d
 *     * TL_3DX_LineXPlane      (RVA 0x305d0)
 *     * TL_GetClosestPoint     (RVA 0x214540 / 0x214670)
 *     * TL_ArcLengthPointsOnNurb (RVA 0xe37a0)
 *     * TL_DeformableArc2      (RVA 0x37b220)
 *     * TL_DeformableCircle    (RVA 0x37b790)
 */

#include "../tl_export.h"
#include "../tl_promote.h"
#include "../intersect/tl_intersect.h"
#include "../closest_point/tl_closest_point.h"
#include "../fitting_loft_sweep/tl_fitting_loft_sweep.h"
#include "../meshing/tl_meshing.h"
#include "../booleans/tl_booleans.h"
#include "../subd/tl_subd.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace opentl::mvp {

// ============================================================================
// 1. RhinoCommon-aligned Value & Geometry Types backed by openNURBS
// ============================================================================
using Point3d     = ON_3dPoint;
using Vector3d    = ON_3dVector;
using Line        = ON_Line;
using Polyline    = ON_Polyline;
using Circle      = ON_Circle;
using Arc         = ON_Arc;
using Plane       = ON_Plane;
using BoundingBox = ON_BoundingBox;
using NurbsCurve  = ON_NurbsCurve;
using Transform   = ON_Xform;

enum class ObjectKind : int {
  Point      = 1,
  Line       = 2,
  Polyline   = 3,
  Circle     = 4,
  Arc        = 5,
  Plane      = 6,
  NurbsCurve = 7
};

TL_FUNCTION const char* ObjectKindToString(ObjectKind kind);
TL_FUNCTION bool ObjectKindFromString(const std::string& str, ObjectKind* out_kind);

struct Layer {
  int index = 0;
  std::string name = "Default";
  ON_UUID id = ON_nil_uuid;
  int color_r = 0;
  int color_g = 0;
  int color_b = 0;
  bool visible = true;
};

/**
 *Deterministic UUID generator from a 64-bit integer seed so objects and layers
 * have stable, reproducible IDs across runs and serialization formats.
 */
TL_FUNCTION ON_UUID MakeDeterministicUuid(uint64_t seed);
TL_FUNCTION std::string UuidToString(const ON_UUID& id);
TL_FUNCTION ON_UUID StringToUuid(const std::string& str);

struct SceneObject {
  uint64_t id = 0;
  ON_UUID uuid = ON_nil_uuid;
  std::string name;
  int layer_index = 0;
  ObjectKind kind = ObjectKind::Point;

  // Value payloads for exact Rhino-like geometric primitives
  Point3d point_val = Point3d::Origin;
  Line line_val;
  Polyline polyline_val;
  Circle circle_val;
  Arc arc_val;
  Plane plane_val = Plane::World_xy;
  NurbsCurve nurbs_val;

  /**
   * Compute tight 3D axis-aligned BoundingBox for this object.
   */
  BoundingBox GetBoundingBox() const;

  /**
   * Apply a 4x4 affine/similarity transformation to this object in-place.
   */
  bool ApplyTransform(const Transform& xform);

  /**
   * Convert curve-like objects (Line, Polyline, Circle, Arc, NurbsCurve) into
   * an equivalent ON_NurbsCurve representation. Returns false for Point/Plane.
   */
  bool ToNurbsCurve(NurbsCurve& out_curve) const;

  /**
   * Compute minimum 3D Euclidean distance from this object to a test point.
   */
  double DistanceTo(const Point3d& pt) const;
};

// ============================================================================
// 2. Intersection & Curve Sampling Results (implemented in intersect_sample.cpp)
// ============================================================================

struct LineLineIntersection {
  bool intersects = false;        // True if solver succeeded and within tolerance (when finite/tol set)
  bool is_parallel = false;       // True if lines are parallel/collinear within angle tolerance
  double param_a = 0.0;           // Parameter on line A (0..1 is on segment)
  double param_b = 0.0;           // Parameter on line B (0..1 is on segment)
  Point3d point_a = Point3d::Origin;
  Point3d point_b = Point3d::Origin;
  double distance = 0.0;          // Chord distance between point_a and point_b
};

struct LinePlaneIntersection {
  bool intersects = false;        // True if line is not parallel to plane (and within segment if finite)
  bool is_parallel = false;       // True if line is parallel to plane
  int status = 0;                 // TL_3DX_LineXPlane: -2 coplanar, -1 parallel, 0 t==0, 1 unique
  double line_param = 0.0;        // Parameter t along line: P(t) = from + t*(to - from)
  double plane_u = 0.0;           // Local plane U coordinate
  double plane_v = 0.0;           // Local plane V coordinate
  Point3d point = Point3d::Origin;
};

struct CurveSamplePoint {
  double t = 0.0;                 // Curve parameter
  double normalized_t = 0.0;      // Normalized parameter in [0, 1]
  double arc_length = 0.0;        // Cumulative chord/arc length from start
  Point3d point = Point3d::Origin;
  Vector3d tangent = Vector3d::ZeroVector;
};

/**
 * Intersect two 3D lines (matches RhinoCommon Intersection.LineLine -> rhcommon_c!ON_Intersect_LineLine
 * & tl.dll!TL_GetClosestPoint(ON_Line, ON_Line)).
 *
 * @param line_a           First 3D line
 * @param line_b           Second 3D line
 * @param tolerance        Maximum distance between closest points to count as intersecting (<= 0 means any closest point)
 * @param finite_segments  If true, clamps parameters to [0, 1] segment bounds
 */
TL_FUNCTION LineLineIntersection IntersectLineLine(
  const Line& line_a,
  const Line& line_b,
  double tolerance = 1e-6,
  bool finite_segments = false
);

/**
 * Intersect a 3D line with a 3D plane (matches RhinoCommon Intersection.LinePlane ->
 * rhcommon_c!ON_Intersect_LinePlane & tl.dll!TL_3DX_LineXPlane).
 */
TL_FUNCTION LinePlaneIntersection IntersectLinePlane(
  const Line& line,
  const Plane& plane,
  bool finite_segment = false,
  double tolerance = 1e-9
);

/**
 * Sample a general ON_Curve (or ON_NurbsCurve) uniformly by parameter into `sample_count` points (>= 2).
 */
TL_FUNCTION std::vector<CurveSamplePoint> SampleCurveByCount(
  const ON_Curve& curve,
  int sample_count,
  bool by_arc_length = false
);

/**
 * Create a cubic (or given degree) interpolating or control-polygon NURBS curve from 3D control points.
 */
TL_FUNCTION bool CreateClampedNurbsCurve(
  const std::vector<Point3d>& control_points,
  int degree,
  NurbsCurve& out_curve
);

// ============================================================================
// 3. Transform Builders & Distance Queries (implemented in geometry_scene.cpp)
// ============================================================================

TL_FUNCTION Transform MakeTranslation(double dx, double dy, double dz);
TL_FUNCTION Transform MakeTranslation(const Vector3d& delta);
TL_FUNCTION Transform MakeRotation(double angle_radians, const Vector3d& axis, const Point3d& center = Point3d::Origin);
TL_FUNCTION Transform MakeScale(const Point3d& anchor, double sx, double sy, double sz);
TL_FUNCTION Transform MakeUniformScale(const Point3d& anchor, double scale_factor);

TL_FUNCTION double DistanceBetweenObjects(const SceneObject& a, const SceneObject& b);

// ============================================================================
// 4. Rhino-like Scene Document (Create / Delete / Query / Transform / BBox / IO)
// ============================================================================

class TL_CLASS SceneDocument {
public:
  SceneDocument();
  ~SceneDocument() = default;

  void Clear();

  // Layer management (stable index + name + UUID)
  int AddLayer(const std::string& name, int r = 0, int g = 0, int b = 0, bool visible = true);
  int FindLayerByName(const std::string& name) const;
  const Layer* GetLayer(int layer_index) const;
  const std::vector<Layer>& Layers() const { return layers_; }

  // Object creation (returns stable 64-bit ID >= 1)
  uint64_t AddPoint(const Point3d& pt, const std::string& name = "", const std::string& layer = "Default");
  uint64_t AddLine(const Line& line, const std::string& name = "", const std::string& layer = "Default");
  uint64_t AddPolyline(const std::vector<Point3d>& pts, const std::string& name = "", const std::string& layer = "Default");
  uint64_t AddCircle(const Circle& circle, const std::string& name = "", const std::string& layer = "Default");
  uint64_t AddArc(const Arc& arc, const std::string& name = "", const std::string& layer = "Default");
  uint64_t AddPlane(const Plane& plane, const std::string& name = "", const std::string& layer = "Default");
  uint64_t AddNurbsCurve(const NurbsCurve& curve, const std::string& name = "", const std::string& layer = "Default");
  uint64_t AddNurbsCurveFromPoints(const std::vector<Point3d>& control_pts, int degree, const std::string& name = "", const std::string& layer = "Default");

  // Delete & Query
  bool DeleteObject(uint64_t id);
  const SceneObject* FindById(uint64_t id) const;
  SceneObject* FindByIdMutable(uint64_t id);
  const SceneObject* FindByUuid(const ON_UUID& uuid) const;
  std::vector<const SceneObject*> FindByName(const std::string& name) const;
  std::vector<const SceneObject*> FindByLayer(const std::string& layer_name) const;
  std::vector<const SceneObject*> FindByKind(ObjectKind kind) const;
  const std::vector<SceneObject>& Objects() const { return objects_; }
  size_t ObjectCount() const { return objects_.size(); }

  // Transformations on objects by ID
  bool TransformObject(uint64_t id, const Transform& xform);
  bool TranslateObject(uint64_t id, const Vector3d& delta);
  bool RotateObject(uint64_t id, double angle_radians, const Vector3d& axis, const Point3d& center = Point3d::Origin);
  bool ScaleObject(uint64_t id, double sx, double sy, double sz, const Point3d& anchor = Point3d::Origin);

  // Scene-wide Bounding Box
  BoundingBox GetSceneBoundingBox() const;

  // Native deterministic JSON scene serialization (.otscene / .json)
  bool SaveJsonScene(const std::string& file_path, std::string* error_out = nullptr) const;
  bool LoadJsonScene(const std::string& file_path, std::string* error_out = nullptr);

  // OpenNURBS .3dm file read/write integration via ONX_Model
  // Note: ON_Plane is persisted as an ON_PlaneSurface with [-10,10]^2 domain and a user string tag
  //       so that planes round-trip losslessly alongside Points, Lines, Polylines, Circles, Arcs, and NurbsCurves.
  bool Save3dm(const std::string& file_path, std::string* error_out = nullptr) const;
  bool Load3dm(const std::string& file_path, std::string* error_out = nullptr);

  // Deterministic multi-line summary for CLI demo & regression testing
  std::string FormatDeterministicSummary() const;

  // Low-level insertion for deserializers preserving original stable ID & UUID
  uint64_t InsertDeserializedObject(SceneObject obj);
  void SetLayersForDeserialization(std::vector<Layer> layers);

private:
  int EnsureLayerIndex(const std::string& layer_name);
  uint64_t AllocateId();

  std::vector<Layer> layers_;
  std::vector<SceneObject> objects_;
  uint64_t next_id_ = 1;
};

} // namespace opentl::mvp
