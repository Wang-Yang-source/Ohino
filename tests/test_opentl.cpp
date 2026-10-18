/**
 * @file test_opentl.cpp
 * @brief OpenTL MVP automated test suite.
 *
 * The suite is intentionally dependency free (no gtest / catch) so it builds
 * with the plain `opentl` static library target. Every section prints its own
 * pass/fail tally and the process exits non zero when anything fails.
 *
 * Coverage map
 *   01 geometry value types (Point3d / Vector3d / Line / Polyline / Circle /
 *      Arc / Plane / BoundingBox / NurbsCurve)
 *   02 scene document: layers, stable ids, uuids, names
 *   03 queries (id / uuid / name / layer / kind) and deletion
 *   04 transformations (translate / rotate / scale) and bounding boxes
 *   05 distance queries (point to object, object to object)
 *   06 line-line intersections (crossing / skew / parallel / finite clip)
 *   07 line-plane intersections (crossing / parallel / finite segment)
 *   08 curve-line and curve-plane intersections
 *   09 NURBS creation, evaluation, uniform + arc length sampling, closest points
 *   10 reverse engineered helpers (TL_Brep::PointIsOnFace, TL_Brep_PointIsOnFace,
 *      TL_IsMeshEdgeUnwelded, TLC_IsMeshEdgeUnwelded, TL_DeformableArc2,
 *      TL_DeformableCircle, TL_NurbsCurve::RemoveKnots, TL_LoftCurves,
 *      TL_CreateSubDBox)
 *   11 local JSON scene round-trip
 *   12 openNURBS .3dm round-trip
 */

#include "opentl/opentl.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

// ============================================================================
// Tiny assertion / reporting framework
// ============================================================================

struct TestState {
  const char* section_name = "";
  int section_index = 0;
  int section_checks = 0;
  int section_failures = 0;
  int total_checks = 0;
  int total_failures = 0;
  int sections_with_failures = 0;
};

TestState g_state;

void BeginSection(const char* name) {
  g_state.section_name = name;
  ++g_state.section_index;
  g_state.section_checks = 0;
  g_state.section_failures = 0;
  std::printf("\n[%02d] %s\n", g_state.section_index, name);
}

void EndSection() {
  g_state.total_checks += g_state.section_checks;
  g_state.total_failures += g_state.section_failures;
  if (g_state.section_failures > 0) ++g_state.sections_with_failures;
  std::printf("     %s  checks=%d failures=%d\n",
              g_state.section_failures == 0 ? "PASS" : "FAIL", g_state.section_checks,
              g_state.section_failures);
}

void RecordResult(bool ok) {
  ++g_state.section_checks;
  if (!ok) ++g_state.section_failures;
}

void CheckImpl(bool ok, const char* expression, const char* file, int line,
               const std::string& detail) {
  RecordResult(ok);
  if (!ok) {
    std::printf("     [FAIL] %s:%d  %s", file, line, expression);
    if (!detail.empty()) std::printf("  (%s)", detail.c_str());
    std::printf("\n");
  }
}

std::string NumberText(double value) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  return std::string(buffer);
}

void CheckNearImpl(double actual, double expected, double tolerance, const char* expression,
                   const char* file, int line) {
  const bool ok = std::isfinite(actual) && std::fabs(actual - expected) <= tolerance;
  RecordResult(ok);
  if (!ok) {
    std::printf("     [FAIL] %s:%d  %s  actual=%s expected=%s tol=%s\n", file, line,
                expression, NumberText(actual).c_str(), NumberText(expected).c_str(),
                NumberText(tolerance).c_str());
  }
}

void CheckPointImpl(const ON_3dPoint& actual, const ON_3dPoint& expected, double tolerance,
                    const char* expression, const char* file, int line) {
  const double distance = actual.DistanceTo(expected);
  const bool ok = std::isfinite(distance) && distance <= tolerance;
  RecordResult(ok);
  if (!ok) {
    std::printf("     [FAIL] %s:%d  %s  actual=(%s,%s,%s) expected=(%s,%s,%s) distance=%s\n",
                file, line, expression, NumberText(actual.x).c_str(),
                NumberText(actual.y).c_str(), NumberText(actual.z).c_str(),
                NumberText(expected.x).c_str(), NumberText(expected.y).c_str(),
                NumberText(expected.z).c_str(), NumberText(distance).c_str());
  }
}

#define CHECK(expr) CheckImpl((expr) ? true : false, #expr, __FILE__, __LINE__, std::string())
#define CHECK_MSG(expr, msg) \
  CheckImpl((expr) ? true : false, #expr, __FILE__, __LINE__, std::string(msg))
#define CHECK_NEAR(actual, expected, tolerance) \
  CheckNearImpl((actual), (expected), (tolerance), #actual " ~= " #expected, __FILE__, __LINE__)
#define CHECK_POINT(actual, expected, tolerance) \
  CheckPointImpl((actual), (expected), (tolerance), #actual " ~= " #expected, __FILE__, __LINE__)
#define CHECK_EQ_INT(actual, expected)                                             \
  CheckImpl((actual) == (expected), #actual " == " #expected, __FILE__, __LINE__,  \
            std::string("actual=") + std::to_string(static_cast<long long>(actual)) + \
                " expected=" + std::to_string(static_cast<long long>(expected)))

// ============================================================================
// Shared helpers
// ============================================================================

int EnsureLayer(opentl::mvp::SceneDocument& document, const std::string& name, int r, int g,
                int b) {
  const int existing = document.FindLayerByName(name);
  if (existing >= 0) return existing;
  return document.AddLayer(name, r, g, b, true);
}

opentl::mvp::NurbsCurve MakeCubicWave() {
  opentl::mvp::NurbsCurve curve;
  std::vector<opentl::mvp::Point3d> points;
  points.push_back(opentl::mvp::Point3d(0.0, 0.0, 0.0));
  points.push_back(opentl::mvp::Point3d(3.0, 6.0, 0.0));
  points.push_back(opentl::mvp::Point3d(6.0, -6.0, 0.0));
  points.push_back(opentl::mvp::Point3d(9.0, 0.0, 0.0));
  points.push_back(opentl::mvp::Point3d(12.0, 6.0, 0.0));
  if (!opentl::mvp::CreateClampedNurbsCurve(points, 3, curve)) {
    curve = opentl::mvp::NurbsCurve();
  }
  return curve;
}

bool BuildTwoTriangleMesh(ON_Mesh& mesh, bool welded) {
  if (welded) {
    mesh = ON_Mesh(2, 4, false, false);
    if (!mesh.SetVertex(0, ON_3dPoint(0.0, 0.0, 0.0))) return false;
    if (!mesh.SetVertex(1, ON_3dPoint(1.0, 0.0, 0.0))) return false;
    if (!mesh.SetVertex(2, ON_3dPoint(1.0, 1.0, 0.0))) return false;
    if (!mesh.SetVertex(3, ON_3dPoint(0.0, 1.0, 0.0))) return false;
    if (!mesh.SetTriangle(0, 0, 1, 2)) return false;
    if (!mesh.SetTriangle(1, 0, 2, 3)) return false;
  } else {
    mesh = ON_Mesh(2, 6, false, false);
    if (!mesh.SetVertex(0, ON_3dPoint(0.0, 0.0, 0.0))) return false;
    if (!mesh.SetVertex(1, ON_3dPoint(1.0, 0.0, 0.0))) return false;
    if (!mesh.SetVertex(2, ON_3dPoint(0.0, 1.0, 0.0))) return false;
    if (!mesh.SetVertex(3, ON_3dPoint(0.0, 0.0, 0.0))) return false;
    if (!mesh.SetVertex(4, ON_3dPoint(1.0, 0.0, 0.0))) return false;
    if (!mesh.SetVertex(5, ON_3dPoint(1.0, 1.0, 0.0))) return false;
    if (!mesh.SetTriangle(0, 0, 1, 2)) return false;
    if (!mesh.SetTriangle(1, 3, 4, 5)) return false;
  }
  return true;
}

int FirstInteriorTopologyEdge(const ON_Mesh& mesh) {
  const ON_MeshTopology& topology = mesh.Topology();
  for (int i = 0; i < topology.m_tope.Count(); ++i) {
    if (topology.m_tope[i].m_topf_count >= 2) return i;
  }
  return -1;
}

// ---------------------------------------------------------------------------
// Scene comparison used by the serialization tests
// ---------------------------------------------------------------------------

bool PointsClose(const ON_3dPoint& a, const ON_3dPoint& b, double tolerance) {
  return std::isfinite(a.DistanceTo(b)) && a.DistanceTo(b) <= tolerance;
}

bool VectorsClose(const ON_3dVector& a, const ON_3dVector& b, double tolerance) {
  return std::isfinite((a - b).Length()) && (a - b).Length() <= tolerance;
}

bool PlanesClose(const ON_Plane& a, const ON_Plane& b, double tolerance) {
  return PointsClose(a.origin, b.origin, tolerance) &&
         VectorsClose(a.xaxis, b.xaxis, tolerance) &&
         VectorsClose(a.yaxis, b.yaxis, tolerance) &&
         VectorsClose(a.zaxis, b.zaxis, tolerance);
}

std::string PayloadMismatch(const opentl::mvp::SceneObject& a,
                            const opentl::mvp::SceneObject& b, double tolerance) {
  using namespace opentl::mvp;
  if (a.kind != b.kind) return "kind differs";
  switch (a.kind) {
    case ObjectKind::Point:
      if (!PointsClose(a.point_val, b.point_val, tolerance)) return "point payload differs";
      return std::string();
    case ObjectKind::Line:
      if (!PointsClose(a.line_val.from, b.line_val.from, tolerance)) return "line.from differs";
      if (!PointsClose(a.line_val.to, b.line_val.to, tolerance)) return "line.to differs";
      return std::string();
    case ObjectKind::Polyline:
      if (a.polyline_val.PointCount() != b.polyline_val.PointCount()) {
        return "polyline point count differs";
      }
      for (int i = 0; i < a.polyline_val.PointCount(); ++i) {
        if (!PointsClose(a.polyline_val[i], b.polyline_val[i], tolerance)) {
          return "polyline point differs";
        }
      }
      return std::string();
    case ObjectKind::Circle:
      if (!PlanesClose(a.circle_val.plane, b.circle_val.plane, tolerance)) {
        return "circle plane differs";
      }
      if (std::fabs(a.circle_val.radius - b.circle_val.radius) > tolerance) {
        return "circle radius differs";
      }
      return std::string();
    case ObjectKind::Arc:
      if (!PlanesClose(a.arc_val.plane, b.arc_val.plane, tolerance)) return "arc plane differs";
      if (std::fabs(a.arc_val.radius - b.arc_val.radius) > tolerance) {
        return "arc radius differs";
      }
      if (std::fabs(a.arc_val.AngleRadians() - b.arc_val.AngleRadians()) > tolerance) {
        return "arc angle differs";
      }
      if (!PointsClose(a.arc_val.StartPoint(), b.arc_val.StartPoint(), tolerance)) {
        return "arc start differs";
      }
      return std::string();
    case ObjectKind::Plane:
      if (!PlanesClose(a.plane_val, b.plane_val, tolerance)) return "plane payload differs";
      return std::string();
    case ObjectKind::NurbsCurve: {
      if (a.nurbs_val.m_dim != b.nurbs_val.m_dim) return "nurbs dim differs";
      if (a.nurbs_val.m_is_rat != b.nurbs_val.m_is_rat) return "nurbs is_rat differs";
      if (a.nurbs_val.m_order != b.nurbs_val.m_order) return "nurbs order differs";
      if (a.nurbs_val.m_cv_count != b.nurbs_val.m_cv_count) return "nurbs cv_count differs";
      if (a.nurbs_val.KnotCount() != b.nurbs_val.KnotCount()) return "nurbs knot count differs";
      for (int i = 0; i < a.nurbs_val.KnotCount(); ++i) {
        if (std::fabs(a.nurbs_val.Knot(i) - b.nurbs_val.Knot(i)) > tolerance) {
          return "nurbs knot differs";
        }
      }
      for (int i = 0; i < a.nurbs_val.m_cv_count; ++i) {
        ON_3dPoint ca = ON_3dPoint::Origin;
        ON_3dPoint cb = ON_3dPoint::Origin;
        if (!a.nurbs_val.GetCV(i, ca) || !b.nurbs_val.GetCV(i, cb)) return "nurbs cv read failed";
        if (!PointsClose(ca, cb, tolerance)) return "nurbs cv differs";
      }
      return std::string();
    }
  }
  return std::string();
}

std::string ScenesMismatch(const opentl::mvp::SceneDocument& a,
                           const opentl::mvp::SceneDocument& b, double tolerance) {
  if (a.Layers().size() != b.Layers().size()) return "layer count differs";
  for (size_t i = 0; i < a.Layers().size(); ++i) {
    const opentl::mvp::Layer& la = a.Layers()[i];
    const opentl::mvp::Layer& lb = b.Layers()[i];
    if (la.name != lb.name) return "layer name differs at index " + std::to_string(i);
    if (0 != ON_UuidCompare(la.id, lb.id)) return "layer uuid differs at index " + std::to_string(i);
    if (la.color_r != lb.color_r || la.color_g != lb.color_g || la.color_b != lb.color_b) {
      return "layer color differs at index " + std::to_string(i);
    }
    if (la.visible != lb.visible) return "layer visibility differs at index " + std::to_string(i);
  }

  if (a.ObjectCount() != b.ObjectCount()) return "object count differs";
  for (size_t i = 0; i < a.Objects().size(); ++i) {
    const opentl::mvp::SceneObject& oa = a.Objects()[i];
    const opentl::mvp::SceneObject& ob = b.Objects()[i];
    if (oa.id != ob.id) return "object id differs at index " + std::to_string(i);
    if (0 != ON_UuidCompare(oa.uuid, ob.uuid)) {
      return "object uuid differs for id " + std::to_string(oa.id);
    }
    if (oa.name != ob.name) return "object name differs for id " + std::to_string(oa.id);
    if (oa.layer_index != ob.layer_index) {
      return "object layer differs for id " + std::to_string(oa.id);
    }
    const std::string payload = PayloadMismatch(oa, ob, tolerance);
    if (!payload.empty()) return payload + " (id " + std::to_string(oa.id) + ")";
  }
  return std::string();
}

// ---------------------------------------------------------------------------
// Demo scene shared by the serialization tests
// ---------------------------------------------------------------------------

struct TestScene {
  opentl::mvp::SceneDocument document;
  std::vector<uint64_t> ids;
};

void BuildSerializationScene(TestScene& scene) {
  using namespace opentl::mvp;
  SceneDocument& document = scene.document;
  document.Clear();

  // Guard against the default document already owning a "Default" layer.
  if (document.FindLayerByName("Default") < 0) {
    document.AddLayer("Default", 0, 0, 0, true);
  }
  document.AddLayer("Construction", 128, 128, 128, true);
  document.AddLayer("Curves", 0, 0, 255, true);
  document.AddLayer("Reference", 255, 0, 0, true);

  scene.ids.push_back(document.AddPoint(Point3d(1.5, -2.25, 3.75), "Point_A", "Construction"));

  Line line;
  line.from = Point3d(-1.0, 2.0, 0.5);
  line.to = Point3d(7.5, 2.0, 0.5);
  scene.ids.push_back(document.AddLine(line, "Line_B", "Curves"));

  std::vector<Point3d> polyline_points;
  polyline_points.push_back(Point3d(0.0, 0.0, 0.0));
  polyline_points.push_back(Point3d(2.0, 3.0, 0.0));
  polyline_points.push_back(Point3d(4.0, 0.0, 1.0));
  polyline_points.push_back(Point3d(6.0, 3.0, 1.0));
  scene.ids.push_back(document.AddPolyline(polyline_points, "Polyline_C", "Curves"));

  Circle circle;
  circle.Create(Point3d(2.0, 1.0, -1.0), 4.25);
  scene.ids.push_back(document.AddCircle(circle, "Circle_D", "Curves"));

  Arc arc(Plane::World_xy, 3.5, ON_PI * 0.75);
  arc.Translate(Vector3d(1.0, 1.0, 2.0));
  scene.ids.push_back(document.AddArc(arc, "Arc_E", "Curves"));

  Plane plane(Point3d(0.0, 0.0, 2.0), Vector3d(0.0, 1.0, 0.0));
  scene.ids.push_back(document.AddPlane(plane, "Plane_F", "Reference"));

  std::vector<Point3d> control_points;
  control_points.push_back(Point3d(0.0, 0.0, 0.0));
  control_points.push_back(Point3d(3.0, 6.0, 0.0));
  control_points.push_back(Point3d(6.0, -6.0, 0.0));
  control_points.push_back(Point3d(9.0, 0.0, 0.0));
  control_points.push_back(Point3d(12.0, 6.0, 0.0));
  scene.ids.push_back(
      document.AddNurbsCurveFromPoints(control_points, 3, "Nurbs_G", "Curves"));
}

// ============================================================================
// 01 - geometry value types
// ============================================================================

void TestGeometryTypes() {
  using namespace opentl::mvp;
  BeginSection("geometry value types (Point3d / Vector3d / BoundingBox / 7 kinds)");

  // ---- Vector3d algebra ------------------------------------------------
  const Vector3d a(1.0, 2.0, 3.0);
  const Vector3d b(4.0, -5.0, 6.0);
  CHECK_NEAR(a.Length(), std::sqrt(14.0), 1e-12);
  CHECK_NEAR((a + b).x, 5.0, 1e-12);
  CHECK_NEAR((a - b).y, 7.0, 1e-12);
  CHECK_NEAR((a * b), 1.0 * 4.0 + 2.0 * -5.0 + 3.0 * 6.0, 1e-12);
  const Vector3d cross = Vector3d::CrossProduct(Vector3d(1.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
  CHECK_NEAR(cross.z, 1.0, 1e-12);
  Vector3d unit = a;
  CHECK(unit.Unitize());
  CHECK_NEAR(unit.Length(), 1.0, 1e-12);

  // ---- Point3d ---------------------------------------------------------
  const Point3d p0(1.0, 2.0, 3.0);
  const Point3d p1(4.0, 6.0, 3.0);
  CHECK_NEAR(p0.DistanceTo(p1), 5.0, 1e-12);
  CHECK_POINT(p0 + Vector3d(1.0, -1.0, 1.0), Point3d(2.0, 1.0, 4.0), 1e-12);

  // ---- BoundingBox -----------------------------------------------------
  BoundingBox box;
  box.Set(p0);
  box.Set(p1, true);
  CHECK(box.IsSet());
  CHECK_POINT(box.Min(), Point3d(1.0, 2.0, 3.0), 1e-12);
  CHECK_POINT(box.Max(), Point3d(4.0, 6.0, 3.0), 1e-12);
  CHECK_NEAR(box.Diagonal().x, 3.0, 1e-12);
  CHECK_NEAR(box.Volume(), 0.0, 1e-12);
  CHECK(box.IsPointIn(Point3d(2.0, 3.0, 3.0), false));

  // ---- Line ------------------------------------------------------------
  const Line line(Point3d(0.0, 0.0, 0.0), Point3d(3.0, 4.0, 0.0));
  CHECK(line.IsValid());
  CHECK_NEAR(line.Length(), 5.0, 1e-12);
  CHECK_POINT(line.PointAt(0.5), Point3d(1.5, 2.0, 0.0), 1e-12);
  CHECK_NEAR(line.DistanceTo(Point3d(0.0, 0.0, 7.0)), 7.0, 1e-12);

  // ---- Polyline --------------------------------------------------------
  Polyline polyline;
  polyline.Append(Point3d(0.0, 0.0, 0.0));
  polyline.Append(Point3d(3.0, 0.0, 0.0));
  polyline.Append(Point3d(3.0, 4.0, 0.0));
  CHECK_EQ_INT(polyline.PointCount(), 3);
  CHECK_NEAR(polyline.Length(), 7.0, 1e-12);
  CHECK(polyline.IsValid());

  // ---- Circle ----------------------------------------------------------
  Circle circle;
  CHECK(circle.Create(Point3d(1.0, 2.0, 3.0), 2.5));
  CHECK(circle.IsValid());
  CHECK_NEAR(circle.Radius(), 2.5, 1e-12);
  CHECK_NEAR(circle.Circumference(), 2.0 * ON_PI * 2.5, 1e-9);
  CHECK_POINT(circle.PointAt(0.0), Point3d(3.5, 2.0, 3.0), 1e-9);

  // ---- Arc -------------------------------------------------------------
  Arc arc(Plane::World_xy, 2.0, ON_PI * 0.5);
  CHECK(arc.IsValid());
  CHECK_NEAR(arc.AngleRadians(), ON_PI * 0.5, 1e-12);
  CHECK_NEAR(arc.AngleDegrees(), 90.0, 1e-9);
  CHECK_NEAR(arc.Length(), ON_PI, 1e-9);
  CHECK_POINT(arc.StartPoint(), Point3d(2.0, 0.0, 0.0), 1e-9);
  CHECK_POINT(arc.EndPoint(), Point3d(0.0, 2.0, 0.0), 1e-9);

  // ---- Plane -----------------------------------------------------------
  const Plane plane(Point3d(0.0, 0.0, 5.0), Vector3d(0.0, 0.0, 1.0));
  CHECK(plane.IsValid());
  CHECK_POINT(plane.Origin(), Point3d(0.0, 0.0, 5.0), 1e-12);
  CHECK_NEAR(plane.DistanceTo(Point3d(0.0, 0.0, 9.0)), 4.0, 1e-12);
  CHECK_POINT(plane.PointAt(2.0, 3.0), Point3d(2.0, 3.0, 5.0), 1e-9);

  // ---- NurbsCurve ------------------------------------------------------
  const NurbsCurve wave = MakeCubicWave();
  CHECK_EQ_INT(wave.m_dim, 3);
  CHECK_EQ_INT(wave.m_order, 4);
  CHECK_EQ_INT(wave.m_cv_count, 5);
  CHECK(wave.IsValid());
  CHECK_POINT(wave.PointAt(wave.Domain()[0]), Point3d(0.0, 0.0, 0.0), 1e-9);
  CHECK_POINT(wave.PointAt(wave.Domain()[1]), Point3d(12.0, 6.0, 0.0), 1e-9);

  EndSection();
}

// ============================================================================
// 02 - scene document: layers, ids, uuids, names
// ============================================================================

void TestSceneDocumentBasics() {
  using namespace opentl::mvp;
  BeginSection("scene document: layers, stable ids, uuids, names");

  SceneDocument document;
  const int default_layer = EnsureLayer(document, "Default", 0, 0, 0);
  const int construction = document.AddLayer("Construction", 128, 128, 128, true);
  const int curves = document.AddLayer("Curves", 0, 0, 255, true);
  const int reference = document.AddLayer("Reference", 255, 0, 0, true);
  (void)default_layer;

  CHECK_EQ_INT(document.FindLayerByName("Default"), default_layer);
  CHECK_EQ_INT(document.FindLayerByName("Construction"), construction);
  CHECK_EQ_INT(document.FindLayerByName("Curves"), curves);
  CHECK_EQ_INT(document.FindLayerByName("Reference"), reference);
  CHECK_EQ_INT(document.FindLayerByName("DoesNotExist"), -1);
  CHECK_EQ_INT(static_cast<int>(document.Layers().size()), 4);

  const Layer* layer = document.GetLayer(construction);
  CHECK(nullptr != layer);
  if (nullptr != layer) {
    CHECK(layer->name == "Construction");
    CHECK_EQ_INT(layer->color_r, 128);
    CHECK_EQ_INT(layer->color_g, 128);
    CHECK_EQ_INT(layer->color_b, 128);
    CHECK(layer->visible);
  }

  const uint64_t point_id = document.AddPoint(Point3d(1.0, 2.0, 3.0), "Point_A", "Construction");
  const uint64_t line_id = document.AddLine(Line(Point3d(0.0, 0.0, 0.0), Point3d(1.0, 0.0, 0.0)),
                                            "Line_B", "Curves");
  const uint64_t circle_id = document.AddCircle(Circle(Point3d::Origin, 2.0), "Circle_C", "Curves");

  CHECK(point_id >= 1);
  CHECK(line_id > point_id);
  CHECK(circle_id > line_id);
  CHECK_EQ_INT(static_cast<int>(document.ObjectCount()), 3);

  const SceneObject* point = document.FindById(point_id);
  CHECK(nullptr != point);
  if (nullptr != point) {
    CHECK(point->id == point_id);
    CHECK(point->name == "Point_A");
    CHECK_EQ_INT(point->layer_index, construction);
    CHECK(point->kind == ObjectKind::Point);
    CHECK(point->uuid != ON_nil_uuid);
    CHECK_POINT(point->point_val, Point3d(1.0, 2.0, 3.0), 1e-12);
  }

  // Stable ids and deterministic uuids.
  CHECK(point_id == 1);
  CHECK(line_id == 2);
  CHECK(circle_id == 3);
  const ON_UUID same_seed_a = MakeDeterministicUuid(42);
  const ON_UUID same_seed_b = MakeDeterministicUuid(42);
  CHECK(0 == ON_UuidCompare(same_seed_a, same_seed_b));
  CHECK(0 != ON_UuidCompare(MakeDeterministicUuid(42), MakeDeterministicUuid(43)));

  // No two objects share a uuid.
  CHECK(0 != ON_UuidCompare(document.Objects()[0].uuid, document.Objects()[1].uuid));
  CHECK(0 != ON_UuidCompare(document.Objects()[1].uuid, document.Objects()[2].uuid));

  // Every kind is creatable and reports its BoundingBox.
  const int nurbs_layer = EnsureLayer(document, "Curves", 0, 0, 255);
  const uint64_t polyline_id = document.AddPolyline(
      std::vector<Point3d>{Point3d(0.0, 0.0, 0.0), Point3d(1.0, 1.0, 0.0), Point3d(2.0, 0.0, 0.0)},
      "Polyline_D", "Curves");
  const uint64_t arc_id = document.AddArc(Arc(Plane::World_xy, 1.0, ON_PI * 0.5), "Arc_E", "Curves");
  const uint64_t plane_id = document.AddPlane(Plane::World_xy, "Plane_F", "Reference");
  const uint64_t nurbs_id = document.AddNurbsCurveFromPoints(
      std::vector<Point3d>{Point3d(0, 0, 0), Point3d(1, 2, 0), Point3d(2, -2, 0), Point3d(3, 0, 0)}, 3,
      "Nurbs_G", "Curves");
  (void)nurbs_layer;

  CHECK(polyline_id > circle_id);
  CHECK(arc_id > polyline_id);
  CHECK(plane_id > arc_id);
  CHECK(nurbs_id > plane_id);
  CHECK_EQ_INT(static_cast<int>(document.ObjectCount()), 7);

  const SceneObject* polyline = document.FindById(polyline_id);
  CHECK(nullptr != polyline);
  if (nullptr != polyline) {
    CHECK(polyline->kind == ObjectKind::Polyline);
    CHECK_EQ_INT(polyline->polyline_val.PointCount(), 3);
  }

  const SceneObject* arc = document.FindById(arc_id);
  CHECK(nullptr != arc);
  if (nullptr != arc) {
    CHECK(arc->kind == ObjectKind::Arc);
    CHECK_NEAR(arc->arc_val.AngleDegrees(), 90.0, 1e-9);
  }

  const SceneObject* plane = document.FindById(plane_id);
  CHECK(nullptr != plane);
  if (nullptr != plane) {
    CHECK(plane->kind == ObjectKind::Plane);
    CHECK(plane->plane_val.IsValid());
  }

  for (size_t i = 0; i < document.Objects().size(); ++i) {
    const BoundingBox object_box = document.Objects()[i].GetBoundingBox();
    CHECK_MSG(object_box.IsSet(), "every object must have a bounding box");
  }

  const BoundingBox scene_box = document.GetSceneBoundingBox();
  CHECK(scene_box.IsSet());
  CHECK(scene_box.IsNotEmpty());
  CHECK(scene_box.Min().x <= 0.0 + 1e-9);
  CHECK(scene_box.Max().x >= 3.0 - 1e-9);

  // Kind <-> string round-trip for all 7 kinds.
  const ObjectKind kinds[7] = {ObjectKind::Point,   ObjectKind::Line,   ObjectKind::Polyline,
                               ObjectKind::Circle,  ObjectKind::Arc,    ObjectKind::Plane,
                               ObjectKind::NurbsCurve};
  for (int i = 0; i < 7; ++i) {
    const char* text = ObjectKindToString(kinds[i]);
    CHECK(text != nullptr);
    ObjectKind parsed = ObjectKind::Point;
    CHECK(ObjectKindFromString(std::string(text == nullptr ? "" : text), &parsed));
    CHECK(parsed == kinds[i]);
  }

  EndSection();
}

// ============================================================================
// 03 - queries and deletion
// ============================================================================

void TestQueriesAndDelete() {
  using namespace opentl::mvp;
  BeginSection("queries by id / uuid / name / layer / kind and deletion");

  TestScene scene;
  BuildSerializationScene(scene);
  SceneDocument& document = scene.document;

  CHECK_EQ_INT(static_cast<int>(document.ObjectCount()), 7);

  // By id.
  const SceneObject* by_id = document.FindById(scene.ids[2]);
  CHECK(nullptr != by_id);
  if (nullptr != by_id) CHECK(by_id->name == "Polyline_C");
  CHECK(nullptr == document.FindById(999999));

  // By uuid.
  const SceneObject* seed = document.FindById(scene.ids[0]);
  CHECK(nullptr != seed);
  if (nullptr != seed) {
    const SceneObject* by_uuid = document.FindByUuid(seed->uuid);
    CHECK(nullptr != by_uuid);
    if (nullptr != by_uuid) CHECK(by_uuid->id == seed->id);
  }
  CHECK(nullptr == document.FindByUuid(ON_nil_uuid));

  // By name.
  const std::vector<const SceneObject*> named = document.FindByName("Circle_D");
  CHECK_EQ_INT(static_cast<int>(named.size()), 1);
  if (named.size() == 1) CHECK(named[0]->kind == ObjectKind::Circle);
  CHECK_EQ_INT(static_cast<int>(document.FindByName("NoSuchName").size()), 0);

  // By layer.
  const std::vector<const SceneObject*> curves = document.FindByLayer("Curves");
  CHECK_EQ_INT(static_cast<int>(curves.size()), 5);
  for (size_t i = 0; i < curves.size(); ++i) {
    const Layer* layer = document.GetLayer(curves[i]->layer_index);
    CHECK(nullptr != layer && layer->name == "Curves");
  }
  const std::vector<const SceneObject*> reference = document.FindByLayer("Reference");
  CHECK_EQ_INT(static_cast<int>(reference.size()), 1);
  CHECK_EQ_INT(static_cast<int>(document.FindByLayer("Nope").size()), 0);

  // By kind.
  CHECK_EQ_INT(static_cast<int>(document.FindByKind(ObjectKind::Point).size()), 1);
  CHECK_EQ_INT(static_cast<int>(document.FindByKind(ObjectKind::Line).size()), 1);
  CHECK_EQ_INT(static_cast<int>(document.FindByKind(ObjectKind::Polyline).size()), 1);
  CHECK_EQ_INT(static_cast<int>(document.FindByKind(ObjectKind::Circle).size()), 1);
  CHECK_EQ_INT(static_cast<int>(document.FindByKind(ObjectKind::Arc).size()), 1);
  CHECK_EQ_INT(static_cast<int>(document.FindByKind(ObjectKind::Plane).size()), 1);
  CHECK_EQ_INT(static_cast<int>(document.FindByKind(ObjectKind::NurbsCurve).size()), 1);

  // Deletion.
  const size_t before = document.ObjectCount();
  const uint64_t temp_id = document.AddPoint(Point3d(9.0, 9.0, 9.0), "Temp", "Default");
  CHECK_EQ_INT(static_cast<int>(document.ObjectCount()), static_cast<int>(before) + 1);
  CHECK(nullptr != document.FindByIdMutable(temp_id));
  CHECK(document.DeleteObject(temp_id));
  CHECK_EQ_INT(static_cast<int>(document.ObjectCount()), static_cast<int>(before));
  CHECK(nullptr == document.FindById(temp_id));
  CHECK(!document.DeleteObject(temp_id));
  CHECK(!document.DeleteObject(123456789));

  EndSection();
}

// ============================================================================
// 04 - transformations
// ============================================================================

void TestTransforms() {
  using namespace opentl::mvp;
  BeginSection("transformations: translate / rotate / scale + bounding boxes");

  SceneDocument document;
  EnsureLayer(document, "Default", 0, 0, 0);

  const uint64_t point_id = document.AddPoint(Point3d(2.0, 4.0, 6.0), "P", "Default");
  const uint64_t line_id =
      document.AddLine(Line(Point3d(0.0, 0.0, 0.0), Point3d(10.0, 0.0, 0.0)), "L", "Default");
  const uint64_t circle_id = document.AddCircle(Circle(Point3d::Origin, 5.0), "C", "Default");
  const uint64_t polyline_id = document.AddPolyline(
      std::vector<Point3d>{Point3d(0.0, 0.0, 0.0), Point3d(2.0, 1.0, 0.0), Point3d(4.0, 0.0, 0.0)},
      "PL", "Default");

  // ---- translate -------------------------------------------------------
  CHECK(document.TranslateObject(point_id, Vector3d(1.0, -2.0, 3.0)));
  const SceneObject* point = document.FindById(point_id);
  CHECK(nullptr != point);
  if (nullptr != point) {
    CHECK_POINT(point->point_val, Point3d(3.0, 2.0, 9.0), 1e-12);
    const BoundingBox box = point->GetBoundingBox();
    CHECK_POINT(box.Min(), Point3d(3.0, 2.0, 9.0), 1e-12);
    CHECK_POINT(box.Max(), Point3d(3.0, 2.0, 9.0), 1e-12);
  }

  // ---- rotate ----------------------------------------------------------
  CHECK(document.RotateObject(line_id, ON_PI * 0.5, Vector3d(0.0, 0.0, 1.0)));
  const SceneObject* line = document.FindById(line_id);
  CHECK(nullptr != line);
  if (nullptr != line) {
    CHECK_POINT(line->line_val.from, Point3d(0.0, 0.0, 0.0), 1e-9);
    CHECK_POINT(line->line_val.to, Point3d(0.0, 10.0, 0.0), 1e-9);
    CHECK_NEAR(line->line_val.Length(), 10.0, 1e-9);
    const BoundingBox box = line->GetBoundingBox();
    CHECK_POINT(box.Min(), Point3d(0.0, 0.0, 0.0), 1e-9);
    CHECK_POINT(box.Max(), Point3d(0.0, 10.0, 0.0), 1e-9);
  }

  // Rotate about a non origin center.
  CHECK(document.RotateObject(line_id, ON_PI, Vector3d(0.0, 0.0, 1.0), Point3d(0.0, 10.0, 0.0)));
  line = document.FindById(line_id);
  if (nullptr != line) {
    CHECK_POINT(line->line_val.from, Point3d(0.0, 20.0, 0.0), 1e-9);
    CHECK_POINT(line->line_val.to, Point3d(0.0, 10.0, 0.0), 1e-9);
  }

  // ---- scale -----------------------------------------------------------
  CHECK(document.ScaleObject(circle_id, 2.0, 2.0, 2.0));
  const SceneObject* circle = document.FindById(circle_id);
  CHECK(nullptr != circle);
  if (nullptr != circle) {
    CHECK_NEAR(circle->circle_val.Radius(), 10.0, 1e-9);
    CHECK_POINT(circle->circle_val.Center(), Point3d(0.0, 0.0, 0.0), 1e-9);
    const BoundingBox box = circle->GetBoundingBox();
    CHECK(box.Min().x <= -10.0 + 1e-6);
    CHECK(box.Max().x >= 10.0 - 1e-6);
  }

  // Non uniform scale around a non origin anchor.
  CHECK(document.ScaleObject(polyline_id, 2.0, 3.0, 1.0, Point3d(0.0, 0.0, 0.0)));
  const SceneObject* polyline = document.FindById(polyline_id);
  CHECK(nullptr != polyline);
  if (nullptr != polyline) {
    CHECK_EQ_INT(polyline->polyline_val.PointCount(), 3);
    CHECK_POINT(polyline->polyline_val[0], Point3d(0.0, 0.0, 0.0), 1e-9);
    CHECK_POINT(polyline->polyline_val[1], Point3d(4.0, 3.0, 0.0), 1e-9);
    CHECK_POINT(polyline->polyline_val[2], Point3d(8.0, 0.0, 0.0), 1e-9);
  }

  // ---- transform builders ---------------------------------------------
  const Transform translation = MakeTranslation(1.0, 2.0, 3.0);
  const ON_3dPoint moved = translation * ON_3dPoint(0.0, 0.0, 0.0);
  CHECK_POINT(moved, Point3d(1.0, 2.0, 3.0), 1e-12);

  const Transform rotation = MakeRotation(ON_PI * 0.5, Vector3d(0.0, 0.0, 1.0));
  const ON_3dPoint rotated = rotation * ON_3dPoint(1.0, 0.0, 0.0);
  CHECK_POINT(rotated, Point3d(0.0, 1.0, 0.0), 1e-9);

  const Transform scaling = MakeUniformScale(Point3d::Origin, 3.0);
  const ON_3dPoint scaled = scaling * ON_3dPoint(1.0, 2.0, 3.0);
  CHECK_POINT(scaled, Point3d(3.0, 6.0, 9.0), 1e-12);

  // ---- invalid id ------------------------------------------------------
  CHECK(!document.TranslateObject(999999, Vector3d(1.0, 0.0, 0.0)));
  CHECK(!document.RotateObject(999999, 1.0, Vector3d(0.0, 0.0, 1.0)));
  CHECK(!document.ScaleObject(999999, 2.0, 2.0, 2.0));

  // ---- scene bounding box ---------------------------------------------
  const BoundingBox scene_box = document.GetSceneBoundingBox();
  CHECK(scene_box.IsSet());
  CHECK(scene_box.Min().x <= -10.0 + 1e-6);
  CHECK(scene_box.Max().y >= 20.0 - 1e-6);

  EndSection();
}

// ============================================================================
// 05 - distance queries
// ============================================================================

void TestDistances() {
  using namespace opentl::mvp;
  BeginSection("distance queries: point to object, object to object");

  SceneDocument document;
  EnsureLayer(document, "Default", 0, 0, 0);

  SceneObject point;
  point.kind = ObjectKind::Point;
  point.point_val = Point3d(2.0, 4.0, 6.0);
  CHECK_NEAR(point.DistanceTo(Point3d(2.0, 4.0, 7.0)), 1.0, 1e-9);

  SceneObject line;
  line.kind = ObjectKind::Line;
  line.line_val = Line(Point3d(0.0, 0.0, 0.0), Point3d(10.0, 0.0, 0.0));
  CHECK_NEAR(line.DistanceTo(Point3d(5.0, 3.0, 4.0)), 5.0, 1e-9);
  CHECK_NEAR(line.DistanceTo(Point3d(5.0, 0.0, 0.0)), 0.0, 1e-9);

  SceneObject polyline;
  polyline.kind = ObjectKind::Polyline;
  polyline.polyline_val.Append(Point3d(0.0, 0.0, 0.0));
  polyline.polyline_val.Append(Point3d(10.0, 0.0, 0.0));
  polyline.polyline_val.Append(Point3d(10.0, 10.0, 0.0));
  CHECK_NEAR(polyline.DistanceTo(Point3d(5.0, 0.0, 3.0)), 3.0, 1e-9);

  SceneObject circle;
  circle.kind = ObjectKind::Circle;
  circle.circle_val = Circle(Point3d::Origin, 5.0);
  CHECK_NEAR(circle.DistanceTo(Point3d(10.0, 0.0, 0.0)), 5.0, 1e-9);
  CHECK_NEAR(circle.DistanceTo(Point3d(0.0, 0.0, 0.0)), 5.0, 1e-9);

  SceneObject arc;
  arc.kind = ObjectKind::Arc;
  arc.arc_val = Arc(Plane::World_xy, 3.0, ON_PI * 0.5);
  CHECK_NEAR(arc.DistanceTo(Point3d(0.0, 0.0, 0.0)), 3.0, 1e-6);
  CHECK_NEAR(arc.DistanceTo(Point3d(3.0, 0.0, 0.0)), 0.0, 1e-6);

  SceneObject plane;
  plane.kind = ObjectKind::Plane;
  plane.plane_val = Plane::World_xy;
  CHECK_NEAR(plane.DistanceTo(Point3d(0.0, 0.0, 7.0)), 7.0, 1e-9);

  SceneObject nurbs;
  nurbs.kind = ObjectKind::NurbsCurve;
  nurbs.nurbs_val = MakeCubicWave();
  CHECK_NEAR(nurbs.DistanceTo(Point3d(0.0, 0.0, 5.0)), 5.0, 1e-6);

  // Object to object.
  CHECK_NEAR(DistanceBetweenObjects(point, line), std::sqrt(52.0), 1e-6);
  SceneObject far_point;
  far_point.kind = ObjectKind::Point;
  far_point.point_val = Point3d(5.0, 0.0, 20.0);
  CHECK_NEAR(DistanceBetweenObjects(far_point, line), 20.0, 1e-6);
  CHECK_NEAR(DistanceBetweenObjects(far_point, far_point), 0.0, 1e-12);
  CHECK_NEAR(DistanceBetweenObjects(circle, far_point), 20.0, 1e-6);

  // Document level object distances stay consistent with the standalone helper.
  const uint64_t a = document.AddPoint(Point3d(0.0, 0.0, 0.0), "A", "Default");
  const uint64_t b = document.AddPoint(Point3d(3.0, 4.0, 0.0), "B", "Default");
  const SceneObject* object_a = document.FindById(a);
  const SceneObject* object_b = document.FindById(b);
  CHECK(nullptr != object_a && nullptr != object_b);
  if (nullptr != object_a && nullptr != object_b) {
    CHECK_NEAR(DistanceBetweenObjects(*object_a, *object_b), 5.0, 1e-9);
  }

  EndSection();
}

// ============================================================================
// 06 - line / line intersections
// ============================================================================

void TestLineLineIntersections() {
  using namespace opentl::mvp;
  BeginSection("line-line intersections: crossing / skew / parallel / finite clip");

  const Line line_a(Point3d(0.0, 0.0, 0.0), Point3d(10.0, 0.0, 0.0));

  // Crossing.
  const Line crossing(Point3d(5.0, -5.0, 0.0), Point3d(5.0, 5.0, 0.0));
  const LineLineIntersection hit = IntersectLineLine(line_a, crossing, 1e-6, false);
  CHECK(hit.intersects);
  CHECK(!hit.is_parallel);
  CHECK_NEAR(hit.param_a, 0.5, 1e-9);
  CHECK_NEAR(hit.param_b, 0.5, 1e-9);
  CHECK_POINT(hit.point_a, Point3d(5.0, 0.0, 0.0), 1e-9);
  CHECK_POINT(hit.point_b, Point3d(5.0, 0.0, 0.0), 1e-9);
  CHECK_NEAR(hit.distance, 0.0, 1e-9);

  // Skew (closest points 5 units apart).
  const Line skew(Point3d(5.0, -5.0, 5.0), Point3d(5.0, 5.0, 5.0));
  const LineLineIntersection skew_hit = IntersectLineLine(line_a, skew, 1e-6, false);
  CHECK(!skew_hit.intersects);
  CHECK(!skew_hit.is_parallel);
  CHECK_POINT(skew_hit.point_a, Point3d(5.0, 0.0, 0.0), 1e-6);
  CHECK_POINT(skew_hit.point_b, Point3d(5.0, 0.0, 5.0), 1e-6);
  CHECK_NEAR(skew_hit.distance, 5.0, 1e-6);

  // Parallel.
  const Line parallel(Point3d(0.0, 1.0, 0.0), Point3d(10.0, 1.0, 0.0));
  const LineLineIntersection parallel_hit = IntersectLineLine(line_a, parallel, 1e-6, false);
  CHECK(parallel_hit.is_parallel);
  CHECK(!parallel_hit.intersects);
  CHECK_NEAR(parallel_hit.distance, 1.0, 1e-6);

  // Coincident lines are parallel too.
  const Line coincident(Point3d(10.0, 0.0, 0.0), Point3d(20.0, 0.0, 0.0));
  const LineLineIntersection coincident_hit = IntersectLineLine(line_a, coincident, 1e-6, false);
  CHECK(coincident_hit.is_parallel);

  // Finite segment clipping: the infinite solution is outside both segments.
  const Line outside(Point3d(20.0, -5.0, 0.0), Point3d(20.0, 5.0, 0.0));
  const LineLineIntersection infinite_hit = IntersectLineLine(line_a, outside, 1e-6, false);
  CHECK(infinite_hit.intersects);
  CHECK_NEAR(infinite_hit.param_a, 2.0, 1e-9);
  const LineLineIntersection finite_hit = IntersectLineLine(line_a, outside, 1e-6, true);
  CHECK(!finite_hit.intersects);

  // Finite segments that do cross.
  // Crosses line_a inside both finite segments (infinite solution is at x = 7).
  const Line within(Point3d(12.0, -5.0, 0.0), Point3d(2.0, 5.0, 0.0));
  const LineLineIntersection finite_cross = IntersectLineLine(line_a, within, 1e-6, true);
  CHECK(finite_cross.intersects);
  CHECK(finite_cross.param_a >= 0.0 && finite_cross.param_a <= 1.0);
  CHECK(finite_cross.param_b >= 0.0 && finite_cross.param_b <= 1.0);

  EndSection();
}

// ============================================================================
// 07 - line / plane intersections
// ============================================================================

void TestLinePlaneIntersections() {
  using namespace opentl::mvp;
  BeginSection("line-plane intersections: crossing / parallel / finite segment");

  const Plane world_xy = Plane::World_xy;

  const Line through(Point3d(0.0, 0.0, -5.0), Point3d(0.0, 0.0, 5.0));
  const LinePlaneIntersection hit = IntersectLinePlane(through, world_xy, false, 1e-9);
  CHECK(hit.intersects);
  CHECK(!hit.is_parallel);
  CHECK_NEAR(hit.line_param, 0.5, 1e-9);
  CHECK_POINT(hit.point, Point3d(0.0, 0.0, 0.0), 1e-9);
  CHECK_NEAR(hit.plane_u, 0.0, 1e-9);
  CHECK_NEAR(hit.plane_v, 0.0, 1e-9);

  // Off origin so the plane u/v are meaningful.
  const Line offset(Point3d(3.0, -4.0, -2.0), Point3d(3.0, -4.0, 2.0));
  const LinePlaneIntersection offset_hit = IntersectLinePlane(offset, world_xy, false, 1e-9);
  CHECK(offset_hit.intersects);
  CHECK_POINT(offset_hit.point, Point3d(3.0, -4.0, 0.0), 1e-9);
  CHECK_NEAR(offset_hit.plane_u, 3.0, 1e-9);
  CHECK_NEAR(offset_hit.plane_v, -4.0, 1e-9);

  // Parallel.
  const Line parallel(Point3d(0.0, 0.0, 3.0), Point3d(10.0, 5.0, 3.0));
  const LinePlaneIntersection parallel_hit = IntersectLinePlane(parallel, world_xy, false, 1e-9);
  CHECK(!parallel_hit.intersects);
  CHECK(parallel_hit.is_parallel);

  // Finite segment that stops before the plane.
  const Line short_line(Point3d(0.0, 0.0, -5.0), Point3d(0.0, 0.0, -1.0));
  const LinePlaneIntersection short_hit = IntersectLinePlane(short_line, world_xy, true, 1e-9);
  CHECK(!short_hit.intersects);
  const LinePlaneIntersection short_infinite = IntersectLinePlane(short_line, world_xy, false, 1e-9);
  CHECK(short_infinite.intersects);
  CHECK(short_infinite.line_param > 1.0);

  // Tilted plane.
  const Plane tilted(Point3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
  const Line diagonal(Point3d(-3.0, -3.0, -3.0), Point3d(3.0, 3.0, 3.0));
  const LinePlaneIntersection tilted_hit = IntersectLinePlane(diagonal, tilted, false, 1e-9);
  CHECK(tilted_hit.intersects);
  CHECK_POINT(tilted_hit.point, Point3d(0.0, 0.0, 0.0), 1e-6);

  EndSection();
}

// ============================================================================
// 08 - curve intersections
// ============================================================================

void TestCurveIntersections() {
  using namespace opentl::mvp;
  BeginSection("curve-line and curve-plane intersections");

  // ---- curve-line: a circle crossed by a line through its center --------
  const ON_ArcCurve circle_curve(ON_Circle(ON_3dPoint::Origin, 5.0));
  const Line diameter(Point3d(-100.0, 0.0, 0.0), Point3d(100.0, 0.0, 0.0));
  ON_SimpleArray<TL_CurveCurveIntersectionEvent> events;
  const int count = TL_IntersectCurveLine(circle_curve, diameter, 1e-6, events);
  CHECK(count > 0);
  // A closed curve must not report the seam crossing twice (t_min == t_max).
  CHECK_EQ_INT(events.Count(), 2);
  CHECK_EQ_INT(count, 2);
  for (int i = 0; i < events.Count(); ++i) {
    CHECK_NEAR(events[i].point_a.DistanceTo(ON_3dPoint::Origin), 5.0, 1e-4);
    CHECK_NEAR(events[i].point_a.z, 0.0, 1e-6);
  }

  // ---- curve-line: a line that misses the circle ------------------------
  ON_SimpleArray<TL_CurveCurveIntersectionEvent> miss_events;
  const Line far_line(Point3d(-100.0, 50.0, 0.0), Point3d(100.0, 50.0, 0.0));
  CHECK_EQ_INT(TL_IntersectCurveLine(circle_curve, far_line, 1e-6, miss_events), 0);

  // ---- curve-line: a cubic NURBS crossed by a vertical line -------------
  {
    NurbsCurve wave = MakeCubicWave();
    CHECK(wave.IsValid());
    const Line vertical(Point3d(6.0, -20.0, 0.0), Point3d(6.0, 20.0, 0.0));
    ON_SimpleArray<TL_CurveCurveIntersectionEvent> wave_events;
    const int wave_count = TL_IntersectCurveLine(wave, vertical, 1e-6, wave_events);
    CHECK(wave_count > 0);
    for (int i = 0; i < wave_events.Count(); ++i) {
      CHECK_NEAR(wave_events[i].point_a.x, 6.0, 1e-4);
    }
  }

  // ---- curve-plane: circle against the world YZ plane -------------------
  {
    ON_SimpleArray<double> parameters;
    ON_SimpleArray<ON_3dPoint> points;
    const int plane_count =
        TL_IntersectCurvePlane(circle_curve, ON_Plane::World_yz, 1e-6, parameters, points);
    CHECK(plane_count > 0);
    CHECK(points.Count() >= 2);
    CHECK_EQ_INT(parameters.Count(), points.Count());
    for (int i = 0; i < points.Count(); ++i) {
      CHECK_NEAR(points[i].x, 0.0, 1e-6);
      CHECK_NEAR(std::fabs(points[i].y), 5.0, 1e-4);
      CHECK_NEAR(points[i].z, 0.0, 1e-6);
    }
  }

  // ---- curve-plane: 3d NURBS crossing the world XY plane ----------------
  {
    ON_NurbsCurve curve(3, false, 4, 4);
    curve.SetCV(0, ON_3dPoint(0.0, 0.0, 0.0));
    curve.SetCV(1, ON_3dPoint(3.0, 6.0, 3.0));
    curve.SetCV(2, ON_3dPoint(6.0, -6.0, -3.0));
    curve.SetCV(3, ON_3dPoint(9.0, 0.0, 0.0));
    curve.MakeClampedUniformKnotVector(1.0);
    CHECK(curve.IsValid());

    ON_SimpleArray<double> parameters;
    ON_SimpleArray<ON_3dPoint> points;
    const int plane_count =
        TL_IntersectCurvePlane(curve, ON_Plane::World_xy, 1e-6, parameters, points);
    CHECK(plane_count > 0);
    for (int i = 0; i < points.Count(); ++i) {
      CHECK_NEAR(points[i].z, 0.0, 1e-4);
      CHECK(parameters[i] >= curve.Domain()[0] - 1e-6);
      CHECK(parameters[i] <= curve.Domain()[1] + 1e-6);
    }
  }

  EndSection();
}

// ============================================================================
// 09 - NURBS curve evaluation and sampling
// ============================================================================

void TestNurbsAndSampling() {
  using namespace opentl::mvp;
  BeginSection("NURBS curve: evaluation, uniform + arc length sampling, closest points");

  const NurbsCurve wave = MakeCubicWave();
  CHECK(wave.IsValid());
  CHECK_EQ_INT(wave.m_order, 4);
  CHECK_EQ_INT(wave.m_cv_count, 5);

  // Evaluation at the ends matches the clamped control polygon.
  const ON_3dPoint start = wave.PointAt(wave.Domain()[0]);
  const ON_3dPoint end = wave.PointAt(wave.Domain()[1]);
  CHECK_POINT(start, Point3d(0.0, 0.0, 0.0), 1e-9);
  CHECK_POINT(end, Point3d(12.0, 6.0, 0.0), 1e-9);

  // First derivative is finite and the curve is parameterized in increasing order.
  double derivatives[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const double mid = wave.Domain().Mid();
  CHECK(wave.Evaluate(mid, 1, 3, derivatives));
  CHECK(std::isfinite(derivatives[0]) && std::isfinite(derivatives[3]));

  // Uniform parameter sampling.
  const std::vector<CurveSamplePoint> uniform = SampleCurveByCount(wave, 9, false);
  CHECK_EQ_INT(static_cast<int>(uniform.size()), 9);
  if (uniform.size() == 9) {
    CHECK_NEAR(uniform.front().normalized_t, 0.0, 1e-9);
    CHECK_NEAR(uniform.back().normalized_t, 1.0, 1e-9);
    CHECK_POINT(uniform.front().point, start, 1e-6);
    CHECK_POINT(uniform.back().point, end, 1e-6);
    for (size_t i = 1; i < uniform.size(); ++i) {
      CHECK(uniform[i].t > uniform[i - 1].t);
      CHECK(uniform[i].arc_length >= uniform[i - 1].arc_length - 1e-9);
    }
  }

  // Arc length sampling: the samples must be evenly spaced along the curve.
  const std::vector<CurveSamplePoint> arclength = SampleCurveByCount(wave, 9, true);
  CHECK_EQ_INT(static_cast<int>(arclength.size()), 9);
  if (arclength.size() == 9) {
    CHECK_POINT(arclength.front().point, start, 1e-6);
    CHECK_POINT(arclength.back().point, end, 1e-6);
    const double total = arclength.back().arc_length;
    CHECK(total > 0.0);
    for (size_t i = 1; i + 1 < arclength.size(); ++i) {
      const double expected = total * static_cast<double>(i) / 8.0;
      CHECK_NEAR(arclength[i].arc_length, expected, 1e-3 * (1.0 + total));
    }
    // Degenerate sample counts must not produce a huge or corrupted result.
    CHECK(SampleCurveByCount(wave, 1, false).size() <= 2);
  }

  // Closest point on the curve.
  double closest_t = 0.0;
  const bool closest_ok = TL_GetCurveClosestPoint(wave, Point3d(0.0, 0.0, 7.0), &closest_t);
  CHECK(closest_ok);
  const ON_3dPoint closest_point = wave.PointAt(closest_t);
  // The curve starts at (0,0,0), so that is the closest point to the probe at
  // (0,0,7) and its distance to the probe is exactly 7.
  CHECK_POINT(closest_point, start, 1e-4);
  CHECK_NEAR(closest_point.DistanceTo(Point3d(0.0, 0.0, 7.0)), 7.0, 1e-4);

  // Closest point on a plane surface (used as the surface closest point probe).
  ON_PlaneSurface surface(ON_Plane::World_xy);
  surface.SetExtents(0, ON_Interval(-10.0, 10.0), true);
  surface.SetExtents(1, ON_Interval(-10.0, 10.0), true);
  CHECK(surface.IsValid());
  double u = 0.0;
  double v = 0.0;
  const bool surface_ok = TL_GetSurfaceClosestPoint(surface, Point3d(3.0, 4.0, 5.0), &u, &v);
  CHECK(surface_ok);
  const ON_3dPoint surface_point = surface.PointAt(u, v);
  CHECK_NEAR(surface_point.DistanceTo(Point3d(3.0, 4.0, 0.0)), 1e-6, 1e-4);

  // CreateClampedNurbsCurve rejects too few control points.
  NurbsCurve too_few;
  CHECK(!CreateClampedNurbsCurve(std::vector<Point3d>{Point3d(0, 0, 0)}, 3, too_few));

  EndSection();
}

// ============================================================================
// 10 - reverse engineered helpers
// ============================================================================

void TestReverseEngineeredHelpers() {
  using namespace opentl::mvp;
  BeginSection("reverse engineered helpers (TL_Brep / mesh / deformable / loft / SubD)");

  // ---- TL_Brep::PointIsOnFace / TL_Brep_PointIsOnFace -------------------
  ON_3dPoint corners[8];
  corners[0] = ON_3dPoint(0.0, 0.0, 0.0);
  corners[1] = ON_3dPoint(10.0, 0.0, 0.0);
  corners[2] = ON_3dPoint(10.0, 10.0, 0.0);
  corners[3] = ON_3dPoint(0.0, 10.0, 0.0);
  corners[4] = ON_3dPoint(0.0, 0.0, 10.0);
  corners[5] = ON_3dPoint(10.0, 0.0, 10.0);
  corners[6] = ON_3dPoint(10.0, 10.0, 10.0);
  corners[7] = ON_3dPoint(0.0, 10.0, 10.0);

  ON_Brep box;
  CHECK(nullptr != ON_BrepBox(corners, &box));
  CHECK(box.m_F.Count() >= 5);
  if (box.m_F.Count() >= 1) {
    const TL_Brep* promoted = TL_Brep::Promote(&box);
    CHECK(nullptr != promoted);
    if (nullptr != promoted) {
      const int inside = promoted->PointIsOnFace(0, ON_2dPoint(5.0, 5.0), 1e-6, 1e-6);
      CHECK_MSG(inside == 1 || inside == 2, "uv in the middle of a box face must be on the face");
      const int outside = promoted->PointIsOnFace(0, ON_2dPoint(-50.0, -50.0), 1e-6, 1e-6);
      CHECK_EQ_INT(outside, 0);
      CHECK(promoted->IsPointOnFace(0, 5.0, 5.0, 1e-6));
      CHECK(!promoted->IsPointOnFace(0, -50.0, -50.0, 1e-6));
    }
    CHECK(TL_Brep_PointIsOnFace(&box, 0, 5.0, 5.0, 1e-6));
    CHECK(!TL_Brep_PointIsOnFace(&box, 0, -50.0, -50.0, 1e-6));
    CHECK(!TL_Brep_PointIsOnFace(nullptr, 0, 0.0, 0.0, 1e-6));
  }

  // ---- TL_IsMeshEdgeUnwelded / TLC_IsMeshEdgeUnwelded ------------------
  {
    ON_Mesh welded;
    CHECK(BuildTwoTriangleMesh(welded, true));
    const int welded_edge = FirstInteriorTopologyEdge(welded);
    CHECK(welded_edge >= 0);
    if (welded_edge >= 0) {
      CHECK(!TL_IsMeshEdgeUnwelded(welded_edge, &welded));
      CHECK(!TLC_IsMeshEdgeUnwelded(&welded, welded_edge));
    }

    ON_Mesh unwelded;
    CHECK(BuildTwoTriangleMesh(unwelded, false));
    const int unwelded_edge = FirstInteriorTopologyEdge(unwelded);
    CHECK(unwelded_edge >= 0);
    if (unwelded_edge >= 0) {
      CHECK(TL_IsMeshEdgeUnwelded(unwelded_edge, &unwelded));
      CHECK(TLC_IsMeshEdgeUnwelded(&unwelded, unwelded_edge));
    }

    CHECK(!TL_IsMeshEdgeUnwelded(0, nullptr));
    CHECK(!TL_IsMeshEdgeUnwelded(-1, &unwelded));
    CHECK(!TLC_IsMeshEdgeUnwelded(nullptr, 0));
  }

  // ---- TL_DeformableArc2 ----------------------------------------------
  {
    const ON_Arc arc(ON_Plane::World_xy, 3.0, ON_PI * 0.5);
    ON_NurbsCurve* deformable = TL_DeformableArc2(arc, 3, 8, nullptr);
    CHECK(nullptr != deformable);
    if (nullptr != deformable) {
      CHECK_EQ_INT(deformable->m_order, 4);
      CHECK_EQ_INT(deformable->m_cv_count, 8);
      CHECK_EQ_INT(deformable->m_dim, 3);
      CHECK(deformable->IsValid());
      const ON_3dPoint curve_start = deformable->PointAt(deformable->Domain()[0]);
      const ON_3dPoint curve_end = deformable->PointAt(deformable->Domain()[1]);
      CHECK_NEAR(curve_start.DistanceTo(arc.StartPoint()), 0.0, 1e-6);
      CHECK_NEAR(curve_end.DistanceTo(arc.EndPoint()), 0.0, 1e-6);
      // The deformable approximation must stay close to the arc.
      double max_deviation = 0.0;
      for (int i = 1; i < 8; ++i) {
        const double normalized = static_cast<double>(i) / 8.0;
        const ON_3dPoint on_curve =
            deformable->PointAt(deformable->Domain().ParameterAt(normalized));
        const double radius = on_curve.DistanceTo(arc.Center());
        max_deviation = std::max(max_deviation, std::fabs(radius - arc.Radius()));
      }
      CHECK_NEAR(max_deviation, 0.0, 0.05);
      delete deformable;
    }
  }

  // ---- TL_DeformableCircle --------------------------------------------
  {
    const ON_Circle circle(ON_3dPoint(1.0, 2.0, 3.0), 2.0);
    ON_NurbsCurve* deformable = TL_DeformableCircle(circle, 3, 12, nullptr);
    CHECK(nullptr != deformable);
    if (nullptr != deformable) {
      CHECK_EQ_INT(deformable->m_order, 4);
      CHECK(deformable->IsValid());
      const ON_3dPoint curve_start = deformable->PointAt(deformable->Domain()[0]);
      const ON_3dPoint curve_end = deformable->PointAt(deformable->Domain()[1]);
      CHECK_NEAR(curve_start.DistanceTo(circle.PointAt(0.0)), 0.0, 1e-3);
      CHECK_NEAR(curve_start.DistanceTo(curve_end), 0.0, 1e-3);
      delete deformable;
    }
  }

  // ---- TL_NurbsCurve::RemoveKnots -------------------------------------
  {
    NurbsCurve curve;
    std::vector<Point3d> points;
    points.push_back(Point3d(0.0, 0.0, 0.0));
    points.push_back(Point3d(3.0, 6.0, 0.0));
    points.push_back(Point3d(6.0, -6.0, 0.0));
    points.push_back(Point3d(9.0, 0.0, 0.0));
    points.push_back(Point3d(12.0, 6.0, 0.0));
    CHECK(CreateClampedNurbsCurve(points, 3, curve));
    const int knots_before = curve.KnotCount();
    const ON_3dPoint start_before = curve.PointAt(curve.Domain()[0]);
    const ON_3dPoint end_before = curve.PointAt(curve.Domain()[1]);

    TL_NurbsCurve* promoted = TL_NurbsCurve::Promote(&curve);
    CHECK(nullptr != promoted);
    if (nullptr != promoted) {
      const bool removed = promoted->RemoveKnots(0, knots_before - 1, 1e-6);
      if (removed) {
        CHECK(curve.KnotCount() <= knots_before);
      } else {
        CHECK_EQ_INT(curve.KnotCount(), knots_before);
      }
      // Knot removal must not move the curve endpoints.
      CHECK_NEAR(curve.PointAt(curve.Domain()[0]).DistanceTo(start_before), 0.0, 1e-6);
      CHECK_NEAR(curve.PointAt(curve.Domain()[1]).DistanceTo(end_before), 0.0, 1e-6);
    }
    CHECK(nullptr == TL_NurbsCurve::Promote(static_cast<const ON_Object*>(nullptr)));
  }

  // ---- TL_LoftCurves ---------------------------------------------------
  {
    ON_LineCurve section_a(ON_Line(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(10.0, 0.0, 0.0)));
    ON_LineCurve section_b(ON_Line(ON_3dPoint(0.0, 10.0, 0.0), ON_3dPoint(10.0, 10.0, 0.0)));
    ON_SimpleArray<const ON_Curve*> sections;
    sections.Append(&section_a);
    sections.Append(&section_b);
    ON_NurbsSurface* loft = TL_LoftCurves(sections, nullptr);
    CHECK(nullptr != loft);
    if (nullptr != loft) {
      const ON_Interval domain_u = loft->Domain(0);
      const ON_Interval domain_v = loft->Domain(1);
      CHECK(domain_u.IsIncreasing());
      CHECK(domain_v.IsIncreasing());
      const ON_3dPoint p00 = loft->PointAt(domain_u[0], domain_v[0]);
      const ON_3dPoint p11 = loft->PointAt(domain_u[1], domain_v[1]);
      CHECK(std::fabs(p00.z) < 1e-6 && std::fabs(p11.z) < 1e-6);
      CHECK(std::fabs(p00.x - p11.x) < 1e-6 || std::fabs(p00.x + p11.x - 10.0) < 1e-6);
      ON_BoundingBox loft_box;
      CHECK(loft->GetBoundingBox(loft_box, false));
      CHECK_NEAR(loft_box.m_max.y - loft_box.m_min.y, 10.0, 1e-6);
      delete loft;
    }
  }

  // ---- TL_CreateSubDBox -----------------------------------------------
  {
    const ON_BoundingBox box_limits(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(4.0, 4.0, 4.0));
    ON_SubD* subd = TL_CreateSubDBox(box_limits, 1, nullptr);
    CHECK(nullptr != subd);
    if (nullptr != subd) {
      CHECK(subd->IsNotEmpty());
      CHECK(subd->VertexCount() >= 8);
      CHECK(subd->FaceCount() >= 6);
      CHECK(subd->EdgeCount() >= 12);
      ON_BoundingBox subd_box;
      CHECK(subd->GetBoundingBox(subd_box, false));
      CHECK(subd_box.m_max.x >= subd_box.m_min.x);
      delete subd;
    }
    ON_SubD* no_subdivision = TL_CreateSubDBox(box_limits, 0, nullptr);
    CHECK(nullptr != no_subdivision);
    if (nullptr != no_subdivision) {
      CHECK(no_subdivision->FaceCount() >= 6);
      CHECK(no_subdivision->VertexCount() >= 8);
      delete no_subdivision;
    }
  }

  EndSection();
}

// ============================================================================
// 11 - local JSON scene round-trip
// ============================================================================

void TestJsonRoundTrip() {
  using namespace opentl::mvp;
  BeginSection("local JSON scene format round-trip (.otscene.json)");

  TestScene source;
  BuildSerializationScene(source);

  const std::string path = "opentl_test_scene.otscene.json";
  std::string error;
  const bool saved = source.document.SaveJsonScene(path, &error);
  CHECK_MSG(saved, saved ? "" : error);
  if (!saved) {
    EndSection();
    return;
  }

  SceneDocument loaded;
  std::string load_error;
  const bool load_ok = loaded.LoadJsonScene(path, &load_error);
  CHECK_MSG(load_ok, load_ok ? "" : load_error);
  if (!load_ok) {
    EndSection();
    return;
  }

  const std::string mismatch = ScenesMismatch(source.document, loaded, 1e-9);
  CHECK_MSG(mismatch.empty(), mismatch);
  CHECK_EQ_INT(static_cast<int>(loaded.ObjectCount()), 7);
  CHECK_EQ_INT(static_cast<int>(loaded.Layers().size()), 4);

  // IDs, UUIDs, names and layer assignments survive exactly.
  for (size_t i = 0; i < source.document.Objects().size() && i < loaded.Objects().size(); ++i) {
    const SceneObject& a = source.document.Objects()[i];
    const SceneObject& b = loaded.Objects()[i];
    CHECK(a.id == b.id);
    CHECK(0 == ON_UuidCompare(a.uuid, b.uuid));
    CHECK(a.name == b.name);
    CHECK_EQ_INT(a.layer_index, b.layer_index);
    CHECK(a.kind == b.kind);
  }

  // Deterministic output: saving the reloaded scene must reproduce the file.
  const std::string second_path = "opentl_test_scene_2.otscene.json";
  CHECK(loaded.SaveJsonScene(second_path, &load_error));
  std::FILE* first = std::fopen(path.c_str(), "rb");
  std::FILE* second = std::fopen(second_path.c_str(), "rb");
  CHECK(nullptr != first && nullptr != second);
  if (nullptr != first && nullptr != second) {
    std::string text_a;
    std::string text_b;
    char buffer[4096];
    size_t read = 0;
    while ((read = std::fread(buffer, 1, sizeof(buffer), first)) > 0) text_a.append(buffer, read);
    while ((read = std::fread(buffer, 1, sizeof(buffer), second)) > 0) text_b.append(buffer, read);
    std::fclose(first);
    std::fclose(second);
    CHECK_MSG(!text_a.empty(), "saved scene file must not be empty");
    CHECK_MSG(text_a == text_b, "JSON serialization must be byte-for-byte deterministic");
  }

  // Malformed input is rejected without crashing.
  SceneDocument broken;
  CHECK(!broken.LoadJsonScene("opentl_this_file_does_not_exist.json", &error));

  std::remove(path.c_str());
  std::remove(second_path.c_str());
  EndSection();
}

// ============================================================================
// 12 - openNURBS .3dm round-trip
// ============================================================================

void Test3dmRoundTrip() {
  using namespace opentl::mvp;
  BeginSection("openNURBS .3dm archive round-trip");

  TestScene source;
  BuildSerializationScene(source);

  const std::string path = "opentl_test_scene.3dm";
  std::string error;
  const bool saved = source.document.Save3dm(path, &error);
  CHECK_MSG(saved, saved ? "" : error);
  if (!saved) {
    EndSection();
    return;
  }

  SceneDocument loaded;
  std::string load_error;
  const bool load_ok = loaded.Load3dm(path, &load_error);
  CHECK_MSG(load_ok, load_ok ? "" : load_error);
  if (!load_ok) {
    EndSection();
    return;
  }

  const std::string mismatch = ScenesMismatch(source.document, loaded, 1e-9);
  CHECK_MSG(mismatch.empty(), mismatch);
  CHECK_EQ_INT(static_cast<int>(loaded.ObjectCount()), 7);
  CHECK_EQ_INT(static_cast<int>(loaded.Layers().size()), 4);

  for (size_t i = 0; i < source.document.Objects().size() && i < loaded.Objects().size(); ++i) {
    const SceneObject& a = source.document.Objects()[i];
    const SceneObject& b = loaded.Objects()[i];
    CHECK(a.id == b.id);
    CHECK(0 == ON_UuidCompare(a.uuid, b.uuid));
    CHECK(a.name == b.name);
    CHECK(a.kind == b.kind);
    CHECK_EQ_INT(a.layer_index, b.layer_index);
  }

  // A second write/read cycle must be stable (ids and uuids do not drift).
  const std::string second_path = "opentl_test_scene_2.3dm";
  CHECK(loaded.Save3dm(second_path, &load_error));
  SceneDocument reloaded;
  CHECK(reloaded.Load3dm(second_path, &load_error));
  const std::string second_mismatch = ScenesMismatch(loaded, reloaded, 1e-9);
  CHECK_MSG(second_mismatch.empty(), second_mismatch);

  // Missing file is reported, not fatal.
  CHECK(!reloaded.Load3dm("opentl_this_file_does_not_exist.3dm", &error));

  std::remove(path.c_str());
  std::remove(second_path.c_str());
  EndSection();
}

}  // namespace

int main() {
  ON::Begin();

  std::printf("OpenTL MVP test suite\n");
  std::printf("=====================\n");

  TestGeometryTypes();
  TestSceneDocumentBasics();
  TestQueriesAndDelete();
  TestTransforms();
  TestDistances();
  TestLineLineIntersections();
  TestLinePlaneIntersections();
  TestCurveIntersections();
  TestNurbsAndSampling();
  TestReverseEngineeredHelpers();
  TestJsonRoundTrip();
  Test3dmRoundTrip();

  std::printf("\n=====================\n");
  std::printf("sections=%d checks=%d failures=%d failed_sections=%d\n", g_state.section_index,
              g_state.total_checks, g_state.total_failures, g_state.sections_with_failures);
  std::printf("RESULT: %s\n", g_state.total_failures == 0 ? "ALL TESTS PASSED" : "FAILURES");

  ON::End();
  return g_state.total_failures == 0 ? 0 : 1;
}
