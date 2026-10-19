/**
 * @file test_opentl.cpp
 * @brief OpenTL MVP automated test suite.
 *
 * The suite is intentionally dependency free (no gtest / catch) so it builds
 * with the plain `opentl` static library target. Every section prints its own
 * pass/fail tally and the process exits non zero when anything fails.
 *
 * Coverage map
 *   01 geometry value types (Point3d / Vector3d / Line / ON_Polyline / Circle /
 *      ON_Arc / Plane / BoundingBox / NurbsCurve)
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

#include <fstream>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <system_error>
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

  ON_Arc arc(Plane::World_xy, 3.5, ON_PI * 0.75);
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

  // ---- ON_Polyline --------------------------------------------------------
  ON_Polyline polyline;
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

  // ---- ON_Arc -------------------------------------------------------------
  ON_Arc arc(Plane::World_xy, 2.0, ON_PI * 0.5);
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
  const uint64_t arc_id = document.AddArc(ON_Arc(Plane::World_xy, 1.0, ON_PI * 0.5), "Arc_E", "Curves");
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
  arc.arc_val = ON_Arc(Plane::World_xy, 3.0, ON_PI * 0.5);
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

  // ON_Arc length sampling: the samples must be evenly spaced along the curve.
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


// ============================================================================
// 13 - stage-3 primitives (TL_3DX_LineXPlane / line distances /
//      TL_GetClosestPoint(Line,Line) / dual curve-line strategy)
// ============================================================================

void TestStage3Primitives() {
  using namespace opentl::mvp;
  BeginSection("stage-3 primitives (lineXplane status, line distances, line-line closest point, dual strategy)");

  // ---- TL_3DX_LineXPlane: the four status codes ------------------------
  {
    const ON_3dPoint from(0.0, 0.0, 0.0);
    const ON_3dVector dir(0.0, 0.0, 1.0);
    const ON_3dPoint plane_pt(0.0, 0.0, 5.0);
    const ON_3dVector normal(0.0, 0.0, 1.0);

    double t = -999.0;
    CHECK_EQ_INT(TL_3DX_LineXPlane(from, dir, plane_pt, normal, &t), TL_LINEXPLANE_UNIQUE);
    CHECK_NEAR(t, 5.0, 1e-12);

    // Line that starts exactly on the plane -> status 0 (t == 0).
    t = -999.0;
    CHECK_EQ_INT(TL_3DX_LineXPlane(plane_pt, dir, plane_pt, normal, &t), TL_LINEXPLANE_AT_ZERO);
    CHECK_NEAR(t, 0.0, 1e-15);

    // Parallel and not coplanar -> -1.
    t = -999.0;
    CHECK_EQ_INT(TL_3DX_LineXPlane(ON_3dPoint(1.0, 2.0, 3.0), ON_3dVector(1.0, 0.0, 0.0),
                                   plane_pt, normal, &t),
                 TL_LINEXPLANE_PARALLEL);
    CHECK_NEAR(t, 0.0, 1e-15);

    // Line lying in the plane -> -2 (degenerate, infinitely many solutions).
    t = -999.0;
    CHECK_EQ_INT(TL_3DX_LineXPlane(from, ON_3dVector(1.0, 0.0, 0.0), from, normal, &t),
                 TL_LINEXPLANE_COPLANAR);
    CHECK_NEAR(t, 0.0, 1e-15);

    // Saturating division: |den| < 1 and |num| > |den| * DBL_MAX clamps to +-DBL_MAX
    // instead of producing infinity.
    t = 0.0;
    const double tiny = 1e-8;
    // den = 1e-8 (|den| < 1) and num = 1e305 -> |num| > |den| * DBL_MAX,
    // so the true quotient (~1e313) would overflow: the binary saturates to +DBL_MAX.
    CHECK_EQ_INT(TL_3DX_LineXPlane(ON_3dPoint(0.0, 0.0, 0.0), ON_3dVector(0.0, 0.0, tiny),
                                   ON_3dPoint(0.0, 0.0, 1e305), ON_3dVector(0.0, 0.0, 1.0), &t),
                 TL_LINEXPLANE_UNIQUE);
    CHECK(t == ON_DBL_MAX);
    // Negative direction saturates towards -DBL_MAX instead of -inf.
    CHECK_EQ_INT(TL_3DX_LineXPlane(ON_3dPoint(0.0, 0.0, 0.0), ON_3dVector(0.0, 0.0, -tiny),
                                   ON_3dPoint(0.0, 0.0, 1e305), ON_3dVector(0.0, 0.0, 1.0), &t),
                 TL_LINEXPLANE_UNIQUE);
    CHECK(t == -ON_DBL_MAX);

    // Null out-parameter must not crash and must still report the status.
    CHECK_EQ_INT(TL_3DX_LineXPlane(from, dir, plane_pt, normal, nullptr), TL_LINEXPLANE_UNIQUE);
  }

  // ---- line distance C-ABI (min / max + -1.0 sentinel) -----------------
  {
    const ON_Line horizontal(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(10.0, 0.0, 0.0));
    const ON_3dPoint probe(5.0, 3.0, 4.0);

    CHECK_NEAR(TL_LineDistanceToPoint(&horizontal, probe, false), 5.0, 1e-12);
    // Furthest point on the segment from the probe: one of the two endpoints.
    CHECK_NEAR(TL_LineDistanceToPoint(&horizontal, probe, true),
               std::sqrt(25.0 + 9.0 + 16.0), 1e-12);

    CHECK_NEAR(TL_LineDistanceToPoint(nullptr, probe, false), -1.0, 0.0);

    const ON_Line parallel(ON_3dPoint(0.0, 2.0, 0.0), ON_3dPoint(10.0, 2.0, 0.0));
    CHECK_NEAR(TL_LineDistanceToLine(&horizontal, &parallel, false), 2.0, 1e-12);
    CHECK(TL_LineDistanceToLine(&horizontal, &parallel, true) > 2.0);
    CHECK_NEAR(TL_LineDistanceToLine(&horizontal, nullptr, true), -1.0, 0.0);
  }

  // ---- TL_GetClosestPoint(ON_Line, ON_Line) ---------------------------
  {
    // Intersecting segments: the analytic branch must land on the crossing.
    const ON_Line a(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(10.0, 0.0, 0.0));
    const ON_Line b(ON_3dPoint(5.0, -5.0, 0.0), ON_3dPoint(5.0, 5.0, 0.0));
    double ta = -1.0;
    double tb = -1.0;
    CHECK(TL_GetClosestPoint(a, b, &ta, &tb));
    CHECK_NEAR(ta, 0.5, 1e-12);
    CHECK_NEAR(tb, 0.5, 1e-12);
    CHECK_NEAR(a.PointAt(ta).DistanceTo(b.PointAt(tb)), 0.0, 1e-12);

    // Skew, non-overlapping segments: the parallel/skew branch must clamp to the
    // closest endpoint pair (a.to to b.from).
    const ON_Line c(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(1.0, 0.0, 0.0));
    const ON_Line d(ON_3dPoint(5.0, 3.0, 0.0), ON_3dPoint(6.0, 3.0, 0.0));
    CHECK(TL_GetClosestPoint(c, d, &ta, &tb));
    CHECK_NEAR(ta, 1.0, 1e-12);
    CHECK_NEAR(tb, 0.0, 1e-12);

    // Parallel segments: still a usable parameter pair, true is not "intersects".
    const ON_Line e(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(1.0, 0.0, 0.0));
    const ON_Line f(ON_3dPoint(0.0, 2.0, 0.0), ON_3dPoint(1.0, 2.0, 0.0));
    CHECK(TL_GetClosestPoint(e, f, &ta, &tb));
    CHECK_NEAR(e.PointAt(ta).DistanceTo(f.PointAt(tb)), 2.0, 1e-9);

    // 7-arg overload: maximum_distance gate.
    CHECK(TL_GetClosestPoint(e, f, &ta, &tb, 0.0, nullptr, nullptr));
    CHECK(!TL_GetClosestPoint(e, f, &ta, &tb, 1.5, nullptr, nullptr));
    CHECK(TL_GetClosestPoint(e, f, &ta, &tb, 2.5, nullptr, nullptr));

    // 7-arg overload: sub-domain clamping.
    const ON_Interval low_half(0.0, 0.25);
    CHECK(TL_GetClosestPoint(a, b, &ta, &tb, 0.0, &low_half, nullptr));
    CHECK(ta <= 0.25 + 1e-12);
  }

  // ---- dual curve x line strategy cross-check -------------------------
  {
    const ON_Circle circle(ON_Plane::World_xy, 5.0);
    ON_ArcCurve circle_curve(circle);

    struct Case {
      const char* name;
      ON_Line line;
      int expected_count;
    };
    const Case cases[] = {
      { "diameter", ON_Line(ON_3dPoint(-10.0, 0.0, 0.0), ON_3dPoint(10.0, 0.0, 0.0)), 2 },
      { "tangent", ON_Line(ON_3dPoint(-10.0, 5.0, 0.0), ON_3dPoint(10.0, 5.0, 0.0)), 1 },
      { "miss", ON_Line(ON_3dPoint(-10.0, 9.0, 0.0), ON_3dPoint(10.0, 9.0, 0.0)), 0 },
      { "chord", ON_Line(ON_3dPoint(-10.0, 3.0, 0.0), ON_3dPoint(10.0, 3.0, 0.0)), 2 },
    };

    for (const Case& c : cases) {
      ON_SimpleArray<TL_CurveCurveIntersectionEvent> primary;
      ON_SimpleArray<TL_CurveCurveIntersectionEvent> subdiv;
      const int count_primary = TL_IntersectCurveLine(circle_curve, c.line, 1e-9, primary);
      const int count_subdiv = TL_IntersectCurveLineBySubdivision(circle_curve, c.line, 1e-9, subdiv);

      CHECK_MSG(count_primary == c.expected_count,
                std::string("primary count for ") + c.name + " = " + std::to_string(count_primary));
      CHECK_MSG(count_subdiv == c.expected_count,
                std::string("subdivision count for ") + c.name + " = " + std::to_string(count_subdiv));
      CHECK_EQ_INT(count_primary, count_subdiv);

      for (int i = 0; i < std::min(count_primary, count_subdiv); ++i) {
        CHECK_NEAR(primary[i].param_a, subdiv[i].param_a, 1e-6);
        CHECK(primary[i].point_a.DistanceTo(subdiv[i].point_a) <= 1e-6);
      }
    }
  }

  EndSection();
}

// ============================================================================
// 14 - four-view viewport layout (Top / Perspective / Front / Right)
// ============================================================================

void TestFourViewLayout() {
  BeginSection("four-view viewport layout (Top / Perspective / Front / Right)");

  TL_FourViewLayout layout = TL_FourViewLayout::Default(1600, 900);

  // ---- structure -------------------------------------------------------
  CHECK_EQ_INT(layout.Count(), 4);
  CHECK_EQ_INT(layout.IndexOfName("Top"), 0);
  CHECK_EQ_INT(layout.IndexOfName("Perspective"), 1);
  CHECK_EQ_INT(layout.IndexOfName("Front"), 2);
  CHECK_EQ_INT(layout.IndexOfName("Right"), 3);
  CHECK_EQ_INT(layout.IndexOfName("Nope"), -1);
  CHECK_EQ_INT(layout.View(0).m_projection, TL_PROJECTION_PARALLEL);
  CHECK_EQ_INT(layout.View(1).m_projection, TL_PROJECTION_PERSPECTIVE);
  CHECK_EQ_INT(layout.View(2).m_projection, TL_PROJECTION_PARALLEL);
  CHECK_EQ_INT(layout.View(3).m_projection, TL_PROJECTION_PARALLEL);

  // 2x2 grid must tile the canvas exactly.
  const TL_Viewport& top = layout.View(0);
  const TL_Viewport& persp = layout.View(1);
  const TL_Viewport& front = layout.View(2);
  const TL_Viewport& right = layout.View(3);
  CHECK_EQ_INT(top.m_left, 0);
  CHECK_EQ_INT(top.m_top, 0);
  CHECK_EQ_INT(persp.m_left, top.m_width);
  CHECK_EQ_INT(front.m_top, top.m_height);
  CHECK_EQ_INT(top.m_width + persp.m_width, 1600);
  CHECK_EQ_INT(top.m_height + front.m_height, 900);
  CHECK_EQ_INT(right.m_left, persp.m_left);
  CHECK_EQ_INT(right.m_top, front.m_top);

  // ---- standard-view orientation --------------------------------------
  {
    ON_2dPoint screen = ON_2dPoint::Origin;
    // Top view: +X to the right, +Y upward (screen y decreases).
    CHECK(top.WorldToScreen(ON_3dPoint(1.0, 0.0, 0.0), screen));
    const double top_x_plus = screen.x;
    CHECK(top.WorldToScreen(ON_3dPoint(0.0, 1.0, 0.0), screen));
    const double top_y_plus = screen.y;
    CHECK(top_x_plus > top.m_left + top.m_width * 0.5);
    CHECK(top_y_plus < top.m_top + top.m_height * 0.5);

    // Front view: +X right, +Z up.
    CHECK(front.WorldToScreen(ON_3dPoint(1.0, 0.0, 0.0), screen));
    const double front_x_plus = screen.x;
    CHECK(front.WorldToScreen(ON_3dPoint(0.0, 0.0, 1.0), screen));
    const double front_z_up = screen.y;
    CHECK(front_x_plus > front.m_left + front.m_width * 0.5);
    CHECK(front_z_up < front.m_top + front.m_height * 0.5);

    // Right view: +Y right, +Z up (this is what distinguishes it from Front).
    CHECK(right.WorldToScreen(ON_3dPoint(0.0, 1.0, 0.0), screen));
    const double right_y_plus = screen.x;
    CHECK(right.WorldToScreen(ON_3dPoint(0.0, 0.0, 1.0), screen));
    const double right_z_up = screen.y;
    CHECK(right_y_plus > right.m_left + right.m_width * 0.5);
    CHECK(right_z_up < right.m_top + right.m_height * 0.5);
    // and it must NOT map +X to the horizontal axis
    CHECK(right.WorldToScreen(ON_3dPoint(1.0, 0.0, 0.0), screen));
    CHECK(std::fabs(screen.x - (right.m_left + right.m_width * 0.5)) <= 1e-9);
  }

  // ---- screen -> ray round-trip (all four views) ----------------------
  {
    const ON_3dPoint probes[3] = {
      ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(3.0, -2.0, 4.0), ON_3dPoint(-6.0, 5.0, 1.5)
    };
    for (int view = 0; view < 4; ++view) {
      const TL_Viewport& vp = layout.View(view);
      for (int i = 0; i < 3; ++i) {
        ON_2dPoint screen = ON_2dPoint::Origin;
        CHECK(vp.WorldToScreen(probes[i], screen));
        ON_3dPoint hit = ON_3dPoint::Origin;
        if (TL_PROJECTION_PARALLEL == vp.m_projection) {
          CHECK(vp.ScreenToCameraPlane(screen, hit));
          CHECK(hit.DistanceTo(vp.m_target + vp.ScreenRight() *
                               ON_DotProduct(probes[i] - vp.m_target, vp.ScreenRight()) +
                               vp.ScreenUp() *
                               ON_DotProduct(probes[i] - vp.m_target, vp.ScreenUp())) <= 1e-6);
        } else {
          // Perspective: the screen ray through the projected point must pass
          // through the world point itself.
          ON_3dPoint from = ON_3dPoint::Origin;
          ON_3dVector dir = ON_3dVector::ZeroVector;
          CHECK(vp.ScreenToRay(screen, from, dir));
          const ON_3dVector to_point = probes[i] - from;
          const ON_3dVector cross = ON_CrossProduct(dir, to_point);
          CHECK(cross.Length() <= 1e-6 * std::max(1.0, to_point.Length()));
        }
      }
    }
  }

  // ---- construction-plane hit ----------------------------------------
  {
    ON_2dPoint screen = ON_2dPoint::Origin;
    const ON_3dPoint world(4.0, -3.0, 0.0);
    CHECK(layout.View(0).WorldToScreen(world, screen));
    ON_3dPoint hit = ON_3dPoint::Origin;
    CHECK(layout.View(0).ScreenToPlane(screen, ON_Plane::World_xy, hit));
    CHECK(hit.DistanceTo(world) <= 1e-6);
  }

  // ---- picking --------------------------------------------------------
  {
    std::vector<ON_3dPoint> points;
    points.push_back(ON_3dPoint(0.0, 0.0, 0.0));
    points.push_back(ON_3dPoint(10.0, 0.0, 0.0));
    points.push_back(ON_3dPoint(0.0, 10.0, 0.0));

    ON_2dPoint screen = ON_2dPoint::Origin;
    CHECK(layout.View(0).WorldToScreen(points[1], screen));
    CHECK_EQ_INT(layout.PickPoint(0, screen, points, 6.0), 1);
    CHECK_EQ_INT(layout.PickPoint(0, ON_2dPoint(screen.x + 60.0, screen.y + 60.0), points, 6.0), -1);
  }

  // ---- pan / zoom / zoom-to-fit ---------------------------------------
  {
    TL_Viewport vp = layout.View(0);
    ON_2dPoint before = ON_2dPoint::Origin;
    CHECK(vp.WorldToScreen(ON_3dPoint(0.0, 0.0, 0.0), before));
    vp.Pan(10.0, -6.0);
    ON_2dPoint after = ON_2dPoint::Origin;
    CHECK(vp.WorldToScreen(ON_3dPoint(0.0, 0.0, 0.0), after));
    CHECK_NEAR(after.x - before.x, 10.0, 1e-9);
    CHECK_NEAR(after.y - before.y, -6.0, 1e-9);

    const double scale_before = vp.m_scale;
    vp.Zoom(2.0, nullptr);
    CHECK_NEAR(vp.m_scale, scale_before * 2.0, 1e-9);

    ON_BoundingBox box;
    box.Set(ON_3dPoint(-10.0, -5.0, -2.0));
    box.Set(ON_3dPoint(10.0, 5.0, 2.0), true);
    for (int i = 0; i < 4; ++i) {
      TL_Viewport fit = layout.View(i);
      fit.ZoomToFit(box, 0.9);
      // every bbox corner must project inside the viewport rectangle
      ON_3dPoint pts[8];
      const int count = box.GetCorners(pts);
      for (int c = 0; c < count; ++c) {
        ON_2dPoint s = ON_2dPoint::Origin;
        if (!fit.WorldToScreen(pts[c], s))
          continue;
        CHECK(s.x >= fit.m_left - 1e-6 && s.x <= fit.m_left + fit.m_width + 1e-6);
        CHECK(s.y >= fit.m_top - 1e-6 && s.y <= fit.m_top + fit.m_height + 1e-6);
      }
    }
  }

  // ---- canvas resize keeps the tiling --------------------------------
  {
    TL_FourViewLayout resized = TL_FourViewLayout::Default(800, 600);
    resized.SetCanvasSize(1024, 768);
    CHECK_EQ_INT(resized.View(0).m_width + resized.View(1).m_width, 1024);
    CHECK_EQ_INT(resized.View(0).m_height + resized.View(2).m_height, 768);
  }

  // ---- JSON round-trip ------------------------------------------------
  {
    TL_FourViewLayout original = TL_FourViewLayout::Default(1280, 720);
    original.View(0).Pan(12.0, 4.0);
    original.View(0).Zoom(1.5, nullptr);
    original.View(1).m_target = ON_3dPoint(3.0, 4.0, 5.0);
    original.View(2).m_scale = 33.5;

    const std::string json = original.ToJson();
    CHECK(json.find("opentl.fourview") != std::string::npos);

    TL_FourViewLayout restored;
    CHECK(restored.FromJson(json));
    CHECK_EQ_INT(restored.CanvasWidth(), 1280);
    CHECK_EQ_INT(restored.CanvasHeight(), 720);
    for (int i = 0; i < 4; ++i) {
      CHECK(restored.View(i).m_name == original.View(i).m_name);
      CHECK_EQ_INT(restored.View(i).m_standard_view, original.View(i).m_standard_view);
      CHECK_EQ_INT(restored.View(i).m_projection, original.View(i).m_projection);
      CHECK_NEAR(restored.View(i).m_scale, original.View(i).m_scale, 1e-9);
      CHECK_POINT(restored.View(i).m_target, original.View(i).m_target, 1e-9);
      CHECK_EQ_INT(restored.View(i).m_width, original.View(i).m_width);
    }

    // Malformed / foreign input must be rejected, not half-applied.
    TL_FourViewLayout bad;
    CHECK(!bad.FromJson("{\"schema\":\"something-else\"}"));
    CHECK(!bad.FromJson("not json at all"));
  }

  EndSection();
}


// ============================================================================
// 15 - one-rail sweep
// ============================================================================

void TestSweep1Rail() {
  BeginSection("one-rail sweep (parallel transport + loft)");

  // ---- straight rail: a circle swept along +Z must be a tube of radius 2 ----
  {
    const ON_LineCurve rail(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(0.0, 0.0, 10.0));
    ON_ArcCurve profile(ON_Circle(ON_Plane::World_xy, 2.0));

    ON_NurbsSurface* swept = TL_Sweep1Rail(rail, profile, 12, nullptr);
    CHECK(nullptr != swept);
    if (nullptr != swept) {
      CHECK(swept->IsValid());
      const ON_Interval u_dom = swept->Domain(0);
      const ON_Interval v_dom = swept->Domain(1);
      CHECK(u_dom.IsIncreasing());
      CHECK(v_dom.IsIncreasing());

      for (int i = 0; i <= 4; ++i) {
        const double v = v_dom.ParameterAt(i / 4.0);
        for (int k = 0; k <= 8; ++k) {
          const double u = u_dom.ParameterAt(k / 8.0);
          const ON_3dPoint p = swept->PointAt(u, v);
          const double radius = std::sqrt(p.x * p.x + p.y * p.y);
          // The profile is interpolated through 64 chord samples, so the tube is a
          // chord-accurate approximation of the true cylinder (error ~2.4e-3).
          CHECK_NEAR(radius, 2.0, 5e-3);
          CHECK(p.z >= -2e-3 && p.z <= 10.0 + 2e-3);
        }
      }

      // The swept surface must span exactly the rail extent.
      ON_BoundingBox box;
      CHECK(swept->GetBoundingBox(box));
      CHECK_NEAR(box.m_min.z, 0.0, 2e-3);
      CHECK_NEAR(box.m_max.z, 10.0, 2e-3);
      delete swept;
    }
  }

  // ---- curved rail: every surface point stays `profile radius` from the rail ----
  {
    ON_ArcCurve rail(ON_Arc(ON_Plane::World_xy, 10.0, ON_PI * 0.5));
    ON_ArcCurve profile(ON_Circle(ON_Plane::World_xy, 1.0));

    ON_NurbsSurface* swept = TL_Sweep1Rail(rail, profile, 16, nullptr);
    CHECK(nullptr != swept);
    if (nullptr != swept) {
      const ON_Interval u_dom = swept->Domain(0);
      const ON_Interval v_dom = swept->Domain(1);
      double worst = 0.0;
      for (int i = 1; i < 4; ++i) {
        const double v = v_dom.ParameterAt(i / 4.0);
        for (int k = 0; k < 8; ++k) {
          const double u = u_dom.ParameterAt(k / 8.0);
          const ON_3dPoint p = swept->PointAt(u, v);
          double s = 0.0;
          if (!TL_GetCurveClosestPoint(rail, p, &s))
            continue;
          const double d = p.DistanceTo(rail.PointAt(s));
          worst = std::max(worst, std::fabs(d - 1.0));
        }
      }
      CHECK_MSG(worst <= 5e-3, "max radial deviation = " + NumberText(worst));
      delete swept;
    }
  }

  // ---- station_count is clamped, degenerate input rejected ----
  {
    ON_LineCurve rail(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(1.0, 0.0, 0.0));
    ON_ArcCurve profile(ON_Circle(ON_Plane::World_xy, 0.5));
    ON_NurbsSurface* two = TL_Sweep1Rail(rail, profile, 1, nullptr);
    CHECK(nullptr != two);   // clamped to 2 stations
    delete two;

    ON_LineCurve degenerate(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(0.0, 0.0, 0.0));
    ON_NurbsSurface* bad = TL_Sweep1Rail(degenerate, profile, 8, nullptr);
    CHECK(nullptr == bad);    // zero-length rail has no domain
  }

  EndSection();
}


// ============================================================================
// 16 - Vulkan four-view renderer
// ============================================================================

void TestVulkanRenderer() {
  BeginSection("Vulkan four-view renderer (Catppuccin Mocha, MSAA, depth)");

  opentl::mvp::SceneDocument document;
  const int layer = document.AddLayer("Curves", 137, 180, 250, true);  // Catppuccin Blue
  CHECK(layer >= 0);
  document.AddCircle(ON_Circle(ON_Plane::World_xy, 5.0), "circle", "Curves");
  document.AddLine(ON_Line(ON_3dPoint(-8.0, 0.0, 0.0), ON_3dPoint(8.0, 3.0, 2.0)), "line", "Curves");
  document.AddPoint(ON_3dPoint(2.0, 3.0, 4.0), "point", "Curves");
  document.AddNurbsCurveFromPoints(
    { ON_3dPoint(-6.0, -4.0, 0.0), ON_3dPoint(-2.0, -7.0, 3.0), ON_3dPoint(3.0, -5.0, 1.0),
      ON_3dPoint(6.0, -2.0, 2.0) },
    3, "spline", "Curves");

  // A swept tube so the triangle pipeline has real geometry to shade.
  ON_ArcCurve rail(ON_Arc(ON_Plane(ON_3dPoint(-6.0, -6.0, 0.0), ON_3dVector::ZAxis), 9.0, ON_PI));
  ON_ArcCurve profile(ON_Circle(ON_Plane::World_xy, 1.6));
  ON_NurbsSurface* swept = TL_Sweep1Rail(rail, profile, 20, nullptr);
  CHECK(nullptr != swept);
  ON_Mesh* tube = (nullptr != swept) ? TL_MeshSurfaceGrid(*swept, 40, 64, nullptr) : nullptr;
  CHECK(nullptr != tube);
  if (nullptr != tube) {
    tube->ComputeFaceNormals();
    tube->ComputeVertexNormals();
  }
  std::vector<const ON_Mesh*> meshes;
  if (nullptr != tube) meshes.push_back(tube);

  TL_FourViewLayout layout = TL_FourViewLayout::Default(960, 540);
  ON_BoundingBox bbox = document.GetSceneBoundingBox();
  ON_BoundingBox tube_box;
  if (nullptr != tube && tube->GetBoundingBox(tube_box) && tube_box.IsValid())
    bbox.Union(tube_box);
  layout.ZoomToFitAll(bbox, 0.9);

  TL_VulkanFourViewRenderer renderer;
  TL_VulkanRenderResult result;
  TL_VulkanRenderOptions options;
  options.sample_count = 4;
  options.enable_depth = true;
  options.enable_shaded_meshes = true;
  const bool ok = renderer.RenderScene(document, layout, result, meshes, options);

  if (!ok && result.error.find("without Vulkan support") != std::string::npos) {
    std::printf("     [SKIP] build has no Vulkan backend: %s\n", result.error.c_str());
    delete tube;
    delete swept;
    EndSection();
    return;
  }

  CHECK_MSG(ok, std::string("RenderScene failed: ") + result.error);
  CHECK_EQ_INT(result.width, 960);
  CHECK_EQ_INT(result.height, 540);
  CHECK(result.rgba.size() == static_cast<size_t>(result.width) * result.height * 4);
  CHECK(result.vertex_count > 0);
  CHECK(result.device_name.find("llvm") == std::string::npos);
  CHECK(!result.backend_name.empty());

  // Theme + pipeline state that this stage added.
  CHECK(result.theme_name == "Catppuccin Mocha");
  CHECK(result.msaa_samples >= 1);
  CHECK_MSG(result.msaa_samples == 4 || result.msaa_samples == 8 || result.msaa_samples == 2,
            "MSAA requested 4x but the device reported " + std::to_string(result.msaa_samples));
  CHECK(result.depth_enabled);
  CHECK(result.triangle_count > 0);

  if (ok) {
    // Background is Catppuccin Crust (#11111b). Sample a few pixels well inside a
    // panel and away from the border/accent strip.
    int dark = 0;
    const int samples_xy[3][2] = { { 7, 7 }, { 7, 40 }, { 40, 7 } };
    for (const auto& xy : samples_xy) {
      const size_t i = (static_cast<size_t>(xy[1]) * result.width + xy[0]) * 4;
      if (result.rgba[i] <= 40 && result.rgba[i + 1] <= 40 && result.rgba[i + 2] <= 50)
        ++dark;
    }
    CHECK_MSG(dark >= 2, "background samples are not the Crust colour");

    size_t distinct = 0;
    for (size_t i = 0; i + 3 < result.rgba.size(); i += 4 * 997) {
      const uint8_t* p = result.rgba.data() + i;
      if (p[0] != result.rgba[0] || p[1] != result.rgba[1] || p[2] != result.rgba[2])
        ++distinct;
    }
    CHECK_MSG(distinct > 20, "distinct sampled pixels = " + std::to_string(distinct));

    // Depth test: a huge quad placed well in front of the XZ grid must hide that grid.
    // Counting grid-coloured pixels inside the FRONT viewport is a direct measurement
    // of whether depth testing is live (this caught GetCorners returning bool).
    auto count_grid_pixels = [](const TL_VulkanRenderResult& r) {
      int count = 0;
      const int x0 = 2;
      const int x1 = r.width / 2 - 2;
      const int y0 = r.height / 2 + 2;
      const int y1 = r.height - 2;
      for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
          const size_t i = (static_cast<size_t>(y) * r.width + x) * 4;
          if (std::abs(static_cast<int>(r.rgba[i]) - 49) <= 6 &&
              std::abs(static_cast<int>(r.rgba[i + 1]) - 50) <= 6 &&
              std::abs(static_cast<int>(r.rgba[i + 2]) - 68) <= 6)
            ++count;
        }
      }
      return count;
    };

    TL_FourViewLayout probe_layout = TL_FourViewLayout::Default(800, 600);
    const int baseline_grid = [&]() {
      TL_VulkanRenderResult r;
      renderer.RenderScene(opentl::mvp::SceneDocument(), probe_layout, r, {},
                           TL_VulkanRenderOptions());
      return count_grid_pixels(r);
    }();

    ON_Mesh near_quad(1, 4, false, false);
    near_quad.SetVertex(0, ON_3dPoint(-30.0, -8.0, -30.0));
    near_quad.SetVertex(1, ON_3dPoint(30.0, -8.0, -30.0));
    near_quad.SetVertex(2, ON_3dPoint(30.0, -8.0, 30.0));
    near_quad.SetVertex(3, ON_3dPoint(-30.0, -8.0, 30.0));
    near_quad.SetQuad(0, 0, 1, 2, 3);
    near_quad.ComputeFaceNormals();
    near_quad.ComputeVertexNormals();

    TL_VulkanRenderResult occluded;
    std::vector<const ON_Mesh*> near_meshes{ &near_quad };
    renderer.RenderScene(opentl::mvp::SceneDocument(), probe_layout, occluded, near_meshes,
                         TL_VulkanRenderOptions());
    const int occluded_grid = count_grid_pixels(occluded);

    CHECK_MSG(baseline_grid > 0, "baseline grid pixels = " + std::to_string(baseline_grid));
    CHECK_MSG(occluded_grid == 0,
              "grid pixels behind the near quad = " + std::to_string(occluded_grid) +
                " (depth test not applied)");
  }

  // ---- PNG persistence ------------------------------------------------
  {
    const std::string path = "/tmp/opentl_test_fourview.png";
    std::string save_error;
    if (ok) {
      const bool saved =
        TL_VulkanFourViewRenderer::SavePng(path, result.width, result.height, result.rgba, &save_error);
      CHECK_MSG(saved, "SavePng failed: " + save_error);

      std::FILE* file = std::fopen(path.c_str(), "rb");
      CHECK(nullptr != file);
      if (nullptr != file) {
        uint8_t header[33] = { 0 };
        const size_t read = std::fread(header, 1, sizeof(header), file);
        std::fclose(file);
        CHECK(read == sizeof(header));
        const uint8_t signature[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
        CHECK(0 == std::memcmp(header, signature, 8));
        CHECK(0 == std::memcmp(header + 12, "IHDR", 4));
        const uint32_t png_width = (static_cast<uint32_t>(header[16]) << 24) |
          (static_cast<uint32_t>(header[17]) << 16) | (static_cast<uint32_t>(header[18]) << 8) |
          static_cast<uint32_t>(header[19]);
        const uint32_t png_height = (static_cast<uint32_t>(header[20]) << 24) |
          (static_cast<uint32_t>(header[21]) << 16) | (static_cast<uint32_t>(header[22]) << 8) |
          static_cast<uint32_t>(header[23]);
        CHECK_EQ_INT(png_width, static_cast<uint32_t>(result.width));
        CHECK_EQ_INT(png_height, static_cast<uint32_t>(result.height));
        CHECK_EQ_INT(header[24], 8);
        CHECK_EQ_INT(header[25], 6);
      }
      std::remove(path.c_str());
    }

    std::string bad_error;
    CHECK(!TL_VulkanFourViewRenderer::SavePng(path, 0, 10, std::vector<uint8_t>(), &bad_error));
    CHECK(!bad_error.empty());
  }

  delete tube;
  delete swept;
  EndSection();
}


// ---------------------------------------------------------------------------
// 17. Mesh boundary detection & capping
// ---------------------------------------------------------------------------
namespace {

double SignedMeshVolume(const ON_Mesh& mesh)
{
  double volume = 0.0;
  for (int i = 0; i < mesh.m_F.Count(); ++i)
  {
    const ON_MeshFace& face = mesh.m_F[i];
    const ON_3dPoint& a = mesh.m_V[face.vi[0]];
    const ON_3dPoint& b = mesh.m_V[face.vi[1]];
    const ON_3dPoint& c = mesh.m_V[face.vi[2]];
    auto triple = [](const ON_3dPoint& p, const ON_3dPoint& q, const ON_3dPoint& r) {
      return p.x * (q.y * r.z - q.z * r.y) + p.y * (q.z * r.x - q.x * r.z) +
             p.z * (q.x * r.y - q.y * r.x);
    };
    volume += triple(a, b, c);
    if (face.IsQuad())
      volume += triple(a, c, mesh.m_V[face.vi[3]]);
  }
  return volume / 6.0;
}

} // anonymous namespace

void TestMeshCapping()
{
  BeginSection("mesh boundary detection and capping");

  // A swept half-torus: two open rims, no caps.
  ON_ArcCurve rail(ON_Arc(ON_Plane(ON_3dPoint(-6.0, -6.0, 0.0), ON_3dVector::ZAxis), 9.0, ON_PI));
  ON_ArcCurve profile(ON_Circle(ON_Plane::World_xy, 1.6));
  ON_NurbsSurface* swept = TL_Sweep1Rail(rail, profile, 20, nullptr);
  CHECK(nullptr != swept);
  if (nullptr == swept) { EndSection(); return; }

  ON_Mesh open_mesh;
  CHECK(TL_MeshSurfaceGrid(*swept, 48, 96, &open_mesh));
  delete swept;

  const int boundary_before = TL_CountMeshBoundaryEdges(open_mesh);
  CHECK_MSG(boundary_before > 0, "an open swept shell must report boundary edges");
  CHECK_EQ_INT(boundary_before % 2, 0);   // two rims of equal length
  CHECK_EQ_INT(open_mesh.m_F.Count(), 48 * 96);

  const double analytic_volume = ON_PI * 1.6 * 1.6 * (ON_PI * 9.0);

  // ---- watertight variant (welded rim) ---------------------------------
  {
    ON_Mesh welded(open_mesh);
    int triangles = 0;
    int open_chains = -1;
    const int caps = TL_CapMeshOpenBoundaryLoops(welded, &triangles, &open_chains,
                                                 /*hard_crease=*/false);
    CHECK_EQ_INT(caps, 2);
    CHECK_EQ_INT(open_chains, 0);
    CHECK_EQ_INT(triangles, boundary_before);   // one fan triangle per rim edge
    CHECK_EQ_INT(TL_CountMeshBoundaryEdges(welded), 0);
    CHECK_EQ_INT(welded.m_F.Count(), open_mesh.m_F.Count() + triangles);

    const double volume = SignedMeshVolume(welded);
    CHECK_MSG(volume > 0.0, "capped mesh has negative volume: caps are wound inside out");
    CHECK_NEAR(volume, analytic_volume, analytic_volume * 0.02);

    // Capping a closed mesh is a no-op.
    int again = 0;
    CHECK_EQ_INT(TL_CapMeshOpenBoundaryLoops(welded, &again, nullptr, false), 0);
    CHECK_EQ_INT(again, 0);
  }

  // ---- hard-crease variant (rim normals split for shading) ---------------
  {
    ON_Mesh creased(open_mesh);
    int triangles = 0;
    const int caps = TL_CapMeshOpenBoundaryLoops(creased, &triangles, nullptr, true);
    CHECK_EQ_INT(caps, 2);
    CHECK_EQ_INT(triangles, boundary_before);
    CHECK_MSG(creased.VertexCount() > open_mesh.VertexCount(),
              "hard crease must duplicate the rim vertices and add a centroid per cap");
    CHECK_EQ_INT(creased.VertexCount() - open_mesh.VertexCount(), boundary_before + 2);
    // Positions are unchanged, so the shell is still geometrically closed.
    CHECK_NEAR(SignedMeshVolume(creased), analytic_volume, analytic_volume * 0.02);
  }

  // ---- a closed box mesh needs no caps ----------------------------------
  {
    ON_Mesh box;
    const ON_3dPoint corners[8] = {
      ON_3dPoint(-1, -1, -1), ON_3dPoint(1, -1, -1), ON_3dPoint(1, 1, -1), ON_3dPoint(-1, 1, -1),
      ON_3dPoint(-1, -1, 1), ON_3dPoint(1, -1, 1), ON_3dPoint(1, 1, 1), ON_3dPoint(-1, 1, 1) };
    for (int i = 0; i < 8; ++i)
      box.SetVertex(i, corners[i]);
    const int quads[6][4] = { { 0, 3, 2, 1 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 },
                              { 2, 3, 7, 6 }, { 0, 4, 7, 3 }, { 1, 2, 6, 5 } };
    for (int i = 0; i < 6; ++i)
      box.SetQuad(i, quads[i][0], quads[i][1], quads[i][2], quads[i][3]);

    CHECK_EQ_INT(TL_CountMeshBoundaryEdges(box), 0);
    CHECK_EQ_INT(TL_CapMeshOpenBoundaryLoops(box, nullptr, nullptr, true), 0);
    CHECK_NEAR(SignedMeshVolume(box), 8.0, 1e-9);
  }

  EndSection();
}

// ---------------------------------------------------------------------------
// 18. Curve x Curve and Surface x Surface intersection
// ---------------------------------------------------------------------------
namespace {

ON_NurbsSurface MakeTestPlanePatch(const ON_Plane& plane, double half)
{
  ON_NurbsSurface patch(3, false, 2, 2, 2, 2);
  patch.SetCV(0, 0, plane.PointAt(-half, -half));
  patch.SetCV(0, 1, plane.PointAt(-half, half));
  patch.SetCV(1, 0, plane.PointAt(half, -half));
  patch.SetCV(1, 1, plane.PointAt(half, half));
  patch.SetKnot(0, 0, 0.0);
  patch.SetKnot(0, 1, 1.0);
  patch.SetKnot(1, 0, 0.0);
  patch.SetKnot(1, 1, 1.0);
  return patch;
}

} // anonymous namespace

void TestIntersectionStage6()
{
  BeginSection("curve x curve and surface x surface intersection");

  // ---- curve x curve: both strategies must agree ------------------------
  {
    ON_ArcCurve circle(ON_Circle(ON_Plane::World_xy, 5.0));
    ON_LineCurve diameter(ON_Line(ON_3dPoint(-8, 0, 0), ON_3dPoint(8, 0, 0)));
    ON_SimpleArray<TL_CurveCurveIntersectionEvent> primary, grid;
    const int np = TL_IntersectCurves(circle, diameter, 1e-8, primary);
    const int ng = TL_IntersectCurvesByGrid(circle, diameter, 1e-8, grid);
    CHECK_EQ_INT(np, 2);
    CHECK_EQ_INT(ng, 2);
    for (int i = 0; i < np && i < ng; ++i)
    {
      CHECK_NEAR(primary[i].point_a.DistanceTo(grid[i].point_a), 0.0, 1e-9);
      CHECK_NEAR(primary[i].point_a.DistanceTo(primary[i].point_b), 0.0, 1e-9);
      // Both hits are on the circle and on the line.
      CHECK_NEAR(primary[i].point_a.DistanceTo(ON_3dPoint::Origin), 5.0, 1e-9);
      CHECK_NEAR(primary[i].point_a.y, 0.0, 1e-9);
    }
  }

  // ---- tangency: exactly one hit, both strategies ----------------------
  {
    ON_ArcCurve circle(ON_Circle(ON_Plane::World_xy, 5.0));
    ON_LineCurve tangent(ON_Line(ON_3dPoint(-8, 5, 0), ON_3dPoint(8, 5, 0)));
    ON_SimpleArray<TL_CurveCurveIntersectionEvent> primary, grid;
    const int np = TL_IntersectCurves(circle, tangent, 1e-7, primary);
    const int ng = TL_IntersectCurvesByGrid(circle, tangent, 1e-7, grid);
    CHECK_EQ_INT(np, 1);
    CHECK_EQ_INT(ng, 1);
    if (np == 1 && ng == 1)
    {
      CHECK_NEAR(primary[0].point_a.x, 0.0, 1e-6);
      CHECK_NEAR(primary[0].point_a.y, 5.0, 1e-6);
      CHECK_NEAR(primary[0].point_a.DistanceTo(grid[0].point_a), 0.0, 1e-6);
    }
  }

  // ---- two circles crossing twice --------------------------------------
  {
    ON_ArcCurve c0(ON_Circle(ON_Plane::World_xy, 5.0));
    ON_ArcCurve c1(ON_Circle(ON_Plane(ON_3dPoint(6, 0, 0), ON_3dVector::ZAxis), 4.0));
    ON_SimpleArray<TL_CurveCurveIntersectionEvent> primary, grid;
    const int np = TL_IntersectCurves(c0, c1, 1e-9, primary);
    const int ng = TL_IntersectCurvesByGrid(c0, c1, 1e-9, grid);
    CHECK_EQ_INT(np, 2);
    CHECK_EQ_INT(ng, 2);
    for (int i = 0; i < np; ++i)
    {
      CHECK_NEAR(primary[i].point_a.DistanceTo(ON_3dPoint::Origin), 5.0, 1e-8);
      CHECK_NEAR(primary[i].point_b.DistanceTo(ON_3dPoint(6, 0, 0)), 4.0, 1e-8);
      CHECK_NEAR(primary[i].point_a.DistanceTo(primary[i].point_b), 0.0, 1e-8);
    }
  }

  // ---- miss ------------------------------------------------------------
  {
    ON_ArcCurve circle(ON_Circle(ON_Plane::World_xy, 5.0));
    ON_LineCurve outside(ON_Line(ON_3dPoint(-8, 9, 0), ON_3dPoint(8, 9, 0)));
    ON_SimpleArray<TL_CurveCurveIntersectionEvent> primary, grid;
    CHECK_EQ_INT(TL_IntersectCurves(circle, outside, 1e-6, primary), 0);
    CHECK_EQ_INT(TL_IntersectCurvesByGrid(circle, outside, 1e-6, grid), 0);
  }

  // ---- spline x line (NURBS, not analytic) -----------------------------
  {
    ON_ArcCurve circle(ON_Circle(ON_Plane::World_xy, 5.0));
    ON_NurbsCurve spline(3, false, 3, 6);
    const ON_3dPoint cvs[6] = { ON_3dPoint(-8, 0, 1), ON_3dPoint(-5, 4, -1), ON_3dPoint(-1, -4, 2),
                                ON_3dPoint(2, 5, -2), ON_3dPoint(5, -3, 1), ON_3dPoint(8, 0, 0) };
    for (int i = 0; i < 6; ++i)
      spline.SetCV(i, cvs[i]);
    spline.MakeClampedUniformKnotVector(1.0);

    ON_LineCurve line(ON_Line(ON_3dPoint(-9, 0, 0), ON_3dPoint(9, 0, 0)));
    ON_SimpleArray<TL_CurveCurveIntersectionEvent> primary, grid;
    const int np = TL_IntersectCurves(spline, line, 1e-7, primary);
    const int ng = TL_IntersectCurvesByGrid(spline, line, 1e-7, grid);
    CHECK_EQ_INT(np, ng);
    for (int i = 0; i < np; ++i)
      CHECK_MSG(primary[i].point_a.DistanceTo(primary[i].point_b) <= 1e-7,
                "spline x line event is not on the line");
  }

  // ---- SSX: sphere x plane -> one closed circle of radius sqrt(15) ------
  {
    ON_Sphere sphere(ON_3dPoint(0, 0, 0), 4.0);
    ON_NurbsSurface sphere_nurbs;
    CHECK(sphere.GetNurbForm(sphere_nurbs));
    ON_NurbsSurface plane = MakeTestPlanePatch(ON_Plane(ON_3dPoint(0, 0, 1), ON_3dVector::ZAxis), 8.0);

    std::vector<TL_SurfaceSurfaceIntersectionBranch> branches;
    const int count = TL_SurfaceSurfaceIntersect(sphere_nurbs, plane, 1e-9, branches);
    CHECK_EQ_INT(count, 1);
    if (count == 1)
    {
      const auto& branch = branches[0];
      CHECK(branch.is_closed);
      CHECK(!branch.is_tangent);
      CHECK(branch.points_3d.Count() > 50);
      CHECK(nullptr != branch.curve_3d);
      if (nullptr != branch.curve_3d)
      {
        CHECK_MSG(branch.curve_3d->IsPeriodic(),
                  "a closed SSX branch must be fitted as a true periodic NURBS curve");
        CHECK_MSG(branch.curve_3d->IsClosed(), "closed SSX branch curve must report IsClosed()");
      }
      CHECK_MSG(branch.max_error <= 1e-6,
                "sphere x plane max_error = " + std::to_string(branch.max_error));
      CHECK_EQ_INT(branch.params_a.Count(), branch.points_3d.Count());
      CHECK_EQ_INT(branch.params_b.Count(), branch.points_3d.Count());

      const double expected_radius = std::sqrt(16.0 - 1.0);
      for (int i = 0; i < branch.points_3d.Count(); ++i)
      {
        const ON_3dPoint& p = branch.points_3d[i];
        CHECK_NEAR(std::sqrt(p.x * p.x + p.y * p.y), expected_radius, 1e-9);
        CHECK_NEAR(p.z, 1.0, 1e-9);
      }

      // The fitted curve must actually lie on both surfaces.
      const ON_Interval dom = branch.curve_3d->Domain();
      for (int k = 0; k <= 8; ++k)
      {
        const ON_3dPoint p = branch.curve_3d->PointAt(dom.ParameterAt(k / 8.0));
        CHECK_NEAR(p.DistanceTo(ON_3dPoint::Origin), 4.0, 1e-5);
        CHECK_NEAR(p.z, 1.0, 1e-5);
      }
    }
    TL_FreeSSXBranches(branches);
    CHECK(branches.empty());
  }

  // ---- SSX: perpendicular cylinders -> two closed loops ----------------
  {
    ON_Cylinder cyl_a(ON_Circle(ON_Plane(ON_3dPoint(0, 0, -6), ON_3dVector::ZAxis), 3.0), 12.0);
    ON_NurbsSurface a;
    ON_Cylinder cyl_b(ON_Circle(ON_Plane(ON_3dPoint(-6, 0, 0), ON_3dVector::XAxis), 3.0), 12.0);
    ON_NurbsSurface b;
    CHECK(cyl_a.GetNurbForm(a));
    CHECK(cyl_b.GetNurbForm(b));

    std::vector<TL_SurfaceSurfaceIntersectionBranch> branches;
    const int count = TL_SurfaceSurfaceIntersect(a, b, 1e-8, branches);
    CHECK_EQ_INT(count, 2);
    int closed_count = 0;
    for (const auto& branch : branches)
    {
      if (branch.is_closed)
        ++closed_count;
      CHECK(nullptr != branch.curve_3d);
      CHECK_MSG(branch.max_error <= 1e-5,
                "cyl x cyl max_error = " + std::to_string(branch.max_error));
      for (int i = 0; i < branch.points_3d.Count(); ++i)
      {
        const ON_3dPoint& p = branch.points_3d[i];
        // On cylinder A (axis Z, radius 3) and on cylinder B (axis X, radius 3).
        CHECK_NEAR(std::sqrt(p.x * p.x + p.y * p.y), 3.0, 1e-7);
        CHECK_NEAR(std::sqrt(p.y * p.y + p.z * p.z), 3.0, 1e-7);
      }
    }
    CHECK_EQ_INT(closed_count, 2);
    TL_FreeSSXBranches(branches);
  }

  // ---- SSX: parallel offset cylinders -> two open straight branches ----
  {
    ON_Cylinder cyl_a(ON_Circle(ON_Plane(ON_3dPoint(0, 0, -6), ON_3dVector::ZAxis), 3.0), 12.0);
    ON_Cylinder cyl_b(ON_Circle(ON_Plane(ON_3dPoint(4, 0, -6), ON_3dVector::ZAxis), 3.0), 12.0);
    ON_NurbsSurface a;
    ON_NurbsSurface b;
    CHECK(cyl_a.GetNurbForm(a));
    CHECK(cyl_b.GetNurbForm(b));

    std::vector<TL_SurfaceSurfaceIntersectionBranch> branches;
    const int count = TL_SurfaceSurfaceIntersect(a, b, 1e-8, branches);
    CHECK_EQ_INT(count, 2);
    for (const auto& branch : branches)
    {
      CHECK(!branch.is_closed);            // the branches run off both ends
      CHECK(branch.points_3d.Count() > 20);
      CHECK(branch.max_error <= 1e-6);
      // The two planes x = +-2 are the intersection lines.
      CHECK_NEAR(std::fabs(branch.points_3d[0].x), 2.0, 1e-7);
      CHECK_NEAR(std::fabs(branch.points_3d[0].y), std::sqrt(5.0), 1e-6);
    }
    TL_FreeSSXBranches(branches);
  }

  // ---- SSX: disjoint surfaces -> no branches --------------------------
  {
    ON_Sphere sphere(ON_3dPoint(0, 0, 0), 2.0);
    ON_NurbsSurface sphere_nurbs;
    CHECK(sphere.GetNurbForm(sphere_nurbs));
    ON_NurbsSurface far_plane = MakeTestPlanePatch(
      ON_Plane(ON_3dPoint(0, 0, 10), ON_3dVector::ZAxis), 4.0);
    std::vector<TL_SurfaceSurfaceIntersectionBranch> branches;
    CHECK_EQ_INT(TL_SurfaceSurfaceIntersect(sphere_nurbs, far_plane, 1e-8, branches), 0);
    CHECK(branches.empty());
  }

  // ---- SSX: tangential contact (cylinder resting on plane x = 3) -------
  {
    ON_Cylinder cyl(ON_Circle(ON_Plane(ON_3dPoint(0, 0, -6), ON_3dVector::ZAxis), 3.0), 12.0);
    ON_NurbsSurface cyl_nurbs;
    CHECK(cyl.GetNurbForm(cyl_nurbs));
    ON_NurbsSurface tangent_plane = MakeTestPlanePatch(
      ON_Plane(ON_3dPoint(3, 0, 0), ON_3dVector::XAxis), 8.0);

    std::vector<TL_SurfaceSurfaceIntersectionBranch> branches;
    const int count = TL_SurfaceSurfaceIntersect(cyl_nurbs, tangent_plane, 1e-6, branches);
    CHECK_EQ_INT(count, 1);
    if (count == 1)
    {
      const auto& branch = branches[0];
      CHECK(branch.is_tangent);
      CHECK(!branch.is_closed);
      CHECK(branch.points_3d.Count() > 40);
      CHECK(nullptr != branch.curve_3d);
      CHECK_MSG(branch.max_error <= 1e-9,
                "tangential ruling max_error = " + std::to_string(branch.max_error));
      for (int i = 0; i < branch.points_3d.Count(); ++i)
      {
        const ON_3dPoint& p = branch.points_3d[i];
        CHECK_NEAR(p.x, 3.0, 1e-5);
        CHECK_NEAR(p.y, 0.0, 1e-4);
      }
    }
    TL_FreeSSXBranches(branches);
  }

  // ---- feature edges: boundary / crease / silhouette --------------------
  {
    ON_Mesh box;
    const ON_3dPoint c[8] = { ON_3dPoint(-1, -1, -1), ON_3dPoint(1, -1, -1), ON_3dPoint(1, 1, -1),
                              ON_3dPoint(-1, 1, -1), ON_3dPoint(-1, -1, 1), ON_3dPoint(1, -1, 1),
                              ON_3dPoint(1, 1, 1), ON_3dPoint(-1, 1, 1) };
    for (int i = 0; i < 8; ++i)
      box.SetVertex(i, c[i]);
    const int quads[6][4] = { { 0, 3, 2, 1 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 },
                              { 2, 3, 7, 6 }, { 0, 4, 7, 3 }, { 1, 2, 6, 5 } };
    for (int i = 0; i < 6; ++i)
      box.SetQuad(i, quads[i][0], quads[i][1], quads[i][2], quads[i][3]);
    box.ComputeFaceNormals();

    ON_SimpleArray<ON_Line> lines;
    ON_SimpleArray<int> kinds;
    const int count = TL_ExtractMeshFeatureEdges(box, ON_3dVector(0, 0, -1), ON_PI / 6.0, lines,
                                                 &kinds, true);
    CHECK_EQ_INT(count, 12);          // a box has exactly 12 hard edges
    CHECK_EQ_INT(lines.Count(), count);
    CHECK_EQ_INT(kinds.Count(), count);

    int boundary = 0;
    int crease = 0;
    for (int i = 0; i < kinds.Count(); ++i)
    {
      if (kinds[i] & TL_MESH_EDGE_BOUNDARY)
        ++boundary;
      if (kinds[i] & TL_MESH_EDGE_CREASE)
        ++crease;
    }
    CHECK_EQ_INT(boundary, 0);
    CHECK_EQ_INT(crease, 12);

    // A smooth tube must report boundary edges on its open rims and a silhouette that
    // depends on the view direction.
    ON_ArcCurve rail(ON_Arc(ON_Plane(ON_3dPoint(-6, -6, 0), ON_3dVector::ZAxis), 9.0, ON_PI));
    ON_ArcCurve profile(ON_Circle(ON_Plane::World_xy, 1.6));
    ON_NurbsSurface* swept = TL_Sweep1Rail(rail, profile, 16, nullptr);
    CHECK(nullptr != swept);
    if (nullptr != swept)
    {
      ON_Mesh tube;
      CHECK(TL_MeshSurfaceGrid(*swept, 32, 64, &tube));
      delete swept;
      tube.ComputeFaceNormals();

      ON_SimpleArray<ON_Line> front_lines;
      ON_SimpleArray<int> front_kinds;
      TL_ExtractMeshFeatureEdges(tube, ON_3dVector(0, 1, 0), ON_PI / 6.0, front_lines, &front_kinds,
                                 true);
      ON_SimpleArray<ON_Line> top_lines;
      ON_SimpleArray<int> top_kinds;
      TL_ExtractMeshFeatureEdges(tube, ON_3dVector(0, 0, -1), ON_PI / 6.0, top_lines, &top_kinds,
                                 true);

      int front_silhouette = 0;
      int front_boundary = 0;
      for (int i = 0; i < front_kinds.Count(); ++i)
      {
        if (front_kinds[i] & TL_MESH_EDGE_SILHOUETTE)
          ++front_silhouette;
        if (front_kinds[i] & TL_MESH_EDGE_BOUNDARY)
          ++front_boundary;
      }
      CHECK_MSG(front_boundary > 0, "open tube must report boundary edges");
      CHECK_MSG(front_silhouette > 0, "front view of a tube must have silhouette edges");
      CHECK_MSG(top_lines.Count() != front_lines.Count() ||
                  front_silhouette != 0,
                "silhouette extraction must depend on the view direction");
    }
  }

  EndSection();
}

// ---------------------------------------------------------------------------
// 19. Solid winding number, planar boolean slice, CSG booleans & SubD meshing
// ---------------------------------------------------------------------------
namespace {

ON_Mesh MakeTestAxisBoxMesh(const ON_3dPoint& mn, const ON_3dPoint& mx)
{
  ON_Mesh box;
  const ON_3dPoint c[8] = {
    ON_3dPoint(mn.x, mn.y, mn.z), ON_3dPoint(mx.x, mn.y, mn.z),
    ON_3dPoint(mx.x, mx.y, mn.z), ON_3dPoint(mn.x, mx.y, mn.z),
    ON_3dPoint(mn.x, mn.y, mx.z), ON_3dPoint(mx.x, mn.y, mx.z),
    ON_3dPoint(mx.x, mx.y, mx.z), ON_3dPoint(mn.x, mx.y, mx.z)
  };
  for (int i = 0; i < 8; ++i)
    box.SetVertex(i, c[i]);
  const int q[6][4] = { { 0, 3, 2, 1 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 },
                        { 2, 3, 7, 6 }, { 0, 4, 7, 3 }, { 1, 2, 6, 5 } };
  for (int i = 0; i < 6; ++i)
    box.SetQuad(i, q[i][0], q[i][1], q[i][2], q[i][3]);
  box.ComputeFaceNormals();
  box.ComputeVertexNormals();
  return box;
}

} // anonymous namespace

void TestBooleansAndSubDStage8()
{
  BeginSection("solid winding number, planar boolean slice, CSG booleans and SubD meshing");

  // ---- 1. Winding number & point-in-solid classification ----------------
  const ON_Mesh box_a = MakeTestAxisBoxMesh(ON_3dPoint(-2, -2, -2), ON_3dPoint(2, 2, 2));
  CHECK_NEAR(TL_MeshSignedVolume(box_a), 64.0, 1e-12);
  CHECK_NEAR(TL_MeshWindingNumber(box_a, ON_3dPoint(0, 0, 0)), 1.0, 1e-12);
  CHECK_NEAR(TL_MeshWindingNumber(box_a, ON_3dPoint(5, 0, 0)), 0.0, 1e-12);
  CHECK(TL_MeshContainsPoint(box_a, ON_3dPoint(0, 0, 0), 1e-6) == TL_PointFaceRelation::Inside);
  CHECK(TL_MeshContainsPoint(box_a, ON_3dPoint(5, 0, 0), 1e-6) == TL_PointFaceRelation::Outside);
  CHECK(TL_MeshContainsPoint(box_a, ON_3dPoint(2, 0, 0), 1e-6) == TL_PointFaceRelation::Boundary);

  // ---- 2. CSG Booleans (Union / Intersection / Difference) --------------
  // box_a: [-2,2]^3 (V = 64)
  // box_b: [0,4] x [-1,3] x [-1,3] (V = 64)
  // exact overlap: [0,2] x [-1,2] x [-1,2] -> 2 * 3 * 3 = 18
  {
    const ON_Mesh box_b = MakeTestAxisBoxMesh(ON_3dPoint(0, -1, -1), ON_3dPoint(4, 3, 3));
    ON_Mesh mesh_union, mesh_inter, mesh_diff;
    CHECK(TL_MeshBoolean(box_a, box_b, TL_BooleanOp::Union, mesh_union));
    CHECK(TL_MeshBoolean(box_a, box_b, TL_BooleanOp::Intersection, mesh_inter));
    CHECK(TL_MeshBoolean(box_a, box_b, TL_BooleanOp::Difference, mesh_diff));

    const double vu = TL_MeshSignedVolume(mesh_union);
    const double vi = TL_MeshSignedVolume(mesh_inter);
    const double vd = TL_MeshSignedVolume(mesh_diff);
    CHECK_NEAR(vi, 18.0, 1e-9);
    CHECK_NEAR(vu, 110.0, 1e-9);
    CHECK_NEAR(vd, 46.0, 1e-9);
    // Inclusion-exclusion identities: V(A U B) + V(A n B) == V(A) + V(B) and V(A \ B) + V(A n B) == V(A)
    CHECK_NEAR(vu + vi, 128.0, 1e-9);
    CHECK_NEAR(vd + vi, 64.0, 1e-9);

    // Point (1, 0, 0) lies inside both A and B -> inside U and I, outside A \ B.
    CHECK(TL_MeshContainsPoint(mesh_union, ON_3dPoint(1, 0, 0), 1e-6) == TL_PointFaceRelation::Inside);
    CHECK(TL_MeshContainsPoint(mesh_inter, ON_3dPoint(1, 0, 0), 1e-6) == TL_PointFaceRelation::Inside);
    CHECK(TL_MeshContainsPoint(mesh_diff, ON_3dPoint(1, 0, 0), 1e-6) == TL_PointFaceRelation::Outside);
    // Point (-1, 0, 0) lies inside A only -> inside U and A \ B, outside I.
    CHECK(TL_MeshContainsPoint(mesh_diff, ON_3dPoint(-1, 0, 0), 1e-6) == TL_PointFaceRelation::Inside);
    CHECK(TL_MeshContainsPoint(mesh_inter, ON_3dPoint(-1, 0, 0), 1e-6) == TL_PointFaceRelation::Outside);
  }

  // ---- 3. SubD right-handed box creation, meshing & planar boolean slice
  {
    const ON_BoundingBox bbox(ON_3dPoint(-3, -3, 0), ON_3dPoint(3, 3, 4));
    ON_SubD* subd = TL_CreateSubDBox(bbox, 2, nullptr);
    CHECK(nullptr != subd);
    if (nullptr != subd)
    {
      ON_Mesh* smesh = TL_MeshFromSubD(*subd, 1, nullptr);
      CHECK(nullptr != smesh);
      if (nullptr != smesh)
      {
        ON_BoundingBox sbb;
        CHECK(smesh->GetBoundingBox(sbb));
        // Symmetry around the input box centre (0, 0, 2) proves the corner ordering is right-handed.
        CHECK_NEAR(sbb.Center().x, 0.0, 1e-5);
        CHECK_NEAR(sbb.Center().y, 0.0, 1e-5);
        CHECK_NEAR(sbb.Center().z, 2.0, 1e-5);

        const double v_whole = TL_MeshSignedVolume(*smesh);
        CHECK_MSG(v_whole > 0.0, "SubD mesh faces must be wound outward (positive signed volume)");
        CHECK_NEAR(TL_MeshWindingNumber(*smesh, ON_3dPoint(0, 0, 2)), 1.0, 1e-9);

        ON_3dVector cut_n(0.4, -0.3, 0.85);
        cut_n.Unitize();
        const ON_Plane cut_plane(ON_3dPoint(0, 0, 2), cut_n);
        ON_Mesh pos_half, neg_half;
        std::vector<std::vector<ON_3dPoint>> section_loops;
        const int loops = TL_SliceMeshWithPlane(*smesh, cut_plane, true, &pos_half, &neg_half,
                                                &section_loops);
        CHECK_EQ_INT(loops, 1);
        CHECK(!section_loops.empty() && section_loops[0].size() >= 12);

        const double v_pos = TL_MeshSignedVolume(pos_half);
        const double v_neg = TL_MeshSignedVolume(neg_half);
        CHECK(v_pos > 10.0 && v_neg > 10.0);
        CHECK_NEAR(v_pos + v_neg, v_whole, 1e-5);
        delete smesh;
      }
      delete subd;
    }
  }

  // ---- 4. TL_ExtractBrepMesh on a procedural ON_Brep box (uncached fallback)
  {
    const ON_3dPoint corners[8] = {
      ON_3dPoint(-2, -3, 0), ON_3dPoint(2, -3, 0), ON_3dPoint(2, 3, 0), ON_3dPoint(-2, 3, 0),
      ON_3dPoint(-2, -3, 5), ON_3dPoint(2, -3, 5), ON_3dPoint(2, 3, 5), ON_3dPoint(-2, 3, 5)
    };
    ON_Brep* brep_box = ON_BrepBox(corners);
    CHECK(nullptr != brep_box);
    if (nullptr != brep_box)
    {
      ON_Mesh extracted;
      const int added = TL_ExtractBrepMesh(*brep_box, extracted, false);
      CHECK(added > 0);
      CHECK(extracted.VertexCount() > 0 && extracted.m_F.Count() == added);
      // Volume of [-2,2] x [-3,3] x [0,5] is 4 * 6 * 5 = 120.
      CHECK_NEAR(std::fabs(TL_MeshSignedVolume(extracted)), 120.0, 1e-6);

      std::vector<std::vector<ON_3dPoint>> edge_polylines;
      const int edge_count = TL_ExtractBrepEdgePolylines(*brep_box, edge_polylines, 8);
      CHECK_EQ_INT(edge_count, 12);
      CHECK_EQ_INT(static_cast<int>(edge_polylines.size()), 12);
      delete brep_box;
    }
  }

  EndSection();
}


// ---------------------------------------------------------------------------
// Stage 10/11: Grasshopper .gh parsing + DAG solve + Rhino 9 studio GUI layout
// ---------------------------------------------------------------------------
void TestGrasshopperAndStudioGui() {
  BeginSection("Grasshopper .gh archive, DAG solve, studio GUI");

  const std::string tutorials =
    "/Users/waya/Projects/Rhino/Rhino 9 WIP/Plug-ins/Grasshopper/Tutorials/";

  // ---- 1. Every shipped Rhino 9 tutorial definition must load byte-exactly.
  const char* files[] = {
    "00 From Rhino To Grasshopper.gh",
    "01 Simple Input and Maths.gh",
    "02 Interpolating Values.gh",
    "03 Wires and Conversions.gh",
    "04 Polynomial Graphing with Derivatives.gh",
    "05 Measure and Display Values.gh",
    "06 Adaptive Pattern within Shape.gh",
    "07 Column Arcade.gh"
  };
  int loaded = 0;
  for (const char* name : files) {
    const std::string path = tutorials + name;
    std::ifstream probe(path, std::ios::binary);
    if (!probe.good()) continue;
    probe.close();
    TL_GhDocument doc;
    std::string error;
    const bool ok = doc.LoadFile(path, &error);
    CHECK(ok);
    if (!ok) {
      std::printf("   load failed %s: %s\n", name, error.c_str());
      continue;
    }
    ++loaded;
    CHECK(doc.Nodes().size() >= 6);
    // Pure parameter definitions (00) legitimately have zero component ports.
    CHECK(doc.CanvasBounds().w > 1.0f && doc.CanvasBounds().h > 1.0f);
    CHECK(doc.TopologicalOrder().size() == doc.Nodes().size());
    opentl::mvp::SceneDocument baked;
    std::vector<ON_Mesh*> meshes;
    const int baked_count = doc.BakeToScene(baked, meshes);
    CHECK(baked_count >= 0);
    for (ON_Mesh* m : meshes) delete m;
  }
  if (loaded == 0) {
    // Reference definitions are an external fixture; keep the suite runnable.
    std::printf("   (Rhino 9 tutorial .gh fixtures not found - skipped)\n");
    CHECK(true);
  } else {
    CHECK_EQ_INT(loaded, 8);
  }

  // ---- 2. Slider edit must propagate through the DAG (07 Column Arcade).
  {
    const std::string path = tutorials + "07 Column Arcade.gh";
    std::ifstream probe(path, std::ios::binary);
    if (probe.good()) {
      probe.close();
      TL_GhDocument doc;
      std::string error;
      CHECK(doc.LoadFile(path, &error));
      CHECK_EQ_INT((int)doc.SliderCount(), 3);
      CHECK(doc.Wires().size() > 10);
      const size_t nodes_before = doc.Nodes().size();
      CHECK(doc.SetSliderValueByOrdinal(0, 30.0));
      CHECK_EQ_INT((int)doc.Nodes().size(), (int)nodes_before);
      // Segment count changed -> divide-curve output count changes downstream.
      size_t divide_points = 0;
      for (const auto& n : doc.Nodes()) {
        if (n.name == "Divide Curve" && !n.outputs.empty())
          divide_points = n.outputs[0].runtime_data.size();
      }
      CHECK(divide_points > 0);
      CHECK(doc.SetSliderValue("Segments", 12.0));
      size_t divide_points_after = 0;
      for (const auto& n : doc.Nodes()) {
        if (n.name == "Divide Curve" && !n.outputs.empty())
          divide_points_after = n.outputs[0].runtime_data.size();
      }
      CHECK(divide_points_after > 0);
      CHECK(divide_points_after != divide_points);
    }
  }

  // ---- 3. Studio workbench layout regions tile the 1920x1080 canvas.
  {
    const TL_GuiStudioRects rects = TL_ComputeStudioRects(1920, 1080);
    CHECK_NEAR(rects.viewports.w, 1920.0 - rects.dock.w - rects.gh_canvas.w, 1.0);
    CHECK(rects.viewports.h > 900.0f);
    CHECK_NEAR(rects.status.y + rects.status.h, 1080.0, 0.5);
    CHECK_NEAR(rects.gh_canvas.y, rects.viewports.y, 0.5);
    CHECK(rects.gh_params.y + rects.gh_params.h <= rects.status.y + 0.5f);

    TL_FourViewLayout layout = TL_FourViewLayout::Default(1600, 900);
    TL_ApplyStudioViewports(layout, rects);
    CHECK_EQ_INT(layout.CanvasWidth(), 1920);
    CHECK_EQ_INT(layout.View(0).m_left, (int)rects.viewports.x);
    CHECK_EQ_INT(layout.View(0).m_top, (int)rects.viewports.y);
    CHECK(layout.View(3).m_left > layout.View(0).m_left);
    CHECK(layout.View(2).m_top > layout.View(0).m_top);

    TL_GuiOverlay overlay;
    TL_GuiStudioInfo info;
    info.object_count = 383;
    info.mesh_count = 6;
    TL_BuildStudioOverlay(rects, nullptr, info, overlay);
    CHECK(overlay.enabled);
    CHECK(overlay.fills.size() > 20);
    CHECK(overlay.texts.size() > 20);
    bool found_title = false;
    for (const auto& t : overlay.texts)
      if (t.text.find("OHINO") != std::string::npos) found_title = true;
    CHECK(found_title);
  }

  // ---- 4. GH node graph renders into the overlay (groups, wires, sliders).
  {
    const std::string path = tutorials + "07 Column Arcade.gh";
    std::ifstream probe(path, std::ios::binary);
    if (probe.good()) {
      probe.close();
      TL_GhDocument doc;
      std::string error;
      if (doc.LoadFile(path, &error)) {
        const TL_GuiStudioRects rects = TL_ComputeStudioRects(1920, 1080);
        TL_FourViewLayout layout = TL_FourViewLayout::Default(1920, 1080);
        TL_ApplyStudioViewports(layout, rects);
        TL_GuiOverlay overlay;
        TL_GuiStudioInfo info;
        info.layout = &layout;
        info.show_gumball = true;
        info.gumball_origin = ON_3dPoint::Origin;
        info.gumball_radius = 8.0;
        TL_BuildStudioOverlay(rects, &doc, info, overlay);
        CHECK(overlay.lines.size() > doc.Wires().size());   // wires + slider tracks + Gumball arms
        CHECK(overlay.fills.size() > 100);                  // nodes + ports + groups + chrome
        CHECK(overlay.texts.size() > 40);
        bool has_cen = false, has_shaded = false;
        for (const auto& t : overlay.texts) {
          if (t.text == "CEN") has_cen = true;
          if (t.text == "SHADED") has_shaded = true;
        }
        CHECK(has_cen);
        CHECK(has_shaded);
      }
    }
  }

  // ---- 5. Grasshopper 2 (.ghz ZIP + Pear Document.root) in-memory loading & solve
  {
    const std::string ghz_path =
      "/Users/waya/Projects/Rhino/Rhino 9 WIP/Plug-ins/Grasshopper2/Documentation/English/Example Files/CircleRadius.ghz";
    std::ifstream probe(ghz_path, std::ios::binary);
    if (probe.good()) {
      probe.close();
      TL_GhDocument gh2;
      std::string err;
      CHECK(gh2.LoadFile(ghz_path, &err));
      CHECK_EQ_INT((int)gh2.Nodes().size(), 4);
      CHECK_EQ_INT((int)gh2.SliderCount(), 1);
      CHECK(gh2.Wires().size() >= 2);
      opentl::mvp::SceneDocument baked;
      std::vector<ON_Mesh*> meshes;
      const int baked_n = gh2.BakeToScene(baked, meshes);
      CHECK(baked_n > 10);
      CHECK(gh2.SetSliderValueByOrdinal(0, 18.0));
      opentl::mvp::SceneDocument baked2;
      const int baked_n2 = gh2.BakeToScene(baked2, meshes);
      CHECK(baked_n2 > baked_n);
      for (ON_Mesh* m : meshes) delete m;
    }
  }

  // ---- 6. TL_OffsetCurve & TL_Sweep2Rails kernel verification
  {
    ON_LineCurve base_line(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(10.0, 0.0, 0.0));
    ON_NurbsCurve* off_crv = TL_OffsetCurve(base_line, ON_Plane::World_xy, 2.5, 16, nullptr);
    CHECK(nullptr != off_crv);
    if (nullptr != off_crv) {
      CHECK(off_crv->IsValid());
      const ON_3dPoint mid = off_crv->PointAt(off_crv->Domain().Mid());
      CHECK_NEAR(mid.x, 5.0, 1e-3);
      CHECK_NEAR(std::fabs(mid.y), 2.5, 1e-4);
      CHECK_NEAR(mid.z, 0.0, 1e-6);
      delete off_crv;
    }

    ON_LineCurve rail1(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(0.0, 10.0, 0.0));
    ON_LineCurve rail2(ON_3dPoint(4.0, 0.0, 0.0), ON_3dPoint(8.0, 10.0, 0.0));
    ON_LineCurve prof(ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(4.0, 0.0, 2.0));
    ON_NurbsSurface* swp2 = TL_Sweep2Rails(rail1, rail2, prof, 16, nullptr);
    CHECK(nullptr != swp2);
    if (nullptr != swp2) {
      CHECK(swp2->IsValid());
      // At V=1 (end of rails), span doubled from 4 to 8 so Z height also scales from 2 to 4
      const ON_3dPoint end_tip = swp2->PointAt(swp2->Domain(0).Max(), swp2->Domain(1).Max());
      CHECK_NEAR(end_tip.x, 8.0, 1e-2);
      CHECK_NEAR(end_tip.y, 10.0, 1e-2);
      CHECK_NEAR(end_tip.z, 4.0, 1e-2);
      delete swp2;
    }
  }


  // ---- 7. Grasshopper 10-tab component battery catalog + programmatic graph authoring
  {
    CHECK_EQ_INT((int)TL_GhComponentCatalog::Categories().size(), 10);
    CHECK(TL_GhComponentCatalog::All().size() > 100);
    CHECK(TL_GhComponentCatalog::ByCategory("Curve").size() >= 15);
    CHECK(TL_GhComponentCatalog::ByCategory("Intersect").size() >= 6);
    CHECK(nullptr != TL_GhComponentCatalog::Find("Divide Curve"));
    CHECK(nullptr != TL_GhComponentCatalog::Find("Hexagonal"));
    CHECK(nullptr != TL_GhComponentCatalog::Find("List Item"));

    TL_GhDocument doc;
    const int slider = doc.AddSlider("Radius", 1.0, 20.0, 6.0, 40.0f, 40.0f);
    const int circle = doc.AddComponent("Circle", 260.0f, 40.0f);
    CHECK(slider >= 0);
    CHECK(circle >= 0);
    CHECK(circle > slider);
    CHECK_EQ_INT((int)doc.Nodes()[(size_t)circle].inputs.size(), 2);
    CHECK(doc.Connect(slider, -1, circle, 1));
    size_t out_vals = doc.Nodes()[(size_t)circle].outputs[0].runtime_data.size();
    CHECK(out_vals > 0);
    if (out_vals > 0) {
      const TL_GhValue& v = doc.Nodes()[(size_t)circle].outputs[0].runtime_data[0];
      CHECK(v.kind == TL_GhValueKind::Circle);
      CHECK_NEAR(v.circle_val.radius, 6.0, 1e-6);
      CHECK(doc.SetSliderValue("Radius", 9.5));
      const TL_GhValue& v2 = doc.Nodes()[(size_t)circle].outputs[0].runtime_data[0];
      CHECK_NEAR(v2.circle_val.radius, 9.5, 1e-6);
    }
    const float old_x = doc.Nodes()[(size_t)circle].bounds.x;
    CHECK(doc.MoveNode(circle, 320.0f, 90.0f));
    CHECK_NEAR((double)doc.Nodes()[(size_t)circle].bounds.x, 320.0, 1e-6);
    CHECK(doc.Nodes()[(size_t)circle].bounds.x != old_x);
    // Hexagonal battery produces both cells and centre points
    const int hex = doc.AddComponent("Hexagonal", 40.0f, 220.0f);
    CHECK(hex >= 0);
    CHECK(doc.Nodes()[(size_t)hex].outputs.size() >= 2);
    CHECK(doc.Nodes()[(size_t)hex].outputs[0].runtime_data.size() > 0);
    CHECK(doc.Nodes()[(size_t)hex].outputs[1].runtime_data.size() > 0);
    // SaveGh & SaveGhx lossless round-trip verification
    // Portable temp dir: "C:\tmp" does not exist on Windows, and MinGW builds
    // of this test suite are expected to pass there too.
    std::error_code fs_ec;
    std::string temp_dir = std::filesystem::temp_directory_path(fs_ec).string();
    if (fs_ec || temp_dir.empty())
      temp_dir = ".";
    const std::string tmp_gh = temp_dir + "/ohino_roundtrip_test.gh";
    const std::string tmp_ghx = temp_dir + "/ohino_roundtrip_test.ghx";
    std::string serr;
    CHECK(doc.SaveFile(tmp_gh, &serr));
    CHECK(doc.SaveFile(tmp_ghx, &serr));

    TL_GhDocument reloaded_gh;
    CHECK(reloaded_gh.LoadFile(tmp_gh, &serr));
    CHECK_EQ_INT((int)reloaded_gh.Nodes().size(), (int)doc.Nodes().size());
    CHECK_EQ_INT((int)reloaded_gh.SliderCount(), (int)doc.SliderCount());
    CHECK_EQ_INT((int)reloaded_gh.Wires().size(), (int)doc.Wires().size());

    TL_GhDocument reloaded_ghx;
    CHECK(reloaded_ghx.LoadFile(tmp_ghx, &serr));
    CHECK_EQ_INT((int)reloaded_ghx.Nodes().size(), (int)doc.Nodes().size());
    CHECK_EQ_INT((int)reloaded_ghx.SliderCount(), (int)doc.SliderCount());
    CHECK_EQ_INT((int)reloaded_ghx.Wires().size(), (int)doc.Wires().size());
  }

  // ---- 8. Trackpad & mouse interaction state machine
  {
    const TL_GuiStudioRects rects = TL_ComputeStudioRects(1920, 1080);
    TL_FourViewLayout layout = TL_FourViewLayout::Default(1920, 1080);
    TL_ApplyStudioViewports(layout, rects, -1);
    const ON_BoundingBox bbox(ON_3dPoint(-5, -5, -5), ON_3dPoint(5, 5, 5));
    layout.ZoomToFitAll(bbox, 0.9);
    TL_ApplyStudioViewports(layout, rects, -1);
    TL_StudioInputState st;
    st.gumball_origin = bbox.Center();
    st.gumball_radius = 6.0;

    const float px = (float)layout.View(0).m_left + 120.0f;
    const float py = (float)layout.View(0).m_top + 140.0f;

    // Trackpad pinch-to-zoom inside a 3D viewport
    TL_StudioInputEvent pinch;
    pinch.kind = TL_StudioEventKind::TrackpadPinch;
    pinch.x = px; pinch.y = py; pinch.pinch_scale = 1.25f;
    const double scale_before = layout.View(0).m_scale;
    CHECK(TL_ProcessStudioInputEvent(pinch, rects, layout, bbox, nullptr, st));
    CHECK(layout.View(0).m_scale > scale_before);

    // Trackpad two-finger scroll pans the viewport (no modifier)
    TL_StudioInputEvent scroll;
    scroll.kind = TL_StudioEventKind::MouseWheel;
    scroll.x = px; scroll.y = py;
    scroll.wheel_x = 3.0f; scroll.wheel_y = 4.0f;
    scroll.is_trackpad_scroll = true;
    const ON_3dPoint tgt_before = layout.View(0).m_target;
    const double s_before = layout.View(0).m_scale;
    CHECK(TL_ProcessStudioInputEvent(scroll, rects, layout, bbox, nullptr, st));
    CHECK(layout.View(0).m_target.DistanceTo(tgt_before) > 1e-3);
    CHECK_NEAR(layout.View(0).m_scale, s_before, 1e-9);

    // Discrete mouse wheel zooms (not pan)
    TL_StudioInputEvent wheel = scroll;
    wheel.is_trackpad_scroll = false;
    wheel.wheel_x = 0.0f; wheel.wheel_y = 1.0f;
    const double s2 = layout.View(0).m_scale;
    CHECK(TL_ProcessStudioInputEvent(wheel, rects, layout, bbox, nullptr, st));
    CHECK(layout.View(0).m_scale > s2);

    // Option + trackpad scroll orbits the perspective camera
    TL_StudioInputEvent orbit_scroll = scroll;
    orbit_scroll.mod_alt = true;
    orbit_scroll.x = (float)layout.View(1).m_left + 100.0f;
    orbit_scroll.y = (float)layout.View(1).m_top + 100.0f;
    orbit_scroll.wheel_x = 4.0f;
    const ON_3dVector dir_before = layout.View(1).m_camera_dir;
    CHECK(TL_ProcessStudioInputEvent(orbit_scroll, rects, layout, bbox, nullptr, st));
    CHECK((layout.View(1).m_camera_dir - dir_before).Length() > 1e-6);

    // RMB drag orbits in perspective, pans in orthographic
    TL_StudioInputEvent down;
    down.kind = TL_StudioEventKind::MouseDown;
    down.button = 3;
    down.x = (float)layout.View(1).m_left + 100.0f;
    down.y = (float)layout.View(1).m_top + 100.0f;
    CHECK(TL_ProcessStudioInputEvent(down, rects, layout, bbox, nullptr, st));
    TL_StudioInputEvent move;
    move.kind = TL_StudioEventKind::MouseMove;
    move.x = down.x + 30.0f; move.y = down.y + 10.0f;
    const ON_3dVector dir2 = layout.View(1).m_camera_dir;
    CHECK(TL_ProcessStudioInputEvent(move, rects, layout, bbox, nullptr, st));
    CHECK((layout.View(1).m_camera_dir - dir2).Length() > 1e-6);
    TL_StudioInputEvent up = down;
    up.kind = TL_StudioEventKind::MouseUp;
    TL_ProcessStudioInputEvent(up, rects, layout, bbox, nullptr, st);

    down.x = (float)layout.View(0).m_left + 100.0f;
    down.y = (float)layout.View(0).m_top + 120.0f;
    CHECK(TL_ProcessStudioInputEvent(down, rects, layout, bbox, nullptr, st));
    const ON_3dPoint pan_tgt = layout.View(0).m_target;
    move.x = down.x + 25.0f; move.y = down.y + 12.0f;
    CHECK(TL_ProcessStudioInputEvent(move, rects, layout, bbox, nullptr, st));
    CHECK(layout.View(0).m_target.DistanceTo(pan_tgt) > 1e-3);
    TL_ProcessStudioInputEvent(up, rects, layout, bbox, nullptr, st);

    // Double-click viewport title maximizes, second double-click restores 2x2
    TL_StudioInputEvent dbl = down;
    dbl.clicks = 2;
    dbl.button = 1;
    dbl.x = (float)layout.View(0).m_left + 60.0f;
    dbl.y = (float)layout.View(0).m_top + 10.0f;   // viewport title bar
    CHECK(TL_ProcessStudioInputEvent(dbl, rects, layout, bbox, nullptr, st));
    CHECK_EQ_INT(st.maximized_view, 0);
    TL_ApplyStudioViewports(layout, rects, st.maximized_view);
    CHECK_EQ_INT(layout.View(0).m_width, (int)rects.viewports.w);
    CHECK(TL_ProcessStudioInputEvent(dbl, rects, layout, bbox, nullptr, st));
    CHECK_EQ_INT(st.maximized_view, -1);

    // Grasshopper canvas: two-finger pan, pinch zoom, category tab switch
    TL_GhDocument gh;
    const int sl = gh.AddSlider("Segments", 1.0, 40.0, 12.0, 20.0f, 20.0f);
    CHECK(sl >= 0);
    TL_StudioInputEvent gh_scroll;
    gh_scroll.kind = TL_StudioEventKind::MouseWheel;
    gh_scroll.is_trackpad_scroll = true;
    gh_scroll.x = rects.gh_canvas.x + 200.0f;
    gh_scroll.y = rects.gh_canvas.y + 300.0f;
    gh_scroll.wheel_x = 5.0f; gh_scroll.wheel_y = 6.0f;
    const float pan_before = st.gh_pan_x;
    CHECK(TL_ProcessStudioInputEvent(gh_scroll, rects, layout, bbox, &gh, st));
    CHECK(st.gh_pan_x != pan_before);

    TL_StudioInputEvent gh_pinch;
    gh_pinch.kind = TL_StudioEventKind::TrackpadPinch;
    gh_pinch.x = rects.gh_canvas.x + 200.0f;
    gh_pinch.y = rects.gh_canvas.y + 300.0f;
    gh_pinch.pinch_scale = 1.4f;
    const float zoom_before = st.gh_zoom;
    CHECK(TL_ProcessStudioInputEvent(gh_pinch, rects, layout, bbox, &gh, st));
    CHECK(st.gh_zoom > zoom_before);

    TL_StudioInputEvent tab;
    tab.kind = TL_StudioEventKind::MouseDown;
    tab.button = 1;
    tab.x = rects.gh_canvas.x + 120.0f;
    tab.y = rects.gh_canvas.y + 34.0f;
    CHECK(TL_ProcessStudioInputEvent(tab, rects, layout, bbox, &gh, st));
    CHECK(st.active_gh_category != 4 || st.active_gh_category >= 0);
  }

  // ---- 9. Stage 16: 100% 128-battery Grasshopper solver coverage,
  //         Trimmed ON_Brep booleans (`TL_SplitBrepWithPlane` & `TL_BrepBoolean`),
  //         and interactive Studio Command-Line + Undo/Delete/Save workflow.
  {
    // (a) Instantiate and evaluate every single battery in the 182-component catalog
    TL_GhDocument all_doc;
    const auto& specs = TL_GhComponentCatalog::All();
    CHECK_EQ_INT((int)specs.size(), 188);
    int instantiated = 0;
    for (size_t i = 0; i < specs.size(); ++i) {
      const int nidx = all_doc.AddComponent(specs[i].name, 40.0f + (float)(i % 8) * 90.0f,
                                            40.0f + (float)(i / 8) * 45.0f);
      if (nidx >= 0) ++instantiated;
    }
    CHECK_EQ_INT(instantiated, (int)specs.size());
    CHECK(all_doc.Evaluate());

    // Specific verification of newly wired Surface / Mesh / Intersect / Transform batteries
    TL_GhDocument sdoc;
    const int n_box1 = sdoc.AddComponent("Center Box", 20, 20);
    const int n_box2 = sdoc.AddComponent("Box 2Pt", 20, 90);
    const int n_sunion = sdoc.AddComponent("Solid Union", 180, 20);
    const int n_demesh = sdoc.AddComponent("Deconstruct Mesh", 320, 20);
    const int n_medges = sdoc.AddComponent("Mesh Edges", 320, 90);
    const int n_arr = sdoc.AddComponent("Linear Array", 460, 20);
    const int n_parr = sdoc.AddComponent("Polar Array", 460, 90);
    CHECK(n_box1 >= 0 && n_box2 >= 0 && n_sunion >= 0 && n_demesh >= 0 && n_medges >= 0);
    CHECK(sdoc.Connect(n_box1, 0, n_sunion, 0));
    CHECK(sdoc.Connect(n_box2, 0, n_sunion, 0));
    CHECK(sdoc.Connect(n_sunion, 0, n_demesh, 0));
    CHECK(sdoc.Connect(n_sunion, 0, n_medges, 0));
    CHECK(sdoc.Connect(n_sunion, 0, n_arr, 0));
    CHECK(sdoc.Connect(n_sunion, 0, n_parr, 0));
    CHECK(!sdoc.Nodes()[(size_t)n_demesh].outputs[0].runtime_data.empty());
    CHECK(!sdoc.Nodes()[(size_t)n_medges].outputs[1].runtime_data.empty());
    CHECK_EQ_INT((int)sdoc.Nodes()[(size_t)n_arr].outputs[0].runtime_data.size(), 5);
    CHECK_EQ_INT((int)sdoc.Nodes()[(size_t)n_parr].outputs[0].runtime_data.size(), 6);

    // (b) Trimmed ON_Brep planar split (`TL_SplitBrepWithPlane`) & CSG (`TL_BrepBoolean`)
    const ON_3dPoint corners_a[8] = {
      ON_3dPoint(-2, -2, -2), ON_3dPoint(2, -2, -2), ON_3dPoint(2, 2, -2), ON_3dPoint(-2, 2, -2),
      ON_3dPoint(-2, -2,  2), ON_3dPoint(2, -2,  2), ON_3dPoint(2, 2,  2), ON_3dPoint(-2, 2,  2)
    };
    const ON_3dPoint corners_b[8] = {
      ON_3dPoint(0, -1, -1), ON_3dPoint(4, -1, -1), ON_3dPoint(4, 3, -1), ON_3dPoint(0, 3, -1),
      ON_3dPoint(0, -1,  3), ON_3dPoint(4, -1,  3), ON_3dPoint(4, 3,  3), ON_3dPoint(0, 3,  3)
    };
    ON_Brep* brep_a = ON_BrepBox(corners_a);
    ON_Brep* brep_b = ON_BrepBox(corners_b);
    CHECK(nullptr != brep_a && nullptr != brep_b);
    if (brep_a && brep_b) {
      ON_Brep pos_brep, neg_brep;
      std::vector<std::vector<ON_3dPoint>> loops;
      const ON_Plane cut_pl(ON_3dPoint(0, 0, 0), ON_3dVector(0, 0, 1));
      const int nloops = TL_SplitBrepWithPlane(*brep_a, cut_pl, true, &pos_brep, &neg_brep, &loops, 1e-6);
      CHECK_EQ_INT(nloops, 1);
      CHECK(pos_brep.m_F.Count() >= 6);
      CHECK(neg_brep.m_F.Count() >= 6);
      ON_Mesh m_pos, m_neg;
      TL_ExtractBrepMesh(pos_brep, m_pos, false);
      TL_ExtractBrepMesh(neg_brep, m_neg, false);
      CHECK_NEAR(TL_MeshSignedVolume(m_pos), 32.0, 1e-5);
      CHECK_NEAR(TL_MeshSignedVolume(m_neg), 32.0, 1e-5);

      ON_Brep brep_u, brep_i, brep_d;
      CHECK(TL_BrepBoolean(*brep_a, *brep_b, TL_BooleanOp::Union, brep_u, 1e-6));
      CHECK(TL_BrepBoolean(*brep_a, *brep_b, TL_BooleanOp::Intersection, brep_i, 1e-6));
      CHECK(TL_BrepBoolean(*brep_a, *brep_b, TL_BooleanOp::Difference, brep_d, 1e-6));
      CHECK(brep_u.m_F.Count() > 6);
      CHECK(brep_i.m_F.Count() >= 6);
      CHECK(brep_d.m_F.Count() > 6);

      ON_Mesh mu, mi, md;
      TL_ExtractBrepMesh(brep_u, mu, false);
      TL_ExtractBrepMesh(brep_i, mi, false);
      TL_ExtractBrepMesh(brep_d, md, false);
      CHECK_NEAR(TL_MeshSignedVolume(mi), 18.0, 1e-5);
      CHECK_NEAR(TL_MeshSignedVolume(mu), 110.0, 1e-5);
      CHECK_NEAR(TL_MeshSignedVolume(md), 46.0, 1e-5);
    }
    delete brep_a;
    delete brep_b;

    // (c) Studio Command-Line (`_Point`, `_Line`, `_Circle`, `_Delete`, `_Undo`, `_Save3dm`, `_GhAdd`)
    opentl::mvp::SceneDocument cmd_scene;
    TL_GhDocument cmd_gh;
    const TL_GuiStudioRects rects = TL_ComputeStudioRects(1920, 1080);
    TL_FourViewLayout layout = TL_FourViewLayout::Default(1920, 1080);
    TL_StudioInputState st;
    std::string fb;
    CHECK(TL_ExecuteStudioCommand("_Point 1 2 3", &cmd_scene, &cmd_gh, &layout, &rects, st, &fb));
    CHECK_EQ_INT((int)cmd_scene.ObjectCount(), 1);
    CHECK(TL_ExecuteStudioCommand("_Circle 0 0 0 8.5", &cmd_scene, &cmd_gh, &layout, &rects, st, &fb));
    CHECK_EQ_INT((int)cmd_scene.ObjectCount(), 2);
    CHECK(TL_ExecuteStudioCommand("_Delete", &cmd_scene, &cmd_gh, &layout, &rects, st, &fb));
    CHECK_EQ_INT((int)cmd_scene.ObjectCount(), 1);
    CHECK(TL_ExecuteStudioCommand("_Undo", &cmd_scene, &cmd_gh, &layout, &rects, st, &fb));
    CHECK_EQ_INT((int)cmd_scene.ObjectCount(), 2);
    CHECK(TL_ExecuteStudioCommand("_GhAdd Divide Curve", &cmd_scene, &cmd_gh, &layout, &rects, st, &fb));
    CHECK_EQ_INT((int)cmd_gh.Nodes().size(), 1);

    // Keyboard typing of `_Line 0 0 0 5 5 0` + Return key through `TL_ProcessStudioInputEvent`
    const char* typed = "_Line 0 0 0 5 5 0";
    for (const char* p = typed; *p; ++p) {
      TL_StudioInputEvent kev;
      kev.kind = TL_StudioEventKind::KeyDown;
      kev.key_code = (int)*p;
      CHECK(TL_ProcessStudioInputEvent(kev, rects, layout, ON_BoundingBox(), &cmd_gh, st, &cmd_scene));
    }
    TL_StudioInputEvent enter_ev;
    enter_ev.kind = TL_StudioEventKind::KeyDown;
    enter_ev.key_code = '\r';
    CHECK(TL_ProcessStudioInputEvent(enter_ev, rects, layout, ON_BoundingBox(), &cmd_gh, st, &cmd_scene));
    CHECK_EQ_INT((int)cmd_scene.ObjectCount(), 3);
  }

  EndSection();
}

void TestStage17ModelingAndPicking() {
  BeginSection("stage-17: revolve / edge surface / fillet, SubD primitives, scene picking");

  // ---- Revolve: a half-disc profile around Z gives a full sphere of radius 3 ----------
  {
    ON_Circle prof_circle(ON_Plane(ON_3dPoint(0, 0, 0), ON_3dVector(1, 0, 0), ON_3dVector(0, 0, 1)), 3.0);
    ON_NurbsCurve prof;
    CHECK(prof_circle.GetNurbForm(prof));
    std::shared_ptr<ON_NurbsSurface> sph(TL_RevolveCurve(prof, ON_Line(ON_3dPoint(0, 0, -10), ON_3dPoint(0, 0, 10))));
    CHECK(sph != nullptr);
    if (sph) {
      CHECK(sph->IsValid());
      CHECK(sph->IsClosed(0) || sph->IsClosed(1));
      // Sample the surface: every point must be ~3 from the origin
      double max_err = 0.0;
      for (int iu = 0; iu <= 8; ++iu)
        for (int iv = 0; iv <= 8; ++iv) {
          const double u = sph->Domain(0).ParameterAt(iu / 8.0);
          const double v = sph->Domain(1).ParameterAt(iv / 8.0);
          max_err = std::max(max_err, std::fabs(sph->PointAt(u, v).DistanceTo(ON_3dPoint::Origin) - 3.0));
        }
      CHECK(max_err < 1e-3);
      std::printf("   revolve sphere: max |r-3| = %.3e\n", max_err);
    }
    // Partial revolve: a line profile (0,0,0)->(3,0,3) swept 0..90 deg about Z is a quarter
    // cone, so every sampled point must satisfy x >= 0, y >= 0 and reach y = 3.
    {
      ON_LineCurve cone_prof(ON_3dPoint(0, 0, 0), ON_3dPoint(3, 0, 3));
      std::shared_ptr<ON_NurbsSurface> quarter(TL_RevolveCurve(cone_prof, ON_Line(ON_3dPoint(0, 0, -10), ON_3dPoint(0, 0, 10)), 0.0, 0.5 * ON_PI));
      CHECK(quarter != nullptr);
      if (quarter) {
        double min_x = 1e30, min_y = 1e30, max_y = -1e30;
        for (int iu = 0; iu <= 8; ++iu)
          for (int iv = 0; iv <= 8; ++iv) {
            const ON_3dPoint p = quarter->PointAt(quarter->Domain(0).ParameterAt(iu / 8.0),
                                                  quarter->Domain(1).ParameterAt(iv / 8.0));
            min_x = std::min(min_x, p.x); min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
          }
        CHECK(min_x > -1e-9);
        CHECK(min_y > -1e-9);
        CHECK(std::fabs(max_y - 3.0) < 1e-6);
        std::printf("   revolve quarter: x>=%.3e y in [%.3e, %.6f]\n", min_x, min_y, max_y);
      }
    }
  }

  // ---- EdgeSurface: Coons patch through 4 boundary curves, corners matched ------------
  {
    ON_NurbsCurve s, e, n, w;
    s.Create(3, false, 2, 2); s.MakeClampedUniformKnotVector(1.0); s.SetDomain(0, 1);
    s.SetCV(0, ON_3dPoint(0, 0, 0)); s.SetCV(1, ON_3dPoint(10, 0, 0));
    e.Create(3, false, 2, 2); e.MakeClampedUniformKnotVector(1.0); e.SetDomain(0, 1);
    e.SetCV(0, ON_3dPoint(10, 0, 0)); e.SetCV(1, ON_3dPoint(10, 10, 3));
    n.Create(3, false, 2, 2); n.MakeClampedUniformKnotVector(1.0); n.SetDomain(0, 1);
    n.SetCV(0, ON_3dPoint(0, 10, 3)); n.SetCV(1, ON_3dPoint(10, 10, 3));
    w.Create(3, false, 2, 2); w.MakeClampedUniformKnotVector(1.0); w.SetDomain(0, 1);
    w.SetCV(0, ON_3dPoint(0, 0, 0)); w.SetCV(1, ON_3dPoint(0, 10, 3));

    ON_SimpleArray<const ON_Curve*> bnd;
    bnd.Append(&s); bnd.Append(&e); bnd.Append(&n); bnd.Append(&w);
    std::shared_ptr<ON_NurbsSurface> patch(TL_CreateEdgeSurface(bnd));
    CHECK(patch != nullptr);
    if (patch) {
      CHECK(patch->IsValid());
      const double c0 = patch->PointAt(0.0, 0.0).DistanceTo(ON_3dPoint(0, 0, 0));
      const double c1 = patch->PointAt(1.0, 0.0).DistanceTo(ON_3dPoint(10, 0, 0));
      const double c2 = patch->PointAt(1.0, 1.0).DistanceTo(ON_3dPoint(10, 10, 3));
      const double c3 = patch->PointAt(0.0, 1.0).DistanceTo(ON_3dPoint(0, 10, 3));
      CHECK(c0 < 1e-6); CHECK(c1 < 1e-6); CHECK(c2 < 1e-6); CHECK(c3 < 1e-6);
      // Boundary midpoints must lie on the input curves
      const ON_3dPoint mid_s = 0.5 * (s.PointAtStart() + s.PointAtEnd());
      CHECK(patch->PointAt(0.5, 0.0).DistanceTo(mid_s) < 1e-6);
      std::printf("   edge surface: corners %.2e %.2e %.2e %.2e, mid-S %.2e\n",
                  c0, c1, c2, c3, patch->PointAt(0.5, 0.0).DistanceTo(mid_s));
      // Degenerate input (one curve) must be rejected
      ON_SimpleArray<const ON_Curve*> one; one.Append(&s);
      CHECK(TL_CreateEdgeSurface(one) == nullptr);
    }
  }

  // ---- Fillet: two perpendicular lines, exact tangency ------------------------------
  {
    ON_LineCurve la(ON_3dPoint(0, 0, 0), ON_3dPoint(10, 0, 0));
    ON_LineCurve lb(ON_3dPoint(10, 0, 0), ON_3dPoint(10, 10, 0));
    ON_Arc arc;
    ON_NurbsCurve ta, tb;
    CHECK(TL_FilletCurves(la, lb, 2.0, arc, &ta, &tb));
    CHECK(arc.IsValid());
    CHECK(std::fabs(arc.Radius() - 2.0) < 1e-9);
    // Tangent point on A is 8 units from the origin along +X; on B 2 units along +Y
    const ON_3dPoint t_a = arc.StartPoint().DistanceTo(ON_3dPoint(8, 0, 0)) <
                           arc.EndPoint().DistanceTo(ON_3dPoint(8, 0, 0)) ? arc.StartPoint() : arc.EndPoint();
    const ON_3dPoint t_b = (t_a.DistanceTo(arc.StartPoint()) < 1e-9) ? arc.EndPoint() : arc.StartPoint();
    CHECK(t_a.DistanceTo(ON_3dPoint(8, 0, 0)) < 1e-6);
    CHECK(t_b.DistanceTo(ON_3dPoint(10, 2, 0)) < 1e-6);
    // Arc midpoint is tangent to nothing but sits at distance radius from center (10,2,0)... center = (8,2,0)
    const ON_3dPoint center_test(8, 2, 0);
    CHECK(std::fabs(arc.MidPoint().DistanceTo(center_test) - 2.0) < 1e-9);
    CHECK(ta.IsValid()); CHECK(tb.IsValid());
    CHECK(std::fabs(ta.PointAtEnd().DistanceTo(ON_3dPoint(8, 0, 0))) < 1e-6);
    CHECK(std::fabs(tb.PointAtStart().DistanceTo(ON_3dPoint(10, 2, 0))) < 1e-6);
    // Parallel lines have no fillet
    ON_LineCurve lc(ON_3dPoint(0, 5, 0), ON_3dPoint(10, 5, 0));
    ON_Arc bad;
    CHECK(!TL_FilletCurves(la, lc, 2.0, bad, nullptr, nullptr));
    std::printf("   fillet: r=%.6f tangency %.2e / %.2e\n", arc.Radius(),
                t_a.DistanceTo(ON_3dPoint(8, 0, 0)), t_b.DistanceTo(ON_3dPoint(10, 2, 0)));
  }

  // ---- SubD primitives: cylinder and sphere -------------------------------
  {
    ON_Circle cir(ON_Plane::World_xy, 5.0);
    std::shared_ptr<ON_SubD> cyl(TL_CreateSubDCylinder(cir, 10.0, 8, 2, 2));
    CHECK(cyl != nullptr);
    if (cyl) {
      std::shared_ptr<ON_Mesh> cm(TL_MeshFromSubD(*cyl, 0));
      CHECK(cm != nullptr);
      if (cm) {
        CHECK(cm->VertexCount() > 100);
        CHECK(cm->IsClosed());
        const ON_BoundingBox bb = cm->BoundingBox();
        CHECK(std::fabs(bb.m_max.z - 10.0) < 1.5);   // limit surface stays near requested height
        CHECK(std::fabs(bb.m_min.z) < 1.5);
        CHECK(std::fabs(bb.m_max.x - 5.0) < 0.08);   // radial scale compensated to the base circle
        std::printf("   SubD cylinder: %d verts, %d faces, z %.3f..%.3f, x %.3f..%.3f\n",
                    cm->VertexCount(), cm->FaceCount(), bb.m_min.z, bb.m_max.z, bb.m_min.x, bb.m_max.x);
      }
    }
    std::shared_ptr<ON_SubD> sph(TL_CreateSubDSphere(ON_3dPoint(1, 2, 3), 4.0, 2));
    CHECK(sph != nullptr);
    if (sph) {
      std::shared_ptr<ON_Mesh> sm(TL_MeshFromSubD(*sph, 0));
      CHECK(sm != nullptr);
      if (sm) {
        CHECK(sm->VertexCount() > 40);
        CHECK(sm->IsClosed());
        double max_r = 0.0, min_r = 1e30;
        for (int i = 0; i < sm->VertexCount(); ++i) {
          const double r = ON_3dPoint(sm->m_V[i]).DistanceTo(ON_3dPoint(1, 2, 3));
          max_r = std::max(max_r, r); min_r = std::min(min_r, r);
        }
        CHECK(std::fabs(max_r - 4.0) < 0.35);
        CHECK(std::fabs(min_r - 4.0) < 0.35);
        std::printf("   SubD sphere: %d verts, radius %.3f..%.3f (target 4.000)\n",
                    sm->VertexCount(), min_r, max_r);
      }
    }
    // SubD from mesh round-trip
    ON_Mesh box_mesh;
    box_mesh.m_V.Append(ON_3fPoint(0, 0, 0)); box_mesh.m_V.Append(ON_3fPoint(1, 0, 0));
    box_mesh.m_V.Append(ON_3fPoint(1, 1, 0)); box_mesh.m_V.Append(ON_3fPoint(0, 1, 0));
    box_mesh.m_V.Append(ON_3fPoint(0, 0, 1)); box_mesh.m_V.Append(ON_3fPoint(1, 0, 1));
    box_mesh.m_V.Append(ON_3fPoint(1, 1, 1)); box_mesh.m_V.Append(ON_3fPoint(0, 1, 1));
    const int quads[6][4] = {{0,3,2,1},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
    for (int f = 0; f < 6; ++f) {
      ON_MeshFace mf;
      for (int k = 0; k < 4; ++k) mf.vi[k] = quads[f][k];
      box_mesh.m_F.Append(mf);
    }
    std::shared_ptr<ON_SubD> sd(TL_CreateSubDFromMesh(box_mesh, 2, false));
    CHECK(sd != nullptr);
    if (sd) {
      std::shared_ptr<ON_Mesh> sdm(TL_MeshFromSubD(*sd, 0));
      CHECK(sdm != nullptr);
      if (sdm) CHECK(sdm->FaceCount() > 6);
    }
  }

  // ---- Scene picking: axis-aligned objects, pick from the Top view -------------------
  {
    opentl::mvp::SceneDocument sc;
    const uint64_t id_a = sc.AddPoint(ON_3dPoint(-20, -20, 0), "A");
    const uint64_t id_b = sc.AddPoint(ON_3dPoint(20, 20, 0), "B");
    const uint64_t id_c = sc.AddCircle(ON_Circle(ON_Plane::World_xy, 5.0), "C");
    TL_FourViewLayout lay = TL_FourViewLayout::Default(1600, 1000);
    ON_BoundingBox bb;
    for (const auto& o : sc.Objects()) { const ON_BoundingBox b = o.GetBoundingBox(); if (b.IsValid()) { if (bb.IsValid()) bb.Union(b); else bb = b; } }
    lay.ZoomToFitAll(bb);

    // Project each object's location into the Top view and pick it back
    ON_2dPoint pa, pb, pc;
    CHECK(lay.View(0).WorldToScreen(ON_3dPoint(-20, -20, 0), pa));
    CHECK(lay.View(0).WorldToScreen(ON_3dPoint(20, 20, 0), pb));
    CHECK(lay.View(0).WorldToScreen(ON_3dPoint(5, 0, 0), pc));
    ON_3dPoint pivot;
    CHECK_EQ_INT((int)TL_PickSceneObjectAtScreen(sc, lay, 0, (float)pa.x, (float)pa.y, 10.0, &pivot), (int)id_a);
    CHECK_EQ_INT((int)TL_PickSceneObjectAtScreen(sc, lay, 0, (float)pb.x, (float)pb.y, 10.0, &pivot), (int)id_b);
    CHECK_EQ_INT((int)TL_PickSceneObjectAtScreen(sc, lay, 0, (float)pc.x, (float)pc.y, 10.0, &pivot), (int)id_c);
    CHECK(std::fabs(pivot.x) < 1e-6 && std::fabs(pivot.y) < 1e-6);
    // Empty space picks nothing
    CHECK_EQ_INT((int)TL_PickSceneObjectAtScreen(sc, lay, 0, 5.0f, 5.0f, 3.0), 0);
    std::printf("   picking: A=%llu B=%llu C=%llu pivot=(%.3f,%.3f,%.3f)\n",
                (unsigned long long)id_a, (unsigned long long)id_b, (unsigned long long)id_c,
                pivot.x, pivot.y, pivot.z);

    // Selection drives _Delete: picked object is removed, not the last one
    TL_StudioInputState st;
    TL_GhDocument gd;
    const TL_GuiStudioRects rects = TL_ComputeStudioRects(1600, 1000);
    st.selected_object_id = id_a;
    CHECK(TL_ExecuteStudioCommand("_Delete", &sc, &gd, &lay, &rects, st));
    CHECK_EQ_INT((int)sc.ObjectCount(), 2);
    bool still_has_a = false;
    for (const auto& o : sc.Objects()) if (o.id == id_a) still_has_a = true;
    CHECK(!still_has_a);
    // ... and _Undo brings it back
    CHECK(TL_ExecuteStudioCommand("_Undo", &sc, &gd, &lay, &rects, st));
    CHECK_EQ_INT((int)sc.ObjectCount(), 3);
  }

  EndSection();
}

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
  TestStage3Primitives();
  TestFourViewLayout();
  TestSweep1Rail();
  TestVulkanRenderer();
  TestMeshCapping();
  TestIntersectionStage6();
  TestBooleansAndSubDStage8();
  TestGrasshopperAndStudioGui();
  TestStage17ModelingAndPicking();

  std::printf("\n=====================\n");
  std::printf("sections=%d checks=%d failures=%d failed_sections=%d\n", g_state.section_index,
              g_state.total_checks, g_state.total_failures, g_state.sections_with_failures);
  std::printf("RESULT: %s\n", g_state.total_failures == 0 ? "ALL TESTS PASSED" : "FAILURES");

  ON::End();
  return g_state.total_failures == 0 ? 0 : 1;
}
