/**
 * @file opentl_cli.cpp
 * @brief OpenTL MVP command line demo / inspector.
 *
 * Usage
 *   opentl_cli                         run the full deterministic MVP demo
 *   opentl_cli --demo [--out-dir DIR]  same, writing scene files into DIR
 *   opentl_cli --inspect-3dm FILE.3dm  load a real openNURBS archive, dump a summary
 *   opentl_cli --inspect-json FILE     load an .otscene.json file, dump a summary
 *
 * Everything printed by the demo is deterministic: fixed precision numbers,
 * fixed iteration order (layers and objects by index / stable id) and no
 * pointers, timings or file mtimes. The scene is saved as both the local JSON
 * scene format and a real .3dm archive and reloaded into fresh SceneDocument
 * instances; the runtime digests must be byte identical.
 */

#include "opentl/opentl.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace {

// ============================================================================
// Deterministic text helpers
// ============================================================================

std::string Fixed(double value, int decimals = 6) {
  if (value == 0.0) value = 0.0;  // normalize -0.0 so the digest is stable
  char text[32];
  std::snprintf(text, sizeof(text), "%.*f", decimals, value);
  return std::string(text);
}

std::string Coord(const opentl::mvp::Point3d& point) {
  return "(" + Fixed(point.x) + ", " + Fixed(point.y) + ", " + Fixed(point.z) + ")";
}

std::string Vec(const opentl::mvp::Vector3d& vector) {
  return "(" + Fixed(vector.x) + ", " + Fixed(vector.y) + ", " + Fixed(vector.z) + ")";
}

std::string Box(const opentl::mvp::BoundingBox& box) {
  if (!box.IsSet() || box.IsEmpty()) return "<empty>";
  return Coord(box.Min()) + " .. " + Coord(box.Max());
}

std::string UuidText(const ON_UUID& id) {
  char buffer[64];
  std::memset(buffer, 0, sizeof(buffer));
  if (nullptr == ON_UuidToString(id, buffer)) return std::string("<nil>");
  return std::string(buffer);
}

uint64_t Fnv1a64(const std::string& text) {
  uint64_t hash = 1469598103934665603ULL;
  for (size_t i = 0; i < text.size(); ++i) {
    hash ^= static_cast<unsigned char>(text[i]);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string HashHex(const std::string& text) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%016llx",
                static_cast<unsigned long long>(Fnv1a64(text)));
  return std::string(buffer);
}

// ============================================================================
// Deterministic scene digest (the value compared across serialization formats)
// ============================================================================

std::string KindName(opentl::mvp::ObjectKind kind) {
  return std::string(opentl::mvp::ObjectKindToString(kind));
}

std::string LayerNameOf(const opentl::mvp::SceneDocument& document,
                        const opentl::mvp::SceneObject& object) {
  const opentl::mvp::Layer* layer = document.GetLayer(object.layer_index);
  return (nullptr != layer) ? layer->name : std::string("<none>");
}

std::string GeometryText(const opentl::mvp::SceneObject& object) {
  using opentl::mvp::ObjectKind;
  std::string text;
  switch (object.kind) {
    case ObjectKind::Point:
      text = "point" + Coord(object.point_val);
      break;
    case ObjectKind::Line:
      text = "line" + Coord(object.line_val.from) + "->" + Coord(object.line_val.to) +
             " length=" + Fixed(object.line_val.Length());
      break;
    case ObjectKind::Polyline: {
      text = "polyline[" + std::to_string(object.polyline_val.PointCount()) + "]";
      for (int i = 0; i < object.polyline_val.PointCount(); ++i) {
        text += (i == 0) ? " " : "-";
        text += Coord(object.polyline_val[i]);
      }
      text += " length=" + Fixed(object.polyline_val.Length());
      break;
    }
    case ObjectKind::Circle:
      text = "circle center=" + Coord(object.circle_val.Center()) + " radius=" +
             Fixed(object.circle_val.Radius()) + " normal=" + Vec(object.circle_val.Normal());
      break;
    case ObjectKind::Arc:
      text = "arc center=" + Coord(object.arc_val.Center()) + " radius=" +
             Fixed(object.arc_val.Radius()) + " angle_deg=" + Fixed(object.arc_val.AngleDegrees()) +
             " start=" + Coord(object.arc_val.StartPoint()) +
             " end=" + Coord(object.arc_val.EndPoint());
      break;
    case ObjectKind::Plane:
      text = "plane origin=" + Coord(object.plane_val.origin) + " xaxis=" +
             Vec(object.plane_val.xaxis) + " yaxis=" + Vec(object.plane_val.yaxis) +
             " zaxis=" + Vec(object.plane_val.zaxis);
      break;
    case ObjectKind::NurbsCurve:
      text = "nurbs dim=" + std::to_string(object.nurbs_val.m_dim) +
             " is_rat=" + std::to_string(object.nurbs_val.m_is_rat) +
             " order=" + std::to_string(object.nurbs_val.m_order) +
             " cv_count=" + std::to_string(object.nurbs_val.m_cv_count);
      for (int i = 0; i < object.nurbs_val.m_cv_count; ++i) {
        ON_3dPoint cv = ON_3dPoint::Origin;
        if (!object.nurbs_val.GetCV(i, cv)) break;
        text += " cv" + std::to_string(i) + "=" + Coord(cv);
      }
      break;
  }
  return text;
}

std::string BuildSceneDigest(const opentl::mvp::SceneDocument& document) {
  std::string digest;
  const std::vector<opentl::mvp::Layer>& layers = document.Layers();
  digest += "format=opentl_scene_v1\n";
  digest += "layer_count=" + std::to_string(layers.size()) + "\n";
  for (size_t i = 0; i < layers.size(); ++i) {
    const opentl::mvp::Layer& layer = layers[i];
    digest += "layer[" + std::to_string(i) + "]=name=" + layer.name +
              " color=(" + std::to_string(layer.color_r) + "," +
              std::to_string(layer.color_g) + "," + std::to_string(layer.color_b) + ")" +
              " visible=" + std::to_string(layer.visible ? 1 : 0) +
              " uuid=" + UuidText(layer.id) + "\n";
  }
  const std::vector<opentl::mvp::SceneObject>& objects = document.Objects();
  digest += "object_count=" + std::to_string(objects.size()) + "\n";
  for (size_t i = 0; i < objects.size(); ++i) {
    const opentl::mvp::SceneObject& object = objects[i];
    digest += "object[" + std::to_string(i) + "]=id=" + std::to_string(object.id) +
              " kind=" + KindName(object.kind) +
              " name=" + object.name +
              " layer=" + LayerNameOf(document, object) +
              " uuid=" + UuidText(object.uuid) +
              " bbox=" + Box(object.GetBoundingBox()) +
              " geom=" + GeometryText(object) + "\n";
  }
  digest += "scene_bbox=" + Box(document.GetSceneBoundingBox()) + "\n";
  return digest;
}

void PrintDigest(const char* label, const opentl::mvp::SceneDocument& document) {
  const std::string digest = BuildSceneDigest(document);
  std::printf("  %s.digest_lines=%d\n", label,
              static_cast<int>(std::count(digest.begin(), digest.end(), '\n')));
  std::printf("  %s.digest_hash=%s\n", label, HashHex(digest).c_str());
  std::printf("  %s.objects=%d layers=%d\n", label,
              static_cast<int>(document.ObjectCount()),
              static_cast<int>(document.Layers().size()));
}

// ============================================================================
// Demo scene construction
// ============================================================================

struct DemoScene {
  opentl::mvp::SceneDocument document;
  uint64_t point_id = 0;
  uint64_t line_id = 0;
  uint64_t polyline_id = 0;
  uint64_t circle_id = 0;
  uint64_t arc_id = 0;
  uint64_t plane_id = 0;
  uint64_t nurbs_id = 0;
};

void BuildDemoScene(DemoScene& scene) {
  using namespace opentl::mvp;

  scene.document.Clear();
  // The default document already owns a "Default" layer; only create it when the
  // implementation does not (FindLayerByName guards against a duplicate layer).
  if (scene.document.FindLayerByName("Default") < 0) {
    scene.document.AddLayer("Default", 0, 0, 0, true);
  }
  scene.document.AddLayer("Construction", 108, 112, 134, true);   // Catppuccin Overlay0
  scene.document.AddLayer("Curves", 137, 180, 250, true);        // Catppuccin Blue
  scene.document.AddLayer("Reference", 243, 139, 168, true);     // Catppuccin Red

  scene.point_id = scene.document.AddPoint(Point3d(1.0, 2.0, 3.0), "Point_A", "Construction");

  scene.line_id = scene.document.AddLine(Line(Point3d(0.0, 0.0, 0.0), Point3d(10.0, 0.0, 0.0)),
                                        "Line_X", "Curves");

  std::vector<Point3d> zigzag;
  zigzag.push_back(Point3d(0.0, 0.0, 0.0));
  zigzag.push_back(Point3d(5.0, 5.0, 0.0));
  zigzag.push_back(Point3d(10.0, 0.0, 0.0));
  zigzag.push_back(Point3d(15.0, 5.0, 0.0));
  scene.polyline_id = scene.document.AddPolyline(zigzag, "Polyline_Zigzag", "Curves");

  scene.circle_id = scene.document.AddCircle(Circle(Point3d(0.0, 0.0, 0.0), 5.0),
                                            "Circle_R5", "Curves");

  scene.arc_id = scene.document.AddArc(Arc(Plane::World_xy, 3.0, ON_PI * 0.5),
                                      "Arc_Quarter", "Curves");

  scene.plane_id = scene.document.AddPlane(Plane(Point3d::Origin, Vector3d(0.0, 0.0, 1.0)),
                                          "Plane_XY", "Reference");

  std::vector<Point3d> control_points;
  control_points.push_back(Point3d(0.0, 0.0, 0.0));
  control_points.push_back(Point3d(3.0, 6.0, 0.0));
  control_points.push_back(Point3d(6.0, -6.0, 0.0));
  control_points.push_back(Point3d(9.0, 0.0, 0.0));
  control_points.push_back(Point3d(12.0, 6.0, 0.0));
  scene.nurbs_id = scene.document.AddNurbsCurveFromPoints(control_points, 3, "Nurbs_Wave", "Curves");
}

// ============================================================================
// Demo stages
// ============================================================================

void PrintSceneHeader(const opentl::mvp::SceneDocument& document) {
  std::printf("  layers=%d objects=%d scene_bbox=%s\n",
              static_cast<int>(document.Layers().size()),
              static_cast<int>(document.ObjectCount()),
              Box(document.GetSceneBoundingBox()).c_str());
  for (size_t i = 0; i < document.Layers().size(); ++i) {
    const opentl::mvp::Layer& layer = document.Layers()[i];
    std::printf("  layer[%d] name=%-12s color=(%3d,%3d,%3d) visible=%d\n",
                static_cast<int>(i), layer.name.c_str(), layer.color_r, layer.color_g,
                layer.color_b, layer.visible ? 1 : 0);
  }
  for (size_t i = 0; i < document.Objects().size(); ++i) {
    const opentl::mvp::SceneObject& object = document.Objects()[i];
    std::printf("  object[%d] id=%llu kind=%-11s layer=%-12s name=%-18s bbox=%s\n",
                static_cast<int>(i), static_cast<unsigned long long>(object.id),
                KindName(object.kind).c_str(), LayerNameOf(document, object).c_str(),
                object.name.c_str(), Box(object.GetBoundingBox()).c_str());
  }
}

void StageQueries(DemoScene& scene) {
  using namespace opentl::mvp;
  SceneDocument& document = scene.document;

  const SceneObject* by_id = document.FindById(scene.circle_id);
  std::printf("  query.by_id id=%llu found=%d kind=%s name=%s\n",
              static_cast<unsigned long long>(scene.circle_id), nullptr != by_id ? 1 : 0,
              nullptr != by_id ? KindName(by_id->kind).c_str() : "-",
              nullptr != by_id ? by_id->name.c_str() : "-");

  const SceneObject* line = document.FindById(scene.line_id);
  const SceneObject* by_uuid =
      (nullptr != line) ? document.FindByUuid(line->uuid) : nullptr;
  std::printf("  query.by_uuid uuid=%s found=%d name=%s\n",
              nullptr != line ? UuidText(line->uuid).c_str() : "-",
              nullptr != by_uuid ? 1 : 0,
              nullptr != by_uuid ? by_uuid->name.c_str() : "-");

  const std::vector<const SceneObject*> by_name = document.FindByName("Circle_R5");
  std::printf("  query.by_name name=Circle_R5 count=%d\n", static_cast<int>(by_name.size()));

  const std::vector<const SceneObject*> curves = document.FindByLayer("Curves");
  std::printf("  query.by_layer layer=Curves count=%d\n", static_cast<int>(curves.size()));

  const std::vector<const SceneObject*> nurbs_objects = document.FindByKind(ObjectKind::NurbsCurve);
  std::printf("  query.by_kind kind=NurbsCurve count=%d\n",
              static_cast<int>(nurbs_objects.size()));

  std::printf("  query.find_layer Construction=%d Missing=%d\n",
              document.FindLayerByName("Construction"), document.FindLayerByName("Missing"));

  // Delete verification with a temporary object.
  const size_t before = document.ObjectCount();
  const uint64_t temp_id = document.AddPoint(Point3d(99.0, 99.0, 99.0), "Temp_DeleteMe", "Default");
  const size_t during = document.ObjectCount();
  const bool deleted = document.DeleteObject(temp_id);
  const size_t after = document.ObjectCount();
  const bool gone = (nullptr == document.FindById(temp_id));
  std::printf("  delete.temp id=%llu count %d->%d->%d deleted=%d gone=%d stable=%d\n",
              static_cast<unsigned long long>(temp_id), static_cast<int>(before),
              static_cast<int>(during), static_cast<int>(after), deleted ? 1 : 0,
              gone ? 1 : 0, (before == after) ? 1 : 0);
}

void StageTransforms(DemoScene& scene) {
  using namespace opentl::mvp;
  SceneDocument& document = scene.document;

  const bool translated = document.TranslateObject(scene.point_id, Vector3d(1.0, 2.0, 3.0));
  const SceneObject* point = document.FindById(scene.point_id);
  std::printf("  transform.translate id=%llu ok=%d at=%s\n",
              static_cast<unsigned long long>(scene.point_id), translated ? 1 : 0,
              nullptr != point ? Coord(point->point_val).c_str() : "-");

  const bool rotated = document.RotateObject(scene.line_id, ON_PI * 0.5, Vector3d(0.0, 0.0, 1.0));
  const SceneObject* line = document.FindById(scene.line_id);
  std::printf("  transform.rotate id=%llu ok=%d from=%s to=%s\n",
              static_cast<unsigned long long>(scene.line_id), rotated ? 1 : 0,
              nullptr != line ? Coord(line->line_val.from).c_str() : "-",
              nullptr != line ? Coord(line->line_val.to).c_str() : "-");

  const bool scaled = document.ScaleObject(scene.circle_id, 2.0, 2.0, 2.0);
  const SceneObject* circle = document.FindById(scene.circle_id);
  std::printf("  transform.scale id=%llu ok=%d radius=%s\n",
              static_cast<unsigned long long>(scene.circle_id), scaled ? 1 : 0,
              nullptr != circle ? Fixed(circle->circle_val.Radius()).c_str() : "-");

  const SceneObject* polyline = document.FindById(scene.polyline_id);
  if (nullptr != point && nullptr != circle) {
    std::printf("  distance.point_circle=%s\n",
                Fixed(DistanceBetweenObjects(*point, *circle)).c_str());
  }
  if (nullptr != line && nullptr != polyline) {
    std::printf("  distance.line_polyline=%s\n",
                Fixed(DistanceBetweenObjects(*line, *polyline)).c_str());
  }
  if (nullptr != point) {
    std::printf("  distance.point_to_plane=%s\n",
                Fixed(DistanceBetweenObjects(*point, *document.FindById(scene.plane_id))).c_str());
  }
  std::printf("  scene_bbox=%s\n", Box(document.GetSceneBoundingBox()).c_str());
}

void StageIntersections(DemoScene& scene) {
  using namespace opentl::mvp;
  SceneDocument& document = scene.document;

  const Line line_a(Point3d(0.0, 0.0, 0.0), Point3d(10.0, 0.0, 0.0));
  const Line line_cross(Point3d(5.0, -5.0, 0.0), Point3d(5.0, 5.0, 0.0));
  const Line line_skew(Point3d(5.0, -5.0, 5.0), Point3d(5.0, 5.0, 5.0));
  const Line line_parallel(Point3d(0.0, 1.0, 0.0), Point3d(10.0, 1.0, 0.0));
  const Line line_outside(Point3d(20.0, -5.0, 0.0), Point3d(20.0, 5.0, 0.0));

  const LineLineIntersection hit = IntersectLineLine(line_a, line_cross, 1e-6, false);
  std::printf("  line_line.cross intersects=%d parallel=%d ta=%s tb=%s point=%s distance=%s\n",
              hit.intersects ? 1 : 0, hit.is_parallel ? 1 : 0, Fixed(hit.param_a).c_str(),
              Fixed(hit.param_b).c_str(), Coord(hit.point_a).c_str(),
              Fixed(hit.distance).c_str());

  const LineLineIntersection skew = IntersectLineLine(line_a, line_skew, 1e-6, false);
  std::printf("  line_line.skew intersects=%d parallel=%d pa=%s pb=%s distance=%s\n",
              skew.intersects ? 1 : 0, skew.is_parallel ? 1 : 0, Coord(skew.point_a).c_str(),
              Coord(skew.point_b).c_str(), Fixed(skew.distance).c_str());

  const LineLineIntersection parallel = IntersectLineLine(line_a, line_parallel, 1e-6, false);
  std::printf("  line_line.parallel intersects=%d parallel=%d distance=%s\n",
              parallel.intersects ? 1 : 0, parallel.is_parallel ? 1 : 0,
              Fixed(parallel.distance).c_str());

  const LineLineIntersection finite = IntersectLineLine(line_a, line_outside, 1e-6, true);
  const LineLineIntersection infinite = IntersectLineLine(line_a, line_outside, 1e-6, false);
  std::printf("  line_line.finite_clip finite_intersects=%d infinite_intersects=%d t=%s\n",
              finite.intersects ? 1 : 0, infinite.intersects ? 1 : 0,
              Fixed(infinite.param_a).c_str());

  const Plane world_xy = Plane::World_xy;
  const Line through(Point3d(0.0, 0.0, -5.0), Point3d(0.0, 0.0, 5.0));
  const LinePlaneIntersection plane_hit = IntersectLinePlane(through, world_xy, false, 1e-9);
  std::printf("  line_plane.hit intersects=%d parallel=%d t=%s point=%s uv=(%s,%s)\n",
              plane_hit.intersects ? 1 : 0, plane_hit.is_parallel ? 1 : 0,
              Fixed(plane_hit.line_param).c_str(), Coord(plane_hit.point).c_str(),
              Fixed(plane_hit.plane_u).c_str(), Fixed(plane_hit.plane_v).c_str());

  const Line parallel_to_plane(Point3d(0.0, 0.0, 1.0), Point3d(1.0, 0.0, 1.0));
  const LinePlaneIntersection plane_parallel =
      IntersectLinePlane(parallel_to_plane, world_xy, false, 1e-9);
  std::printf("  line_plane.parallel intersects=%d parallel=%d\n",
              plane_parallel.intersects ? 1 : 0, plane_parallel.is_parallel ? 1 : 0);

  const Line short_of_plane(Point3d(0.0, 0.0, -5.0), Point3d(0.0, 0.0, -1.0));
  const LinePlaneIntersection plane_finite =
      IntersectLinePlane(short_of_plane, world_xy, true, 1e-9);
  std::printf("  line_plane.finite_segment intersects=%d parallel=%d\n",
              plane_finite.intersects ? 1 : 0, plane_finite.is_parallel ? 1 : 0);

  // ---- curve-line ------------------------------------------------------
  {
    const SceneObject* circle_object = document.FindById(scene.circle_id);
    if (nullptr != circle_object) {
      ON_ArcCurve curve(circle_object->circle_val);
      const Line cutter(Point3d(-100.0, 0.0, 0.0), Point3d(100.0, 0.0, 0.0));
      ON_SimpleArray<TL_CurveCurveIntersectionEvent> events;
      const int count = TL_IntersectCurveLine(curve, cutter, 1e-6, events);
      std::string points;
      for (int i = 0; i < events.Count(); ++i) {
        points += (i == 0) ? "" : "; ";
        points += Coord(events[i].point_a);
      }
      std::printf("  curve_line.count=%d points=%s\n", count > 0 ? count : 0, points.c_str());
    }
  }

  // ---- curve-plane -----------------------------------------------------
  {
    const SceneObject* circle_object = document.FindById(scene.circle_id);
    if (nullptr != circle_object) {
      ON_ArcCurve curve(circle_object->circle_val);
      const Plane world_yz = Plane::World_yz;
      ON_SimpleArray<double> parameters;
      ON_SimpleArray<ON_3dPoint> points;
      const int count = TL_IntersectCurvePlane(curve, world_yz, 1e-6, parameters, points);
      std::string text;
      for (int i = 0; i < points.Count(); ++i) {
        text += (i == 0) ? "" : "; ";
        text += Coord(points[i]) + "@t=" + Fixed(parameters[i]);
      }
      std::printf("  curve_plane.count=%d points=%s\n", count > 0 ? count : 0, text.c_str());
    }
  }

  // ---- NURBS sampling --------------------------------------------------
  {
    const SceneObject* nurbs_object = document.FindById(scene.nurbs_id);
    if (nullptr != nurbs_object) {
      const std::vector<CurveSamplePoint> uniform = SampleCurveByCount(nurbs_object->nurbs_val, 9, false);
      const std::vector<CurveSamplePoint> arc_length = SampleCurveByCount(nurbs_object->nurbs_val, 9, true);
      std::printf("  nurbs.sample_uniform count=%d arc_length=%s mid=%s\n",
                  static_cast<int>(uniform.size()),
                  uniform.empty() ? "-" : Fixed(uniform.back().arc_length).c_str(),
                  uniform.empty() ? "-" : Coord(uniform[uniform.size() / 2].point).c_str());
      std::printf("  nurbs.sample_arclength count=%d arc_length=%s mid=%s\n",
                  static_cast<int>(arc_length.size()),
                  arc_length.empty() ? "-" : Fixed(arc_length.back().arc_length).c_str(),
                  arc_length.empty() ? "-" : Coord(arc_length[arc_length.size() / 2].point).c_str());
      double t = 0.0;
      const bool closest = TL_GetCurveClosestPoint(nurbs_object->nurbs_val,
                                                  Point3d(6.0, 10.0, 0.0), &t);
      const ON_3dPoint closest_point = nurbs_object->nurbs_val.PointAt(t);
      std::printf("  nurbs.closest_point ok=%d t=%s point=%s\n", closest ? 1 : 0,
                  Fixed(t).c_str(), Coord(closest_point).c_str());
    }
  }
}

bool StageSerialization(DemoScene& scene, const std::string& out_dir, int* exit_code) {
  using namespace opentl::mvp;

  // Make the output directory exist so `--out-dir` works for fresh paths and
  // for the CTest working directory (which has no demo_out/ yet).
  {
    std::error_code directory_error;
    if (!out_dir.empty() && !std::filesystem::exists(out_dir, directory_error)) {
      std::filesystem::create_directories(out_dir, directory_error);
      if (directory_error) {
        std::printf("  io.save out_dir_error=%s (%s)\n", out_dir.c_str(),
                    directory_error.message().c_str());
        if (nullptr != exit_code) *exit_code = 1;
        return false;
      }
    }
  }

  const std::string json_path = out_dir + "/demo_scene.otscene.json";
  const std::string dm_path = out_dir + "/demo_scene.3dm";

  const std::string json_digest = BuildSceneDigest(scene.document);
  const std::string json_digest_hash = HashHex(json_digest);

  std::string json_error;
  const bool json_saved = scene.document.SaveJsonScene(json_path, &json_error);
  std::string dm_error;
  const bool dm_saved = scene.document.Save3dm(dm_path, &dm_error);
  std::printf("  io.save json=%d (%s) 3dm=%d (%s)\n", json_saved ? 1 : 0, json_path.c_str(),
              dm_saved ? 1 : 0, dm_path.c_str());
  if (!json_saved) std::printf("  io.save json_error=%s\n", json_error.c_str());
  if (!dm_saved) std::printf("  io.save 3dm_error=%s\n", dm_error.c_str());

  SceneDocument json_loaded;
  std::string json_load_error;
  const bool json_loaded_ok = json_loaded.LoadJsonScene(json_path, &json_load_error);
  const std::string json_loaded_digest = BuildSceneDigest(json_loaded);

  SceneDocument dm_loaded;
  std::string dm_load_error;
  const bool dm_loaded_ok = dm_loaded.Load3dm(dm_path, &dm_load_error);
  const std::string dm_loaded_digest = BuildSceneDigest(dm_loaded);

  std::printf("  io.load json=%d 3dm=%d\n", json_loaded_ok ? 1 : 0, dm_loaded_ok ? 1 : 0);
  if (!json_loaded_ok) std::printf("  io.load json_error=%s\n", json_load_error.c_str());
  if (!dm_loaded_ok) std::printf("  io.load 3dm_error=%s\n", dm_load_error.c_str());

  PrintDigest("original", scene.document);
  PrintDigest("json_reload", json_loaded);
  PrintDigest("3dm_reload", dm_loaded);

  const bool json_identical = json_loaded_ok && (json_loaded_digest == json_digest);
  const bool dm_identical = dm_loaded_ok && (dm_loaded_digest == json_digest);
  std::printf("  io.roundtrip json_digest_identical=%d 3dm_digest_identical=%d\n",
              json_identical ? 1 : 0, dm_identical ? 1 : 0);
  std::printf("  io.digest original=%s json=%s 3dm=%s\n", json_digest_hash.c_str(),
              HashHex(json_loaded_digest).c_str(), HashHex(dm_loaded_digest).c_str());

  if (!json_saved || !dm_saved || !json_loaded_ok || !dm_loaded_ok) {
    if (nullptr != exit_code) *exit_code = 1;
    return false;
  }
  if (!json_identical || !dm_identical) {
    if (nullptr != exit_code) *exit_code = 1;
    return false;
  }
  if (nullptr != exit_code) *exit_code = 0;
  return true;
}

// ---------------------------------------------------------------------------
// --views : four-view layout demo (Top / Perspective / Front / Right)
// ---------------------------------------------------------------------------

int RunFourViewDemo(const std::string& out_dir, int* exit_code) {
  TL_FourViewLayout layout = TL_FourViewLayout::Default(1600, 900);

  ON_BoundingBox box;
  box.Set(ON_3dPoint(-10.0, -6.0, -2.0));
  box.Set(ON_3dPoint(10.0, 6.0, 8.0), true);
  layout.ZoomToFitAll(box, 0.9);

  const ON_3dPoint probes[3] = {
    ON_3dPoint(0.0, 0.0, 0.0), ON_3dPoint(10.0, 0.0, 0.0), ON_3dPoint(0.0, 6.0, 8.0)
  };

  std::printf("views.canvas=%dx%d count=%d\n", layout.CanvasWidth(), layout.CanvasHeight(),
              layout.Count());
  for (int i = 0; i < layout.Count(); ++i) {
    const TL_Viewport& vp = layout.View(i);
    const ON_3dPoint eye = vp.CameraLocation();
    std::printf("views[%d].name=%s projection=%s\n", i, vp.m_name.c_str(),
                (TL_PROJECTION_PERSPECTIVE == vp.m_projection) ? "perspective" : "parallel");
    std::printf("views[%d].rect=%d,%d %dx%d\n", i, vp.m_left, vp.m_top, vp.m_width, vp.m_height);
    std::printf("views[%d].camera=%s target=%s dir=%s\n", i, Coord(ON_3dPoint(eye)).c_str(),
                Coord(vp.m_target).c_str(), Vec(vp.m_camera_dir).c_str());
    std::printf("views[%d].scale=%s lens=%s distance=%s\n", i, Fixed(vp.m_scale, 6).c_str(),
                Fixed(vp.m_lens_mm, 3).c_str(), Fixed(vp.m_target_distance, 6).c_str());
    for (int k = 0; k < 3; ++k) {
      ON_2dPoint screen = ON_2dPoint::Origin;
      if (!vp.WorldToScreen(probes[k], screen)) {
        std::printf("views[%d].proj[%d]=<behind>\n", i, k);
        continue;
      }
      std::printf("views[%d].proj[%d]=%s,%s\n", i, k, Fixed(screen.x, 3).c_str(),
                  Fixed(screen.y, 3).c_str());
    }
  }

  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  const std::string path = (std::filesystem::path(out_dir) / "fourview.json").string();
  const bool saved = layout.SaveJson(path);
  std::printf("views.save=%s path=%s\n", saved ? "ok" : "failed", path.c_str());

  TL_FourViewLayout reloaded;
  const bool loaded = saved && reloaded.LoadJson(path);
  bool identical = loaded;
  if (loaded) {
    for (int i = 0; i < layout.Count() && identical; ++i) {
      const TL_Viewport& a = layout.View(i);
      const TL_Viewport& b = reloaded.View(i);
      identical = a.m_name == b.m_name && a.m_standard_view == b.m_standard_view &&
                  a.m_projection == b.m_projection && a.m_left == b.m_left &&
                  a.m_top == b.m_top && a.m_width == b.m_width && a.m_height == b.m_height &&
                  std::fabs(a.m_scale - b.m_scale) <= 1e-9 &&
                  a.m_target.DistanceTo(b.m_target) <= 1e-9;
    }
  }
  std::printf("views.roundtrip=%s\n", identical ? "identical" : "mismatch");
  std::printf("views.status=%s\n", (saved && identical) ? "OK" : "FAILED");

  if (nullptr != exit_code) *exit_code = (saved && identical) ? 0 : 1;
  return (saved && identical) ? 0 : 1;
}

// ---------------------------------------------------------------------------
// --render-vulkan : Vulkan four-view render to PNG
// ---------------------------------------------------------------------------

int RunVulkanRender(const std::string& out_dir, bool use_demo_scene, int* exit_code) {
  opentl::mvp::SceneDocument document;
  if (use_demo_scene) {
    DemoScene scene;
    BuildDemoScene(scene);
    document = scene.document;
  } else {
    document.AddLayer("Curves", 116, 199, 236, true);              // Catppuccin Sapphire
    document.AddCircle(ON_Circle(ON_Plane::World_xy, 5.0), "circle", "Curves");
    document.AddPoint(ON_3dPoint(2.0, 3.0, 4.0), "point", "Curves");
  }

  // A shaded surface so the renderer exercises its triangle pipeline: sweep a circle
  // along a quarter arc, then tessellate it into an ON_Mesh grid.
  std::vector<ON_Mesh*> owned_meshes;
  ON_NurbsSurface* swept = nullptr;
  {
    ON_ArcCurve rail(ON_Arc(ON_Plane(ON_3dPoint(-6.0, -6.0, 0.0), ON_3dVector::ZAxis), 9.0, ON_PI));
    ON_ArcCurve profile(ON_Circle(ON_Plane::World_xy, 1.6));
    swept = TL_Sweep1Rail(rail, profile, 24, nullptr);
    if (nullptr != swept && swept->IsValid()) {
      ON_Mesh* mesh = TL_MeshSurfaceGrid(*swept, 72, 128, nullptr);
      if (nullptr != mesh) {
        // Close the open ends so the tube reads as a solid and backface culling is safe.
        int cap_triangles = 0;
        int open_chains = 0;
        const int caps = TL_CapMeshOpenBoundaryLoops(*mesh, &cap_triangles, &open_chains);
        std::printf("vulkan.mesh_caps=%d cap_triangles=%d open_chains=%d boundary_edges=%d\n",
                    caps, cap_triangles, open_chains, TL_CountMeshBoundaryEdges(*mesh));
        mesh->ComputeFaceNormals();
        mesh->ComputeVertexNormals();
        owned_meshes.push_back(mesh);
      }
    }
  }

  // Real surface x surface intersection, computed at render time and added to the scene
  // as yellow polylines: the curve the kernel actually solved, not a decorative circle.
  std::vector<TL_SurfaceSurfaceIntersectionBranch> ssx_branches;
  if (nullptr != swept && swept->IsValid()) {
    ON_NurbsSurface plane_patch(3, false, 2, 2, 2, 2);
    const ON_Plane plane(ON_3dPoint(0.0, 0.0, 0.0), ON_3dVector::ZAxis);
    const double half = 22.0;
    plane_patch.SetCV(0, 0, plane.PointAt(-half, -half));
    plane_patch.SetCV(0, 1, plane.PointAt(-half, half));
    plane_patch.SetCV(1, 0, plane.PointAt(half, -half));
    plane_patch.SetCV(1, 1, plane.PointAt(half, half));
    plane_patch.SetKnot(0, 0, 0.0);
    plane_patch.SetKnot(0, 1, 1.0);
    plane_patch.SetKnot(1, 0, 0.0);
    plane_patch.SetKnot(1, 1, 1.0);

    const int branch_count = TL_SurfaceSurfaceIntersect(*swept, plane_patch, 1e-7, ssx_branches);
    if (branch_count > 0) {
      document.AddLayer("Intersection", 249, 226, 175, true);   // Catppuccin Yellow
      double worst_error = 0.0;
      int points_total = 0;
      for (const auto& branch : ssx_branches) {
        std::vector<ON_3dPoint> pts;
        for (int i = 0; i < branch.points_3d.Count(); ++i)
          pts.push_back(branch.points_3d[i]);
        if (pts.size() >= 2)
          document.AddPolyline(pts, "tube_x_plane", "Intersection");
        worst_error = std::max(worst_error, branch.max_error);
        points_total += branch.points_3d.Count();
      }
      std::printf("vulkan.ssx_branches=%d ssx_points=%d ssx_max_error=%.3e\n", branch_count,
                  points_total, worst_error);
    }
  }

  std::vector<const ON_Mesh*> meshes;
  for (const ON_Mesh* mesh : owned_meshes)
    meshes.push_back(mesh);

  TL_FourViewLayout layout = TL_FourViewLayout::Default(1600, 900);
  ON_BoundingBox bbox = document.GetSceneBoundingBox();
  for (const ON_Mesh* mesh : owned_meshes) {
    ON_BoundingBox mesh_box;
    if (mesh->GetBoundingBox(mesh_box) && mesh_box.IsValid())
      bbox.Union(mesh_box);
  }
  if (bbox.IsValid())
    layout.ZoomToFitAll(bbox, 0.9);

  TL_VulkanFourViewRenderer renderer;
  TL_VulkanRenderResult result;
  TL_VulkanRenderOptions options;
  options.sample_count = 4;
  options.enable_depth = true;
  options.enable_shaded_meshes = true;
  options.cull_backfaces = true;
  const bool rendered = renderer.RenderScene(document, layout, result, meshes, options);

  std::printf("vulkan.backend=%s\n", result.backend_name.c_str());
  std::printf("vulkan.device=%s\n", result.device_name.c_str());
  std::printf("vulkan.canvas=%dx%d vertices=%zu triangles=%zu\n", result.width, result.height,
              result.vertex_count, result.triangle_count);
  std::printf("vulkan.theme=%s msaa=%dx depth=%s\n", result.theme_name.c_str(),
              result.msaa_samples, result.depth_enabled ? "D32_SFLOAT" : "disabled");
  std::printf("vulkan.feature_edges=%zu (silhouette + crease)\n", result.feature_edge_count);
  std::printf("vulkan.render=%s\n", rendered ? "ok" : "failed");
  if (!rendered) {
    std::printf("vulkan.error=%s\n", result.error.c_str());
    std::printf("vulkan.status=FAILED\n");
    for (ON_Mesh* mesh : owned_meshes)
      delete mesh;
    delete swept;
    if (nullptr != exit_code) *exit_code = 1;
    return 1;
  }

  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  const std::string path = (std::filesystem::path(out_dir) / "fourview.png").string();
  std::string save_error;
  const bool saved = TL_VulkanFourViewRenderer::SavePng(path, result.width, result.height,
                                                        result.rgba, &save_error);
  std::printf("vulkan.png=%s path=%s bytes=%zu\n", saved ? "ok" : "failed", path.c_str(),
              result.rgba.size());
  if (!saved)
    std::printf("vulkan.png_error=%s\n", save_error.c_str());
  std::printf("vulkan.status=%s\n", saved ? "OK" : "FAILED");
  TL_FreeSSXBranches(ssx_branches);
  for (ON_Mesh* mesh : owned_meshes)
    delete mesh;
  delete swept;
  if (nullptr != exit_code) *exit_code = saved ? 0 : 1;
  return saved ? 0 : 1;
}

void PrintUsage() {
  std::printf("usage:\n");
  std::printf("  opentl_cli [--demo] [--out-dir <dir>]     run the deterministic MVP demo\n");
  std::printf("  opentl_cli --inspect-3dm <file.3dm>       summarize a .3dm archive\n");
  std::printf("  opentl_cli --inspect-json <file.json>     summarize an .otscene.json file\n");
  std::printf("  opentl_cli --views [--out-dir <dir>]     four-view layout demo (Top/Persp/Front/Right)\n");
  std::printf("  opentl_cli --render-vulkan [--scene demo] [--out-dir <dir>]\n");
  std::printf("                                           render four views offline with Vulkan -> fourview.png\n");
}

}  // namespace

int main(int argc, char** argv) {
  ON::Begin();

  std::string mode = "demo";
  std::string out_dir = ".";
  std::string inspect_path;
  std::string scene_mode = "demo";

  for (int i = 1; i < argc; ++i) {
    const std::string arg = (nullptr != argv[i]) ? argv[i] : "";
    if ("--demo" == arg) {
      mode = "demo";
    } else if ("--out-dir" == arg) {
      if (i + 1 >= argc) {
        std::printf("error: --out-dir requires a directory argument\n");
        PrintUsage();
        return 2;
      }
      out_dir = argv[++i];
    } else if ("--inspect-3dm" == arg) {
      if (i + 1 >= argc) {
        std::printf("error: --inspect-3dm requires a file argument\n");
        PrintUsage();
        return 2;
      }
      mode = "inspect-3dm";
      inspect_path = argv[++i];
    } else if ("--inspect-json" == arg) {
      if (i + 1 >= argc) {
        std::printf("error: --inspect-json requires a file argument\n");
        PrintUsage();
        return 2;
      }
      mode = "inspect-json";
      inspect_path = argv[++i];
    } else if ("--views" == arg) {
      mode = "views";
    } else if ("--render-vulkan" == arg) {
      mode = "render-vulkan";
    } else if ("--scene" == arg) {
      if (i + 1 >= argc) {
        std::printf("error: --scene requires 'demo' or 'minimal'\n");
        PrintUsage();
        return 2;
      }
      scene_mode = argv[++i];
    } else if ("--help" == arg || "-h" == arg) {
      PrintUsage();
      return 0;
    } else {
      std::printf("error: unknown argument '%s'\n", arg.c_str());
      PrintUsage();
      return 2;
    }
  }

  std::printf("opentl_cli mode=%s\n", mode.c_str());

  if ("render-vulkan" == mode) {
    int code = 0;
    RunVulkanRender(out_dir, scene_mode != "minimal", &code);
    ON::End();
    return code;
  }

  if ("views" == mode) {
    int code = 0;
    RunFourViewDemo(out_dir, &code);
    ON::End();
    return code;
  }

  if ("inspect-3dm" == mode || "inspect-json" == mode) {
    opentl::mvp::SceneDocument document;
    std::string error;
    const bool ok = ("inspect-3dm" == mode) ? document.Load3dm(inspect_path, &error)
                                            : document.LoadJsonScene(inspect_path, &error);
    std::printf("inspect.file=%s ok=%d\n", inspect_path.c_str(), ok ? 1 : 0);
    if (!ok) {
      std::printf("inspect.error=%s\n", error.c_str());
      ON::End();
      return 1;
    }
    const std::string digest = BuildSceneDigest(document);
    PrintDigest("inspect", document);
    std::printf("inspect.scene_bbox=%s\n", Box(document.GetSceneBoundingBox()).c_str());
    std::printf("inspect.digest_begin\n%s", digest.c_str());
    std::printf("inspect.digest_end\n");
    ON::End();
    return 0;
  }

  // ------------------------------------------------------------------------
  // Full MVP demo
  // ------------------------------------------------------------------------
  std::printf("\n== 1. scene construction ==\n");
  DemoScene scene;
  BuildDemoScene(scene);
  PrintSceneHeader(scene.document);

  std::printf("\n== 2. queries and delete ==\n");
  StageQueries(scene);

  std::printf("\n== 3. transformations, distances, bounds ==\n");
  StageTransforms(scene);

  std::printf("\n== 4. intersections and curve sampling ==\n");
  StageIntersections(scene);

  std::printf("\n== 5. serialization round-trip (.otscene.json + .3dm) ==\n");
  int exit_code = 0;
  const bool roundtrip_ok = StageSerialization(scene, out_dir, &exit_code);

  std::printf("\n== 6. demo summary ==\n");
  std::printf("  status=%s\n", roundtrip_ok ? "OK" : "FAILED");
  std::printf("  layers=%d objects=%d\n",
              static_cast<int>(scene.document.Layers().size()),
              static_cast<int>(scene.document.ObjectCount()));
  std::printf("  scene_bbox=%s\n", Box(scene.document.GetSceneBoundingBox()).c_str());
  std::printf("  scene_digest=%s\n", HashHex(BuildSceneDigest(scene.document)).c_str());

  ON::End();
  return exit_code;
}
