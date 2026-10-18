/**
 * @file geometry_scene.cpp
 * @brief OpenTL MVP: Rhino-like SceneDocument, SceneObject geometry operations,
 *        affine transform builders, distance queries and deterministic summary.
 *
 * Reverse-engineering provenance (see re-database/rea/mvp_static_evidence.json):
 *   - RhinoCommon.dll  SHA256 7371baa5...ee89  -> Rhino.Geometry.{Point3d,Vector3d,
 *     Line,Polyline,Circle,Arc,Plane,BoundingBox,NurbsCurve,Transform}
 *   - rhcommon_c.dll   SHA256 d9006bce...29c1  -> ON_Line_*, ON_Circle_*, ON_Plane_*,
 *     ON_Arc_*, ON_NurbsCurve_*, ON_BoundingBox_* C ABI shims
 *   - tl.dll           SHA256 550a5410...3a7f  -> TL_GetClosestPoint (RVA 0x214540 /
 *     0x214670), TL_DeformableArc2 (0x37b220), TL_DeformableCircle (0x37b790)
 *
 * The geometry math itself is delegated to the legitimately redistributable
 * openNURBS kernel (external/opennurbs); no Rhino binary is loaded or executed.
 */

#include "opentl/mvp/rhino_mvp.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace opentl {
namespace mvp {
namespace {

// FNV-1a 64-bit, used to derive stable per-name layer uuids.
uint64_t Fnv1a64(const std::string& text, uint64_t seed) {
  uint64_t h = seed;
  for (size_t i = 0; i < text.size(); ++i) {
    h ^= static_cast<uint64_t>(static_cast<unsigned char>(text[i]));
    h *= 1099511628211ULL;
  }
  return h;
}

// splitmix64 finaliser: deterministic, well distributed 64-bit mixing.
uint64_t SplitMix64(uint64_t x) {
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

bool UuidIsNil(const ON_UUID& id) { return 0 == ON_UuidCompare(id, ON_nil_uuid); }

void FormatDouble(char* buffer, size_t size, double value) {
  if (!std::isfinite(value)) {
    std::snprintf(buffer, size, "nan");
    return;
  }
  std::snprintf(buffer, size, "%.6f", value);
}

std::string NumbersToText(const char* prefix, double a, double b, double c) {
  char x[32];
  char y[32];
  char z[32];
  FormatDouble(x, sizeof(x), a);
  FormatDouble(y, sizeof(y), b);
  FormatDouble(z, sizeof(z), c);
  std::string out(prefix);
  out += "(";
  out += x;
  out += ",";
  out += y;
  out += ",";
  out += z;
  out += ")";
  return out;
}

std::string PointToText(const ON_3dPoint& pt) { return NumbersToText("", pt.x, pt.y, pt.z); }

// Distance from a point to a plane-backed object, in absolute value.
double PlaneAbsDistance(const ON_Plane& plane, const ON_3dPoint& pt) {
  ON_Plane copy = plane;
  copy.UpdateEquation();
  return std::fabs(copy.DistanceTo(pt));
}

// Collect representative sample points of a curve-like object. Used for the
// plane-distance and generic fallback distance paths.
void CollectCurveSamples(const SceneObject& object, std::vector<ON_3dPoint>& points) {
  points.clear();
  switch (object.kind) {
    case ObjectKind::Point:
      points.push_back(object.point_val);
      break;
    case ObjectKind::Line:
      points.push_back(object.line_val.from);
      points.push_back(object.line_val.to);
      break;
    case ObjectKind::Polyline:
      for (int i = 0; i < object.polyline_val.PointCount(); ++i) {
        points.push_back(object.polyline_val[i]);
      }
      break;
    case ObjectKind::Circle:
    case ObjectKind::Arc: {
      ON_NurbsCurve curve;
      if (object.ToNurbsCurve(curve)) {
        const ON_Interval domain = curve.Domain();
        const int count = 64;
        for (int i = 0; i <= count; ++i) {
          const double t = domain.ParameterAt(static_cast<double>(i) / static_cast<double>(count));
          points.push_back(curve.PointAt(t));
        }
      }
      break;
    }
    case ObjectKind::Plane:
      points.push_back(object.plane_val.origin);
      break;
    case ObjectKind::NurbsCurve: {
      if (!object.nurbs_val.IsValid()) break;
      const ON_Interval domain = object.nurbs_val.Domain();
      const int count = 64;
      for (int i = 0; i <= count; ++i) {
        const double t = domain.ParameterAt(static_cast<double>(i) / static_cast<double>(count));
        points.push_back(object.nurbs_val.PointAt(t));
      }
      break;
    }
  }
}

}  // namespace

// ============================================================================
// 1. ObjectKind <-> string
// ============================================================================

const char* ObjectKindToString(ObjectKind kind) {
  switch (kind) {
    case ObjectKind::Point:      return "Point";
    case ObjectKind::Line:       return "Line";
    case ObjectKind::Polyline:   return "Polyline";
    case ObjectKind::Circle:     return "Circle";
    case ObjectKind::Arc:        return "Arc";
    case ObjectKind::Plane:      return "Plane";
    case ObjectKind::NurbsCurve: return "NurbsCurve";
  }
  return "Unknown";
}

bool ObjectKindFromString(const std::string& str, ObjectKind* out_kind) {
  if (nullptr == out_kind) return false;
  static const ObjectKind kKinds[7] = {
      ObjectKind::Point,   ObjectKind::Line,   ObjectKind::Polyline, ObjectKind::Circle,
      ObjectKind::Arc,     ObjectKind::Plane,  ObjectKind::NurbsCurve};
  for (int i = 0; i < 7; ++i) {
    const char* text = ObjectKindToString(kKinds[i]);
    if (nullptr != text && str == text) {
      *out_kind = kKinds[i];
      return true;
    }
  }
  return false;
}

// ============================================================================
// 2. Deterministic UUIDs
// ============================================================================

ON_UUID MakeDeterministicUuid(uint64_t seed) {
  const uint64_t a = SplitMix64(seed ^ 0x0F1E2D3C4B5A6978ULL);
  const uint64_t b = SplitMix64(a + 0x9E3779B97F4A7C15ULL);

  ON_UUID id = ON_nil_uuid;
  id.Data1 = static_cast<unsigned int>(a & 0xFFFFFFFFULL);
  id.Data2 = static_cast<unsigned short>((a >> 32) & 0xFFFFULL);
  id.Data3 = static_cast<unsigned short>(((a >> 48) & 0x0FFFULL) | 0x4000ULL);  // version 4
  for (int i = 0; i < 8; ++i) {
    id.Data4[i] = static_cast<unsigned char>((b >> (8 * i)) & 0xFFULL);
  }
  id.Data4[0] = static_cast<unsigned char>((id.Data4[0] & 0x3F) | 0x80);  // RFC 4122 variant

  if (UuidIsNil(id)) id.Data1 = 1;  // never hand out the nil uuid
  return id;
}

std::string UuidToString(const ON_UUID& id) {
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%08x-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                static_cast<unsigned int>(id.Data1), static_cast<unsigned int>(id.Data2),
                static_cast<unsigned int>(id.Data3), static_cast<unsigned int>(id.Data4[0]),
                static_cast<unsigned int>(id.Data4[1]), static_cast<unsigned int>(id.Data4[2]),
                static_cast<unsigned int>(id.Data4[3]), static_cast<unsigned int>(id.Data4[4]),
                static_cast<unsigned int>(id.Data4[5]), static_cast<unsigned int>(id.Data4[6]),
                static_cast<unsigned int>(id.Data4[7]));
  return std::string(buffer);
}

ON_UUID StringToUuid(const std::string& str) {
  unsigned int data1 = 0;
  unsigned int data2 = 0;
  unsigned int data3 = 0;
  unsigned int d4[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  const int parsed = std::sscanf(str.c_str(),
                                 "%8x-%4x-%4x-%2x%2x-%2x%2x%2x%2x%2x%2x",
                                 &data1, &data2, &data3, &d4[0], &d4[1], &d4[2], &d4[3],
                                 &d4[4], &d4[5], &d4[6], &d4[7]);
  if (11 != parsed) return ON_nil_uuid;

  ON_UUID id = ON_nil_uuid;
  id.Data1 = static_cast<unsigned int>(data1 & 0xFFFFFFFFU);
  id.Data2 = static_cast<unsigned short>(data2 & 0xFFFFU);
  id.Data3 = static_cast<unsigned short>(data3 & 0xFFFFU);
  for (int i = 0; i < 8; ++i) id.Data4[i] = static_cast<unsigned char>(d4[i] & 0xFFU);
  return id;
}

// ============================================================================
// 3. Transform builders
// ============================================================================

Transform MakeTranslation(double dx, double dy, double dz) {
  return ON_Xform::TranslationTransformation(dx, dy, dz);
}

Transform MakeTranslation(const Vector3d& delta) {
  return ON_Xform::TranslationTransformation(delta);
}

Transform MakeRotation(double angle_radians, const Vector3d& axis, const Point3d& center) {
  ON_Xform xform = ON_Xform::IdentityTransformation;
  ON_3dVector unit_axis = axis;
  if (!unit_axis.Unitize() || !std::isfinite(angle_radians)) {
    return xform;  // degenerate axis: identity
  }
  xform.Rotation(angle_radians, unit_axis, center);
  return xform;
}

Transform MakeScale(const Point3d& anchor, double sx, double sy, double sz) {
  return ON_Xform::ScaleTransformation(anchor, sx, sy, sz);
}

Transform MakeUniformScale(const Point3d& anchor, double scale_factor) {
  return ON_Xform::ScaleTransformation(anchor, scale_factor);
}

// ============================================================================
// 4. SceneObject geometry operations
// ============================================================================

BoundingBox SceneObject::GetBoundingBox() const {
  ON_BoundingBox box;  // unset by default
  switch (kind) {
    case ObjectKind::Point: {
      box.Set(point_val);
      break;
    }
    case ObjectKind::Line: {
      box.Set(line_val.from);
      box.Set(line_val.to, true);
      break;
    }
    case ObjectKind::Polyline: {
      for (int i = 0; i < polyline_val.PointCount(); ++i) {
        box.Set(polyline_val[i], i > 0 || box.IsSet());
      }
      break;
    }
    case ObjectKind::Circle: {
      // Analytic tight bounding box of a circle in an arbitrary plane:
      // extent along world axis k is r * sqrt(u_k^2 + v_k^2).
      const ON_3dPoint& c = circle_val.plane.origin;
      const ON_3dVector& u = circle_val.plane.xaxis;
      const ON_3dVector& v = circle_val.plane.yaxis;
      const double r = circle_val.radius;
      if (!(r > 0.0) || !std::isfinite(r)) break;
      ON_3dPoint minimum;
      ON_3dPoint maximum;
      const double uu[3] = {u.x, u.y, u.z};
      const double vv[3] = {v.x, v.y, v.z};
      const double cc[3] = {c.x, c.y, c.z};
      double lo[3];
      double hi[3];
      for (int k = 0; k < 3; ++k) {
        const double extent = r * std::sqrt(uu[k] * uu[k] + vv[k] * vv[k]);
        lo[k] = cc[k] - extent;
        hi[k] = cc[k] + extent;
      }
      minimum.Set(lo[0], lo[1], lo[2]);
      maximum.Set(hi[0], hi[1], hi[2]);
      box = ON_BoundingBox(minimum, maximum);
      break;
    }
    case ObjectKind::Arc: {
      ON_BoundingBox arc_box;
      if (arc_val.GetBoundingBox(arc_box, false) && arc_box.IsSet()) {
        box = arc_box;
      } else {
        box = ON_BoundingBox(arc_val.StartPoint(), arc_val.EndPoint());
      }
      break;
    }
    case ObjectKind::Plane: {
      // Matches the [-10,10]^2 extent used when persisting a plane as an
      // ON_PlaneSurface in the .3dm writer.
      const ON_3dPoint& o = plane_val.origin;
      const ON_3dVector& u = plane_val.xaxis;
      const ON_3dVector& v = plane_val.yaxis;
      ON_3dPoint corners[4];
      corners[0] = o + 10.0 * u + 10.0 * v;
      corners[1] = o + 10.0 * u - 10.0 * v;
      corners[2] = o - 10.0 * u + 10.0 * v;
      corners[3] = o - 10.0 * u - 10.0 * v;
      box = ON_BoundingBox(corners[0], corners[0]);
      for (int i = 1; i < 4; ++i) box.Set(corners[i], true);
      break;
    }
    case ObjectKind::NurbsCurve: {
      if (nurbs_val.IsValid()) {
        ON_BoundingBox curve_box;
        if (nurbs_val.GetBoundingBox(curve_box, false) && curve_box.IsSet()) {
          box = curve_box;
        }
      }
      break;
    }
  }
  return box;
}

bool SceneObject::ApplyTransform(const Transform& xform) {
  switch (kind) {
    case ObjectKind::Point: {
      point_val = xform * point_val;
      return std::isfinite(point_val.x) && std::isfinite(point_val.y) &&
             std::isfinite(point_val.z);
    }
    case ObjectKind::Line: {
      line_val.Transform(xform);
      return std::isfinite(line_val.from.x) && std::isfinite(line_val.to.x);
    }
    case ObjectKind::Polyline: {
      // ON_Polyline has no Transform(); transform each vertex (affine).
      for (int i = 0; i < polyline_val.PointCount(); ++i) {
        polyline_val[i] = xform * polyline_val[i];
      }
      return true;
    }
    case ObjectKind::Circle: {
      ON_Circle transformed = circle_val;
      if (transformed.Transform(xform) && transformed.IsValid() && transformed.radius > 0.0) {
        circle_val = transformed;
        return true;
      }
      // Fallback for sheared / non-similarity transforms: transform the frame
      // manually and scale the radius by the average linear scale factor.
      ON_Circle fallback;
      const ON_3dPoint center = xform * circle_val.plane.origin;
      ON_3dVector xaxis = xform * circle_val.plane.xaxis;
      const double scale = xaxis.Length();
      if (!xaxis.Unitize()) return false;
      fallback.Create(ON_Plane(center, xaxis), circle_val.radius * scale);
      if (!fallback.IsValid()) return false;
      circle_val = fallback;
      return true;
    }
    case ObjectKind::Arc: {
      ON_Arc transformed = arc_val;
      if (transformed.Transform(xform) && transformed.IsValid() && transformed.radius > 0.0) {
        arc_val = transformed;
        return true;
      }
      ON_Arc fallback;
      const ON_3dPoint center = xform * arc_val.plane.origin;
      ON_3dVector xaxis = xform * arc_val.plane.xaxis;
      const double scale = xaxis.Length();
      if (!xaxis.Unitize()) return false;
      fallback.Create(ON_Plane(center, xaxis), arc_val.radius * scale,
                      arc_val.AngleRadians());
      if (!fallback.IsValid()) return false;
      arc_val = fallback;
      return true;
    }
    case ObjectKind::Plane: {
      if (!plane_val.Transform(xform)) return false;
      plane_val.UpdateEquation();
      return plane_val.IsValid();
    }
    case ObjectKind::NurbsCurve: {
      if (!nurbs_val.IsValid()) return false;
      nurbs_val.Transform(xform);
      return nurbs_val.IsValid();
    }
  }
  return false;
}

bool SceneObject::ToNurbsCurve(NurbsCurve& out_curve) const {
  switch (kind) {
    case ObjectKind::Line: {
      ON_LineCurve line_curve(line_val);
      return 0 != line_curve.GetNurbForm(out_curve);
    }
    case ObjectKind::Polyline: {
      if (polyline_val.PointCount() < 2) return false;
      ON_PolylineCurve polyline_curve(polyline_val);
      return 0 != polyline_curve.GetNurbForm(out_curve);
    }
    case ObjectKind::Circle:
      return 0 != circle_val.GetNurbForm(out_curve);
    case ObjectKind::Arc:
      return 0 != arc_val.GetNurbForm(out_curve);
    case ObjectKind::NurbsCurve:
      out_curve = nurbs_val;
      return out_curve.IsValid();
    case ObjectKind::Point:
    case ObjectKind::Plane:
      return false;
  }
  return false;
}

double SceneObject::DistanceTo(const Point3d& pt) const {
  switch (kind) {
    case ObjectKind::Point:
      return point_val.DistanceTo(pt);

    case ObjectKind::Line:
      return line_val.MinimumDistanceTo(pt);

    case ObjectKind::Polyline: {
      if (polyline_val.PointCount() <= 0) return 0.0;
      if (polyline_val.PointCount() == 1) return polyline_val[0].DistanceTo(pt);
      return polyline_val.ClosestPointTo(pt).DistanceTo(pt);
    }

    case ObjectKind::Circle:
      return circle_val.ClosestPointTo(pt).DistanceTo(pt);

    case ObjectKind::Arc:
      return arc_val.ClosestPointTo(pt).DistanceTo(pt);

    case ObjectKind::Plane:
      return PlaneAbsDistance(plane_val, pt);

    case ObjectKind::NurbsCurve: {
      if (!nurbs_val.IsValid()) return 0.0;
      double t = 0.0;
      if (TL_GetCurveClosestPoint(nurbs_val, pt, &t)) {
        const ON_3dPoint closest = nurbs_val.PointAt(t);
        const double distance = closest.DistanceTo(pt);
        if (std::isfinite(distance)) return distance;
      }
      // Fallback: dense sampling.
      std::vector<ON_3dPoint> samples;
      CollectCurveSamples(*this, samples);
      double best = 0.0;
      bool have = false;
      for (size_t i = 0; i < samples.size(); ++i) {
        const double d = samples[i].DistanceTo(pt);
        if (!have || d < best) {
          best = d;
          have = true;
        }
      }
      return have ? best : 0.0;
    }
  }
  return 0.0;
}

// ============================================================================
// 5. Object to object distance
// ============================================================================

double DistanceBetweenObjects(const SceneObject& a, const SceneObject& b) {
  // Point cases reduce to the exact point-to-object distance.
  if (ObjectKind::Point == a.kind) return b.DistanceTo(a.point_val);
  if (ObjectKind::Point == b.kind) return a.DistanceTo(b.point_val);

  // Plane cases: minimum absolute signed distance of the other object's samples.
  if (ObjectKind::Plane == a.kind || ObjectKind::Plane == b.kind) {
    const SceneObject& curve_object = (ObjectKind::Plane == a.kind) ? b : a;
    const ON_Plane& plane = (ObjectKind::Plane == a.kind) ? a.plane_val : b.plane_val;
    std::vector<ON_3dPoint> samples;
    CollectCurveSamples(curve_object, samples);
    if (samples.empty()) return 0.0;
    double best = PlaneAbsDistance(plane, samples[0]);
    for (size_t i = 1; i < samples.size(); ++i) {
      const double d = PlaneAbsDistance(plane, samples[i]);
      if (d < best) best = d;
    }
    return best;
  }

  // Curve-curve: alternating closest-point projection seeded by dense sampling.
  ON_NurbsCurve curve_a;
  ON_NurbsCurve curve_b;
  if (!a.ToNurbsCurve(curve_a) || !b.ToNurbsCurve(curve_b)) return 0.0;
  if (!curve_a.IsValid() || !curve_b.IsValid()) return 0.0;

  const ON_Interval domain_a = curve_a.Domain();
  const ON_Interval domain_b = curve_b.Domain();

  double best = 0.0;
  bool have = false;
  double best_ta = domain_a[0];
  double best_tb = domain_b[0];

  const int kSamples = 128;
  for (int i = 0; i <= kSamples; ++i) {
    const double ta = domain_a.ParameterAt(static_cast<double>(i) / kSamples);
    const ON_3dPoint pa = curve_a.PointAt(ta);
    double tb = domain_b[0];
    if (!TL_GetCurveClosestPoint(curve_b, pa, &tb)) continue;
    const ON_3dPoint pb = curve_b.PointAt(tb);
    const double distance = pa.DistanceTo(pb);
    if (!std::isfinite(distance)) continue;
    if (!have || distance < best) {
      best = distance;
      best_ta = ta;
      best_tb = tb;
      have = true;
    }
  }
  if (!have) return 0.0;

  // Alternate projection: converges quadratically for transversal pairs.
  for (int iteration = 0; iteration < 32; ++iteration) {
    const ON_3dPoint pa = curve_a.PointAt(best_ta);
    double tb = best_tb;
    if (!TL_GetCurveClosestPoint(curve_b, pa, &tb)) break;
    double ta = best_ta;
    const ON_3dPoint pb = curve_b.PointAt(tb);
    if (!TL_GetCurveClosestPoint(curve_a, pb, &ta)) break;
    const double distance = curve_a.PointAt(ta).DistanceTo(curve_b.PointAt(tb));
    if (!std::isfinite(distance)) break;
    const double previous = best;
    best_ta = ta;
    best_tb = tb;
    best = distance;
    if (std::fabs(previous - best) < 1e-13) break;
  }
  return best;
}

// ============================================================================
// 6. SceneDocument
// ============================================================================

SceneDocument::SceneDocument() { Clear(); }

void SceneDocument::Clear() {
  layers_.clear();
  objects_.clear();
  next_id_ = 1;

  Layer default_layer;
  default_layer.index = 0;
  default_layer.name = "Default";
  default_layer.id = MakeDeterministicUuid(Fnv1a64("Default", 0x4F50454E544CULL));
  default_layer.color_r = 0;
  default_layer.color_g = 0;
  default_layer.color_b = 0;
  default_layer.visible = true;
  layers_.push_back(default_layer);
}

int SceneDocument::AddLayer(const std::string& name, int r, int g, int b, bool visible) {
  const int existing = FindLayerByName(name);
  if (existing >= 0) return existing;

  Layer layer;
  layer.index = static_cast<int>(layers_.size());
  layer.name = name;
  layer.id = MakeDeterministicUuid(
      Fnv1a64(name, 0x4C41594552ULL + static_cast<uint64_t>(layer.index)));
  layer.color_r = r;
  layer.color_g = g;
  layer.color_b = b;
  layer.visible = visible;
  layers_.push_back(layer);
  return layer.index;
}

int SceneDocument::FindLayerByName(const std::string& name) const {
  for (size_t i = 0; i < layers_.size(); ++i) {
    if (layers_[i].name == name) return static_cast<int>(i);
  }
  return -1;
}

const Layer* SceneDocument::GetLayer(int layer_index) const {
  if (layer_index < 0 || layer_index >= static_cast<int>(layers_.size())) return nullptr;
  return &layers_[static_cast<size_t>(layer_index)];
}

int SceneDocument::EnsureLayerIndex(const std::string& layer_name) {
  const int existing = FindLayerByName(layer_name);
  if (existing >= 0) return existing;
  return AddLayer(layer_name, 128, 128, 128, true);
}

uint64_t SceneDocument::AllocateId() { return next_id_++; }

uint64_t SceneDocument::AddPoint(const Point3d& pt, const std::string& name,
                                 const std::string& layer) {
  SceneObject object;
  object.id = AllocateId();
  object.uuid = MakeDeterministicUuid(object.id);
  object.name = name;
  object.layer_index = EnsureLayerIndex(layer);
  object.kind = ObjectKind::Point;
  object.point_val = pt;
  objects_.push_back(object);
  return object.id;
}

uint64_t SceneDocument::AddLine(const Line& line, const std::string& name,
                                const std::string& layer) {
  SceneObject object;
  object.id = AllocateId();
  object.uuid = MakeDeterministicUuid(object.id);
  object.name = name;
  object.layer_index = EnsureLayerIndex(layer);
  object.kind = ObjectKind::Line;
  object.line_val = line;
  objects_.push_back(object);
  return object.id;
}

uint64_t SceneDocument::AddPolyline(const std::vector<Point3d>& pts, const std::string& name,
                                    const std::string& layer) {
  SceneObject object;
  object.id = AllocateId();
  object.uuid = MakeDeterministicUuid(object.id);
  object.name = name;
  object.layer_index = EnsureLayerIndex(layer);
  object.kind = ObjectKind::Polyline;
  for (size_t i = 0; i < pts.size(); ++i) object.polyline_val.Append(pts[i]);
  objects_.push_back(object);
  return object.id;
}

uint64_t SceneDocument::AddCircle(const Circle& circle, const std::string& name,
                                  const std::string& layer) {
  SceneObject object;
  object.id = AllocateId();
  object.uuid = MakeDeterministicUuid(object.id);
  object.name = name;
  object.layer_index = EnsureLayerIndex(layer);
  object.kind = ObjectKind::Circle;
  object.circle_val = circle;
  objects_.push_back(object);
  return object.id;
}

uint64_t SceneDocument::AddArc(const Arc& arc, const std::string& name,
                               const std::string& layer) {
  SceneObject object;
  object.id = AllocateId();
  object.uuid = MakeDeterministicUuid(object.id);
  object.name = name;
  object.layer_index = EnsureLayerIndex(layer);
  object.kind = ObjectKind::Arc;
  object.arc_val = arc;
  objects_.push_back(object);
  return object.id;
}

uint64_t SceneDocument::AddPlane(const Plane& plane, const std::string& name,
                                 const std::string& layer) {
  SceneObject object;
  object.id = AllocateId();
  object.uuid = MakeDeterministicUuid(object.id);
  object.name = name;
  object.layer_index = EnsureLayerIndex(layer);
  object.kind = ObjectKind::Plane;
  object.plane_val = plane;
  objects_.push_back(object);
  return object.id;
}

uint64_t SceneDocument::AddNurbsCurve(const NurbsCurve& curve, const std::string& name,
                                      const std::string& layer) {
  if (!curve.IsValid()) return 0;
  SceneObject object;
  object.id = AllocateId();
  object.uuid = MakeDeterministicUuid(object.id);
  object.name = name;
  object.layer_index = EnsureLayerIndex(layer);
  object.kind = ObjectKind::NurbsCurve;
  object.nurbs_val = curve;
  objects_.push_back(object);
  return object.id;
}

uint64_t SceneDocument::AddNurbsCurveFromPoints(const std::vector<Point3d>& control_pts,
                                                int degree, const std::string& name,
                                                const std::string& layer) {
  NurbsCurve curve;
  if (!CreateClampedNurbsCurve(control_pts, degree, curve)) return 0;
  return AddNurbsCurve(curve, name, layer);
}

bool SceneDocument::DeleteObject(uint64_t id) {
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].id == id) {
      objects_.erase(objects_.begin() + static_cast<std::ptrdiff_t>(i));
      return true;
    }
  }
  return false;
}

const SceneObject* SceneDocument::FindById(uint64_t id) const {
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].id == id) return &objects_[i];
  }
  return nullptr;
}

SceneObject* SceneDocument::FindByIdMutable(uint64_t id) {
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].id == id) return &objects_[i];
  }
  return nullptr;
}

const SceneObject* SceneDocument::FindByUuid(const ON_UUID& uuid) const {
  if (UuidIsNil(uuid)) return nullptr;
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (0 == ON_UuidCompare(objects_[i].uuid, uuid)) return &objects_[i];
  }
  return nullptr;
}

std::vector<const SceneObject*> SceneDocument::FindByName(const std::string& name) const {
  std::vector<const SceneObject*> result;
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].name == name) result.push_back(&objects_[i]);
  }
  return result;
}

std::vector<const SceneObject*> SceneDocument::FindByLayer(const std::string& layer_name) const {
  std::vector<const SceneObject*> result;
  const int layer_index = FindLayerByName(layer_name);
  if (layer_index < 0) return result;
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].layer_index == layer_index) result.push_back(&objects_[i]);
  }
  return result;
}

std::vector<const SceneObject*> SceneDocument::FindByKind(ObjectKind kind) const {
  std::vector<const SceneObject*> result;
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].kind == kind) result.push_back(&objects_[i]);
  }
  return result;
}

bool SceneDocument::TransformObject(uint64_t id, const Transform& xform) {
  SceneObject* object = FindByIdMutable(id);
  if (nullptr == object) return false;
  return object->ApplyTransform(xform);
}

bool SceneDocument::TranslateObject(uint64_t id, const Vector3d& delta) {
  return TransformObject(id, MakeTranslation(delta));
}

bool SceneDocument::RotateObject(uint64_t id, double angle_radians, const Vector3d& axis,
                                 const Point3d& center) {
  return TransformObject(id, MakeRotation(angle_radians, axis, center));
}

bool SceneDocument::ScaleObject(uint64_t id, double sx, double sy, double sz,
                                const Point3d& anchor) {
  return TransformObject(id, MakeScale(anchor, sx, sy, sz));
}

BoundingBox SceneDocument::GetSceneBoundingBox() const {
  ON_BoundingBox box;
  bool have = false;
  for (size_t i = 0; i < objects_.size(); ++i) {
    const ON_BoundingBox object_box = objects_[i].GetBoundingBox();
    if (!object_box.IsSet()) continue;
    if (!have) {
      box = object_box;
      have = true;
    } else {
      box.Union(object_box);
    }
  }
  return box;
}

std::string SceneDocument::FormatDeterministicSummary() const {
  std::string out;
  char line[512];

  std::snprintf(line, sizeof(line), "layers=%d objects=%d\n",
                static_cast<int>(layers_.size()), static_cast<int>(objects_.size()));
  out += line;

  for (size_t i = 0; i < layers_.size(); ++i) {
    const Layer& layer = layers_[i];
    std::snprintf(line, sizeof(line), "layer[%d] name=%s rgb=%d,%d,%d visible=%d\n",
                  static_cast<int>(i), layer.name.c_str(), layer.color_r, layer.color_g,
                  layer.color_b, layer.visible ? 1 : 0);
    out += line;
  }

  for (size_t i = 0; i < objects_.size(); ++i) {
    const SceneObject& object = objects_[i];
    const char* kind_text = ObjectKindToString(object.kind);
    const std::string uuid_text = UuidToString(object.uuid);
    const char* layer_name = "-";
    const Layer* layer = GetLayer(object.layer_index);
    if (nullptr != layer) layer_name = layer->name.c_str();

    const ON_BoundingBox box = object.GetBoundingBox();
    std::string bbox_text = "unset";
    if (box.IsSet()) {
      bbox_text = PointToText(box.Min()) + "-" + PointToText(box.Max());
    }

    std::snprintf(line, sizeof(line), "object[%d] id=%llu uuid=%s name=%s kind=%s layer=%s bbox=%s\n",
                  static_cast<int>(i), static_cast<unsigned long long>(object.id),
                  uuid_text.c_str(), object.name.empty() ? "-" : object.name.c_str(),
                  nullptr == kind_text ? "Unknown" : kind_text, layer_name,
                  bbox_text.c_str());
    out += line;
  }

  const ON_BoundingBox scene_box = GetSceneBoundingBox();
  if (scene_box.IsSet()) {
    out += "scene_bbox=" + PointToText(scene_box.Min()) + "-" + PointToText(scene_box.Max()) + "\n";
  } else {
    out += "scene_bbox=unset\n";
  }
  return out;
}

uint64_t SceneDocument::InsertDeserializedObject(SceneObject obj) {
  if (0 == obj.id) obj.id = AllocateId();

  // Resolve id collisions deterministically.
  bool collision = true;
  while (collision) {
    collision = false;
    for (size_t i = 0; i < objects_.size(); ++i) {
      if (objects_[i].id == obj.id) {
        collision = true;
        ++obj.id;
        break;
      }
    }
  }

  if (UuidIsNil(obj.uuid)) obj.uuid = MakeDeterministicUuid(obj.id);

  if (obj.layer_index < 0) obj.layer_index = 0;
  if (obj.layer_index >= static_cast<int>(layers_.size())) {
    obj.layer_index = EnsureLayerIndex("Default");
  }

  objects_.push_back(obj);
  if (obj.id >= next_id_) next_id_ = obj.id + 1;
  return obj.id;
}

void SceneDocument::SetLayersForDeserialization(std::vector<Layer> layers) {
  layers_.clear();
  if (layers.empty()) {
    Layer fallback;
    fallback.index = 0;
    fallback.name = "Default";
    fallback.id = MakeDeterministicUuid(Fnv1a64("Default", 0x4F50454E544CULL));
    layers_.push_back(fallback);
    return;
  }
  for (size_t i = 0; i < layers.size(); ++i) {
    Layer layer = layers[i];
    layer.index = static_cast<int>(i);
    if (UuidIsNil(layer.id)) {
      layer.id = MakeDeterministicUuid(Fnv1a64(layer.name, 0x4C41594552ULL + i));
    }
    layers_.push_back(layer);
  }
}

}  // namespace mvp
}  // namespace opentl
