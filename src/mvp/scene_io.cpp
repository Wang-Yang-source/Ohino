/**
 * @file scene_io.cpp
 * @brief OpenTL MVP scene serialization: deterministic local JSON scene format
 *        ("opentl_scene_v1") and real openNURBS .3dm archive read/write.
 *
 * Design goals
 * ------------
 *  1. Self contained: the JSON reader/writer only uses the C++17 standard library
 *     (no third party JSON dependency), the output is byte-for-byte deterministic
 *     and uses a fixed "%.12g" number format so the same scene always produces
 *     the same file.
 *  2. Lossless: every one of the 7 ObjectKind payloads plus the stable 64-bit ID,
 *     the ON_UUID, the name, the layer table and the layer assignment round-trips.
 *  3. Real 3DM: Save3dm()/Load3dm() drive ONX_Model, so the files produced here are
 *     genuine Rhino .3dm archives that any openNURBS based application can open.
 *     Because openNURBS has no "plane" model object, an ON_Plane is persisted as a
 *     trimmed-less ON_PlaneSurface with a [-10,10]^2 extent. The original
 *     ObjectKind is kept as the "opentl_kind" user string and the stable 64-bit ID
 *     as the "opentl_id" user string, so .3dm round-trips are exact as well.
 */

#include "opentl/mvp/rhino_mvp.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace opentl {
namespace mvp {
namespace {

constexpr const char* kSceneFormatTag = "opentl_scene_v1";

// ============================================================================
// 1. Deterministic JSON output helpers
// ============================================================================

/*
 * Fixed precision number formatting. "%.12g" keeps 12 significant decimal
 * digits which is more than enough for exact demo geometry round-trips while
 * staying short and stable (no locale, no exponent churn for small values).
 * JSON has no NaN/Infinity literals, so non finite values are written as 0.
 */
void AppendNumber(std::string& out, double value) {
  if (!std::isfinite(value)) {
    out += '0';
    return;
  }
  if (value == 0.0) {
    // Normalizes -0.0 to 0 so the file is stable.
    out += '0';
    return;
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  out += buffer;
}

void AppendEscaped(std::string& out, const std::string& text) {
  out += '"';
  for (size_t i = 0; i < text.size(); ++i) {
    const unsigned char c = static_cast<unsigned char>(text[i]);
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b";  break;
      case '\f': out += "\\f";  break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:
        if (c < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(c));
          out += buffer;
        } else {
          // Bytes >= 0x20 are copied verbatim so UTF-8 sequences survive.
          out += static_cast<char>(c);
        }
        break;
    }
  }
  out += '"';
}

void AppendInt(std::string& out, long long value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%lld", value);
  out += buffer;
}

void WriteXYZ(std::string& out, double x, double y, double z) {
  out += '[';
  AppendNumber(out, x);
  out += ", ";
  AppendNumber(out, y);
  out += ", ";
  AppendNumber(out, z);
  out += ']';
}

void WritePlane(std::string& out, const ON_Plane& plane) {
  out += "{\"origin\": ";
  WriteXYZ(out, plane.origin.x, plane.origin.y, plane.origin.z);
  out += ", \"xaxis\": ";
  WriteXYZ(out, plane.xaxis.x, plane.xaxis.y, plane.xaxis.z);
  out += ", \"yaxis\": ";
  WriteXYZ(out, plane.yaxis.x, plane.yaxis.y, plane.yaxis.z);
  out += ", \"zaxis\": ";
  WriteXYZ(out, plane.zaxis.x, plane.zaxis.y, plane.zaxis.z);
  out += '}';
}

void WriteNurbsCurve(std::string& out, const ON_NurbsCurve& curve) {
  const int dim = curve.m_dim;
  const int order = curve.m_order;
  const int cv_count = curve.m_cv_count;
  const bool is_rat = (0 != curve.m_is_rat);

  out += "{\"dim\": ";
  AppendInt(out, dim);
  out += ", \"is_rat\": ";
  out += (is_rat ? "true" : "false");
  out += ", \"order\": ";
  AppendInt(out, order);
  out += ", \"cv_count\": ";
  AppendInt(out, cv_count);
  out += ", \"knots\": [";

  const int knot_count = (order + cv_count - 2);
  const int cv_size = dim + (is_rat ? 1 : 0);
  const bool payload_ok =
      (dim >= 1 && order >= 2 && cv_count >= order && nullptr != curve.m_knot && nullptr != curve.m_cv);

  if (payload_ok) {
    for (int i = 0; i < knot_count; ++i) {
      if (i > 0) out += ", ";
      AppendNumber(out, curve.m_knot[i]);
    }
  }
  out += "], \"cvs\": [";
  if (payload_ok) {
    for (int i = 0; i < cv_count; ++i) {
      if (i > 0) out += ", ";
      out += '[';
      for (int j = 0; j < cv_size; ++j) {
        if (j > 0) out += ", ";
        AppendNumber(out, curve.m_cv[i * curve.m_cv_stride + j]);
      }
      out += ']';
    }
  }
  out += "]}";
}

void WritePoint(std::string& out, const ON_3dPoint& point) {
  WriteXYZ(out, point.x, point.y, point.z);
}

// ============================================================================
// 2. Minimal, dependency free JSON DOM + recursive descent parser
// ============================================================================

struct JsonValue {
  enum class Type { Null, Bool, Number, String, Array, Object };

  Type type = Type::Null;
  bool boolean = false;
  double number = 0.0;
  std::string text;
  std::vector<JsonValue> items;
  std::vector<std::pair<std::string, JsonValue> > members;

  const JsonValue* Member(const char* key) const {
    if (Type::Object != type) return nullptr;
    for (size_t i = 0; i < members.size(); ++i) {
      if (members[i].first == key) return &members[i].second;
    }
    return nullptr;
  }
};

/*
 * Tolerant but strict recursive descent JSON parser.
 * Supports objects, arrays, strings (with \" \\ \/ \b \f \n \r \t and \uXXXX
 * escapes), numbers, true/false/null. Object member lookup is linear which is
 * plenty fast for scene files.
 */
class JsonParser {
 public:
  explicit JsonParser(const std::string& source) : source_(source) {}

  bool Parse(JsonValue& out) {
    SkipWhitespace();
    if (!ParseValue(out)) return false;
    SkipWhitespace();
    if (position_ != source_.size()) {
      Fail("trailing characters after top level value");
      return false;
    }
    return true;
  }

  const std::string& error() const { return error_; }

 private:
  void Fail(const char* message) {
    if (error_.empty()) {
      error_ = message;
      error_ += " (offset ";
      char buffer[32];
      std::snprintf(buffer, sizeof(buffer), "%llu",
                    static_cast<unsigned long long>(position_));
      error_ += buffer;
      error_ += ")";
    }
  }

  void SkipWhitespace() {
    while (position_ < source_.size()) {
      const char c = source_[position_];
      if (' ' == c || '\t' == c || '\r' == c || '\n' == c) {
        ++position_;
      } else {
        break;
      }
    }
  }

  bool Consume(char expected) {
    SkipWhitespace();
    if (position_ < source_.size() && source_[position_] == expected) {
      ++position_;
      return true;
    }
    Fail("unexpected character");
    return false;
  }

  bool Literal(const char* text) {
    const size_t length = std::strlen(text);
    if (source_.compare(position_, length, text) != 0) {
      Fail("invalid literal");
      return false;
    }
    position_ += length;
    return true;
  }

  bool ParseValue(JsonValue& out) {
    SkipWhitespace();
    if (position_ >= source_.size()) {
      Fail("unexpected end of input");
      return false;
    }
    const char c = source_[position_];
    switch (c) {
      case '{': return ParseObject(out);
      case '[': return ParseArray(out);
      case '"': {
        out.type = JsonValue::Type::String;
        return ParseString(out.text);
      }
      case 't':
        if (!Literal("true")) return false;
        out.type = JsonValue::Type::Bool;
        out.boolean = true;
        return true;
      case 'f':
        if (!Literal("false")) return false;
        out.type = JsonValue::Type::Bool;
        out.boolean = false;
        return true;
      case 'n':
        if (!Literal("null")) return false;
        out.type = JsonValue::Type::Null;
        return true;
      default:
        if ('-' == c || ('0' <= c && c <= '9')) return ParseNumber(out);
        Fail("unexpected value");
        return false;
    }
  }

  bool ParseObject(JsonValue& out) {
    out.type = JsonValue::Type::Object;
    out.members.clear();
    if (!Consume('{')) return false;
    SkipWhitespace();
    if (position_ < source_.size() && source_[position_] == '}') {
      ++position_;
      return true;
    }
    for (;;) {
      SkipWhitespace();
      std::string key;
      if (!ParseString(key)) return false;
      if (!Consume(':')) return false;
      JsonValue value;
      if (!ParseValue(value)) return false;
      out.members.push_back(std::make_pair(key, value));
      SkipWhitespace();
      if (position_ < source_.size() && source_[position_] == ',') {
        ++position_;
        continue;
      }
      if (position_ < source_.size() && source_[position_] == '}') {
        ++position_;
        return true;
      }
      Fail("unterminated object");
      return false;
    }
  }

  bool ParseArray(JsonValue& out) {
    out.type = JsonValue::Type::Array;
    out.items.clear();
    if (!Consume('[')) return false;
    SkipWhitespace();
    if (position_ < source_.size() && source_[position_] == ']') {
      ++position_;
      return true;
    }
    for (;;) {
      JsonValue value;
      if (!ParseValue(value)) return false;
      out.items.push_back(value);
      SkipWhitespace();
      if (position_ < source_.size() && source_[position_] == ',') {
        ++position_;
        continue;
      }
      if (position_ < source_.size() && source_[position_] == ']') {
        ++position_;
        return true;
      }
      Fail("unterminated array");
      return false;
    }
  }

  bool ParseNumber(JsonValue& out) {
    const size_t start = position_;
    if (position_ < source_.size() && ('-' == source_[position_] || '+' == source_[position_])) {
      ++position_;
    }
    while (position_ < source_.size()) {
      const char c = source_[position_];
      if (('0' <= c && c <= '9') || '.' == c || 'e' == c || 'E' == c || '+' == c || '-' == c) {
        ++position_;
      } else {
        break;
      }
    }
    if (position_ == start) {
      Fail("invalid number");
      return false;
    }
    const std::string token = source_.substr(start, position_ - start);
    char* end = nullptr;
    const double value = std::strtod(token.c_str(), &end);
    if (nullptr == end || end == token.c_str()) {
      Fail("invalid number");
      return false;
    }
    out.type = JsonValue::Type::Number;
    out.number = value;
    return true;
  }

  static void AppendUtf8(std::string& out, unsigned int code_point) {
    if (code_point < 0x80) {
      out += static_cast<char>(code_point);
    } else if (code_point < 0x800) {
      out += static_cast<char>(0xC0 | (code_point >> 6));
      out += static_cast<char>(0x80 | (code_point & 0x3F));
    } else if (code_point < 0x10000) {
      out += static_cast<char>(0xE0 | (code_point >> 12));
      out += static_cast<char>(0x80 | ((code_point >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (code_point & 0x3F));
    } else {
      out += static_cast<char>(0xF0 | (code_point >> 18));
      out += static_cast<char>(0x80 | ((code_point >> 12) & 0x3F));
      out += static_cast<char>(0x80 | ((code_point >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (code_point & 0x3F));
    }
  }

  bool ParseHex4(unsigned int& value) {
    if (position_ + 4 > source_.size()) {
      Fail("truncated \\u escape");
      return false;
    }
    unsigned int result = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = source_[position_++];
      result <<= 4;
      if ('0' <= c && c <= '9') {
        result |= static_cast<unsigned int>(c - '0');
      } else if ('a' <= c && c <= 'f') {
        result |= static_cast<unsigned int>(c - 'a' + 10);
      } else if ('A' <= c && c <= 'F') {
        result |= static_cast<unsigned int>(c - 'A' + 10);
      } else {
        Fail("invalid \\u escape");
        return false;
      }
    }
    value = result;
    return true;
  }

  bool ParseString(std::string& out) {
    out.clear();
    SkipWhitespace();
    if (position_ >= source_.size() || source_[position_] != '"') {
      Fail("expected string");
      return false;
    }
    ++position_;
    while (position_ < source_.size()) {
      const char c = source_[position_++];
      if ('"' == c) return true;
      if ('\\' == c) {
        if (position_ >= source_.size()) {
          Fail("truncated escape");
          return false;
        }
        const char escape = source_[position_++];
        switch (escape) {
          case '"':  out += '"'; break;
          case '\\': out += '\\'; break;
          case '/':  out += '/'; break;
          case 'b':  out += '\b'; break;
          case 'f':  out += '\f'; break;
          case 'n':  out += '\n'; break;
          case 'r':  out += '\r'; break;
          case 't':  out += '\t'; break;
          case 'u': {
            unsigned int code_point = 0;
            if (!ParseHex4(code_point)) return false;
            if (code_point >= 0xD800 && code_point <= 0xDBFF &&
                position_ + 1 < source_.size() && '\\' == source_[position_] &&
                'u' == source_[position_ + 1]) {
              const size_t saved = position_;
              position_ += 2;
              unsigned int low = 0;
              if (!ParseHex4(low)) return false;
              if (low >= 0xDC00 && low <= 0xDFFF) {
                code_point = 0x10000 + ((code_point - 0xD800) << 10) + (low - 0xDC00);
              } else {
                position_ = saved;
              }
            }
            AppendUtf8(out, code_point);
            break;
          }
          default:
            Fail("invalid escape");
            return false;
        }
      } else {
        out += c;
      }
    }
    Fail("unterminated string");
    return false;
  }

  const std::string& source_;
  size_t position_ = 0;
  std::string error_;
};

// ---------------- typed accessors -------------------------------------------

double JsonNumber(const JsonValue* value, double fallback) {
  if (nullptr == value) return fallback;
  if (JsonValue::Type::Number == value->type) return value->number;
  if (JsonValue::Type::Bool == value->type) return value->boolean ? 1.0 : 0.0;
  return fallback;
}

long long JsonInteger(const JsonValue* value, long long fallback) {
  const double as_double = JsonNumber(value, static_cast<double>(fallback));
  if (!std::isfinite(as_double)) return fallback;
  return static_cast<long long>(as_double);
}

std::string JsonString(const JsonValue* value, const std::string& fallback) {
  if (nullptr == value || JsonValue::Type::String != value->type) return fallback;
  return value->text;
}

bool JsonBool(const JsonValue* value, bool fallback) {
  if (nullptr == value) return fallback;
  if (JsonValue::Type::Bool == value->type) return value->boolean;
  if (JsonValue::Type::Number == value->type) return value->number != 0.0;
  return fallback;
}

bool JsonToXyz(const JsonValue* value, double& x, double& y, double& z) {
  if (nullptr == value || JsonValue::Type::Array != value->type || value->items.size() < 3) {
    return false;
  }
  x = JsonNumber(&value->items[0], 0.0);
  y = JsonNumber(&value->items[1], 0.0);
  z = JsonNumber(&value->items[2], 0.0);
  return true;
}

bool JsonToPlane(const JsonValue* value, ON_Plane& plane) {
  if (nullptr == value || JsonValue::Type::Object != value->type) return false;
  ON_3dPoint origin(0.0, 0.0, 0.0);
  ON_3dVector xaxis(1.0, 0.0, 0.0);
  ON_3dVector yaxis(0.0, 1.0, 0.0);
  ON_3dVector zaxis(0.0, 0.0, 1.0);
  JsonToXyz(value->Member("origin"), origin.x, origin.y, origin.z);
  JsonToXyz(value->Member("xaxis"), xaxis.x, xaxis.y, xaxis.z);
  JsonToXyz(value->Member("yaxis"), yaxis.x, yaxis.y, yaxis.z);
  JsonToXyz(value->Member("zaxis"), zaxis.x, zaxis.y, zaxis.z);

  // Restore the frame verbatim so no re-orthonormalization drift is introduced.
  plane.origin = origin;
  plane.xaxis = xaxis;
  plane.yaxis = yaxis;
  plane.zaxis = zaxis;
  plane.UpdateEquation();
  return true;
}

bool JsonToNurbsCurve(const JsonValue* value, ON_NurbsCurve& curve) {
  if (nullptr == value || JsonValue::Type::Object != value->type) return false;

  const int dim = static_cast<int>(JsonInteger(value->Member("dim"), 3));
  const bool is_rat = JsonBool(value->Member("is_rat"), false);
  const int order = static_cast<int>(JsonInteger(value->Member("order"), 0));
  const int cv_count = static_cast<int>(JsonInteger(value->Member("cv_count"), 0));

  const JsonValue* knots = value->Member("knots");
  const JsonValue* cvs = value->Member("cvs");

  if (dim < 1 || order < 2 || cv_count < order) {
    // Degenerate / empty NURBS payload: restore a default constructed curve.
    curve = ON_NurbsCurve();
    return true;
  }

  const int knot_count = order + cv_count - 2;
  const int cv_size = dim + (is_rat ? 1 : 0);
  if (nullptr == knots || JsonValue::Type::Array != knots->type ||
      static_cast<int>(knots->items.size()) != knot_count) {
    return false;
  }
  if (nullptr == cvs || JsonValue::Type::Array != cvs->type ||
      static_cast<int>(cvs->items.size()) != cv_count) {
    return false;
  }

  ON_NurbsCurve rebuilt(dim, is_rat, order, cv_count);
  rebuilt.ReserveKnotCapacity(knot_count);
  for (int i = 0; i < knot_count; ++i) {
    rebuilt.SetKnot(i, JsonNumber(&knots->items[i], 0.0));
  }
  std::vector<double> cv_values(static_cast<size_t>(cv_size), 0.0);
  for (int i = 0; i < cv_count; ++i) {
    const JsonValue& cv = cvs->items[i];
    if (JsonValue::Type::Array != cv.type ||
        static_cast<int>(cv.items.size()) != cv_size) {
      return false;
    }
    for (int j = 0; j < cv_size; ++j) {
      cv_values[static_cast<size_t>(j)] = JsonNumber(&cv.items[j], 0.0);
    }
    rebuilt.SetCV(i, is_rat ? ON::homogeneous_rational : ON::not_rational,
                  &cv_values[0]);
  }
  curve = rebuilt;
  return true;
}

// ============================================================================
// 3. openNURBS string / uuid helpers (deterministic, locale independent)
// ============================================================================

std::string WideToUtf8(const ON_wString& wide) {
  if (wide.IsEmpty()) return std::string();
  ON_String utf8(wide);
  const char* text = static_cast<const char*>(utf8);
  return (nullptr != text) ? std::string(text) : std::string();
}

std::string UuidToSceneString(const ON_UUID& id) {
  char buffer[64];
  std::memset(buffer, 0, sizeof(buffer));
  if (nullptr == ON_UuidToString(id, buffer)) return std::string();
  return std::string(buffer);
}

ON_UUID SceneStringToUuid(const std::string& text) {
  if (text.empty()) return ON_nil_uuid;
  return ON_UuidFromString(text.c_str());
}

bool IsNilUuid(const ON_UUID& id) {
  return 0 == ON_UuidCompare(id, ON_nil_uuid);
}

void SetIoError(std::string* error_out, const std::string& message) {
  if (nullptr != error_out) *error_out = message;
}

bool IsArcLikeKind(ObjectKind kind) {
  return ObjectKind::Circle == kind || ObjectKind::Arc == kind;
}

ObjectKind InferredKindFromGeometry(const ON_Geometry* geometry, bool* payload_ok) {
  if (payload_ok) *payload_ok = true;
  if (nullptr == geometry) {
    if (payload_ok) *payload_ok = false;
    return ObjectKind::Point;
  }
  if (nullptr != ON_Point::Cast(geometry)) return ObjectKind::Point;
  if (nullptr != ON_LineCurve::Cast(geometry)) return ObjectKind::Line;
  if (nullptr != ON_PolylineCurve::Cast(geometry)) return ObjectKind::Polyline;
  if (nullptr != ON_ArcCurve::Cast(geometry)) {
    const ON_ArcCurve* arc_curve = ON_ArcCurve::Cast(geometry);
    return arc_curve->IsCircle() ? ObjectKind::Circle : ObjectKind::Arc;
  }
  if (nullptr != ON_PlaneSurface::Cast(geometry)) return ObjectKind::Plane;
  if (nullptr != ON_NurbsCurve::Cast(geometry)) return ObjectKind::NurbsCurve;
  if (nullptr != ON_Curve::Cast(geometry)) return ObjectKind::NurbsCurve;
  if (payload_ok) *payload_ok = false;
  return ObjectKind::Point;
}

/*
 * Fills the SceneObject payload from a live ON_Geometry. Returns false when the
 * geometry type is not one of the 7 supported ObjectKind values.
 */
bool FillPayloadFromGeometry(const ON_Geometry* geometry, SceneObject& object) {
  bool payload_ok = true;
  const ObjectKind inferred = InferredKindFromGeometry(geometry, &payload_ok);
  if (!payload_ok) return false;
  object.kind = inferred;

  switch (inferred) {
    case ObjectKind::Point: {
      const ON_Point* point = ON_Point::Cast(geometry);
      object.point_val = point->point;
      return true;
    }
    case ObjectKind::Line: {
      const ON_LineCurve* line_curve = ON_LineCurve::Cast(geometry);
      object.line_val = line_curve->m_line;
      return true;
    }
    case ObjectKind::Polyline: {
      const ON_PolylineCurve* polyline_curve = ON_PolylineCurve::Cast(geometry);
      object.polyline_val = polyline_curve->m_pline;
      return true;
    }
    case ObjectKind::Circle: {
      const ON_ArcCurve* arc_curve = ON_ArcCurve::Cast(geometry);
      object.circle_val = arc_curve->m_arc;
      return true;
    }
    case ObjectKind::Arc: {
      const ON_ArcCurve* arc_curve = ON_ArcCurve::Cast(geometry);
      object.arc_val = arc_curve->m_arc;
      return true;
    }
    case ObjectKind::Plane: {
      const ON_PlaneSurface* plane_surface = ON_PlaneSurface::Cast(geometry);
      object.plane_val = plane_surface->m_plane;
      return true;
    }
    case ObjectKind::NurbsCurve: {
      if (const ON_NurbsCurve* nurbs = ON_NurbsCurve::Cast(geometry)) {
        object.nurbs_val = *nurbs;
        return true;
      }
      if (const ON_Curve* generic_curve = ON_Curve::Cast(geometry)) {
        if (generic_curve->GetNurbForm(object.nurbs_val) > 0 && object.nurbs_val.IsValid()) {
          return true;
        }
      }
      return false;
    }
  }
  return false;
}

// ============================================================================
// 4. Scene -> JSON text
// ============================================================================

void WriteJsonLayer(std::string& out, const Layer& layer) {
  out += "    {\"index\": ";
  AppendInt(out, layer.index);
  out += ", \"name\": ";
  AppendEscaped(out, layer.name);
  out += ", \"uuid\": ";
  AppendEscaped(out, UuidToSceneString(layer.id));
  out += ", \"color_r\": ";
  AppendInt(out, layer.color_r);
  out += ", \"color_g\": ";
  AppendInt(out, layer.color_g);
  out += ", \"color_b\": ";
  AppendInt(out, layer.color_b);
  out += ", \"visible\": ";
  out += (layer.visible ? "true" : "false");
  out += '}';
}

void WriteJsonObjectPayload(std::string& out, const SceneObject& object) {
  switch (object.kind) {
    case ObjectKind::Point:
      out += ", \"point\": ";
      WritePoint(out, object.point_val);
      break;
    case ObjectKind::Line:
      out += ", \"line_from\": ";
      WritePoint(out, object.line_val.from);
      out += ", \"line_to\": ";
      WritePoint(out, object.line_val.to);
      break;
    case ObjectKind::Polyline: {
      out += ", \"polyline\": [";
      for (int i = 0; i < object.polyline_val.PointCount(); ++i) {
        if (i > 0) out += ", ";
        WritePoint(out, object.polyline_val[i]);
      }
      out += ']';
      break;
    }
    case ObjectKind::Circle:
      out += ", \"circle_plane\": ";
      WritePlane(out, object.circle_val.plane);
      out += ", \"circle_radius\": ";
      AppendNumber(out, object.circle_val.radius);
      break;
    case ObjectKind::Arc:
      out += ", \"arc_plane\": ";
      WritePlane(out, object.arc_val.plane);
      out += ", \"arc_radius\": ";
      AppendNumber(out, object.arc_val.radius);
      out += ", \"arc_angle\": [";
      AppendNumber(out, object.arc_val.DomainRadians()[0]);
      out += ", ";
      AppendNumber(out, object.arc_val.DomainRadians()[1]);
      out += ']';
      break;
    case ObjectKind::Plane:
      out += ", \"plane\": ";
      WritePlane(out, object.plane_val);
      break;
    case ObjectKind::NurbsCurve:
      out += ", \"nurbs\": ";
      WriteNurbsCurve(out, object.nurbs_val);
      break;
  }
}

void WriteJsonObject(std::string& out, const SceneObject& object) {
  out += "    {\"id\": ";
  AppendInt(out, static_cast<long long>(object.id));
  out += ", \"uuid\": ";
  AppendEscaped(out, UuidToSceneString(object.uuid));
  out += ", \"name\": ";
  AppendEscaped(out, object.name);
  out += ", \"layer_index\": ";
  AppendInt(out, object.layer_index);
  out += ", \"kind\": ";
  AppendEscaped(out, std::string(ObjectKindToString(object.kind)));
  out += ", \"kind_code\": ";
  AppendInt(out, static_cast<int>(object.kind));
  WriteJsonObjectPayload(out, object);
  out += '}';
}

std::string BuildJsonSceneText(const SceneDocument& document) {
  const std::vector<Layer>& layers = document.Layers();
  const std::vector<SceneObject>& objects = document.Objects();

  std::string out;
  out.reserve(1024 + objects.size() * 256);
  out += "{\n";
  out += "  \"format\": \"";
  out += kSceneFormatTag;
  out += "\",\n";

  uint64_t next_id = 1;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i].id >= next_id) next_id = objects[i].id + 1;
  }
  out += "  \"next_id\": ";
  AppendInt(out, static_cast<long long>(next_id));
  out += ",\n";

  out += "  \"layer_count\": ";
  AppendInt(out, static_cast<long long>(layers.size()));
  out += ",\n  \"layers\": [";
  if (!layers.empty()) out += '\n';
  for (size_t i = 0; i < layers.size(); ++i) {
    WriteJsonLayer(out, layers[i]);
    if (i + 1 < layers.size()) out += ',';
    out += '\n';
  }
  out += "  ],\n";

  out += "  \"object_count\": ";
  AppendInt(out, static_cast<long long>(objects.size()));
  out += ",\n  \"objects\": [";
  if (!objects.empty()) out += '\n';
  for (size_t i = 0; i < objects.size(); ++i) {
    WriteJsonObject(out, objects[i]);
    if (i + 1 < objects.size()) out += ',';
    out += '\n';
  }
  out += "  ]\n";
  out += "}\n";
  return out;
}

// ============================================================================
// 5. JSON text -> scene
// ============================================================================

bool ParseJsonLayer(const JsonValue& node, Layer& layer) {
  if (JsonValue::Type::Object != node.type) return false;
  layer.index = static_cast<int>(JsonInteger(node.Member("index"), 0));
  layer.name = JsonString(node.Member("name"), std::string());
  layer.id = SceneStringToUuid(JsonString(node.Member("uuid"), std::string()));
  layer.color_r = static_cast<int>(JsonInteger(node.Member("color_r"), 0));
  layer.color_g = static_cast<int>(JsonInteger(node.Member("color_g"), 0));
  layer.color_b = static_cast<int>(JsonInteger(node.Member("color_b"), 0));
  layer.visible = JsonBool(node.Member("visible"), true);
  return true;
}

bool ParseJsonObject(const JsonValue& node, SceneObject& object) {
  if (JsonValue::Type::Object != node.type) return false;

  object.id = static_cast<uint64_t>(JsonInteger(node.Member("id"), 0));
  object.uuid = SceneStringToUuid(JsonString(node.Member("uuid"), std::string()));
  object.name = JsonString(node.Member("name"), std::string());
  object.layer_index = static_cast<int>(JsonInteger(node.Member("layer_index"), 0));

  const std::string kind_text = JsonString(node.Member("kind"), std::string());
  ObjectKind kind = ObjectKind::Point;
  if (!kind_text.empty()) {
    if (!ObjectKindFromString(kind_text, &kind)) {
      kind = static_cast<ObjectKind>(
          static_cast<int>(JsonInteger(node.Member("kind_code"), 1)));
    }
  } else {
    kind = static_cast<ObjectKind>(
        static_cast<int>(JsonInteger(node.Member("kind_code"), 1)));
  }

  switch (kind) {
    case ObjectKind::Point: {
      ON_3dPoint point = ON_3dPoint::Origin;
      if (!JsonToXyz(node.Member("point"), point.x, point.y, point.z)) return false;
      object.point_val = point;
      break;
    }
    case ObjectKind::Line: {
      ON_3dPoint from = ON_3dPoint::Origin;
      ON_3dPoint to = ON_3dPoint::Origin;
      if (!JsonToXyz(node.Member("line_from"), from.x, from.y, from.z)) return false;
      if (!JsonToXyz(node.Member("line_to"), to.x, to.y, to.z)) return false;
      object.line_val = ON_Line(from, to);
      break;
    }
    case ObjectKind::Polyline: {
      const JsonValue* points = node.Member("polyline");
      if (nullptr == points || JsonValue::Type::Array != points->type) return false;
      ON_Polyline polyline;
      for (size_t i = 0; i < points->items.size(); ++i) {
        ON_3dPoint point = ON_3dPoint::Origin;
        if (!JsonToXyz(&points->items[i], point.x, point.y, point.z)) return false;
        polyline.Append(point);
      }
      object.polyline_val = polyline;
      break;
    }
    case ObjectKind::Circle: {
      ON_Plane plane = ON_Plane::World_xy;
      if (!JsonToPlane(node.Member("circle_plane"), plane)) return false;
      object.circle_val.plane = plane;
      object.circle_val.radius = JsonNumber(node.Member("circle_radius"), 1.0);
      break;
    }
    case ObjectKind::Arc: {
      ON_Plane plane = ON_Plane::World_xy;
      if (!JsonToPlane(node.Member("arc_plane"), plane)) return false;
      const double radius = JsonNumber(node.Member("arc_radius"), 1.0);
      const JsonValue* angle = node.Member("arc_angle");
      double t0 = 0.0;
      double t1 = ON_PI;
      if (nullptr != angle && JsonValue::Type::Array == angle->type &&
          angle->items.size() >= 2) {
        t0 = JsonNumber(&angle->items[0], 0.0);
        t1 = JsonNumber(&angle->items[1], ON_PI);
      }
      ON_Circle circle(plane, radius);
      object.arc_val = ON_Arc(circle, ON_Interval(t0, t1));
      break;
    }
    case ObjectKind::Plane: {
      ON_Plane plane = ON_Plane::World_xy;
      if (!JsonToPlane(node.Member("plane"), plane)) return false;
      object.plane_val = plane;
      break;
    }
    case ObjectKind::NurbsCurve: {
      ON_NurbsCurve curve;
      if (!JsonToNurbsCurve(node.Member("nurbs"), curve)) return false;
      object.nurbs_val = curve;
      break;
    }
    default:
      return false;
  }

  object.kind = kind;
  return true;
}

// ============================================================================
// 6. .3dm helpers
// ============================================================================

std::string ModelErrorLogToString(const ON_wString& wide) {
  return WideToUtf8(wide);
}

/*
 * Layer metadata record stored as an ONX_Model document user string.
 *
 * model.AddLayer() creates a layer with a freshly generated id and always
 * visible, which would make the layer ON_UUID / visibility drift across a
 * .3dm round-trip. The record below is the authoritative layer table used by
 * Load3dm() so the ON_UUID, color and visibility survive exactly, while the
 * .3dm layer table itself still holds the real geometry layer assignment.
 *
 * Format: one record per line, tab separated
 *   index \t name \t uuid \t r \t g \t b \t visible
 */
const wchar_t* kLayerTableKey = L"opentl_layers";

std::string BuildLayerTableRecord(const std::vector<Layer>& layers) {
  std::string out;
  for (size_t i = 0; i < layers.size(); ++i) {
    const Layer& layer = layers[i];
    out += std::to_string(static_cast<int>(i));
    out += '\t';
    out += layer.name;
    out += '\t';
    out += UuidToSceneString(layer.id);
    out += '\t';
    out += std::to_string(layer.color_r);
    out += '\t';
    out += std::to_string(layer.color_g);
    out += '\t';
    out += std::to_string(layer.color_b);
    out += '\t';
    out += (layer.visible ? "1" : "0");
    out += '\n';
  }
  return out;
}

std::vector<std::string> SplitTabFields(const std::string& line) {
  std::vector<std::string> fields;
  std::string current;
  for (size_t i = 0; i < line.size(); ++i) {
    if ('\t' == line[i]) {
      fields.push_back(current);
      current.clear();
    } else {
      current += line[i];
    }
  }
  fields.push_back(current);
  return fields;
}

/*
 * Applies the stored layer records on top of the layers read from the .3dm
 * layer table. Records that do not line up with a parsed layer are ignored.
 */
void ApplyLayerTableRecord(const std::string& record, std::vector<Layer>& layers) {
  size_t start = 0;
  while (start < record.size()) {
    size_t end = record.find('\n', start);
    if (std::string::npos == end) end = record.size();
    const std::string line = record.substr(start, end - start);
    start = end + 1;
    if (line.empty()) continue;

    const std::vector<std::string> fields = SplitTabFields(line);
    if (fields.size() < 7) continue;

    const int index = std::atoi(fields[0].c_str());
    if (index < 0 || index >= static_cast<int>(layers.size())) continue;

    Layer& layer = layers[static_cast<size_t>(index)];
    if (!fields[1].empty()) layer.name = fields[1];
    const ON_UUID stored_id = SceneStringToUuid(fields[2]);
    if (!IsNilUuid(stored_id)) layer.id = stored_id;
    layer.color_r = std::atoi(fields[3].c_str());
    layer.color_g = std::atoi(fields[4].c_str());
    layer.color_b = std::atoi(fields[5].c_str());
    layer.visible = (0 != std::atoi(fields[6].c_str()));
  }
}

/*
 * Deterministic stand-in for a missing user string id: derive it from the
 * object position so repeated loads of the same external .3dm yield the same
 * stable ids.
 */
uint64_t StableFallbackId(unsigned int ordinal) {
  return static_cast<uint64_t>(ordinal) + 1;
}

}  // namespace

// ============================================================================
// 7. SceneDocument JSON serialization
// ============================================================================

bool SceneDocument::SaveJsonScene(const std::string& file_path, std::string* error_out) const {
  if (file_path.empty()) {
    SetIoError(error_out, "SaveJsonScene: empty file path");
    return false;
  }

  const std::string text = BuildJsonSceneText(*this);

  std::FILE* file = std::fopen(file_path.c_str(), "wb");
  if (nullptr == file) {
    SetIoError(error_out, "SaveJsonScene: unable to open '" + file_path + "' for writing");
    return false;
  }
  const size_t written = std::fwrite(text.data(), 1, text.size(), file);
  const int close_status = std::fclose(file);
  if (written != text.size() || 0 != close_status) {
    SetIoError(error_out, "SaveJsonScene: short write to '" + file_path + "'");
    return false;
  }

  if (nullptr != error_out) error_out->clear();
  return true;
}

bool SceneDocument::LoadJsonScene(const std::string& file_path, std::string* error_out) {
  if (file_path.empty()) {
    SetIoError(error_out, "LoadJsonScene: empty file path");
    return false;
  }

  std::FILE* file = std::fopen(file_path.c_str(), "rb");
  if (nullptr == file) {
    SetIoError(error_out, "LoadJsonScene: unable to open '" + file_path + "' for reading");
    return false;
  }

  std::string text;
  char chunk[4096];
  for (;;) {
    const size_t read = std::fread(chunk, 1, sizeof(chunk), file);
    if (read > 0) text.append(chunk, read);
    if (read < sizeof(chunk)) break;
  }
  std::fclose(file);

  JsonValue root;
  JsonParser parser(text);
  if (!parser.Parse(root) || JsonValue::Type::Object != root.type) {
    SetIoError(error_out, "LoadJsonScene: invalid JSON in '" + file_path +
                              "' (" + parser.error() + ")");
    return false;
  }

  const std::string format = JsonString(root.Member("format"), std::string());
  if (format != kSceneFormatTag) {
    SetIoError(error_out, "LoadJsonScene: unexpected format tag '" + format +
                              "' in '" + file_path + "'");
    return false;
  }

  std::vector<Layer> layers;
  const JsonValue* json_layers = root.Member("layers");
  if (nullptr != json_layers && JsonValue::Type::Array == json_layers->type) {
    for (size_t i = 0; i < json_layers->items.size(); ++i) {
      Layer layer;
      if (!ParseJsonLayer(json_layers->items[i], layer)) {
        SetIoError(error_out, "LoadJsonScene: malformed layer entry");
        return false;
      }
      // The array position is authoritative for internal references.
      layer.index = static_cast<int>(i);
      layers.push_back(layer);
    }
  }
  if (layers.empty()) {
    Layer fallback;
    fallback.index = 0;
    fallback.name = "Default";
    layers.push_back(fallback);
  }

  std::vector<SceneObject> objects;
  const JsonValue* json_objects = root.Member("objects");
  if (nullptr != json_objects && JsonValue::Type::Array == json_objects->type) {
    for (size_t i = 0; i < json_objects->items.size(); ++i) {
      SceneObject object;
      if (!ParseJsonObject(json_objects->items[i], object)) {
        SetIoError(error_out, "LoadJsonScene: malformed object entry");
        return false;
      }
      if (object.layer_index < 0 ||
          object.layer_index >= static_cast<int>(layers.size())) {
        object.layer_index = 0;
      }
      if (IsNilUuid(object.uuid)) {
        object.uuid = MakeDeterministicUuid(object.id);
      }
      objects.push_back(object);
    }
  }

  // Deterministic object order for every downstream consumer / summary.
  std::stable_sort(objects.begin(), objects.end(),
                   [](const SceneObject& a, const SceneObject& b) { return a.id < b.id; });

  Clear();
  SetLayersForDeserialization(layers);
  for (size_t i = 0; i < objects.size(); ++i) {
    InsertDeserializedObject(objects[i]);
  }

  if (nullptr != error_out) error_out->clear();
  return true;
}

// ============================================================================
// 8. SceneDocument openNURBS .3dm serialization
// ============================================================================

bool SceneDocument::Save3dm(const std::string& file_path, std::string* error_out) const {
  if (file_path.empty()) {
    SetIoError(error_out, "Save3dm: empty file path");
    return false;
  }

  ONX_Model model;

  // Deterministic archive metadata so repeated runs produce comparable files.
  model.m_sStartSectionComments = "OpenTL MVP scene archive (opentl_scene_v1)";
  model.m_properties.m_Application.m_application_name = L"OpenTL MVP";
  model.m_properties.m_Application.m_application_URL = L"https://github.com/opentl";
  model.m_properties.m_RevisionHistory.NewRevision();

  // ---- layer table ------------------------------------------------------
  // model_layer_index[i] is the .3dm layer index used for layers_[i].
  std::vector<int> model_layer_index(layers_.size(), 0);
  if (layers_.empty()) {
    model_layer_index.clear();
    model.AddLayer(L"Default", ON_Color(0, 0, 0));
  } else {
    for (size_t i = 0; i < layers_.size(); ++i) {
      const Layer& layer = layers_[i];
      ON_wString layer_name(layer.name.c_str());
      if (layer_name.IsEmpty()) {
        layer_name = ON_wString::FormatToString(L"Layer_%u", static_cast<unsigned>(i));
      }
      const ON_Color color(layer.color_r, layer.color_g, layer.color_b);
      int index = model.AddLayer(static_cast<const wchar_t*>(layer_name), color);
      if (index < 0) index = 0;
      model_layer_index[i] = index;
    }
  }

  // Preserve the exact layer ON_UUID / visibility / colors: the .3dm layer table
  // generated by AddLayer() cannot carry them verbatim.
  {
    const ON_wString layer_record(BuildLayerTableRecord(layers_).c_str());
    if (!layer_record.IsEmpty()) {
      model.SetDocumentUserString(kLayerTableKey, static_cast<const wchar_t*>(layer_record));
    }
  }

  // ---- object table -----------------------------------------------------
  for (size_t i = 0; i < objects_.size(); ++i) {
    const SceneObject& object = objects_[i];

    ON_3dmObjectAttributes attributes;
    attributes.m_uuid = IsNilUuid(object.uuid) ? MakeDeterministicUuid(object.id)
                                               : object.uuid;
    attributes.m_name = ON_wString(object.name.c_str());
    if (object.layer_index >= 0 &&
        object.layer_index < static_cast<int>(model_layer_index.size())) {
      attributes.m_layer_index = model_layer_index[static_cast<size_t>(object.layer_index)];
    } else {
      attributes.m_layer_index = 0;
    }

    // Stable 64-bit id + original ObjectKind survive the .3dm round-trip.
    const ON_wString wide_id(std::to_string(object.id).c_str());
    attributes.SetUserString(L"opentl_id", static_cast<const wchar_t*>(wide_id));
    const ON_wString wide_kind(ObjectKindToString(object.kind));
    attributes.SetUserString(L"opentl_kind", static_cast<const wchar_t*>(wide_kind));

    bool added = false;
    switch (object.kind) {
      case ObjectKind::Point: {
        ON_Point point(object.point_val);
        added = !model.AddModelGeometryComponent(&point, &attributes, true).IsEmpty();
        break;
      }
      case ObjectKind::Line: {
        ON_LineCurve line_curve(object.line_val);
        added = !model.AddModelGeometryComponent(&line_curve, &attributes, true).IsEmpty();
        break;
      }
      case ObjectKind::Polyline: {
        ON_PolylineCurve polyline_curve(object.polyline_val);
        added = !model.AddModelGeometryComponent(&polyline_curve, &attributes, true).IsEmpty();
        break;
      }
      case ObjectKind::Circle: {
        ON_ArcCurve arc_curve(object.circle_val);
        added = !model.AddModelGeometryComponent(&arc_curve, &attributes, true).IsEmpty();
        break;
      }
      case ObjectKind::Arc: {
        ON_ArcCurve arc_curve(object.arc_val);
        added = !model.AddModelGeometryComponent(&arc_curve, &attributes, true).IsEmpty();
        break;
      }
      case ObjectKind::Plane: {
        // ON_Plane has no model object of its own: persist a finite
        // ON_PlaneSurface. SetExtents() keeps IsValid() true.
        ON_PlaneSurface plane_surface(object.plane_val);
        plane_surface.SetExtents(0, ON_Interval(-10.0, 10.0), true);
        plane_surface.SetExtents(1, ON_Interval(-10.0, 10.0), true);
        added = !model.AddModelGeometryComponent(&plane_surface, &attributes, true).IsEmpty();
        break;
      }
      case ObjectKind::NurbsCurve: {
        added = !model.AddModelGeometryComponent(&object.nurbs_val, &attributes, true).IsEmpty();
        break;
      }
    }

    if (!added) {
      SetIoError(error_out, "Save3dm: failed to add object id " +
                                std::to_string(object.id) + " to the model");
      return false;
    }
  }

  ON_wString wide_log;
  ON_TextLog error_log(wide_log);
  if (!model.Write(file_path.c_str(), 0, &error_log)) {
    std::string detail = ModelErrorLogToString(wide_log);
    if (detail.empty()) detail = "no details reported by openNURBS";
    SetIoError(error_out, "Save3dm: ONX_Model::Write failed for '" + file_path +
                              "' (" + detail + ")");
    return false;
  }

  if (nullptr != error_out) error_out->clear();
  return true;
}

bool SceneDocument::Load3dm(const std::string& file_path, std::string* error_out) {
  if (file_path.empty()) {
    SetIoError(error_out, "Load3dm: empty file path");
    return false;
  }

  ONX_Model model;
  ON_wString wide_log;
  ON_TextLog error_log(wide_log);
  if (!model.Read(file_path.c_str(), &error_log)) {
    std::string detail = ModelErrorLogToString(wide_log);
    if (detail.empty()) detail = "no details reported by openNURBS";
    SetIoError(error_out, "Load3dm: ONX_Model::Read failed for '" + file_path +
                              "' (" + detail + ")");
    return false;
  }

  // ---- layer table ------------------------------------------------------
  std::vector<Layer> layers;
  std::vector<int> model_layer_to_scene_layer;

  {
    ONX_ModelComponentIterator layer_it(model, ON_ModelComponent::Type::Layer);
    for (const ON_ModelComponent* component = layer_it.FirstComponent();
         nullptr != component; component = layer_it.NextComponent()) {
      const ON_Layer* layer = ON_Layer::Cast(component);
      if (nullptr == layer) continue;

      Layer scene_layer;
      scene_layer.index = static_cast<int>(layers.size());
      scene_layer.name = WideToUtf8(layer->Name());
      if (scene_layer.name.empty()) {
        scene_layer.name = "Layer_" + std::to_string(scene_layer.index);
      }
      scene_layer.id = layer->Id();
      const ON_Color color = layer->Color();
      scene_layer.color_r = color.IsUnset() ? 0 : color.Red();
      scene_layer.color_g = color.IsUnset() ? 0 : color.Green();
      scene_layer.color_b = color.IsUnset() ? 0 : color.Blue();
      scene_layer.visible = layer->IsVisible();

      const int model_index = layer->Index();
      if (model_index >= 0) {
        if (static_cast<int>(model_layer_to_scene_layer.size()) <= model_index) {
          model_layer_to_scene_layer.resize(static_cast<size_t>(model_index) + 1, 0);
        }
        model_layer_to_scene_layer[static_cast<size_t>(model_index)] =
            scene_layer.index;
      }
      layers.push_back(scene_layer);
    }
  }
  if (layers.empty()) {
    Layer fallback;
    fallback.index = 0;
    fallback.name = "Default";
    layers.push_back(fallback);
  }

  // Restore exact layer ids / colors / visibility when this archive was written
  // by Save3dm(). External .3dm files simply keep what the layer table reports.
  {
    ON_wString layer_record;
    if (model.GetDocumentUserString(kLayerTableKey, layer_record)) {
      ApplyLayerTableRecord(WideToUtf8(layer_record), layers);
    }
  }

  // ---- object table -----------------------------------------------------
  std::vector<SceneObject> objects;
  std::vector<uint64_t> used_ids;
  unsigned int ordinal = 0;

  {
    ONX_ModelComponentIterator geometry_it(model, ON_ModelComponent::Type::ModelGeometry);
    for (const ON_ModelComponent* component = geometry_it.FirstComponent();
         nullptr != component; component = geometry_it.NextComponent()) {
      const ON_ModelGeometryComponent* geometry_component =
          ON_ModelGeometryComponent::Cast(component);
      if (nullptr == geometry_component) continue;

      const ON_Geometry* geometry = geometry_component->Geometry(nullptr);
      if (nullptr == geometry) continue;

      SceneObject object;
      if (!FillPayloadFromGeometry(geometry, object)) {
        // Not one of the 7 supported kinds (mesh, brep, ...): skip silently.
        continue;
      }

      const ON_3dmObjectAttributes* attributes = geometry_component->Attributes(nullptr);
      if (nullptr != attributes) {
        object.uuid = attributes->m_uuid;
        object.name = WideToUtf8(attributes->Name());

        const int model_layer_index = attributes->m_layer_index;
        if (model_layer_index >= 0 &&
            model_layer_index < static_cast<int>(model_layer_to_scene_layer.size())) {
          object.layer_index = model_layer_to_scene_layer[static_cast<size_t>(model_layer_index)];
        } else {
          object.layer_index = 0;
        }

        // Stable id from our own user string when present.
        ON_wString wide_value;
        if (attributes->GetUserString(L"opentl_id", wide_value)) {
          const std::string text = WideToUtf8(wide_value);
          if (!text.empty()) {
            char* end = nullptr;
            const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
            if (nullptr != end && end != text.c_str()) {
              object.id = static_cast<uint64_t>(parsed);
            }
          }
        }

        // Original ObjectKind hint, only honored when it is compatible with the
        // geometry we just read (Circle/Arc share ON_ArcCurve).
        if (attributes->GetUserString(L"opentl_kind", wide_value)) {
          ObjectKind hinted = object.kind;
          if (ObjectKindFromString(WideToUtf8(wide_value), &hinted)) {
            const bool compatible =
                (hinted == object.kind) ||
                (IsArcLikeKind(hinted) && IsArcLikeKind(object.kind));
            if (compatible) object.kind = hinted;
          }
        }
      } else {
        object.layer_index = 0;
      }

      // External 3DM files have no opentl_id: assign a deterministic, unused id.
      if (0 == object.id) {
        uint64_t candidate = StableFallbackId(ordinal);
        while (std::find(used_ids.begin(), used_ids.end(), candidate) != used_ids.end()) {
          ++candidate;
        }
        object.id = candidate;
      } else if (std::find(used_ids.begin(), used_ids.end(), object.id) != used_ids.end()) {
        uint64_t candidate = object.id + 1;
        while (std::find(used_ids.begin(), used_ids.end(), candidate) != used_ids.end()) {
          ++candidate;
        }
        object.id = candidate;
      }
      used_ids.push_back(object.id);

      if (IsNilUuid(object.uuid)) {
        object.uuid = MakeDeterministicUuid(object.id);
      }

      objects.push_back(object);
      ++ordinal;
    }
  }

  std::stable_sort(objects.begin(), objects.end(),
                   [](const SceneObject& a, const SceneObject& b) { return a.id < b.id; });

  Clear();
  SetLayersForDeserialization(layers);
  for (size_t i = 0; i < objects.size(); ++i) {
    InsertDeserializedObject(objects[i]);
  }

  if (nullptr != error_out) error_out->clear();
  return true;
}

}  // namespace mvp
}  // namespace opentl
