/**
 * @file tl_gh.cpp
 * @brief Ohino Grasshopper (.gh / .ghx / .ghz) archive reader and parametric DAG solver.
 */

#include "opentl/gh/tl_gh.h"

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>

namespace {

// ---------------------------------------------------------------------------
// Little-endian reader over the inflated GH_Archive byte stream.
// ---------------------------------------------------------------------------
struct BinReader {
  const unsigned char* data = nullptr;
  size_t size = 0;
  size_t pos = 0;
  bool ok = true;

  bool Has(size_t n) const { return ok && pos + n <= size; }

  unsigned char U8() {
    if (!Has(1)) { ok = false; return 0; }
    return data[pos++];
  }
  int32_t I32() {
    if (!Has(4)) { ok = false; return 0; }
    int32_t v;
    std::memcpy(&v, data + pos, 4);
    pos += 4;
    return v;
  }
  int64_t I64() {
    if (!Has(8)) { ok = false; return 0; }
    int64_t v;
    std::memcpy(&v, data + pos, 8);
    pos += 8;
    return v;
  }
  float F32() {
    float v = 0.0f;
    if (Has(4)) { std::memcpy(&v, data + pos, 4); pos += 4; } else ok = false;
    return v;
  }
  double F64() {
    double v = 0.0;
    if (Has(8)) { std::memcpy(&v, data + pos, 8); pos += 8; } else ok = false;
    return v;
  }

  // 7-bit encoded length prefix + UTF-8 payload (.NET BinaryWriter string).
  std::string Str7() {
    uint64_t len = 0;
    int shift = 0;
    while (true) {
      unsigned char b = U8();
      if (!ok) return std::string();
      len |= (uint64_t)(b & 0x7F) << shift;
      if ((b & 0x80) == 0) break;
      shift += 7;
      if (shift > 35) { ok = false; return std::string(); }
    }
    if (!Has((size_t)len)) { ok = false; return std::string(); }
    std::string s((const char*)data + pos, (size_t)len);
    pos += (size_t)len;
    return s;
  }
};

// ---------------------------------------------------------------------------
// GH_Chunk / GH_Item in-memory tree.
// ---------------------------------------------------------------------------
struct GhItem {
  std::string name;
  int index = -1;
  int type = 0;
  bool b = false;
  int64_t i = 0;
  double d = 0.0;
  std::string s;
  std::vector<double> dbls;
  std::vector<unsigned char> bytes;
  ON_3dPoint pt = ON_3dPoint::Origin;
  ON_Interval iv = ON_Interval(0.0, 1.0);
  ON_Line line;
  ON_Plane plane = ON_Plane::World_xy;
  ON_BoundingBox bbox;
  unsigned char rgba[4] = {0, 0, 0, 255};
};

struct GhChunk {
  std::string name;
  int index = -1;
  std::vector<GhItem> items;
  std::vector<GhChunk> children;

  const GhItem* Find(const char* n) const {
    for (const auto& it : items) if (it.name == n) return &it;
    return nullptr;
  }
  const GhChunk* Child(const char* n) const {
    for (const auto& c : children) if (c.name == n) return &c;
    return nullptr;
  }
  std::string Str(const char* n, const std::string& def = std::string()) const {
    const GhItem* it = Find(n);
    if (!it) return def;
    if (it->type == 10 || it->type == 9) return it->s;
    return def;
  }
  bool HasStr(const char* n) const { return Find(n) != nullptr; }
  double Num(const char* n, double def = 0.0) const {
    const GhItem* it = Find(n);
    if (!it) return def;
    if (it->type == 6 || it->type == 5) return it->d != 0.0 ? it->d : (double)it->i;
    if (it->type == 3 || it->type == 4) return (double)it->i;
    if (it->type == 1) return it->b ? 1.0 : 0.0;
    return def;
  }
  bool Bool(const char* n, bool def = false) const {
    const GhItem* it = Find(n);
    if (!it) return def;
    if (it->type == 1) return it->b;
    if (it->type == 3 || it->type == 4) return it->i != 0;
    return def;
  }
};

int32_t ReadI32(BinReader& r) { return r.I32(); }

bool ReadItem(BinReader& r, GhItem& out) {
  out.name = r.Str7();
  out.index = r.I32();
  out.type = r.I32();
  if (!r.ok) return false;
  const int t = out.type;
  switch (t) {
    case 1: out.b = r.U8() != 0; break;
    case 2: out.i = r.U8(); break;
    case 3: out.i = r.I32(); break;
    case 4: out.i = r.I64(); break;
    case 5: out.d = r.F32(); break;
    case 6: out.d = r.F64(); break;
    case 7: { for (int k = 0; k < 16; ++k) r.U8(); break; }
    case 8: out.i = r.I64(); break;
    case 9: {
      unsigned char g[16];
      for (int k = 0; k < 16; ++k) g[k] = r.U8();
      if (!r.ok) return false;
      char buf[40];
      std::snprintf(buf, sizeof(buf), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                    g[3], g[2], g[1], g[0], g[5], g[4], g[7], g[6],
                    g[8], g[9], g[10], g[11], g[12], g[13], g[14], g[15]);
      out.s = buf;
      break;
    }
    case 10: out.s = r.Str7(); break;
    case 20: {
      int32_t n = r.I32();
      if (n < 0 || !r.Has((size_t)n)) return false;
      out.bytes.assign(r.data + r.pos, r.data + r.pos + n);
      r.pos += (size_t)n;
      break;
    }
    case 21: {
      int32_t n = r.I32();
      if (n < 0 || !r.Has((size_t)n * 8)) return false;
      out.dbls.resize((size_t)n);
      for (int32_t k = 0; k < n; ++k) out.dbls[(size_t)k] = r.F64();
      break;
    }
    case 30: case 32: { int32_t a = r.I32(); int32_t b = r.I32(); out.dbls = {(double)a, (double)b}; break; }
    case 31: case 33: { float a = r.F32(); float b = r.F32(); out.dbls = {a, b}; break; }
    case 34: { for (int k = 0; k < 4; ++k) out.dbls.push_back(r.I32()); break; }
    case 35: { for (int k = 0; k < 4; ++k) out.dbls.push_back(r.F32()); break; }
    case 36: { for (int k = 0; k < 4; ++k) out.rgba[k] = r.U8(); break; }
    case 37: { int32_t n = r.I32(); if (n > 0) { if (!r.Has((size_t)n)) return false; r.pos += (size_t)n; } break; }
    case 50: { double x = r.F64(); double y = r.F64(); out.dbls = {x, y}; break; }
    case 51: { double x = r.F64(); double y = r.F64(); double z = r.F64(); out.pt = ON_3dPoint(x, y, z); break; }
    case 52: { double x = r.F64(); double y = r.F64(); double z = r.F64(); double w = r.F64(); out.pt = ON_3dPoint(x, y, z); (void)w; break; }
    case 60: { double a = r.F64(); double b = r.F64(); out.iv = ON_Interval(a, b); break; }
    case 61: { for (int k = 0; k < 4; ++k) out.dbls.push_back(r.F64()); break; }
    case 70: {
      ON_3dPoint a, b;
      a.x = r.F64(); a.y = r.F64(); a.z = r.F64();
      b.x = r.F64(); b.y = r.F64(); b.z = r.F64();
      out.line = ON_Line(a, b);
      break;
    }
    case 71: {
      ON_3dPoint a, b;
      a.x = r.F64(); a.y = r.F64(); a.z = r.F64();
      b.x = r.F64(); b.y = r.F64(); b.z = r.F64();
      out.bbox = ON_BoundingBox(a, b);
      break;
    }
    case 72: {
      ON_3dPoint o, x, y;
      o.x = r.F64(); o.y = r.F64(); o.z = r.F64();
      x.x = r.F64(); x.y = r.F64(); x.z = r.F64();
      y.x = r.F64(); y.y = r.F64(); y.z = r.F64();
      out.plane = ON_Plane(o, x, y);
      break;
    }
    case 80: { int32_t a = r.I32(); int32_t b = r.I32(); int32_t c = r.I32(); out.dbls = {(double)a, (double)b, (double)c}; break; }
    default:
      // Unknown type: cannot advance safely.
      return false;
  }
  return r.ok;
}

bool ReadChunk(BinReader& r, GhChunk& out, int depth) {
  if (depth > 64) return false;
  out.name = r.Str7();
  out.index = r.I32();
  int32_t item_count = r.I32();
  int32_t chunk_count = r.I32();
  if (!r.ok) return false;
  if (item_count < 0 || item_count > 200000) return false;
  if (chunk_count < 0 || chunk_count > 200000) return false;
  out.items.resize((size_t)item_count);
  for (int32_t k = 0; k < item_count; ++k) {
    if (!ReadItem(r, out.items[(size_t)k])) return false;
  }
  out.children.resize((size_t)chunk_count);
  for (int32_t k = 0; k < chunk_count; ++k) {
    if (!ReadChunk(r, out.children[(size_t)k], depth + 1)) return false;
  }
  return true;
}

bool InflateRaw(const std::vector<unsigned char>& in, std::vector<unsigned char>& out) {
  z_stream s;
  std::memset(&s, 0, sizeof(s));
  if (inflateInit2(&s, -15) != Z_OK) return false;
  out.assign(in.size() * 6 + 65536, 0);
  s.next_in = const_cast<Bytef*>(in.data());
  s.avail_in = (uInt)in.size();
  int rc = Z_OK;
  for (;;) {
    if (s.total_out >= out.size()) out.resize(out.size() * 2);
    s.next_out = out.data() + s.total_out;
    s.avail_out = (uInt)(out.size() - s.total_out);
    rc = inflate(&s, Z_NO_FLUSH);
    if (rc == Z_STREAM_END) break;
    if (rc != Z_OK && rc != Z_BUF_ERROR) { inflateEnd(&s); return false; }
    if (rc == Z_BUF_ERROR && s.avail_in == 0) { inflateEnd(&s); return false; }
  }
  out.resize(s.total_out);
  inflateEnd(&s);
  return !out.empty();
}

// ---------------------------------------------------------------------------
// Minimal XML (.ghx) reader: chunk / items / item elements.
// ---------------------------------------------------------------------------
struct XmlTok {
  std::string tag;
  bool closing = false;
  bool self_closing = false;
  std::map<std::string, std::string> attrs;
  std::string text;
};

std::string XmlUnescape(const std::string& s) {
  std::string out;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '&') {
      size_t semi = s.find(';', i);
      if (semi != std::string::npos && semi - i <= 6) {
        std::string ent = s.substr(i + 1, semi - i - 1);
        if (ent == "amp") out += '&';
        else if (ent == "lt") out += '<';
        else if (ent == "gt") out += '>';
        else if (ent == "quot") out += '"';
        else if (ent == "apos") out += '\'';
        else out += s.substr(i, semi - i + 1);
        i = semi;
        continue;
      }
    }
    out += s[i];
  }
  return out;
}

std::map<std::string, std::string> XmlAttrs(const std::string& s) {
  std::map<std::string, std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
    size_t k0 = i;
    while (i < s.size() && s[i] != '=' && !std::isspace((unsigned char)s[i]) && s[i] != '/' && s[i] != '>') ++i;
    std::string key = s.substr(k0, i - k0);
    while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
    if (i < s.size() && s[i] == '=') {
      ++i;
      while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
      if (i < s.size() && (s[i] == '"' || s[i] == '\'')) {
        char q = s[i++];
        size_t v0 = i;
        while (i < s.size() && s[i] != q) ++i;
        out[key] = XmlUnescape(s.substr(v0, i - v0));
        if (i < s.size()) ++i;
      }
    }
    if (key.empty()) break;
  }
  return out;
}

bool ConvertXmlItem(const std::string& type_name, int type_code, const std::string& value, GhItem& out) {
  out.type = type_code;
  out.s = XmlUnescape(value);
  switch (type_code) {
    case 1: out.b = (value == "True" || value == "true" || value == "1"); break;
    case 3: case 4: out.i = (int64_t)std::atoll(value.c_str()); break;
    case 5: case 6: out.d = std::atof(value.c_str()); break;
    case 10: break;
    case 50: case 51: case 52: {
      std::vector<double> nums;
      std::string cur;
      for (char c : value) {
        if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') cur += c;
        else if (!cur.empty()) { nums.push_back(std::atof(cur.c_str())); cur.clear(); }
      }
      if (!cur.empty()) nums.push_back(std::atof(cur.c_str()));
      if (nums.size() >= 3) out.pt = ON_3dPoint(nums[0], nums[1], nums[2]);
      else if (nums.size() == 2) out.dbls = nums;
      break;
    }
    case 60: {
      std::vector<double> nums;
      std::string cur;
      for (char c : value) {
        if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') cur += c;
        else if (!cur.empty()) { nums.push_back(std::atof(cur.c_str())); cur.clear(); }
      }
      if (!cur.empty()) nums.push_back(std::atof(cur.c_str()));
      if (nums.size() >= 2) out.iv = ON_Interval(nums[0], nums[1]);
      break;
    }
    case 72: {
      std::vector<double> nums;
      std::string cur;
      for (char c : value) {
        if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') cur += c;
        else if (!cur.empty()) { nums.push_back(std::atof(cur.c_str())); cur.clear(); }
      }
      if (!cur.empty()) nums.push_back(std::atof(cur.c_str()));
      if (nums.size() >= 9) out.plane = ON_Plane(ON_3dPoint(nums[0], nums[1], nums[2]),
                                                 ON_3dVector(nums[3], nums[4], nums[5]),
                                                 ON_3dVector(nums[6], nums[7], nums[8]));
      break;
    }
    default: break;
  }
  (void)type_name;
  return true;
}

// Parse the .ghx XML tree into a synthetic Root chunk whose first child is "Definition".
bool ParseGhx(const std::string& xml, GhChunk& root) {
  struct Frame { GhChunk* chunk; };
  std::vector<Frame> stack;
  root = GhChunk();
  root.name = "Root";
  stack.push_back({&root});

  size_t i = 0;
  int items_depth = 0;
  GhChunk* item_host = nullptr;

  while (i < xml.size()) {
    size_t lt = xml.find('<', i);
    if (lt == std::string::npos) break;
    // skip comments / declarations
    if (xml.compare(lt, 4, "<!--") == 0) {
      size_t e = xml.find("-->", lt);
      i = (e == std::string::npos) ? xml.size() : e + 3;
      continue;
    }
    if (xml.compare(lt, 9, "<![CDATA[") == 0) {
      size_t e = xml.find("]]>", lt);
      i = (e == std::string::npos) ? xml.size() : e + 3;
      continue;
    }
    size_t gt = xml.find('>', lt);
    if (gt == std::string::npos) break;
    std::string inner = xml.substr(lt + 1, gt - lt - 1);
    i = gt + 1;
    if (!inner.empty() && inner[0] == '?') continue;
    if (!inner.empty() && inner[0] == '!') continue;

    bool closing = false;
    bool self_closing = false;
    if (!inner.empty() && inner[0] == '/') { closing = true; inner.erase(0, 1); }
    if (!inner.empty() && inner.back() == '/') { self_closing = true; inner.pop_back(); }

    size_t sp = inner.find_first_of(" \t\r\n");
    std::string tag = (sp == std::string::npos) ? inner : inner.substr(0, sp);
    std::string attr_str = (sp == std::string::npos) ? std::string() : inner.substr(sp);
    std::map<std::string, std::string> attrs = XmlAttrs(attr_str);

    if (tag == "chunk" || tag == "Archive") {
      if (closing) {
        if (stack.size() > 1) stack.pop_back();
      } else {
        GhChunk c;
        c.name = attrs.count("name") ? attrs["name"] : (tag == "Archive" ? "Root" : "chunk");
        if (attrs.count("index")) c.index = std::atoi(attrs["index"].c_str());
        GhChunk* parent = stack.back().chunk;
        parent->children.push_back(c);
        if (!self_closing) stack.push_back({&parent->children.back()});
      }
    } else if (tag == "items") {
      if (!closing && !self_closing && !stack.empty()) items_depth++;
      else if (closing && items_depth > 0) items_depth--;
      item_host = stack.empty() ? nullptr : stack.back().chunk;
    } else if (tag == "item") {
      if (closing) continue;
      GhChunk* host = stack.empty() ? nullptr : stack.back().chunk;
      if (!host) continue;
      std::string name = attrs.count("name") ? attrs["name"] : std::string();
      int index = attrs.count("index") ? std::atoi(attrs["index"].c_str()) : -1;
      int code = 10;
      if (attrs.count("type_code")) code = std::atoi(attrs["type_code"].c_str());
      std::string value;
      if (self_closing) {
        value = attrs.count("value") ? attrs["value"] : std::string();
      } else {
        size_t end = xml.find("</item>", i);
        if (end == std::string::npos) { i = xml.size(); }
        else { value = xml.substr(i, end - i); i = end + 7; }
      }
      GhItem it;
      it.name = name;
      it.index = index;
      ConvertXmlItem(attrs.count("type_name") ? attrs["type_name"] : std::string(), code, value, it);
      host->items.push_back(it);
    } else if (tag == "item" && closing) {
      continue;
    }
  }
  (void)items_depth;
  (void)item_host;
  return !root.children.empty();
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------
std::shared_ptr<ON_NurbsCurve> NurbsThrough(const std::vector<ON_3dPoint>& pts, int degree) {
  if (pts.size() < 2) return nullptr;
  int deg = std::max(1, std::min<int>(degree, (int)pts.size() - 1));
  auto c = std::make_shared<ON_NurbsCurve>();
  if (!c->Create(3, false, deg + 1, (int)pts.size())) return nullptr;
  for (int k = 0; k < (int)pts.size(); ++k) c->SetCV(k, pts[(size_t)k]);
  c->MakeClampedUniformKnotVector(1.0);
  return c;
}



// GH "internalised" Rhino geometry is stored as an ON_BinaryArchive byte buffer
// inside the PersistentData item (GH_IO type 20). Decode it with openNURBS.
bool DecodeOnData(const std::vector<unsigned char>& bytes, TL_GhValue& out) {
  if (bytes.size() < 32) return false;
  ON_Read3dmBufferArchive ar(bytes.size(), bytes.data(), true, 50, 20200101);
  ON_Object* obj = nullptr;
  if (!ar.ReadObject(&obj) || !obj) return false;
  bool ok = false;
  if (const ON_NurbsCurve* nc = ON_NurbsCurve::Cast(obj)) { out = TL_GhValue::MakeCurve(*nc); ok = true; }
  else if (const ON_NurbsSurface* ns = ON_NurbsSurface::Cast(obj)) { out = TL_GhValue::MakeSurface(*ns); ok = true; }
  else if (const ON_Mesh* m = ON_Mesh::Cast(obj)) { out = TL_GhValue::MakeMesh(*m); ok = true; }
  else if (const ON_Brep* b = ON_Brep::Cast(obj)) {
    ON_Mesh mesh;
    if (TL_ExtractBrepMesh(*b, mesh, false)) { out = TL_GhValue::MakeMesh(mesh); ok = true; }
  }
  else if (const ON_Curve* c = ON_Curve::Cast(obj)) {
    ON_NurbsCurve nc;
    if (c->GetNurbForm(nc)) { out = TL_GhValue::MakeCurve(nc); ok = true; }
  }
  delete obj;
  return ok;
}

double CurveLengthApprox(const ON_NurbsCurve& c) {
  ON_Interval d = c.Domain();
  const int n = 64;
  double len = 0.0;
  ON_3dPoint prev = c.PointAt(d.ParameterAt(0.0));
  for (int i = 1; i <= n; ++i) {
    ON_3dPoint p = c.PointAt(d.ParameterAt((double)i / (double)n));
    len += (p - prev).Length();
    prev = p;
  }
  return len;
}

std::shared_ptr<ON_Mesh> BoxMesh(const ON_BoundingBox& bbox) {
  if (!bbox.IsValid()) return nullptr;
  auto m = std::make_shared<ON_Mesh>();
  const ON_3dPoint& a = bbox.m_min;
  const ON_3dPoint& b = bbox.m_max;
  const ON_3dPoint v[8] = {
    ON_3dPoint(a.x, a.y, a.z), ON_3dPoint(b.x, a.y, a.z),
    ON_3dPoint(b.x, b.y, a.z), ON_3dPoint(a.x, b.y, a.z),
    ON_3dPoint(a.x, a.y, b.z), ON_3dPoint(b.x, a.y, b.z),
    ON_3dPoint(b.x, b.y, b.z), ON_3dPoint(a.x, b.y, b.z)};
  for (const auto& p : v) m->m_V.Append(ON_3fPoint(p));
  const int f[12][4] = {{0,3,2,1},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
  for (int q = 0; q < 6; ++q) {
    ON_MeshFace mf;
    mf.vi[0] = f[q][0]; mf.vi[1] = f[q][1]; mf.vi[2] = f[q][2]; mf.vi[3] = f[q][3];
    m->m_F.Append(mf);
  }
  m->ComputeVertexNormals();
  return m;
}

bool CircumCircle(const ON_3dPoint& p0, const ON_3dPoint& p1, const ON_3dPoint& p2, ON_Circle& out) {
  ON_3dVector u = p1 - p0;
  ON_3dVector v = p2 - p0;
  ON_3dVector n = ON_CrossProduct(u, v);
  double n2 = n.LengthSquared();
  if (n2 < 1e-20) return false;
  ON_3dVector c = (ON_CrossProduct(n, u) * v.LengthSquared() + ON_CrossProduct(v, n) * u.LengthSquared()) / (2.0 * n2);
  ON_3dPoint center = p0 + c;
  ON_3dVector normal = n;
  normal.Unitize();
  out = ON_Circle(ON_Plane(center, normal), (p0 - center).Length());
  return true;
}

// ---------------------------------------------------------------------------
// Small expression evaluator for GH "Expression" / "Format" components.
// Supports + - * / ^ ( ), unary minus, variables x/y/z, numbers.
// ---------------------------------------------------------------------------
struct ExprEval {
  const std::string* src = nullptr;
  size_t p = 0;
  double vars[3] = {0.0, 0.0, 0.0};
  bool ok = true;

  void Skip() { while (p < src->size() && std::isspace((unsigned char)(*src)[p])) ++p; }
  double Parse() {
    double v = ParseExpr();
    return v;
  }
  double ParseExpr() {
    double v = ParseTerm();
    while (ok) {
      Skip();
      if (p < src->size() && ((*src)[p] == '+' || (*src)[p] == '-')) {
        char op = (*src)[p++];
        double r = ParseTerm();
        v = (op == '+') ? v + r : v - r;
      } else break;
    }
    return v;
  }
  double ParseTerm() {
    double v = ParseUnary();
    while (ok) {
      Skip();
      if (p < src->size() && ((*src)[p] == '*' || (*src)[p] == '/')) {
        char op = (*src)[p++];
        double r = ParseUnary();
        v = (op == '*') ? v * r : (std::fabs(r) < 1e-300 ? 0.0 : v / r);
      } else break;
    }
    return v;
  }
  double ParseUnary() {
    Skip();
    if (p < src->size() && ((*src)[p] == '-' || (*src)[p] == '+')) {
      char op = (*src)[p++];
      double v = ParseUnary();
      return op == '-' ? -v : v;
    }
    return ParsePow();
  }
  double ParsePow() {
    double v = ParseAtom();
    Skip();
    if (p < src->size() && (*src)[p] == '^') {
      ++p;
      double e = ParseUnary();
      return std::pow(v, e);
    }
    return v;
  }
  double ParseAtom() {
    Skip();
    if (p >= src->size()) { ok = false; return 0.0; }
    char ch = (*src)[p];
    if (ch == '(') {
      ++p;
      double v = ParseExpr();
      Skip();
      if (p < src->size() && (*src)[p] == ')') ++p; else ok = false;
      return v;
    }
    if (std::isalpha((unsigned char)ch)) {
      size_t s0 = p;
      while (p < src->size() && (std::isalnum((unsigned char)(*src)[p]) || (*src)[p] == '_')) ++p;
      std::string id = src->substr(s0, p - s0);
      if (id == "x" || id == "X") return vars[0];
      if (id == "y" || id == "Y") return vars[1];
      if (id == "z" || id == "Z") return vars[2];
      if (id == "pi" || id == "PI") return 3.14159265358979323846;
      // simple single-arg functions
      Skip();
      if (p < src->size() && (*src)[p] == '(') {
        ++p;
        double a = ParseExpr();
        Skip();
        if (p < src->size() && (*src)[p] == ')') ++p; else ok = false;
        if (id == "sin") return std::sin(a);
        if (id == "cos") return std::cos(a);
        if (id == "tan") return std::tan(a);
        if (id == "sqrt") return a < 0 ? 0.0 : std::sqrt(a);
        if (id == "abs") return std::fabs(a);
        if (id == "exp") return std::exp(a);
        if (id == "log") return a <= 0 ? 0.0 : std::log(a);
        if (id == "floor") return std::floor(a);
        if (id == "ceil") return std::ceil(a);
        if (id == "round") return std::round(a);
      }
      ok = false;
      return 0.0;
    }
    size_t s0 = p;
    while (p < src->size() && (std::isdigit((unsigned char)(*src)[p]) || (*src)[p] == '.')) ++p;
    if (s0 == p) { ok = false; return 0.0; }
    return std::atof(src->substr(s0, p - s0).c_str());
  }
};

double EvalExpr(const std::string& expr, double x, double y, double z, bool* ok = nullptr) {
  ExprEval e;
  e.src = &expr;
  e.vars[0] = x; e.vars[1] = y; e.vars[2] = z;
  double v = e.Parse();
  e.Skip();
  if (e.p != expr.size()) e.ok = false;
  if (ok) *ok = e.ok;
  return v;
}

std::vector<std::string> SplitList(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == '\n' || c == '\r') { if (!cur.empty()) out.push_back(cur); cur.clear(); }
    else cur += c;
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

// Component name normalization for GH1 nicknames/names.
bool NameIs(const std::string& n, const char* a, const char* b = nullptr, const char* c = nullptr) {
  if (n == a) return true;
  if (b && n == b) return true;
  if (c && n == c) return true;
  return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// TL_GhValue
// ---------------------------------------------------------------------------
TL_GhValue TL_GhValue::MakeNumber(double v) { TL_GhValue g; g.kind = TL_GhValueKind::Number; g.num_val = v; return g; }
TL_GhValue TL_GhValue::MakeBool(bool b) { TL_GhValue g; g.kind = TL_GhValueKind::Bool; g.bool_val = b; g.num_val = b ? 1.0 : 0.0; return g; }
TL_GhValue TL_GhValue::MakeString(const std::string& s) { TL_GhValue g; g.kind = TL_GhValueKind::String; g.str_val = s; return g; }
TL_GhValue TL_GhValue::MakePoint(const ON_3dPoint& p) { TL_GhValue g; g.kind = TL_GhValueKind::Point; g.pt_val = p; return g; }
TL_GhValue TL_GhValue::MakeVector(const ON_3dVector& v) { TL_GhValue g; g.kind = TL_GhValueKind::Vector; g.vec_val = v; return g; }
TL_GhValue TL_GhValue::MakeInterval(double a, double b) { TL_GhValue g; g.kind = TL_GhValueKind::Interval; g.interval_val = ON_Interval(a, b); return g; }
TL_GhValue TL_GhValue::MakeLine(const ON_Line& l) { TL_GhValue g; g.kind = TL_GhValueKind::Line; g.line_val = l; return g; }
TL_GhValue TL_GhValue::MakePlane(const ON_Plane& pl) { TL_GhValue g; g.kind = TL_GhValueKind::Plane; g.plane_val = pl; return g; }
TL_GhValue TL_GhValue::MakeCircle(const ON_Circle& c) { TL_GhValue g; g.kind = TL_GhValueKind::Circle; g.circle_val = c; return g; }
TL_GhValue TL_GhValue::MakeArc(const ON_Arc& a) { TL_GhValue g; g.kind = TL_GhValueKind::Arc; g.arc_val = a; return g; }
TL_GhValue TL_GhValue::MakeBox(const ON_BoundingBox& b) { TL_GhValue g; g.kind = TL_GhValueKind::Box; g.box_val = b; return g; }
TL_GhValue TL_GhValue::MakeColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
  TL_GhValue v; v.kind = TL_GhValueKind::Color; v.color_rgba[0] = r; v.color_rgba[1] = g; v.color_rgba[2] = b; v.color_rgba[3] = a; return v;
}
TL_GhValue TL_GhValue::MakeCurve(const ON_NurbsCurve& c) {
  TL_GhValue v; v.kind = TL_GhValueKind::Curve;
  v.curve_val = std::make_shared<ON_NurbsCurve>(c);
  return v;
}
TL_GhValue TL_GhValue::MakeSurface(const ON_NurbsSurface& s) {
  TL_GhValue v; v.kind = TL_GhValueKind::Surface;
  v.surface_val = std::make_shared<ON_NurbsSurface>(s);
  return v;
}
TL_GhValue TL_GhValue::MakeMesh(const ON_Mesh& m) {
  TL_GhValue v; v.kind = TL_GhValueKind::Mesh;
  v.mesh_val = std::make_shared<ON_Mesh>(m);
  return v;
}

double TL_GhValue::AsNumber(double fallback) const {
  switch (kind) {
    case TL_GhValueKind::Number: return num_val;
    case TL_GhValueKind::Bool: return bool_val ? 1.0 : 0.0;
    case TL_GhValueKind::Point: return pt_val.x;
    case TL_GhValueKind::Vector: return vec_val.x;
    case TL_GhValueKind::Interval: return interval_val.m_t[0];
    case TL_GhValueKind::String: return std::atof(str_val.c_str());
    default: return fallback;
  }
}

ON_3dPoint TL_GhValue::AsPoint(const ON_3dPoint& fallback) const {
  if (kind == TL_GhValueKind::Point) return pt_val;
  if (kind == TL_GhValueKind::Vector) return ON_3dPoint(vec_val.x, vec_val.y, vec_val.z);
  return fallback;
}

ON_3dVector TL_GhValue::AsVector(const ON_3dVector& fallback) const {
  if (kind == TL_GhValueKind::Vector) return vec_val;
  if (kind == TL_GhValueKind::Point) return pt_val - ON_3dPoint::Origin;
  return fallback;
}

std::string TL_GhValue::ToDisplayString() const {
  char buf[256];
  switch (kind) {
    case TL_GhValueKind::Null: return "null";
    case TL_GhValueKind::Bool: return bool_val ? "True" : "False";
    case TL_GhValueKind::Number: {
      std::snprintf(buf, sizeof(buf), "%.6g", num_val);
      return buf;
    }
    case TL_GhValueKind::String: return str_val;
    case TL_GhValueKind::Point: {
      std::snprintf(buf, sizeof(buf), "{%.4g, %.4g, %.4g}", pt_val.x, pt_val.y, pt_val.z);
      return buf;
    }
    case TL_GhValueKind::Vector: {
      std::snprintf(buf, sizeof(buf), "{% .4g, % .4g, % .4g}", vec_val.x, vec_val.y, vec_val.z);
      return buf;
    }
    case TL_GhValueKind::Interval: {
      std::snprintf(buf, sizeof(buf), "[%.4g, %.4g]", interval_val.m_t[0], interval_val.m_t[1]);
      return buf;
    }
    case TL_GhValueKind::Line: {
      std::snprintf(buf, sizeof(buf), "{%.4g,%.4g,%.4g}->{%.4g,%.4g,%.4g}",
                    line_val.from.x, line_val.from.y, line_val.from.z,
                    line_val.to.x, line_val.to.y, line_val.to.z);
      return buf;
    }
    case TL_GhValueKind::Plane: {
      std::snprintf(buf, sizeof(buf), "Plane(%.4g,%.4g,%.4g)", plane_val.origin.x, plane_val.origin.y, plane_val.origin.z);
      return buf;
    }
    case TL_GhValueKind::Circle: {
      std::snprintf(buf, sizeof(buf), "Circle(r=%.4g)", circle_val.radius);
      return buf;
    }
    case TL_GhValueKind::Arc: return "Arc";
    case TL_GhValueKind::Box: return "BoundingBox";
    case TL_GhValueKind::Color: {
      std::snprintf(buf, sizeof(buf), "ARGB(%d,%d,%d,%d)", color_rgba[3], color_rgba[0], color_rgba[1], color_rgba[2]);
      return buf;
    }
    case TL_GhValueKind::Curve: return "Curve";
    case TL_GhValueKind::Surface: return "Surface";
    case TL_GhValueKind::Mesh: return "Mesh";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// TL_GhDocument
// ---------------------------------------------------------------------------
void TL_GhDocument::Clear() {
  file_path_.clear();
  document_name_.clear();
  archive_version_ = "0.2.2";
  nodes_.clear();
  wires_.clear();
  topo_order_.clear();
}

bool TL_GhDocument::LoadFile(const std::string& file_path, std::string* error_out) {
  Clear();
  file_path_ = file_path;

  std::vector<unsigned char> raw;
  {
    FILE* f = std::fopen(file_path.c_str(), "rb");
    if (!f) {
      if (error_out) *error_out = "cannot open file: " + file_path;
      return false;
    }
    std::fseek(f, 0, SEEK_END);
    long len = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (len <= 0) { std::fclose(f); if (error_out) *error_out = "empty file"; return false; }
    raw.resize((size_t)len);
    size_t got = std::fread(raw.data(), 1, (size_t)len, f);
    std::fclose(f);
    if (got != (size_t)len) { if (error_out) *error_out = "short read"; return false; }
  }

  GhChunk root;
  const bool is_zip = raw.size() > 4 && raw[0] == 'P' && raw[1] == 'K';
  const bool is_xml = raw.size() > 1 && (raw[0] == '<' || (raw.size() > 5 && std::memcmp(raw.data(), "<?xml", 5) == 0));

  if (is_zip) {
    if (error_out) *error_out = ".ghz (ZIP container) is not supported by the built-in reader; extract the .gh inside first";
    return false;
  }

  if (is_xml) {
    std::string xml((const char*)raw.data(), raw.size());
    if (!ParseGhx(xml, root)) { if (error_out) *error_out = "malformed .ghx XML"; return false; }
  } else {
    std::vector<unsigned char> inflated;
    if (!InflateRaw(raw, inflated)) {
      if (error_out) *error_out = "GH_Archive deflate decompression failed (raw wbits=-15)";
      return false;
    }
    BinReader r;
    r.data = inflated.data();
    r.size = inflated.size();
    if (!ReadChunk(r, root, 0)) {
      if (error_out) *error_out = "GH_Chunk tree parse failed at byte " + std::to_string(r.pos);
      return false;
    }
  }

  // Root -> Definition -> DefinitionObjects
  const GhChunk* def = nullptr;
  for (const auto& c : root.children) if (c.name == "Definition") def = &c;
  if (!def && root.name == "Definition") def = &root;
  if (!def) { if (error_out) *error_out = "no Definition chunk"; return false; }
  const GhChunk* objects = nullptr;
  for (const auto& c : def->children) if (c.name == "DefinitionObjects") objects = &c;
  if (!objects) { if (error_out) *error_out = "no DefinitionObjects chunk"; return false; }

  if (def->HasStr("Name")) document_name_ = def->Str("Name");
  if (objects->Find("ObjectCount")) document_name_ = document_name_;

  nodes_.clear();
  int ordinal = 0;
  for (const auto& obj : objects->children) {
    if (obj.name != "Object") continue;
    TL_GhNode node;
    node.index = ordinal++;
    node.component_guid = obj.Str("GUID");
    node.name = obj.Str("Name");

    const GhChunk* cont = obj.Child("Container");
    if (cont) {
      node.instance_guid = cont->Str("InstanceGuid");
      if (!cont->HasStr("InstanceGuid")) node.instance_guid = node.component_guid + "#" + std::to_string(node.index);
      node.nickname = cont->Str("NickName", node.name);
      node.description = cont->Str("Description");
      node.hidden = cont->Bool("Hidden", false);
      if (cont->HasStr("Text") && (node.name == "Panel" || node.name == "Scribble")) node.panel_text = cont->Str("Text");
      if (cont->HasStr("UserText") && node.name == "Panel") node.panel_text = cont->Str("UserText");
      if (cont->HasStr("Expression")) node.expression = cont->Str("Expression");
      if (cont->HasStr("SwatchColor")) {
        const GhItem* it = cont->Find("SwatchColor");
        if (it && it->type == 36) std::memcpy(node.swatch_rgba, it->rgba, 4);
      }
      if (cont->HasStr("ID")) {
        for (const auto& it : cont->items) {
          if (it.name == "ID" && (it.type == 9 || it.type == 10)) node.group_member_guids.push_back(it.s);
        }
      }
      const GhChunk* slider = cont->Child("Slider");
      if (slider) {
        node.is_slider = true;
        node.slider.min_val = slider->Num("Min", 0.0);
        node.slider.max_val = slider->Num("Max", 10.0);
        node.slider.value = slider->Num("Value", 1.0);
        node.slider.digits = (int)slider->Num("Digits", 2.0);
        node.slider.is_integer = slider->Bool("Integer", false) || (slider->Num("Interval", 0.0) > 0.0);
      }
    }
    if (obj.HasStr("InstanceGuid") && node.instance_guid.empty()) node.instance_guid = obj.Str("InstanceGuid");

    if (node.name == "Group") node.is_group = true;
    if (node.name == "Scribble") node.is_scribble = true;
    if (node.name == "Panel") node.is_panel = true;
    if (node.name == "Number Slider" || node.name == "Control Knob" || node.name == "Digit Scroller") node.is_slider = true;

    const GhChunk* attrs = cont ? cont->Child("Attributes") : obj.Child("Attributes");
    if (!attrs) attrs = obj.Child("Attributes");
    if (attrs) {
      if (attrs->HasStr("Bounds")) {
        const GhItem* it = attrs->Find("Bounds");
        if (it && it->dbls.size() >= 4) {
          node.bounds.x = (float)it->dbls[0];
          node.bounds.y = (float)it->dbls[1];
          node.bounds.w = (float)it->dbls[2];
          node.bounds.h = (float)it->dbls[3];
        }
      }
      if (attrs->HasStr("Pivot")) {
        const GhItem* it = attrs->Find("Pivot");
        if (it && it->dbls.size() >= 2) {
          node.pivot_x = (float)it->dbls[0];
          node.pivot_y = (float)it->dbls[1];
        }
      }
    }

    // Ports: fixed components use param_input/param_output; variable-parameter
    // components (Subtraction, Addition, Expression, ...) use ParameterData ->
    // InputParam/OutputParam. Both layouts occur in official files.
    auto collect_port = [&](const GhChunk& p, bool is_output) {
      TL_GhPort port;
      port.instance_guid = p.Str("InstanceGuid");
      port.name = p.Str("Name");
      port.nickname = p.Str("NickName", port.name);
      port.description = p.Str("Description");
      const GhChunk* pattr = p.Child("Attributes");
      const GhChunk* src = pattr ? pattr : &p;
      if (src->HasStr("Bounds")) {
        const GhItem* it = src->Find("Bounds");
        if (it && it->dbls.size() >= 4) {
          port.bounds.x = (float)it->dbls[0];
          port.bounds.y = (float)it->dbls[1];
          port.bounds.w = (float)it->dbls[2];
          port.bounds.h = (float)it->dbls[3];
        }
      }
      if (src->HasStr("Pivot")) {
        const GhItem* it = src->Find("Pivot");
        if (it && it->dbls.size() >= 2) {
          port.pivot_x = (float)it->dbls[0];
          port.pivot_y = (float)it->dbls[1];
        }
      }
      std::vector<std::pair<int, std::string>> sources;
      for (const auto& it : p.items) {
        if (it.name == "Source" && (it.type == 9 || it.type == 10)) sources.emplace_back(it.index, it.s);
      }
      std::sort(sources.begin(), sources.end(),
                [](const std::pair<int, std::string>& a, const std::pair<int, std::string>& b) { return a.first < b.first; });
      for (const auto& s : sources) port.source_guids.push_back(s.second);

      const GhChunk* pd = p.Child("PersistentData");
      if (pd) {
        for (const auto& branch : pd->children) {
          if (branch.name != "Branch") continue;
          for (const auto& item : branch.children) {
            if (item.name != "Item") continue;
            TL_GhValue v;
            bool have = false;
            for (const auto& it : item.items) {
              if (it.name == "Coordinate" || it.name == "Point" || it.name == "Location" || it.type == 51) {
                v = TL_GhValue::MakePoint(it.pt); have = true; break;
              }
              if (it.type == 6 || it.type == 5 || it.type == 3 || it.type == 4 || it.name == "number") {
                v = TL_GhValue::MakeNumber(it.type == 3 || it.type == 4 ? (double)it.i : it.d);
                have = true; break;
              }
              if (it.type == 20) {
                if (DecodeOnData(it.bytes, v)) { have = true; break; }
                continue;
              }
              if (it.type == 1 || it.name == "boolean") { v = TL_GhValue::MakeBool(it.b); have = true; break; }
              if (it.type == 60) { v = TL_GhValue::MakeInterval(it.iv.m_t[0], it.iv.m_t[1]); have = true; break; }
              if (it.type == 72) { v = TL_GhValue::MakePlane(it.plane); have = true; break; }
              if (it.type == 70) { v = TL_GhValue::MakeLine(it.line); have = true; break; }
              if (it.type == 10 && it.name != "TypeName") { v = TL_GhValue::MakeString(it.s); have = true; break; }
              if (it.type == 36) {
                v = TL_GhValue::MakeColor(it.rgba[1], it.rgba[2], it.rgba[3], it.rgba[0]);
                have = true; break;
              }
            }
            if (have) port.persistent_data.push_back(v);
          }
        }
      }

      if (is_output) node.outputs.push_back(port);
      else node.inputs.push_back(port);
    };

    // Ports live under Container (GH1 layout); .ghx files sometimes place them
    // directly under the Object chunk, so both are scanned.
    const GhChunk* port_host = cont ? cont : &obj;
    for (const auto& c : port_host->children) {
      if (c.name == "param_input" || c.name == "InputParam") collect_port(c, false);
      else if (c.name == "param_output" || c.name == "OutputParam") collect_port(c, true);
      else if (c.name == "ParameterData") {
        for (const auto& p : c.children) {
          if (p.name == "InputParam") collect_port(p, false);
          else if (p.name == "OutputParam") collect_port(p, true);
        }
      }
    }

    // Standalone parameter nodes (Point / Curve / Number / Panel ...) store their
    // payload in a PersistentData chunk directly under the Object or Container.
    const GhChunk* obj_pd = obj.Child("PersistentData");
    if (!obj_pd && cont) obj_pd = cont->Child("PersistentData");
    if (obj_pd) {
      for (const auto& branch : obj_pd->children) {
        if (branch.name != "Branch") continue;
        for (const auto& item : branch.children) {
          if (item.name != "Item") continue;
          for (const auto& it : item.items) {
            if (it.type == 51 || it.name == "Coordinate") { node.persistent_data.push_back(TL_GhValue::MakePoint(it.pt)); break; }
            if (it.type == 6) { node.persistent_data.push_back(TL_GhValue::MakeNumber(it.d)); break; }
            if (it.type == 3 || it.type == 4) { node.persistent_data.push_back(TL_GhValue::MakeNumber((double)it.i)); break; }
            if (it.type == 20) {
              TL_GhValue dv;
              if (DecodeOnData(it.bytes, dv)) { node.persistent_data.push_back(dv); break; }
              continue;
            }
            if (it.type == 60) { node.persistent_data.push_back(TL_GhValue::MakeInterval(it.iv.m_t[0], it.iv.m_t[1])); break; }
            if (it.type == 72) { node.persistent_data.push_back(TL_GhValue::MakePlane(it.plane)); break; }
            if (it.type == 70) { node.persistent_data.push_back(TL_GhValue::MakeLine(it.line)); break; }
            if (it.type == 10 && it.name != "TypeName" && it.name != "Path") {
              node.persistent_data.push_back(TL_GhValue::MakeString(it.s));
              break;
            }
          }
        }
      }
    }
    if (node.is_slider && node.persistent_data.empty()) {
      node.persistent_data.push_back(TL_GhValue::MakeNumber(node.slider.value));
    }

    nodes_.push_back(std::move(node));
  }

  if (nodes_.empty()) { if (error_out) *error_out = "definition contains no objects"; return false; }

  RebuildWiresAndTopology();
  return Evaluate(error_out);
}

void TL_GhDocument::RebuildWiresAndTopology() {
  wires_.clear();
  std::unordered_map<std::string, std::pair<int, int>> producer;
  for (int i = 0; i < (int)nodes_.size(); ++i) {
    if (!nodes_[(size_t)i].instance_guid.empty()) producer[nodes_[(size_t)i].instance_guid] = {i, 0};
    for (int p = 0; p < (int)nodes_[(size_t)i].outputs.size(); ++p) {
      const std::string& g = nodes_[(size_t)i].outputs[(size_t)p].instance_guid;
      if (!g.empty()) producer[g] = {i, p};
    }
  }

  for (int ni = 0; ni < (int)nodes_.size(); ++ni) {
    TL_GhNode& node = nodes_[(size_t)ni];
    for (int pi = 0; pi < (int)node.inputs.size(); ++pi) {
      TL_GhPort& port = node.inputs[(size_t)pi];
      for (const std::string& src : port.source_guids) {
        auto it = producer.find(src);
        if (it == producer.end()) continue;
        TL_GhWire w;
        w.from_node = it->second.first;
        w.from_output = it->second.second;
        w.to_node = ni;
        w.to_input = pi;
        const TL_GhNode& src_node = nodes_[(size_t)w.from_node];
        w.x0 = src_node.pivot_x;
        w.y0 = src_node.pivot_y;
        w.x1 = port.pivot_x;
        w.y1 = port.pivot_y;
        wires_.push_back(w);
      }
    }
  }

  // Kahn topological sort.
  std::vector<int> indeg(nodes_.size(), 0);
  std::vector<std::vector<int>> succ(nodes_.size());
  for (const auto& w : wires_) {
    if (w.from_node < 0 || w.to_node < 0) continue;
    succ[(size_t)w.from_node].push_back(w.to_node);
    indeg[(size_t)w.to_node]++;
  }
  std::vector<int> queue;
  for (int i = 0; i < (int)nodes_.size(); ++i) if (indeg[(size_t)i] == 0) queue.push_back(i);
  topo_order_.clear();
  for (size_t q = 0; q < queue.size(); ++q) {
    int n = queue[q];
    topo_order_.push_back(n);
    for (int s : succ[(size_t)n]) {
      if (--indeg[(size_t)s] == 0) queue.push_back(s);
    }
  }
  // Cycle / disconnected leftovers still get evaluated once, in index order.
  if (topo_order_.size() != nodes_.size()) {
    std::vector<char> seen(nodes_.size(), 0);
    for (int n : topo_order_) seen[(size_t)n] = 1;
    for (int i = 0; i < (int)nodes_.size(); ++i) if (!seen[(size_t)i]) topo_order_.push_back(i);
  }
}

namespace {

const TL_GhValue* FirstValue(const std::vector<TL_GhValue>& v) {
  return v.empty() ? nullptr : &v.front();
}

}  // namespace

void TL_GhDocument::EvaluateSingleNode(TL_GhNode& node) {
  // Resolve inputs: wire data first, then persistent data.
  std::vector<std::vector<TL_GhValue>> in(node.inputs.size());
  for (size_t pi = 0; pi < node.inputs.size(); ++pi) {
    TL_GhPort& port = node.inputs[pi];
    for (const std::string& src : port.source_guids) {
      int si = -1;
      int sp = -1;
      for (int i = 0; i < (int)nodes_.size(); ++i) {
        if (nodes_[(size_t)i].instance_guid == src) { si = i; sp = 0; break; }
        for (int p = 0; p < (int)nodes_[(size_t)i].outputs.size(); ++p) {
          if (nodes_[(size_t)i].outputs[(size_t)p].instance_guid == src) { si = i; sp = p; break; }
        }
        if (si >= 0) break;
      }
      if (si < 0) continue;
      if (sp >= 0 && sp < (int)nodes_[(size_t)si].outputs.size()) {
        for (const auto& v : nodes_[(size_t)si].outputs[(size_t)sp].runtime_data) in[pi].push_back(v);
      }
      if (in[pi].empty()) {
        for (const auto& v : nodes_[(size_t)si].runtime_data) in[pi].push_back(v);
      }
    }
    if (in[pi].empty()) in[pi] = port.persistent_data;
  }

  auto num_in = [&](size_t i, double def) -> double {
    if (i < in.size() && !in[i].empty()) return in[i][0].AsNumber(def);
    return def;
  };
  auto pt_in = [&](size_t i, const ON_3dPoint& def) -> ON_3dPoint {
    if (i < in.size() && !in[i].empty()) return in[i][0].AsPoint(def);
    return def;
  };
  auto list_pts = [&](size_t i) {
    std::vector<ON_3dPoint> pts;
    if (i < in.size()) for (const auto& v : in[i]) {
      if (v.kind == TL_GhValueKind::Point || v.kind == TL_GhValueKind::Vector) pts.push_back(v.AsPoint());
    }
    return pts;
  };
  auto list_curves = [&](size_t i) {
    std::vector<std::shared_ptr<ON_NurbsCurve>> out;
    if (i < in.size()) for (const auto& v : in[i]) {
      if (v.kind == TL_GhValueKind::Curve && v.curve_val) out.push_back(v.curve_val);
      else if (v.kind == TL_GhValueKind::Line) {
        auto c = NurbsThrough({v.line_val.from, v.line_val.to}, 1);
        if (c) out.push_back(c);
      } else if (v.kind == TL_GhValueKind::Circle || v.kind == TL_GhValueKind::Arc) {
        ON_NurbsCurve nc;
        const ON_Circle& ci = (v.kind == TL_GhValueKind::Circle) ? v.circle_val : v.arc_val;
        if (v.kind == TL_GhValueKind::Circle) {
          if (ci.GetNurbForm(nc)) out.push_back(std::make_shared<ON_NurbsCurve>(nc));
        } else {
          if (v.arc_val.GetNurbForm(nc)) out.push_back(std::make_shared<ON_NurbsCurve>(nc));
        }
      }
    }
    return out;
  };
  auto set_out = [&](int idx, std::vector<TL_GhValue> vals) {
    if (idx >= 0 && idx < (int)node.outputs.size()) node.outputs[(size_t)idx].runtime_data = std::move(vals);
  };
  auto set_all = [&](const std::vector<TL_GhValue>& vals) {
    node.runtime_data = vals;
    for (auto& o : node.outputs) o.runtime_data = vals;
  };

  const std::string& nm = node.name;

  // --- Parameters & plain pass-throughs -----------------------------------
  if (node.is_slider) {
    TL_GhValue v = TL_GhValue::MakeNumber(node.slider.value);
    if (node.slider.is_integer) v.num_val = std::round(node.slider.value);
    set_all({v});
    return;
  }
  if (nm == "Number" || nm == "Digit Scroller") {
    double v = node.persistent_data.empty() ? num_in(0, 0.0) : node.persistent_data[0].AsNumber(0.0);
    if (!node.persistent_data.empty() && !in.empty() && !in[0].empty()) v = num_in(0, v);
    set_all({TL_GhValue::MakeNumber(v)});
    return;
  }
  if (nm == "Boolean Toggle" || nm == "Boolean") {
    bool b = node.persistent_data.empty() ? (num_in(0, 0.0) != 0.0) : node.persistent_data[0].bool_val;
    set_all({TL_GhValue::MakeBool(b)});
    return;
  }
  if (nm == "Panel" || nm == "Scribble") {
    if (nm == "Panel") {
      std::vector<TL_GhValue> vals = in.empty() ? std::vector<TL_GhValue>() : in[0];
      if (vals.empty()) {
        for (const auto& v : node.persistent_data) node.panel_text += v.ToDisplayString() + "\n";
      } else {
        node.panel_text.clear();
        for (const auto& v : vals) node.panel_text += v.ToDisplayString() + "\n";
      }
      set_all(vals);
    }
    return;
  }
  if (nm == "Point" || nm == "Pt" || nm == "Point Parameter") {
    std::vector<TL_GhValue> vals = node.persistent_data;
    if (vals.empty()) vals = in.empty() ? std::vector<TL_GhValue>() : in[0];
    set_all(vals);
    return;
  }
  if (nm == "Curve" || nm == "Crv" || nm == "Geometry" || nm == "Custom Preview" || nm == "Preview") {
    std::vector<TL_GhValue> vals;
    for (const auto& b : in) for (const auto& v : b) vals.push_back(v);
    if (vals.empty()) vals = node.persistent_data;
    set_all(vals);
    return;
  }
  if (nm == "Colour Swatch" || nm == "Color Swatch") {
    set_all({TL_GhValue::MakeColor(node.swatch_rgba[0], node.swatch_rgba[1], node.swatch_rgba[2], node.swatch_rgba[3])});
    return;
  }
  if (nm == "Group" || nm == "Scribble") { return; }

  // --- Math ---------------------------------------------------------------
  if (nm == "Addition" || nm == "Add") {
    double acc = 0.0;
    for (size_t i = 0; i < in.size(); ++i) for (const auto& v : in[i]) acc += v.AsNumber(0.0);
    set_all({TL_GhValue::MakeNumber(acc)});
    return;
  }
  if (nm == "Subtraction" || nm == "Subtract") {
    double a = num_in(0, 0.0);
    double b = num_in(1, 0.0);
    set_all({TL_GhValue::MakeNumber(a - b)});
    return;
  }
  if (nm == "Multiplication" || nm == "Multiply") {
    double acc = 1.0;
    bool any = false;
    for (size_t i = 0; i < in.size(); ++i) for (const auto& v : in[i]) { acc *= v.AsNumber(0.0); any = true; }
    set_all({TL_GhValue::MakeNumber(any ? acc : 0.0)});
    return;
  }
  if (nm == "Division" || nm == "Divide") {
    double a = num_in(0, 0.0);
    double b = num_in(1, 1.0);
    set_all({TL_GhValue::MakeNumber(std::fabs(b) < 1e-300 ? 0.0 : a / b)});
    return;
  }
  if (nm == "Average" || nm == "Mean") {
    double acc = 0.0;
    size_t n = 0;
    for (size_t i = 0; i < in.size(); ++i) for (const auto& v : in[i]) { acc += v.AsNumber(0.0); ++n; }
    set_all({TL_GhValue::MakeNumber(n ? acc / (double)n : 0.0)});
    return;
  }
  if (nm == "Range") {
    ON_Interval dom(0.0, 1.0);
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Interval) dom = in[0][0].interval_val;
    int steps = (int)std::lround(num_in(1, 10.0));
    if (steps < 1) steps = 1;
    if (steps > 100000) steps = 100000;
    std::vector<TL_GhValue> vals;
    vals.reserve((size_t)steps + 1);
    for (int i = 0; i <= steps; ++i) {
      double t = (double)i / (double)steps;
      vals.push_back(TL_GhValue::MakeNumber(dom.ParameterAt(t)));
    }
    set_all(vals);
    return;
  }
  if (nm == "Bounds" || nm == "Bounding") {
    double lo = 1e300, hi = -1e300;
    for (size_t i = 0; i < in.size(); ++i) for (const auto& v : in[i]) {
      double d = v.AsNumber(0.0);
      lo = std::min(lo, d);
      hi = std::max(hi, d);
    }
    if (lo > hi) { lo = 0.0; hi = 0.0; }
    set_all({TL_GhValue::MakeInterval(lo, hi)});
    return;
  }
  if (nm == "Remap Numbers" || nm == "Remap") {
    ON_Interval s(0.0, 1.0), t(0.0, 1.0);
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Interval) s = in[1][0].interval_val;
    if (in.size() > 2 && !in[2].empty() && in[2][0].kind == TL_GhValueKind::Interval) t = in[2][0].interval_val;
    std::vector<TL_GhValue> vals;
    if (!in.empty()) for (const auto& v : in[0]) {
      double d = v.AsNumber(0.0);
      double u = (std::fabs(s.Length()) < 1e-300) ? 0.0 : (d - s.m_t[0]) / s.Length();
      vals.push_back(TL_GhValue::MakeNumber(t.m_t[0] + u * t.Length()));
    }
    set_all(vals);
    return;
  }
  if (nm == "Expression") {
    std::string expr = node.expression.empty() ? node.panel_text : node.expression;
    if (expr.empty()) expr = "x";
    std::vector<TL_GhValue> vals;
    size_t n = 1;
    for (const auto& b : in) n = std::max(n, b.size());
    for (size_t i = 0; i < n; ++i) {
      double x = (in.size() > 0 && i < in[0].size()) ? in[0][i].AsNumber(0.0) : 0.0;
      double y = (in.size() > 1 && i < in[1].size()) ? in[1][i].AsNumber(0.0) : 0.0;
      double z = (in.size() > 2 && i < in[2].size()) ? in[2][i].AsNumber(0.0) : 0.0;
      bool ok = false;
      double r = EvalExpr(expr, x, y, z, &ok);
      vals.push_back(TL_GhValue::MakeNumber(ok ? r : x));
    }
    set_all(vals);
    return;
  }
  if (nm == "Format") {
    std::string fmt = "{0:0.00}";
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::String) fmt = in[0][0].str_val;
    std::vector<TL_GhValue> vals;
    size_t start = (in.empty() || in[0].empty() || in[0][0].kind != TL_GhValueKind::String) ? 0 : 1;
    for (size_t b = start; b < in.size(); ++b) {
      for (const auto& v : in[b]) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%.4g", v.AsNumber(0.0));
        vals.push_back(TL_GhValue::MakeString(buf));
      }
    }
    set_all(vals);
    return;
  }

  // --- Points & vectors ---------------------------------------------------
  if (nm == "Construct Point" || nm == "Pt") {
    double x = num_in(0, 0.0), y = num_in(1, 0.0), z = num_in(2, 0.0);
    set_all({TL_GhValue::MakePoint(ON_3dPoint(x, y, z))});
    return;
  }
  if (nm == "Deconstruct" || nm == "Deconstruct Point") {
    ON_3dPoint p = pt_in(0, ON_3dPoint::Origin);
    set_out(0, {TL_GhValue::MakeNumber(p.x)});
    set_out(1, {TL_GhValue::MakeNumber(p.y)});
    set_out(2, {TL_GhValue::MakeNumber(p.z)});
    return;
  }
  if (nm == "Vector XYZ" || nm == "Vector") {
    double x = num_in(0, 0.0), y = num_in(1, 0.0), z = num_in(2, 0.0);
    set_all({TL_GhValue::MakeVector(ON_3dVector(x, y, z))});
    return;
  }
  if (nm == "Distance") {
    ON_3dPoint a = pt_in(0, ON_3dPoint::Origin);
    ON_3dPoint b = pt_in(1, ON_3dPoint::Origin);
    set_all({TL_GhValue::MakeNumber((b - a).Length())});
    return;
  }
  if (nm == "XY Plane") {
    ON_3dPoint o = pt_in(0, ON_3dPoint::Origin);
    set_all({TL_GhValue::MakePlane(ON_Plane(o, ON_3dVector::XAxis, ON_3dVector::YAxis))});
    return;
  }
  if (nm == "Plane Offset") {
    ON_Plane pl = ON_Plane::World_xy;
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Plane) pl = in[0][0].plane_val;
    double d = num_in(1, 0.0);
    pl.origin += pl.zaxis * d;
    set_all({TL_GhValue::MakePlane(pl)});
    return;
  }

  // --- Curves -------------------------------------------------------------
  if (nm == "Line") {
    ON_3dPoint a = pt_in(0, ON_3dPoint::Origin);
    ON_3dPoint b = pt_in(1, ON_3dPoint(1, 0, 0));
    set_all({TL_GhValue::MakeLine(ON_Line(a, b))});
    return;
  }
  if (nm == "Line SDL") {
    ON_3dPoint s = pt_in(0, ON_3dPoint::Origin);
    ON_3dVector d = ON_3dVector::XAxis;
    if (in.size() > 1 && !in[1].empty()) d = in[1][0].AsVector(ON_3dVector::XAxis);
    double L = num_in(2, 1.0);
    if (d.Length() > 1e-12) d.Unitize();
    set_all({TL_GhValue::MakeLine(ON_Line(s, s + d * L))});
    return;
  }
  if (nm == "Circle") {
    ON_Plane pl = ON_Plane::World_xy;
    double r = 1.0;
    if (!in.empty() && !in[0].empty()) {
      if (in[0][0].kind == TL_GhValueKind::Plane) pl = in[0][0].plane_val;
      else pl = ON_Plane(in[0][0].AsPoint(), ON_3dVector::XAxis, ON_3dVector::YAxis);
    }
    if (in.size() > 1) r = std::fabs(num_in(1, 1.0));
    set_all({TL_GhValue::MakeCircle(ON_Circle(pl, r))});
    return;
  }
  if (nm == "Circle 3Pt" || nm == "Circle 3 Point") {
    std::vector<ON_3dPoint> pts;
    for (size_t i = 0; i < 3; ++i) pts.push_back(pt_in(i, ON_3dPoint::Origin));
    ON_Circle ci;
    if (CircumCircle(pts[0], pts[1], pts[2], ci)) set_all({TL_GhValue::MakeCircle(ci)});
    return;
  }
  if (nm == "Arc" || nm == "Arc SED" || nm == "Arc 3Pt") {
    std::vector<ON_3dPoint> pts;
    for (size_t i = 0; i < 3; ++i) pts.push_back(pt_in(i, ON_3dPoint(i, 0, 0)));
    ON_Arc a(pts[0], pts[1], pts[2]);
    if (a.IsValid()) set_all({TL_GhValue::MakeArc(a)});
    return;
  }
  if (nm == "Nurbs Curve" || nm == "Interpolate data" || nm == "Interpolate" ||
      nm == "Polyline" || nm == "Curve") {
    std::vector<ON_3dPoint> pts = list_pts(0);
    if (pts.size() < 2) { set_all({}); return; }
    int deg = 3;
    if (nm == "Nurbs Curve") deg = (int)std::lround(num_in(1, 3.0));
    else if (nm == "Interpolate data" || nm == "Interpolate") deg = (int)std::lround(num_in(1, 3.0));
    else if (nm == "Polyline") deg = 1;
    auto c = NurbsThrough(pts, deg);
    if (c) set_all({TL_GhValue::MakeCurve(*c)});
    return;
  }
  if (nm == "Divide Curve") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    int n = std::max(1, (int)std::lround(num_in(1, 10.0)));
    std::vector<TL_GhValue> pts, tans, params;
    for (const auto& c : curves) {
      if (!c) continue;
      ON_Interval dom = c->Domain();
      for (int i = 0; i <= n; ++i) {
        double t = dom.ParameterAt((double)i / (double)n);
        ON_3dPoint p;
        ON_3dVector d;
        if (c->Ev1Der(t, p, d)) {
          pts.push_back(TL_GhValue::MakePoint(p));
          if (d.Length() > 1e-12) d.Unitize();
          tans.push_back(TL_GhValue::MakeVector(d));
          params.push_back(TL_GhValue::MakeNumber(t));
        }
      }
    }
    set_out(0, pts);
    set_out(1, tans);
    set_out(2, params);
    return;
  }
  if (nm == "Curve Closest Point" || nm == "Crv CP") {
    ON_3dPoint p = pt_in(0, ON_3dPoint::Origin);
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(1);
    if (!curves.empty() && curves[0]) {
      double t = 0.0;
      if (TL_GetCurveClosestPoint(*curves[0], p, &t)) {
        ON_3dPoint q = curves[0]->PointAt(t);
        set_out(0, {TL_GhValue::MakePoint(q)});
        set_out(1, {TL_GhValue::MakeNumber(t)});
        set_out(2, {TL_GhValue::MakeNumber((q - p).Length())});
        return;
      }
    }
    set_all({});
    return;
  }
  if (nm == "Curve Length" || nm == "Length") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    std::vector<TL_GhValue> vals;
    for (const auto& c : curves) if (c) vals.push_back(TL_GhValue::MakeNumber(CurveLengthApprox(*c)));
    set_all(vals);
    return;
  }
  if (nm == "Bounding Box") {
    ON_BoundingBox bb;
    std::vector<ON_3dPoint> pts = list_pts(0);
    for (const auto& p : pts) bb.Set(p, true);
    for (const auto& c : list_curves(0)) if (c) bb.Union(c->BoundingBox());
    if (bb.IsValid()) set_all({TL_GhValue::MakeBox(bb)});
    return;
  }
  if (nm == "Box Corners") {
    ON_BoundingBox bb;
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Box) bb = in[0][0].box_val;
    std::vector<ON_3dPoint> pts;
    ON_3dPoint corners[8];
    if (bb.GetCorners(corners)) for (int i = 0; i < 8; ++i) pts.push_back(corners[i]);
    std::vector<TL_GhValue> vals;
    for (const auto& p : pts) vals.push_back(TL_GhValue::MakePoint(p));
    set_all(vals);
    return;
  }

  // --- Surfaces & meshes --------------------------------------------------
  if (nm == "Loft") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    if (curves.size() >= 2) {
      ON_SimpleArray<const ON_Curve*> arr;
      for (const auto& c : curves) if (c) arr.Append(c.get());
      ON_NurbsSurface* s = TL_LoftCurves(arr, nullptr);
      if (s) {
        TL_GhValue v = TL_GhValue::MakeSurface(*s);
        delete s;
        set_all({v});
        return;
      }
    }
    set_all({});
    return;
  }
  if (nm == "Sweep1" || nm == "Sweep 1 Rail") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> rail = list_curves(0);
    std::vector<std::shared_ptr<ON_NurbsCurve>> prof = list_curves(1);
    if (!rail.empty() && !prof.empty() && rail[0] && prof[0]) {
      ON_NurbsSurface* s = TL_Sweep1Rail(*rail[0], *prof[0], 32, nullptr);
      if (s) {
        TL_GhValue v = TL_GhValue::MakeSurface(*s);
        delete s;
        set_all({v});
        return;
      }
    }
    set_all({});
    return;
  }
  if (nm == "SubD Box" || nm == "SubDBox") {
    ON_BoundingBox bb;
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Box) bb = in[0][0].box_val;
    else {
      ON_3dPoint a = pt_in(0, ON_3dPoint(-1, -1, 0));
      ON_3dPoint b = pt_in(1, ON_3dPoint(1, 1, 2));
      bb.Set(a, true);
      bb.Set(b, true);
    }
    if (bb.IsValid()) {
      if (auto m = BoxMesh(bb)) set_all({TL_GhValue::MakeMesh(*m)});
    }
    return;
  }
  if (nm == "Mesh Plane" || nm == "Mesh") {
    ON_Plane pl = ON_Plane::World_xy;
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Plane) pl = in[0][0].plane_val;
    double sx = num_in(1, 10.0), sy = num_in(2, 10.0);
    ON_3dPoint a = pl.PointAt(-sx * 0.5, -sy * 0.5);
    ON_3dPoint b = pl.PointAt(sx * 0.5, sy * 0.5);
    ON_BoundingBox bb;
    bb.Set(a, true);
    bb.Set(b, true);
    // thin slab so it renders as a solid
    ON_BoundingBox slab(bb.m_min, ON_3dPoint(bb.m_max.x, bb.m_max.y, bb.m_min.z + std::max(0.01, sx * 0.002)));
    if (auto m = BoxMesh(slab)) set_all({TL_GhValue::MakeMesh(*m)});
    return;
  }

  // Default: propagate all input values to all outputs so preview chains survive.
  {
    std::vector<TL_GhValue> vals;
    for (const auto& b : in) for (const auto& v : b) vals.push_back(v);
    if (vals.empty()) vals = node.persistent_data;
    set_all(vals);
  }
}

bool TL_GhDocument::Evaluate(std::string* error_out) {
  (void)error_out;
  if (topo_order_.size() != nodes_.size()) RebuildWiresAndTopology();
  for (int idx : topo_order_) {
    if (idx >= 0 && idx < (int)nodes_.size()) EvaluateSingleNode(nodes_[(size_t)idx]);
  }
  return true;
}

bool TL_GhDocument::SetSliderValue(const std::string& name_or_nickname, double value) {
  for (auto& n : nodes_) {
    if (!n.is_slider) continue;
    if (n.nickname == name_or_nickname || n.name == name_or_nickname) {
      n.slider.value = std::max(n.slider.min_val, std::min(n.slider.max_val, value));
      if (n.is_slider && !n.persistent_data.empty()) n.persistent_data[0] = TL_GhValue::MakeNumber(n.slider.value);
      return Evaluate();
    }
  }
  return false;
}

bool TL_GhDocument::SetSliderValueByOrdinal(size_t slider_ordinal, double value) {
  size_t k = 0;
  for (auto& n : nodes_) {
    if (!n.is_slider) continue;
    if (k++ != slider_ordinal) continue;
    n.slider.value = std::max(n.slider.min_val, std::min(n.slider.max_val, value));
    if (!n.persistent_data.empty()) n.persistent_data[0] = TL_GhValue::MakeNumber(n.slider.value);
    return Evaluate();
  }
  return false;
}

size_t TL_GhDocument::SliderCount() const {
  size_t k = 0;
  for (const auto& n : nodes_) if (n.is_slider) ++k;
  return k;
}

size_t TL_GhDocument::Ports() const {
  size_t k = 0;
  for (const auto& n : nodes_) k += n.inputs.size() + n.outputs.size();
  return k;
}

size_t TL_GhDocument::ComponentCount() const {
  size_t k = 0;
  for (const auto& n : nodes_) if (!n.is_group && !n.is_scribble && !n.is_panel && !n.is_slider) ++k;
  return k;
}

TL_GhRectF TL_GhDocument::CanvasBounds() const {
  TL_GhRectF r;
  if (nodes_.empty()) return r;
  float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
  for (const auto& n : nodes_) {
    x0 = std::min(x0, n.bounds.x);
    y0 = std::min(y0, n.bounds.y);
    x1 = std::max(x1, n.bounds.x + n.bounds.w);
    y1 = std::max(y1, n.bounds.y + n.bounds.h);
  }
  r.x = x0;
  r.y = y0;
  r.w = std::max(1.0f, x1 - x0);
  r.h = std::max(1.0f, y1 - y0);
  return r;
}

int TL_GhDocument::BakeToScene(opentl::mvp::SceneDocument& out_scene,
                               std::vector<ON_Mesh*>& out_meshes,
                               std::vector<const float*>* out_mesh_colors) const {
  using namespace opentl::mvp;
  int baked = 0;
  static const float kMauve[3] = {0.796f, 0.651f, 0.969f};
  static const float kTeal[3] = {0.580f, 0.886f, 0.835f};

  const std::string layer = "Grasshopper";
  out_scene.AddLayer(layer, 203, 166, 247);

  for (const auto& n : nodes_) {
    if (n.hidden || n.is_group || n.is_scribble || n.is_panel) continue;
    std::vector<const std::vector<TL_GhValue>*> outs;
    for (const auto& o : n.outputs) outs.push_back(&o.runtime_data);
    if (outs.empty()) outs.push_back(&n.runtime_data);

    const std::string base = n.nickname.empty() ? n.name : n.nickname;
    for (const auto* vec : outs) {
      int k = 0;
      for (const auto& v : *vec) {
        const std::string nm = base + "_" + std::to_string(n.index) + "_" + std::to_string(k++);
        switch (v.kind) {
          case TL_GhValueKind::Point: out_scene.AddPoint(v.pt_val, nm, layer); ++baked; break;
          case TL_GhValueKind::Line: out_scene.AddLine(v.line_val, nm, layer); ++baked; break;
          case TL_GhValueKind::Circle:
            if (v.circle_val.IsValid()) { out_scene.AddCircle(v.circle_val, nm, layer); ++baked; }
            break;
          case TL_GhValueKind::Arc:
            if (v.arc_val.IsValid()) { out_scene.AddArc(v.arc_val, nm, layer); ++baked; }
            break;
          case TL_GhValueKind::Plane: out_scene.AddPlane(v.plane_val, nm, layer); ++baked; break;
          case TL_GhValueKind::Curve:
            if (v.curve_val) { out_scene.AddNurbsCurve(*v.curve_val, nm, layer); ++baked; }
            break;
          case TL_GhValueKind::Mesh:
            if (v.mesh_val) {
              out_meshes.push_back(new ON_Mesh(*v.mesh_val));
              if (out_mesh_colors) out_mesh_colors->push_back(kTeal);
              ++baked;
            }
            break;
          case TL_GhValueKind::Surface:
            if (v.surface_val) {
              ON_Mesh* m = TL_MeshSurfaceGrid(*v.surface_val, 24, 24, nullptr);
              if (m) {
                out_meshes.push_back(m);
                if (out_mesh_colors) out_mesh_colors->push_back(kMauve);
                ++baked;
              }
            }
            break;
          default: break;
        }
      }
    }
  }
  return baked;
}

std::string TL_GhDocument::FormatSummary() const {
  std::ostringstream os;
  size_t ports = 0, wires_in = 0;
  for (const auto& n : nodes_) {
    ports += n.inputs.size() + n.outputs.size();
    for (const auto& p : n.inputs) wires_in += p.source_guids.size();
  }
  const TL_GhRectF cb = CanvasBounds();
  os << "gh.document=" << (document_name_.empty() ? "(unnamed)" : document_name_)
     << "\ngh.nodes=" << nodes_.size()
     << "\ngh.sliders=" << SliderCount()
     << "\ngh.components=" << ComponentCount()
     << "\ngh.ports=" << ports
     << "\ngh.wires=" << wires_in
     << "\ngh.canvas=" << (int)cb.x << "," << (int)cb.y << " " << (int)cb.w << "x" << (int)cb.h;
  return os.str();
}
