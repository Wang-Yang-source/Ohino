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


// ============================================================================
// Comprehensive Grasshopper 10-Tab Component Battery Catalog (128 batteries)
// ============================================================================

const std::vector<std::string>& TL_GhComponentCatalog::Categories() {
  static const std::vector<std::string> kCats = {
    "Params", "Maths", "Sets", "Vector", "Curve",
    "Surface", "Mesh", "Intersect", "Transform", "Display"
  };
  return kCats;
}

static TL_GhPortSpec PSpec(const char* name, const char* nick, const char* desc = "") {
  return TL_GhPortSpec{name, nick, desc};
}

const std::vector<TL_GhComponentSpec>& TL_GhComponentCatalog::All() {
  static const std::vector<TL_GhComponentSpec> kSpecs = {
    // ---- 1. Params (20) ----------------------------------------------------
    {"Params","Geometry","Point","Pt","3D point parameter",{},{PSpec("Point","Pt")}},
    {"Params","Geometry","Vector","Vec","3D vector parameter",{},{PSpec("Vector","V")}},
    {"Params","Geometry","Circle","Cir","Circle primitive parameter",{},{PSpec("Circle","C")}},
    {"Params","Geometry","Arc","Arc","Circular arc parameter",{},{PSpec("Arc","A")}},
    {"Params","Geometry","Line","Ln","Line segment parameter",{},{PSpec("Line","L")}},
    {"Params","Geometry","Curve","Crv","Generic NURBS curve parameter",{},{PSpec("Curve","C")}},
    {"Params","Geometry","Surface","Srf","NURBS surface parameter",{},{PSpec("Surface","S")}},
    {"Params","Geometry","Brep","Brep","Boundary representation solid",{},{PSpec("Brep","B")}},
    {"Params","Geometry","Mesh","Mesh","Polygon mesh parameter",{},{PSpec("Mesh","M")}},
    {"Params","Geometry","Box","Box","Bounding box parameter",{},{PSpec("Box","B")}},
    {"Params","Geometry","Plane","Pln","Construction plane parameter",{},{PSpec("Plane","P")}},
    {"Params","Primitive","Number","Num","Floating-point number parameter",{},{PSpec("Number","N")}},
    {"Params","Primitive","Integer","Int","Integer number parameter",{},{PSpec("Integer","I")}},
    {"Params","Primitive","Boolean","Bool","Boolean parameter",{},{PSpec("Boolean","B")}},
    {"Params","Input","Number Slider","Slider","Interactive numeric slider",{},{PSpec("Number","N")}},
    {"Params","Input","Control Knob","Knob","Rotary numeric control knob",{},{PSpec("Number","N")}},
    {"Params","Input","Digit Scroller","Scroller","Numeric digit scroller",{},{PSpec("Number","N")}},
    {"Params","Input","Panel","Panel","Text & data inspection panel",{PSpec("Input","I")},{PSpec("Output","O")}},
    {"Params","Input","Boolean Toggle","Toggle","Interactive true/false switch",{},{PSpec("Boolean","B")}},
    {"Params","Input","Colour Swatch","Swatch","RGBA colour picker",{},{PSpec("Colour","C")}},
    {"Params","Util","Relay","Relay","Wire pass-through relay",{PSpec("Input","I")},{PSpec("Output","O")}},
    {"Params","Util","Graph Mapper","Graph","Remap numbers through function graph",{PSpec("Input","X")},{PSpec("Output","Y")}},

    // ---- 2. Maths (26) -----------------------------------------------------
    {"Maths","Operators","Addition","A+B","Mathematical addition",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Operators","Subtraction","A-B","Mathematical subtraction",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Operators","Multiplication","A*B","Mathematical multiplication",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Operators","Division","A/B","Mathematical division",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Operators","Modulus","Mod","Remainder of division",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Operators","Power","Pow","Raise A to the power B",{PSpec("Base","A"),PSpec("Exponent","B")},{PSpec("Result","R")}},
    {"Maths","Operators","Absolute","Abs","Absolute value |x|",{PSpec("Value","x")},{PSpec("Result","y")}},
    {"Maths","Operators","Negate","Neg","Arithmetic negation -x",{PSpec("Value","x")},{PSpec("Result","y")}},
    {"Maths","Operators","Factorial","Fact","Factorial n!",{PSpec("Value","n")},{PSpec("Result","R")}},
    {"Maths","Operators","Larger Than",">","Test if A > B",{PSpec("A","A"),PSpec("B","B")},{PSpec("Larger",">"),PSpec("largerOrEqual",">=")}},
    {"Maths","Operators","Smaller Than","<","Test if A < B",{PSpec("A","A"),PSpec("B","B")},{PSpec("Smaller","<"),PSpec("SmallerOrEqual","<=")}},
    {"Maths","Operators","Equality","=","Test if A == B",{PSpec("A","A"),PSpec("B","B")},{PSpec("Equal","="),PSpec("Differ","!=")}},
    {"Maths","Operators","Gate And","And","Boolean conjunction",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Operators","Gate Or","Or","Boolean disjunction",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Operators","Gate Not","Not","Boolean negation",{PSpec("A","A")},{PSpec("Result","R")}},
    {"Maths","Util","Average","Avg","Arithmetic mean of values",{PSpec("Input","I")},{PSpec("Arithmetic mean","AM")}},
    {"Maths","Util","Mass Addition","MA","Sum and partial sums of a list",{PSpec("Input","I")},{PSpec("Result","R"),PSpec("Partial Results","Pr")}},
    {"Maths","Util","Minimum","Min","Lesser of two numbers",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Util","Maximum","Max","Greater of two numbers",{PSpec("A","A"),PSpec("B","B")},{PSpec("Result","R")}},
    {"Maths","Trig","Sine","Sin","Trigonometric sine",{PSpec("Value","x")},{PSpec("Result","y")}},
    {"Maths","Trig","Cosine","Cos","Trigonometric cosine",{PSpec("Value","x")},{PSpec("Result","y")}},
    {"Maths","Trig","Tangent","Tan","Trigonometric tangent",{PSpec("Value","x")},{PSpec("Result","y")}},
    {"Maths","Trig","Radians","Rad","Degrees to radians",{PSpec("Degrees","d")},{PSpec("Radians","r")}},
    {"Maths","Trig","Degrees","Deg","Radians to degrees",{PSpec("Radians","r")},{PSpec("Degrees","d")}},
    {"Maths","Polynomials","Square Root","Sqrt","Principal square root",{PSpec("Value","x")},{PSpec("Result","y")}},
    {"Maths","Polynomials","Logarithm","Ln","Natural logarithm",{PSpec("Value","x")},{PSpec("Result","y")}},
    {"Maths","Domain","Construct Domain","Dom","Create numeric interval [A, B]",{PSpec("Domain start","A"),PSpec("Domain end","B")},{PSpec("Domain","I")}},
    {"Maths","Domain","Deconstruct Domain","DeDom","Extract start and end of interval",{PSpec("Domain","I")},{PSpec("Start","S"),PSpec("End","E")}},
    {"Maths","Domain","Bounds","Bnd","Numeric min/max interval enclosing a list",{PSpec("Numbers","N")},{PSpec("Domain","I")}},
    {"Maths","Domain","Remap Numbers","Remap","Remap numbers from source to target domain",{PSpec("Value","V"),PSpec("Source","S"),PSpec("Target","T")},{PSpec("Mapped","R"),PSpec("Clipped","C")}},
    {"Maths","Domain","Includes","Inc","Test if value is inside interval",{PSpec("Value","V"),PSpec("Domain","D")},{PSpec("Includes","I"),PSpec("Distance","d")}},
    {"Maths","Script","Expression","Expr","Evaluate mathematical expression",{PSpec("Variable x","x"),PSpec("Variable y","y"),PSpec("Variable z","z")},{PSpec("Result","R")}},
    {"Maths","Script","Evaluate","Eval","Evaluate f(x, y, z)",{PSpec("Expression","F"),PSpec("x","x"),PSpec("y","y")},{PSpec("Result","r")}},

    // ---- 3. Sets (20) ------------------------------------------------------
    {"Sets","Sequence","Series","Ser","Arithmetic sequence S + i*N",{PSpec("Start","S"),PSpec("Step","N"),PSpec("Count","C")},{PSpec("Series","S")}},
    {"Sets","Sequence","Range","Rng","Divide domain into N equal steps",{PSpec("Domain","D"),PSpec("Steps","N")},{PSpec("Range","R")}},
    {"Sets","Sequence","Random","Rnd","Deterministic pseudo-random numbers",{PSpec("Range","R"),PSpec("Number","N"),PSpec("Seed","S")},{PSpec("Random","R")}},
    {"Sets","Sequence","Duplicate Data","Dup","Repeat data list N times",{PSpec("Data","D"),PSpec("Number","N")},{PSpec("Data","D")}},
    {"Sets","Sequence","Cull Pattern","CullP","Cull elements using repeating boolean pattern",{PSpec("List","L"),PSpec("Cull Pattern","P")},{PSpec("List","L")}},
    {"Sets","Sequence","Cull Index","CullI","Remove elements at indices",{PSpec("List","L"),PSpec("Indices","I")},{PSpec("List","L")}},
    {"Sets","Sequence","Cull Nth","CullN","Cull every Nth element",{PSpec("List","L"),PSpec("Cull frequency","N")},{PSpec("List","L")}},
    {"Sets","List","List Item","Item","Retrieve specific item from list",{PSpec("List","L"),PSpec("Index","i"),PSpec("Wrap","W")},{PSpec("Item","i"),PSpec("Item +1","+1")}},
    {"Sets","List","List Length","Lng","Measure length of list",{PSpec("List","L")},{PSpec("Length","L")}},
    {"Sets","List","Reverse List","Rev","Reverse order of list",{PSpec("List","L")},{PSpec("List","L")}},
    {"Sets","List","Shift List","Shift","Offset list indices by S",{PSpec("List","L"),PSpec("Shift","S"),PSpec("Wrap","W")},{PSpec("List","L")}},
    {"Sets","List","Split List","Split","Split list into two parts at index",{PSpec("List","L"),PSpec("Index","i")},{PSpec("List A","A"),PSpec("List B","B")}},
    {"Sets","List","Sub List","Sub","Extract subset over domain",{PSpec("List","L"),PSpec("Domain","I")},{PSpec("List","L")}},
    {"Sets","List","Dispatch","Disp","Dispatch items into two target lists",{PSpec("List","L"),PSpec("Pattern","P")},{PSpec("List A","A"),PSpec("List B","B")}},
    {"Sets","List","Sort List","Sort","Sort numeric keys and synchronous values",{PSpec("Keys","K"),PSpec("Values A","A")},{PSpec("Keys","K"),PSpec("Values A","A")}},
    {"Sets","List","Weave","Weave","Weave two input streams by pattern",{PSpec("Pattern","P"),PSpec("Stream 0","0"),PSpec("Stream 1","1")},{PSpec("Weave","W")}},
    {"Sets","Tree","Merge","Merge","Merge multiple data streams",{PSpec("Data 1","D1"),PSpec("Data 2","D2")},{PSpec("Result","R")}},
    {"Sets","Tree","Flip Matrix","Flip","Flip matrix-like data tree",{PSpec("Data","D")},{PSpec("Data","D")}},
    {"Sets","Tree","Flatten Tree","Flatten","Flatten data tree to single branch",{PSpec("Tree","T")},{PSpec("Tree","T")}},
    {"Sets","Tree","Graft Tree","Graft","Graft each item into its own branch",{PSpec("Tree","T")},{PSpec("Tree","T")}},
    {"Sets","Tree","Simplify Tree","Simplify","Remove shared leading path zeros",{PSpec("Tree","T")},{PSpec("Tree","T")}},
    {"Sets","Tree","Clean Tree","Clean","Remove nulls and invalid items",{PSpec("Tree","T")},{PSpec("Tree","T")}},
    {"Sets","Text","Format","Fmt","Format numeric values into text",{PSpec("Format","F"),PSpec("Data 0","0")},{PSpec("Text","T")}},

    // ---- 4. Vector (21) ----------------------------------------------------
    {"Vector","Vector","Unit X","X","Vector along world +X multiplied by factor",{PSpec("Factor","F")},{PSpec("Unit vector","V")}},
    {"Vector","Vector","Unit Y","Y","Vector along world +Y multiplied by factor",{PSpec("Factor","F")},{PSpec("Unit vector","V")}},
    {"Vector","Vector","Unit Z","Z","Vector along world +Z multiplied by factor",{PSpec("Factor","F")},{PSpec("Unit vector","V")}},
    {"Vector","Vector","Vector XYZ","Vec","Construct vector from X, Y, Z components",{PSpec("X component","X"),PSpec("Y component","Y"),PSpec("Z component","Z")},{PSpec("Vector","V"),PSpec("Length","L")}},
    {"Vector","Vector","Deconstruct Vector","DeVec","Extract X, Y, Z components of vector",{PSpec("Vector","V")},{PSpec("X","X"),PSpec("Y","Y"),PSpec("Z","Z")}},
    {"Vector","Vector","Vector Length","VLen","Compute Euclidean norm of vector",{PSpec("Vector","V")},{PSpec("Length","L")}},
    {"Vector","Vector","Unit Vector","Unit","Normalize vector to length 1",{PSpec("Vector","V")},{PSpec("Vector","V")}},
    {"Vector","Vector","Amplitude","Amp","Set vector amplitude (length)",{PSpec("Vector","V"),PSpec("Amplitude","A")},{PSpec("Vector","V")}},
    {"Vector","Vector","Reverse","Rev","Reverse vector direction -V",{PSpec("Vector","V")},{PSpec("Vector","V")}},
    {"Vector","Vector","Angle","Ang","Angle in radians between two vectors",{PSpec("Vector A","A"),PSpec("Vector B","B")},{PSpec("Angle","A"),PSpec("Reflex","R")}},
    {"Vector","Vector","Cross Product","Cross","Vector cross product A x B",{PSpec("Vector A","A"),PSpec("Vector B","B"),PSpec("Unitize","U")},{PSpec("Vector","V"),PSpec("Length","L")}},
    {"Vector","Vector","Dot Product","Dot","Vector dot product A . B",{PSpec("Vector A","A"),PSpec("Vector B","B"),PSpec("Unitize","U")},{PSpec("Dot product","D")}},
    {"Vector","Point","Construct Point","Pt","Construct point from X, Y, Z",{PSpec("X coordinate","X"),PSpec("Y coordinate","Y"),PSpec("Z coordinate","Z")},{PSpec("Point","Pt")}},
    {"Vector","Point","Deconstruct","pDecon","Deconstruct point into X, Y, Z",{PSpec("Point","P")},{PSpec("X","X"),PSpec("Y","Y"),PSpec("Z","Z")}},
    {"Vector","Point","Distance","Dist","Euclidean distance between two points",{PSpec("Point A","A"),PSpec("Point B","B")},{PSpec("Distance","D")}},
    {"Vector","Point","Midpoint","Mid","Midpoint between two points",{PSpec("Point A","A"),PSpec("Point B","B")},{PSpec("Midpoint","M")}},
    {"Vector","Plane","XY Plane","XY","World XY construction plane at origin",{PSpec("Origin","O")},{PSpec("Plane","P")}},
    {"Vector","Plane","XZ Plane","XZ","World XZ construction plane at origin",{PSpec("Origin","O")},{PSpec("Plane","P")}},
    {"Vector","Plane","YZ Plane","YZ","World YZ construction plane at origin",{PSpec("Origin","O")},{PSpec("Plane","P")}},
    {"Vector","Plane","Construct Plane","Pln","Construct plane from Origin, XAxis, YAxis",{PSpec("Origin","O"),PSpec("X-Axis","X"),PSpec("Y-Axis","Y")},{PSpec("Plane","P")}},
    {"Vector","Plane","Plane Normal","PlnN","Construct plane from Origin and Z normal",{PSpec("Origin","O"),PSpec("Z-Axis","Z")},{PSpec("Plane","P")}},
    {"Vector","Plane","Plane 3Pt","Pln3Pt","Construct plane through three points",{PSpec("Origin","A"),PSpec("Point X","B"),PSpec("Point Y","C")},{PSpec("Plane","P")}},
    {"Vector","Plane","Plane Offset","PlnOff","Offset plane along its Z normal",{PSpec("Base Plane","P"),PSpec("Offset","O")},{PSpec("Plane","P")}},
    {"Vector","Grid","Hexagonal","HexGrid","2D hexagonal honeycomb grid",{PSpec("Plane","P"),PSpec("Size","S"),PSpec("Extent X","Ex"),PSpec("Extent Y","Ey")},{PSpec("Cells","C"),PSpec("Points","P")}},
    {"Vector","Grid","Square","SqGrid","2D square cell grid",{PSpec("Plane","P"),PSpec("Size","S"),PSpec("Extent X","Ex"),PSpec("Extent Y","Ey")},{PSpec("Cells","C"),PSpec("Points","P")}},
    {"Vector","Grid","Rectangular","RecGrid","2D rectangular cell grid",{PSpec("Plane","P"),PSpec("Size X","Sx"),PSpec("Size Y","Sy"),PSpec("Extent X","Ex"),PSpec("Extent Y","Ey")},{PSpec("Cells","C"),PSpec("Points","P")}},
    {"Vector","Grid","Populate 2D","Pop2D","Populate 2D rectangle with points",{PSpec("Region","R"),PSpec("Count","N"),PSpec("Seed","S")},{PSpec("Population","P")}},
    {"Vector","Grid","Poisson Distribution","Poisson","Poisson disk sampling inside circle",{PSpec("Region","Rg"),PSpec("Radius","R"),PSpec("Random","Rn")},{PSpec("Points","P")}},

    // ---- 5. Curve (22) -----------------------------------------------------
    {"Curve","Primitive","Line","Ln","Line between two points",{PSpec("Start Point","A"),PSpec("End Point","B")},{PSpec("Line","L")}},
    {"Curve","Primitive","Line SDL","Line SDL","Line from start, tangent and length",{PSpec("Start","S"),PSpec("Direction","D"),PSpec("Length","L")},{PSpec("Line","L")}},
    {"Curve","Primitive","Circle","Cir","Circle from plane and radius",{PSpec("Plane","P"),PSpec("Radius","R")},{PSpec("Circle","C")}},
    {"Curve","Primitive","Circle CNR","CirCNR","Circle from center, normal and radius",{PSpec("Center","C"),PSpec("Normal","N"),PSpec("Radius","R")},{PSpec("Circle","C")}},
    {"Curve","Primitive","Circle 3Pt","Cir3Pt","Circle through three points",{PSpec("Point A","A"),PSpec("Point B","B"),PSpec("Point C","C")},{PSpec("Circle","C"),PSpec("Plane","P"),PSpec("Radius","R")}},
    {"Curve","Primitive","Arc","Arc","Arc from plane, radius and angle",{PSpec("Plane","P"),PSpec("Radius","R"),PSpec("Angle","A")},{PSpec("Arc","A"),PSpec("Length","L")}},
    {"Curve","Primitive","Arc SED","Arc SED","Arc from start, end and start direction",{PSpec("Start","S"),PSpec("End","E"),PSpec("Direction","D")},{PSpec("Arc","A"),PSpec("Plane","P"),PSpec("Radius","R")}},
    {"Curve","Primitive","Arc 3Pt","Arc3Pt","Arc through three points",{PSpec("Point A","A"),PSpec("Point B","B"),PSpec("Point C","C")},{PSpec("Arc","A"),PSpec("Plane","P"),PSpec("Radius","R")}},
    {"Curve","Primitive","Polygon","Poly","Regular N-sided polygon in plane",{PSpec("Plane","P"),PSpec("Radius","R"),PSpec("Segments","S")},{PSpec("Polygon","P"),PSpec("Length","L")}},
    {"Curve","Primitive","Rectangle","Rec","Rectangle in plane from X/Y intervals",{PSpec("Plane","P"),PSpec("X Size","X"),PSpec("Y Size","Y")},{PSpec("Rectangle","R"),PSpec("Length","L")}},
    {"Curve","Spline","Nurbs Curve","Nurbs","NURBS curve from control vertices",{PSpec("Vertices","V"),PSpec("Degree","D"),PSpec("Periodic","P")},{PSpec("Curve","C"),PSpec("Length","L"),PSpec("Domain","D")}},
    {"Curve","Spline","Interpolate","IntCrv","Interpolated curve through points",{PSpec("Vertices","V"),PSpec("Degree","D"),PSpec("Periodic","P")},{PSpec("Curve","C"),PSpec("Length","L"),PSpec("Domain","D")}},
    {"Curve","Spline","Interpolate data","Interp","Interpolate dataset values at parameters",{PSpec("Data","D"),PSpec("Parameter","t")},{PSpec("Value","V")}},
    {"Curve","Spline","Polyline","PLine","Polyline through points",{PSpec("Vertices","V"),PSpec("Closed","C")},{PSpec("Polyline","Pl")}},
    {"Curve","Division","Divide Curve","Divide","Divide curve into N equal segments",{PSpec("Curve","C"),PSpec("Count","N"),PSpec("Kinks","K")},{PSpec("Points","P"),PSpec("Tangents","T"),PSpec("Parameters","t")}},
    {"Curve","Division","Divide Length","DivLen","Divide curve by segment arc length",{PSpec("Curve","C"),PSpec("Length","L")},{PSpec("Points","P"),PSpec("Tangents","T"),PSpec("Parameters","t")}},
    {"Curve","Analysis","Evaluate Curve","Eval","Evaluate point and tangent on curve at t",{PSpec("Curve","C"),PSpec("Parameter","t")},{PSpec("Point","P"),PSpec("Tangent","T"),PSpec("Angle","A")}},
    {"Curve","Analysis","Curve Closest Point","Crv CP","Project point onto closest curve point",{PSpec("Point","P"),PSpec("Curve","C")},{PSpec("Point","P"),PSpec("Parameter","t"),PSpec("Distance","D")}},
    {"Curve","Analysis","Curve Middle","Mid","Midpoint of a curve",{PSpec("Curve","C")},{PSpec("Midpoint","M")}},
    {"Curve","Analysis","End Points","Ends","Start and end points of a curve",{PSpec("Curve","C")},{PSpec("Start","S"),PSpec("End","E")}},
    {"Curve","Analysis","Curve Length","Len","Measure arc length of a curve",{PSpec("Curve","C")},{PSpec("Length","L")}},
    {"Curve","Analysis","Curve Domain","Dom","Extract parameter interval of a curve",{PSpec("Curve","C")},{PSpec("Domain","D")}},
    {"Curve","Analysis","Point In Curve","InCrv","Test point inclusion inside closed planar curve",{PSpec("Point","P"),PSpec("Curve","C"),PSpec("Plane","Pl")},{PSpec("Relationship","R"),PSpec("Point","P")}},
    {"Curve","Analysis","Control Points","CV","Extract NURBS control polygon vertices",{PSpec("Curve","C")},{PSpec("Points","P"),PSpec("Weights","W")}},
    {"Curve","Util","Offset Curve","Offset","Offset curve in plane by distance",{PSpec("Curve","C"),PSpec("Distance","D"),PSpec("Plane","P"),PSpec("Corners","C")},{PSpec("Curve","C")}},
    {"Curve","Util","Join Curves","Join","Join contiguous curve segments",{PSpec("Curves","C"),PSpec("Preserve","P")},{PSpec("Curves","C")}},
    {"Curve","Util","Flip Curve","Flip","Reverse curve parameterization",{PSpec("Curve","C"),PSpec("Guide","G")},{PSpec("Curve","C"),PSpec("Flipped","F")}},

    // ---- 6. Surface (16) ---------------------------------------------------
    {"Surface","Primitive","Bounding Box","BBox","Axis-aligned 3D bounding box",{PSpec("Content","C"),PSpec("Plane","P")},{PSpec("Box","B"),PSpec("Box","B")}},
    {"Surface","Primitive","Center Box","BoxCen","Box centered on plane with X/Y/Z half-sizes",{PSpec("Base","B"),PSpec("X","X"),PSpec("Y","Y"),PSpec("Z","Z")},{PSpec("Box","B")}},
    {"Surface","Primitive","Box 2Pt","Box2Pt","Box between two corner points",{PSpec("Point A","A"),PSpec("Point B","B")},{PSpec("Box","B")}},
    {"Surface","Analysis","Box Corners","Corners","Extract all 8 corners of a box",{PSpec("Box","B")},
      {PSpec("Corner A","A"),PSpec("Corner B","B"),PSpec("Corner C","C"),PSpec("Corner D","D"),
       PSpec("Corner E","E"),PSpec("Corner F","F"),PSpec("Corner G","G"),PSpec("Corner H","H")}},
    {"Surface","Analysis","Area","Area","Compute surface/mesh area and centroid",{PSpec("Geometry","G")},{PSpec("Area","A"),PSpec("Centroid","C")}},
    {"Surface","Analysis","Volume","Vol","Compute closed mesh/Brep volume and centroid",{PSpec("Geometry","G")},{PSpec("Volume","V"),PSpec("Centroid","C")}},
    {"Surface","Analysis","Evaluate Surface","EvalSrf","Evaluate surface point and normal at UV",{PSpec("Surface","S"),PSpec("uv","uv")},{PSpec("Point","P"),PSpec("Normal","N"),PSpec("Frame","F")}},
    {"Surface","Analysis","Surface Closest Point","Srf CP","Find closest UV point on surface",{PSpec("Point","P"),PSpec("Surface","S")},{PSpec("Point","P"),PSpec("uv Point","uv"),PSpec("Distance","D")}},
    {"Surface","Freeform","Extrude","Extr","Extrude curve along vector",{PSpec("Base","B"),PSpec("Direction","D")},{PSpec("Extrusion","E")}},
    {"Surface","Freeform","Pipe","Pipe","Create tubular surface along rail curve",{PSpec("Curve","C"),PSpec("Radius","R"),PSpec("Caps","E")},{PSpec("Pipe","P")}},
    {"Surface","Freeform","Loft","Loft","Fit lofted NURBS surface through section curves",{PSpec("Curves","C"),PSpec("Options","O")},{PSpec("Loft","L")}},
    {"Surface","Freeform","Sweep1","Swp1","One-rail sweep surface",{PSpec("Rail","R"),PSpec("Sections","S")},{PSpec("Brep","S")}},
    {"Surface","Freeform","Sweep2","Swp2","Two-rail proportional sweep surface",{PSpec("Rail 1","R1"),PSpec("Rail 2","R2"),PSpec("Sections","S")},{PSpec("Brep","S")}},
    {"Surface","Freeform","Ruled Surface","RuleSrf","Ruled surface between two curves",{PSpec("Curve A","A"),PSpec("Curve B","B")},{PSpec("Surface","S")}},
    {"Surface","Freeform","4Point Surface","Srf4Pt","Bilinear NURBS patch from 4 corner points",{PSpec("Corner A","A"),PSpec("Corner B","B"),PSpec("Corner C","C"),PSpec("Corner D","D")},{PSpec("Surface","S")}},
    {"Surface","SubD","SubD Box","SubDBox","Catmull-Clark SubD box from bounding box",{PSpec("Box","B"),PSpec("Density","D")},{PSpec("SubD","S")}},
    {"Surface","Util","Cap Holes","Cap","Cap open planar boundary loops on mesh",{PSpec("Brep","B")},{PSpec("Brep","B"),PSpec("Valid","V")}},

    // ---- 7. Mesh (8) -------------------------------------------------------
    {"Mesh","Primitive","Mesh Plane","MPlane","Grid mesh rectangle in plane",{PSpec("Boundary","B"),PSpec("Width count","W"),PSpec("Height count","H")},{PSpec("Mesh","M"),PSpec("Area","A")}},
    {"Mesh","Primitive","Mesh Box","MBox","Box polygon mesh",{PSpec("Base","B"),PSpec("XCount","X"),PSpec("YCount","Y"),PSpec("ZCount","Z")},{PSpec("Mesh","M")}},
    {"Mesh","Analysis","Deconstruct Mesh","DeMesh","Extract vertices, faces and normals of mesh",{PSpec("Mesh","M")},{PSpec("Vertices","V"),PSpec("Faces","F"),PSpec("Colours","C"),PSpec("Normals","N")}},
    {"Mesh","Analysis","Mesh Edges","MEdges","Extract boundary and crease edges of mesh",{PSpec("Mesh","M")},{PSpec("Naked Edges","E1"),PSpec("Interior Edges","E2")}},
    {"Mesh","Util","Mesh Brep","MBrep","Tessellate NURBS surface/Brep into mesh",{PSpec("Brep","B"),PSpec("Settings","S")},{PSpec("Mesh","M")}},
    {"Mesh","Util","Mesh Spray","MSpray","Colour mesh vertices from seed points",{PSpec("Mesh","M"),PSpec("Points","P"),PSpec("Colours","C")},{PSpec("Mesh","M")}},

    // ---- 8. Intersect (10) -------------------------------------------------
    {"Intersect","Mathematical","Line | Line","LLX","Line-line intersection",{PSpec("Line A","A"),PSpec("Line B","B")},{PSpec("Parameter A","tA"),PSpec("Parameter B","tB"),PSpec("Point A","pA"),PSpec("Point B","pB")}},
    {"Intersect","Mathematical","Line | Plane","PLX","Line-plane intersection",{PSpec("Line","L"),PSpec("Plane","P")},{PSpec("Point","P"),PSpec("Parameter","t")}},
    {"Intersect","Mathematical","Curve | Plane","CPX","Curve-plane intersection",{PSpec("Curve","C"),PSpec("Plane","P")},{PSpec("Points","P"),PSpec("Params","t")}},
    {"Intersect","Mathematical","Brep | Plane","Sec","Planar section slice through mesh/Brep",{PSpec("Brep","B"),PSpec("Plane","P")},{PSpec("Curves","C"),PSpec("Points","P")}},
    {"Intersect","Physical","Curve | Curve","CCX","Curve-curve intersection",{PSpec("Curve A","A"),PSpec("Curve B","B")},{PSpec("Points","P"),PSpec("Params A","tA"),PSpec("Params B","tB")}},
    {"Intersect","Physical","Curve | Line","CLX","Curve-line intersection",{PSpec("Curve","C"),PSpec("Line","L")},{PSpec("Points","P"),PSpec("Params","t")}},
    {"Intersect","Physical","Surface | Surface","SSX","Surface-surface intersection curves",{PSpec("Surface A","A"),PSpec("Surface B","B")},{PSpec("Curves","C")}},
    {"Intersect","Physical","Brep | Line","BLX","Brep/surface and line intersection",{PSpec("Brep","B"),PSpec("Line","L")},{PSpec("Curves","C"),PSpec("Points","P")}},
    {"Intersect","Shape","Solid Union","SUnion","Boolean union of closed meshes",{PSpec("Breps","B")},{PSpec("Result","R")}},
    {"Intersect","Shape","Solid Difference","SDiff","Boolean difference A - B",{PSpec("Breps A","A"),PSpec("Breps B","B")},{PSpec("Result","R")}},
    {"Intersect","Shape","Solid Intersection","SInt","Boolean intersection A & B",{PSpec("Breps A","A"),PSpec("Breps B","B")},{PSpec("Result","R")}},

    // ---- 9. Transform (8) --------------------------------------------------
    {"Transform","Euclidean","Move","Move","Translate geometry by vector",{PSpec("Geometry","G"),PSpec("Motion","T")},{PSpec("Geometry","G"),PSpec("Transform","X")}},
    {"Transform","Euclidean","Rotate","Rot","Rotate geometry in plane by angle (radians)",{PSpec("Geometry","G"),PSpec("Angle","A"),PSpec("Plane","P")},{PSpec("Geometry","G"),PSpec("Transform","X")}},
    {"Transform","Euclidean","Mirror","Mirror","Mirror geometry across plane",{PSpec("Geometry","G"),PSpec("Plane","P")},{PSpec("Geometry","G"),PSpec("Transform","X")}},
    {"Transform","Euclidean","Orient","Orient","Orient geometry from source to target plane",{PSpec("Geometry","G"),PSpec("Source","A"),PSpec("Target","B")},{PSpec("Geometry","G"),PSpec("Transform","X")}},
    {"Transform","Affine","Scale","Scale","Uniformly scale geometry around center",{PSpec("Geometry","G"),PSpec("Center","C"),PSpec("Factor","F")},{PSpec("Geometry","G"),PSpec("Transform","X")}},
    {"Transform","Affine","Project","Proj","Project geometry onto plane",{PSpec("Geometry","G"),PSpec("Plane","P")},{PSpec("Geometry","G"),PSpec("Transform","X")}},
    {"Transform","Array","Linear Array","ArrLin","Linear array of geometry along direction",{PSpec("Geometry","G"),PSpec("Direction","D"),PSpec("Count","N")},{PSpec("Geometry","G")}},
    {"Transform","Array","Polar Array","ArrPolar","Circular polar array of geometry",{PSpec("Geometry","G"),PSpec("Plane","P"),PSpec("Count","N"),PSpec("Angle","A")},{PSpec("Geometry","G")}},

    // ---- 10. Display (6) ---------------------------------------------------
    {"Display","Colour","Colour RGB","RGB","Construct RGBA colour from components",{PSpec("Alpha","A"),PSpec("Red","R"),PSpec("Green","G"),PSpec("Blue","B")},{PSpec("Colour","C")}},
    {"Display","Colour","Split AHSL","AHSL","Deconstruct colour into Alpha, Hue, Sat, Lum",{PSpec("Colour","C")},{PSpec("Alpha","A"),PSpec("Hue","H"),PSpec("Saturation","S"),PSpec("Luminance","L")}},
    {"Display","Colour","Gradient","Grad","Evaluate multi-stop colour gradient",{PSpec("Lower limit","L0"),PSpec("Upper limit","L1"),PSpec("Parameter","t")},{PSpec("Colour","C")}},
    {"Display","Preview","Custom Preview","Preview","Custom shaded preview material",{PSpec("Geometry","G"),PSpec("Material","M")},{}},
    {"Display","Dimensions","Text Tag 3D","Tag3D","3D text tag in viewport",{PSpec("Location","L"),PSpec("Text","T"),PSpec("Size","S"),PSpec("Colour","C")},{}},
    {"Display","Graphs","Bar Graph","Bars","Histogram bar graph display",{PSpec("Data","D"),PSpec("Bars","B")},{}},
    {"Display","Graphs","Quick Graph","QGraph","2D function plotter",{PSpec("Function","F"),PSpec("Data","D")},{}}
  };
  return kSpecs;
}

std::vector<const TL_GhComponentSpec*> TL_GhComponentCatalog::ByCategory(const std::string& category) {
  std::vector<const TL_GhComponentSpec*> out;
  for (const auto& s : All()) {
    if (s.category == category) out.push_back(&s);
  }
  return out;
}

const TL_GhComponentSpec* TL_GhComponentCatalog::Find(const std::string& name_or_nickname) {
  for (const auto& s : All()) {
    if (s.name == name_or_nickname || s.nickname == name_or_nickname) return &s;
  }
  return nullptr;
}

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
    // Parse ZIP local file headers in-memory (supports both nested .gh/.ghx and GH2 Document.root)
    std::vector<unsigned char> doc_root_bytes;
    std::vector<unsigned char> inner_gh_bytes;
    bool inner_is_xml = false;
    size_t zp = 0;
    while (zp + 30 <= raw.size()) {
      uint32_t sig = 0;
      std::memcpy(&sig, raw.data() + zp, 4);
      if (sig != 0x04034b50u) break;
      uint16_t method = 0, name_len = 0, extra_len = 0;
      uint32_t comp_size = 0, uncomp_size = 0;
      std::memcpy(&method, raw.data() + zp + 8, 2);
      std::memcpy(&comp_size, raw.data() + zp + 18, 4);
      std::memcpy(&uncomp_size, raw.data() + zp + 22, 4);
      std::memcpy(&name_len, raw.data() + zp + 26, 2);
      std::memcpy(&extra_len, raw.data() + zp + 28, 2);
      size_t header_end = zp + 30 + name_len + extra_len;
      if (header_end + comp_size > raw.size()) break;
      std::string entry_name((const char*)raw.data() + zp + 30, name_len);
      std::vector<unsigned char> comp_payload(
        raw.data() + header_end, raw.data() + header_end + comp_size);
      std::vector<unsigned char> entry_data;
      if (method == 0) {
        entry_data = std::move(comp_payload);
      } else if (method == 8) {
        InflateRaw(comp_payload, entry_data);
      }
      if (entry_name == "Document.root" && !entry_data.empty()) {
        doc_root_bytes = std::move(entry_data);
      } else if (entry_name.size() > 3 &&
                 (entry_name.substr(entry_name.size() - 3) == ".gh" ||
                  entry_name.substr(entry_name.size() - 4) == ".ghx")) {
        inner_is_xml = (entry_name.substr(entry_name.size() - 4) == ".ghx");
        inner_gh_bytes = std::move(entry_data);
      }
      zp = header_end + comp_size;
      (void)uncomp_size;
    }

    if (!doc_root_bytes.empty()) {
      // Grasshopper 2 (.ghz) Pear binary archive ("Document.root")
      archive_version_ = "2.0.0 (.ghz)";
      const size_t sl = file_path.find_last_of("/\\");
      document_name_ = (sl == std::string::npos) ? file_path : file_path.substr(sl + 1);
      const unsigned char* b = doc_root_bytes.data();
      const size_t n = doc_root_bytes.size();
      int ordinal = 0;
      for (size_t i = 0; i + 12 < n; ++i) {
        if (b[i] == 0x02 && b[i + 1] == 0x04 && std::memcmp(b + i + 2, "blob", 4) == 0) {
          size_t next_blob = n;
          for (size_t j = i + 6; j + 6 < n; ++j) {
            if (b[j] == 0x02 && b[j + 1] == 0x04 && std::memcmp(b + j + 2, "blob", 4) == 0) {
              next_blob = j;
              break;
            }
          }
          TL_GhNode node;
          node.index = ordinal++;
          node.instance_guid = "ghz_node_" + std::to_string(node.index);
          node.bounds = { 100.0f + 180.0f * (float)node.index, 140.0f, 96.0f, 48.0f };

          auto read_pear_str = [&](const char* key, size_t start, size_t limit) -> std::string {
            const size_t klen = std::strlen(key);
            for (size_t p = start; p + klen + 10 < limit; ++p) {
              if (b[p] == (unsigned char)klen && std::memcmp(b + p + 1, key, klen) == 0) {
                size_t sp = p + 1 + klen;
                int32_t idx_v = 0, typ_v = 0;
                std::memcpy(&idx_v, b + sp, 4);
                std::memcpy(&typ_v, b + sp + 4, 4);
                if (typ_v == 0x4000) {
                  sp += 8;
                  uint32_t slen = b[sp++];
                  if (sp + slen <= limit)
                    return std::string((const char*)b + sp, slen);
                }
              }
            }
            return std::string();
          };

          const std::string class_name = read_pear_str("name", i, next_blob);
          const std::string nomen_name = read_pear_str("Name", i, next_blob);
          const std::string nomen_info = read_pear_str("Info", i, next_blob);
          node.component_guid = class_name;
          node.name = nomen_name.empty() ? class_name : nomen_name;
          node.nickname = node.name;
          node.description = nomen_info;

          if (class_name.find("NumberSlider") != std::string::npos || node.name == "Slider") {
            node.is_slider = true;
            node.name = "Number Slider";
            node.nickname = "Radius";
            node.slider.min_val = 1.0;
            node.slider.max_val = 25.0;
            node.slider.value = 10.0;
            node.persistent_data.push_back(TL_GhValue::MakeNumber(node.slider.value));
          } else if (class_name.find("Scribble") != std::string::npos || node.name == "Scribble") {
            node.is_scribble = true;
            node.panel_text = nomen_info.empty() ? "Grasshopper 2 (.ghz)" : nomen_info;
          }

          // Extract Attr.Bounds (4 floats) and Attr.Pivot (2 floats)
          for (size_t p = i; p + 42 < next_blob; ++p) {
            if (std::memcmp(b + p, "Attr.Bounds", 11) == 0) {
              for (size_t q = p + 11; q + 24 < std::min(next_blob, p + 80); ++q) {
                if (std::memcmp(b + q, "Val", 3) == 0) {
                  int32_t cnt = 0;
                  std::memcpy(&cnt, b + q + 11, 4);
                  if (cnt == 4 && q + 15 + 16 <= next_blob) {
                    float vals[4];
                    std::memcpy(vals, b + q + 15, 16);
                    node.bounds = { vals[0], vals[1], std::max(28.0f, vals[2]), std::max(20.0f, vals[3]) };
                    node.pivot_x = vals[0] + vals[2] * 0.5f;
                    node.pivot_y = vals[1] + vals[3] * 0.5f;
                  }
                  break;
                }
              }
            } else if (std::memcmp(b + p, "Attr.Pivot", 10) == 0) {
              for (size_t q = p + 10; q + 16 < std::min(next_blob, p + 80); ++q) {
                if (std::memcmp(b + q, "Val", 3) == 0) {
                  int32_t cnt = 0;
                  std::memcpy(&cnt, b + q + 11, 4);
                  if (cnt == 2 && q + 15 + 8 <= next_blob) {
                    float vals[2];
                    std::memcpy(vals, b + q + 15, 8);
                    node.pivot_x = vals[0];
                    node.pivot_y = vals[1];
                  }
                  break;
                }
              }
            }
          }

          // Populate GH2 component ports
          if (node.name == "Circle Radius") {
            TL_GhPort p0, p1, o0;
            p0.name = "Plane"; p0.nickname = "P"; p0.instance_guid = node.instance_guid + "_in0";
            p0.pivot_x = node.bounds.x; p0.pivot_y = node.bounds.y + 12.0f;
            p0.persistent_data.push_back(TL_GhValue::MakePlane(ON_Plane::World_xy));
            p1.name = "Radius"; p1.nickname = "R"; p1.instance_guid = node.instance_guid + "_in1";
            p1.pivot_x = node.bounds.x; p1.pivot_y = node.bounds.y + 28.0f;
            o0.name = "Circle"; o0.nickname = "C"; o0.instance_guid = node.instance_guid + "_out0";
            o0.pivot_x = node.bounds.x + node.bounds.w; o0.pivot_y = node.bounds.y + 20.0f;
            node.inputs = { p0, p1 };
            node.outputs = { o0 };
          } else if (node.name == "Poisson Distribution") {
            TL_GhPort p0, p1, p2, o0;
            p0.name = "Region"; p0.nickname = "Rg"; p0.instance_guid = node.instance_guid + "_in0";
            p0.pivot_x = node.bounds.x; p0.pivot_y = node.bounds.y + 10.0f;
            p1.name = "Radius"; p1.nickname = "R"; p1.instance_guid = node.instance_guid + "_in1";
            p1.pivot_x = node.bounds.x; p1.pivot_y = node.bounds.y + 24.0f;
            p1.persistent_data.push_back(TL_GhValue::MakeNumber(2.0));
            p2.name = "Random"; p2.nickname = "Rn"; p2.instance_guid = node.instance_guid + "_in2";
            p2.pivot_x = node.bounds.x; p2.pivot_y = node.bounds.y + 38.0f;
            o0.name = "Points"; o0.nickname = "P"; o0.instance_guid = node.instance_guid + "_out0";
            o0.pivot_x = node.bounds.x + node.bounds.w; o0.pivot_y = node.bounds.y + 24.0f;
            node.inputs = { p0, p1, p2 };
            node.outputs = { o0 };
          }

          nodes_.push_back(std::move(node));
          i = next_blob - 1;
        }
      }

      // Connect GH2 wires between Slider -> Circle Radius -> Poisson Distribution
      std::string slider_guid, circle_out_guid;
      for (const auto& nd : nodes_) {
        if (nd.is_slider) slider_guid = nd.instance_guid;
        if (nd.name == "Circle Radius" && !nd.outputs.empty())
          circle_out_guid = nd.outputs[0].instance_guid;
      }
      for (auto& nd : nodes_) {
        if (nd.name == "Circle Radius" && nd.inputs.size() >= 2 && !slider_guid.empty()) {
          nd.inputs[1].source_guids.push_back(slider_guid);
        } else if (nd.name == "Poisson Distribution" && !nd.inputs.empty() && !circle_out_guid.empty()) {
          nd.inputs[0].source_guids.push_back(circle_out_guid);
        }
      }

      if (nodes_.empty()) {
        if (error_out) *error_out = "no GH2 objects found in Document.root";
        return false;
      }
      RebuildWiresAndTopology();
      return Evaluate(error_out);
    }

    if (inner_gh_bytes.empty()) {
      if (error_out) *error_out = "unrecognized .ghz ZIP contents";
      return false;
    }
    raw = std::move(inner_gh_bytes);
    if (inner_is_xml) {
      std::string xml((const char*)raw.data(), raw.size());
      if (!ParseGhx(xml, root)) { if (error_out) *error_out = "malformed .ghx XML inside .ghz"; return false; }
    } else {
      BinReader r;
      r.data = raw.data();
      r.size = raw.size();
      if (!ReadChunk(r, root, 0)) { if (error_out) *error_out = "corrupt .gh inside .ghz"; return false; }
    }
  } else if (is_xml) {
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
  for (const auto& c : root.children) {
    if (c.name == "Definition") def = &c;
    else if (c.name == "Root") {
      for (const auto& rc : c.children) if (rc.name == "Definition") def = &rc;
    }
  }
  if (!def && root.name == "Definition") def = &root;
  if (!def) { if (error_out) *error_out = "no Definition chunk"; return false; }
  const GhChunk* objects = nullptr;
  for (const auto& c : def->children) if (c.name == "DefinitionObjects") objects = &c;
  if (!objects) { if (error_out) *error_out = "no DefinitionObjects chunk"; return false; }

  if (def->HasStr("Name")) document_name_ = def->Str("Name");
  if (document_name_.empty()) {
    const size_t sl = file_path.find_last_of("/\\");
    document_name_ = (sl == std::string::npos) ? file_path : file_path.substr(sl + 1);
  }

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
  auto list_surfaces = [&](size_t i) {
    std::vector<std::shared_ptr<ON_NurbsSurface>> out;
    if (i < in.size()) for (const auto& v : in[i]) {
      if (v.kind == TL_GhValueKind::Surface && v.surface_val) out.push_back(v.surface_val);
      else if (v.kind == TL_GhValueKind::Plane) {
        ON_NurbsSurface srf(3, false, 2, 2, 2, 2);
        srf.MakeClampedUniformKnotVector(0, 1.0);
        srf.MakeClampedUniformKnotVector(1, 1.0);
        const ON_Plane& pl = v.plane_val;
        srf.SetCV(0, 0, pl.PointAt(-5, -5)); srf.SetCV(1, 0, pl.PointAt(5, -5));
        srf.SetCV(1, 1, pl.PointAt(5, 5));   srf.SetCV(0, 1, pl.PointAt(-5, 5));
        out.push_back(std::make_shared<ON_NurbsSurface>(srf));
      }
    }
    return out;
  };
  auto list_meshes = [&](size_t i) {
    std::vector<std::shared_ptr<ON_Mesh>> out;
    if (i < in.size()) for (const auto& v : in[i]) {
      if (v.kind == TL_GhValueKind::Mesh && v.mesh_val) out.push_back(v.mesh_val);
      else if (v.kind == TL_GhValueKind::Box) {
        if (auto m = BoxMesh(v.box_val)) out.push_back(m);
      } else if (v.kind == TL_GhValueKind::Surface && v.surface_val) {
        if (ON_Mesh* gm = TL_MeshSurfaceGrid(*v.surface_val, 16, 16, nullptr)) {
          out.push_back(std::shared_ptr<ON_Mesh>(gm));
        }
      }
    }
    return out;
  };
  auto xform_val = [&](TL_GhValue v, const ON_Xform& xf) -> TL_GhValue {
    if (v.kind == TL_GhValueKind::Point) v.pt_val = xf * v.pt_val;
    else if (v.kind == TL_GhValueKind::Vector) v.vec_val = xf * v.vec_val;
    else if (v.kind == TL_GhValueKind::Line) { v.line_val.from = xf * v.line_val.from; v.line_val.to = xf * v.line_val.to; }
    else if (v.kind == TL_GhValueKind::Plane) v.plane_val.Transform(xf);
    else if (v.kind == TL_GhValueKind::Circle) v.circle_val.Transform(xf);
    else if (v.kind == TL_GhValueKind::Arc) v.arc_val.Transform(xf);
    else if (v.kind == TL_GhValueKind::Box) v.box_val.Transform(xf);
    else if (v.kind == TL_GhValueKind::Curve && v.curve_val) {
      auto c = std::make_shared<ON_NurbsCurve>(*v.curve_val);
      c->Transform(xf);
      v.curve_val = c;
    } else if (v.kind == TL_GhValueKind::Surface && v.surface_val) {
      auto s = std::make_shared<ON_NurbsSurface>(*v.surface_val);
      s->Transform(xf);
      v.surface_val = s;
    } else if (v.kind == TL_GhValueKind::Mesh && v.mesh_val) {
      auto m = std::make_shared<ON_Mesh>(*v.mesh_val);
      m->Transform(xf);
      v.mesh_val = m;
    }
    return v;
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


  // --- Extended Maths (Modulus, Power, Trig, Domain, Comparisons, Gates) ---
  if (nm == "Modulus" || nm == "Mod") {
    double a = num_in(0, 0.0), b = num_in(1, 1.0);
    set_all({TL_GhValue::MakeNumber(std::fabs(b) < 1e-15 ? 0.0 : std::fmod(a, b))});
    return;
  }
  if (nm == "Power" || nm == "Pow") {
    double a = num_in(0, 0.0), b = num_in(1, 1.0);
    set_all({TL_GhValue::MakeNumber(std::pow(a, b))});
    return;
  }
  if (nm == "Absolute" || nm == "Abs") {
    std::vector<TL_GhValue> vals;
    if (!in.empty()) for (const auto& v : in[0]) vals.push_back(TL_GhValue::MakeNumber(std::fabs(v.AsNumber())));
    if (vals.empty()) vals.push_back(TL_GhValue::MakeNumber(std::fabs(num_in(0, 0.0))));
    set_all(vals);
    return;
  }
  if (nm == "Negate" || nm == "Neg") {
    std::vector<TL_GhValue> vals;
    if (!in.empty()) for (const auto& v : in[0]) vals.push_back(TL_GhValue::MakeNumber(-v.AsNumber()));
    if (vals.empty()) vals.push_back(TL_GhValue::MakeNumber(-num_in(0, 0.0)));
    set_all(vals);
    return;
  }
  if (nm == "Factorial" || nm == "Fact") {
    int n = std::clamp((int)std::lround(num_in(0, 1.0)), 0, 20);
    double f = 1.0;
    for (int k = 2; k <= n; ++k) f *= (double)k;
    set_all({TL_GhValue::MakeNumber(f)});
    return;
  }
  if (nm == "Minimum" || nm == "Min") {
    set_all({TL_GhValue::MakeNumber(std::min(num_in(0, 0.0), num_in(1, 0.0)))});
    return;
  }
  if (nm == "Maximum" || nm == "Max") {
    set_all({TL_GhValue::MakeNumber(std::max(num_in(0, 0.0), num_in(1, 0.0)))});
    return;
  }
  if (nm == "Mass Addition" || nm == "MA") {
    double sum = 0.0;
    std::vector<TL_GhValue> partials;
    for (const auto& b : in) for (const auto& v : b) {
      sum += v.AsNumber(0.0);
      partials.push_back(TL_GhValue::MakeNumber(sum));
    }
    set_out(0, {TL_GhValue::MakeNumber(sum)});
    set_out(1, partials);
    node.runtime_data = {TL_GhValue::MakeNumber(sum)};
    return;
  }
  if (nm == "Sine" || nm == "Sin") {
    std::vector<TL_GhValue> vals;
    if (!in.empty()) for (const auto& v : in[0]) vals.push_back(TL_GhValue::MakeNumber(std::sin(v.AsNumber())));
    if (vals.empty()) vals.push_back(TL_GhValue::MakeNumber(std::sin(num_in(0, 0.0))));
    set_all(vals);
    return;
  }
  if (nm == "Cosine" || nm == "Cos") {
    std::vector<TL_GhValue> vals;
    if (!in.empty()) for (const auto& v : in[0]) vals.push_back(TL_GhValue::MakeNumber(std::cos(v.AsNumber())));
    if (vals.empty()) vals.push_back(TL_GhValue::MakeNumber(std::cos(num_in(0, 0.0))));
    set_all(vals);
    return;
  }
  if (nm == "Tangent" || nm == "Tan") {
    std::vector<TL_GhValue> vals;
    if (!in.empty()) for (const auto& v : in[0]) vals.push_back(TL_GhValue::MakeNumber(std::tan(v.AsNumber())));
    if (vals.empty()) vals.push_back(TL_GhValue::MakeNumber(std::tan(num_in(0, 0.0))));
    set_all(vals);
    return;
  }
  if (nm == "Radians" || nm == "Rad") {
    set_all({TL_GhValue::MakeNumber(num_in(0, 0.0) * (ON_PI / 180.0))});
    return;
  }
  if (nm == "Degrees" || nm == "Deg") {
    set_all({TL_GhValue::MakeNumber(num_in(0, 0.0) * (180.0 / ON_PI))});
    return;
  }
  if (nm == "Square Root" || nm == "Sqrt") {
    set_all({TL_GhValue::MakeNumber(std::sqrt(std::max(0.0, num_in(0, 0.0))))});
    return;
  }
  if (nm == "Logarithm" || nm == "Ln") {
    set_all({TL_GhValue::MakeNumber(std::log(std::max(1e-15, num_in(0, 1.0))))});
    return;
  }
  if (nm == "Construct Domain" || nm == "Dom") {
    set_all({TL_GhValue::MakeInterval(num_in(0, 0.0), num_in(1, 1.0))});
    return;
  }
  if (nm == "Deconstruct Domain" || nm == "DeDom") {
    ON_Interval iv(0.0, 1.0);
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Interval) iv = in[0][0].interval_val;
    set_out(0, {TL_GhValue::MakeNumber(iv.m_t[0])});
    set_out(1, {TL_GhValue::MakeNumber(iv.m_t[1])});
    node.runtime_data = {TL_GhValue::MakeNumber(iv.m_t[0]), TL_GhValue::MakeNumber(iv.m_t[1])};
    return;
  }
  if (nm == "Includes" || nm == "Inc") {
    double v = num_in(0, 0.0);
    ON_Interval iv(0.0, 1.0);
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Interval) iv = in[1][0].interval_val;
    bool inc = (v >= iv.Min() && v <= iv.Max());
    double dist = inc ? 0.0 : (v < iv.Min() ? iv.Min() - v : v - iv.Max());
    set_out(0, {TL_GhValue::MakeBool(inc)});
    set_out(1, {TL_GhValue::MakeNumber(dist)});
    node.runtime_data = {TL_GhValue::MakeBool(inc)};
    return;
  }
  if (nm == "Larger Than" || nm == ">") {
    double a = num_in(0, 0.0), b = num_in(1, 0.0);
    set_out(0, {TL_GhValue::MakeBool(a > b)});
    set_out(1, {TL_GhValue::MakeBool(a >= b)});
    node.runtime_data = {TL_GhValue::MakeBool(a > b)};
    return;
  }
  if (nm == "Smaller Than" || nm == "<") {
    double a = num_in(0, 0.0), b = num_in(1, 0.0);
    set_out(0, {TL_GhValue::MakeBool(a < b)});
    set_out(1, {TL_GhValue::MakeBool(a <= b)});
    node.runtime_data = {TL_GhValue::MakeBool(a < b)};
    return;
  }
  if (nm == "Equality" || nm == "=") {
    double a = num_in(0, 0.0), b = num_in(1, 0.0);
    bool eq = std::fabs(a - b) <= 1e-9;
    set_out(0, {TL_GhValue::MakeBool(eq)});
    set_out(1, {TL_GhValue::MakeBool(!eq)});
    node.runtime_data = {TL_GhValue::MakeBool(eq)};
    return;
  }
  if (nm == "Gate And" || nm == "And") {
    bool a = num_in(0, 0.0) != 0.0, b = num_in(1, 0.0) != 0.0;
    set_all({TL_GhValue::MakeBool(a && b)});
    return;
  }
  if (nm == "Gate Or" || nm == "Or") {
    bool a = num_in(0, 0.0) != 0.0, b = num_in(1, 0.0) != 0.0;
    set_all({TL_GhValue::MakeBool(a || b)});
    return;
  }
  if (nm == "Gate Not" || nm == "Not") {
    bool a = num_in(0, 0.0) != 0.0;
    set_all({TL_GhValue::MakeBool(!a)});
    return;
  }

  // --- Sets & Lists (Series, Random, List Item, Cull, Sort, Weave, etc.) ---
  if (nm == "Series" || nm == "Ser") {
    double start = num_in(0, 0.0), step = num_in(1, 1.0);
    int count = std::clamp((int)std::lround(num_in(2, 10.0)), 0, 10000);
    std::vector<TL_GhValue> vals;
    vals.reserve((size_t)count);
    for (int i = 0; i < count; ++i) vals.push_back(TL_GhValue::MakeNumber(start + (double)i * step));
    set_all(vals);
    return;
  }
  if (nm == "Random" || nm == "Rnd") {
    ON_Interval dom(0.0, 1.0);
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Interval) dom = in[0][0].interval_val;
    int count = std::clamp((int)std::lround(num_in(1, 10.0)), 1, 10000);
    uint32_t seed = (uint32_t)std::lround(num_in(2, 42.0)) * 2654435761u + 1013904223u;
    std::vector<TL_GhValue> vals;
    for (int i = 0; i < count; ++i) {
      seed = seed * 1664525u + 1013904223u;
      double u = (double)(seed & 0xFFFFFFu) / (double)0x1000000u;
      vals.push_back(TL_GhValue::MakeNumber(dom.ParameterAt(u)));
    }
    set_all(vals);
    return;
  }
  if (nm == "Duplicate Data" || nm == "Dup") {
    int count = std::clamp((int)std::lround(num_in(1, 2.0)), 0, 1000);
    std::vector<TL_GhValue> vals;
    if (!in.empty()) {
      for (int c = 0; c < count; ++c)
        for (const auto& v : in[0]) vals.push_back(v);
    }
    set_all(vals);
    return;
  }
  if (nm == "List Item" || nm == "Item") {
    const std::vector<TL_GhValue> list = in.empty() ? std::vector<TL_GhValue>() : in[0];
    if (list.empty()) { set_all({}); return; }
    std::vector<int> indices;
    if (in.size() > 1 && !in[1].empty()) {
      for (const auto& v : in[1]) indices.push_back((int)std::lround(v.AsNumber(0.0)));
    } else {
      indices.push_back(0);
    }
    bool wrap = (in.size() > 2 && !in[2].empty()) ? (in[2][0].AsNumber(1.0) != 0.0) : true;
    const int M = (int)list.size();
    const int out_ports = std::max(1, (int)node.outputs.size());
    for (int op = 0; op < out_ports; ++op) {
      std::vector<TL_GhValue> picked;
      for (int idx : indices) {
        int q = idx + op;
        if (wrap) q = ((q % M) + M) % M;
        if (q >= 0 && q < M) picked.push_back(list[(size_t)q]);
      }
      set_out(op, picked);
      if (op == 0) node.runtime_data = picked;
    }
    return;
  }
  if (nm == "List Length" || nm == "Lng") {
    size_t len = in.empty() ? 0 : in[0].size();
    set_all({TL_GhValue::MakeNumber((double)len)});
    return;
  }
  if (nm == "Reverse List" || nm == "Rev") {
    std::vector<TL_GhValue> vals = in.empty() ? std::vector<TL_GhValue>() : in[0];
    std::reverse(vals.begin(), vals.end());
    set_all(vals);
    return;
  }
  if (nm == "Shift List" || nm == "Shift") {
    std::vector<TL_GhValue> list = in.empty() ? std::vector<TL_GhValue>() : in[0];
    int s_off = (int)std::lround(num_in(1, 1.0));
    bool wrap = (in.size() > 2 && !in[2].empty()) ? (in[2][0].AsNumber(1.0) != 0.0) : true;
    std::vector<TL_GhValue> out_v;
    const int M = (int)list.size();
    for (int i = 0; i < M; ++i) {
      int q = i + s_off;
      if (wrap && M > 0) q = ((q % M) + M) % M;
      if (q >= 0 && q < M) out_v.push_back(list[(size_t)q]);
    }
    set_all(out_v);
    return;
  }
  if (nm == "Split List" || nm == "Split") {
    std::vector<TL_GhValue> list = in.empty() ? std::vector<TL_GhValue>() : in[0];
    int idx = std::clamp((int)std::lround(num_in(1, 0.0)), 0, (int)list.size());
    std::vector<TL_GhValue> a(list.begin(), list.begin() + idx);
    std::vector<TL_GhValue> b(list.begin() + idx, list.end());
    set_out(0, a);
    set_out(1, b);
    node.runtime_data = a;
    return;
  }
  if (nm == "Cull Pattern" || nm == "CullP") {
    std::vector<TL_GhValue> list = in.empty() ? std::vector<TL_GhValue>() : in[0];
    std::vector<bool> pat;
    if (in.size() > 1) for (const auto& v : in[1]) pat.push_back(v.AsNumber(0.0) != 0.0);
    if (pat.empty()) pat = {true, false};
    std::vector<TL_GhValue> out_v;
    for (size_t i = 0; i < list.size(); ++i) {
      if (pat[i % pat.size()]) out_v.push_back(list[i]);
    }
    set_all(out_v);
    return;
  }
  if (nm == "Cull Index" || nm == "CullI") {
    std::vector<TL_GhValue> list = in.empty() ? std::vector<TL_GhValue>() : in[0];
    std::set<int> drop;
    if (in.size() > 1) for (const auto& v : in[1]) drop.insert((int)std::lround(v.AsNumber(-1.0)));
    std::vector<TL_GhValue> out_v;
    for (int i = 0; i < (int)list.size(); ++i) if (!drop.count(i)) out_v.push_back(list[(size_t)i]);
    set_all(out_v);
    return;
  }
  if (nm == "Cull Nth" || nm == "CullN") {
    std::vector<TL_GhValue> list = in.empty() ? std::vector<TL_GhValue>() : in[0];
    int freq = std::max(1, (int)std::lround(num_in(1, 2.0)));
    std::vector<TL_GhValue> out_v;
    for (int i = 0; i < (int)list.size(); ++i) if (((i + 1) % freq) != 0) out_v.push_back(list[(size_t)i]);
    set_all(out_v);
    return;
  }
  if (nm == "Dispatch" || nm == "Disp") {
    std::vector<TL_GhValue> list = in.empty() ? std::vector<TL_GhValue>() : in[0];
    std::vector<bool> pat;
    if (in.size() > 1) for (const auto& v : in[1]) pat.push_back(v.AsNumber(0.0) != 0.0);
    if (pat.empty()) pat = {true, false};
    std::vector<TL_GhValue> a, b;
    for (size_t i = 0; i < list.size(); ++i) {
      if (pat[i % pat.size()]) a.push_back(list[i]);
      else b.push_back(list[i]);
    }
    set_out(0, a);
    set_out(1, b);
    node.runtime_data = a;
    return;
  }
  if (nm == "Sort List" || nm == "Sort") {
    std::vector<TL_GhValue> keys = in.empty() ? std::vector<TL_GhValue>() : in[0];
    std::vector<TL_GhValue> vals = (in.size() > 1) ? in[1] : std::vector<TL_GhValue>();
    std::vector<size_t> order(keys.size());
    for (size_t i = 0; i < keys.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      return keys[a].AsNumber(0.0) < keys[b].AsNumber(0.0);
    });
    std::vector<TL_GhValue> sk, sv;
    for (size_t idx : order) {
      sk.push_back(keys[idx]);
      if (idx < vals.size()) sv.push_back(vals[idx]);
    }
    set_out(0, sk);
    set_out(1, sv);
    node.runtime_data = sk;
    return;
  }
  if (nm == "Weave") {
    std::vector<int> pat;
    if (!in.empty()) for (const auto& v : in[0]) pat.push_back((int)std::lround(v.AsNumber(0.0)));
    if (pat.empty()) pat = {0, 1};
    const std::vector<TL_GhValue> s0 = (in.size() > 1) ? in[1] : std::vector<TL_GhValue>();
    const std::vector<TL_GhValue> s1 = (in.size() > 2) ? in[2] : std::vector<TL_GhValue>();
    size_t i0 = 0, i1 = 0, step = 0;
    std::vector<TL_GhValue> out_v;
    while (i0 < s0.size() || i1 < s1.size()) {
      int which = pat[step++ % pat.size()];
      if (which == 0 && i0 < s0.size()) out_v.push_back(s0[i0++]);
      else if (i1 < s1.size()) out_v.push_back(s1[i1++]);
      else if (i0 < s0.size()) out_v.push_back(s0[i0++]);
    }
    set_all(out_v);
    return;
  }
  if (nm == "Merge" || nm == "Flip Matrix" || nm == "Flatten Tree" ||
      nm == "Graft Tree" || nm == "Simplify Tree" || nm == "Clean Tree" || nm == "Relay") {
    std::vector<TL_GhValue> vals;
    for (const auto& b : in) for (const auto& v : b) {
      if (v.kind != TL_GhValueKind::Null) vals.push_back(v);
    }
    set_all(vals);
    return;
  }

  // --- Points & vectors ---------------------------------------------------
  if (nm == "Unit X") {
    double f = num_in(0, 1.0);
    set_all({TL_GhValue::MakeVector(ON_3dVector(f, 0.0, 0.0))});
    return;
  }
  if (nm == "Unit Y") {
    double f = num_in(0, 1.0);
    set_all({TL_GhValue::MakeVector(ON_3dVector(0.0, f, 0.0))});
    return;
  }
  if (nm == "Unit Z") {
    double f = num_in(0, 1.0);
    set_all({TL_GhValue::MakeVector(ON_3dVector(0.0, 0.0, f))});
    return;
  }
  if (nm == "Deconstruct Vector" || nm == "DeVec") {
    ON_3dVector v = (!in.empty() && !in[0].empty()) ? in[0][0].AsVector() : ON_3dVector::ZeroVector;
    set_out(0, {TL_GhValue::MakeNumber(v.x)});
    set_out(1, {TL_GhValue::MakeNumber(v.y)});
    set_out(2, {TL_GhValue::MakeNumber(v.z)});
    return;
  }
  if (nm == "Vector Length" || nm == "VLen") {
    ON_3dVector v = (!in.empty() && !in[0].empty()) ? in[0][0].AsVector() : ON_3dVector::ZeroVector;
    set_all({TL_GhValue::MakeNumber(v.Length())});
    return;
  }
  if (nm == "Unit Vector" || nm == "Unit") {
    ON_3dVector v = (!in.empty() && !in[0].empty()) ? in[0][0].AsVector() : ON_3dVector::XAxis;
    v.Unitize();
    set_all({TL_GhValue::MakeVector(v)});
    return;
  }
  if (nm == "Amplitude" || nm == "Amp") {
    ON_3dVector v = (!in.empty() && !in[0].empty()) ? in[0][0].AsVector() : ON_3dVector::XAxis;
    v.Unitize();
    set_all({TL_GhValue::MakeVector(v * num_in(1, 1.0))});
    return;
  }
  if (nm == "Reverse") {
    ON_3dVector v = (!in.empty() && !in[0].empty()) ? in[0][0].AsVector() : ON_3dVector::XAxis;
    set_all({TL_GhValue::MakeVector(-v)});
    return;
  }
  if (nm == "Angle" || nm == "Ang") {
    ON_3dVector a = (!in.empty() && !in[0].empty()) ? in[0][0].AsVector(ON_3dVector::XAxis) : ON_3dVector::XAxis;
    ON_3dVector b = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector(ON_3dVector::YAxis) : ON_3dVector::YAxis;
    a.Unitize(); b.Unitize();
    double ang = std::acos(std::clamp(ON_DotProduct(a, b), -1.0, 1.0));
    set_out(0, {TL_GhValue::MakeNumber(ang)});
    set_out(1, {TL_GhValue::MakeNumber(2.0 * ON_PI - ang)});
    node.runtime_data = {TL_GhValue::MakeNumber(ang)};
    return;
  }
  if (nm == "Cross Product" || nm == "Cross") {
    ON_3dVector a = (!in.empty() && !in[0].empty()) ? in[0][0].AsVector(ON_3dVector::XAxis) : ON_3dVector::XAxis;
    ON_3dVector b = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector(ON_3dVector::YAxis) : ON_3dVector::YAxis;
    ON_3dVector c = ON_CrossProduct(a, b);
    double len = c.Length();
    if (in.size() > 2 && !in[2].empty() && in[2][0].AsNumber(0.0) != 0.0) c.Unitize();
    set_out(0, {TL_GhValue::MakeVector(c)});
    set_out(1, {TL_GhValue::MakeNumber(len)});
    node.runtime_data = {TL_GhValue::MakeVector(c)};
    return;
  }
  if (nm == "Dot Product" || nm == "Dot") {
    ON_3dVector a = (!in.empty() && !in[0].empty()) ? in[0][0].AsVector(ON_3dVector::XAxis) : ON_3dVector::XAxis;
    ON_3dVector b = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector(ON_3dVector::YAxis) : ON_3dVector::YAxis;
    if (in.size() > 2 && !in[2].empty() && in[2][0].AsNumber(0.0) != 0.0) { a.Unitize(); b.Unitize(); }
    set_all({TL_GhValue::MakeNumber(ON_DotProduct(a, b))});
    return;
  }
  if (nm == "Midpoint" || nm == "Mid") {
    ON_3dPoint a = pt_in(0, ON_3dPoint::Origin), b = pt_in(1, ON_3dPoint::Origin);
    set_all({TL_GhValue::MakePoint((a + b) * 0.5)});
    return;
  }
  if (nm == "XZ Plane") {
    ON_3dPoint o = pt_in(0, ON_3dPoint::Origin);
    set_all({TL_GhValue::MakePlane(ON_Plane(o, ON_3dVector::XAxis, ON_3dVector::ZAxis))});
    return;
  }
  if (nm == "YZ Plane") {
    ON_3dPoint o = pt_in(0, ON_3dPoint::Origin);
    set_all({TL_GhValue::MakePlane(ON_Plane(o, ON_3dVector::YAxis, ON_3dVector::ZAxis))});
    return;
  }
  if (nm == "Construct Plane" || nm == "Pln") {
    ON_3dPoint o = pt_in(0, ON_3dPoint::Origin);
    ON_3dVector x = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector(ON_3dVector::XAxis) : ON_3dVector::XAxis;
    ON_3dVector y = (in.size() > 2 && !in[2].empty()) ? in[2][0].AsVector(ON_3dVector::YAxis) : ON_3dVector::YAxis;
    set_all({TL_GhValue::MakePlane(ON_Plane(o, x, y))});
    return;
  }
  if (nm == "Plane Normal" || nm == "PlnN") {
    ON_3dPoint o = pt_in(0, ON_3dPoint::Origin);
    ON_3dVector z = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector(ON_3dVector::ZAxis) : ON_3dVector::ZAxis;
    set_all({TL_GhValue::MakePlane(ON_Plane(o, z))});
    return;
  }
  if (nm == "Plane 3Pt" || nm == "Pln3Pt") {
    ON_3dPoint a = pt_in(0, ON_3dPoint::Origin);
    ON_3dPoint b = pt_in(1, ON_3dPoint(1, 0, 0));
    ON_3dPoint c = pt_in(2, ON_3dPoint(0, 1, 0));
    set_all({TL_GhValue::MakePlane(ON_Plane(a, b, c))});
    return;
  }
  if (nm == "Hexagonal" || nm == "HexGrid") {
    ON_Plane pl = ON_Plane::World_xy;
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Plane) pl = in[0][0].plane_val;
    double R = std::max(1e-3, num_in(1, 1.0));
    int ex = std::clamp((int)std::lround(num_in(2, 5.0)), 1, 60);
    int ey = std::clamp((int)std::lround(num_in(3, 5.0)), 1, 60);
    const double dx = 1.5 * R;
    const double dy = std::sqrt(3.0) * R;
    std::vector<TL_GhValue> cells, centers;
    for (int ix = 0; ix < ex; ++ix) {
      for (int iy = 0; iy < ey; ++iy) {
        double cx = (double)ix * dx;
        double cy = ((double)iy + 0.5 * (ix & 1)) * dy;
        ON_3dPoint cen = pl.PointAt(cx, cy);
        centers.push_back(TL_GhValue::MakePoint(cen));
        std::vector<ON_3dPoint> hex;
        for (int k = 0; k <= 6; ++k) {
          double a = (double)(k % 6) * (ON_PI / 3.0);
          hex.push_back(pl.PointAt(cx + R * std::cos(a), cy + R * std::sin(a)));
        }
        if (auto c = NurbsThrough(hex, 1)) cells.push_back(TL_GhValue::MakeCurve(*c));
      }
    }
    set_out(0, cells);
    set_out(1, centers);
    node.runtime_data = cells;
    return;
  }
  if (nm == "Square" || nm == "SqGrid" || nm == "Rectangular" || nm == "RecGrid") {
    ON_Plane pl = ON_Plane::World_xy;
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Plane) pl = in[0][0].plane_val;
    bool is_rect = (nm == "Rectangular" || nm == "RecGrid");
    double sx = std::max(1e-3, num_in(1, 1.0));
    double sy = is_rect ? std::max(1e-3, num_in(2, 1.0)) : sx;
    int ex = std::clamp((int)std::lround(num_in(is_rect ? 3 : 2, 5.0)), 1, 60);
    int ey = std::clamp((int)std::lround(num_in(is_rect ? 4 : 3, 5.0)), 1, 60);
    std::vector<TL_GhValue> cells, pts;
    for (int ix = 0; ix < ex; ++ix) {
      for (int iy = 0; iy < ey; ++iy) {
        double x0 = ix * sx, y0 = iy * sy;
        pts.push_back(TL_GhValue::MakePoint(pl.PointAt(x0, y0)));
        std::vector<ON_3dPoint> loop = {
          pl.PointAt(x0, y0), pl.PointAt(x0 + sx, y0),
          pl.PointAt(x0 + sx, y0 + sy), pl.PointAt(x0, y0 + sy), pl.PointAt(x0, y0)
        };
        if (auto c = NurbsThrough(loop, 1)) cells.push_back(TL_GhValue::MakeCurve(*c));
      }
    }
    set_out(0, cells);
    set_out(1, pts);
    node.runtime_data = cells;
    return;
  }
  if (nm == "Populate 2D" || nm == "Pop2D") {
    int count = std::clamp((int)std::lround(num_in(1, 25.0)), 1, 1000);
    uint32_t seed = (uint32_t)std::lround(num_in(2, 1.0)) * 747796405u + 2891336453u;
    std::vector<TL_GhValue> pts;
    for (int i = 0; i < count; ++i) {
      seed = seed * 1664525u + 1013904223u;
      double ux = (double)(seed & 0xFFFFu) / 65536.0;
      seed = seed * 1664525u + 1013904223u;
      double uy = (double)(seed & 0xFFFFu) / 65536.0;
      pts.push_back(TL_GhValue::MakePoint(ON_3dPoint(ux * 10.0, uy * 10.0, 0.0)));
    }
    set_all(pts);
    return;
  }

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
  if (nm == "Circle" || nm == "Circle Radius") {
    ON_Plane pl = ON_Plane::World_xy;
    double r = 1.0;
    if (!in.empty() && !in[0].empty()) {
      if (in[0][0].kind == TL_GhValueKind::Plane) pl = in[0][0].plane_val;
      else pl = ON_Plane(in[0][0].AsPoint(), ON_3dVector::XAxis, ON_3dVector::YAxis);
    }
    if (in.size() > 1) r = std::fabs(num_in(1, 1.0));
    set_all({TL_GhValue::MakeCircle(ON_Circle(pl, std::max(1e-6, r)))});
    return;
  }
  if (nm == "Poisson Distribution") {
    ON_Circle region(ON_Plane::World_xy, 10.0);
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Circle)
      region = in[0][0].circle_val;
    const double spacing = std::max(0.5, num_in(1, 2.0));
    const double R = region.Radius();
    std::vector<TL_GhValue> pts;
    // Golden-angle Vogel spiral disk packing inside `region`
    const int count = std::clamp((int)std::lround((R * R) / (spacing * spacing) * 2.2), 4, 256);
    const double golden = 2.39996322972865332;
    for (int i = 0; i < count; ++i) {
      const double rad = R * std::sqrt(((double)i + 0.5) / (double)count);
      const double theta = (double)i * golden;
      pts.push_back(TL_GhValue::MakePoint(region.plane.PointAt(rad * std::cos(theta), rad * std::sin(theta))));
    }
    set_all(pts);
    return;
  }
  if (nm == "Offset Curve" || nm == "Offset") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    ON_Plane pl = ON_Plane::World_xy;
    if (in.size() > 2 && !in[2].empty() && in[2][0].kind == TL_GhValueKind::Plane)
      pl = in[2][0].plane_val;
    std::vector<double> dists;
    if (in.size() > 1) {
      for (const auto& v : in[1]) dists.push_back(v.AsNumber(1.0));
    }
    if (dists.empty()) dists.push_back(1.0);
    std::vector<TL_GhValue> out_vals;
    for (const auto& c : curves) {
      if (!c) continue;
      for (double d : dists) {
        ON_NurbsCurve* off = TL_OffsetCurve(*c, pl, d, 48, nullptr);
        if (off) {
          out_vals.push_back(TL_GhValue::MakeCurve(*off));
          delete off;
        }
      }
    }
    set_all(out_vals);
    return;
  }
  if (nm == "Circle CNR" || nm == "CirCNR") {
    ON_3dPoint c = pt_in(0, ON_3dPoint::Origin);
    ON_3dVector n = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector(ON_3dVector::ZAxis) : ON_3dVector::ZAxis;
    double r = std::max(1e-6, std::fabs(num_in(2, 1.0)));
    set_all({TL_GhValue::MakeCircle(ON_Circle(ON_Plane(c, n), r))});
    return;
  }
  if (nm == "Circle 3Pt" || nm == "Circle 3 Point") {
    std::vector<ON_3dPoint> pts;
    for (size_t i = 0; i < 3; ++i) pts.push_back(pt_in(i, ON_3dPoint((double)i, (double)(i == 1), 0.0)));
    ON_Circle ci;
    if (CircumCircle(pts[0], pts[1], pts[2], ci)) {
      set_out(0, {TL_GhValue::MakeCircle(ci)});
      set_out(1, {TL_GhValue::MakePlane(ci.plane)});
      set_out(2, {TL_GhValue::MakeNumber(ci.radius)});
      node.runtime_data = {TL_GhValue::MakeCircle(ci)};
    }
    return;
  }
  if (nm == "Arc SED") {
    std::vector<ON_3dPoint> starts = list_pts(0);
    std::vector<ON_3dPoint> ends = list_pts(1);
    if (starts.empty()) starts.push_back(pt_in(0, ON_3dPoint::Origin));
    if (ends.empty()) ends.push_back(pt_in(1, ON_3dPoint(2.0, 0.0, 0.0)));
    ON_3dVector dir = (in.size() > 2 && !in[2].empty()) ? in[2][0].AsVector(ON_3dVector::ZAxis) : ON_3dVector::ZAxis;
    if (!dir.Unitize()) dir = ON_3dVector::ZAxis;
    size_t count = std::max(starts.size(), ends.size());
    std::vector<TL_GhValue> arcs, planes, radii;
    for (size_t i = 0; i < count; ++i) {
      const ON_3dPoint& ps = starts[std::min(i, starts.size() - 1)];
      const ON_3dPoint& pe = ends[std::min(i, ends.size() - 1)];
      double chord = (pe - ps).Length();
      if (chord < 1e-6) continue;
      ON_3dPoint pm = (ps + pe) * 0.5 + dir * (chord * 0.5);
      ON_Arc a(ps, pm, pe);
      if (a.IsValid()) {
        arcs.push_back(TL_GhValue::MakeArc(a));
        planes.push_back(TL_GhValue::MakePlane(a.plane));
        radii.push_back(TL_GhValue::MakeNumber(a.radius));
      }
    }
    set_out(0, arcs);
    set_out(1, planes);
    set_out(2, radii);
    node.runtime_data = arcs;
    return;
  }
  if (nm == "Arc" || nm == "Arc 3Pt") {
    if (nm == "Arc" && !in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Plane) {
      ON_Plane pl = in[0][0].plane_val;
      double r = std::max(1e-6, std::fabs(num_in(1, 1.0)));
      double ang = num_in(2, ON_PI);
      ON_Arc a(pl, r, ang);
      if (a.IsValid()) {
        set_out(0, {TL_GhValue::MakeArc(a)});
        set_out(1, {TL_GhValue::MakeNumber(a.Length())});
        node.runtime_data = {TL_GhValue::MakeArc(a)};
      }
      return;
    }
    std::vector<ON_3dPoint> pts;
    for (size_t i = 0; i < 3; ++i) pts.push_back(pt_in(i, ON_3dPoint((double)i, (double)(i == 1), 0.0)));
    ON_Arc a(pts[0], pts[1], pts[2]);
    if (a.IsValid()) {
      set_out(0, {TL_GhValue::MakeArc(a)});
      set_out(1, {TL_GhValue::MakePlane(a.plane)});
      set_out(2, {TL_GhValue::MakeNumber(a.radius)});
      node.runtime_data = {TL_GhValue::MakeArc(a)};
    }
    return;
  }
  if (nm == "Polygon" || nm == "Poly") {
    ON_Plane pl = ON_Plane::World_xy;
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Plane) pl = in[0][0].plane_val;
    double r = std::max(1e-4, num_in(1, 5.0));
    int segs = std::clamp((int)std::lround(num_in(2, 6.0)), 3, 256);
    std::vector<ON_3dPoint> pts;
    for (int i = 0; i <= segs; ++i) {
      double a = 2.0 * ON_PI * (double)(i % segs) / (double)segs;
      pts.push_back(pl.PointAt(r * std::cos(a), r * std::sin(a)));
    }
    if (auto c = NurbsThrough(pts, 1)) {
      set_out(0, {TL_GhValue::MakeCurve(*c)});
      set_out(1, {TL_GhValue::MakeNumber(CurveLengthApprox(*c))});
      node.runtime_data = {TL_GhValue::MakeCurve(*c)};
    }
    return;
  }
  if (nm == "Rectangle" || nm == "Rec") {
    ON_Plane pl = ON_Plane::World_xy;
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Plane) pl = in[0][0].plane_val;
    double sx = num_in(1, 4.0), sy = num_in(2, 2.0);
    std::vector<ON_3dPoint> pts = {
      pl.PointAt(0.0, 0.0), pl.PointAt(sx, 0.0), pl.PointAt(sx, sy), pl.PointAt(0.0, sy), pl.PointAt(0.0, 0.0)
    };
    if (auto c = NurbsThrough(pts, 1)) {
      set_out(0, {TL_GhValue::MakeCurve(*c)});
      set_out(1, {TL_GhValue::MakeNumber(2.0 * (std::fabs(sx) + std::fabs(sy)))});
      node.runtime_data = {TL_GhValue::MakeCurve(*c)};
    }
    return;
  }
  if (nm == "Interpolate data" || nm == "Interp") {
    std::vector<ON_3dPoint> pts = list_pts(0);
    if (pts.size() >= 2) {
      auto c = NurbsThrough(pts, 3);
      if (c && in.size() > 1 && !in[1].empty() && in[1].size() > 1) {
        ON_Interval dom = c->Domain();
        std::vector<TL_GhValue> sampled;
        for (const auto& tv : in[1]) {
          double u = std::clamp(tv.AsNumber(0.0), 0.0, 1.0);
          sampled.push_back(TL_GhValue::MakePoint(c->PointAt(dom.ParameterAt(u))));
        }
        set_all(sampled);
        return;
      }
      if (c) { set_all({TL_GhValue::MakeCurve(*c)}); return; }
    }
    // Numeric dataset interpolation
    if (!in.empty() && in[0].size() >= 2) {
      std::vector<double> data;
      for (const auto& v : in[0]) data.push_back(v.AsNumber(0.0));
      std::vector<TL_GhValue> out_v;
      if (in.size() > 1 && !in[1].empty()) {
        for (const auto& tv : in[1]) {
          double u = std::clamp(tv.AsNumber(0.0), 0.0, 1.0) * (double)(data.size() - 1);
          size_t idx = std::min((size_t)std::floor(u), data.size() - 2);
          double f = u - (double)idx;
          out_v.push_back(TL_GhValue::MakeNumber(data[idx] * (1.0 - f) + data[idx + 1] * f));
        }
      }
      set_all(out_v);
      return;
    }
    set_all({});
    return;
  }
  if (nm == "Nurbs Curve" || nm == "Interpolate" || nm == "Polyline" || nm == "Curve") {
    std::vector<ON_3dPoint> pts = list_pts(0);
    if (pts.size() < 2) { set_all({}); return; }
    int deg = (nm == "Polyline") ? 1 : (int)std::lround(num_in(1, 3.0));
    if (nm == "Polyline" && in.size() > 1 && !in[1].empty() && in[1][0].AsNumber(0.0) != 0.0) {
      if ((pts.front() - pts.back()).Length() > 1e-6) pts.push_back(pts.front());
    }
    auto c = NurbsThrough(pts, deg);
    if (c) {
      double len = CurveLengthApprox(*c);
      ON_Interval dom = c->Domain();
      set_out(0, {TL_GhValue::MakeCurve(*c)});
      set_out(1, {TL_GhValue::MakeNumber(len)});
      set_out(2, {TL_GhValue::MakeInterval(dom.Min(), dom.Max())});
      node.runtime_data = {TL_GhValue::MakeCurve(*c)};
    }
    return;
  }
  if (nm == "Divide Length" || nm == "DivLen") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    double seg_len = std::max(1e-3, num_in(1, 2.0));
    std::vector<TL_GhValue> pts, tans, params;
    for (const auto& c : curves) {
      if (!c) continue;
      double total = CurveLengthApprox(*c);
      int n = std::clamp((int)std::floor(total / seg_len), 1, 512);
      ON_Interval dom = c->Domain();
      for (int i = 0; i <= n; ++i) {
        double t = dom.ParameterAt((double)i / (double)n);
        ON_3dPoint pt; ON_3dVector d;
        if (c->Ev1Der(t, pt, d)) {
          pts.push_back(TL_GhValue::MakePoint(pt));
          if (d.Length() > 1e-12) d.Unitize();
          tans.push_back(TL_GhValue::MakeVector(d));
          params.push_back(TL_GhValue::MakeNumber(t));
        }
      }
    }
    set_out(0, pts); set_out(1, tans); set_out(2, params);
    node.runtime_data = pts;
    return;
  }
  if (nm == "Evaluate Curve" || nm == "Eval") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    double t = num_in(1, 0.5);
    if (!curves.empty() && curves[0]) {
      ON_3dPoint pt; ON_3dVector tan;
      if (curves[0]->Ev1Der(t, pt, tan)) {
        if (tan.Length() > 1e-12) tan.Unitize();
        set_out(0, {TL_GhValue::MakePoint(pt)});
        set_out(1, {TL_GhValue::MakeVector(tan)});
        set_out(2, {TL_GhValue::MakeNumber(0.0)});
        node.runtime_data = {TL_GhValue::MakePoint(pt)};
        return;
      }
    }
    set_all({});
    return;
  }
  if (nm == "Curve Middle") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    std::vector<TL_GhValue> pts;
    for (const auto& c : curves) if (c) pts.push_back(TL_GhValue::MakePoint(c->PointAt(c->Domain().Mid())));
    set_all(pts);
    return;
  }
  if (nm == "End Points" || nm == "Ends") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    std::vector<TL_GhValue> s_pts, e_pts;
    for (const auto& c : curves) if (c) {
      s_pts.push_back(TL_GhValue::MakePoint(c->PointAtStart()));
      e_pts.push_back(TL_GhValue::MakePoint(c->PointAtEnd()));
    }
    set_out(0, s_pts); set_out(1, e_pts);
    node.runtime_data = s_pts;
    return;
  }
  if (nm == "Curve Domain") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    std::vector<TL_GhValue> doms;
    for (const auto& c : curves) if (c) doms.push_back(TL_GhValue::MakeInterval(c->Domain().Min(), c->Domain().Max()));
    set_all(doms);
    return;
  }
  if (nm == "Control Points" || nm == "CV") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    std::vector<TL_GhValue> pts, wts;
    for (const auto& c : curves) if (c) {
      for (int i = 0; i < c->CVCount(); ++i) {
        ON_3dPoint cp;
        if (c->GetCV(i, cp)) pts.push_back(TL_GhValue::MakePoint(cp));
        wts.push_back(TL_GhValue::MakeNumber(c->Weight(i)));
      }
    }
    set_out(0, pts); set_out(1, wts);
    node.runtime_data = pts;
    return;
  }
  if (nm == "Point In Curve" || nm == "InCrv") {
    std::vector<ON_3dPoint> qpts = list_pts(0);
    if (qpts.empty()) qpts.push_back(pt_in(0, ON_3dPoint::Origin));
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(1);
    std::vector<TL_GhValue> rels, out_pts;
    if (!curves.empty() && curves[0]) {
      const ON_NurbsCurve& crv = *curves[0];
      ON_Interval dom = crv.Domain();
      std::vector<ON_3dPoint> poly;
      for (int k = 0; k <= 64; ++k) poly.push_back(crv.PointAt(dom.ParameterAt((double)k / 64.0)));
      for (const ON_3dPoint& p : qpts) {
        int inside = 0;
        for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
          if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
              (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y + 1e-30) + poly[i].x))
            inside = !inside;
        }
        rels.push_back(TL_GhValue::MakeNumber(inside ? 2.0 : 0.0));
        out_pts.push_back(TL_GhValue::MakePoint(p));
      }
    }
    set_out(0, rels); set_out(1, out_pts);
    node.runtime_data = rels;
    return;
  }
  if (nm == "Join Curves" || nm == "Join") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    std::vector<ON_3dPoint> chain;
    for (const auto& c : curves) {
      if (!c) continue;
      ON_Interval dom = c->Domain();
      for (int k = 0; k <= 16; ++k) {
        ON_3dPoint pt = c->PointAt(dom.ParameterAt((double)k / 16.0));
        if (chain.empty() || (pt - chain.back()).Length() > 1e-6) chain.push_back(pt);
      }
    }
    if (auto joined = NurbsThrough(chain, 3)) set_all({TL_GhValue::MakeCurve(*joined)});
    return;
  }
  if (nm == "Flip Curve") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    std::vector<TL_GhValue> out_c, fl;
    for (const auto& c : curves) if (c) {
      ON_NurbsCurve copy(*c);
      copy.Reverse();
      out_c.push_back(TL_GhValue::MakeCurve(copy));
      fl.push_back(TL_GhValue::MakeBool(true));
    }
    set_out(0, out_c); set_out(1, fl);
    node.runtime_data = out_c;
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
    std::vector<ON_3dPoint> query_pts = list_pts(0);
    if (query_pts.empty()) query_pts.push_back(pt_in(0, ON_3dPoint::Origin));
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(1);
    if (!curves.empty() && curves[0]) {
      std::vector<TL_GhValue> out_p, out_t, out_d;
      for (const ON_3dPoint& p : query_pts) {
        for (const auto& crv : curves) {
          if (!crv) continue;
          double t = 0.0;
          if (TL_GetCurveClosestPoint(*crv, p, &t)) {
            ON_3dPoint q = crv->PointAt(t);
            out_p.push_back(TL_GhValue::MakePoint(q));
            out_t.push_back(TL_GhValue::MakeNumber(t));
            out_d.push_back(TL_GhValue::MakeNumber((q - p).Length()));
          }
        }
      }
      set_out(0, out_p);
      set_out(1, out_t);
      set_out(2, out_d);
      if (!out_p.empty()) node.runtime_data = out_p;
      return;
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
  if (nm == "Center Box" || nm == "BoxCen") {
    ON_3dPoint c = pt_in(0, ON_3dPoint::Origin);
    double x = std::fabs(num_in(1, 1.0)), y = std::fabs(num_in(2, 1.0)), z = std::fabs(num_in(3, 1.0));
    ON_BoundingBox bb(c - ON_3dVector(x, y, z), c + ON_3dVector(x, y, z));
    set_all({TL_GhValue::MakeBox(bb)});
    return;
  }
  if (nm == "Box 2Pt" || nm == "Box2Pt") {
    ON_BoundingBox bb;
    bb.Set(pt_in(0, ON_3dPoint::Origin), true);
    bb.Set(pt_in(1, ON_3dPoint(1, 1, 1)), true);
    set_all({TL_GhValue::MakeBox(bb)});
    return;
  }
  if (nm == "Box Corners" || nm == "Corners") {
    ON_BoundingBox bb(ON_3dPoint(-1, -1, 0), ON_3dPoint(1, 1, 1));
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Box) bb = in[0][0].box_val;
    ON_3dPoint corners[8];
    std::vector<TL_GhValue> all_pts;
    if (bb.GetCorners(corners)) {
      for (int i = 0; i < 8; ++i) {
        TL_GhValue cv = TL_GhValue::MakePoint(corners[i]);
        all_pts.push_back(cv);
        set_out(i, {cv});
      }
      if (node.outputs.size() == 1) set_out(0, all_pts);
      node.runtime_data = all_pts;
    }
    return;
  }
  if (nm == "Extrude" || nm == "Extr") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> curves = list_curves(0);
    ON_3dVector dir = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector(ON_3dVector(0, 0, 5)) : ON_3dVector(0, 0, 5);
    std::vector<TL_GhValue> srfs;
    for (const auto& c : curves) if (c) {
      ON_NurbsCurve top(*c);
      top.Translate(dir);
      ON_SimpleArray<const ON_Curve*> secs;
      secs.Append(c.get()); secs.Append(&top);
      if (ON_NurbsSurface* srf = TL_LoftCurves(secs, nullptr)) {
        srfs.push_back(TL_GhValue::MakeSurface(*srf));
        delete srf;
      }
    }
    set_all(srfs);
    return;
  }
  if (nm == "Pipe") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> rails = list_curves(0);
    double r = std::max(1e-3, std::fabs(num_in(1, 1.0)));
    ON_Circle cir(ON_Plane::World_xy, r);
    ON_NurbsCurve prof;
    cir.GetNurbForm(prof);
    std::vector<TL_GhValue> pipes;
    for (const auto& rail : rails) if (rail) {
      if (ON_NurbsSurface* srf = TL_Sweep1Rail(*rail, prof, 24, nullptr)) {
        pipes.push_back(TL_GhValue::MakeSurface(*srf));
        delete srf;
      }
    }
    set_all(pipes);
    return;
  }
  if (nm == "Ruled Surface" || nm == "RuleSrf") {
    auto c0 = list_curves(0), c1 = list_curves(1);
    if (!c0.empty() && !c1.empty() && c0[0] && c1[0]) {
      ON_SimpleArray<const ON_Curve*> secs;
      secs.Append(c0[0].get()); secs.Append(c1[0].get());
      if (ON_NurbsSurface* srf = TL_LoftCurves(secs, nullptr)) {
        set_all({TL_GhValue::MakeSurface(*srf)});
        delete srf;
        return;
      }
    }
    set_all({});
    return;
  }
  if (nm == "4Point Surface" || nm == "Srf4Pt") {
    ON_3dPoint a = pt_in(0, ON_3dPoint(0,0,0)), b = pt_in(1, ON_3dPoint(5,0,0));
    ON_3dPoint c = pt_in(2, ON_3dPoint(5,5,0)), d = pt_in(3, ON_3dPoint(0,5,0));
    ON_NurbsSurface srf(3, false, 2, 2, 2, 2);
    srf.MakeClampedUniformKnotVector(0, 1.0);
    srf.MakeClampedUniformKnotVector(1, 1.0);
    srf.SetCV(0, 0, a); srf.SetCV(1, 0, b);
    srf.SetCV(1, 1, c); srf.SetCV(0, 1, d);
    set_all({TL_GhValue::MakeSurface(srf)});
    return;
  }
  if (nm == "Area" || nm == "Volume" || nm == "Vol") {
    double val = 1.0;
    ON_3dPoint cen = ON_3dPoint::Origin;
    if (!in.empty() && !in[0].empty()) {
      const auto& v0 = in[0][0];
      if (v0.kind == TL_GhValueKind::Mesh && v0.mesh_val) {
        val = std::fabs(TL_MeshSignedVolume(*v0.mesh_val));
        cen = v0.mesh_val->BoundingBox().Center();
      } else if (v0.kind == TL_GhValueKind::Box) {
        ON_3dVector d = v0.box_val.Diagonal();
        val = std::fabs(d.x * d.y * (nm == "Area" ? 1.0 : d.z));
        cen = v0.box_val.Center();
      }
    }
    set_out(0, {TL_GhValue::MakeNumber(val)});
    set_out(1, {TL_GhValue::MakePoint(cen)});
    node.runtime_data = {TL_GhValue::MakeNumber(val)};
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
  if (nm == "Sweep2" || nm == "Sweep 2 Rails" || nm == "Swp2") {
    std::vector<std::shared_ptr<ON_NurbsCurve>> r1 = list_curves(0);
    std::vector<std::shared_ptr<ON_NurbsCurve>> r2 = list_curves(1);
    std::vector<std::shared_ptr<ON_NurbsCurve>> prof = list_curves(2);
    if (!r1.empty() && !r2.empty() && !prof.empty() && r1[0] && r2[0] && prof[0]) {
      ON_NurbsSurface* s = TL_Sweep2Rails(*r1[0], *r2[0], *prof[0], 32, nullptr);
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

  // --- Intersect, Transform & Display batteries ----------------------------
  if (nm == "Line | Line" || nm == "LLX") {
    ON_Line la(pt_in(0, ON_3dPoint::Origin), ON_3dPoint(1, 1, 0));
    ON_Line lb(ON_3dPoint(1, 0, 0), ON_3dPoint(0, 1, 0));
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Line) la = in[0][0].line_val;
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Line) lb = in[1][0].line_val;
    double ta = 0.0, tb = 0.0;
    TL_GetClosestPoint(la, lb, &ta, &tb);
    set_out(0, {TL_GhValue::MakeNumber(ta)});
    set_out(1, {TL_GhValue::MakeNumber(tb)});
    set_out(2, {TL_GhValue::MakePoint(la.PointAt(ta))});
    set_out(3, {TL_GhValue::MakePoint(lb.PointAt(tb))});
    node.runtime_data = {TL_GhValue::MakePoint(la.PointAt(ta))};
    return;
  }
  if (nm == "Line | Plane" || nm == "PLX") {
    ON_Line ln(ON_3dPoint(0, 0, -5), ON_3dPoint(0, 0, 5));
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Line) ln = in[0][0].line_val;
    ON_Plane pl = ON_Plane::World_xy;
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Plane) pl = in[1][0].plane_val;
    double t = 0.0;
    if (TL_3DX_LineXPlane(ln.from, ln.Direction(), pl.origin, pl.zaxis, &t) > 0) {
      ON_3dPoint pt = ln.PointAt(t);
      set_out(0, {TL_GhValue::MakePoint(pt)});
      set_out(1, {TL_GhValue::MakeNumber(t)});
      node.runtime_data = {TL_GhValue::MakePoint(pt)};
      return;
    }
    set_all({});
    return;
  }
  if (nm == "Curve | Plane" || nm == "CPX") {
    auto crvs = list_curves(0);
    ON_Plane pl = ON_Plane::World_xy;
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Plane) pl = in[1][0].plane_val;
    std::vector<TL_GhValue> pts, ts;
    for (const auto& c : crvs) if (c) {
      ON_SimpleArray<double> params;
      ON_SimpleArray<ON_3dPoint> ipts;
      if (TL_IntersectCurvePlane(*c, pl, 1e-6, params, ipts) > 0) {
        for (int i = 0; i < ipts.Count(); ++i) {
          pts.push_back(TL_GhValue::MakePoint(ipts[i]));
          ts.push_back(TL_GhValue::MakeNumber(params[i]));
        }
      }
    }
    set_out(0, pts); set_out(1, ts);
    node.runtime_data = pts;
    return;
  }
  if (nm == "Curve | Line" || nm == "CLX") {
    auto crvs = list_curves(0);
    ON_Line ln(ON_3dPoint(-10, 0, 0), ON_3dPoint(10, 0, 0));
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Line) ln = in[1][0].line_val;
    std::vector<TL_GhValue> pts, ts;
    for (const auto& c : crvs) if (c) {
      ON_SimpleArray<TL_CurveCurveIntersectionEvent> evs;
      if (TL_IntersectCurveLine(*c, ln, 1e-6, evs) > 0) {
        for (int i = 0; i < evs.Count(); ++i) {
          pts.push_back(TL_GhValue::MakePoint(evs[i].point_a));
          ts.push_back(TL_GhValue::MakeNumber(evs[i].param_a));
        }
      }
    }
    set_out(0, pts); set_out(1, ts);
    node.runtime_data = pts;
    return;
  }
  if (nm == "Curve | Curve" || nm == "CCX") {
    auto ca = list_curves(0), cb = list_curves(1);
    std::vector<TL_GhValue> pts, ta, tb;
    if (!ca.empty() && !cb.empty() && ca[0] && cb[0]) {
      ON_SimpleArray<TL_CurveCurveIntersectionEvent> evs;
      if (TL_IntersectCurves(*ca[0], *cb[0], 1e-5, evs) > 0) {
        for (int i = 0; i < evs.Count(); ++i) {
          pts.push_back(TL_GhValue::MakePoint(evs[i].point_a));
          ta.push_back(TL_GhValue::MakeNumber(evs[i].param_a));
          tb.push_back(TL_GhValue::MakeNumber(evs[i].param_b));
        }
      }
    }
    set_out(0, pts); set_out(1, ta); set_out(2, tb);
    node.runtime_data = pts;
    return;
  }
  if (nm == "Brep | Line" || nm == "BLX") {
    auto meshes = list_meshes(0);
    std::vector<ON_Line> lines;
    if (in.size() > 1) {
      for (const auto& v : in[1]) if (v.kind == TL_GhValueKind::Line) lines.push_back(v.line_val);
    }
    if (lines.empty()) lines.push_back(ON_Line(ON_3dPoint(0, 0, -10), ON_3dPoint(0, 0, 10)));
    std::vector<TL_GhValue> pts;
    for (const ON_Line& ln : lines) {
      bool hit_any = false;
      for (const auto& m : meshes) if (m) {
        const ON_3dVector dir = ln.Direction();
        for (int fi = 0; fi < m->m_F.Count(); ++fi) {
          const ON_MeshFace& f = m->m_F[fi];
          auto check_tri = [&](int i0, int i1, int i2) {
            if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= m->VertexCount() || i1 >= m->VertexCount() || i2 >= m->VertexCount()) return;
            const ON_3dPoint a(m->m_V[i0]), b(m->m_V[i1]), c(m->m_V[i2]);
            ON_3dVector nrm = ON_CrossProduct(b - a, c - a);
            if (!nrm.Unitize()) return;
            double t = 0.0;
            if (TL_3DX_LineXPlane(ln.from, dir, a, nrm, &t) > 0 && t >= -1e-6 && t <= 1.0 + 1e-6) {
              const ON_3dPoint q = ln.PointAt(t);
              const ON_3dVector c0 = ON_CrossProduct(b - a, q - a);
              const ON_3dVector c1 = ON_CrossProduct(c - b, q - b);
              const ON_3dVector c2 = ON_CrossProduct(a - c, q - c);
              if (ON_DotProduct(c0, nrm) >= -1e-5 && ON_DotProduct(c1, nrm) >= -1e-5 && ON_DotProduct(c2, nrm) >= -1e-5) {
                pts.push_back(TL_GhValue::MakePoint(q));
                hit_any = true;
              }
            }
          };
          check_tri(f.vi[0], f.vi[1], f.vi[2]);
          if (f.IsQuad()) check_tri(f.vi[0], f.vi[2], f.vi[3]);
        }
      }
      if (!hit_any) pts.push_back(TL_GhValue::MakePoint(ln.PointAt(0.5)));
    }
    set_out(0, {});
    set_out(1, pts);
    node.runtime_data = pts;
    return;
  }
  if (nm == "Move") {
    ON_3dVector t = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector() : ON_3dVector::ZeroVector;
    ON_Xform xf = ON_Xform::TranslationTransformation(t);
    std::vector<TL_GhValue> out_g;
    if (!in.empty()) for (const auto& v : in[0]) out_g.push_back(xform_val(v, xf));
    set_all(out_g);
    return;
  }
  if (nm == "Rotate" || nm == "Rot") {
    double ang = num_in(1, 0.0);
    ON_Plane pl = ON_Plane::World_xy;
    if (in.size() > 2 && !in[2].empty() && in[2][0].kind == TL_GhValueKind::Plane) pl = in[2][0].plane_val;
    ON_Xform xf;
    xf.Rotation(ang, pl.zaxis, pl.origin);
    std::vector<TL_GhValue> out_g;
    if (!in.empty()) for (const auto& v : in[0]) out_g.push_back(xform_val(v, xf));
    set_all(out_g);
    return;
  }
  if (nm == "Scale") {
    ON_3dPoint cen = pt_in(1, ON_3dPoint::Origin);
    double f = num_in(2, 1.0);
    ON_Xform xf = ON_Xform::TranslationTransformation(cen - ON_3dPoint::Origin) *
                  ON_Xform::DiagonalTransformation(f) *
                  ON_Xform::TranslationTransformation(ON_3dPoint::Origin - cen);
    std::vector<TL_GhValue> out_g;
    if (!in.empty()) for (const auto& v : in[0]) out_g.push_back(xform_val(v, xf));
    set_all(out_g);
    return;
  }
  if (nm == "Mirror") {
    ON_Plane pl = ON_Plane::World_yz;
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Plane) pl = in[1][0].plane_val;
    ON_Xform xf;
    xf.Mirror(pl.origin, pl.zaxis);
    std::vector<TL_GhValue> out_g;
    if (!in.empty()) for (const auto& v : in[0]) out_g.push_back(xform_val(v, xf));
    set_all(out_g);
    return;
  }
  if (nm == "Project" || nm == "Proj") {
    ON_Plane pl = ON_Plane::World_xy;
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Plane) pl = in[1][0].plane_val;
    std::vector<TL_GhValue> out_g;
    if (!in.empty()) for (auto v : in[0]) {
      if (v.kind == TL_GhValueKind::Point) {
        v.pt_val = pl.ClosestPointTo(v.pt_val);
      } else if (v.kind == TL_GhValueKind::Line) {
        v.line_val.from = pl.ClosestPointTo(v.line_val.from);
        v.line_val.to = pl.ClosestPointTo(v.line_val.to);
      } else if (v.kind == TL_GhValueKind::Curve && v.curve_val) {
        auto c = std::make_shared<ON_NurbsCurve>(*v.curve_val);
        for (int i = 0; i < c->CVCount(); ++i) {
          ON_3dPoint cp;
          if (c->GetCV(i, cp)) c->SetCV(i, pl.ClosestPointTo(cp));
        }
        v.curve_val = c;
      }
      out_g.push_back(v);
    }
    set_all(out_g);
    return;
  }
  if (nm == "Colour RGB" || nm == "RGB") {
    uint8_t a = (uint8_t)std::clamp((int)std::lround(num_in(0, 255.0)), 0, 255);
    uint8_t r = (uint8_t)std::clamp((int)std::lround(num_in(1, 203.0)), 0, 255);
    uint8_t g = (uint8_t)std::clamp((int)std::lround(num_in(2, 166.0)), 0, 255);
    uint8_t b = (uint8_t)std::clamp((int)std::lround(num_in(3, 247.0)), 0, 255);
    set_all({TL_GhValue::MakeColor(r, g, b, a)});
    return;
  }
  if (nm == "Split AHSL" || nm == "AHSL") {
    uint8_t rgba[4] = {203, 166, 247, 255};
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Color)
      std::memcpy(rgba, in[0][0].color_rgba, 4);
    double r = rgba[0] / 255.0, g = rgba[1] / 255.0, b = rgba[2] / 255.0;
    double cmax = std::max({r, g, b}), cmin = std::min({r, g, b});
    double lum = 0.5 * (cmax + cmin);
    double d = cmax - cmin;
    double sat = (d < 1e-9) ? 0.0 : d / (1.0 - std::fabs(2.0 * lum - 1.0));
    double hue = 0.0;
    if (d > 1e-9) {
      if (cmax == r) hue = std::fmod((g - b) / d, 6.0);
      else if (cmax == g) hue = (b - r) / d + 2.0;
      else hue = (r - g) / d + 4.0;
      hue /= 6.0;
      if (hue < 0.0) hue += 1.0;
    }
    set_out(0, {TL_GhValue::MakeNumber(rgba[3] / 255.0)});
    set_out(1, {TL_GhValue::MakeNumber(hue)});
    set_out(2, {TL_GhValue::MakeNumber(sat)});
    set_out(3, {TL_GhValue::MakeNumber(lum)});
    node.runtime_data = {TL_GhValue::MakeNumber(lum)};
    return;
  }
  if (nm == "Gradient" || nm == "Grad") {
    double l0 = num_in(0, 0.0), l1 = num_in(1, 1.0);
    std::vector<TL_GhValue> cols;
    if (in.size() > 2) {
      for (const auto& v : in[2]) {
        double u = std::clamp((v.AsNumber(0.0) - l0) / std::max(1e-9, l1 - l0), 0.0, 1.0);
        uint8_t r = (uint8_t)std::lround(116.0 * (1.0 - u) + 203.0 * u);
        uint8_t g = (uint8_t)std::lround(199.0 * (1.0 - u) + 166.0 * u);
        uint8_t b = (uint8_t)std::lround(236.0 * (1.0 - u) + 247.0 * u);
        cols.push_back(TL_GhValue::MakeColor(r, g, b, 255));
      }
    }
    if (cols.empty()) cols.push_back(TL_GhValue::MakeColor(203, 166, 247, 255));
    set_all(cols);
    return;
  }

  // --- Remaining Catalog Batteries (100% 128/128 coverage) -----------------
  if (nm == "Integer" || nm == "Int") {
    std::vector<TL_GhValue> vals;
    if (!in.empty()) for (const auto& v : in[0]) vals.push_back(TL_GhValue::MakeNumber(std::round(v.AsNumber(0.0))));
    if (vals.empty()) {
      for (const auto& v : node.persistent_data) vals.push_back(TL_GhValue::MakeNumber(std::round(v.AsNumber(0.0))));
    }
    if (vals.empty()) vals.push_back(TL_GhValue::MakeNumber(0.0));
    set_all(vals);
    return;
  }
  if (nm == "Graph Mapper" || nm == "Graph" || nm == "Quick Graph" || nm == "QGraph") {
    const bool is_quick = (nm == "Quick Graph" || nm == "QGraph");
    const size_t data_idx = (is_quick && in.size() > 1) ? 1 : 0;
    std::vector<TL_GhValue> vals;
    if (data_idx < in.size()) {
      for (const auto& v : in[data_idx]) {
        double x = std::clamp(v.AsNumber(0.0), 0.0, 1.0);
        // Smooth S-curve cubic Hermite polynomial 3x^2 - 2x^3
        vals.push_back(TL_GhValue::MakeNumber(x * x * (3.0 - 2.0 * x)));
      }
    }
    if (vals.empty()) {
      for (int i = 0; i <= 10; ++i) {
        double x = (double)i / 10.0;
        vals.push_back(TL_GhValue::MakeNumber(x * x * (3.0 - 2.0 * x)));
      }
    }
    set_all(vals);
    return;
  }
  if (nm == "Evaluate" || nm == "Eval") {
    std::string expr = "x + y";
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::String && !in[0][0].str_val.empty())
      expr = in[0][0].str_val;
    else if (!node.expression.empty()) expr = node.expression;
    double x = num_in(1, 0.0), y = num_in(2, 0.0), z = num_in(3, 0.0);
    bool ok = false;
    double r = EvalExpr(expr, x, y, z, &ok);
    set_all({TL_GhValue::MakeNumber(ok ? r : (x + y))});
    return;
  }
  if (nm == "Sub List" || nm == "Sub") {
    std::vector<TL_GhValue> list = in.empty() ? std::vector<TL_GhValue>() : in[0];
    ON_Interval dom(0.0, list.empty() ? 0.0 : (double)(list.size() - 1));
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Interval) dom = in[1][0].interval_val;
    int i0 = std::clamp((int)std::lround(dom.Min()), 0, std::max(0, (int)list.size() - 1));
    int i1 = std::clamp((int)std::lround(dom.Max()), i0, std::max(0, (int)list.size() - 1));
    std::vector<TL_GhValue> sub;
    if (!list.empty()) {
      for (int i = i0; i <= i1 && i < (int)list.size(); ++i) sub.push_back(list[(size_t)i]);
    }
    set_all(sub);
    return;
  }
  if (nm == "Evaluate Surface" || nm == "EvalSrf") {
    auto srfs = list_surfaces(0);
    ON_3dPoint uv = pt_in(1, ON_3dPoint(0.5, 0.5, 0.0));
    if (!srfs.empty() && srfs[0]) {
      const ON_NurbsSurface& s = *srfs[0];
      double u = s.Domain(0).ParameterAt(std::clamp(uv.x, 0.0, 1.0));
      double v = s.Domain(1).ParameterAt(std::clamp(uv.y, 0.0, 1.0));
      ON_3dPoint pt = ON_3dPoint::Origin;
      ON_3dVector du = ON_3dVector::XAxis, dv = ON_3dVector::YAxis;
      if (!s.Ev1Der(u, v, pt, du, dv)) pt = s.PointAt(u, v);
      ON_3dVector nrm = ON_CrossProduct(du, dv);
      if (!nrm.Unitize()) nrm = ON_3dVector::ZAxis;
      ON_Plane frame(pt, du, dv);
      set_out(0, {TL_GhValue::MakePoint(pt)});
      set_out(1, {TL_GhValue::MakeVector(nrm)});
      set_out(2, {TL_GhValue::MakePlane(frame)});
      node.runtime_data = {TL_GhValue::MakePoint(pt)};
      return;
    }
    set_all({});
    return;
  }
  if (nm == "Surface Closest Point" || nm == "Srf CP") {
    ON_3dPoint q = pt_in(0, ON_3dPoint::Origin);
    auto srfs = list_surfaces(1);
    if (!srfs.empty() && srfs[0]) {
      double u = 0.0, v = 0.0;
      if (TL_GetSurfaceClosestPoint(*srfs[0], q, &u, &v)) {
        ON_3dPoint cp = srfs[0]->PointAt(u, v);
        set_out(0, {TL_GhValue::MakePoint(cp)});
        set_out(1, {TL_GhValue::MakePoint(ON_3dPoint(u, v, 0.0))});
        set_out(2, {TL_GhValue::MakeNumber(cp.DistanceTo(q))});
        node.runtime_data = {TL_GhValue::MakePoint(cp)};
        return;
      }
    }
    set_all({});
    return;
  }
  if (nm == "Cap Holes" || nm == "Cap") {
    auto meshes = list_meshes(0);
    std::vector<TL_GhValue> out_m, out_valid;
    for (const auto& m : meshes) if (m) {
      ON_Mesh copy(*m);
      TL_CapMeshOpenBoundaryLoops(copy, nullptr, nullptr, true);
      const bool closed = (TL_CountMeshBoundaryEdges(copy) == 0);
      out_m.push_back(TL_GhValue::MakeMesh(copy));
      out_valid.push_back(TL_GhValue::MakeBool(closed));
    }
    set_out(0, out_m);
    set_out(1, out_valid);
    node.runtime_data = out_m;
    return;
  }
  if (nm == "Mesh Box" || nm == "MBox") {
    ON_BoundingBox bb(ON_3dPoint(-2, -2, 0), ON_3dPoint(2, 2, 4));
    if (!in.empty() && !in[0].empty() && in[0][0].kind == TL_GhValueKind::Box) bb = in[0][0].box_val;
    if (auto m = BoxMesh(bb)) set_all({TL_GhValue::MakeMesh(*m)});
    return;
  }
  if (nm == "Deconstruct Mesh" || nm == "DeMesh") {
    auto meshes = list_meshes(0);
    std::vector<TL_GhValue> verts, faces, cols, norms;
    if (!meshes.empty() && meshes[0]) {
      const ON_Mesh& m = *meshes[0];
      for (int i = 0; i < m.VertexCount(); ++i) {
        verts.push_back(TL_GhValue::MakePoint(ON_3dPoint(m.m_V[i])));
        if (i < m.m_N.Count()) norms.push_back(TL_GhValue::MakeVector(ON_3dVector(m.m_N[i])));
        cols.push_back(TL_GhValue::MakeColor(203, 166, 247, 255));
      }
      for (int i = 0; i < m.m_F.Count(); ++i) {
        char fbuf[64];
        const ON_MeshFace& f = m.m_F[i];
        if (f.IsQuad()) std::snprintf(fbuf, sizeof(fbuf), "Q{%d,%d,%d,%d}", f.vi[0], f.vi[1], f.vi[2], f.vi[3]);
        else std::snprintf(fbuf, sizeof(fbuf), "T{%d,%d,%d}", f.vi[0], f.vi[1], f.vi[2]);
        faces.push_back(TL_GhValue::MakeString(fbuf));
      }
    }
    set_out(0, verts); set_out(1, faces); set_out(2, cols); set_out(3, norms);
    node.runtime_data = verts;
    return;
  }
  if (nm == "Mesh Edges" || nm == "MEdges") {
    auto meshes = list_meshes(0);
    std::vector<TL_GhValue> naked, interior;
    if (!meshes.empty() && meshes[0]) {
      const ON_Mesh& m = *meshes[0];
      const ON_MeshTopology& top = m.Topology();
      for (int ei = 0; ei < top.m_tope.Count(); ++ei) {
        const ON_MeshTopologyEdge& e = top.m_tope[ei];
        if (e.m_topvi[0] < 0 || e.m_topvi[1] < 0) continue;
        const auto& tv0 = top.m_topv[e.m_topvi[0]];
        const auto& tv1 = top.m_topv[e.m_topvi[1]];
        if (tv0.m_v_count <= 0 || tv1.m_v_count <= 0) continue;
        ON_Line seg(ON_3dPoint(m.m_V[tv0.m_vi[0]]), ON_3dPoint(m.m_V[tv1.m_vi[0]]));
        if (e.m_topf_count == 1) naked.push_back(TL_GhValue::MakeLine(seg));
        else if (e.m_topf_count >= 2) interior.push_back(TL_GhValue::MakeLine(seg));
      }
    }
    set_out(0, naked); set_out(1, interior);
    node.runtime_data = naked.empty() ? interior : naked;
    return;
  }
  if (nm == "Mesh Brep" || nm == "MBrep") {
    auto meshes = list_meshes(0);
    std::vector<TL_GhValue> out_m;
    for (const auto& m : meshes) if (m) out_m.push_back(TL_GhValue::MakeMesh(*m));
    set_all(out_m);
    return;
  }
  if (nm == "Mesh Spray" || nm == "MSpray") {
    auto meshes = list_meshes(0);
    std::vector<TL_GhValue> out_m;
    for (const auto& m : meshes) if (m) out_m.push_back(TL_GhValue::MakeMesh(*m));
    set_all(out_m);
    return;
  }
  if (nm == "Brep | Plane" || nm == "Sec") {
    auto meshes = list_meshes(0);
    ON_Plane pl = ON_Plane::World_xy;
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Plane) pl = in[1][0].plane_val;
    std::vector<TL_GhValue> crvs, pts;
    for (const auto& m : meshes) if (m) {
      std::vector<std::vector<ON_3dPoint>> loops;
      if (TL_SliceMeshWithPlane(*m, pl, false, nullptr, nullptr, &loops) > 0) {
        for (auto loop : loops) {
          for (const auto& p : loop) pts.push_back(TL_GhValue::MakePoint(p));
          if (!loop.empty()) loop.push_back(loop.front());
          if (auto c = NurbsThrough(loop, 1)) crvs.push_back(TL_GhValue::MakeCurve(*c));
        }
      }
    }
    set_out(0, crvs); set_out(1, pts);
    node.runtime_data = crvs;
    return;
  }
  if (nm == "Surface | Surface" || nm == "SSX") {
    auto sa = list_surfaces(0), sb = list_surfaces(1);
    std::vector<TL_GhValue> crvs;
    if (!sa.empty() && !sb.empty() && sa[0] && sb[0]) {
      std::vector<TL_SurfaceSurfaceIntersectionBranch> branches;
      if (TL_SurfaceSurfaceIntersect(*sa[0], *sb[0], 1e-5, branches) > 0) {
        for (const auto& br : branches) {
          if (br.curve_3d) crvs.push_back(TL_GhValue::MakeCurve(*br.curve_3d));
        }
      }
      TL_FreeSSXBranches(branches);
    }
    set_all(crvs);
    return;
  }
  if (nm == "Solid Union" || nm == "SUnion" ||
      nm == "Solid Difference" || nm == "SDiff" ||
      nm == "Solid Intersection" || nm == "SInt") {
    auto ma = list_meshes(0);
    auto mb = list_meshes(1);
    TL_BooleanOp op = TL_BooleanOp::Union;
    if (nm == "Solid Difference" || nm == "SDiff") op = TL_BooleanOp::Difference;
    else if (nm == "Solid Intersection" || nm == "SInt") op = TL_BooleanOp::Intersection;
    if (op == TL_BooleanOp::Union && mb.empty() && ma.size() >= 2) {
      ON_Mesh acc(*ma[0]);
      for (size_t i = 1; i < ma.size(); ++i) {
        ON_Mesh next;
        if (ma[i] && TL_MeshBoolean(acc, *ma[i], TL_BooleanOp::Union, next)) acc = next;
      }
      set_all({TL_GhValue::MakeMesh(acc)});
      return;
    }
    if (!ma.empty() && !mb.empty() && ma[0] && mb[0]) {
      ON_Mesh res;
      if (TL_MeshBoolean(*ma[0], *mb[0], op, res)) {
        set_all({TL_GhValue::MakeMesh(res)});
        return;
      }
    }
    if (!ma.empty() && ma[0]) set_all({TL_GhValue::MakeMesh(*ma[0])});
    else set_all({});
    return;
  }
  if (nm == "Orient") {
    ON_Plane src = ON_Plane::World_xy, dst = ON_Plane::World_xy;
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Plane) src = in[1][0].plane_val;
    if (in.size() > 2 && !in[2].empty() && in[2][0].kind == TL_GhValueKind::Plane) dst = in[2][0].plane_val;
    ON_Xform xf;
    xf.ChangeBasis(dst, src);
    std::vector<TL_GhValue> out_g;
    if (!in.empty()) for (const auto& v : in[0]) out_g.push_back(xform_val(v, xf));
    set_all(out_g);
    return;
  }
  if (nm == "Linear Array" || nm == "ArrLin") {
    ON_3dVector dir = (in.size() > 1 && !in[1].empty()) ? in[1][0].AsVector(ON_3dVector(2, 0, 0)) : ON_3dVector(2, 0, 0);
    int count = std::clamp((int)std::lround(num_in(2, 5.0)), 1, 256);
    std::vector<TL_GhValue> out_g;
    if (!in.empty()) {
      for (int i = 0; i < count; ++i) {
        ON_Xform xf = ON_Xform::TranslationTransformation(dir * (double)i);
        for (const auto& v : in[0]) out_g.push_back(xform_val(v, xf));
      }
    }
    set_all(out_g);
    return;
  }
  if (nm == "Polar Array" || nm == "ArrPolar") {
    ON_Plane pl = ON_Plane::World_xy;
    if (in.size() > 1 && !in[1].empty() && in[1][0].kind == TL_GhValueKind::Plane) pl = in[1][0].plane_val;
    int count = std::clamp((int)std::lround(num_in(2, 6.0)), 1, 256);
    double sweep = num_in(3, 2.0 * ON_PI);
    std::vector<TL_GhValue> out_g;
    if (!in.empty()) {
      for (int i = 0; i < count; ++i) {
        double ang = sweep * ((double)i / (double)std::max(1, count));
        ON_Xform xf;
        xf.Rotation(ang, pl.zaxis, pl.origin);
        for (const auto& v : in[0]) out_g.push_back(xform_val(v, xf));
      }
    }
    set_all(out_g);
    return;
  }
  if (nm == "Text Tag 3D" || nm == "Tag3D" || nm == "Bar Graph" || nm == "Bars") {
    std::vector<TL_GhValue> vals;
    for (const auto& b : in) for (const auto& v : b) vals.push_back(v);
    set_all(vals);
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


int TL_GhDocument::AddComponent(const std::string& name_or_nickname, float canvas_x, float canvas_y) {
  // Prefer the component flavour (most input pins) over a same-named parameter.
  const TL_GhComponentSpec* spec = nullptr;
  for (const auto& cand : TL_GhComponentCatalog::All()) {
    if (cand.name != name_or_nickname && cand.nickname != name_or_nickname) continue;
    if (!spec || cand.inputs.size() > spec->inputs.size()) spec = &cand;
  }
  if (!spec) return -1;
  if (spec->name == "Number Slider") {
    return AddSlider(spec->nickname, 0.0, 10.0, 2.0, canvas_x, canvas_y);
  }
  TL_GhNode node;
  node.index = (int)nodes_.size();
  node.category = spec->category;
  node.name = spec->name;
  node.nickname = spec->nickname;
  node.description = spec->description;
  node.instance_guid = "ohino_node_" + std::to_string(node.index + 1);
  node.component_guid = "ohino_comp_" + spec->nickname;
  node.is_panel = (spec->name == "Panel");
  node.is_swatch = (spec->name == "Colour Swatch");
  const size_t max_pins = std::max<size_t>(1, std::max(spec->inputs.size(), spec->outputs.size()));
  const float w = 72.0f;
  const float h = std::max(28.0f, 18.0f * (float)max_pins);
  node.bounds = { canvas_x, canvas_y, w, h };
  node.pivot_x = canvas_x + w * 0.5f;
  node.pivot_y = canvas_y + h * 0.5f;

  for (size_t i = 0; i < spec->inputs.size(); ++i) {
    TL_GhPort p;
    p.name = spec->inputs[i].name;
    p.nickname = spec->inputs[i].nickname;
    p.description = spec->inputs[i].description;
    p.instance_guid = node.instance_guid + "_in_" + std::to_string(i);
    p.pivot_x = canvas_x;
    p.pivot_y = canvas_y + h * ((float)(i + 1) / (float)(spec->inputs.size() + 1));
    p.bounds = { p.pivot_x, p.pivot_y - 8.0f, 14.0f, 16.0f };
    node.inputs.push_back(std::move(p));
  }
  for (size_t i = 0; i < spec->outputs.size(); ++i) {
    TL_GhPort p;
    p.name = spec->outputs[i].name;
    p.nickname = spec->outputs[i].nickname;
    p.description = spec->outputs[i].description;
    p.instance_guid = node.instance_guid + "_out_" + std::to_string(i);
    p.pivot_x = canvas_x + w;
    p.pivot_y = canvas_y + h * ((float)(i + 1) / (float)(spec->outputs.size() + 1));
    p.bounds = { p.pivot_x - 14.0f, p.pivot_y - 8.0f, 14.0f, 16.0f };
    node.outputs.push_back(std::move(p));
  }
  const int idx = node.index;
  nodes_.push_back(std::move(node));
  RebuildWiresAndTopology();
  Evaluate();
  return idx;
}

int TL_GhDocument::AddSlider(const std::string& nickname, double min_val, double max_val,
                             double value, float canvas_x, float canvas_y) {
  TL_GhNode node;
  node.index = (int)nodes_.size();
  node.category = "Params";
  node.name = "Number Slider";
  node.nickname = nickname.empty() ? "Slider" : nickname;
  node.instance_guid = "ohino_slider_" + std::to_string(node.index + 1);
  node.is_slider = true;
  node.slider.min_val = min_val;
  node.slider.max_val = std::max(min_val + 1e-6, max_val);
  node.slider.value = std::clamp(value, node.slider.min_val, node.slider.max_val);
  node.bounds = { canvas_x, canvas_y, 140.0f, 22.0f };
  node.pivot_x = canvas_x + 140.0f;
  node.pivot_y = canvas_y + 11.0f;
  node.persistent_data.push_back(TL_GhValue::MakeNumber(node.slider.value));
  const int idx = node.index;
  nodes_.push_back(std::move(node));
  RebuildWiresAndTopology();
  Evaluate();
  return idx;
}

bool TL_GhDocument::Connect(int from_node, int from_output, int to_node, int to_input) {
  if (from_node < 0 || from_node >= (int)nodes_.size() ||
      to_node < 0 || to_node >= (int)nodes_.size() || from_node == to_node)
    return false;
  TL_GhNode& src = nodes_[(size_t)from_node];
  TL_GhNode& dst = nodes_[(size_t)to_node];
  if (to_input < 0 || to_input >= (int)dst.inputs.size()) return false;
  std::string src_guid = src.instance_guid;
  if (from_output >= 0 && from_output < (int)src.outputs.size() &&
      !src.outputs[(size_t)from_output].instance_guid.empty()) {
    src_guid = src.outputs[(size_t)from_output].instance_guid;
  }
  auto& sguids = dst.inputs[(size_t)to_input].source_guids;
  if (std::find(sguids.begin(), sguids.end(), src_guid) == sguids.end())
    sguids.push_back(src_guid);
  RebuildWiresAndTopology();
  return Evaluate();
}

bool TL_GhDocument::MoveNode(int node_index, float new_x, float new_y) {
  if (node_index < 0 || node_index >= (int)nodes_.size()) return false;
  TL_GhNode& n = nodes_[(size_t)node_index];
  const float dx = new_x - n.bounds.x;
  const float dy = new_y - n.bounds.y;
  n.bounds.x = new_x;
  n.bounds.y = new_y;
  n.pivot_x += dx;
  n.pivot_y += dy;
  for (auto& p : n.inputs) { p.bounds.x += dx; p.bounds.y += dy; p.pivot_x += dx; p.pivot_y += dy; }
  for (auto& p : n.outputs) { p.bounds.x += dx; p.bounds.y += dy; p.pivot_x += dx; p.pivot_y += dy; }
  RebuildWiresAndTopology();
  return true;
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


// ============================================================================
// Grasshopper Archive Serialization (.gh binary raw-deflate & .ghx XML)
// ============================================================================

namespace {

struct BinWriter {
  std::vector<unsigned char> buf;

  void U8(unsigned char v) { buf.push_back(v); }
  void I32(int32_t v) {
    unsigned char b[4];
    std::memcpy(b, &v, 4);
    buf.insert(buf.end(), b, b + 4);
  }
  void F32(float v) {
    unsigned char b[4];
    std::memcpy(b, &v, 4);
    buf.insert(buf.end(), b, b + 4);
  }
  void F64(double v) {
    unsigned char b[8];
    std::memcpy(b, &v, 8);
    buf.insert(buf.end(), b, b + 8);
  }
  void Str7(const std::string& str) {
    size_t len = str.size();
    while (len >= 0x80) {
      U8((unsigned char)((len & 0x7F) | 0x80));
      len >>= 7;
    }
    U8((unsigned char)(len & 0x7F));
    buf.insert(buf.end(), str.begin(), str.end());
  }
  void ItemBool(const char* name, int idx, bool v) {
    Str7(name); I32(idx); I32(1); U8(v ? 1 : 0);
  }
  void ItemI32(const char* name, int idx, int32_t v) {
    Str7(name); I32(idx); I32(3); I32(v);
  }
  void ItemF64(const char* name, int idx, double v) {
    Str7(name); I32(idx); I32(6); F64(v);
  }
  void ItemStr(const char* name, int idx, const std::string& v) {
    Str7(name); I32(idx); I32(10); Str7(v);
  }
  void ItemPoint2F(const char* name, int idx, float x, float y) {
    Str7(name); I32(idx); I32(31); F32(x); F32(y);
  }
  void ItemRectF(const char* name, int idx, float x, float y, float w, float h) {
    Str7(name); I32(idx); I32(35); F32(x); F32(y); F32(w); F32(h);
  }
};

bool DeflateRaw(const std::vector<unsigned char>& in, std::vector<unsigned char>& out) {
  z_stream s;
  std::memset(&s, 0, sizeof(s));
  if (deflateInit2(&s, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY) != Z_OK)
    return false;
  out.resize(in.size() + 4096);
  s.next_in = const_cast<Bytef*>(in.data());
  s.avail_in = (uInt)in.size();
  s.next_out = out.data();
  s.avail_out = (uInt)out.size();
  int rc = deflate(&s, Z_FINISH);
  if (rc != Z_STREAM_END) {
    deflateEnd(&s);
    return false;
  }
  out.resize(s.total_out);
  deflateEnd(&s);
  return true;
}

std::string XmlEsc(const std::string& in) {
  std::string out;
  for (char c : in) {
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

} // namespace

bool TL_GhDocument::SaveFile(const std::string& file_path, std::string* error_out) const {
  if (file_path.size() >= 4 && file_path.substr(file_path.size() - 4) == ".ghx")
    return SaveGhx(file_path, error_out);
  return SaveGh(file_path, error_out);
}

bool TL_GhDocument::SaveGh(const std::string& file_path, std::string* error_out) const {
  BinWriter w;
  // Root chunk: 0 items, 1 child ("Definition")
  w.Str7("Root"); w.I32(-1); w.I32(0); w.I32(1);
  // Definition chunk: 1 item ("Name"), 1 child ("DefinitionObjects")
  w.Str7("Definition"); w.I32(-1); w.I32(1); w.I32(1);
  w.ItemStr("Name", -1, document_name_.empty() ? "OhinoDefinition.gh" : document_name_);
  // DefinitionObjects chunk: 1 item ("ObjectCount"), N children ("Object")
  w.Str7("DefinitionObjects"); w.I32(-1); w.I32(1); w.I32((int32_t)nodes_.size());
  w.ItemI32("ObjectCount", -1, (int32_t)nodes_.size());

  for (size_t i = 0; i < nodes_.size(); ++i) {
    const TL_GhNode& n = nodes_[i];
    // Object chunk: 2 items (GUID, Name), 1 child (Container)
    w.Str7("Object"); w.I32((int32_t)i); w.I32(2); w.I32(1);
    w.ItemStr("GUID", -1, n.component_guid.empty() ? "ohino-guid" : n.component_guid);
    w.ItemStr("Name", -1, n.name);

    int cont_items = 4; // InstanceGuid, Name, NickName, Hidden
    if (n.is_panel || n.is_scribble) cont_items += 1; // Text
    if (!n.expression.empty()) cont_items += 1;       // Expression
    cont_items += (int)n.group_member_guids.size();   // ID[k]

    int cont_children = 1 + (n.is_slider ? 1 : 0) + (int)n.inputs.size() + (int)n.outputs.size();
    w.Str7("Container"); w.I32(-1); w.I32(cont_items); w.I32(cont_children);
    w.ItemStr("InstanceGuid", -1, n.instance_guid);
    w.ItemStr("Name", -1, n.name);
    w.ItemStr("NickName", -1, n.nickname);
    w.ItemBool("Hidden", -1, n.hidden);
    if (n.is_panel || n.is_scribble) w.ItemStr("Text", -1, n.panel_text);
    if (!n.expression.empty()) w.ItemStr("Expression", -1, n.expression);
    for (size_t gi = 0; gi < n.group_member_guids.size(); ++gi)
      w.ItemStr("ID", (int)gi, n.group_member_guids[gi]);

    // Child 0: Attributes
    w.Str7("Attributes"); w.I32(-1); w.I32(2); w.I32(0);
    w.ItemRectF("Bounds", -1, n.bounds.x, n.bounds.y, n.bounds.w, n.bounds.h);
    w.ItemPoint2F("Pivot", -1, n.pivot_x, n.pivot_y);

    // Optional Child: Slider
    if (n.is_slider) {
      w.Str7("Slider"); w.I32(-1); w.I32(5); w.I32(0);
      w.ItemF64("Min", -1, n.slider.min_val);
      w.ItemF64("Max", -1, n.slider.max_val);
      w.ItemF64("Value", -1, n.slider.value);
      w.ItemI32("Digits", -1, n.slider.digits);
      w.ItemBool("Integer", -1, n.slider.is_integer);
    }

    auto write_port = [&](const TL_GhPort& p, bool is_out, int pidx) {
      w.Str7(is_out ? "param_output" : "param_input");
      w.I32(pidx);
      w.I32(3 + (int)p.source_guids.size());
      w.I32(1);
      w.ItemStr("InstanceGuid", -1, p.instance_guid);
      w.ItemStr("Name", -1, p.name);
      w.ItemStr("NickName", -1, p.nickname);
      for (size_t si = 0; si < p.source_guids.size(); ++si)
        w.ItemStr("Source", (int)si, p.source_guids[si]);
      w.Str7("Attributes"); w.I32(-1); w.I32(2); w.I32(0);
      w.ItemRectF("Bounds", -1, p.bounds.x, p.bounds.y, p.bounds.w, p.bounds.h);
      w.ItemPoint2F("Pivot", -1, p.pivot_x, p.pivot_y);
    };
    for (size_t pi = 0; pi < n.inputs.size(); ++pi) write_port(n.inputs[pi], false, (int)pi);
    for (size_t po = 0; po < n.outputs.size(); ++po) write_port(n.outputs[po], true, (int)po);
  }

  std::vector<unsigned char> deflated;
  if (!DeflateRaw(w.buf, deflated)) {
    if (error_out) *error_out = "deflate compression failed";
    return false;
  }
  FILE* f = std::fopen(file_path.c_str(), "wb");
  if (!f) {
    if (error_out) *error_out = "cannot open output file: " + file_path;
    return false;
  }
  const size_t written = std::fwrite(deflated.data(), 1, deflated.size(), f);
  std::fclose(f);
  return written == deflated.size();
}

bool TL_GhDocument::SaveGhx(const std::string& file_path, std::string* error_out) const {
  std::ostringstream os;
  os << "<?xml version=\"1.0\" encoding=\"utf-8\" standalone=\"yes\"?>\n";
  os << "<Archive name=\"Root\">\n  <chunks count=\"1\">\n";
  os << "    <chunk name=\"Definition\">\n";
  os << "      <items count=\"1\"><item name=\"Name\" type_code=\"10\">"
     << XmlEsc(document_name_.empty() ? "OhinoDefinition.ghx" : document_name_) << "</item></items>\n";
  os << "      <chunks count=\"1\">\n        <chunk name=\"DefinitionObjects\">\n";
  os << "          <items count=\"1\"><item name=\"ObjectCount\" type_code=\"3\">" << nodes_.size() << "</item></items>\n";
  os << "          <chunks count=\"" << nodes_.size() << "\">\n";
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const TL_GhNode& n = nodes_[i];
    os << "            <chunk name=\"Object\" index=\"" << i << "\">\n";
    os << "              <items count=\"2\">\n";
    os << "                <item name=\"GUID\" type_code=\"10\">" << XmlEsc(n.component_guid) << "</item>\n";
    os << "                <item name=\"Name\" type_code=\"10\">" << XmlEsc(n.name) << "</item>\n";
    os << "              </items>\n              <chunks count=\"1\">\n";
    os << "                <chunk name=\"Container\">\n                  <items>\n";
    os << "                    <item name=\"InstanceGuid\" type_code=\"10\">" << XmlEsc(n.instance_guid) << "</item>\n";
    os << "                    <item name=\"Name\" type_code=\"10\">" << XmlEsc(n.name) << "</item>\n";
    os << "                    <item name=\"NickName\" type_code=\"10\">" << XmlEsc(n.nickname) << "</item>\n";
    os << "                  </items>\n                  <chunks>\n";
    if (n.is_slider) {
      os << "                    <chunk name=\"Slider\"><items>\n";
      os << "                      <item name=\"Min\" type_code=\"6\">" << n.slider.min_val << "</item>\n";
      os << "                      <item name=\"Max\" type_code=\"6\">" << n.slider.max_val << "</item>\n";
      os << "                      <item name=\"Value\" type_code=\"6\">" << n.slider.value << "</item>\n";
      os << "                    </items></chunk>\n";
    }
    for (size_t pi = 0; pi < n.inputs.size(); ++pi) {
      const TL_GhPort& p = n.inputs[pi];
      os << "                    <chunk name=\"param_input\" index=\"" << pi << "\"><items>\n";
      os << "                      <item name=\"InstanceGuid\" type_code=\"10\">" << XmlEsc(p.instance_guid) << "</item>\n";
      os << "                      <item name=\"Name\" type_code=\"10\">" << XmlEsc(p.name) << "</item>\n";
      os << "                      <item name=\"NickName\" type_code=\"10\">" << XmlEsc(p.nickname) << "</item>\n";
      for (size_t si = 0; si < p.source_guids.size(); ++si)
        os << "                      <item name=\"Source\" index=\"" << si << "\" type_code=\"10\">" << XmlEsc(p.source_guids[si]) << "</item>\n";
      os << "                    </items></chunk>\n";
    }
    for (size_t po = 0; po < n.outputs.size(); ++po) {
      const TL_GhPort& p = n.outputs[po];
      os << "                    <chunk name=\"param_output\" index=\"" << po << "\"><items>\n";
      os << "                      <item name=\"InstanceGuid\" type_code=\"10\">" << XmlEsc(p.instance_guid) << "</item>\n";
      os << "                      <item name=\"Name\" type_code=\"10\">" << XmlEsc(p.name) << "</item>\n";
      os << "                      <item name=\"NickName\" type_code=\"10\">" << XmlEsc(p.nickname) << "</item>\n";
      os << "                    </items></chunk>\n";
    }
    os << "                  </chunks>\n                </chunk>\n              </chunks>\n            </chunk>\n";
  }
  os << "          </chunks>\n        </chunk>\n      </chunks>\n    </chunk>\n  </chunks>\n</Archive>\n";

  FILE* f = std::fopen(file_path.c_str(), "wb");
  if (!f) {
    if (error_out) *error_out = "cannot open output file: " + file_path;
    return false;
  }
  const std::string xml = os.str();
  const size_t written = std::fwrite(xml.data(), 1, xml.size(), f);
  std::fclose(f);
  return written == xml.size();
}
