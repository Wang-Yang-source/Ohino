#include "opentl/viewport/tl_viewport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace {

constexpr double kPi = 3.14159265358979323846;

double Deg2Rad(double deg) { return deg * kPi / 180.0; }

/** 36mm film-back half-height -> vertical field of view. */
double FovFromLens(double lens_mm)
{
  const double lens = (lens_mm > 1.0) ? lens_mm : 50.0;
  // 36 x 24 mm frame; Rhino's lens length is the 36mm-equivalent focal length.
  return 2.0 * std::atan(12.0 / lens);
}

std::string FormatDouble(double value)
{
  if (!ON_IsValid(value))
    return "0";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", value);
  return std::string(buf);
}

/** Minimal JSON scanner for the layout files this module writes. */
struct MiniJson
{
  std::string text;
  size_t pos = 0;

  void SkipWs()
  {
    while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t' ||
                                 text[pos] == '\n' || text[pos] == '\r'))
      ++pos;
  }

  bool Eat(char c)
  {
    SkipWs();
    if (pos < text.size() && text[pos] == c)
    {
      ++pos;
      return true;
    }
    return false;
  }

  bool ParseString(std::string& out)
  {
    SkipWs();
    if (pos >= text.size() || text[pos] != '"')
      return false;
    ++pos;
    out.clear();
    while (pos < text.size() && text[pos] != '"')
    {
      if (text[pos] == '\\' && pos + 1 < text.size())
        ++pos;
      out.push_back(text[pos++]);
    }
    if (pos < text.size())
      ++pos;
    return true;
  }

  bool ParseNumber(double& out)
  {
    SkipWs();
    const char* start = text.c_str() + pos;
    char* end = nullptr;
    const double v = std::strtod(start, &end);
    if (end == start)
      return false;
    pos += static_cast<size_t>(end - start);
    out = v;
    return true;
  }

  bool ParseBool(bool& out)
  {
    SkipWs();
    if (text.compare(pos, 4, "true") == 0) { pos += 4; out = true; return true; }
    if (text.compare(pos, 5, "false") == 0) { pos += 5; out = false; return true; }
    return false;
  }

  /** Skip any value (used for unknown members so the reader stays forward-compatible). */
  bool SkipValue()
  {
    SkipWs();
    if (pos >= text.size())
      return false;
    const char c = text[pos];
    if (c == '"')
    {
      std::string ignored;
      return ParseString(ignored);
    }
    if (c == '{' || c == '[')
    {
      const char close = (c == '{') ? '}' : ']';
      ++pos;
      while (pos < text.size())
      {
        SkipWs();
        if (text[pos] == close) { ++pos; return true; }
        if (text[pos] == ',') { ++pos; continue; }
        if (!SkipValue())
          return false;
        SkipWs();
        if (pos < text.size() && text[pos] == ':')
        {
          ++pos;
          if (!SkipValue())
            return false;
        }
      }
      return false;
    }
    double d = 0.0;
    if (ParseNumber(d))
      return true;
    bool b = false;
    if (ParseBool(b))
      return true;
    if (text.compare(pos, 4, "null") == 0) { pos += 4; return true; }
    return false;
  }

  bool ParseNumberArray(std::vector<double>& out)
  {
    if (!Eat('['))
      return false;
    out.clear();
    SkipWs();
    if (Eat(']'))
      return true;
    for (;;)
    {
      double v = 0.0;
      if (!ParseNumber(v))
        return false;
      out.push_back(v);
      SkipWs();
      if (Eat(']'))
        return true;
      if (!Eat(','))
        return false;
    }
  }
};

} // anonymous namespace

// ============================================================================
// TL_Viewport
// ============================================================================

TL_Viewport::TL_Viewport() = default;

TL_Viewport::TL_Viewport(const std::string& name, int standard_view)
  : m_name(name)
{
  SetStandardView(standard_view);
  m_name = name;
}

ON_3dVector TL_Viewport::ScreenRight() const
{
  ON_3dVector right = ON_CrossProduct(m_camera_dir, m_up);
  if (!right.Unitize())
  {
    // Degenerate up vector: fall back to a world axis that is not parallel to the view.
    const ON_3dVector fallback = (std::fabs(m_camera_dir.z) < 0.9)
      ? ON_3dVector(0.0, 0.0, 1.0)
      : ON_3dVector(1.0, 0.0, 0.0);
    right = ON_CrossProduct(m_camera_dir, fallback);
    if (!right.Unitize())
      right = ON_3dVector(1.0, 0.0, 0.0);
  }
  return right;
}

ON_3dVector TL_Viewport::ScreenUp() const
{
  ON_3dVector up = ON_CrossProduct(ScreenRight(), m_camera_dir);
  if (!up.Unitize())
    up = ON_3dVector(0.0, 1.0, 0.0);
  return up;
}

ON_3dPoint TL_Viewport::CameraLocation() const
{
  if (TL_PROJECTION_PERSPECTIVE == m_projection)
  {
    const double dist = (m_target_distance > 1e-9) ? m_target_distance : 10.0;
    return m_target - m_camera_dir * dist;
  }
  return m_target;
}

double TL_Viewport::VerticalFieldOfView() const
{
  return FovFromLens(m_lens_mm);
}

bool TL_Viewport::IsValid() const
{
  return m_width > 0 && m_height > 0 &&
         ON_IsValid(m_scale) && m_scale > 0.0 &&
         !m_camera_dir.IsZero() && !m_up.IsZero();
}

bool TL_Viewport::WorldToScreen(const ON_3dPoint& world, ON_2dPoint& screen) const
{
  const ON_3dVector right = ScreenRight();
  const ON_3dVector up = ScreenUp();
  const double half_w = 0.5 * static_cast<double>(m_width);
  const double half_h = 0.5 * static_cast<double>(m_height);
  const ON_3dPoint center(static_cast<double>(m_left) + half_w,
                          static_cast<double>(m_top) + half_h, 0.0);

  if (TL_PROJECTION_PERSPECTIVE == m_projection)
  {
    const ON_3dPoint eye = CameraLocation();
    const ON_3dVector v = world - eye;
    const double z = ON_DotProduct(v, m_camera_dir);
    if (!(z > 1e-9))
      return false; // behind the eye
    const double tan_half = std::tan(0.5 * VerticalFieldOfView());
    const double aspect = static_cast<double>(m_width) / static_cast<double>(m_height);
    const double ndc_x = ON_DotProduct(v, right) / (z * tan_half * aspect);
    const double ndc_y = ON_DotProduct(v, up) / (z * tan_half);
    screen.x = center.x + ndc_x * half_w;
    screen.y = center.y - ndc_y * half_h;
    return ON_IsValid(screen.x) && ON_IsValid(screen.y);
  }

  const ON_3dVector d = world - m_target;
  screen.x = center.x + ON_DotProduct(d, right) * m_scale;
  screen.y = center.y - ON_DotProduct(d, up) * m_scale;
  return ON_IsValid(screen.x) && ON_IsValid(screen.y);
}

bool TL_Viewport::ScreenToRay(const ON_2dPoint& screen, ON_3dPoint& ray_from, ON_3dVector& ray_dir) const
{
  const ON_3dVector right = ScreenRight();
  const ON_3dVector up = ScreenUp();
  const double half_w = 0.5 * static_cast<double>(m_width);
  const double half_h = 0.5 * static_cast<double>(m_height);
  const ON_2dPoint center(static_cast<double>(m_left) + half_w,
                          static_cast<double>(m_top) + half_h);
  const double px = screen.x - center.x;   // +right
  const double py = center.y - screen.y;   // +up

  if (TL_PROJECTION_PERSPECTIVE == m_projection)
  {
    const ON_3dPoint eye = CameraLocation();
    const double tan_half = std::tan(0.5 * VerticalFieldOfView());
    const double aspect = static_cast<double>(m_width) / static_cast<double>(m_height);
    const double world_x = (px / half_w) * tan_half * aspect;
    const double world_y = (py / half_h) * tan_half;
    ray_from = eye;
    ray_dir = m_camera_dir + right * world_x + up * world_y;
    return ray_dir.Unitize();
  }

  const double world_x = px / m_scale;   // pixels -> world units on the target plane
  const double world_y = py / m_scale;
  ray_from = m_target + right * world_x + up * world_y;
  ray_dir = m_camera_dir;
  return true;
}

bool TL_Viewport::ScreenToCameraPlane(const ON_2dPoint& screen, ON_3dPoint& world) const
{
  ON_3dPoint from = ON_3dPoint::Origin;
  ON_3dVector dir = ON_3dVector::ZeroVector;
  if (!ScreenToRay(screen, from, dir))
    return false;

  if (TL_PROJECTION_PARALLEL == m_projection)
  {
    world = from;  // the ray already starts on the target plane
    return true;
  }

  const ON_Plane plane(m_target, m_camera_dir);
  double t = 0.0;
  if (!ON_Intersect(ON_Line(from, from + dir), plane, &t) || !ON_IsValid(t))
    return false;
  world = from + dir * t;
  return true;
}

bool TL_Viewport::ScreenToPlane(const ON_2dPoint& screen, const ON_Plane& plane, ON_3dPoint& world) const
{
  ON_3dPoint from = ON_3dPoint::Origin;
  ON_3dVector dir = ON_3dVector::ZeroVector;
  if (!ScreenToRay(screen, from, dir))
    return false;

  double t = 0.0;
  if (!ON_Intersect(ON_Line(from, from + dir * 1.0e6), plane, &t) || !ON_IsValid(t))
    return false;
  world = from + dir * t;
  return true;
}

void TL_Viewport::Pan(double dx_pixels, double dy_pixels)
{
  const ON_3dVector right = ScreenRight();
  const ON_3dVector up = ScreenUp();

  if (TL_PROJECTION_PARALLEL == m_projection)
  {
    const double s = (m_scale > 1e-12) ? m_scale : 1.0;
    m_target = m_target - right * (dx_pixels / s) + up * (dy_pixels / s);
    return;
  }

  // Perspective: translate the eye and the target together by the world-space
  // distance the drag would cover at the target plane.
  const double tan_half = std::tan(0.5 * VerticalFieldOfView());
  const double world_per_pixel = 2.0 * m_target_distance * tan_half / static_cast<double>(m_height);
  const ON_3dVector delta = right * (dx_pixels * world_per_pixel) - up * (dy_pixels * world_per_pixel);
  m_target = m_target - delta;
}

void TL_Viewport::Zoom(double factor, const ON_2dPoint* anchor_screen)
{
  if (!(factor > 0.0) || !ON_IsValid(factor))
    return;

  ON_3dPoint anchor_world = m_target;
  if (nullptr != anchor_screen)
  {
    ON_3dPoint hit = ON_3dPoint::Origin;
    if (ScreenToCameraPlane(*anchor_screen, hit))
      anchor_world = hit;
  }

  if (TL_PROJECTION_PARALLEL == m_projection)
  {
    const double new_scale = std::clamp(m_scale * factor, 1e-6, 1e12);
    if (nullptr != anchor_screen)
    {
      // Keep the world point under the cursor pinned.
      ON_2dPoint after = ON_2dPoint::Origin;
      const ON_3dPoint saved_target = m_target;
      m_scale = new_scale;
      if (WorldToScreen(anchor_world, after))
      {
        const ON_3dVector right = ScreenRight();
        const ON_3dVector up = ScreenUp();
        const double dx = after.x - anchor_screen->x;
        const double dy = after.y - anchor_screen->y;
        m_target = saved_target + right * (dx / m_scale) - up * (dy / m_scale);
      }
      m_target = saved_target;
      m_scale = new_scale;
    }
    else
    {
      m_scale = new_scale;
    }
    return;
  }

  const double new_distance = (m_target_distance > 1e-9) ? m_target_distance / factor : 10.0;
  m_target_distance = std::clamp(new_distance, 1e-6, 1e12);
}

void TL_Viewport::ZoomToFit(const ON_BoundingBox& world_bbox, double margin)
{
  if (!world_bbox.IsValid() || !IsValid())
    return;

  const double m = (margin > 0.0 && margin <= 1.0) ? margin : 0.9;
  const ON_3dPoint center = 0.5 * (world_bbox.m_min + world_bbox.m_max);
  m_target = center;

  const ON_3dVector right = ScreenRight();
  const ON_3dVector up = ScreenUp();

  ON_3dPoint corners[8];
  // NOTE: ON_BoundingBox::GetCorners returns bool, not a count -- treating its return
  // value as `n` silently reduced the fit to a single corner.
  const bool have_corners = world_bbox.GetCorners(corners);
  const int n = have_corners ? 8 : 0;
  double max_x = 0.0;
  double max_y = 0.0;
  double max_r = 0.0;
  for (int i = 0; i < n; ++i)
  {
    const ON_3dVector d = corners[i] - center;
    max_x = std::max(max_x, std::fabs(ON_DotProduct(d, right)));
    max_y = std::max(max_y, std::fabs(ON_DotProduct(d, up)));
    max_r = std::max(max_r, d.Length());
  }

  if (TL_PROJECTION_PARALLEL == m_projection)
  {
    if (max_x <= 1e-12 && max_y <= 1e-12)
      return;
    const double sx = (max_x > 1e-12) ? (0.5 * m * m_width) / max_x : ON_DBL_MAX;
    const double sy = (max_y > 1e-12) ? (0.5 * m * m_height) / max_y : ON_DBL_MAX;
    m_scale = std::min(sx, sy);
    return;
  }

  const double tan_half = std::tan(0.5 * VerticalFieldOfView());
  const double radius = std::max(max_r, 1e-9);
  m_target_distance = radius / (tan_half * m);
}

void TL_Viewport::SetStandardView(int standard_view)
{
  m_standard_view = standard_view;
  switch (standard_view)
  {
    case TL_VIEW_TOP:
      if (m_name.empty()) m_name = "Top";
      m_projection = TL_PROJECTION_PARALLEL;
      m_camera_dir = ON_3dVector(0.0, 0.0, -1.0);
      m_up = ON_3dVector(0.0, 1.0, 0.0);
      break;
    case TL_VIEW_PERSPECTIVE:
      if (m_name.empty()) m_name = "Perspective";
      m_projection = TL_PROJECTION_PERSPECTIVE;
      // Rhino's default 4-view perspective camera: south-east, above the model.
      m_camera_dir = ON_3dVector(-0.5773502691896258, 0.5773502691896258, -0.5773502691896258);
      m_up = ON_3dVector(0.0, 0.0, 1.0);
      m_target_distance = 10.0;
      m_lens_mm = 50.0;
      break;
    case TL_VIEW_FRONT:
      if (m_name.empty()) m_name = "Front";
      m_projection = TL_PROJECTION_PARALLEL;
      m_camera_dir = ON_3dVector(0.0, 1.0, 0.0);   // looking from -Y toward +Y
      m_up = ON_3dVector(0.0, 0.0, 1.0);
      break;
    case TL_VIEW_RIGHT:
      if (m_name.empty()) m_name = "Right";
      m_projection = TL_PROJECTION_PARALLEL;
      m_camera_dir = ON_3dVector(-1.0, 0.0, 0.0);  // looking from +X toward -X
      m_up = ON_3dVector(0.0, 0.0, 1.0);
      break;
    default:
      break;
  }
  if (m_camera_dir.Unitize() == false)
    m_camera_dir = ON_3dVector(0.0, 0.0, -1.0);
  m_up.Unitize();
}

void TL_Viewport::SetCanvasRect(int left, int top, int width, int height)
{
  m_left = left;
  m_top = top;
  m_width = std::max(1, width);
  m_height = std::max(1, height);
}

std::string TL_Viewport::ToJson() const
{
  const ON_3dVector right = ScreenRight();
  const ON_3dVector up = ScreenUp();
  (void)right;
  (void)up;

  std::ostringstream out;
  out << "{";
  out << "\"name\":\"" << m_name << "\"";
  out << ",\"standard\":" << m_standard_view;
  out << ",\"projection\":" << m_projection;
  out << ",\"target\":[" << FormatDouble(m_target.x) << "," << FormatDouble(m_target.y)
      << "," << FormatDouble(m_target.z) << "]";
  out << ",\"dir\":[" << FormatDouble(m_camera_dir.x) << "," << FormatDouble(m_camera_dir.y)
      << "," << FormatDouble(m_camera_dir.z) << "]";
  out << ",\"up\":[" << FormatDouble(m_up.x) << "," << FormatDouble(m_up.y)
      << "," << FormatDouble(m_up.z) << "]";
  out << ",\"distance\":" << FormatDouble(m_target_distance);
  out << ",\"lens\":" << FormatDouble(m_lens_mm);
  out << ",\"scale\":" << FormatDouble(m_scale);
  out << ",\"rect\":[" << m_left << "," << m_top << "," << m_width << "," << m_height << "]";
  out << "}";
  return out.str();
}

// ============================================================================
// TL_FourViewLayout
// ============================================================================

TL_FourViewLayout::TL_FourViewLayout()
{
  SetCanvasSize(m_canvas_width, m_canvas_height);
  m_views[0] = TL_Viewport("Top", TL_VIEW_TOP);
  m_views[1] = TL_Viewport("Perspective", TL_VIEW_PERSPECTIVE);
  m_views[2] = TL_Viewport("Front", TL_VIEW_FRONT);
  m_views[3] = TL_Viewport("Right", TL_VIEW_RIGHT);
  SetCanvasSize(m_canvas_width, m_canvas_height);
}

TL_FourViewLayout TL_FourViewLayout::Default(int canvas_width, int canvas_height)
{
  TL_FourViewLayout layout;
  layout.SetCanvasSize(canvas_width, canvas_height);
  return layout;
}

TL_Viewport& TL_FourViewLayout::View(int index)
{
  const int i = std::clamp(index, 0, 3);
  return m_views[i];
}

const TL_Viewport& TL_FourViewLayout::View(int index) const
{
  const int i = std::clamp(index, 0, 3);
  return m_views[i];
}

int TL_FourViewLayout::IndexOfName(const std::string& name) const
{
  for (int i = 0; i < 4; ++i)
  {
    if (m_views[i].m_name == name)
      return i;
  }
  return -1;
}

void TL_FourViewLayout::SetCanvasSize(int width, int height)
{
  m_canvas_width = std::max(2, width);
  m_canvas_height = std::max(2, height);
  const int half_w = m_canvas_width / 2;
  const int half_h = m_canvas_height / 2;
  const int right_w = m_canvas_width - half_w;
  const int bottom_h = m_canvas_height - half_h;

  // Rhino's classic 2x2 arrangement: Top | Perspective over Front | Right.
  m_views[0].SetCanvasRect(0, 0, half_w, half_h);
  m_views[1].SetCanvasRect(half_w, 0, right_w, half_h);
  m_views[2].SetCanvasRect(0, half_h, half_w, bottom_h);
  m_views[3].SetCanvasRect(half_w, half_h, right_w, bottom_h);
}

bool TL_FourViewLayout::WorldToScreen(int view_index, const ON_3dPoint& world, ON_2dPoint& screen) const
{
  return View(view_index).WorldToScreen(world, screen);
}

bool TL_FourViewLayout::ScreenToRay(int view_index, const ON_2dPoint& screen,
                                    ON_3dPoint& from, ON_3dVector& dir) const
{
  return View(view_index).ScreenToRay(screen, from, dir);
}

int TL_FourViewLayout::PickPoint(int view_index, const ON_2dPoint& screen,
                                 const std::vector<ON_3dPoint>& world_points, double pixel_tol) const
{
  const TL_Viewport& vp = View(view_index);
  int best = -1;
  double best_dist = pixel_tol;
  for (size_t i = 0; i < world_points.size(); ++i)
  {
    ON_2dPoint projected = ON_2dPoint::Origin;
    if (!vp.WorldToScreen(world_points[i], projected))
      continue;
    const double dx = projected.x - screen.x;
    const double dy = projected.y - screen.y;
    const double d = std::sqrt(dx * dx + dy * dy);
    if (d <= best_dist)
    {
      best_dist = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

void TL_FourViewLayout::ZoomToFitAll(const ON_BoundingBox& world_bbox, double margin)
{
  if (!world_bbox.IsValid())
    return;
  for (int i = 0; i < 4; ++i)
    m_views[i].ZoomToFit(world_bbox, margin);
}

std::string TL_FourViewLayout::ToJson() const
{
  std::ostringstream out;
  out << "{\"schema\":\"opentl.fourview\",\"version\":1";
  out << ",\"canvas\":{\"width\":" << m_canvas_width << ",\"height\":" << m_canvas_height << "}";
  out << ",\"views\":[";
  for (int i = 0; i < 4; ++i)
  {
    if (i > 0)
      out << ",";
    out << m_views[i].ToJson();
  }
  out << "]}";
  return out.str();
}

bool TL_FourViewLayout::FromJson(const std::string& json)
{
  MiniJson parser;
  parser.text = json;

  if (!parser.Eat('{'))
    return false;

  int width = m_canvas_width;
  int height = m_canvas_height;
  std::vector<TL_Viewport> parsed;
  bool ok = true;

  while (ok && !parser.Eat('}'))
  {
    std::string key;
    if (!parser.ParseString(key) || !parser.Eat(':'))
      return false;

    if (key == "canvas")
    {
      if (!parser.Eat('{'))
        return false;
      while (!parser.Eat('}'))
      {
        std::string inner;
        double value = 0.0;
        if (!parser.ParseString(inner) || !parser.Eat(':') || !parser.ParseNumber(value))
          return false;
        if (inner == "width")
          width = static_cast<int>(value);
        else if (inner == "height")
          height = static_cast<int>(value);
        if (!parser.Eat(',') && parser.text[parser.pos] != '}')
        {
          // tolerate both "k":v, and "k":v} forms
        }
      }
    }
    else if (key == "views")
    {
      if (!parser.Eat('['))
        return false;
      while (!parser.Eat(']'))
      {
        if (!parser.Eat('{'))
          return false;
        TL_Viewport vp;
        while (!parser.Eat('}'))
        {
          std::string field;
          if (!parser.ParseString(field) || !parser.Eat(':'))
            return false;

          if (field == "name")
          {
            if (!parser.ParseString(vp.m_name))
              return false;
          }
          else if (field == "standard")
          {
            double d = 0.0;
            if (!parser.ParseNumber(d)) return false;
            vp.m_standard_view = static_cast<int>(d);
          }
          else if (field == "projection")
          {
            double d = 0.0;
            if (!parser.ParseNumber(d)) return false;
            vp.m_projection = static_cast<int>(d);
          }
          else if (field == "distance")
          {
            if (!parser.ParseNumber(vp.m_target_distance)) return false;
          }
          else if (field == "lens")
          {
            if (!parser.ParseNumber(vp.m_lens_mm)) return false;
          }
          else if (field == "scale")
          {
            if (!parser.ParseNumber(vp.m_scale)) return false;
          }
          else if (field == "target" || field == "dir" || field == "up")
          {
            std::vector<double> values;
            if (!parser.ParseNumberArray(values) || values.size() != 3)
              return false;
            const ON_3dPoint v(values[0], values[1], values[2]);
            if (field == "target")
              vp.m_target = v;
            else if (field == "dir")
              vp.m_camera_dir = v;
            else
              vp.m_up = v;
          }
          else if (field == "rect")
          {
            std::vector<double> values;
            if (!parser.ParseNumberArray(values) || values.size() != 4)
              return false;
            vp.SetCanvasRect(static_cast<int>(values[0]), static_cast<int>(values[1]),
                             static_cast<int>(values[2]), static_cast<int>(values[3]));
          }
          else
          {
            if (!parser.SkipValue())
              return false;
          }

          parser.SkipWs();
          if (parser.Eat(','))
            continue;
        }
        parsed.push_back(vp);
        parser.SkipWs();
        if (parser.Eat(','))
          continue;
      }
    }
    else
    {
      if (!parser.SkipValue())
        return false;
    }

    parser.SkipWs();
    if (parser.Eat(','))
      continue;
  }

  if (parsed.size() != 4)
    return false;
  if (!ok)
    return false;

  m_canvas_width = std::max(2, width);
  m_canvas_height = std::max(2, height);
  for (int i = 0; i < 4; ++i)
  {
    m_views[i] = parsed[static_cast<size_t>(i)];
    m_views[i].m_camera_dir.Unitize();
    m_views[i].m_up.Unitize();
  }
  return true;
}

bool TL_FourViewLayout::SaveJson(const std::string& path) const
{
  std::ofstream out(path.c_str(), std::ios::binary | std::ios::trunc);
  if (!out)
    return false;
  out << ToJson() << "\n";
  return out.good();
}

bool TL_FourViewLayout::LoadJson(const std::string& path)
{
  std::ifstream in(path.c_str(), std::ios::binary);
  if (!in)
    return false;
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return FromJson(buffer.str());
}
