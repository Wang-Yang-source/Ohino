#pragma once
/**
 * @file tl_gh.h
 * @brief Ohino (OpenTL) Grasshopper (.gh / .ghx / .ghz) Archive Reader & Parametric DAG Solver (gh/)
 *
 * Implements:
 *   1. Native binary `GH_Archive` (.gh) reader (raw deflate `wbits = -15` + 7-bit length-prefixed
 *      strings + 25 strongly-typed `GH_Types` items + recursive `GH_Chunk` hierarchy) and XML (.ghx)
 *      / ZIP (.ghz) readers.
 *   2. Full `DefinitionObjects` graph extraction: component GUIDs, instance UUIDs, 2D canvas
 *      `Bounds` & `Pivot` coordinates, input/output `TL_GhPort` pins, wire connections (`Source`),
 *      `PersistentData` DataTree branches, `Number Slider` ranges/values, `Panel` text, and `Group`
 *      containers.
 *   3. Kahn topological sort + parametric DAG evaluation engine backed by the OpenTL C++ geometry
 *      kernel (Points, Vectors, Lines, Circles, Arcs, NURBS Curves, Interpolate, Divide Curve,
 *      Curve Closest Point, Range, Bounds, Remap, Math operators, Loft, Sweep1, SubD Box, Planar
 *      Slice, and Mesh generation).
 */

#include "../tl_export.h"
#include "../mvp/rhino_mvp.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

enum class TL_GhValueKind : int
{
  Null     = 0,
  Bool     = 1,
  Number   = 2,
  String   = 3,
  Point    = 4,
  Vector   = 5,
  Interval = 6,
  Line     = 7,
  Plane    = 8,
  Circle   = 9,
  Arc      = 10,
  Box      = 11,
  Color    = 12,
  Curve    = 13,
  Surface  = 14,
  Mesh     = 15
};

struct TL_CLASS TL_GhValue
{
  TL_GhValueKind kind = TL_GhValueKind::Null;
  bool bool_val = false;
  double num_val = 0.0;
  std::string str_val;
  ON_3dPoint pt_val = ON_3dPoint::Origin;
  ON_3dVector vec_val = ON_3dVector::ZeroVector;
  ON_Interval interval_val = ON_Interval(0.0, 1.0);
  ON_Line line_val;
  ON_Plane plane_val = ON_Plane::World_xy;
  ON_Circle circle_val;
  ON_Arc arc_val;
  ON_BoundingBox box_val;
  uint8_t color_rgba[4] = { 203, 166, 247, 255 };
  std::shared_ptr<ON_NurbsCurve> curve_val;
  std::shared_ptr<ON_NurbsSurface> surface_val;
  std::shared_ptr<ON_Mesh> mesh_val;

  static TL_GhValue MakeNumber(double v);
  static TL_GhValue MakeBool(bool b);
  static TL_GhValue MakeString(const std::string& s);
  static TL_GhValue MakePoint(const ON_3dPoint& p);
  static TL_GhValue MakeVector(const ON_3dVector& v);
  static TL_GhValue MakeInterval(double a, double b);
  static TL_GhValue MakeLine(const ON_Line& l);
  static TL_GhValue MakePlane(const ON_Plane& pl);
  static TL_GhValue MakeCircle(const ON_Circle& c);
  static TL_GhValue MakeArc(const ON_Arc& a);
  static TL_GhValue MakeBox(const ON_BoundingBox& b);
  static TL_GhValue MakeColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);
  static TL_GhValue MakeCurve(const ON_NurbsCurve& c);
  static TL_GhValue MakeSurface(const ON_NurbsSurface& s);
  static TL_GhValue MakeMesh(const ON_Mesh& m);

  double AsNumber(double fallback = 0.0) const;
  ON_3dPoint AsPoint(const ON_3dPoint& fallback = ON_3dPoint::Origin) const;
  ON_3dVector AsVector(const ON_3dVector& fallback = ON_3dVector::ZAxis) const;
  std::string ToDisplayString() const;
};

struct TL_CLASS TL_GhRectF
{
  float x = 0.0f;
  float y = 0.0f;
  float w = 64.0f;
  float h = 28.0f;
};

struct TL_CLASS TL_GhSliderState
{
  double min_val = 0.0;
  double max_val = 10.0;
  double value = 1.0;
  int digits = 2;
  bool is_integer = false;
};

struct TL_CLASS TL_GhPort
{
  std::string instance_guid;
  std::string name;
  std::string nickname;
  std::string description;
  TL_GhRectF bounds;
  float pivot_x = 0.0f;
  float pivot_y = 0.0f;
  std::vector<std::string> source_guids;    ///< Upstream parameter/node instance GUIDs
  std::vector<TL_GhValue> persistent_data;  ///< Embedded PersistentData branch items
  std::vector<TL_GhValue> runtime_data;     ///< Evaluated values after DAG solve
};

struct TL_CLASS TL_GhNode
{
  int index = 0;
  std::string component_guid;
  std::string instance_guid;
  std::string name;
  std::string nickname;
  std::string description;
  TL_GhRectF bounds;
  float pivot_x = 0.0f;
  float pivot_y = 0.0f;
  bool hidden = false;
  bool is_group = false;
  bool is_scribble = false;
  bool is_slider = false;
  bool is_panel = false;
  bool is_swatch = false;
  TL_GhSliderState slider;
  std::string panel_text;
  std::string expression;
  uint8_t swatch_rgba[4] = { 137, 180, 250, 255 };
  std::vector<std::string> group_member_guids;
  std::vector<TL_GhPort> inputs;
  std::vector<TL_GhPort> outputs;
  std::vector<TL_GhValue> persistent_data;  ///< For standalone parameter nodes (Point, Curve, Number)
  std::vector<TL_GhValue> runtime_data;     ///< Evaluated outputs for standalone parameter nodes
};

struct TL_CLASS TL_GhWire
{
  int from_node = -1;
  int from_output = -1; ///< -1 when the source is a standalone parameter node
  int to_node = -1;
  int to_input = -1;    ///< -1 when the target is a standalone parameter node
  float x0 = 0.0f;
  float y0 = 0.0f;
  float x1 = 0.0f;
  float y1 = 0.0f;
};

class TL_CLASS TL_GhDocument
{
public:
  TL_GhDocument() = default;
  ~TL_GhDocument() = default;

  void Clear();

  /**
   * Load a Grasshopper definition from a binary `.gh` (GH_Archive raw deflate), XML `.ghx`,
   * or Grasshopper 2 `.ghz` file, build the wire graph, and run an initial DAG evaluation.
   */
  bool LoadFile(const std::string& file_path, std::string* error_out = nullptr);

  /**
   * Re-evaluate all nodes in topological order (Kahn's algorithm) after slider or input edits.
   */
  bool Evaluate(std::string* error_out = nullptr);

  /**
   * Update a Number Slider by NickName/Name (or 0-based slider ordinal) and re-evaluate.
   */
  bool SetSliderValue(const std::string& name_or_nickname, double value);
  bool SetSliderValueByOrdinal(size_t slider_ordinal, double value);

  /**
   * Bake all visible (`!hidden`) evaluated 3D geometry (Points, Lines, Circles, Arcs,
   * NURBS Curves, Planes, Surfaces, Meshes) into `out_scene` and `out_meshes`.
   * Caller owns any newly allocated `ON_Mesh*` appended to `out_meshes`.
   */
  int BakeToScene(
    opentl::mvp::SceneDocument& out_scene,
    std::vector<ON_Mesh*>& out_meshes,
    std::vector<const float*>* out_mesh_colors = nullptr
  ) const;

  const std::string& FilePath() const { return file_path_; }
  const std::string& DocumentName() const { return document_name_; }
  const std::string& ArchiveVersion() const { return archive_version_; }
  const std::vector<TL_GhNode>& Nodes() const { return nodes_; }
  std::vector<TL_GhNode>& NodesMutable() { return nodes_; }
  const std::vector<TL_GhWire>& Wires() const { return wires_; }
  const std::vector<int>&TopologicalOrder() const { return topo_order_; }

  size_t SliderCount() const;
  size_t Ports() const;
  size_t ComponentCount() const;
  TL_GhRectF CanvasBounds() const;
  std::string FormatSummary() const;

private:
  void RebuildWiresAndTopology();
  void EvaluateSingleNode(TL_GhNode& node);

  std::string file_path_;
  std::string document_name_;
  std::string archive_version_ = "0.2.2";
  std::vector<TL_GhNode> nodes_;
  std::vector<TL_GhWire> wires_;
  std::vector<int> topo_order_;
};
