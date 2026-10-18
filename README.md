# OpenTL — Rhino-like 原生几何内核与场景 MVP

> **OpenTL**（*Open Rhino Native Geometry Kernel & Scene MVP*）是一个建立在 McNeel 官方开源 **openNURBS** 之上的 C++17 几何内核实验工程。
> 它用「合法开源库 + 静态逆向证据驱动的 clean-room 复现」替代对 Rhino 专有二进制（`RhinoCommon.dll` / `rhcommon_c.dll` / `tl.dll` / `RhinoCore.dll`）的运行时依赖，
> 在保持与 RhinoCommon API 语义对齐的前提下，提供一套可独立构建、可确定性验证、可跨平台移植的几何内核与最小场景文档（Scene Document）。

| 项目 | 值 |
| --- | --- |
| 版本 | `0.1.0`（MVP） |
| 语言标准 | C++17 |
| 几何基座 | [mcneel/opennurbs](https://github.com/mcneel/opennurbs)（`../external/opennurbs`，v8.35 分支，commit `eb92af3`） |
| 构建系统 | CMake ≥ 3.16 |
| 已验证平台 | macOS arm64（AppleClang）；Linux / Windows 使用同一套 CMake 描述 |
| 产物 | `libopentl.a`、`libopentl_shared.dylib/.so/.dll`、`opentl_tests`、`opentl_cli` |

---

## 目录

1. [项目概述](#1-项目概述)
2. [三层架构](#2-三层架构)
3. [设计原则与合规边界](#3-设计原则与合规边界)
4. [MVP 功能清单](#4-mvp-功能清单)
5. [目录结构](#5-目录结构)
6. [构建（Build）](#6-构建build)
7. [运行测试与 CLI 演示](#7-运行测试与-cli-演示)
8. [静态逆向证据来源（Provenance & `re-database`）](#8-静态逆向证据来源provenance--re-database)
9. [当前限制与后续演进边界](#9-当前限制与后续演进边界)
10. [致谢与许可](#10-致谢与许可)

---

## 1. 项目概述

Rhino 的几何能力在实现上被切成三层：托管层 `RhinoCommon.dll`（C#，`UnsafeNativeMethods` 中的 P/Invoke 声明）、
C ABI 桥接层 `rhcommon_c.dll`（把托管调用翻译成 C++ 调用）、以及 C++ 内核层 `tl.dll` / `opennurbs.dll` / `RhinoCore.dll`。
其中 `opennurbs.dll` 对应的 **openNURBS** 是 McNeel 以开源许可发布的 NURBS 几何库，而 `tl.dll`（"TL" = 内核工具层）与 `rhcommon_c.dll` 中的桥接逻辑则是闭源专有实现。

**OpenTL 的定位**：

- 用**开源 openNURBS** 作为唯一几何基座（曲线/曲面/Brep/SubD/Mesh 的数据结构与求值、`.3dm` 归档格式均由它提供）；
- 用**静态逆向证据**（导出表、导入表、RVA、反汇编语义、托管元数据）确定专有层的**函数边界、参数语义与算法行为**，再以 clean-room 方式**重新实现**这些行为；
- 用**可确定性验证**（稳定 64 位 ID、确定性 UUID、确定性 JSON、确定性 CLI 摘要）保证任何一次构建/运行都可复现、可比对。

因此 OpenTL 不是 Rhino 的封装，也不是二进制补丁，而是一个**行为对齐、实现独立**的几何内核：链接 `libopentl` 即可获得与 `RhinoCommon` 同名同义的几何 API 子集，且全程不加载、不执行、不反编译后再分发任何 Rhino 专有二进制。

---

## 2. 三层架构

OpenTL 严格镜像原始产品的三层调用链，但把每一层替换为可审计的自有实现：

```
┌──────────────────────────────────────────────────────────────────────┐
│ Tier 1  RhinoCommon.dll  (C# managed)                                │
│         UnsafeNativeMethods.*  —— 10 254 个托管方法 / 10 267 条 ImplMap│
│         P/Invoke: [DllImport("rhcommon_c", CallingConvention=cdecl)]  │
└───────────────────────────────┬──────────────────────────────────────┘
                                │  extern "C" 平坦 C ABI
┌───────────────────────────────▼──────────────────────────────────────┐
│ Tier 2  rhcommon_c.dll  (C ABI bridge)                               │
│         7 612 个导出（7 598 个 C 导出 / 14 个 C++ 修饰名）            │
│         通过 IAT 转发到 tl.dll (89) / opennurbs.dll (6 070) /         │
│         RhinoCore.dll (5 308)                                        │
└───────────────────────────────┬──────────────────────────────────────┘
                                │  C++ 调用（Promote + 成员函数）
┌───────────────────────────────▼──────────────────────────────────────┐
│ Tier 3  tl.dll / opennurbs.dll / RhinoCore.dll  (C++ kernel)         │
│         tl.dll:        8 642 导出，导入 opennurbs 3 210 个符号        │
│         opennurbs.dll: 9 461 导出（开源 openNURBS 的商业构建）        │
│         RhinoCore.dll: 13 709 导出（文档/显示/命令层，MVP 不涉及）    │
└──────────────────────────────────────────────────────────────────────┘
```

OpenTL 的对应关系：

| 原始层 | OpenTL 中的对应物 | 说明 |
| --- | --- | --- |
| `RhinoCommon.dll`（C#） | `include/opentl/mvp/rhino_mvp.h` 的 `opentl::mvp` 命名空间 | 类型别名直接映射到 `ON_*`，类与方法命名对齐 RhinoCommon |
| `rhcommon_c.dll`（C ABI） | `include/opentl/c_api/rhcommon_c_bridge.h` | `RH_C_FUNCTION` 导出，签名逐参数对齐（含 `const ON_*` 指针与 `int` 索引顺序） |
| `tl.dll`（C++ 内核工具层） | `include/opentl/{intersect,closest_point,fitting_loft_sweep,meshing,booleans,subd}/` + `tl_promote.h` | 同名函数/同名类，`TL_` 前缀与 `TL_Brep`/`TL_NurbsCurve`/`TL_NurbsSurface` 继承关系保持一致 |
| `opennurbs.dll` | `../external/opennurbs`（开源 openNURBS） | 不做替换，直接复用 |

**为什么三层都要保留？** 因为证据链本身就是三层的：`rhcommon_c` 的 C 导出名决定了 ABI，`tl.dll` 的修饰名决定了 C++ 语义（参数个数、`const` 性、返回类型），托管层的 `ImplMap` 决定了调用约定与封送方式。只有三层同时对得上，复现结果才在二进制接口层面可替换。

---

## 3. 设计原则与合规边界

1. **复用合法开源**：几何求值、NURBS 数据结构、Brep/SubD/Mesh 容器、`.3dm` 序列化全部来自 McNeel 官方开源的 openNURBS。OpenTL 不重新发明 NURBS，只在它之上补齐 Rhino 特有的组织方式与算法。
2. **只做静态分析，不执行原始二进制**：所有证据来自 PE 导出表/导入表解析、RVA 定位、反汇编阅读、托管元数据（CLI metadata）读取。运行期不加载 `tl.dll`/`rhcommon_c.dll`，不做 DLL 注入、不做内存挂钩、不做在线调用录制。
3. **clean-room 复现，不逐字复制专有代码**：实现依据是「函数签名 + 控制流语义 + 边界条件」这一类**行为规格**，源码为独立编写；注释中标注的是证据（RVA、符号名、判定分支），不是原始指令序列的搬运。
4. **确定性优先**：所有 ID、UUID、序列化输出、CLI 摘要必须逐字节可复现。没有随机数、没有哈希表迭代顺序泄漏、没有时间戳（除非显式请求）。
5. **失败要显式**：所有可能失败的入口返回 `bool` 并提供 `std::string* error_out`；几何退化（零长度线、退化平面、平行判定）通过结果结构体中的 `bool` 标志暴露，而不是抛异常或静默返回原点。
6. **不假装完整**：MVP 的边界在第 9 节逐条写明。任何"尚未实现"都以显式返回 `false` / 空数组表达，而不是返回看似合理的错误几何。

---

## 4. MVP 功能清单

### 4.1 最小几何内核

所有几何类型都是 openNURBS 类型的**零成本别名**，与 RhinoCommon 的托管类型一一对应，因此可以自由传入任何 `ON_*` API：

| OpenTL 类型 | openNURBS 实体 | `ObjectKind` | 可序列化为曲线 |
| --- | --- | --- | --- |
| `Point3d` | `ON_3dPoint` | `Point = 1` | 否 |
| `Vector3d` | `ON_3dVector` | —（仅作为参数/结果） | — |
| `Line` | `ON_Line` | `Line = 2` | 是（`ON_LineCurve`） |
| `Polyline` | `ON_Polyline` | `Polyline = 3` | 是（`ON_PolylineCurve`） |
| `Circle` | `ON_Circle` | `Circle = 4` | 是（整圆 `ON_ArcCurve`） |
| `Arc` | `ON_Arc` | `Arc = 5` | 是（`ON_ArcCurve`） |
| `Plane` | `ON_Plane` | `Plane = 6` | 否 |
| `BoundingBox` | `ON_BoundingBox` | —（派生量） | — |
| `NurbsCurve` | `ON_NurbsCurve` | `NurbsCurve = 7` | 是（自身） |
| `Transform` | `ON_Xform` | —（4×4 矩阵） | — |

**场景对象标识**——每个 `SceneObject` 同时具备三套互相独立的身份，保证跨进程、跨格式、跨语言可追踪：

```cpp
struct SceneObject {
  uint64_t    id;          // 场景内稳定 64 位 ID，单调递增，从 1 开始，删除后不复用
  ON_UUID     uuid;        // 确定性 UUID（MakeDeterministicUuid(seed)，同 seed 必得同 UUID）
  std::string name;        // 用户可见名称（可为空；FindByName 支持同多名）
  int         layer_index; // 图层索引（指向 SceneDocument::Layers()）
  ObjectKind  kind;        // 7 类几何之一
  /* 每类几何各有一个值语义的 payload：point_val / line_val / polyline_val /
     circle_val / arc_val / plane_val / nurbs_val */
};
```

**图层**——`Layer` 携带 `index`、`name`、`ON_UUID id`、`color_r/g/b`、`visible`；`SceneDocument::AddLayer` 返回稳定索引，`FindLayerByName` 按名查索引，`"Default"` 图层在构造时自动建立（索引 0）。

**确定性 UUID**：

```cpp
ON_UUID MakeDeterministicUuid(uint64_t seed);   // 纯函数：seed -> UUID，无全局状态
std::string UuidToString(const ON_UUID& id);    // 标准 8-4-4-4-12 小写形式
ON_UUID StringToUuid(const std::string& str);   // 解析失败返回 ON_nil_uuid
```

> 之所以不用随机 UUID：`.otscene.json` 与 `.3dm` 的往返比对需要**同一对象在两次序列化中产生完全相同的字节**。确定性 UUID 是这一点的前提。

---

### 4.2 核心几何操作

#### （a）场景管理与多维查询

```cpp
class SceneDocument {
  // 生命周期
  void Clear();

  // 图层
  int  AddLayer(const std::string& name, int r = 0, int g = 0, int b = 0, bool visible = true);
  int  FindLayerByName(const std::string& name) const;
  const Layer* GetLayer(int layer_index) const;
  const std::vector<Layer>& Layers() const;

  // 创建（均返回稳定 uint64_t ID ≥ 1）
  uint64_t AddPoint(const Point3d&, const std::string& name = "", const std::string& layer = "Default");
  uint64_t AddLine(const Line&, ...);
  uint64_t AddPolyline(const std::vector<Point3d>&, ...);
  uint64_t AddCircle(const Circle&, ...);
  uint64_t AddArc(const Arc&, ...);
  uint64_t AddPlane(const Plane&, ...);
  uint64_t AddNurbsCurve(const NurbsCurve&, ...);
  uint64_t AddNurbsCurveFromPoints(const std::vector<Point3d>& ctrl, int degree, ...);

  // 删除与查询
  bool DeleteObject(uint64_t id);
  const SceneObject* FindById(uint64_t id) const;
  SceneObject*       FindByIdMutable(uint64_t id);
  const SceneObject* FindByUuid(const ON_UUID& uuid) const;
  std::vector<const SceneObject*> FindByName(const std::string& name) const;
  std::vector<const SceneObject*> FindByLayer(const std::string& layer_name) const;
  std::vector<const SceneObject*> FindByKind(ObjectKind kind) const;

  // 反序列化专用低层入口（保留原始 id/uuid，不重新分配）
  uint64_t InsertDeserializedObject(SceneObject obj);
  void     SetLayersForDeserialization(std::vector<Layer> layers);
};
```

查询维度覆盖 **ID / UUID / 名称 / 图层 / 类型** 五种，且 `FindBy*` 返回的指针指向 `SceneDocument` 内部存储，生命周期与文档一致（删除对象后失效）。

#### （b）变换（4×4 矩阵）

```cpp
Transform MakeTranslation(double dx, double dy, double dz);
Transform MakeTranslation(const Vector3d& delta);
Transform MakeRotation(double angle_radians, const Vector3d& axis,
                       const Point3d& center = Point3d::Origin);
Transform MakeScale(const Point3d& anchor, double sx, double sy, double sz);
Transform MakeUniformScale(const Point3d& anchor, double scale_factor);

// 文档级便捷入口
bool TranslateObject(uint64_t id, const Vector3d& delta);
bool RotateObject(uint64_t id, double angle_radians, const Vector3d& axis,
                  const Point3d& center = Point3d::Origin);
bool ScaleObject(uint64_t id, double sx, double sy, double sz,
                 const Point3d& anchor = Point3d::Origin);
bool TransformObject(uint64_t id, const Transform& xform);
```

变换以 `ON_Xform` 承载、按 `anchor` 先平移到原点、施加线性部分、再平移回去的次序合成，语义与 Rhino 的 `Transform.Rotation` / `Transform.Scale` 一致。`SceneObject::ApplyTransform(const Transform&)` 对外提供单对象原地版本，对 `Circle`/`Arc`/`Plane`/`NurbsCurve` 采用各自的解析变换（圆在非均匀缩放下会降级为 NURBS 曲线或返回失败）。

#### （c）点到对象距离、对象间最小距离、紧包围盒

```cpp
double      SceneObject::DistanceTo(const Point3d& pt) const;         // 到点最小 3D 欧氏距离
double      DistanceBetweenObjects(const SceneObject& a, const SceneObject& b); // 对象间最小距离
BoundingBox SceneObject::GetBoundingBox() const;                      // 紧致 AABB
BoundingBox SceneDocument::GetSceneBoundingBox() const;               // 场景整体 AABB
```

`DistanceTo` 对 `Point` 退化为点距，对曲线类对象先用牛顿迭代求最近点参数再取距离（初值来自粗采样，避免落在错误分支）。`DistanceBetweenObjects` 先做 AABB 早退，再按「点-曲线 / 曲线-曲线」组合分别处理。

#### （d）线-线求交：`IntersectLineLine`

```cpp
struct LineLineIntersection {
  bool    intersects;   // 求解成功且在容差/线段范围内
  bool    is_parallel;  // 平行或共线（按夹角容差判定）
  double  param_a;      // A 线参数（0..1 落在线段内）
  double  param_b;      // B 线参数
  Point3d point_a;      // A 线上最近点
  Point3d point_b;      // B 线上最近点
  double  distance;     // point_a ↔ point_b 弦距
};

LineLineIntersection IntersectLineLine(const Line& a, const Line& b,
                                       double tolerance = 1e-6,
                                       bool finite_segments = false);
```

覆盖三种情形：**真相交**（`intersects = true`，`distance ≤ tolerance`）、**异面最近点**（`intersects = false` 但 `param_a/param_b/point_a/point_b/distance` 仍有效）、**平行/共线**（`is_parallel = true`，参数无意义）。`finite_segments = true` 时参数被裁剪到 `[0,1]`，裁剪后的最近点可能落在端点上——这正是有限线段求交与无限直线求交的差别。

#### （e）线-面求交：`IntersectLinePlane`

```cpp
struct LinePlaneIntersection {
  bool    intersects;   // 线不平行于平面（有限线段时还要求在段内）
  bool    is_parallel;  // 线平行于平面
  double  line_param;   // P(t) = from + t*(to - from)
  double  plane_u;      // 平面局部 U 坐标
  double  plane_v;      // 平面局部 V 坐标
  Point3d point;
};

LinePlaneIntersection IntersectLinePlane(const Line& line, const Plane& plane,
                                         bool finite_segment = false,
                                         double tolerance = 1e-9);
```

除交点外还给出**平面局部 UV**（`ON_Plane::ClosestPointTo` 逆变换得到），便于后续在 Brep 面上做参数域判定。平行判定使用平面法向与线方向的点积阈值（`tolerance`），退化为"线在平面内"时 `is_parallel = true` 且 `intersects = false`。

#### （f）曲线求交：`TL_IntersectCurveLine` / `TL_IntersectCurvePlane`

```cpp
struct TL_CurveCurveIntersectionEvent {
  double  param_a;    // 曲线参数
  double  param_b;    // 直线参数
  ON_3dPoint point_a;
  ON_3dPoint point_b;
  bool    is_overlap; // 共线重叠段标记
};

int TL_IntersectCurveLine(const ON_Curve& curve, const ON_Line& line, double tolerance,
                          ON_SimpleArray<TL_CurveCurveIntersectionEvent>& events);

int TL_IntersectCurvePlane(const ON_Curve& curve, const ON_Plane& plane, double tolerance,
                           ON_SimpleArray<double>& curve_params,
                           ON_SimpleArray<ON_3dPoint>& intersection_points);
```

实现策略是**包围盒递归细分 + 牛顿-拉夫逊精化**：先按曲线跨度与直线包围盒做区间剔除，把可能相交的参数区间二分到容差尺度，再以「点到直线距离函数」的导数做牛顿迭代收敛；`is_overlap` 在曲线段与直线共线且端点互相落入对方范围时置位。曲线-平面求交则对有符号距离函数 `f(t) = (C(t) - P0)·N` 做变号区间搜索 + 割线/牛顿混合迭代，**切点**（`f(t)` 不变号但触及零）通过局部极小值检测单独收集。两个函数均返回事件数量（`0` 表示无交），并在数组超出 `ON_SimpleArray` 容量时自动扩容。

#### （g）曲线采样：`SampleCurveByCount`

```cpp
struct CurveSamplePoint {
  double   t;             // 曲线参数
  double   normalized_t;  // 归一化参数 ∈ [0,1]
  double   arc_length;    // 从起点累计的弧长
  Point3d  point;
  Vector3d tangent;
};

std::vector<CurveSamplePoint> SampleCurveByCount(const ON_Curve& curve, int sample_count,
                                                 bool by_arc_length = false);
```

- `by_arc_length = false`：均匀**参数**采样（`t_i` 等距），切线取 `Ev1Der` 的一阶导。
- `by_arc_length = true`：等**弧长**采样。弧长由 **5 点 Gauss-Legendre 积分**对 `|C'(t)|` 逐段求积并累加成表，再在累计表上反查目标弧长对应的参数（二分 + 线性插值）。相比弦长累加，积分表在曲率大的区段不会系统性低估弧长，因此采样点分布更接近真实等距。

`sample_count < 2` 时按 2 处理；曲线非法或求值失败时返回空 `vector`。

#### （h）NURBS 构造

```cpp
bool CreateClampedNurbsCurve(const std::vector<Point3d>& control_points,
                             int degree, NurbsCurve& out_curve);
```

按给定次数生成**夹紧（clamped）均匀节点向量**的控制点定义曲线；`degree ≥ 1`、`control_points.size() > degree` 才成功。`SceneObject::ToNurbsCurve(NurbsCurve&)` 把 `Line`/`Polyline`/`Circle`/`Arc` 统一升格为 NURBS 表示（`Point`/`Plane` 返回 `false`），这是曲线求交、采样、导出前的公共归一化步骤。

---

### 4.3 扩展内核子系统（`tl.dll` / `rhcommon_c.dll` 对应实现）

以下子系统直接对应 `tl.dll` 的导出符号与 `rhcommon_c.dll` 的 C 桥接导出，头文件中的注释保留了原始 RVA 与反汇编判定依据。

#### （a）Brep 提升与面内点判定 —— `tl_promote.h` / `booleans/tl_booleans.h`

```cpp
class TL_Brep : public ON_Brep {
  static const TL_Brep* Promote(const ON_Object* obj);   // tl.dll RVA 0x2b2180
  static TL_Brep*       Promote(ON_Object* obj);         // tl.dll RVA 0x2b21b0

  // 0 = Outside, 1 = Inside, 2 = On boundary（tl.dll RVA 0x1fcf30）
  int  PointIsOnFace(int face_index, const ON_2dPoint& uv, double u_tol, double v_tol) const;
  // 便捷布尔版本（tl.dll RVA 0x1fd520）
  bool IsPointOnFace(int face_index, double u, double v, double tol) const;
};

TL_PointFaceRelation TL_ClassifyPointOnBrepFace(const ON_Brep& brep, int face_index,
                                                const ON_2dPoint& uv,
                                                double u_tol = 1e-6, double v_tol = 1e-6);
```

`Promote` 在原二进制中是「`ON_Brep::Cast(obj)` + 同尺寸指针重解释」——因为 `TL_Brep` 是**零字段**派生类，`sizeof(TL_Brep) == sizeof(ON_Brep)`，不需要额外构造。OpenTL 保留同样的 zero-field 派生与 `Cast` 语义，使 `TL_Brep*` 与 `ON_Brep*` 的互换在 ABI 上无开销。

`PointIsOnFace` 的判定顺序（与反汇编一致）：先用面参数域 `Domain(0)/Domain(1)` 做粗筛，再遍历该面的 `m_loop` / `m_tri` 做**参数域点在多边形内**测试，落在容差带内返回 `2`，否则 `0`/`1`。这是布尔面裁剪的前置分类器。

#### （b）网格未焊接边检测 —— `meshing/tl_meshing.h`

```cpp
bool      TL_IsMeshEdgeUnwelded(int edge_index, const ON_Mesh* mesh);   // tl.dll RVA 0x33bd70
ON_Mesh*  TL_MeshSurfaceGrid(const ON_Surface& surface, int u_div, int v_div,
                             ON_Mesh* result_mesh = nullptr);
```

`TL_IsMeshEdgeUnwelded` 是一段**逐分支复现**的逻辑，反汇编证据给出的判定序列为：

1. `mesh == nullptr` → `false`；
2. `ON_Mesh::Topology()` 求拓扑；`edge_index` 越界 → `false`；
3. 取 `ON_MeshTopologyEdge& tope`（`sizeof == 24`），若 `tope.m_topf_count <= 1`（非流形内边 / 边界边）→ `false`；
4. 解析两个拓扑顶点 `topvi[0]`、`topvi[1]`；若两者 `m_v_count <= 1`（各自只映射一个网格顶点）→ 该边不可能未焊接，`false`；
5. 慢路径：遍历 `tope.m_topfi[k]` 的关联面，用 `m_reve[side]` 定位该边在面上的编号，从 `F.vi[]` 取出真实网格顶点对 `(v0, v1)`；**只要有一个关联面使用了不同的顶点对**，即返回 `true`（未焊接，即 crease/接缝）。

这个函数是网格接缝检测、STL 导出前缝合检查、以及"同一拓扑边在两侧不共享顶点"这类破面诊断的核心判据。

`TL_MeshSurfaceGrid` 在参数域上按 `u_div × v_div` 规则取点生成四边形网格（`ON_Mesh`），作为 Brep 面离散化与体积/质量属性估算的起点。

#### （c）可变形圆弧/圆近似 —— `fitting_loft_sweep/tl_fitting_loft_sweep.h`

```cpp
ON_NurbsCurve* TL_DeformableArc2(const ON_Arc& arc, int degree, int cv_count,
                                 ON_NurbsCurve* result = nullptr);     // RVA 0x37b220..0x37b785
ON_NurbsCurve* TL_DeformableCircle(const ON_Circle& circle, int degree, int cv_count,
                                   ON_NurbsCurve* result = nullptr);   // RVA 0x37b790..0x37b7e5
```

`TL_DeformableArc2` 生成**非有理**（`is_rat = 0`）3D NURBS 曲线，用 `degree`/`cv_count` 的控制点逼近圆弧：`Create(3, false, degree+1, cv_count)` + `MakeClampedUniformKnotVector(1.0)`，域取 `arc.DomainRadians()` 或 `arc.Domain()`；内部按 Greville 横标在 `ON_Plane::PointAt(cos θ, sin θ)` 上取点，端点强制等于 `arc.StartPoint()/EndPoint()`，中间控制点按插值反解。

`TL_DeformableCircle` 生成**周期**非有理 NURBS（周期表示下总 CV 数为 `cv_count + degree`），使极限曲线高精度贴合整圆。二者是"圆/弧 → 可自由拖拽控制点的 NURBS"的桥梁，也是 `rhcommon_c!ON_ArcCurve_ToNurbsCurve` 的下游实现。

#### （d）节点消去 —— `TL_NurbsCurve::RemoveKnots` / `TL_NurbsSurface::RemoveKnots`

```cpp
class TL_NurbsCurve : public ON_NurbsCurve {
  bool RemoveKnots(int index0, int index1, double tolerance);
};
class TL_NurbsSurface : public ON_NurbsSurface {
  bool RemoveKnots(int dir, int index0, int index1, double tolerance);  // RVA 0x384d90
};
```

在 `[index0, index1]` 区间内逐个尝试消去节点：对候选节点求「移除后重拟合误差」，误差在 `tolerance` 内则提交并继续，否则跳过；曲面版本额外以 `dir` 指定 U/V 方向。用于在保证几何精度的前提下降低 NURBS 复杂度（导入第三方模型的"减肥"步骤）。

#### （e）Loft 曲面

```cpp
ON_NurbsSurface* TL_LoftCurves(const ON_SimpleArray<const ON_Curve*>& sections,
                               ON_NurbsSurface* result = nullptr);
```

对一组截面曲线做相容化（统一次数、统一节点向量、对齐接缝与方向）后，按线性（2 截面 → 直纹面）或三次跨截面插值生成 `ON_NurbsSurface`。截面数量 < 2 或次数不相容时返回 `nullptr`。

#### （f）SubD 盒体

```cpp
ON_SubD* TL_CreateSubDBox(const ON_BoundingBox& bbox, unsigned int subdivision_levels = 1,
                          ON_SubD* result = nullptr);
```

以 `bbox` 为外框建立 Catmull-Clark 细分盒（6 个四边形面），按 `subdivision_levels` 迭代细分。MVP 只提供这一种 SubD 基元，用于验证 SubD → NURBS/Brep 的转换路径与显示网格提取。

#### （g）最近点求解

```cpp
bool TL_GetCurveClosestPoint(const ON_Curve& curve, const ON_3dPoint& test_point,
                             double* t_out, double maximum_distance = 0.0,
                             const ON_Interval* sub_domain = nullptr);
bool TL_GetSurfaceClosestPoint(const ON_Surface& surface, const ON_3dPoint& test_point,
                               double* u_out, double* v_out, double maximum_distance = 0.0);
```

- 曲线：粗采样定位候选区间后，对 `f(t) = (C(t) − P)·C'(t) = 0` 做阻尼牛顿迭代，`f'(t) = C'·C' + (C − P)·C''`；遇到 `|f'|` 过小或参数越界时退回区间二分（保证收敛）。
- 曲面：`u × v` 网格播种后，对 `f(u,v) = (S − P)·S_u`、`g(u,v) = (S − P)·S_v` 做 2×2 Gauss-Newton / Newton 迭代，Hessian 退化时加阻尼项。
- `maximum_distance > 0` 时，超出该距离直接返回 `false`（远距离早退，避免无谓迭代）；`sub_domain` 允许把搜索限制在曲线子区间。

这套解算器同时服务于 `SceneObject::DistanceTo`、`DistanceBetweenObjects` 与 `TL_IntersectCurveLine` 的精化阶段，是三处共用的数值内核。

#### （h）C ABI 桥接导出

`include/opentl/c_api/rhcommon_c_bridge.h` 以 `RH_C_FUNCTION`（`extern "C"` + 默认可见性）导出与 `rhcommon_c.dll` 逐参数对齐的接口：

| OpenTL 导出 | 对应原始导出 | 原始 RVA | 语义 |
| --- | --- | --- | --- |
| `TL_Brep_PointIsOnFace(const ON_Brep*, int, double, double, double)` | `rhcommon_c.dll!TL_Brep_PointIsOnFace` | `0x153fe0` | `TL_Brep::Promote` + `IsPointOnFace` |
| `TLC_IsMeshEdgeUnwelded(const ON_Mesh*, int)` | `rhcommon_c.dll!TLC_IsMeshEdgeUnwelded` | `0x1555b0` | 尾跳转到 `tl.dll!TL_IsMeshEdgeUnwelded` |
| `ON_NurbsCurve_RemoveKnots(ON_NurbsCurve*, int, int, double)` | `rhcommon_c.dll!ON_NurbsCurve_RemoveKnots` | `0x154d90` | `TL_NurbsCurve::Promote` + `RemoveKnots` |
| `ON_Surface_RemoveKnots(ON_NurbsSurface*, int, int, int, double)` | `rhcommon_c.dll!ON_Surface_RemoveKnots` | `0x1552c0` | `TL_NurbsSurface::Promote` + `RemoveKnots` |

注意 `TLC_IsMeshEdgeUnwelded` 在原始二进制中是一条 `jmp`（尾调用）而非 `call`——这一点在 `three_tier_tl_kernel_map.json` 的 `iat_targets[].insn` 中被明确记录为 `"jmp"`，OpenTL 的实现同样直接转发，不做额外包装。

#### （i）标量原语、线段最近点与双策略求交（阶段 3 补齐）

| API | 对应原始导出 | RVA | 对齐强度 |
| --- | --- | --- | --- |
| `int TL_3DX_LineXPlane(from, dir, plane_pt, normal, &t)` | `tl.dll!TL_3DX_LineXPlane` | `0x305d0` | 逐状态码对齐：`-2` 共面 / `-1` 平行无交 / `0` t 恰为 0 / `1` 唯一交点，含 `|den|>=1` 用 `DBL_MIN`、`|den|<1` 用 `DBL_MAX` 的**饱和除法**（不产生 ±inf） |
| `double TL_LineDistanceToPoint(const ON_Line*, ON_3dPoint, bool maximum)` | `rhcommon_c.dll!ON_Line_DistanceToPoint` | `0xd0210` | `bool` 选 min/max 双模；空指针返回 **−1.0** 哨兵（不是 0） |
| `double TL_LineDistanceToLine(const ON_Line*, const ON_Line*, bool maximum)` | `rhcommon_c.dll!ON_Line_DistanceToLine` | `0xd0290` | 同上 |
| `bool TL_GetClosestPoint(ON_Line, ON_Line, double*, double*)` | `tl.dll!TL_GetClosestPoint` 4 参 | `0x214670` | 解析优先 + `[0,1]` 钳制重投影 + 端点投影回退；`true` 只表示"给出可用参数对"，**不表示真相交** |
| `bool TL_GetClosestPoint(..., double max_dist, const ON_Interval*, const ON_Interval*)` | `tl.dll!TL_GetClosestPoint` 7 参 | `0x214540` | 按二进制把线段提升为 **degree-1 `ON_BezierCurve`** 再委派（数学等价，非逐指令），支持子区间与 `maximum_distance` 门限 |
| `int TL_IntersectCurveLineBySubdivision(...)` | （替代路径，见下） | — | 独立的第二求交算法，用于与主求解器**对拍** |

`TL_3DX_LineXPlane` 的状态码同时被推到 MVP 层：`LinePlaneIntersection::status` 现在直接携带 `-2/-1/0/1`，调用方不必再从 `intersects`/`is_parallel` 反推退化原因。

**关于"双策略对拍"的如实说明**：`rhcommon_c!ON_Intersect_CurveLine` 的真实下游是 opennurbs 的 `ON_Curve::IntersectCurve`，但 `external/opennurbs`（v8.35 分支）**早于 `ON_X_EVENT` 的出现**，既没有该类型也没有 curve×curve 求交器，因此"直调 `IntersectCurve` 与原实现走同一条下游"这条路在本仓库不可行。替代方案是新增 `TL_IntersectCurveLineBySubdivision`：以"点到直线距离是 1-Lipschitz"为**严格下界**做剪枝（`min(D(t_i),D(t_i+1)) − |C(t_i)−C(t_i+1)|`），仅对下界低于容差的子区间做 Newton 抛光，再在测试里与主求解器逐事件对拍（圆×直径 / 圆×弦 / 圆×切线 / 圆×无交）。

---

#### （j）单轨扫掠 —— `TL_Sweep1Rail`

`ON_NurbsSurface* TL_Sweep1Rail(const ON_Curve& rail, const ON_Curve& profile, int station_count, ON_NurbsSurface* result)`

- **平行传输取帧**：把上一站的 up 向量投影到当前切线的法平面后重新正交化，因此剖面沿轨不会翻转或自旋；切线退化时沿用上一有效帧。
- **剖面在自己的 XY 平面解释**（`p.x` / `p.y`），对世界 XY 平面内的圆/矩形等平面剖面是精确的。
- 各站生成折线剖面后交给 `TL_LoftCurves` 做**插值式**蒙皮（U 向为插值而非线性蒙皮）。
- 精度边界：剖面保真度受 loft 的 U 向控制点数（≤ 64 个 CV）限制；`R = 2` 的圆管半径残差约 `2.4e-3`。测试按 `5e-3` 断言，不掩盖这一点。

**仍未实现**：双轨扫掠（2-rail）、沿轨缩放/扭转律、角部 G1/G2 过渡、偏置曲面。

### 4.4 四视图（Top / Perspective / Front / Right）

对应 Rhino 模型空间的经典 2×2 布局。这是**内核侧**实现：相机框架、世界↔屏幕投影、屏幕射线（拾取用）、平移/缩放/缩放到适合、确定性 JSON 持久化；**不含渲染与窗口系统**（OpenTL 不产生像素）。

| 条目 | 语义 |
| --- | --- |
| 布局 | `TL_FourViewLayout`：Top(0) / Perspective(1) / Front(2) / Right(3)，左上顺时针 2×2；`SetCanvasSize` 保持等比切分 |
| 标准视图 | Top：视线 −Z、上方向 +Y（屏幕 x=+X, y=+Y）；Front：视线 +Y、上方向 +Z（x=+X, y=+Z）；Right：视线 −X、上方向 +Z（x=+Y, y=+Z） |
| 投影 | `TL_PROJECTION_PARALLEL`（平行，`m_scale` 像素/单位）与 `TL_PROJECTION_PERSPECTIVE`（`m_lens_mm` 35mm 等效焦距 → 垂直 FOV，`m_target_distance` 决定眼点） |
| 世界→屏幕 | `WorldToScreen`（屏幕坐标原点在**画布**左上、y 向下；透视下落在眼后返回 false） |
| 屏幕→世界 | `ScreenToRay`（拾取射线）、`ScreenToCameraPlane`（过目标点且垂直视线的平面）、`ScreenToPlane`（与任意构造平面求交） |
| 导航 | `Pan(dx,dy)`（屏幕像素增量，平行视图改 target、透视视图同步平移眼点）、`Zoom(factor, anchor)`（带光标锚定）、`ZoomToFit(bbox, margin)` |
| 拾取 | `PickPoint(view, screen, points, pixel_tol)` 返回屏幕距离最近的点索引，无命中返回 −1 |
| 持久化 | `ToJson()/FromJson()/SaveJson()/LoadJson()`，schema `opentl.fourview` v1，往返后相机/尺度/矩形完全一致 |

```cpp
TL_FourViewLayout layout = TL_FourViewLayout::Default(1600, 900);
layout.ZoomToFitAll(scene_bbox, 0.9);

ON_2dPoint screen;
layout.View(0).WorldToScreen(ON_3dPoint(1, 2, 0), screen);   // Top 视图投影
ON_3dPoint from; ON_3dVector dir;
layout.View(1).ScreenToRay(screen, from, dir);               // 透视拾取射线
```

```bash
./opentl/build/opentl_cli --views --out-dir /tmp/opentl_views
```

#### Vulkan 渲染后端（`viewport/tl_vulkan_renderer.h`）

四视图的相机/投影是**后端无关**的；渲染层用标准 **Vulkan 1.2**（Khronos 官方 API），
在 macOS 上由 **MoltenVK** 提供 ICD，在 Windows/Linux 上走原生驱动——同一份代码、同一条管线。

| 条目 | 实现 |
| --- | --- |
| 依赖 | `find_package(Vulkan)`；macOS 上 `brew install vulkan-loader molten-vk vulkan-headers shaderc`，找不到 Vulkan 时该文件降级为无后端（返回明确错误，不静默失败） |
| 目标 | 离屏 `VK_FORMAT_R8G8B8A8_UNORM` 图像 → `vkCmdCopyImageToBuffer` 读回主机内存 |
| 管线 | 单一 `VK_PRIMITIVE_TOPOLOGY_LINE_LIST` 管线；顶点为「像素坐标 + RGBA」，`push_constant` 传画布尺寸，顶点着色器做 像素→NDC |
| 着色器 | GLSL 450 → SPIR-V（`glslc`），以 `uint32_t` 数组内嵌，无运行时文件依赖 |
| 绘制内容 | 每视口独立构造平面网格（Top=XY、Front=XZ、Right=YZ、Perspective=XY）、世界 X/Y/Z 轴（红/绿/蓝）、可见图层颜色的场景几何、视口边框、5×7 位图字体标签（TOP / PERSPECTIVE / FRONT / RIGHT）、原点十字 |
| 裁剪 | CPU 端 Liang–Barsky 按视口矩形裁剪（Vulkan 只有单一 scissor，不做逐视口裁剪） |
| 输出 | 内置 PNG 编码器（zlib `compress2` + CRC32，filter=0，RGBA8），无第三方图像库 |
| macOS 细节 | 实例启用 `VK_KHR_portability_enumeration` + `ENUMERATE_PORTABILITY`，设备在报告时启用 `VK_KHR_portability_subset` |

```bash
# 1600x900 四视图离屏渲染，输出 fourview.png
./opentl/build/opentl_cli --render-vulkan --scene demo --out-dir /tmp/opentl_vk
```

实测：`vulkan.backend=Vulkan 1.2.357`、`vulkan.device=Apple M4`、`vertices=1828`、`fourview.png` 5760000 B（1600×900×4 读回）。

#### 阶段 5：Catppuccin Mocha 主题 + 4× MSAA + 深度缓冲 + 着色曲面

| 能力 | 实现 |
| --- | --- |
| 配色 | `TL_CatppuccinMochaTheme`（Crust/Base/Surface0-2/Overlay0/Text + Red/Green/Blue/Yellow/Sapphire/Mauve/Peach/Teal/Lavender）；背景 `#11111b`，网格 `#313244/#45475a`，轴 `#f38ba8/#a6e3a1/#89b4fa`，原点 `#f9e2af`，曲面 `#cba6f7`，UI `#b4befe` |
| MSAA | `VK_SAMPLE_COUNT_4_BIT`（按 `framebufferColorSampleCounts ∩ DepthSampleCounts` 探测，实际值写回 `result.msaa_samples`），多采样颜色附件 + resolve 附件 |
| 深度 | `VK_FORMAT_D32_SFLOAT`（不可用时回退 D24S8 / D16），`VK_COMPARE_OP_LESS_OR_EQUAL`，每帧清 1.0 |
| 着色曲面 | 第二个 `TRIANGLE_LIST` 管线：顶点 = 像素坐标 + 法线 + RGBA，顶点着色器做 Lambert + Blinn-Phong（`0.44 + 0.72·N·L` + 高光），优先使用网格顶点法线做光滑着色 |
| 绘制顺序 | 先三角形（写深度）后线框；只有**场景线框**带 2e-4 的近视偏置，网格/坐标轴/边框/标签不偏置，因此会被实体正确遮挡 |
| 主题名 | `result.theme_name = "Catppuccin Mocha"` |

```bash
./opentl/build/opentl_cli --render-vulkan --scene demo --out-dir /tmp/opentl_vk
# vulkan.theme=Catppuccin Mocha msaa=4x depth=D32_SFLOAT
# vulkan.canvas=1600x900 vertices=2160 triangles=21504
```

**深度测试是被测量过的，不是被假设的**：测试把一块 ±30 的大四边形放在 FRONT 视口构造平面前方，统计视口内「网格颜色」像素数——基线 >0，遮挡后必须为 0。这条断言正是发现 `ON_BoundingBox::GetCorners` 返回 `bool`（被误当成角点数量）的地方：该错误让深度窗口塌缩到单个角点，整幅图的深度值几乎相同，深度测试退化成画家算法，于是出现「实体被网格穿透 + 锯齿状三角缺口」。

### 4.5 双格式文件序列化能力

OpenTL 同时提供**两种互补**的持久化格式：一种为零依赖、可逐字节比对的本地格式；一种为与 Rhino 生态互通的工业格式。

#### （a）`.otscene.json` —— 确定性场景格式

```cpp
bool SceneDocument::SaveJsonScene(const std::string& path, std::string* error_out = nullptr) const;
bool SceneDocument::LoadJsonScene(const std::string& path, std::string* error_out = nullptr);
```

- **零外部依赖**：自带的极简 JSON 读写器（无 rapidjson/nlohmann 依赖），保证在任意平台、任意编译器下产生同一份字节流。
- **完整覆盖 7 类几何**：`Point` / `Line` / `Polyline` / `Circle` / `Arc` / `Plane` / `NurbsCurve` 的**全部参数**（含 NURBS 的次数、是否有理、节点向量、控制点及权重）均被写出与读回。
- **图层无损**：图层表（索引、名称、UUID、RGB、可见性）与对象的 `layer_index` 一并持久化，读回后 `Layers()` 与对象归属完全一致。
- **确定性**：键顺序固定、浮点以最短可逆十进制（round-trip）输出、对象按 `id` 升序、无时间戳、无平台相关字段。同一场景两次保存 → 字节完全相同。
- **身份保留**：读回时走 `InsertDeserializedObject` / `SetLayersForDeserialization`，**原始 64 位 ID 与 UUID 不被重新分配**，因此"保存 → 载入 → 再保存"是固定点。

结构示意（顶层为 `layers` 与 `objects` 两个数组；每个对象携带 `id`、`uuid`、`name`、`layer`、`kind` 及该类几何的参数块，具体键名以实际写出文件为准）：

```json
{
  "format": "opentl-scene",
  "version": 1,
  "layers": [ { "index": 0, "name": "Default", "uuid": "…", "color": [0,0,0], "visible": true } ],
  "objects": [
    { "id": 1, "uuid": "…", "name": "c0", "layer": 0, "kind": "Circle",
      "circle": { "plane": { "origin": [0,0,0], "xaxis": [1,0,0], "yaxis": [0,1,0] }, "radius": 5.0 } }
  ]
}
```

#### （b）`.3dm` —— 真实 openNURBS 归档读写

```cpp
bool SceneDocument::Save3dm(const std::string& path, std::string* error_out = nullptr) const;
bool SceneDocument::Load3dm(const std::string& path, std::string* error_out = nullptr);
```

基于 `ONX_Model`（openNURBS 的 .3dm 文档模型）实现真实归档读写，生成的文件可被 Rhino / openNURBS 工具直接打开：

| `ObjectKind` | 写出的 openNURBS 实体 | 读回还原 |
| --- | --- | --- |
| `Point` | `ON_Point` | → `point_val` |
| `Line` | `ON_LineCurve` | → `line_val`（起点/终点） |
| `Polyline` | `ON_PolylineCurve` | → `polyline_val` |
| `Circle` | `ON_ArcCurve`（整圆） | → `circle_val`（平面 + 半径） |
| `Arc` | `ON_ArcCurve`（角度域） | → `arc_val` |
| `Plane` | **`ON_PlaneSurface`**，域 `[-10, 10]²`，附 `opentl_id` / `opentl_kind` UserString | → `plane_val`（取曲面平面） |
| `NurbsCurve` | `ON_NurbsCurve`（含权重与节点） | → `nurbs_val`（原样） |

关键设计点：`.3dm` 没有"无限平面"这一实体，因此 `Plane` 以**带有限范围的 `ON_PlaneSurface`**（`[-10,10]²`）落盘，并用 UserString 元数据 `opentl_kind = "Plane"` 与 `opentl_id = <uint64>` 标记真实语义。读回时凭 `opentl_kind` 把它重新折叠为 `ON_Plane`，并把 `opentl_id` 恢复为原始 64 位 ID——因此 Plane 也满足**无损往返**，而不是"看起来像平面的一块曲面"。

图层写入 `ONX_Model` 的 `m_layer_table`（名称、颜色、可见性、UUID）；对象名写入模型组件的 `Name`；对象 UUID 与稳定 ID 通过模型组件 UUID 与 UserString 双写，保证在第三方编辑器中重排对象顺序后仍可回认。

**双格式的分工**：`.otscene.json` 是**测试与 CI 的基准格式**（可 diff、可 review、可手改）；`.3dm` 是**互操作格式**（交给 Rhino、Grasshopper、其他 CAD）。两者对同一场景的读回结果必须逐字段相等——这是 `opentl_tests` 中的核心往返用例。

---

## 5. 目录结构

```
opentl/
├── CMakeLists.txt                       # 库 / 共享库 / 测试 / CLI 的构建描述
├── README.md
├── include/opentl/
│   ├── opentl.h                         # 伞头文件（include 全部子系统）
│   ├── tl_export.h                      # TL_CLASS / TL_FUNCTION / RH_C_FUNCTION 可见性宏
│   ├── tl_promote.h                     # TL_Brep / TL_NurbsCurve / TL_NurbsSurface
│   ├── mvp/rhino_mvp.h                  # MVP 核心：类型、SceneDocument、求交、采样、IO
│   ├── intersect/tl_intersect.h         # 曲线-直线 / 曲线-平面求交
│   ├── closest_point/tl_closest_point.h # 曲线 / 曲面最近点
│   ├── fitting_loft_sweep/tl_fitting_loft_sweep.h  # DeformableArc2/Circle、Loft
│   ├── meshing/tl_meshing.h             # 未焊接边检测、曲面网格化
│   ├── booleans/tl_booleans.h           # 面内点分类（布尔分类器）
│   ├── subd/tl_subd.h                   # SubD 盒体
│   ├── viewport/tl_viewport.h           # 四视图（Top/Perspective/Front/Right）相机与投影
│   └── c_api/rhcommon_c_bridge.h        # 与 rhcommon_c.dll 对齐的 C ABI
├── src/
│   ├── tl_promote.cpp
│   ├── mvp/                             # 场景、变换、距离、JSON/3DM IO、采样
│   ├── intersect/tl_intersect.cpp
│   ├── closest_point/tl_closest_point.cpp
│   ├── fitting_loft_sweep/
│   ├── meshing/
│   ├── booleans/
│   ├── subd/
│   ├── viewport/tl_viewport.cpp
│   └── c_api/rhcommon_c_bridge.cpp
├── tests/test_opentl.cpp                # 自动化测试（ctest 注册为 opentl_tests）
└── build/                               # 构建产物（含 opennurbs_build 子构建）
```

---

## 6. 构建（Build）

### 6.1 前置条件

| 依赖 | 版本 / 位置 | 说明 |
| --- | --- | --- |
| CMake | ≥ 3.16 | 需要 `add_subdirectory` 外部工程与 `ctest` |
| C++ 编译器 | 支持 C++17 | macOS: AppleClang；Linux: GCC ≥ 9 / Clang ≥ 10；Windows: MSVC 2019+ |
| openNURBS 源码 | `../external/opennurbs` | 本仓库以 git submodule / 同级目录方式提供，CMake 通过 `add_subdirectory` 引入 |
| pthread / dl | 系统自带 | `target_link_libraries(opentl PUBLIC opennurbsStatic pthread dl)`；Windows 下等价物由 openNURBS 的 CMake 处理 |

> `external/opennurbs` 必须存在且包含 `CMakeLists.txt`（目标名 `opennurbsStatic`）。缺失时 `cmake` 配置阶段会直接报错，而不是在编译期才暴露。
>
> 同理，`CMakeLists.txt` 中 `add_executable` 引用的源文件（`tests/test_opentl.cpp`、`apps/opentl_cli.cpp`）必须实际存在，否则配置阶段即在 `add_executable` 处失败。当前构建会产出静态库、共享库、`opentl_tests` 与 `opentl_cli` 四个目标，并把测试与 CLI 演示注册为两个 CTest 用例。

### 6.2 macOS arm64 完整构建命令

```bash
cmake -S /Users/waya/Projects/Rhino/opentl -B /Users/waya/Projects/Rhino/opentl/build -DCMAKE_BUILD_TYPE=Release
cmake --build /Users/waya/Projects/Rhino/opentl/build -j8
```

产物位置（默认生成器为 Makefile / Ninja，单配置）：

```
build/libopentl.a
build/libopentl_shared.dylib        # Linux: .so   Windows: opentl_shared.dll
build/opentl_tests
build/opentl_cli
build/opennurbs_build/…             # openNURBS 子构建（静态库）
```

### 6.3 Linux / Windows 可移植构建

同一套 `CMakeLists.txt`，无需改动源码：

```bash
# Linux (x86_64 / aarch64)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"

# Windows (MSVC, 从 "x64 Native Tools Command Prompt" 执行)
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release -j8
```

平台差异全部收敛在 `tl_export.h` 的可见性宏里：Windows 用 `__declspec(dllexport/dllimport)`（由 `OPENTL_EXPORTS` 切换），类 Unix 用 `__attribute__((visibility("default")))`。因此 `TL_FUNCTION` / `TL_CLASS` 标注的函数在三种平台上都进导出表，`RH_C_FUNCTION` 额外带 `extern "C"` 以避免 C++ 修饰名——这与 `rhcommon_c.dll` 的 `cdecl` 约定一致。

### 6.4 构建选项与注意事项

- `-DCMAKE_BUILD_TYPE=Release`：默认推荐。Debug 下 openNURBS 的断言（`ON_ASSERT`）会触发，属于库自身的行为，不是 OpenTL 的错误。
- 静态库 `opentl` 与共享库 `opentl_shared` 由**同一组源文件**构建，二者的导出符号集合一致；需要 P/Invoke 时链接 `opentl_shared`。
- 若只需验证算法，可单独构建测试目标：`cmake --build build --target opentl_tests -j8`。

---

## 7. 运行测试与 CLI 演示

> **当前验证状态（2026-10-19，macOS arm64 / AppleClang）**
>
> | 项目 | 结果 |
> | --- | --- |
> | clean build（删除 `build/` 后重新配置） | 通过，0 error；`opentl/src`、`apps`、`tests` 0 warning（其余 warning 均来自 openNURBS/zlib 第三方源码） |
> | `opentl_tests` | 16 个测试组、772 项检查、0 失败 |
> | `ctest` | 4/4 通过（`opentl_tests`、`opentl_cli_demo`、`opentl_cli_views`、`opentl_cli_vulkan`） |
> | 四视图布局往返 | `opentl_cli --views` 退出码 0，`views.roundtrip=identical` |
> | Vulkan 四视图渲染 | `vulkan.device=Apple M4`、`vulkan.backend=Vulkan 1.2.357`、`msaa=4x`、`depth=D32_SFLOAT`、1600×900 PNG 写出成功 |
> | CLI 端到端演示 | 退出码 0，`io.roundtrip json_digest_identical=1 3dm_digest_identical=1` |
> | 场景摘要哈希 | `scene_digest=a9b93db0b90a1441`（两次独立运行完全一致） |
> | `.otscene.json` 确定性 | 两次运行 SHA-256 相同：`8b4831c9c911fbe1d5b5033695ca680b02a21c179ad7b6408cd4f2e835ab519f` |
> | `--inspect-3dm` / `--inspect-json` | 同一场景两种格式读回摘要哈希一致（`a9b93db0b90a1441`） |


### 7.1 自动化测试

```bash
# 通过 ctest（推荐，失败时打印完整输出）
ctest --test-dir /Users/waya/Projects/Rhino/opentl/build --output-on-failure

# 直接运行测试可执行文件（便于单步调试）
/Users/waya/Projects/Rhino/opentl/build/opentl_tests
```

`opentl_tests` 覆盖的核心用例：

| 用例组 | 验证内容 |
| --- | --- |
| 几何构造 | 7 类对象的创建、`ObjectKind` 双向字符串转换、`ToNurbsCurve` 升格 |
| 身份稳定 | ID 单调递增不复用、`MakeDeterministicUuid` 同 seed 同值、UUID 字符串往返 |
| 查询 | 按 ID / UUID / 名称 / 图层 / 类型五维查询与删除后失效 |
| 变换 | 平移/旋转/缩放后包围盒与关键点坐标、`DistanceTo` 不变性（刚体变换） |
| 距离与包围盒 | 点-曲线、曲线-曲线最小距离，场景 AABB 合并 |
| 求交 | 线-线（相交/异面/平行/有限裁剪）、线-面（含 UV）、曲线-直线（含重叠）、曲线-平面（含切点） |
| 采样 | 均匀参数采样与等弧长采样的一致性、弧长表单调性 |
| 序列化 | `.otscene.json` 往返字节级固定点；`.3dm` 往返逐字段相等（含 Plane 的 `opentl_kind` 折叠） |
| 扩展子系统 | `Promote` 类型判定、`PointIsOnFace` 三态、未焊接边判据、`RemoveKnots` 误差界、Loft/SubD 基本形状 |

### 7.2 CLI 演示

```bash
# 内置演示场景：创建几何、变换、求交、打印确定性和摘要，并落盘两种格式
/Users/waya/Projects/Rhino/opentl/build/opentl_cli --demo

# 检视任意 .3dm：列出图层、对象类型、ID/UUID、包围盒
/Users/waya/Projects/Rhino/opentl/build/opentl_cli --inspect-3dm <file.3dm>

# 检视 .otscene.json：校验格式、列出图层与对象清单
/Users/waya/Projects/Rhino/opentl/build/opentl_cli --inspect-json <file.otscene.json>
```

`--demo` 输出的场景摘要由 `SceneDocument::FormatDeterministicSummary()` 生成：**多行、固定顺序、无地址/无时间戳**，因此可直接作为回归基线（同样输入两次运行必须得到完全相同的文本）。这也是把"几何正确性"变成"文本 diff"的最省力手段。

典型工作流：

```bash
# 1) 生成演示场景（同时写出 demo.otscene.json 与 demo.3dm）
opentl_cli --demo

# 2) 用两种检视器交叉验证同一份场景
opentl_cli --inspect-json demo.otscene.json
opentl_cli --inspect-3dm  demo.3dm

# 3) 交给 Rhino / openNURBS 工具打开 demo.3dm，确认外部可读
```

### 7.3 最小使用示例

```cpp
#include "opentl/opentl.h"
using namespace opentl::mvp;

int main() {
  SceneDocument doc;
  doc.AddLayer("Curves", 0, 128, 255);

  const uint64_t cid = doc.AddCircle(Circle(Plane::World_xy, 5.0), "c0", "Curves");
  const uint64_t lid = doc.AddLine(Line(Point3d(-10, 0, 0), Point3d(10, 0, 0)), "l0");

  doc.TranslateObject(cid, Vector3d(0, 0, 2));
  doc.RotateObject(lid, 3.14159265358979323846 / 6.0, Vector3d(0, 0, 1));

  const LineLineIntersection x =
      IntersectLineLine(Line(Point3d(0, 0, 0), Point3d(1, 1, 0)),
                        Line(Point3d(1, 0, 0), Point3d(0, 1, 0)),
                        /*tolerance*/ 1e-6, /*finite_segments*/ true);

  const double d = DistanceBetweenObjects(*doc.FindById(cid), *doc.FindById(lid));
  const BoundingBox scene_box = doc.GetSceneBoundingBox();

  std::string err;
  doc.SaveJsonScene("scene.otscene.json", &err);   // 确定性本地格式
  doc.Save3dm("scene.3dm", &err);                  // 真实 openNURBS 归档
  return 0;
}
```

编译链接：`-I include -I ../external/opennurbs -L build -lopentl -lopennurbsStatic`。

---

## 8. 静态逆向证据来源（Provenance & `re-database`）

所有与原产品行为对齐的结论都必须在 `re-database/` 中有对应证据。目录组织如下：

```
re-database/
├── summary.json                    # 四个 DLL 的导出入统计 + tl.dll 功能分类 + 三层映射总量
├── exports/
│   ├── tl_exports.json             # tl.dll 8 642 条导出（RVA + 修饰名 + 解修饰名）
│   ├── rhcommon_c_exports.json     # rhcommon_c.dll 7 612 条导出
│   ├── opennurbs_exports.json      # opennurbs.dll 9 461 条导出
│   ├── RhinoCore_exports.json      # RhinoCore.dll 13 709 条导出
│   └── tl_taxonomy.json            # tl.dll 导出按 10 个功能域分类
├── imports/
│   ├── tl_imports.json             # 16 个模块 / 3 459 条导入（opennurbs 占 3 210）
│   ├── rhcommon_c_imports.json     # 22 个模块 / 11 849 条导入（opennurbs 6 070、RhinoCore 5 308）
│   ├── opennurbs_imports.json
│   └── RhinoCore_imports.json      # tl.dll 1 120 条（RhinoCore 是 tl 的主要消费者）
├── mappings/
│   ├── three_tier_full_map.json    # 全量 10 267 条 P/Invoke 映射
│   └── three_tier_tl_kernel_map.json # 119 条真正落到 tl.dll 的三层映射（含 IAT 指令/RVA）
└── rea/                            # REA MCP 静态分析原始证据（evidence graph 落盘）
```

### 8.1 被分析的二进制及其 SHA-256

| 二进制 | SHA-256 | 角色 |
| --- | --- | --- |
| `RhinoCommon.dll` | `7371baa598c5cdd83972eed48539d1e1e455b536899d556a0399f7879677ee89` | 托管层（.NET Framework, IL-only, strong-name, 4 672 424 B） |
| `rhcommon_c.dll` | `d9006bceff5476a671439e7dc4ffacc666c2ca84f3c5e29657325bc688b229c1` | C ABI 桥接层 |
| `tl.dll` | `550a541018ac1b23c3a0dec53d3534fca4d212657dd7309e9b70c38775af3a7f` | C++ 内核工具层（主要复现目标） |
| `opennurbs.dll` | `cc0ab56820f918119a49c46616c2d705df9dc726565e171a64dc0fc78b3f03a7` | openNURBS 商业构建（对位开源替代） |
| `RhinoCore.dll` | `d6c29ac2029821d1413895bd68376a32e0b33f52516df9cacdf265eedaf2e04f` | 文档/显示/命令层（MVP 范围外） |

> 原文路径形如 `/home/waya/Projects/Software/Rhino/Rhino 9 WIP/System/<dll>`（REA 证据中的 `local_path` 字段），属于分析时的采集环境，不影响证据的可验证性——所有结论都以 SHA-256 为锚。

### 8.2 导出/导入规模统计（`summary.json`）

| DLL | 导出总数 | C++ 修饰名 | C 导出 | 导入模块数 | 导入符号数 |
| --- | --- | --- | --- | --- | --- |
| `tl.dll` | 8 642 | 7 922 | 720 | 16 | 3 459 |
| `rhcommon_c.dll` | 7 612 | 14 | 7 598 | 22 | 11 849 |
| `opennurbs.dll` | 9 461 | 8 192 | 1 269 | 26 | 240 |
| `RhinoCore.dll` | 13 709 | 8 192 | 5 517 | 55 | 13 718 |

关键交叉引用（决定了"哪些调用真的落到内核"）：

- `tl.dll` → `opennurbs.dll`：**3 210** 条导入。这是 OpenTL 可以完全复用 openNURBS 的直接依据——TL 层的几何求值本身也建立在 openNURBS 之上。
- `rhcommon_c.dll` → `tl.dll`：**89** 条；→ `opennurbs.dll`：**6 070** 条；→ `RhinoCore.dll`：**5 308** 条。桥接层绝大多数调用直达 openNURBS，只有少量薄封装走 TL。
- `RhinoCore.dll` → `tl.dll`：**1 120** 条，是 TL 的最大消费者（MVP 不涉及该层）。

`tl.dll` 的功能分布（`exports/tl_taxonomy.json`，总计 8 642）：

| 分类 | 数量 | MVP 覆盖 |
| --- | --- | --- |
| `core_utils` | 3 008 | 部分（容器/查表/字符串） |
| `brep_topology` | 1 678 | 部分（`Promote`、`PointIsOnFace`） |
| `intersect` | 1 064 | 部分（曲线-直线、曲线-平面、线-线、线-面） |
| `fitting_loft_sweep` | 981 | 部分（`DeformableArc2/Circle`、`LoftCurves`） |
| `nurbs_eval_core` | 739 | 否（由 openNURBS 承担） |
| `meshing` | 557 | 部分（未焊接边、曲面网格） |
| `math_solver` | 250 | 部分（最近点 Newton 内核） |
| `booleans` | 192 | 部分（面内点分类器） |
| `closest_point` | 171 | 是（曲线 + 曲面） |
| `subd` | 2 | 部分（`CreateSubDBox`） |

### 8.3 三层调用映射

`mappings/three_tier_full_map.json` 记录 **10 267** 条 P/Invoke 的完整调用链，其中 **119** 条经 `rhcommon_c.dll` 真正链接到 `tl.dll`（`tl_linked_count = 119`）。
`mappings/three_tier_tl_kernel_map.json` 是这 119 条的详细版，每条包含三段：

```json
{
  "tier1_managed": {
    "assembly": "RhinoCommon.dll", "class": "UnsafeNativeMethods",
    "method": "TL_Brep_PointIsOnFace", "import_name": "TL_Brep_PointIsOnFace",
    "pinvoke_module": "rhcommon_c", "call_convention": "cdecl",
    "signature": "i4 TL_Brep_PointIsOnFace(native-int, i4, r8, r8, r8)"
  },
  "tier2_c_bridge": {
    "dll": "rhcommon_c.dll", "export_name": "TL_Brep_PointIsOnFace",
    "rva": "0x153fe0", "va": "0x180153fe0"
  },
  "tier3_native_kernel": {
    "has_direct_or_forwarded_tl_call": true,
    "iat_targets": [
      { "insn": "call", "at_rva": "0x154019", "dll": "opennurbs.dll",
        "symbol": "?Count@?$ON_ClassArray@VON_BrepFace@@@@QEBAHXZ" },
      { "insn": "call", "at_rva": "0x15402a", "dll": "tl.dll",
        "symbol": "?Promote@TL_Brep@@SAPEBV1@PEBVON_Object@@@Z" }
    ],
    "internal_calls": []
  }
}
```

正是这张映射表把「托管方法名 → C 导出 → RVA → IAT 目标符号」串成一条可核对的链，OpenTL 头文件中每个函数的注释（导出名、RVA、调用关系）都直接来自这里。三个已确认的样本：

| 托管方法 | C 桥接 RVA | 内核落点 | 跳转形式 |
| --- | --- | --- | --- |
| `TL_Brep_PointIsOnFace` | `0x153fe0` | `opennurbs!ON_ClassArray<ON_BrepFace>::Count` + `tl!TL_Brep::Promote` | `call` ×2 |
| `TLC_IsMeshEdgeUnwelded` | `0x1555b0` | `tl!?TL_IsMeshEdgeUnwelded@@YA_NHPEBVON_Mesh@@@Z` | 单条 `jmp`（尾调用） |
| `ON_3dPointArrayArray_New` | `0xbd6f0` | `opennurbs!onmalloc` + `tl!ON_SimpleArray<ON_Polyline*>` 构造 | `call` ×N |

### 8.4 REA MCP 静态分析证据文件

`re-database/rea/` 下是 REA（Reverse Engineering Analysis）MCP 工具直接落盘的证据对象，每个文件都是 evidence-graph 的一条记录，含 `evidence_id`、`subject.digest.sha256`、`operation`、`normalized_result`、`confidence` 与 `limitations`：

| 文件 | 大小 | operation | `evidence_id` |
| --- | --- | --- | --- |
| `tl_dll_inspect_artifact.json` | 6.6 KB | `inspect_artifact` | `ev_471035d7689edf892dc5834237dafc47fa807dd22ed6edafd19324246952499f` |
| `rhcommon_c_dll_inspect_artifact.json` | 6.6 KB | `inspect_artifact` | `ev_08401cba7632d3431450888f6be86ddf3cb9fd5ff3d8eb2bb9fd58e0bc04ceb5` |
| `opennurbs_dll_inspect_artifact.json` | 6.6 KB | `inspect_artifact` | `ev_de964fecb963c3406cb540494788e2675bd8c4d7692cc87a431b479f12736224` |
| `RhinoCore_dll_inspect_artifact.json` | 6.6 KB | `inspect_artifact` | `ev_d65a37e06c8edf05d12613d9345e09d760fe33c72a52328959c5ec4233097787` |
| `RhinoCommon_dll_inspect_artifact.json` | 6.8 KB | `inspect_artifact` | `ev_06aef2ba2c29561bb91d6dba50f171ea6e6ed0ec2693638f123e6851857fda4f` |
| `RhinoCommon_dll_inspect_managed_artifact.json` | 2.1 MB | `inspect_managed_artifact` | `ev_9ac392acb957e5c1b2f6ce3c187fd5be16d00b825d8b9222c511dc7607ccbb28` |
| `RhinoCommon_dll_managed_native_boundaries.json` | 8.6 MB | `inspect_managed_native_boundaries` | `ev_f3a97f82dc8888251c3e2f23bbbd10c201a07c99bda0d563856c4f8ebabd11ea` |
| `RhinoCommon_dll_managed_members.json` | 1.8 KB | `inspect_managed_members`（全量，受资源上限阻断） | `ev_77209edeec5e7b0b5c880b60e6153e03af77e96e2465a0fe24219449621422a5` |
| `RhinoCommon_UnsafeNativeMethods.json` | 12 MB | `inspect_managed_members`（`type_names=["UnsafeNativeMethods"]`） | `ev_32e9aeb2b312d881d1630f95c2abc732c963094b65229e308c23cd625df89ba7` |
| `tl_dll_binary_layout.json` | 948 B | `inspect_binary_layout` | —（`provider_unavailable`，见下） |
| `rhcommon_c_dll_binary_layout.json` | 948 B | `inspect_binary_layout` | —（同上） |
| `opennurbs_dll_binary_layout.json` | 948 B | `inspect_binary_layout` | —（同上） |
| `RhinoCore_dll_binary_layout.json` | 948 B | `inspect_binary_layout` | —（同上） |

**诚实说明**：四个 `*_binary_layout.json` 当前**不是有效证据**，而是 REA 的结构化错误回执——`inspect_binary_layout` 请求的 `pwntools-elf` provider 需要 `REA_PWNTOOLS_PYTHON` 指向一个自带 pwntools 4.15.0 / pyelftools 0.33 / Unicorn 2.1.2 的 Python，未配置时返回 `{"code":"provider_unavailable","category":"unavailable"}`（`retryable: true`）。因此本项目的段/节布局结论来自 `inspect_artifact` 与 `inspect_managed_*`，**不来自**这四个文件；它们保留在库中是为了让失败可追溯，而不是充当证据。同理，`RhinoCommon_dll_managed_members.json` 明确记录 `coverage.state = "unavailable"`（"Managed metadata exceeds the 67108864 byte representation budget"），全量成员枚举改用按类型名选取的 `RhinoCommon_UnsafeNativeMethods.json` 完成。

MVP 核心函数在头文件中标注的 Evidence IDs（这些 ID 是 evidence graph 的内容寻址标识，可用 REA 的 `get_evidence_bundle` / `export_evidence_bundle` 回溯）：

| Evidence ID | 对应二进制 | 支撑的结论 |
| --- | --- | --- |
| `ev_9cbc13d87953d809d4fd4b58d519236c1440f8a24ff86cf4cd6f392493be47fc` | `RhinoCommon.dll` | `Intersection.LineLine` / `LinePlane` 的 P/Invoke 边界与 `Intersection` 类成员归属 |
| `ev_835ce2eb32a8991259c74c55247c52625e8f5338d4b8ac995e2e59948370c49b` | `rhcommon_c.dll` | `ON_Intersect_LineLine (0xcc020)`、`ON_Intersect_LinePlane (0xcc040)`、`ON_Intersect_CurveLine (0xcc8a0)`、`ON_Curve_IntersectPlane (0xcc7b0)` 的桥接与转发目标 |
| `ev_8e0f70a9b13a407692a4fade183b51c80a5c4e58c0064aceed0348122824ae0d` | `tl.dll` | `TL_3DX_LineXPlane (0x305d0)`、`TL_GetClosestPoint (0x214540/0x214670)`、`TL_ArcLengthPointsOnNurb (0xe37a0)`、`TL_DeformableArc2 (0x37b220)`、`TL_DeformableCircle (0x37b790)` 的存在性与调用形态 |
| `ev_a0104bfe9cdcaa0ba40c811c92f5420ecbf14b12e183c2f73a0bb65c17c05c17` | `RhinoCommon.dll` | 场景对象 / 变换 API 的托管方法签名 |
| `ev_9c800b2a176a9e340d50209899faf78e96d40561b90a32ed8bad239f2e32b125` | `RhinoCommon.dll` | 曲线采样与包围盒相关方法 |
| `ev_9a3df491bed33dacf94031349dae4d9caeea1c03b9541689ac254293c257952c` | `RhinoCommon.dll` | 文档 / 图层 / ID 语义 |

### 8.5 证据文件自身的 SHA-256（可校验采集完整性）

| 文件 | SHA-256 |
| --- | --- |
| `summary.json` | `cefb0bc5c8e485ffe3c58377ee0b74c4afb12804c09306c44eb083787288a860` |
| `exports/tl_exports.json` | `7a2449be34b156526fd1893b06bfee000bea8d4e343a5bcc7673f42084085438` |
| `exports/rhcommon_c_exports.json` | `45c33ec0eb3ebab0e29f76ecf52e321a12ad13f996f51db5d7da61ddce64f5c5` |
| `exports/opennurbs_exports.json` | `4c9f5a09c7b8136c0f5e76d16dcbf08e8258a2ecbc90f4c87682d350d19ba684` |
| `exports/RhinoCore_exports.json` | `c7e9f4b9760d3f4a0261bea466a00fdcd54e9d9365655ca77087d66746464108` |
| `exports/tl_taxonomy.json` | `4189d91dabd16f2ddbbe68210b709417846d400caf886f80ef5d185869a04cd8` |
| `imports/tl_imports.json` | `f8a355f4719134a8cdf180a081328e0423d9a0205d51e130340c7e8883c4acdd` |
| `imports/rhcommon_c_imports.json` | `6a12f7c8d59400b9544a028c5347ff5d763a2ce8993ab978172997f202bf93f0` |
| `imports/opennurbs_imports.json` | `17972361d91ab5c391e293d8e16c2d41bcb42f16cdb62704dde36f9a990fbf6f` |
| `imports/RhinoCore_imports.json` | `d95694913a0e218f8c200a27ef35f98e618140b14722ed2e6bf56735bbf7a8da` |
| `mappings/three_tier_full_map.json` | `d13817b327ac40ba21d51083fcfdc41c59a6dc2bdb0354a4427c067614d23292` |
| `mappings/three_tier_tl_kernel_map.json` | `d26d8f1377827dead40a482c162613c7ffc774207074fceb6a3455b53b3ff190` |
| `rea/RhinoCommon_UnsafeNativeMethods.json` | `06f903cf27c8ecfafb9ff7572d558a3a37e6c538a060fc374a74f27dc58a5a9b` |
| `rea/tl_dll_inspect_artifact.json` | `a03df1647a2dc368f33ea62bc37a0d225507caef07fc11f6d4828d924cb432dd` |
| `rea/rhcommon_c_dll_inspect_artifact.json` | `e87ec7be7f9bd827108ef7d9efc7ebea6d48c267436be48450277a315840347b` |
| `rea/RhinoCommon_dll_inspect_artifact.json` | `6f1a15d4bd2fd07804d967d5370d6b37cc5090a7119bd613509cb53c7b985e53` |

复核方式：

```bash
cd /Users/waya/Projects/Rhino/re-database
shasum -a 256 summary.json exports/*.json imports/*.json mappings/*.json rea/*.json
```

### 8.6 采集工具

`../tools/extract_re_database.py`：PE 导出/导入表解析 + 三层调用链重建（产出 `exports/`、`imports/`、`mappings/`、`summary.json`）。
`../tools/disasm_function.py`：按 RVA 反汇编单个函数，用于核对 `tl.dll`/`rhcommon_c.dll` 中具体分支行为（头文件注释里的"Disassembly verification"段落即由它产出）。

---

## 9. 当前限制与后续演进边界

MVP 的目标是**打通一条端到端可验证的链路**（几何 → 场景 → 运算 → 序列化 → 证据），不是复刻 Rhino。以下是明确的边界，避免误用：

### 9.1 几何能力边界

- **曲面求交（SSX）不完整**。当前实现的求交覆盖「线-线 / 线-平面 / 曲线-直线 / 曲线-平面」。**曲面-曲面**只具备 openNURBS 层面的基础能力，缺少 Rhino 级的完整拓扑跟踪：交线在参数域的**分支识别、闭合环检测、奇点/切触穿越、容差传播与接缝缝合**均未实现，因此不能作为布尔运算的可靠前置。
- **Brep 布尔流水线缺失**。已有的是布尔所需的**分类器**（`TL_Brep::PointIsOnFace` 的 Outside/Inside/Boundary 判定、`TL_ClassifyPointOnBrepFace`）与拓扑工具，但**没有**：面-面分割（`ON_BrepSplitter` 级）、交线嵌入、共面面合并、非流形结果清理、内外壳判定（射线奇偶/绕数）。因此 `TL_Brep` 目前可用于**查询与判定**，不可用于**实体布尔求值**。
- **网格布尔与 Remesh 未包含**。`TL_MeshSurfaceGrid` 只做规则参数网格；没有自适应细分、特征保持、网格修复（孔洞/自交/翻转法向）。
- **NURBS 拟合/重建有限**。`TL_LoftCurves` 只处理相容截面；`TL_Sweep1Rail` 提供单轨扫掠（平行传输取帧 + loft 蒙皮，剖面保真度受 64 CV 限制）。仍然**没有**曲面拟合（`TL2_FitSurface` 系列）、双轨扫掠、偏置曲面、圆角/倒角。
- **SubD 仅一种基元**。`TL_CreateSubDBox` 之外没有 SubD 挤出/桥接/边权重编辑，也没有 SubD → Brep 的精确转换（只有极限曲面求值）。
- **`RemoveKnots` 是简化版**。以"移除后重拟合误差"为判据，未实现原产品在有理曲线权重扰动、跨接缝节点、曲面双向耦合情形下的完整策略。
- **数值鲁棒性**：容差是显式参数而非全局模型容差；极端尺度（1e-9 或 1e9 量级）下的求交/最近点可能退化。没有区间算术或多精度回退。

### 9.2 场景与数据模型边界

- **场景是"几何容器"，不是 Rhino 文档**。没有 Undo/Redo、没有事务、没有对象历史（construction history）、没有组/块实例、没有物件属性（`ObjectAttributes`）的完整模型——只有 `name`、`layer_index`、`id`、`uuid`。
- **图层是扁平表**。没有图层嵌套、图层状态、每视口图层覆盖、图层锁定/打印颜色等。
- **无线框/着色/材质/渲染**。Rhino 专有的显示管线、材质库、贴图映射、RDK（Render Development Kit）数据完全不在 MVP 内。OpenTL 不产生任何像素。
- **无命令层与交互**。没有 `RhinoApp`/`CRhinoCommand` 等价物，没有选择集、捕捉、Gumball。CLI 仅用于演示与检视。

### 9.3 序列化边界

- **`.3dm` 往返只保证 MVP 的 7 类对象 + 图层属性**。具体是：`Point`/`Line`/`Polyline`/`Circle`/`Arc`/`Plane`/`NurbsCurve` 的几何参数、名称、图层归属、稳定 ID/UUID。
- **第三方 `.3dm` 只读容忍，不做保真重写**。含**注解（Annotation）**、**块定义/块实例（InstanceDefinition/InstanceReference）**、**群组**、**材质与渲染网格**、**RDK 数据**、**用户数据（UserData）插件对象**、**历史记录**的文件可以打开并列出可识别的几何，但：
  - 未识别的对象在保存时**不会被重建**（属于"读时忽略、写时丢弃"的边界）；
  - 不做 `ONX_Model` 的全组件保真（`m_object_table` 之外的 `m_idef_table`、`m_texture_mapping` 等表不参与往返）；
  - 不保证 `.3dm` 版本降级（写出的版本由 openNURBS 决定）。
- **`.otscene.json` 是本项目私有格式**，不是行业标准；版本号变更不承诺向后兼容（MVP 阶段只有 `version: 1`）。
- **Plane 的表示是有损中介**：`.3dm` 侧用 `[-10,10]²` 的 `ON_PlaneSurface` 承载无限平面。往返靠 `opentl_kind`/`opentl_id` UserString 折叠，**若第三方编辑器删除了 UserString，Plane 会被当作 20×20 的有界平面曲面读回**。

### 9.4 证据与合规边界

- **只有静态证据**。没有动态插桩、没有 API 跟踪、没有内存快照。因此"行为对齐"的强度是：**签名/控制流/边界条件对齐**，而非"逐指令等价"。原产品内部未导出的私有辅助函数（`tl.dll` 中大量 `core_utils`）没有被复现。
- **`tl.dll` 的 8 642 个导出中，MVP 覆盖不到 1%**。`core_utils`(3 008)、`brep_topology`(1 678)、`nurbs_eval_core`(739) 的主体均未触及；`subd` 分类只有 2 个导出，说明原产品的 SubD 主体不在 `tl.dll` 而在别处（`RhinoCore.dll`），本项目未跟进该线索。
- **四个 `*_binary_layout.json` 是失败回执**（见 8.4），段布局结论来自 `inspect_artifact`；若后续配置 `REA_PWNTOOLS_PYTHON`，应重跑并把结果并入证据集，同时更新本 README 的对应行。
- **证据合订本已落盘，但三层链并非处处闭合**。`re-database/rea/mvp_static_evidence.json` 已记录 12 个函数、1 297 条逐指令清单、80 个唯一 IAT 目标，三层校验 26/29 通过。未通过的 3 项集中在 `tl.dll` 侧：`TL_3DX_LineXPlane`、`TL_GetClosestPoint`（4 参/7 参重载）在导出表中存在，但 `three_tier_full_map.json` 中没有指向它们的 tier-2 → tier-3 引用边，因此**只有"导出存在"这一层证据**，其内部实现是按数学等价重写的（`re-database/rea/mvp_static_evidence.json` 的 `functions[].opentl_status` 已逐项标注 `gap` / `partial` / `declared_no_implementation`）。不要把这三项当作"逐指令对齐"。
- **三处注释/实现曾与二进制不符，已在阶段 3 订正**：`TL_IsMeshEdgeUnwelded`（要求 `m_topf_count == 2` 而非 `<= 1`，判据是"四个网格顶点索引两两互异"）、`TL_Brep::PointIsOnFace`（越界返回 **−1** 而非 0；容差钳位到 `[sqrt(DBL_EPSILON), 1% 面域长度]`；iso 参数分解先于 loop 包含判定）、`TL_DeformableArc2`（结果参数域是**弧长** `[0, Length()]`，不是弧度域）。修正后的行为均有测试断言。
- **`ON_Curve::IntersectCurve` 不可用**：`external/opennurbs` 版本没有 `ON_X_EVENT`，无法复刻 `ON_Intersect_CurveLine` 的下游路径。次级策略改为自研 `TL_IntersectCurveLineBySubdivision`（Lipschitz 严格下界剪枝 + Newton），用于对拍而非冒充原实现。
- **四视图渲染只到线框 + 平面着色**：`viewport/` 提供世界↔屏幕、拾取射线、平移/缩放/缩放到适合、JSON 往返，以及 Vulkan 离屏渲染（Catppuccin Mocha 主题、4× MSAA、D32 深度、Gouraud/Blinn-Phong 着色三角形）。仍然**没有**窗口/交换链、鼠标交互、Gumball、阴影、透明排序与 SSAO。`m_pbox` 类缓存也未实现（`PointIsOnFace` 的 pbox 早退由 trim 曲线采样推导）。
- **不执行原始二进制**是硬约束：任何"跑一下原 DLL 看看结果"的验证方式都被排除。这既是合规要求，也意味着**没有金标准输出可比对**——正确性依赖数学性质（往返固定点、距离不变性、弧长单调性）与 openNURBS 自身的参考实现，而不是与原产品逐值对齐。

### 9.5 后续演进的优先级建议

| 优先级 | 方向 | 依赖 |
| --- | --- | --- |
| P0 | 曲面-曲面求交的拓扑跟踪（分支/闭环/奇点） | openNURBS `ON_Intersect` 基座 + 参数域步进器 |
| P0 | Brep 布尔流水线（分割 → 嵌入 → 分类 → 合并） | SSX + `PointIsOnFace` 分类器 |
| P1 | 扫掠（1/2 rail）与偏置曲面 | `TL_LoftCurves` 相容化框架 |
| P1 | SubD ↔ Brep 双向转换 + 边权重 | `TL_CreateSubDBox` + `ON_SubD` 极限求值 |
| P1 | 模型级容差与鲁棒性（区间算术回退） | 求交/最近点内核 |
| P2 | 对象属性完整模型（颜色/线型/材质引用）与 Undo 栈 | 场景层重构 |
| P2 | `.3dm` 组件保真（idef、annotation、userdata 透传） | `ONX_Model` 全表读写 |
| P2 | 四视图渲染后端（OpenGL/Metal）、视口内图层覆盖、Gumball | `viewport/` 相机与投影（已就绪） |
| P3 | 显示网格（渲染用三角化、自适应细分） | 网格化 + 光栅化 |

---

## 10. 致谢与许可

- **openNURBS**：几何数据结构、NURBS 求值、Brep/SubD/Mesh 容器与 `.3dm` 归档格式均来自 McNeel 的开源发布（`external/opennurbs`，v8.35 分支），遵循其自带 `LICENSE`。本项目的价值在于其上的组织方式与算法补齐，而非重复实现 NURBS。
- **逆向证据**：`re-database/` 中的导出表、导入表、三层映射与 REA 证据对象仅用于**接口与行为规格**的确定。所有 OpenTL 源码为独立编写，不含原产品反编译代码、不含逐字复制的专有实现。
- **商标**：Rhino、RhinoCommon、openNURBS 是 Robert McNeel & Associates 的商标/产品名。本项目为独立的互操作性研究实现，与其无隶属或背书关系；名称中的 "Rhino-like" 仅描述 API 形态的对齐目标。

---

*本文档随 `opentl` 版本演进。所有功能声明以 `include/opentl/` 下的公开头文件为准；所有证据声明以 `re-database/` 下的原始 JSON 为准。*
