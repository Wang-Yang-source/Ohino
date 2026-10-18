# OpenTL Studio PRD：对标 Rhino 9 WIP 的 Grasshopper 脚本引擎与 GUI 工作台

- **文档版本**：v1.0（2026-10-19）
- **对标基准**：Rhino 9 WIP（`9.0.26286`）四视图建模工作台 + Grasshopper 1/2 参数化节点画布
- **图形与窗口栈**：标准 Khronos Vulkan 1.2（macOS 经 MoltenVK 桥接，Windows/Linux 原生驱动）+ SDL3 跨平台窗口与事件系统
- **视觉语言**：Catppuccin Mocha 全家桶配色（4× 硬件 MSAA、`D32_SFLOAT` 深度缓冲、Blinn-Phong 着色曲面、亚像素抗锯齿粗轮廓线与 Bezier 节点连线）

---

## 1. 背景与目标

OpenTL 已在阶段 1–8 完成几何内核的核心闭环：
- 基础与高阶几何（`ON_Point` / `ON_Line` / `ON_Arc` / `ON_Circle` / `ON_NurbsCurve` / `ON_NurbsSurface` / `ON_Brep` / `ON_SubD` / `ON_Mesh`）；
- 求交与最近点（`TL_3DX_LineXPlane`、双策略 `TL_IntersectCurveLine`、双策略 `TL_IntersectCurves`、支持周期闭合与相切母线的 `TL_SurfaceSurfaceIntersect`）；
- 造型与实体布尔（`TL_DeformableArc2`、`TL_DeformableCircle`、`TL_LoftCurves`、`TL_Sweep1Rail`、`TL_CapMeshOpenBoundaryLoops`、van Oosterom–Strackee 立体角广义绕数 `TL_MeshWindingNumber`、平面切割封口 `TL_SliceMeshWithPlane`、多面体 CSG `TL_MeshBoolean`、`TL_CreateSubDBox` / `TL_MeshFromSubD`）；
- 工业级 `.3dm` 档案全量加载与四视图 Vulkan 离屏渲染（已在 383 对象 / 309 Brep / 5215 面 / 131.9 万三角形的真实笔记本电脑 `.3dm` 上验收通过）。

下一阶段（阶段 9 & 10）的核心目标是**补齐 Rhino 9 最具标志性的两大支柱**：
1. **Grasshopper (`.gh` / `.ghx`) 原生脚本加载器与有向无环图（DAG）参数化求解引擎**；
2. **对标 Rhino 9 WIP 布局的 GUI 工作台（含四视图 3D 视口 + Grasshopper 节点编辑器分屏画布 + 命令行与图层面板 + 实时交互窗口/离屏高分渲染双模）**。

---

## 2. 子系统一：Grasshopper 脚本加载与参数化求解引擎（`include/opentl/gh/tl_gh.h`）

### 2.1 `.gh` 二进制与 `.ghx` XML 档案格式规范（100% 兼容 Rhino 9 官方教程档案）

经对 `/Users/waya/Projects/Rhino/Rhino 9 WIP/Plug-ins/Grasshopper/Tutorials/*.gh` 全部 8 个官方 `.gh` 档案实测验证，Grasshopper `.gh` 档案底层为 `GH_Archive` 树结构：

1. **压缩层**：
   - `.gh` 文件整体为 `.NET DeflateStream` 原始流（`zlib` 窗口比特 `wbits = -15` raw deflate，或标准 zlib/gzip 魔数自动探测）。
   - `.ghx` 文件为未压缩 UTF-8 XML 树（`<Archive><chunks><chunk name="Root">...</chunk></chunks></Archive>`）。
2. **`GH_Chunk` 二进制递归块结构**：
   - `name`：7-bit 变长整数长度前缀 + UTF-8 字符串；
   - `index`：`int32` 小端序（无索引为 `-1`）；
   - `item_count`：`int32` 小端序；
   - `chunk_count`：`int32` 小端序；
   - `items[0 .. item_count-1]`：每个 Item 含 `name`（7-bit 字符串）、`index`（`int32`）、`type`（`int32` `GH_Types` 枚举码）及对应二进制负载：
     - `1` (`bool`, 1B), `2` (`byte`, 1B), `3` (`int32`, 4B), `4` (`int64`, 8B), `5` (`float32`, 4B), `6` (`float64`, 8B), `7` (`decimal`, 16B), `8` (`Date`, 8B), `9` (`Guid`, 16B LE), `10` (`String`, 7-bit 长度前缀)
     - `20` (`byte[]`, 4B len + bytes), `21` (`double[]`, 4B len + doubles)
     - `30` (`Drawing_Point`, 2×int32), `31` (`Drawing_PointF`, 2×float32), `32` (`Drawing_Size`, 2×int32), `33` (`Drawing_SizeF`, 2×float32), `34` (`Drawing_Rectangle`, 4×int32), `35` (`Drawing_RectangleF`, 4×float32), `36` (`Drawing_Color`, 4B ARGB), `37` (`Drawing_Bitmap`, 4B len + PNG bytes)
     - `50` (`GH_Point2D`, 2×double), `51` (`GH_Point3D`, 3×double), `52` (`GH_Point4D`, 4×double)
     - `60` (`GH_Interval1D`, 2×double), `61` (`GH_Interval2D`, 4×double)
     - `70` (`GH_Line`, 6×double), `71` (`GH_BoundingBox`, 6×double), `72` (`GH_Plane`, 9×double: `Origin + XAxis + YAxis`)
     - `80` (`GH_Version`, 3×int32: `major, minor, revision`)
   - `chunks[0 .. chunk_count-1]`：递归子块。

### 2.2 节点图模型（`TL_GhDocument`）

从 `Root -> Definition -> DefinitionObjects` 提取：
- **`TL_GhNode`（组件/参数节点）**：
  - `component_guid`：组件类型 GUID；
  - `instance_guid`：节点实例唯一 UUID；
  - `name` / `nickname` / `description`；
  - `bounds` $(x, y, w, h)$ 与 `pivot` $(x, y)$：在 Grasshopper 2D 画布上的精确排版坐标；
  - `hidden`：是否在 3D 视口隐藏预览；
  - `is_slider` & `TL_GhSliderState`：`min_val`, `max_val`, `value`, `digits`, `is_integer`（支持从 `.gh` 读取，并支持在 GUI/代码中动态修改 `SetSliderValue`）；
  - `panel_text`：`Panel` / `Scribble` 文本内容；
  - `group_members`：`Group` 节点的包含实例 UUID 列表与分组框颜色；
  - `inputs` / `outputs`（`std::vector<TL_GhPort>`）：每个端口包含 `instance_guid`、`name`、`nickname`、`pivot`、`bounds`、`source_guids`（上游连线 UUID 列表）以及 `persistent_values`（内嵌持久化常量：数值、3D 点、直线、区间、平面、字符串）。

### 2.3 DAG 拓扑排序与内置组件求解器（`TL_GhSolver`）

1. **连线解析**：构建 `port_or_node_guid -> (node_index, output_port_index)` 映射表，生成有向边列表 `TL_GhWire`；
2. **Kahn 拓扑排序**：按依赖关系排序节点，检测并跳过环路；
3. **内置组件计算库（直接调用 OpenTL C++ 内核）**：
   - **输入与数值类**：`Number Slider`, `Control Knob`, `Digit Scroller`, `Number`, `Panel`, `Addition`, `Subtraction`, `Multiplication`, `Division`, `Average`, `Range`, `Bounds`, `Remap Numbers`, `Format`, `Expression`（支持基础多项式与算术求值）；
   - **点与向量类**：`Point`, `Construct Point`, `Deconstruct Point`, `Distance`, `Vector XYZ`, `XY Plane`, `XZ Plane`, `YZ Plane`, `Plane Offset`;
   - **曲线类**：`Line`, `Line SDL`, `Circle`, `Circle 3Pt`, `Arc`, `Arc SED`, `Nurbs Curve`, `Interpolate data` / `Interpolate`, `Divide Curve`, `Curve Closest Point`（调用 `TL_GetCurveClosestPoint`）, `Offset Curve`, `Join Curves`, `Project`;
   - **曲面、网格与实体布尔类**：`Surface`, `Loft`（调用 `TL_LoftCurves`）, `Sweep1`（调用 `TL_Sweep1Rail`）, `Bounding Box`, `Box Corners`, `Mesh Plane`, `SubD Box`（调用 `TL_CreateSubDBox` + `TL_MeshFromSubD`）, `Brep | Line`（调用 `TL_IntersectCurveLine`）, `Slice`（调用 `TL_SliceMeshWithPlane`）；
4. **3D 预览生成与 Bake（`BakeToScene`）**：
   - 将所有未隐藏（`!hidden`）节点的输出几何体（3D 点、曲线、平面、网格/曲面）自动汇总到 `SceneDocument` 与着色 `ON_Mesh` 列表中，供四视图实时预览或导出 `.3dm`。

---

## 3. 子系统二：对标 Rhino 9 WIP 的 GUI 工作台（`include/opentl/gui/tl_gui.h`）

### 3.1 界面分区架构（1920×1080 标准工作台布局）

```
+--------------------------------------------------------------------------------------------------+
| [Menu / Title Bar]  OpenTL Studio 9 WIP  —  File | Edit | View | Curve | Surface | SubD | GH     |
+--------------------------------------------------------------------------------------------------+
| [Command Bar]       Command: _Grasshopper _Open "07 Column Arcade.gh" (Solved 34 nodes in 0.8ms) |
+--------------------------------------------------------------------------------------------------+
| [Toolbar Tabs]      [Standard] [CPlanes] [Set View] [Select] [Curve] [Surface] [Solid] [SubD] [GH]|
+----------------+---------------------------------------------+-----------------------------------+
| [Left Dock]    | [Center-Left: 2x2 3D Four-Viewports]        | [Center-Right: Grasshopper Canvas]|
|                | +--------------------+--------------------+ | +-------------------------------+ |
| Layers (14)    | | TOP (XY)           | PERSPECTIVE (3D)   | | | GH Document: 07 Column Arcade | |
|  [o] Default   | |                    |                    | | |                               | |
|  [o] Chassis   | |   (Ortho Grid +    |  (4x MSAA Shaded   | | |  [Slider]--->\                | |
|  [o] Display   | |    Wireframe +     |   Solids + GH      | | |               [Component]-->  | |
|  [o] Keycaps   | |    Shaded Solids)  |   Preview Curves)  | | |  [Point ]--->/                | |
|  [o] Legends   | +--------------------+--------------------+ | |                               | |
|  [o] GH_Preview| | FRONT (XZ)         | RIGHT (YZ)         | | |  (Bezier Wires, Groups,       | |
|                | |                    |                    | | |   Sliders, Panels, Ports)     | |
| Properties     | |                    |                    | | +-------------------------------+ |
|  Objects: 383  | +--------------------+--------------------+ | [GH Sliders & Inspector Panel]    |
|  GH Nodes: 34  | [Viewport Tabs: Top | Persp | Front | Right]|  Samples: [====|----] 100.0       |
+----------------+---------------------------------------------+-----------------------------------+
| [Status Bar]   X: 138.26  Y: -106.94  Z: 12.40 | Millimeters | Grid Snap | OSnap | Gumball | M4  |
+--------------------------------------------------------------------------------------------------+
```

### 3.2 核心渲染与交互特性

1. **双模运行（Windowed Interactive + Headless Framebuffer）**：
   - **离屏高分渲染模式（`opentl_cli --render-gui`）**：无需窗口服务器即可通过 Vulkan 1.2 渲染完整的 1920×1080 Rhino 9 WIP + Grasshopper 分屏工作台 UI 到 PNG，100% 可用于自动化视觉回归测试与微信直发预览；
   - **实时桌面窗口模式（`opentl_gui` / `opentl_cli --gui`）**：基于 SDL3 创建高分窗口，支持：
     - 鼠标在任意 3D 视口内拖拽平移（`Pan`）、右键轨道旋转（`Orbit`）、滚轮锚点缩放（`Zoom`）、双击最大化/恢复单视口、快捷键 `F` 缩放到适合（`ZoomToFitAll`）；
     - 鼠标在 Grasshopper 画布或右侧参数栏直接拖动 `Number Slider` 滑块，实时触发 `TL_GhSolver::Evaluate()` 并刷新左边的 3D 四视图！
2. **Grasshopper 节点画布矢量级 GPU 渲染**：
   - 背景深色网格点阵（Catppuccin Mantle `#181825` + Surface0 网格）；
   - `Group` 半透明圆角分组框；
   - `Component` 标准三段式节点盒（左侧输入端口区、中央竖排/横排 NickName 标题区、右侧输出端口区）+ 端口小圆环；
   - `Number Slider` 带实时进度条、滑块手柄与当前数值读数；
   - `Panel` 黄色便签式数据看板；
   - `Wire` 三次 Bezier 平滑切线连线（单值实线、列表双线标识）。

---

## 4. 验收标准（Definition of Done）

1. **`.gh` 官方档案解析率**：Rhino 9 WIP 自带的全部 8 个官方 `.gh` 教程文件（`00` 至 `07`）100% 字节级无错加载，节点数、端口数、连线数、Slider 参数与 PersistentData 全部精确提取。
2. **DAG 参数化求解闭环**：修改 `Number Slider` 参数后重新求解，下游输出点/曲线/曲面数量与几何坐标实时变化，且烘焙到 `SceneDocument` 后可在四视图中正确渲染。
3. **GUI 工作台渲染与交互**：1920×1080 Rhino 9 风格工作台（含菜单栏、命令栏、工具栏、左侧图层面板、中央 2×2 四视图、右侧 Grasshopper 节点图画布、底部状态栏）通过 Vulkan 4× MSAA 渲染输出，并通过 `vision_analyze` 视觉验收与 `ctest` 自动化测试。
