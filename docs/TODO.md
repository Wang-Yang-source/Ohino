# Ohino (OpenTL Studio) — 开发路线图与 TODO 清单

> **Ohino** — *A visual programming and parametric modeling tool and design software.*
> Repository: <https://github.com/Wang-Yang-source/Ohino.git>

---

## 已完成阶段（Stages 1 – 8）

- [x] **Stage 1–2：Rhino-like 原生几何内核与场景 MVP（v0.1.0）**
  - [x] 三层架构对齐（`opentl::mvp` ↔ `rhcommon_c_bridge` ↔ `tl_*` 内核层 + 开源 `openNURBS`）
  - [x] 7 类基础几何对象（`Point`, `Line`, `Polyline`, `Circle`, `Arc`, `Plane`, `NurbsCurve`）、稳定 64 位 ID、确定性 UUID、多维查询与变换
  - [x] `.otscene.json` 与 `.3dm`（`ONX_Model`）确定性无损往返序列化
- [x] **Stage 3：二进制静态证据对齐、四视图相机内核与单轨扫掠**
  - [x] `TL_IsMeshEdgeUnwelded`、`TL_Brep::PointIsOnFace`、`TL_DeformableArc2`、`TL_3DX_LineXPlane`、`TL_GetClosestPoint(ON_Line, ON_Line)`
  - [x] `TL_IntersectCurveLineBySubdivision`（Lipschitz 严格下界独立第二算法对拍）
  - [x] `TL_FourViewLayout` 四视图内核（`Top` / `Perspective` / `Front` / `Right`、拾取射线 `ScreenToRay`、`ZoomToFit`、`PickPoint`、JSON 持久化）
  - [x] `TL_Sweep1Rail`（平行传输标架 + `TL_LoftCurves` 蒙皮）
- [x] **Stage 4–5：跨平台 Vulkan 1.2 离屏渲染器与 Catppuccin Mocha 主题**
  - [x] 标准 Khronos Vulkan 1.2（macOS MoltenVK / Linux & Windows 原生驱动）、内嵌 SPIR-V 着色器、无损 PNG 编码器
  - [x] `TL_CatppuccinMochaTheme` 调色板、4× 硬件 MSAA（`VK_SAMPLE_COUNT_4_BIT`）、`D32_SFLOAT` 深度缓冲、双管线（Blinn-Phong 着色三角面 + 抗锯齿线框）
  - [x] 网格开放边界检测 `TL_CountMeshBoundaryEdges` 与硬边端盖封口 `TL_CapMeshOpenBoundaryLoops`
- [x] **Stage 6–7：曲线×曲线 / 曲面×曲面（SSX）求交、周期 NURBS 拟合与屏幕空间粗轮廓线**
  - [x] `TL_IntersectCurves`（AABB 分枝限界）与 `TL_IntersectCurvesByGrid`（三角不等式网格下界）双求解器对拍
  - [x] `TL_SurfaceSurfaceIntersect`（4D 最小范数 Gauss-Newton 校正 + 切向追踪 + 相切母线分支 `MarchSSXTangent` + 真周期三次 B 样条 `FitPeriodicTraceToNurbs`）
  - [x] `TL_ExtractMeshFeatureEdges`（Boundary / Crease / Silhouette）+ 屏幕空间亚像素平行簇粗线 + 正交视口远平面背景网格垫底
- [x] **Stage 8：立体角广义绕数、平面布尔切割封口、多面体 CSG 布尔、SubD 网格化与真实 `.3dm` 渲染**
  - [x] `TL_MeshSignedVolume`、`TL_MeshWindingNumber`（van Oosterom–Strackee 立体角）、`TL_MeshContainsPoint`
  - [x] `TL_SliceMeshWithPlane`（壳体/端盖分层顶点焊接，满足体积守恒 $V_+ + V_- = V$）与 `TL_MeshBoolean`（`Union` / `Intersection` / `Difference`）
  - [x] 修复 `TL_CreateSubDBox` 右手系角点顺序并新增 `TL_MeshFromSubD`
  - [x] 新增 `TL_ExtractBrepMesh`、`TL_ExtractBrepEdgePolylines` 与 `--input-3dm`，完成 383 对象 / 309 Brep / 5215 面 / 131.9 万三角形笔记本电脑 `.3dm` 渲染

---

## 当前推进阶段（Stages 9 – 12）

- [x] **Stage 9：文档体系重构与 GitHub 仓库标准化**
  - [x] 编写 PRD 文档 `docs/PRD_RHINO9_GH_AND_GUI.md`
  - [x] 编写路线图与任务清单 `docs/TODO.md`
  - [x] 重写并美化仓库根目录 `README.md`（项目名 **Ohino**，包含核心简介 `A visual programming and parametric modeling tool and design software.`）
- [x] **Stage 10：Grasshopper (`.gh` / `.ghx` / `.ghz`) 脚本加载与 DAG 参数化求解引擎（`include/opentl/gh/tl_gh.h`、`src/gh/tl_gh.cpp`）**
  - [x] 实现 `GH_Archive` 二进制 `.gh`（raw deflate `wbits = -15` + 7-bit 字符串 + 25 种 `GH_Types` 强类型 Item + 递归 `GH_Chunk` 树）解析器
  - [x] 实现 `.ghx`（XML 树）与 `.ghz`（ZIP 容器）格式解析支持
  - [x] 提取完整的 `TL_GhDocument`：组件节点 `TL_GhNode`、输入/输出端口 `TL_GhPort`、连线 `TL_GhWire`、分组 `TL_GhGroup`、`Number Slider` 状态与 `PersistentData` DataTree 分支数据
  - [x] 实现 Kahn 拓扑排序与 25+ 核心 Grasshopper 几何/数学组件求解器（`Number Slider`, `Point`, `Construct Point`, `Line`, `Line SDL`, `Circle`, `Circle 3Pt`, `Arc SED`, `Nurbs Curve`, `Interpolate data`, `Range`, `Divide Curve`, `Curve Closest Point`, `Distance`, `Bounding Box`, `Box Corners`, `XY Plane`, `Plane Offset`, `Addition`, `Subtraction`, `Multiplication`, `Division`, `Average`, `Remap Numbers`, `Surface`, `Loft`, `Sweep1`, `SubD Box`, `Brep | Line`, `Custom Preview`, `Panel`）
  - [x] 实现 `SetSliderValue` 动态调参重算与 `BakeToScene` 3D 几何烘焙
  - [x] 通过 Rhino 9 WIP 全部 8 个官方 `.gh` 教程档案（`00`–`07`）100% 字节级解析与求解测试
- [x] **Stage 11：对标 Rhino 9 WIP 的跨平台 GUI 工作台（`include/opentl/gui/tl_gui.h`、`src/gui/tl_gui.cpp`）**
  - [x] 实现 1920×1080 Rhino 9 WIP 标准工作台布局（顶部菜单栏、命令行提示栏、工具栏选项卡、左侧图层与对象属性面板、底部坐标与 OSnap/Gumball 状态栏）
  - [x] 中央左半区：2×2 四视图（`TOP` / `PERSPECTIVE` / `FRONT` / `RIGHT`）+ 3D Gumball 操纵器（X/Y/Z 平移箭头、旋转弧、缩放柄）+ OSnap 捕捉高亮标记
  - [x] 中央右半区：Grasshopper 节点画布 GPU 渲染器（深色点阵网格、`Group` 半透明圆角分组框、三段式胶囊组件盒、输入/输出端口圆环、三次 Bezier 连线、`Number Slider` 实时滑块轨、`Panel` 黄色数据看板）
  - [x] 支持 `--render-gui [--input-3dm <file.3dm>] [--input-gh <file.gh>]` 离屏 Vulkan 4× MSAA 渲染输出与 `--gui` SDL3 实时交互窗口（视口平移/旋转/缩放 + 拖动 GH Slider 实时重算 3D 几何）
- [x] **Stage 12：全量回归验证、截图验收与 GitHub 持续同步**
  - [x] 新增单元测试 Section 20（Grasshopper `.gh`/`.ghx` 解析与 DAG 求解）与 Section 21（Rhino 9 GUI + GH 画布渲染）
  - [x] 运行 `ctest` 全量通过 + `vision_analyze` 检验 GUI 渲染图
  - [x] `git commit` 并 `git push origin main` 同步至 <https://github.com/Wang-Yang-source/Ohino.git>
- [x] **Stage 13–15：10-Tab 电池面板、交互式连线与导出、三平台（macOS / Linux / Windows）原生打包发布（v0.2.0）**
  - [x] `TL_GhComponentCatalog` 10 大分类选项卡、画布拖拽连线、`.gh` / `.ghx` 导出与无损往返
  - [x] macOS (`.dmg`/`.pkg`/`.tar.gz`)、Linux (`.tar.gz`/`.zip`)、Windows MinGW-w64 PE32+ x64 (`.exe` NSIS 安装包 / `.zip` 便携版，含交叉编译 `SDL3.dll` + `vulkan-1.dll`)
- [x] **Stage 16：182 个 Grasshopper 算子 100% 求值覆盖、参数域 Trimmed Brep 布尔分割与 Studio 命令行交互闭环**
  - [x] 补齐 `TL_GhComponentCatalog` 全部 182 个组件在 `TL_GhDocument::EvaluateSingleNode` 中的真实内核求值（`Evaluate Surface`, `Surface Closest Point`, `Cap Holes`, `Mesh Box`, `Deconstruct Mesh`, `Mesh Edges`, `Brep | Plane`, `Surface | Surface`, `Solid Union/Difference/Intersection`, `Orient`, `Linear/Polar Array` 等）
  - [x] 实现参数域 Trimmed `ON_Brep` 平面切割封口 `TL_SplitBrepWithPlane` 与 CSG 布尔运算 `TL_BrepBoolean`（自动构建 `ON_BrepLoop` / `ON_BrepTrim` / `ON_BrepEdge` 并满足体积守恒）
  - [x] 实现 Studio 交互式命令行 `TL_ExecuteStudioCommand`（支持 `_Point`, `_Line`, `_Circle`, `_Arc`, `_Box`, `_BooleanUnion`, `_Delete`, `_Undo`, `_ZoomExtents`, `_Save3dm`, `_SaveGh`, `_GhAdd` 及键盘实时输入与 `Cmd+Z` 撤销）
- [x] **Stage 17：曲面造型内核补齐（Revolve / Edge Surface / Fillet）、SubD 图元与视口对象拾取**
  - [x] `TL_RevolveCurve`：基于 `ON_RevSurface::GetNurbForm` 的精确旋转曲面（整圈/任意角度区间），球面采样半径误差 `4.4e-16`
  - [x] `TL_CreateEdgeSurface`：2 / 3 / 4 条边界曲线的 Coons（Gordon）双线性混合补面，角点与边中点误差 `0.0`
  - [x] `TL_FilletCurves`：共面曲线精确 G1 圆角弧 + 双曲线裁剪，切点误差 `0.0 / 1.8e-15`，并拒绝平行/非切向退化构型
  - [x] `TL_CreateSubDFromMesh` / `TL_CreateSubDCylinder` / `TL_CreateSubDSphere`：Catmull-Clark SubD 图元，圆柱/球体半径按极限曲面自动补偿（`5.000` / `4.000` 实测命中）
  - [x] `TL_PickSceneObjectAtScreen`：屏幕空间对象拾取（7 类几何采样投影），选择状态驱动 `_Delete` 与 Gumball 支点重定位
  - [x] Grasshopper 目录扩充至 188 个算子（`Revolve`, `Edge Surface`, `Fillet`, `SubD From Mesh`, `SubD Cylinder`, `SubD Sphere`）
  - [x] 新增测试 Section 21：`opentl_tests` 21 组 **3003 项断言 0 失败**，`ctest 5/5` 全通过
- [x] **Stage 18：Brep 实体边缘圆角/倒角、Window/Crossing 矩形框选、黄色选中高亮与 Gumball 拖拽实时几何变换**
  - [x] `TL_ChamferBrepEdge` / `TL_FilletBrepEdge`：Brep 实体棱边精确倒角面与等半径滚球圆柱 NURBS 圆角面（`ON_RevSurface::GetNurbForm` + 端盖扇形水密缝合），`10×10×10` 立方体实测体积 `V_chamfer = 980.0000`（解析解 `980.0000`）、`V_fillet = 991.3655`（解析解 `991.4159`）
  - [x] `TL_SelectSceneObjectsInRect`：支持 Rhino 标准左→右 Window 框选（全包含，蓝色框）与右→左 Crossing 交选（相交即选，绿色框），选中对象在四视图实时绘制黄色高亮线框
  - [x] 拖拽 3D Gumball X/Y/Z 轴实时平移选中对象并自动压入 `undo_stack`；新增 `_Move`、`_Rotate`、`_Scale` 命令；Grasshopper 目录扩至 **190** 个算子（新增 `Fillet Edge`、`Chamfer Edge`），`opentl_tests` **21 组 3020 项断言 0 失败**，`ctest 5/5` 全过
- [x] **Stage 19：变半径滚球圆角、弯曲面三次 2D NURBS 参数域裁剪、3D 网格射线求交与 OBJ/STL 工业格式导入导出**
  - [x] `TL_VariableFilletBrepEdge`：Brep 棱边变半径滚球圆角（`r_start -> r_end` 线性过渡），构建 (1,3) 阶 NURBS 过渡面与两端圆弓封口，`10×10×10` 立方体 `r = 1 -> 3` 实测水密体积 `V = 990.6459`（解析解 `990.7006`），接入 Grasshopper `Variable Fillet Edge`（总计 **191/191** 算子）
  - [x] 升级 `AppendTrimmedBrepFaceFromUvLoop`：对非平面弯曲曲面自动拟合三次（4 控制点）2D `(u, v)` NURBS 裁剪曲线与 3D 边缘曲线
  - [x] `TL_RaycastMesh`：AABB 快速剔除 + Möller–Trumbore 精确射线-三角形/四边形求交（返回最近距离 `t`、命中点、面索引与外法向）
  - [x] `TL_ExportMeshOBJ` / `TL_ImportMeshOBJ` 与 `TL_ExportMeshSTL` / `TL_ImportMeshSTL`（二进制 + ASCII 自动识别与顶点焊接），Studio 新增 `_ExportObj` / `_ExportStl` 命令，无损体积往返校验通过（`V = 980.0000`），`opentl_tests` **21 组 3034 项断言 0 失败**，`ctest 5/5` 全过
- [x] **Stage 20：自交曲面求交、STEP / IGES 工业标准交换格式、3D Gumball 旋转弧/缩放柄全自由度交互与 v0.3.0 三平台发布**
  - [x] `TL_SurfaceSelfIntersect`：参数域子片分割 + 非邻接子片 `TL_SurfaceSurfaceIntersect` + 原曲面 4D Gauss-Newton 精修，8 字形自交带状曲面实测交线误差 `1.07e-9`
  - [x] `TL_ExportBrepSTEP` / `TL_ImportBrepSTEP`（ISO 10303-21 AP214 `B_SPLINE_SURFACE_WITH_KNOTS`）与 `TL_ExportCurvesIGES` / `TL_ImportCurvesIGES`（ANSI Y14.26M Type 126 NURBS 曲线），Studio 新增 `_ExportStep` / `_ExportIges` 命令
  - [x] 3D Gumball 补齐 70% 臂长处的缩放方块手柄拖拽与 22px 旋转圆弧手柄拖拽，支持平移/旋转/缩放全自由度操纵与 `undo_stack` 撤销
  - [x] `opentl_tests` **21 组 3044 项断言 0 失败**，`ctest 5/5` 全过，构建并发布 `v0.3.0` 三平台（macOS `.dmg`/`.pkg`/`.tar.gz`、Linux `.tar.gz`/`.zip`、Windows `.exe` NSIS 安装包/`.zip`）原生发行版
- [x] **Stage 21：分视口独立 Wireframe/Shaded 渲染管线与点击切换、3D 着色网格屏幕射线拾取、`.ghz` 与内嵌 `ON_Data` 二进制几何无损序列化**
  - [x] `TL_VulkanRenderOptions::viewport_shaded_mask`：分视口独立控制 `WIREFRAME`（仅绘制特征/轮廓线，透出底格）与 `SHADED`（Blinn-Phong 着色三角面），支持鼠标单击视口标题旁胶囊徽章实时切换
  - [x] `TL_PickMeshAtScreen`：将视口屏幕坐标经 `ScreenToRay`（含正交视口射线后移修正）投射并调用 `TL_RaycastMesh`，直接拾取 3D 着色网格表面并重定位 Gumball 支点
  - [x] `TL_GhDocument::SaveGhz` 与 `EncodeOnData`：支持 `.ghz` PKZIP 容器保存及 `ON_NurbsCurve` / `ON_NurbsSurface` / `ON_Mesh` 内嵌 `ON_Data`（`ON_Write3dmBufferArchive`）无损往返，`opentl_tests` **21 组 3058 项断言 0 失败**，`ctest 5/5` 全过，`vision_analyze` 验证 Top 视口真线框与其余三视口着色完全一致
