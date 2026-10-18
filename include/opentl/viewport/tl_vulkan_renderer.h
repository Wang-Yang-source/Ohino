#pragma once
/**
 * @file tl_vulkan_renderer.h
 * @brief Cross-platform Vulkan 1.2 headless/offscreen renderer for OpenTL's 4-view layout.
 *
 * Uses standard Khronos Vulkan API (`VK_KHR_portability_enumeration` + `VK_KHR_portability_subset`
 * when running on MoltenVK on macOS, or native Vulkan drivers on Windows/Linux).
 *
 * Stage 5 features:
 *   - Catppuccin Mocha color theme (`TL_CatppuccinMochaTheme`)
 *   - 4x MSAA (`VK_SAMPLE_COUNT_4_BIT`) hardware multisample anti-aliasing + resolve attachment
 *   - 32-bit floating-point depth buffer (`VK_FORMAT_D32_SFLOAT`) with depth bias for wireframe-over-shaded
 *   - Dual pipeline: filled triangle shaded surfaces (`VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST` with
 *     hemisphere + directional Lambertian + Blinn-Phong lighting) + anti-aliased wireframe/grid/HUD
 *     lines (`VK_PRIMITIVE_TOPOLOGY_LINE_LIST`).
 */

#include "../tl_export.h"
#include "tl_viewport.h"
#include "../mvp/rhino_mvp.h"

#include <cstdint>
#include <string>
#include <vector>

/**
 * Catppuccin Mocha palette (normalized linear/sRGB floats).
 * Reference: https://catppuccin.com/palette (Mocha flavor)
 */
struct TL_CatppuccinMochaTheme
{
  static constexpr float kCrust[4]    = { 0.067f, 0.067f, 0.106f, 1.0f }; // #11111b
  static constexpr float kMantle[4]   = { 0.094f, 0.094f, 0.145f, 1.0f }; // #181825
  static constexpr float kBase[4]     = { 0.118f, 0.118f, 0.180f, 1.0f }; // #1e1e2e
  static constexpr float kSurface0[4] = { 0.192f, 0.196f, 0.267f, 1.0f }; // #313244
  static constexpr float kSurface1[4] = { 0.271f, 0.278f, 0.353f, 1.0f }; // #45475a
  static constexpr float kSurface2[4] = { 0.345f, 0.357f, 0.439f, 1.0f }; // #585b70
  static constexpr float kOverlay0[4] = { 0.424f, 0.439f, 0.525f, 1.0f }; // #6c7086
  static constexpr float kSubtext0[4] = { 0.651f, 0.678f, 0.784f, 1.0f }; // #a6adc8
  static constexpr float kText[4]     = { 0.804f, 0.839f, 0.957f, 1.0f }; // #cdd6f4

  // Accents
  static constexpr float kRed[4]      = { 0.953f, 0.545f, 0.659f, 1.0f }; // #f38ba8 (X axis)
  static constexpr float kGreen[4]    = { 0.651f, 0.890f, 0.631f, 1.0f }; // #a6e3a1 (Y axis)
  static constexpr float kBlue[4]     = { 0.537f, 0.706f, 0.980f, 1.0f }; // #89b4fa (Z axis)
  static constexpr float kYellow[4]   = { 0.976f, 0.886f, 0.686f, 1.0f }; // #f9e2af (Origin / highlights)
  static constexpr float kSapphire[4] = { 0.455f, 0.780f, 0.925f, 1.0f }; // #74c7ec (Primary curves)
  static constexpr float kMauve[4]    = { 0.796f, 0.651f, 0.969f, 1.0f }; // #cba6f7 (Shaded surfaces / SubD)
  static constexpr float kPeach[4]    = { 0.980f, 0.702f, 0.529f, 1.0f }; // #fab387 (Secondary curves / Loft)
  static constexpr float kTeal[4]     = { 0.580f, 0.886f, 0.835f, 1.0f }; // #94e2d5 (Sweep / Iso-curves)
  static constexpr float kLavender[4] = { 0.706f, 0.745f, 0.996f, 1.0f }; // #b4befe (Active border / HUD)
};

struct TL_VulkanRenderOptions
{
  int sample_count = 4;            ///< MSAA samples (4 = 4x MSAA, 1 = off)
  bool enable_depth = true;        ///< 32-bit float depth buffer (`D32_SFLOAT`)
  bool enable_shaded_meshes = true;///< Render shaded triangle meshes alongside wireframes
  bool cull_backfaces = true;      ///< CULL_MODE_BACK for closed meshes (see TL_CapMeshOpenBoundaryLoops)
  bool enable_feature_edges = true;///< Draw mesh silhouette + hard crease edges over the shading
  double crease_angle_degrees = 30.0; ///< Dihedral angle above which an edge counts as a crease
  float feature_line_width = 2.4f; ///< Screen-space width (px) for silhouette & crease lines
  float curve_line_width = 2.0f;   ///< Screen-space width (px) for scene curves & SSX traces
};

struct TL_VulkanRenderResult
{
  bool success = false;
  std::string backend_name;      ///< e.g. "Vulkan 1.2.357"
  std::string device_name;       ///< Physical device name reported by VkPhysicalDeviceProperties
  std::string theme_name = "Catppuccin Mocha";
  uint32_t api_version = 0;      ///< Vulkan API version word
  int width = 0;
  int height = 0;
  int msaa_samples = 1;          ///< Actual MSAA sample count used by the render pass
  bool depth_enabled = false;    ///< True when D32_SFLOAT depth attachment was active
  size_t vertex_count = 0;       ///< Total line segments submitted
  size_t triangle_count = 0;     ///< Total shaded triangles submitted
  size_t feature_edge_count = 0; ///< Silhouette + crease segments drawn from the meshes
  std::vector<uint8_t> rgba;     ///< Row-major RGBA8 pixel buffer (width * height * 4 bytes)
  std::string error;
};

class TL_CLASS TL_VulkanFourViewRenderer
{
public:
  TL_VulkanFourViewRenderer() = default;
  ~TL_VulkanFourViewRenderer() = default;

  /**
   * Render `scene` (plus any explicit `shaded_meshes`) through `layout` using a
   * Catppuccin Mocha themed Vulkan graphics pipeline with 4x MSAA + D32_SFLOAT depth.
   */
  bool RenderScene(
    const opentl::mvp::SceneDocument& scene,
    const TL_FourViewLayout& layout,
    TL_VulkanRenderResult& out_result,
    const std::vector<const ON_Mesh*>& extra_meshes = {},
    const TL_VulkanRenderOptions& options = TL_VulkanRenderOptions()
  ) const;

  /**
   * Save an RGBA8 buffer to a standard lossless PNG file (uses zlib compression).
   */
  static bool SavePng(
    const std::string& path,
    int width,
    int height,
    const std::vector<uint8_t>& rgba,
    std::string* error_out = nullptr
  );
};
