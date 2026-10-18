#pragma once
/**
 * @file tl_vulkan_renderer.h
 * @brief Cross-platform Vulkan 1.2 headless/offscreen renderer for OpenTL's 4-view layout.
 *
 * Uses standard Khronos Vulkan API (`VK_KHR_portability_enumeration` + `VK_KHR_portability_subset`
 * when running on MoltenVK on macOS, or native Vulkan drivers on Windows/Linux).
 * Renders the 2x2 Four-View layout (Top / Perspective / Front / Right) with CPlane grids,
 * coordinate axes, viewport labels, borders, and all SceneDocument geometry into an offscreen
 * RGBA8 image buffer and writes lossless PNG / PPM artifacts.
 */

#include "../tl_export.h"
#include "tl_viewport.h"
#include "../mvp/rhino_mvp.h"

#include <cstdint>
#include <string>
#include <vector>

struct TL_VulkanRenderResult
{
  bool success = false;
  std::string backend_name;      ///< e.g. "Vulkan 1.2 (Apple M4 / MoltenVK)"
  std::string device_name;       ///< Physical device name reported by VkPhysicalDeviceProperties
  uint32_t api_version = 0;      ///< Vulkan API version word
  int width = 0;
  int height = 0;
  size_t vertex_count = 0;       ///< Total line vertices submitted to the Vulkan pipeline
  std::vector<uint8_t> rgba;     ///< Row-major RGBA8 pixel buffer (width * height * 4 bytes)
  std::string error;
};

class TL_CLASS TL_VulkanFourViewRenderer
{
public:
  TL_VulkanFourViewRenderer() = default;
  ~TL_VulkanFourViewRenderer() = default;

  /**
   * Render `scene` through `layout` using a hardware/portable Vulkan graphics pipeline
   * into an offscreen RGBA8 target and read the pixels back to host memory.
   */
  bool RenderScene(
    const opentl::mvp::SceneDocument& scene,
    const TL_FourViewLayout& layout,
    TL_VulkanRenderResult& out_result
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
