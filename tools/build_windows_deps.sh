#!/usr/bin/env bash
# Build the Windows-only third-party runtime (SDL3 + the Khronos Vulkan loader)
# with MinGW-w64, so the Windows package ships the interactive studio window and
# the Vulkan renderer without needing MSVC or a Windows SDK.
#
#   tools/build_windows_deps.sh [prefix]
#
# Output: <prefix>/SDL3.dll, <prefix>/vulkan-1.dll, plus the CMake configs the
# Ohino Windows build needs (SDL3_DIR, Vulkan_INCLUDE_DIR, Vulkan_LIBRARY).
set -euo pipefail

PREFIX="${1:-${TMPDIR:-/tmp}/ohino-win-deps}"
WORK="${TMPDIR:-/tmp}/ohino-win-deps-src"
mkdir -p "${PREFIX}" "${WORK}"

TOOLCHAIN="${WORK}/mingw_toolchain.cmake"
cat > "${TOOLCHAIN}" <<'EOF'
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER x86_64-w64-mingw32-windres)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
EOF

# ------------------------------------------------------------------ SDL3 -----
if [[ ! -f "${PREFIX}/SDL3.dll" ]]; then
  [[ -d "${WORK}/SDL" ]] || git clone --depth 1 https://github.com/libsdl-org/SDL.git "${WORK}/SDL"
  cmake -S "${WORK}/SDL" -B "${WORK}/SDL-build" -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
    -DCMAKE_BUILD_TYPE=Release -DSDL_SHARED=ON -DSDL_STATIC=OFF \
    -DSDL_TESTS=OFF -DSDL_EXAMPLES=OFF -DSDL_TEST_LIBRARY=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 > /dev/null
  cmake --build "${WORK}/SDL-build" -j"$(nproc 2>/dev/null || echo 8)" > /dev/null
  cp "${WORK}/SDL-build/SDL3.dll" "${PREFIX}/SDL3.dll"
  # Keep the generated config next to the DLL for -DSDL3_DIR.
  cp -R "${WORK}/SDL-build" "${PREFIX}/SDL3-build"
fi

# --------------------------------------------------------- Vulkan headers ----
if [[ ! -f "${PREFIX}/include/vulkan/vulkan.h" ]]; then
  [[ -d "${WORK}/Vulkan-Headers" ]] || git clone --depth 1 https://github.com/KhronosGroup/Vulkan-Headers.git "${WORK}/Vulkan-Headers"
  cmake -S "${WORK}/Vulkan-Headers" -B "${WORK}/vkh-build" \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 > /dev/null
  cmake --install "${WORK}/vkh-build" > /dev/null
fi

# ---------------------------------------------------------- Vulkan loader ----
if [[ ! -f "${PREFIX}/vulkan-1.dll" ]]; then
  [[ -d "${WORK}/Vulkan-Loader" ]] || git clone --depth 1 https://github.com/KhronosGroup/Vulkan-Loader.git "${WORK}/Vulkan-Loader"
  cmake -S "${WORK}/Vulkan-Loader" -B "${WORK}/vkl-build" -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
    -DCMAKE_BUILD_TYPE=Release -DVulkanHeaders_DIR="${PREFIX}/share/cmake/VulkanHeaders" \
    -DBUILD_TESTS=OFF -DUPDATE_DEPS=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 > /dev/null
  cmake --build "${WORK}/vkl-build" -j"$(nproc 2>/dev/null || echo 8)" > /dev/null
  cp "${WORK}/vkl-build/loader/vulkan-1.dll" "${PREFIX}/vulkan-1.dll"
  cp "${WORK}/vkl-build/loader/libvulkan-1.dll.a" "${PREFIX}/libvulkan-1.dll.a"
fi

echo "Windows deps ready in ${PREFIX}"
echo "  -DSDL3_DIR=${PREFIX}/SDL3-build"
echo "  -DVulkan_INCLUDE_DIR=${PREFIX}/include -DVulkan_LIBRARY=${PREFIX}/libvulkan-1.dll.a"
