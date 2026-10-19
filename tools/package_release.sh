#!/usr/bin/env bash
# Build Ohino release packages for every platform we can reach from this host.
#
#   macOS  : native build        -> .dmg / .pkg / .tar.gz
#   Linux  : zig cross-build     -> .tar.gz / .zip
#   Windows: MinGW-w64 cross     -> NSIS .exe installer + .zip
#
# Windows is a real native PE build (x86_64-w64-mingw32-g++), fully static CRT;
# it needs no MSVC and no Windows machine. See packaging/mingw_compat.h for the
# MinGW shims opennurbs needs.
#
# Usage: tools/package_release.sh [version]
set -euo pipefail

VERSION="${1:-0.3.0}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${ROOT}/dist"
WORK="${TMPDIR:-/tmp}/ohino-pkg"
mkdir -p "${OUT}" "${WORK}"
rm -rf "${WORK:?}"/*

echo "== Ohino ${VERSION} =="

# ---------------------------------------------------------------- macOS ------
if [[ "$(uname -s)" == "Darwin" ]]; then
  cmake -S "${ROOT}" -B "${WORK}/build-macos" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "${WORK}/build-macos" -j"$(sysctl -n hw.ncpu)" >/dev/null
  "${WORK}/build-macos/opentl_tests" | tail -2

  cmake --install "${WORK}/build-macos" --prefix "${WORK}/macos/Ohino-macos-$(uname -m)"
  tar -czf "${OUT}/Ohino-v${VERSION}-macos-$(uname -m).tar.gz" -C "${WORK}/macos" "Ohino-macos-$(uname -m)"
  pkgbuild --root "${WORK}/macos/Ohino-macos-$(uname -m)" \
           --identifier org.ohino.studio --version "${VERSION}" \
           --install-location /usr/local "${OUT}/Ohino-v${VERSION}-macos-$(uname -m).pkg"
  hdiutil create -volname "Ohino Studio" -srcfolder "${WORK}/macos/Ohino-macos-$(uname -m)" \
                 -ov -format UDZO "${OUT}/Ohino-v${VERSION}-macos-$(uname -m).dmg" >/dev/null
  echo "   macOS   -> .dmg .pkg .tar.gz"
fi

# ---------------------------------------------------------------- Linux ------
if [[ "$(uname -s)" == "Darwin" ]]; then
  ZIG_SHIMS="${WORK}/zig"
  mkdir -p "${ZIG_SHIMS}"
  cat > "${ZIG_SHIMS}/linux-cxx" <<'EOF'
#!/bin/sh
exec zig c++ -target x86_64-linux-gnu.2.31 -Wno-date-time -Wno-nontrivial-memcall "$@"
EOF
  cat > "${ZIG_SHIMS}/linux-cc" <<'EOF'
#!/bin/sh
exec zig cc -target x86_64-linux-gnu.2.31 -Wno-date-time -Wno-deprecated-non-prototype "$@"
EOF
  cat > "${ZIG_SHIMS}/ar" <<'EOF'
#!/bin/sh
exec zig ar "$@"
EOF
  cat > "${ZIG_SHIMS}/ranlib" <<'EOF'
#!/bin/sh
exec zig ranlib "$@"
EOF
  chmod +x "${ZIG_SHIMS}"/*
  cmake -S "${ROOT}" -B "${WORK}/build-linux" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=x86_64 \
    -DCMAKE_C_COMPILER="${ZIG_SHIMS}/linux-cc" -DCMAKE_CXX_COMPILER="${ZIG_SHIMS}/linux-cxx" \
    -DCMAKE_AR="${ZIG_SHIMS}/ar" -DCMAKE_RANLIB="${ZIG_SHIMS}/ranlib" \
    -DCMAKE_BUILD_TYPE=Release -DOPENTL_ENABLE_SDL3=OFF \
    -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY \
    -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY >/dev/null
  cmake --build "${WORK}/build-linux" -j8 >/dev/null
  cmake --install "${WORK}/build-linux" --prefix "${WORK}/linux/Ohino-linux-x86_64"
  strip -x "${WORK}/linux/Ohino-linux-x86_64/bin/opentl_cli" || true
  strip -x "${WORK}/linux/Ohino-linux-x86_64/lib/libopentl_shared.so" || true
  tar -czf "${OUT}/Ohino-v${VERSION}-linux-x86_64.tar.gz" -C "${WORK}/linux" Ohino-linux-x86_64
  (cd "${WORK}/linux" && zip -qr "${OUT}/Ohino-v${VERSION}-linux-x86_64.zip" Ohino-linux-x86_64)
  echo "   Linux   -> .tar.gz .zip"
fi

# -------------------------------------------------------------- Windows ------
if command -v x86_64-w64-mingw32-g++ >/dev/null && command -v makensis >/dev/null; then
  cat > "${WORK}/mingw_toolchain.cmake" <<'EOF'
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER x86_64-w64-mingw32-windres)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
EOF
  # SDL3 + the Vulkan loader are cross-built once by tools/build_windows_deps.sh;
  # without them the package still builds, just without the interactive window.
  WIN_DEPS="${OHINO_WIN_DEPS:-${TMPDIR:-/tmp}/ohino-win-deps}"
  WIN_EXTRA=(-DOPENTL_ENABLE_SDL3=OFF)
  if [[ -f "${WIN_DEPS}/SDL3.dll" ]]; then
    WIN_EXTRA=(-DSDL3_DIR="${WIN_DEPS}/SDL3-build"
               -DVulkan_INCLUDE_DIR="${WIN_DEPS}/include"
               -DVulkan_LIBRARY="${WIN_DEPS}/libvulkan-1.dll.a")
  fi
  cmake -S "${ROOT}" -B "${WORK}/build-win" \
    -DCMAKE_TOOLCHAIN_FILE="${WORK}/mingw_toolchain.cmake" \
    -DCMAKE_BUILD_TYPE=Release "${WIN_EXTRA[@]}" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 >/dev/null
  cmake --build "${WORK}/build-win" -j8 >/dev/null
  x86_64-w64-mingw32-strip "${WORK}/build-win/opentl_cli.exe" || true
  x86_64-w64-mingw32-strip "${WORK}/build-win/opentl_tests.exe" || true

  cmake --install "${WORK}/build-win" --prefix "${WORK}/win/Ohino-windows-x86_64"
  rm -f "${WORK}/win/Ohino-windows-x86_64/lib/libopentl.a"
  for dll in SDL3.dll vulkan-1.dll; do
    [[ -f "${WIN_DEPS}/${dll}" ]] && cp "${WIN_DEPS}/${dll}" "${WORK}/win/Ohino-windows-x86_64/bin/"
  done
  (cd "${WORK}/win" && zip -qr "${OUT}/Ohino-v${VERSION}-windows-x86_64.zip" Ohino-windows-x86_64)

  makensis -DVERSION="${VERSION}" \
           -DSTAGE_DIR="${WORK}/win/Ohino-windows-x86_64" \
           -DOUT_FILE="${OUT}/Ohino-v${VERSION}-windows-x86_64.exe" \
           "${ROOT}/packaging/windows/ohino.nsi" >/dev/null
  echo "   Windows -> .exe (NSIS installer) .zip"
fi

ls -lh "${OUT}" | tail -n +2
