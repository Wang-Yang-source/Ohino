# Ohino release packaging

`tools/package_release.sh` builds every package from a single host:

| Platform | Toolchain | Output |
| --- | --- | --- |
| macOS (native) | Apple clang + MoltenVK | `.dmg`, `.pkg`, `.tar.gz` |
| Linux x86_64 | `zig c++ -target x86_64-linux-gnu.2.31` | `.tar.gz`, `.zip` |
| Windows x86_64 | `x86_64-w64-mingw32-g++` (MinGW-w64) + `makensis` | `.exe` (NSIS installer), `.zip` |

## Windows without MSVC

The Windows build is a real native PE32+ x86-64 binary; it does not need a
Windows machine, Visual Studio, or the Windows SDK:

```sh
brew install mingw-w64 makensis
cmake -S . -B build-win -DCMAKE_TOOLCHAIN_FILE=<mingw_toolchain.cmake> \
      -DCMAKE_BUILD_TYPE=Release -DOPENTL_ENABLE_SDL3=OFF
cmake --build build-win -j8
```

`packaging/mingw_compat.h` is force-included into the vendored opennurbs/zlib
translation units and supplies what MinGW-w64 lacks compared to MSVC and POSIX:

* `WIN32_LEAN_AND_MEAN` + `<shlobj.h>`/`<knownfolders.h>` for
  `SHGetKnownFolderPath` (used by `ON_FileSystemPath::PlatformPath`).
* `readdir_r` and `NAME_MAX` on top of MinGW's `<dirent.h>`.
* POSIX-spelled `sprintf_l`/`snprintf_l`/`vsnprintf_l`/`sscanf_l` (MSVCRT only
  has the locale-last `_sprintf_l` family).
* BSD-order `qsort_r` (opennurbs calls `qsort_r(base, nel, width, ctx, cmp)`).
* `localtime_r`/`gmtime_r` wrappers over `localtime_s`/`gmtime_s`.

Three small upstream-side patches are required as well:

* `opennurbs_system.h` — MinGW's `<_mingw.h>` defines `WIN32` even for x64;
  `#undef` it instead of `#error`.
* `opennurbs_internal_defines.h` — `PRIVATE_CHECK` is an MSVC-only debug size
  assertion (it relies on `_PRIVATE` being in scope); disable it on MinGW.
* `opennurbs_uuid.cpp` — route `ON_CreateUuid` to `UuidCreate` for MinGW too.

The produced `.exe` links the C/C++ runtime statically (`-static
-static-libgcc -static-libstdc++`), so the only imports are Windows system
DLLs (`KERNEL32`, `USER32`, `GDI32`, `SHELL32`, `ole32`, `RPCRT4`, `DWrite`,
`ADVAPI32` and the UCRT api-set).

## GitHub Actions

`packaging/github-workflows-release.yml` is a ready-to-use CI matrix that builds
all three platforms on native runners (including a real MSVC Windows build and
an NSIS installer). Copy it to `.github/workflows/release.yml` — pushing that
path requires a token with the `workflow` scope.
