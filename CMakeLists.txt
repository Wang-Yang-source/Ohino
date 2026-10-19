cmake_minimum_required(VERSION 3.16)
project(opentl VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Link against mcneel/opennurbs in ../external/opennurbs
set(OPENNURBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../external/opennurbs")

if(MINGW)
  # MinGW-w64 is neither MSVC nor POSIX: force UNICODE TCHARs, supply the
  # missing <dirent.h> bits, and pull in the Win32 headers opennurbs expects.
  add_compile_definitions(UNICODE _UNICODE _WIN32_WINNT=0x0601)
endif()

add_subdirectory("${OPENNURBS_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/opennurbs_build" EXCLUDE_FROM_ALL)

if(MINGW)
  # Only the vendored library needs the POSIX/CRT shims; keeping the shim out of
  # Ohino's own translation units avoids pulling Win32 GDI names (Arc, Polyline)
  # into scope where they collide with ON_Arc / ON_Polyline.
  set(OHINO_MINGW_SHIM "${CMAKE_CURRENT_SOURCE_DIR}/packaging/mingw_compat.h")
  foreach(tgt opennurbsStatic zlib)
    if(TARGET ${tgt})
      target_compile_options(${tgt} PRIVATE -include "${OHINO_MINGW_SHIM}")
    endif()
  endforeach()
endif()

set(OPENTL_SOURCES
    src/tl_promote.cpp
    src/intersect/tl_intersect.cpp
    src/meshing/tl_meshing.cpp
    src/booleans/tl_booleans.cpp
    src/fitting_loft_sweep/tl_fitting_loft_sweep.cpp
    src/subd/tl_subd.cpp
    src/closest_point/tl_closest_point.cpp
    src/c_api/rhcommon_c_bridge.cpp
    src/mvp/geometry_scene.cpp
    src/mvp/intersect_sample.cpp
    src/mvp/scene_io.cpp
    src/viewport/tl_viewport.cpp
    src/viewport/tl_vulkan_renderer.cpp
    src/gh/tl_gh.cpp
    src/gui/tl_gui.cpp
)

# Optional Vulkan backend for the four-view renderer (MoltenVK on macOS, native on Win/Linux).
option(OPENTL_ENABLE_SDL3 "Enable SDL3 interactive studio window" ON)
find_package(Vulkan QUIET)
if(OPENTL_ENABLE_SDL3)
  find_package(SDL3 QUIET)
  if(NOT SDL3_FOUND)
    find_path(SDL3_INCLUDE_DIR SDL3/SDL.h HINTS /opt/homebrew/include /usr/local/include)
    find_library(SDL3_LIBRARY NAMES SDL3 HINTS /opt/homebrew/lib /usr/local/lib)
  endif()
endif()

if(MINGW)
  # Ship self-contained .exe files: no libstdc++/libgcc/libwinpthread DLLs.
  add_link_options(-static -static-libgcc -static-libstdc++)
  set(OPENTL_PLATFORM_LIBS shlwapi usp10 dwrite d2d1 ole32 oleaut32 uuid rpcrt4 gdi32 user32 advapi32 shell32 windowscodecs dxgi d3d11)
elseif(UNIX AND NOT APPLE)
  set(OPENTL_PLATFORM_LIBS pthread dl)
else()
  set(OPENTL_PLATFORM_LIBS "")
endif()

add_library(opentl STATIC ${OPENTL_SOURCES})
target_include_directories(opentl PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${OPENNURBS_DIR}"
)
if(MINGW)
  # opennurbs' bundled zlib and opennurbsStatic reference each other (zcalloc),
  # so the linker has to rescan the pair on Windows.
  target_link_libraries(opentl PUBLIC "$<LINK_GROUP:RESCAN,opennurbsStatic,zlib>" ${OPENTL_PLATFORM_LIBS})
else()
  target_link_libraries(opentl PUBLIC opennurbsStatic ${OPENTL_PLATFORM_LIBS})
endif()
if(Vulkan_FOUND)
  target_compile_definitions(opentl PRIVATE OPENTL_HAS_VULKAN=1)
  target_include_directories(opentl PRIVATE ${Vulkan_INCLUDE_DIRS})
  target_link_libraries(opentl PUBLIC ${Vulkan_LIBRARIES})
  message(STATUS "OpenTL: Vulkan found -> four-view renderer enabled (${Vulkan_LIBRARIES})")
else()
  message(STATUS "OpenTL: Vulkan not found -> four-view renderer built without a backend")
endif()
if(SDL3_FOUND)
  target_compile_definitions(opentl PUBLIC OPENTL_HAS_SDL3=1)
  target_link_libraries(opentl PUBLIC SDL3::SDL3)
  message(STATUS "OpenTL: SDL3 found via CMake config -> interactive studio window enabled")
elseif(SDL3_INCLUDE_DIR AND SDL3_LIBRARY)
  target_compile_definitions(opentl PUBLIC OPENTL_HAS_SDL3=1)
  target_include_directories(opentl PUBLIC "${SDL3_INCLUDE_DIR}")
  target_link_libraries(opentl PUBLIC "${SDL3_LIBRARY}")
  message(STATUS "OpenTL: SDL3 found (${SDL3_LIBRARY}) -> interactive studio window enabled")
endif()

add_library(opentl_shared SHARED ${OPENTL_SOURCES})
target_include_directories(opentl_shared PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${OPENNURBS_DIR}"
)
if(MINGW)
  target_link_libraries(opentl_shared PUBLIC "$<LINK_GROUP:RESCAN,opennurbsStatic,zlib>" ${OPENTL_PLATFORM_LIBS})
else()
  target_link_libraries(opentl_shared PUBLIC opennurbsStatic ${OPENTL_PLATFORM_LIBS})
endif()
if(Vulkan_FOUND)
  target_compile_definitions(opentl_shared PRIVATE OPENTL_HAS_VULKAN=1)
  target_include_directories(opentl_shared PRIVATE ${Vulkan_INCLUDE_DIRS})
  target_link_libraries(opentl_shared PUBLIC ${Vulkan_LIBRARIES})
endif()
if(SDL3_FOUND)
  target_compile_definitions(opentl_shared PUBLIC OPENTL_HAS_SDL3=1)
  target_link_libraries(opentl_shared PUBLIC SDL3::SDL3)
elseif(SDL3_INCLUDE_DIR AND SDL3_LIBRARY)
  target_compile_definitions(opentl_shared PUBLIC OPENTL_HAS_SDL3=1)
  target_include_directories(opentl_shared PUBLIC "${SDL3_INCLUDE_DIR}")
  target_link_libraries(opentl_shared PUBLIC "${SDL3_LIBRARY}")
endif()

enable_testing()
add_executable(opentl_tests tests/test_opentl.cpp)
target_link_libraries(opentl_tests PRIVATE opentl)
add_test(NAME opentl_tests COMMAND opentl_tests)

add_executable(opentl_cli apps/opentl_cli.cpp)
target_link_libraries(opentl_cli PRIVATE opentl)
add_test(NAME opentl_cli_demo COMMAND opentl_cli --demo --out-dir "${CMAKE_CURRENT_BINARY_DIR}/demo_out")
add_test(NAME opentl_cli_views COMMAND opentl_cli --views --out-dir "${CMAKE_CURRENT_BINARY_DIR}/views_out")
add_test(NAME opentl_cli_vulkan COMMAND opentl_cli --render-vulkan --out-dir "${CMAKE_CURRENT_BINARY_DIR}/vulkan_out")
add_test(NAME opentl_cli_gui COMMAND opentl_cli --render-gui --out-dir "${CMAKE_CURRENT_BINARY_DIR}/gui_out")

# ----------------------------------------------------------------------------
# Install & Cross-Platform CPack Packaging (Windows / macOS / Linux)
# ----------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS opentl_cli opentl opentl_shared
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/opentl DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DOCDIR})

set(CPACK_PACKAGE_NAME "Ohino")
set(CPACK_PACKAGE_VENDOR "Ohino Project")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Ohino — A visual programming and parametric modeling tool and design software")
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 3)
set(CPACK_PACKAGE_VERSION_PATCH 0)
set(CPACK_PACKAGE_CONTACT "wayamac@local")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ohino Project <wayamac@local>")
set(CPACK_DEBIAN_PACKAGE_SECTION "graphics")
include(CPack)
