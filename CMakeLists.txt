cmake_minimum_required(VERSION 3.16)
project(opentl VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Link against mcneel/opennurbs in ../external/opennurbs
set(OPENNURBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../external/opennurbs")
add_subdirectory("${OPENNURBS_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/opennurbs_build" EXCLUDE_FROM_ALL)

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
)

add_library(opentl STATIC ${OPENTL_SOURCES})
target_include_directories(opentl PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${OPENNURBS_DIR}"
)
target_link_libraries(opentl PUBLIC opennurbsStatic pthread dl)

add_library(opentl_shared SHARED ${OPENTL_SOURCES})
target_include_directories(opentl_shared PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${OPENNURBS_DIR}"
)
target_link_libraries(opentl_shared PUBLIC opennurbsStatic pthread dl)

enable_testing()
add_executable(opentl_tests tests/test_opentl.cpp)
target_link_libraries(opentl_tests PRIVATE opentl)
add_test(NAME opentl_tests COMMAND opentl_tests)

add_executable(opentl_cli apps/opentl_cli.cpp)
target_link_libraries(opentl_cli PRIVATE opentl)
add_test(NAME opentl_cli_demo COMMAND opentl_cli --demo --out-dir "${CMAKE_CURRENT_BINARY_DIR}/demo_out")
add_test(NAME opentl_cli_views COMMAND opentl_cli --views --out-dir "${CMAKE_CURRENT_BINARY_DIR}/views_out")
