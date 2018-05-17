cmake_minimum_required(VERSION 3.1)
project(fish_deformation)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
if (WIN32)
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-DCHOLMOD")
  include_directories("C:/ab7512/SuiteSparse/include")
  include_directories("C:/ab7512/OpenBLAS-v0.2.19-Win64-int32/include")
else ()
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-DNDEBUG -O3 -g -DCHOLMOD")
  include_directories(/usr/include/suitesparse) # Ubuntu puts cholmod.h here
endif ()
option(BUILD_TESTS "Build test binaries" OFF)



# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" ON)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           ${BUILD_TESTS})
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         OFF)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_NANOGUI           "Use Nanogui menu"   OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_PNG               "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON            "Use Python"         ON)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ${BUILD_TESTS})
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

find_package(LIBIGL REQUIRED QUIET)

set(INCLUDE_DIRS src src/utils src/tests)

file(GLOB UTILS_SRCS src/utils/*.cpp)
list(FILTER UTILS_SRCS EXCLUDE REGEX cgal*)

message(STATUS "UTILS SRC FILES: ${UTILS_SRCS}")



add_executable(extract_surface_bin src/extract_surface.cpp ${UTILS_SRCS})
target_include_directories(extract_surface_bin PRIVATE ${INCLUDE_DIRS})
target_link_libraries(extract_surface_bin igl::core)

add_executable(preprocess_fish_bin src/preprocess_fish.cpp src/deformation_constraints.cpp ${UTILS_SRCS})
target_include_directories(preprocess_fish_bin PRIVATE ${INCLUDE_DIRS})
target_link_libraries(preprocess_fish_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui)
if (WIN32)
  target_link_libraries(preprocess_fish_bin
    "C:/ab7512/OpenBLAS-v0.2.19-Win64-int32/lib/libopenblas.dll.a"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libcholmodd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/suitesparseconfigd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/metisd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libamdd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libbtfd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libcamdd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libccolamdd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libcolamdd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libcxsparsed.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libklud.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libldld.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libspqrd.lib"
    debug "C:/ab7512/SuiteSparse/lib/Debug/x64/libumfpackd.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libcholmod.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/suitesparseconfig.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/metis.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libamd.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libbtf.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libcamd.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libccolamd.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libcolamd.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libcxsparse.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libklu.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libldl.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libspqr.lib"
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libumfpack.lib"
  )
else ()
  target_link_libraries(preprocess_fish_bin cholmod)
endif ()


add_executable(volume_test src/volume_main.cpp)
target_include_directories(volume_test PRIVATE ${INCLUDE_DIRS})
target_link_libraries(volume_test igl::core igl::opengl_glfw)

IF (BUILD_TESTS)
  # Tet Mesh Viewer
  add_executable(view_tet_bin src/tests/view_tet_file.cpp ${UTILS_SRCS})
  target_include_directories(view_tet_bin PRIVATE ${INCLUDE_DIRS})
  target_link_libraries(view_tet_bin igl::core igl::opengl_glfw)

  # Test and debug the rasterizer
  add_executable(debug_tex_bin src/test_rasterizer.cpp ${UTILS_SRCS})
  target_include_directories(debug_tex_bin PRIVATE ${INCLUDE_DIRS})
  target_link_libraries(debug_tex_bin igl::core igl::opengl_glfw)

  # Test loading Harish's dat files
  add_executable(test_meshing_harish_data_bin src/tests/test_meshing_harish_data.cpp src/utils/yixin_loader.cpp)
  target_include_directories(test_meshing_harish_data_bin PRIVATE ${INCLUDE_DIRS})
  target_link_libraries(test_meshing_harish_data_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui)

  # Test the basic cases for marching tets
  add_executable(test_marching_tets_cases_bin src/tests/test_marching_tets_basic.cpp)
  target_include_directories(test_marching_tets_cases_bin PRIVATE ${INCLUDE_DIRS})
  target_link_libraries(test_marching_tets_cases_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui)

  # Test marching tets on a mesh
  add_executable(test_cgal_tet_meshing_bin src/tests/test_cgal_tet_meshing.cpp)
  target_include_directories(test_cgal_tet_meshing_bin PRIVATE ${INCLUDE_DIRS})
  target_link_libraries(test_cgal_tet_meshing_bin igl::cgal)

  # Example for calling the CGAL tet-mesher
  add_executable(test_marching_tets_bin src/tests/test_marching_tets.cpp src/utils/cgal_tetrahedralize.cpp)
  target_include_directories(test_marching_tets_bin PRIVATE ${INCLUDE_DIRS})
  target_link_libraries(test_marching_tets_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui igl::cgal)
ENDIF()
