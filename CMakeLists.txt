cmake_minimum_required(VERSION 3.1)
project(fish_deformation)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-DNDEBUG -O3 -g -DCHOLMOD") # O3 with asserts and debug symbols
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include_directories(/usr/include/suitesparse) # Ubuntu puts cholmod.h here

# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" ON)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           ON)
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
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ON)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

find_package(LIBIGL REQUIRED QUIET)

set(INCLUDE_DIRS src src/utils src/tests)

add_executable(${PROJECT_NAME}_bin src/main.cpp src/utils/yixin_loader.cpp)
target_link_libraries(${PROJECT_NAME}_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui cholmod)
target_include_directories(${PROJECT_NAME}_bin PRIVATE ${INCLUDE_DIRS})

add_executable(preprocess_fish_bin src/preprocess_fish.cpp src/utils/yixin_loader.cpp)
target_include_directories(preprocess_fish_bin PRIVATE ${INCLUDE_DIRS})
target_link_libraries(preprocess_fish_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui cholmod)

add_executable(extract_surface_bin src/extract_surface.cpp)
target_include_directories(extract_surface_bin PRIVATE ${INCLUDE_DIRS})
target_link_libraries(extract_surface_bin igl::core)

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


