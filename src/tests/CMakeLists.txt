add_executable(volume_test test_volume_rendering.cpp)
target_include_directories(volume_test PRIVATE ${PROJECT_SOURCE_DIR/src})
target_link_libraries(volume_test igl::core igl::opengl igl::opengl_glfw)

# Test Harish's segmentation code
add_executable(segmentation test_segmentation.cpp)
target_compile_options(segmentation PRIVATE "-fopenmp")
target_link_libraries(segmentation contourtree utils igl::core igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui Qt5::Core Qt5::Widgets -fopenmp)

# Tet Mesh Viewer
add_executable(view_tet_bin view_tet_file.cpp)
target_link_libraries(view_tet_bin utils igl::core igl::opengl_glfw)

# Test and debug the rasterizer
add_executable(debug_tex_bin test_rasterizer.cpp)
target_link_libraries(debug_tex_bin utils igl::core igl::opengl igl::opengl_glfw)

# Test loading Harish's dat files
add_executable(test_meshing_harish_data_bin test_meshing_harish_data.cpp)
target_link_libraries(test_meshing_harish_data_bin utils igl::core igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui)

# Test the basic cases for marching tets
add_executable(test_marching_tets_cases_bin test_marching_tets_basic.cpp)
target_link_libraries(test_marching_tets_cases_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui)

# Test marching tets on a mesh
if (TARGET igl::cgal)
  add_executable(test_cgal_tet_meshing_bin test_cgal_tet_meshing.cpp)
  target_link_libraries(test_cgal_tet_meshing_bin igl::cgal)

  # Example for calling the CGAL tet-mesher
  add_executable(test_marching_tets_bin test_marching_tets.cpp ${SRC_DIR}/utils/cgal_tetrahedralize.cpp)
  target_link_libraries(test_marching_tets_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui igl::cgal)
endif()
