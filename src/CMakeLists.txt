# Main application binary
file(GLOB UI_SRCS ui/*.cpp)
file(GLOB UI_HEADER ui/*.h)
add_executable(unwind main.cpp ${UI_SRCS} ${UI_HEADER} ".editorconfig")
set_property(TARGET unwind PROPERTY CXX_STANDARD 14)
set_property(TARGET unwind PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(unwind quartet contourtree utils vor3d spdlog
  igl::core igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui)
