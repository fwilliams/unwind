
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-DNDEBUG -g")

# Main application binary
file(GLOB UI_SRCS ui/*.cpp)
add_executable(fish_deformation main.cpp ${UI_SRCS})
set_target_properties(fish_deformation PROPERTIES CXX_STANDARD 14)
set_target_properties(fish_deformation PROPERTIES CXX_STANDARD_REQUIRED ON)
target_link_libraries(fish_deformation quartet contourtree utils vor3d spdlog
  igl::core igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui igl::cgal igl::triangle)


# Binary to extract a surface mesh from a volume datfile (Deprecated, remove soon)
add_executable(extract_surface_bin extract_surface.cpp)
target_link_libraries(extract_surface_bin igl::core igl::opengl_glfw igl::opengl_glfw_imgui utils)


# Fish straightening prototype (Deprecated, remove soon)
add_executable(preprocess_fish_bin preprocess_fish.cpp deformation_constraints.cpp)
target_link_libraries(preprocess_fish_bin igl::core igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui utils)
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
    optimized "C:/ab7512/SuiteSparse/lib/Release/x64/libumfpack.lib")
else ()
  target_link_libraries(preprocess_fish_bin cholmod)
endif ()

