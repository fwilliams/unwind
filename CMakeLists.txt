cmake_minimum_required(VERSION 3.8)
project(fish_deformation)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/voroffset/cmake)




#################################
## Download external libraries ##
#################################

if (NOT DEFINED ${DL_EXTERNAL_PROJ_DIRECTORY})
  set(DL_EXTERNAL_PROJ_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external)
endif()
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(DownloadExternalProject)

download_external_project(glm
  GIT_REPOSITORY https://github.com/fwilliams/glm.git
  GIT_TAG        master
)
download_external_project(spdlog
  GIT_REPOSITORY https://github.com/fwilliams/spdlog.git
  GIT_TAG        v1.x
)
download_external_project(quartet
  GIT_REPOSITORY https://github.com/fwilliams/quartet.git
  GIT_TAG        master
)
download_external_project(Segmentangling
  GIT_REPOSITORY https://github.com/ViDA-NYU/Segmentangling.git
  GIT_TAG        fish_deformation
)
download_external_project(libigl
  GIT_REPOSITORY https://github.com/fwilliams/libigl.git
  GIT_TAG        fish_deformation
)
download_external_project(cgal
  GIT_REPOSITORY https://github.com/fwilliams/cgal.git
  GIT_TAG        master
)




##############################
## Build external libraries ##
##############################

# Find Suitesparse
if (WIN32)
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCHOLMOD")
  include_directories("C:/ab7512/SuiteSparse/include")
  include_directories("C:/ab7512/SuiteSparse/CHOLMOD/include/include")
  include_directories("C:/ab7512/OpenBLAS-v0.2.19-Win64-int32/include")
else ()
  include_directories(/usr/include/suitesparse) # Ubuntu puts cholmod.h here
endif ()


# LibIGL
set(CGAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/cgal)
option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" OFF)
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
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ${BUILD_TESTS})
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)
find_package(LIBIGL REQUIRED QUIET)


# Qt for Harish' image loading library
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)


# Spdlog interface library
add_library(spdlog INTERFACE)
target_include_directories(spdlog INTERFACE external/spdlog/include)


# Quartet static library
set(QUARTET_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/external/quartet/src")
file(GLOB QUARTET_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/external/quartet/src/*.cpp")
file(GLOB QUARTET_INCS "${CMAKE_CURRENT_SOURCE_DIR}/external/quartet/src/*.h")
list(REMOVE_ITEM QUARTET_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/external/quartet/src/main.cpp")
list(REMOVE_ITEM QUARTET_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/external/quartet/src/gluvi.cpp")
list(REMOVE_ITEM QUARTET_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/external/quartet/src/view_tet.cpp")
add_library(quartet STATIC ${QUARTET_SRCS})
target_include_directories(quartet PUBLIC ${QUARTET_INCLUDE_DIRS})


# ContourTree static library
file(GLOB CT_SRCS external/Segmentangling/ContourTree/*.cpp)
set(CT_INCLUDE_DIRS external/Segmentangling/ContourTree)
list(REMOVE_ITEM CT_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/external/Segmentangling/ContourTree/main.cpp")
add_library(contourtree STATIC ${CT_SRCS})
target_compile_options(contourtree PRIVATE "-fopenmp")
target_link_libraries(contourtree Qt5::Core Qt5::Widgets -fopenmp spdlog)
target_include_directories(contourtree PUBLIC ${CT_INCLUDE_DIRS})
target_compile_definitions(contourtree PUBLIC CONTOUR_TREE_USE_SPDLOG)


# Utils static library
set(UTILS_INCLUDE_DIRS src src/utils)
file(GLOB UTILS_SRCS src/utils/*.cpp src/utils/gl/*.cpp)
file(GLOB UTILS_HEADER src/utils/*.h src/utils/gl/*.h)
list(FILTER UTILS_SRCS EXCLUDE REGEX cgal*)
list(FILTER UTILS_HEADER EXCLUDE REGEX cgal*)
add_library(utils STATIC ${UTILS_SRCS} ${UTILS_HEADER})
set_property(TARGET utils PROPERTY CXX_STANDARD 14)
set_property(TARGET utils PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(utils igl::core igl::opengl igl::cgal igl::triangle spdlog Qt5::Core Qt5::Widgets spdlog)
target_include_directories(utils PUBLIC ${UTILS_INCLUDE_DIRS})
target_include_directories(utils SYSTEM PUBLIC "${PROJECT_SOURCE_DIR}/external/glm")


# Eigen library
add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE src/utils/voroffset/3rdparty/eigen)


# Vor3d static library
set(VOR3D_DIR src/utils/voroffset/src/vor3d)
file(GLOB VOR3D_SRCS ${VOR3D_DIR}/*.cpp ${VOR3D_DIR}/*.h)
list(FILTER VOR3D_SRCS EXCLUDE REGEX ".*Dexelize.[h|cpp]")
add_library(vor3d STATIC ${VOR3D_SRCS})
target_include_directories(vor3d PUBLIC ${VOR3D_DIR}/..)
set_target_properties(vor3d PROPERTIES CXX_STANDARD 14)
set_target_properties(vor3d PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(vor3d PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(vor3d PUBLIC eigen)




############################
## Build main application ##
############################

add_subdirectory("src")
