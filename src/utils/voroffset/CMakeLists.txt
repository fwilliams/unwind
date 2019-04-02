################################################################################
# General Information
################################################################################

cmake_minimum_required(VERSION 3.3)
project(voroffset)

################################################################################

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${THIRD_PARTY_DIR}/sanitizers-cmake/cmake)

if(NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release")
endif()

################################################################################

# Use folder in Visual Studio
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Color output
include(UseColors)

# Export compile flags(used for autocompletion of the C++ code)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# CMake plugin for vscode
include(CMakeToolsHelpers OPTIONAL)

# Generate position independent code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build shared or static?
# set(BUILD_SHARED_LIBRARIES OFF)

################################################################################

# Options for enabling/disabling optional libraries
option(SANITIZE_ADDRESS          "Sanitize Address"   OFF)
option(SANITIZE_MEMORY           "Sanitize Memory"    OFF)
option(SANITIZE_THREAD           "Sanitize Thread"    OFF)
option(SANITIZE_UNDEFINED        "Sanitize Undefined" OFF)
option(ENABLE_TBB                "Enable TBB"         ON)

# Override cached options
set(SANITIZE_ADDRESS          OFF CACHE BOOL "" FORCE)
set(ENABLE_TBB                OFF CACHE BOOL "" FORCE)

################################################################################
# 3rd party libraries
################################################################################

# Sanitizers
find_package(Sanitizers)

# Eigen3 library
add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE ${THIRD_PARTY_DIR}/eigen)

# CLI11 library
add_library(cli11 INTERFACE)
target_include_directories(cli11 SYSTEM INTERFACE ${THIRD_PARTY_DIR}/cli11)

# Geogram library
include(geogram)

# Json library
add_library(json INTERFACE)
target_include_directories(json SYSTEM INTERFACE ${THIRD_PARTY_DIR}/json)

# Nanosvg library
add_subdirectory(${THIRD_PARTY_DIR}/nanosvg)

# YImage library
add_library(yimg STATIC ${THIRD_PARTY_DIR}/yimg/YImage.cpp)
target_include_directories(yimg PUBLIC ${THIRD_PARTY_DIR}/yimg)

# TBB library
if(ENABLE_TBB)
	set(TBB_BUILD_STATIC ON CACHE BOOL " " FORCE)
	set(TBB_BUILD_SHARED OFF CACHE BOOL " " FORCE)
	set(TBB_BUILD_TBBMALLOC OFF CACHE BOOL " " FORCE)
	set(TBB_BUILD_TBBMALLOC_PROXY OFF CACHE BOOL " " FORCE)
	set(TBB_BUILD_TESTS OFF CACHE BOOL " " FORCE)

	add_subdirectory(${THIRD_PARTY_DIR}/tbb tbb)
	set_property(TARGET tbb_static tbb_def_files PROPERTY FOLDER "dependencies")
	target_compile_definitions(tbb_static PUBLIC -DUSE_TBB)
	target_include_directories(tbb_static SYSTEM PUBLIC ${THIRD_PARTY_DIR}/tbb/include)
endif()

################################################################################

# 2D version prototype
add_subdirectory(src/vor2d)

# 3D version prototype
add_subdirectory(src/vor3d)

# Binary executables
add_subdirectory(app)
