################################################################################

cmake_minimum_required(VERSION 3.3)
project(vor2d)

################################################################################

add_library(${PROJECT_NAME}
		Common.cpp
		Common.h
		CompressedImage.cpp
		CompressedImage.h
		DistanceTransform.cpp
		DistanceTransform.h
		Dexelize.cpp
		Dexelize.h
		DoubleCompressedImage.cpp
		DoubleCompressedImage.h
		DoubleVoronoi.cpp
		DoubleVoronoi.h
		Image.h
		MorphologyOperators.cpp
		MorphologyOperators.h
		Voronoi.cpp
		Voronoi.h
)

target_include_directories(${PROJECT_NAME} PUBLIC ..)

################################################################################

# Let's get a little bit paranoid
include(SetWarnings)
target_compile_options(${PROJECT_NAME} PRIVATE ${ALL_WARNINGS})

# Use C++14
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD_REQUIRED ON)

# Generate position independent code
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Sanitizers
add_sanitizers(${PROJECT_NAME})

################################################################################
# Required libraries
################################################################################

# Eigen3 library
target_link_libraries(${PROJECT_NAME} PUBLIC eigen)

# Geogram library
target_link_libraries(${PROJECT_NAME} PUBLIC geogram)

# nanosvg library
target_link_libraries(${PROJECT_NAME} PUBLIC nanosvg)
