# We need cmake version 2.8 or higher
cmake_minimum_required(VERSION 2.8)

# Create a new solution
project(NRRD-Converters)

# Search for ITK
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

# Add Heaeder-Only libraries
include_directories(../)
include_directories(.)

# Convert between any two formats and types that ITk supports
add_executable(itk_convert
	itk_convert.cpp
)
target_link_libraries(itk_convert ${ITK_LIBRARIES})

# Convert NRRD to any ITK supported format
add_executable(nrrd2itk
	nrrd2itk.cpp
)
target_link_libraries(nrrd2itk ${ITK_LIBRARIES})

# Convert ITK readable format to NRRD
add_executable(itk2nrrd
	itk2nrrd.cpp
)
target_link_libraries(itk2nrrd ${ITK_LIBRARIES})


install(TARGETS nrrd2itk itk2nrrd itk_convert RUNTIME DESTINATION ".")
