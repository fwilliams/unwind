################################################################################

cmake_minimum_required(VERSION 3.3)
project(offset3d)

################################################################################

# Add executable
add_executable(${PROJECT_NAME} offset3d.cpp)

# Let's get a little bit paranoid
include(SetWarnings)
target_compile_options(${PROJECT_NAME} PRIVATE ${ALL_WARNINGS} -g)

# Sanitizers
add_sanitizers(${PROJECT_NAME})

# Use C++14
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD_REQUIRED ON)

# Dependencies
target_link_libraries(${PROJECT_NAME} PRIVATE vor3d cli11 json)

# Output directory for binaries
set_target_properties(${PROJECT_NAME}
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
