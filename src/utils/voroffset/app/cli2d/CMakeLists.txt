################################################################################
# General Information
################################################################################

cmake_minimum_required(VERSION 3.3)

################################################################################

function(add_application APP_SOURCE)
	# Add executable
	get_filename_component(APP_NAME ${APP_SOURCE} NAME_WE)
	add_executable(${APP_NAME} ${APP_SOURCE})

	# Let's get a little bit paranoid
	include(SetWarnings)
	target_compile_options(${APP_NAME} PRIVATE ${ALL_WARNINGS})

	# Use C++14
	set_target_properties(${APP_NAME} PROPERTIES CXX_STANDARD 14)
	set_target_properties(${APP_NAME} PROPERTIES CXX_STANDARD_REQUIRED ON)

	# Vor2d library
	target_link_libraries(${APP_NAME} PRIVATE vor2d yimg cli11 nanosvg)

	# Output directory for binaries
	set_target_properties(${APP_NAME}
		PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
	)
endfunction()

################################################################################

set(ALL_APPS
		offset2d.cpp
		readsvg.cpp
)

foreach(APP ${ALL_APPS})
	message(STATUS "Compiling single-source application: ${APP}")
	add_application(${APP})
endforeach()
