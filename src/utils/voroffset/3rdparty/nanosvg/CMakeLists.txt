################################################################################
cmake_minimum_required(VERSION 3.3)
################################################################################

if(NOT TARGET nanosvg)
	add_library(nanosvg nanosvg.cpp)
	target_include_directories(nanosvg SYSTEM PUBLIC .)
endif()
