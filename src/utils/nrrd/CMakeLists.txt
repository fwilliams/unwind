cmake_minimum_required(VERSION 2.8)

PROJECT(NRRD-example)

set(NRRD_LIBRARY "./NRRD" CACHE PATH "Path to the directory containing the NRRD include files.")

add_executable(Example example.cpp)
include_directories(Example "${NRRD_LIBRARY}")
