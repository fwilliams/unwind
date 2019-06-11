# We need cmake version 2.8 or higher
cmake_minimum_required(VERSION 2.8)

# Create a new solution
PROJECT(NRRD-Tools)

# Add Heaeder-Only libraries
include_directories(../)

# Get and Set Meta info in NRRD header
add_executable(nrrdInfo
	nrrdInfo.cpp
)

# Create NRRD file from Raw data. This tool does not actually use ITK.
add_executable(raw2nrrd
	raw2nrrd.cpp
)

# Extract raw data chunk from a NRRD file
add_executable(nrrd2raw
	nrrd2raw.cpp
)

# Convert existing NRRD file of arbitrary type to float
add_executable(nrrd2float
	nrrd2float.cpp
)

# Create NRRD file from Raw data stored in Dennerlein's bin format
add_executable(dennerlein2nrrd
	dennerlein2nrrd.cpp
)

# Convert between NRRD and MetaIO MHA image (does not use ITK)
add_executable(nrrd2mha
	nrrd2mha.cpp
)
# Convert between MetaIO MHA image and NRRD (Please use ITK version instead)
#add_executable(mha2nrrd
#	mha2nrrd.cpp
#)

# Scale, flip and normalize NRRD files
add_executable(nrrdResample
	nrrdResample.cpp
)

install(TARGETS nrrdInfo raw2nrrd nrrd2raw nrrd2float nrrd2mha nrrdResample RUNTIME DESTINATION ".")
