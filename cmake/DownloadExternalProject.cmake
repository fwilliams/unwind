################################################################################
include(DownloadProject)

# With CMake 3.8 and above, we can hide warnings about git being in a
# detached head by passing an extra GIT_CONFIG option
if(NOT (${CMAKE_VERSION} VERSION_LESS "3.8.0"))
  set(DL_EXTERNAL_PROJ_EXTRA_OPTIONS "GIT_CONFIG advice.detachedHead=false")
else()
  set(DL_EXTERNAL_PROJ_EXTRA_OPTIONS "")
endif()

# Shortcut function
function(download_external_project name)
  download_project(
    PROJ         ${name}
    SOURCE_DIR   ${DL_EXTERNAL_PROJ_DIRECTORY}/${name}
    DOWNLOAD_DIR ${DL_EXTERNAL_PROJ_DIRECTORY}/.cache/${name}
    QUIET
    ${DL_EXTERNAL_PROJ_EXTRA_OPTIONS}
    ${ARGN}
  )
endfunction()


