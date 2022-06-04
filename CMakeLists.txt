cmake_minimum_required(VERSION 3.16.2) # For boost 1.72

project(mve VERSION 0.1.0 DESCRIPTION "mve")

if("${MULTIVIEW_DEPS_DIR}" STREQUAL "")
  message(FATAL_ERROR "Set MULTIVIEW_DEPS_DIR as the path to a conda env having the dependencies.")
endif()

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Setting build type: ${CMAKE_BUILD_TYPE}")

# Where to search for dependencies
set(CMAKE_PREFIX_PATH "${MULTIVIEW_DEPS_DIR};${CMAKE_PREFIX_PATH}")
set(CMAKE_MODULE_PATH "${MULTIVIEW_DEPS_DIR}/lib/cmake/Ceres;${CMAKE_MODULE_PATH}")

set(CMAKE_CXX_STANDARD 11)

# Image libs
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(TIFF REQUIRED)

add_subdirectory(libs/util)
add_subdirectory(libs/mve)

