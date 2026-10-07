cmake_minimum_required(VERSION 3.10)
project(colorise_map CXX)

# Export bundle of the cube-mode map colourisation (ColoriseMap --cube).
#
# This file is a copy of ColoriseMap/CMakeLists.txt from Point_Cloud_Colorising
# (commit f6a9202) with the source paths adjusted to this export layout:
#
#     ColoriseMap/src/*  ->  ColoriseMap/*
#     colorise/*         ->  colorise/*
#
# It is the only file in the bundle that differs from the repository; the C++
# sources are byte-identical. The projection core (gates, pinhole/fisheye
# projection, occlusion z-buffer, RGB packing) is shared with the offline scan
# tool, so the already validated colorise/ sources are compiled straight into
# this target instead of being duplicated.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Projecting millions of map points per frame: Release by default, like the
# repository this was exported from.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Wall -Wextra -Wuninitialized")

set(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/colorise")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/ColoriseMap
    ${SHARED_DIR}/include
    ${SHARED_DIR}/third_party
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(colorise_map
    ColoriseMap/main.cpp
    ColoriseMap/cube_colourise.cpp
    ${SHARED_DIR}/src/io.cpp
    ${SHARED_DIR}/src/trajectory.cpp
    ${SHARED_DIR}/src/project.cpp
)
target_link_libraries(colorise_map ${OpenCV_LIBS} Threads::Threads)
