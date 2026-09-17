cmake_minimum_required(VERSION 3.10)
project(colorise_map CXX)

# ROS-free (offline) counterpart of the ColoriseMap node: colourises a pre-built
# point cloud map from timestamped photos and a CSV of lidar poses.
#
# The projection core is not duplicated — the already validated sources of the
# colorise/ port are compiled straight into this target, so the gates, fisheye
# projection, occlusion z-buffer and RGB packing stay the same code.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Projecting millions of map points per frame: Release by default, like the
# other two tools.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Wall -Wextra -Wuninitialized")

set(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../colorise")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SHARED_DIR}/include
    ${SHARED_DIR}/third_party
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(colorise_map
    src/main.cpp
    src/cube_colourise.cpp
    ${SHARED_DIR}/src/io.cpp
    ${SHARED_DIR}/src/trajectory.cpp
    ${SHARED_DIR}/src/project.cpp
)
target_link_libraries(colorise_map ${OpenCV_LIBS} Threads::Threads)