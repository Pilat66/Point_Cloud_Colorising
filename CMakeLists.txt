cmake_minimum_required(VERSION 3.5)
project(point_cloud_projection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Colourisation projects millions of points per frame — an unoptimised build is
# far too slow to keep up with a bag at 10 Hz. Default to Release.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fomit-frame-pointer -funroll-loops")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Wall -Wuninitialized -fno-omit-frame-pointer")

find_package(PCL 1.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    nav_msgs
    geometry_msgs
    pcl_ros
    pcl_conversions
    roslib
    visualization_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    nav_msgs
    geometry_msgs
    pcl_ros
    pcl_conversions
    roslib
    visualization_msgs
)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Shared calibration loading and projection core.
add_library(colorise_core
    src/calibration.cpp
    src/colorise.cpp
)
target_link_libraries(colorise_core
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(ColoriseScan src/ColoriseScan.cpp)
target_link_libraries(ColoriseScan colorise_core)

add_executable(ColoriseMap src/ColoriseMap.cpp)
target_link_libraries(ColoriseMap colorise_core)
