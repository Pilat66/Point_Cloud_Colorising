cmake_minimum_required(VERSION 3.10)
project(colorise_offline CXX)

# Standalone (ROS-free) C++ port of colorise_offline.py.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Projecting millions of points per frame — Release by default, like the ROS
# package this mirrors.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Wall -Wextra -Wuninitialized")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(colorise_offline
    src/main.cpp
    src/io.cpp
    src/trajectory.cpp
    src/project.cpp
)
target_link_libraries(colorise_offline ${OpenCV_LIBS} Threads::Threads)