cmake_minimum_required(VERSION 3.5)
project(VO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

find_package(OpenCV 3.4.2 REQUIRED)
# find_package(OpenCV REQUIRED)

add_executable(pose_estimate_2d2d pose_estimate_2d2d.cpp)
target_link_libraries(pose_estimate_2d2d ${OpenCV_LIBS})