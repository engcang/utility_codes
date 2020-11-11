cmake_minimum_required(VERSION 3.0.2)
project(cv_cam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
)

find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cv_cam
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(cv_cam_node src/cv_cam.cpp)
target_link_libraries(cv_cam_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
