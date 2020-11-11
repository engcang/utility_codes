cmake_minimum_required(VERSION 2.8.3)
project(octomap_grapher)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package()

add_executable(octomap_grapher_node src/main.cpp)
target_link_libraries(octomap_grapher_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})