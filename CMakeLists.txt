cmake_minimum_required(VERSION 3.0.2)
project(uwb_location)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  serial
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  uwb.msg
  RangeArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uwb_location
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime serial geometry_msgs
  #  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_executable(uwb_range src/uwblocation.cpp)

target_link_libraries(uwb_range
  ${catkin_LIBRARIES}
)

add_executable(uwb_location src/location.cpp
  src/trilateration.cpp)

target_link_libraries(uwb_location
  ${catkin_LIBRARIES}
)

