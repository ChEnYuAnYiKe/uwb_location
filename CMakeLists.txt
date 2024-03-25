cmake_minimum_required(VERSION 3.0.2)
project(uwb_location)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  serial
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  uwb.msg
  
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Uwb_Location
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime serial
#  DEPENDS system_lib
)

find_package(Eigen3  REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(trilateration include/uwb_location/trilateration.h src/trilateration.cpp)
add_dependencies(trilateration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trilateration
         ${EIGEN3_LIBS}
         ${catkin_LIBRARIES})

add_library(autoposition include/uwb_location/autoposition.h src/autoposition.cpp)
add_dependencies(autoposition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoposition
        ${EIGEN3_LIBS}
        ${catkin_LIBRARIES})
        
add_library(uwblocation include/uwb_location/uwblocation.h src/uwblocation.cpp)
add_dependencies(uwblocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwblocation
        ${EIGEN3_LIBS}
        ${catkin_LIBRARIES})        

add_executable(uwblocating src/uwblocation.cpp src/trilateration.cpp src/autoposition.cpp)
add_dependencies(uwblocating ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwblocating
        ${catkin_LIBRARIES})
