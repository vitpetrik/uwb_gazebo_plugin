cmake_minimum_required(VERSION 3.1.2 FATAL_ERROR)
project(uwb_gazebo_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -g -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
  mrs_msgs
)

add_message_files(DIRECTORY msg FILES
  uwbPose.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
   INCLUDE_DIRS inc
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs
   LIBRARIES uwb_range
)

find_package(gazebo REQUIRED)
ign_import_target(UUID)
find_package(roscpp REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} inc)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(uwb_range SHARED src/uwb_range.cpp)
target_link_libraries(uwb_range ${GAZEBO_LIBRARIES})
add_dependencies(uwb_range ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
