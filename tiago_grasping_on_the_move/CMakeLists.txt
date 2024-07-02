cmake_minimum_required(VERSION 3.0.2)
project(tiago_grasping_on_the_move)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  LIBRARIES tiago_grasping_on_the_move
  CATKIN_DEPENDS roscpp rospy std_msgs gazebo_msgs geometry_msgs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src  # Aggiungi questa riga per includere la directory src
)

add_executable(tiago_grasping_on_the_move 
  scripts/tiago_grasping_on_the_move.cpp
  src/base_controller.cpp  # Aggiungi questo
)

target_link_libraries(tiago_grasping_on_the_move
  ${catkin_LIBRARIES}
)
