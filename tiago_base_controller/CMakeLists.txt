cmake_minimum_required(VERSION 3.0.2)
project(tiago_base_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_msgs
  geometry_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning_interface 
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES tiago_grasping_on_the_move
  CATKIN_DEPENDS roscpp std_msgs gazebo_msgs geometry_msgs trajectory_msgs moveit_core moveit_ros_planning_interface tf2_ros tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  src  # Aggiungi questa riga per includere la directory src
)

add_executable(tiago_base_controller
  scripts/tiago_base_script.cpp
  src/base_controller.cpp  
)
target_link_libraries(tiago_base_controller ${catkin_LIBRARIES})
  

