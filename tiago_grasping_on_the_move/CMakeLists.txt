cmake_minimum_required(VERSION 3.0.2)
project(tiago_grasping_on_the_move)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

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
