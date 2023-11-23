cmake_minimum_required(VERSION 3.0.2)
project(ruka_goal)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  gazebo_ros
  message_generation
  trajectory_msgs
  visualization_msgs
  tf2
)
find_package(moveit_visual_tools REQUIRED)
find_package(gazebo 11 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${moveit_visual_tools_LIBRARIES}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include/ruka_goal
  LIBRARIES ruka_goal
  CATKIN_DEPENDS
    roscpp
    std_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    gazebo_ros
    message_generation
    trajectory_msgs
    visualization_msgs
    tf2
#  DEPENDS system_lib
)

add_executable(posli_ruku_niekde /home/alex/catkin_ws/src/ruka_goal/src/posli_ruku_niekde.cpp)

#add_library(posli_ruku_niekde_lib src/scripts/posli_ruku_niekde.cpp)
#target_link_libraries(posli_ruku_niekde_lib ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#target_link_libraries(posli_ruku_niekde_node posli_ruku_niekde_lib ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_executable(posli_ruku_niekde src/scripts/posli_ruku_niekde.cpp)
target_link_libraries(posli_ruku_niekde
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${EIGEN3_INCLUDE_DIRS}
                      ${moveit_visual_tools_LIBRARIES}
                      ${GAZEBO_LIBRARIES}
                      #${}
                      ${moveit_ros_planning_interface}
                      ${moveit_planning_scene_interface}
                      )
