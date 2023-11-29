cmake_minimum_required(VERSION 3.0.2)

project(ruka_description)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS roslaunch rviz joint_state_publisher robot_description gazebo
)

include_directories(${catkin_INCLUDE_DIRS})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
