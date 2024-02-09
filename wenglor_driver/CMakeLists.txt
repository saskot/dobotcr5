cmake_minimum_required(VERSION 3.0.2)
project(wenglor_driver)
#find_package(PCL 1.3 REQUIRED)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES wenglor_driver
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
# include
  Sensor3D/include
  ${catkin_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)


set(WENGLOR_HEADER_FILES Sensor3D/include/sensor3d.h Sensor3D/include/sensor3ddefines.h)
link_directories(Sensor3D/lib)
#link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable(wenglor_test src/main.cpp ${WENGLOR_HEADER_FILES})
#add_executable(pcd_write_test /home/alex/catkin_ws/src/wenglor_driver/src/pcd_write.cpp)

target_link_libraries(wenglor_test Sensor3D ${catkin_LIBRARIES})
#target_link_libraries(pcd_write_test ${PCL_LIBRARIES})






## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/wenglor_driver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/wenglor_driver_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

