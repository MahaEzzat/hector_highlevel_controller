cmake_minimum_required(VERSION 2.8.3)
project(hector_highlevel_controller)
find_package(catkin REQUIRED COMPONENTS actionlib geometry_msgs hector_quadrotor_interface roscpp tf2 tf2_geometry_msgs pcl_conversions pcl_ros tf laser_geometry cv_bridge image_transport)
  
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS roscpp
    DEPENDS
)


add_executable(${PROJECT_NAME} 
  src/hector_highlevel_controller_node.cpp
  src/HectorHighlevelController.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)