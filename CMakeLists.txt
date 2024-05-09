cmake_minimum_required(VERSION 3.0.2)
project(bayesian_mapping)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
	tf2
	tf
	tf2_ros
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy tf tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bayesian_mapping	src/mapping.cpp)
add_dependencies(bayesian_mapping ${bayesian_mapping_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bayesian_mapping
  ${catkin_LIBRARIES}
)

