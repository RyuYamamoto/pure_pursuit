cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  visualization_msgs
  roscpp
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pure_pursuit
#  CATKIN_DEPENDS geometry_msgs roscpp tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit_node src/pure_pursuit.cpp src/pure_pursuit_node.cpp)
target_link_libraries(pure_pursuit_node ${catkin_LIBRARIES})
