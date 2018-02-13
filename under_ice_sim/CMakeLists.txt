cmake_minimum_required(VERSION 2.8.3)
project(under_ice_sim)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES tdma_test
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tdma_node src/tdma_broadcaster.cpp)
target_link_libraries(tdma_node ${catkin_LIBRARIES})
set_target_properties(tdma_node PROPERTIES COMPILE_FLAGS "-O3")

add_executable(ice_rel_odom_publisher src/relative_ice_odom_publisher.cpp)
target_link_libraries(ice_rel_odom_publisher ${catkin_LIBRARIES})
set_target_properties(ice_rel_odom_publisher PROPERTIES COMPILE_FLAGS "-O3")