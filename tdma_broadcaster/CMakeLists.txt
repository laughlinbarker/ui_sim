cmake_minimum_required(VERSION 2.8.3)
project(tdma_broadcaster)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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

add_executable(tdma_test src/tdma_broadcaster.cpp)
target_link_libraries(tdma_test ${catkin_LIBRARIES})
set_target_properties(tdma_test PROPERTIES COMPILE_FLAGS "-O3")