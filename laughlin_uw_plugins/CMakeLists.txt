cmake_minimum_required(VERSION 2.8.3)
project(laughlin_uw_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  rospy
  tf
  std_msgs
  message_generation
  geometry_msgs
  gps_common
)

# For Eigen
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories( include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

find_package(gazebo REQUIRED)

add_message_files(
    FILES
    USBL.msg
    DVL.msg
    OWTT.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp rospy tf message_runtime
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
)

find_package (GeographicLib 1.46 REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})
add_definitions (${GeographicLib_DEFINITIONS})

add_library(dvl_plugin src/dvl_plugin.cpp)
target_link_libraries(dvl_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} RayPlugin)
add_dependencies(dvl_plugin laughlin_uw_plugins_generate_messages_cpp)

add_library(usbl_plugin src/usbl_plugin.cpp)
target_link_libraries(usbl_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(usbl_plugin laughlin_uw_plugins_generate_messages_cpp)

add_library(owtt_plugin src/owtt_plugin.cpp)
target_link_libraries(owtt_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(owtt_plugin laughlin_uw_plugins_generate_messages_cpp)

add_library(gps_plugin src/gps_plugin.cpp)
target_link_libraries(gps_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GeographicLib_LIBRARIES})
add_dependencies(gps_plugin laughlin_uw_plugins_generate_messages_cpp)

add_library(trajectory_forcer src/trajectory_forcer.cpp)
target_link_libraries(trajectory_forcer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(spherical_test src/spherical_test.cpp)
target_link_libraries(spherical_test ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})