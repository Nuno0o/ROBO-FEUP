cmake_minimum_required(VERSION 2.8.3)
project(robo_feup)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    roslib
    roscpp
    tf
    stdr_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)
#string stdr_obstacle_avoidance - robo_wall
#proj = stdr_samples robo_feup
#files = obstacle_avoidance wall_follow
####################### Obstacle avoidance ##################################
add_executable(robo_wall
  src/main.cpp
  src/controller.cpp)
add_dependencies(robo_wall ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(robo_wall
  ${catkin_LIBRARIES}
)

# Install excecutables
install(TARGETS robo_wall
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)