cmake_minimum_required(VERSION 2.8.3)
project(trajectory_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  crazyflie_driver
  cmake_modules
  tf
)
find_package(Eigen REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS Eigen)

include_directories(
    include
    ../../../crazyflie-lib-python
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
add_executable(lqr_controller src/lqr_controller.cpp)
add_executable(ball_follower src/ball_follower.cpp)
add_executable(dynamic_swingup src/dynamic_swingup.cpp)

target_link_libraries(
  trajectory_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(
  ball_follower
  ${catkin_LIBRARIES}
)

target_link_libraries(
    lqr_controller
    ${catkin_LIBRARIES}
)

target_link_libraries(
    dynamic_swingup
    ${catkin_LIBRARIES}
)
