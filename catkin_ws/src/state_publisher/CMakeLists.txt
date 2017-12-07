cmake_minimum_required(VERSION 2.8.3)
project(state_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  message_generation
  std_msgs
  tf
)

add_message_files(
  FILES
  FlyingBilboPose.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_publisher src/state_publisher.cpp)

target_link_libraries(
  state_publisher
  ${catkin_LIBRARIES}
)
