cmake_minimum_required(VERSION 2.8.3)
project(chang_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  teleop_msg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES chang_teleop
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vel_publisher src/vel_publisher.cpp)
target_link_libraries(vel_publisher ${catkin_LIBRARIES})
add_dependencies(vel_publisher chang_teleop_generate_messages_cpp)

add_executable(vel_subscriber src/vel_subscriber.cpp)
target_link_libraries(vel_subscriber ${catkin_LIBRARIES})
add_dependencies(vel_subscriber chang_teleop_generate_messages_cpp)