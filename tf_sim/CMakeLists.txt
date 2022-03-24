cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(tf_sim)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Float32Stamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Defining the package
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
