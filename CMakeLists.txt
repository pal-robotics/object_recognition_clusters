cmake_minimum_required(VERSION 2.8.3)
project(object_recognition_clusters)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  object_recognition_msgs
  roscpp
  rospy
  tf2
  tf
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_recognition_clusters
#  CATKIN_DEPENDS geometry_msgs object_recognition_msgs roscpp rospy tf2
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
