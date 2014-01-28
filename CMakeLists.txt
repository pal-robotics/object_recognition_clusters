cmake_minimum_required(VERSION 2.8.3)
project(object_recognition_clusters)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs object_recognition_msgs roscpp
  rospy tf2 tf ecto object_recognition_core
  ecto_ros pcl_ros
)

catkin_python_setup()

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE 1)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_recognition_clusters
#  CATKIN_DEPENDS geometry_msgs object_recognition_msgs roscpp rospy tf2
#  DEPENDS system_lib
)

add_subdirectory(src)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
