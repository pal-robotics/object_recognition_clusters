cmake_minimum_required(VERSION 2.8.3)
project(object_recognition_clusters)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs object_recognition_msgs roscpp
  rospy tf ecto ecto_ros pcl_ros
)

catkin_python_setup()

catkin_package()

add_subdirectory(src)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
