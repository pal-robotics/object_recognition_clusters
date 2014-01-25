# create the cells to output messages
find_package(Eigen REQUIRED)

include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${ecto_ros_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${object_recognition_core_INCLUDE_DIRS}
)

ectomodule(io_clusters DESTINATION ${PROJECT_NAME}/ecto_cells
                  INSTALL
           module.cpp
           PointCloudMsgAssembler.cpp
)

link_ecto(io_clusters ${catkin_LIBRARIES})
