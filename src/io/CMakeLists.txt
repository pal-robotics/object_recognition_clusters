include_directories(SYSTEM
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
