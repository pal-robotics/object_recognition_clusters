#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, PAL Robotics SL.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the PAL Robotics nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Bence Magyar

#Offers a service to find the principal directions and bounding box of a point cluster

import rospy
import tf
from object_recognition_msgs.msg import RecognizedObjectArray
import object_recognition_clusters.cluster_bounding_box_finder as cluster_bounding_box_finder
#import object_recognition_clusters.cluster_bounding_box_finder_3d as cluster_bounding_box_finder_3d

class ClusterToPose:

    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.cbbf = cluster_bounding_box_finder.ClusterBoundingBoxFinder(self.tf_listener, self.tf_broadcaster)
#        self.cbbf3d = cluster_bounding_box_finder_3d.ClusterBoundingBoxFinder3D(self.tf_listener)

        rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.callback)

    def callback(self, data):
        rospy.loginfo(rospy.get_name() + ": This message contains %d objects." % len(data.objects))
        # for myobject in data.objects:
            #print object.point_clouds[0]
        myobject = data.objects[0]
        self.cbbf.find_object_frame_and_bounding_box(myobject.point_clouds[0])

if __name__ == '__main__':
    rospy.init_node('clusters_to_pose', anonymous=True)
    clusters_to_pose = ClusterToPose()
    rospy.loginfo("cluster bounding box finder is ready for queries")
    rospy.spin()

