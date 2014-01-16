#!/usr/bin/env python
import rospy
from object_recognition_msgs.msg import RecognizedObjectArray

def callback(data):
    rospy.loginfo(rospy.get_name() + ": This message contains %d objects." % len(data.objects))
    # for object in data.objects:
    #print object.point_clouds[0]
    # self.cbbf.find_object_frame_and_bounding_box(object.point_clouds[0]) 

if __name__ == '__main__':
    rospy.init_node('clusters_to_pose', anonymous=True)
    rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, callback)
    rospy.spin()
