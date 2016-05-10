#!/usr/bin/env python
import sys
import rospkg
import rospy
import numpy as np

from std_msgs.msg import *
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from visualization_msgs.msg import Marker

MESH_FILENAME = 'package://grasper_plan/data/dae/'+sys.argv[1]

if __name__ == '__main__':
    rospy.init_node('grasp_marker_pub')
    pub = rospy.Publisher('grasper_ctrl/marker_pub', Marker, queue_size=3)

    m = Marker()
    m.header.frame_id = 'graspable_object'
    m.header.stamp = rospy.Time()
    m.ns = '.'.join(sys.argv[1].split('.')[:-1])    # removes file extension from argv[1]
    m.id = 0
    m.type = Marker.MESH_RESOURCE
    m.action = Marker.ADD
    m.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
    m.scale = Vector3(1,1,1)
    m.color = ColorRGBA(0.5,0.5,0.5,1)
    m.lifetime = rospy.Duration(3, 0)
    m.mesh_resource = MESH_FILENAME

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish(m)
        m.id += 1
        # print m.id
        r.sleep()
