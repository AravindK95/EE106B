#!/usr/bin/env python
import sys
import numpy as np
import rospy
import tf
from tf2_msgs.msg import TFMessage
from grasper_ctrl.msg import FrameCall

frame_dict = {}

def handle_call(data):
    if data.to_add:
        info = ((data.rbt.translation.x, 
                data.rbt.translation.y, 
                data.rbt.translation.z), 
               (data.rbt.rotation.x, 
                data.rbt.rotation.y, 
                data.rbt.rotation.z, 
                data.rbt.rotation.w),
               data.base)

        frame_dict[data.name] = info

    else:
        remove_frame(data.name)


def publish_frame(br, t, q, name, base):
    assert len(t) == 3, "Invalid trans: "+str(t)
    assert len(q) == 4, "Invalid rot: "+str(q)

    br.sendTransform(t, q, rospy.Time.now(), name, base)

def remove_frame(name):
    if name in frame_dict:
        del frame_dict[name]
    else:
        print(name+" not in list of frames to publish!")

if __name__ == '__main__':
    rospy.init_node('grasp_tf_pub')
    br = tf.TransformBroadcaster()

    rospy.Subscriber('grasper_ctrl/tf', FrameCall, handle_call)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        print "Publishing: "+str(frame_dict.keys())
        for name, info in frame_dict.items():
            publish_frame(br, info[0], info[1], name, info[2])

        rate.sleep()
