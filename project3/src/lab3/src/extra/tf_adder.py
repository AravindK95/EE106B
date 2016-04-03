#!/usr/bin/env python
import sys
import numpy as np
import rospy
import tf
from tf2_msgs.msg import TFMessage
from lab3.msg import FrameCall

OBJECT_FRAME_NAME = 'graspable_object'
frame_dict = {}

def handle_call(data):
    if data.to_add:
        rbt = ((data.rbt.translation.x, 
                data.rbt.translation.y, 
                data.rbt.translation.z), 
               (data.rbt.rotation.x, 
                data.rbt.rotation.y, 
                data.rbt.rotation.z, 
                data.rbt.rotation.w))

        frame_dict[data.name] = rbt

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

    rospy.Subscriber('lab3/tf', FrameCall, handle_call)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        print "Publishing: "+str(frame_dict.keys())
        for name, rbt in frame_dict.items():
            publish_frame(br, rbt[0], rbt[1], name, OBJECT_FRAME_NAME)

        rate.sleep()
