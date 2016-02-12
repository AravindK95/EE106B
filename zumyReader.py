#!/usr/bin/env python
import sys
import rospy
import roslib

import numpy as np

from std_msgs.msg import *

class RingBuffer():
    def __init__(self, size):
        self.size = size
        self.data = [0.0 for _ in range(size)]
        self.ptr = 0
        self.buffer_is_full_flag = False

    def push(self, item):
        self.data[ptr] = item
        self.ptr = (self.ptr+1)%self.size
        if not self.buffer_is_full_flag and self.ptr == 0:
            self.buffer_is_full_flag = True

    def clear(self):
        for i in range(self.size):
            self.data[i] = 0.0
        self.buffer_is_full_flag = False
        self.ptr = 0

    def get(self):
        if self.buffer_is_full_flag:
            return self.data
        else:
            return self.data[:self.ptr]

  
def listener(tf_topic):
    #sets up listener to the TF topic, which listens to TF Messages, and calls a callback

    print("Initializing node... ")
    rospy.init_node("EncoderListener")
    print("topic is " + str(tf_topic))

    rospy.Subscriber('l_enc',Int_16,callback)

    print("Construction of listener completed")

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

def callback(data):
    #run every time i see a TFMessage on tf_topic.
    metersPerCount = 0.00009948379 #0.038 m diameter of treads * pi / 1200 encoder counts per revolution
    encoderCount = data
    metersTraveled = encoderCount*countstoMeters


    # print child_frame

    #print("callback run \n\r")

    #right now, hardcoded to look out for AR_Marker_63.  This may not be the world's greatest Idea...
    if  (child_frame==ar_marker):
        now = rospy.Time.now()
        #tf_listener.waitForTransform('ar_marker_63','left_gripper',now,rospy.Dure)
        try:
            print "orig vals"
            print(data)

            #print translations.x
            #print translations.y
            ndata = data
            ndata.transforms[0].transform.translation.x = -translations.x
            ndata.transforms[0].transform.translation.y = -translations.y
            ndata.transforms[0].transform.rotation.x = -quaternion.x
            ndata.transforms[0].transform.rotation.y = -quaternion.y
            ndata.transforms[0].child_frame_id = "/your_mother"
            print "new vals"
            #print ndata.transforms[0].transform.translation.x
            #print ndata.transforms[0].transform.translation.y

            print ndata

        #time0 asks for the most recent one
        except:
            #print("error with the update \n\r")
            pass

        br = tf.TransformBroadcaster()
        br.sendTransform((ndata.transforms[0].transform.translation.x, ndata.transforms[0].transform.translation.y, ndata.transforms[0].transform.translation.z), 
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w), 
        rospy.Time.now(),
        ar_marker + "n", "head_camera")


def get_pose(limb):
    pose = {}

    while len(pose) is 0:
        #sometimes, i'm getting empty poses.  don't know why, but this should take care of it.
        pose = limb.endpoint_pose()
        #print(pose)
        time.sleep(.01)

    pose = {'rot': list(pose['orientation']), 
            'trans' : list(pose['position'])}

    return pose

def init():
    



if __name__ == '__main__':
    init()