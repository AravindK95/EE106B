#!/usr/bin/env python
import sys
import rospy
import roslib
import time

import numpy as np

from std_msgs.msg import *

class RingBuffer():
    def __init__(self, size):
        self.size = size
        self.data = [0.0 for _ in range(size)]
        self.ptr = 0
        self.isFull = False

    def push(self, item):
        self.data[self.ptr] = item
        self.ptr = (self.ptr+1)%self.size
        if not self.isFull and self.ptr == 0:
            self.isFull = True

    # get() does not guarantee FIFO-ordered output
    def get(self):
        if self.isFull:
            return self.data
        else:
            return self.data[:self.ptr]

    def clear(self):
        self.isFull = False
        self.ptr = 0

    def filled(self):
        return self.isFull

    def getAvg(self):
        return np.average(self.get())

  
def listener(l_sub_str, r_sub_str, l_pub, r_pub):
    #sets up listener to the TF topic, which listens to TF Messages, and calls a callback

    print("Initializing node... ")
    rospy.init_node("EncoderListener")

    rospy.Subscriber(l_sub_str, Int16, make_callback(l_pub, 3))
    rospy.Subscriber(r_sub_str, Int16, make_callback(r_pub, 3))

    print("Construction of listeners completed")

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

def make_callback(pub, N):
    posFilter = RingBuffer(N)
    inClock = RingBuffer(N)

    def callback(data):
        #run every time i see a TFMessage on tf_topic.
        metersPerCount = 0.00009948379 #0.038 m diameter of treads * pi / 1200 encoder counts per revolution
        encoderCount = data.data
        metersTraveled = encoderCount*metersPerCount

        posFilter.push(metersTraveled)
        inClock.push(rospy.Time.now().to_sec())
        if not posFilter.filled():
            return

        velocityEst = 0
        # This could be better filtered + more readable
        for i in range(len(posFilter.get())-1):
            velocityEst = velocityEst + (posFilter.get()[i+1]-posFilter.get()[i])/(inClock.get()[i+1]-inClock.get()[i])
        velocityEst = velocityEst / (len(posFilter.get()) - 1)

        pub.publish(velocityEst)

    return callback



#     # print child_frame

#     #print("callback run \n\r")

#     #right now, hardcoded to look out for AR_Marker_63.  This may not be the world's greatest Idea...
#     if  (child_frame==ar_marker):
#         now = rospy.Time.now()
#         #tf_listener.waitForTransform('ar_marker_63','left_gripper',now,rospy.Dure)
#         try:
#             print "orig vals"
#             print(data)

#             #print translations.x
#             #print translations.y
#             ndata = data
#             ndata.transforms[0].transform.translation.x = -translations.x
#             ndata.transforms[0].transform.translation.y = -translations.y
#             ndata.transforms[0].transform.rotation.x = -quaternion.x
#             ndata.transforms[0].transform.rotation.y = -quaternion.y
#             ndata.transforms[0].child_frame_id = "/your_mother"
#             print "new vals"
#             #print ndata.transforms[0].transform.translation.x
#             #print ndata.transforms[0].transform.translation.y

#             print ndata

#         #time0 asks for the most recent one
#         except:
#             #print("error with the update \n\r")
#             pass

#         br = tf.TransformBroadcaster()
#         br.sendTransform((ndata.transforms[0].transform.translation.x, ndata.transforms[0].transform.translation.y, ndata.transforms[0].transform.translation.z), 
#         (quaternion.x, quaternion.y, quaternion.z, quaternion.w), 
#         rospy.Time.now(),
#         ar_marker + "n", "head_camera")


# def get_pose(limb):
#     pose = {}

#     while len(pose) is 0:
#         #sometimes, i'm getting empty poses.  don't know why, but this should take care of it.
#         pose = limb.endpoint_pose()
#         #print(pose)
#         time.sleep(.01)

#     pose = {'rot': list(pose['orientation']), 
#             'trans' : list(pose['position'])}

#     return pose

def init():
    left_pub = rospy.Publisher('proj1/l_vel', Float32, queue_size=10)
    right_pub = rospy.Publisher('proj1/r_vel', Float32, queue_size=10)
    
    listener('/zumy7a/l_enc', '/zumy7a/r_enc', left_pub, right_pub)



if __name__ == '__main__':
    init()