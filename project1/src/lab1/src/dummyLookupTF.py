#!/usr/bin/env python
import tf
import sys
import math
import numpy as np
import rospy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int16, Bool, Float64
from geometry_msgs.msg import Transform, Vector3, Twist
# FOR CHANGING THE ORIENTATION OF THE ZUMY TO TAGS
# NOT IMPLEMENTED YET!!
# import exp_quat_func as eqf

listener = None
zumy_stop = Twist(Vector3(0,0,0),Vector3(0,0,0))

# Check if Zumy is close enought to target position and rotation
def checkTThresh(trans, rot):
    tthresh = .1
    for i in range(0,2):
        if trans(i)>tthresh:
            return False
    return True

def checkRThresh(trans, rot):
    rthresh = .1
    if trans(1)<rthresh:
        return False
    return True

# Tell the zumy what to do
def command(ar_tags):
    listener = tf.TransformListener()
    r = rospy.Rate(10)
    curr = 2
    target = 4
    while not rospy.is_shutdown(): 
        trans,rot = listener.lookupTransform(ar_tags['ar1'], ar_tags['ar2'], rospy.Time(0))
        print 'trans = ' + str(trans)
        print 'rot = ' + str(rot)
        print checkTThresh(trans,rot)
        print checkRThresh(trans,rot)
        r.sleep()

def main():
    rospy.init_node('dummy')
    ar_tags = {}
    for i in range(1,len(sys.argv)):
        ar_tags['ar'+str(i)] = 'ar_marker_' + sys.argv[i]
        # Tell the zumy to move
    command(ar_tags)
    rospy.spin()

if __name__ == '__main__':
    main()