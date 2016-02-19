#!/usr/bin/env python
import tf
import sys
import math
import numpy as np
import rospy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int16
from geometry_msgs.msg import Transform, Vector3, Twist
# FOR CHANGING THE ORIENTATION OF THE ZUMY TO TAGS
# NOT IMPLEMENTED YET!!
# import exp_quat_func as eqf

listener = None
zumy_stop = (Vector3(0,0,0),Vector3(0,0,0))

# Check if Zumy is close enought to target position and rotation
def checkTThresh(trans, rot):
    tthresh = .1
    for i in range(0,2)
        if trans(i)>ttrhesh
            return False
    return True

def checkRThresh(trans, rot):
    rthresh = .1
    for i in range(0,2)
        if trans(i)>rtrhesh
            return False
    return True

# Tell the zumy what to do
def command(ar_tags):
    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher("/zumy5b/cmd_vel", Twist, queue_size=10)
    zumyctrl_enable = rospy.Publisher("/zumy_ctrl/enable", Bool, queue_size=10)
    zumyctrl_setpoint = rospy.Publisher("/zumy_ctrl/setpoint", Float64, queue_size=10)
    r = rospy.Rate(10)
    curr = 0
    target = 1

    while (target<len(ar_tags)):
        # FACE THE ARTAG
        while not rospy.is_shutdown():
            try:
                trans,rot = listener.lookupTransform(ar_tags['ar'+str(target)], ar_tags['ar'+str(curr)], rospy.Time(0))
            except:
                continue
              
            if (checkRThresh(trans,rot)):
                zumy_vel.publish(zumy_stop)
            else:
                zumy_vel.publish(Vector3(0,0,0),Vector3(0,0,.2)) #slowly turn to face the goal AR tag
                r.sleep()

        # GO TO AR TAG USING PID
        zumyctrl_setpoint.publish(0.5)
        zumyctrl_enable.publish(True)
        while not rospy.is_shutdown():
            #thresh = 

            if (checkTThresh(trans,rot)):
                zumyctrl_enable.publish(False)
                zumy_vel.publish(zumy_stop)
            else:
                zumy_vel.publish(Vector3(-0.8*trans[1],0,0),Vector3(0,0,0)) #slowly go to the goal tag

    target+=1

    # stop the zumy
    zumy_vel.publish(zumy_stop)

def main():
    rospy.init_node('zumy_ctrl')
    # THIS IS ONLY FROM POINT A -> B IN STRAIGHT LINE
    # DOES NOT DO THE TURNING STUFF!!
    # if len(sys.argv) < 3:
    #     print "Use: pd.py [AR tag number for Zumy][AR tag number for goal]"
    #     sys.exit()
    ar_tags = {}
    for i range(0,len(sys.argv)-1)
        ar_tags['ar'+str(i)] = 'ar_marker_' + sys.argv[i]
        # Tell the zumy to move
    command(ar_tags)
    rospy.spin()

if __name__ == '__main__':
    main()