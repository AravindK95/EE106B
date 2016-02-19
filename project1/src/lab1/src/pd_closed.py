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
    tthresh = .35
    for i in range(2):
        if np.abs(trans[i])>tthresh:
            return False
    return True

def checkRThresh(trans, rot):
    rthresh = .3
    if trans[0] > 0 and abs(trans[1]) < rthresh:
        return True
    return False

# Tell the zumy what to do
def command(ar_tags):
    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher("/zumy7a/cmd_vel", Twist, queue_size=10)
    zumyctrl_enable = rospy.Publisher("/zumy_ctrl/enable", Bool, queue_size=10)
    zumyctrl_setpoint = rospy.Publisher('/zumy_ctrl/setpoint', Float64, queue_size=10)
    # zumyctrl_setpoint_r = rospy.Publisher('/zumy_ctrl/r_setpoint', Float64, queue_size=10)

    zumyctrl_setpoint.publish(0.05)
    # zumyctrl_setpoint_l.publish(0.9)
    zumyctrl_enable.publish(False)
    pid_enabled = False

    r = rospy.Rate(10)
    targetidx = 1
    print 'Ready to Move'

    # while (target<len(ar_tags)):
    while targetidx < len(ar_tags) and not rospy.is_shutdown():
        while not rospy.is_shutdown():
            try:
                trans,rot = listener.lookupTransform('ar_marker_'+str(ar_tags[0]),
                                                     'ar_marker_'+str(ar_tags[targetidx]), 
                                                      rospy.Time(0))
                # print trans
            except:
                print 'Failed to get Transforms: '+str(ar_tags[targetidx])
                r.sleep()
                continue
            # FACE THE ARTAG
            print 'Transforms Acquired'
            if (checkRThresh(trans,rot)):
                if (checkTThresh(trans,rot)):
                    print 'Arrived'
                    if pid_enabled:
                        zumyctrl_enable.publish(False)
                        pid_enabled = False

                    zumy_vel.publish(zumy_stop)
                    targetidx += 1
                    break
                else:
                    print 'Translating'
                    if not pid_enabled:
                        # zumy_vel.publish(zumy_stop)
                        zumyctrl_enable.publish(True)
                        pid_enabled = True

                    # zumy_vel.publish(Twist(Vector3(0.15,0,0),Vector3(0,0,0))) #slowly go to the goal tag
            else:
                print 'Rotating'
                if pid_enabled:
                    zumyctrl_enable.publish(False)
                    pid_enabled = False

                zumy_vel.publish(Twist(Vector3(0,0,0),Vector3(0,0,-0.2))) #slowly turn to face the goal AR tag

            r.sleep()


        # GO TO AR TAG USING PID
        #zumyctrl_setpoint.publish(0.5)
        #zumyctrl_enable.publish(True)
        # while not rospy.is_shutdown():
        #     #thresh =

        #     if (checkTThresh(trans,rot)):
        #         zumyctrl_enable.publish(False)
        #         zumy_vel.publish(zumy_stop)
        #     else:
        #         zumy_vel.publish(Twist(trans,Vector3(0,0,0))) #slowly go to the goal tag
    rospy.spin()
    # stop the zumy
    zumy_vel.publish(zumy_stop)

def main():
    print 'Initializing zumy_ctrl'
    rospy.init_node('zumy_ctrl')
    print 'zumy_ctrl initialized'
    # THIS IS ONLY FROM POINT A -> B IN STRAIGHT LINE
    # DOES NOT DO THE TURNING STUFF!!
    # if len(sys.argv) < 3:
    #     print "Use: pd.py [AR tag number for Zumy][AR tag number for goal]"
    #     sys.exit()
    ar_tags = []
    for i in range(1,len(sys.argv)):
        ar_tags.append(sys.argv[i])
        # Tell the zumy to move
    print 'ar tags loaded'
    command(ar_tags)
    rospy.spin()

if __name__ == '__main__':
    main()