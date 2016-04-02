#!/usr/bin/env python
import sys
import rospkg
import rospy
import tf
import numpy as np
from lab3.msg import FrameCall
from geometry_msgs.msg import Transform

PROJECT_PATH = rospkg.RosPack().get_path('lab3')
sys.path.append(PROJECT_PATH+'/src/lab3')
sys.path.append(PROJECT_PATH+'/src/extra')

import transformations

DUMMY_TRANSFORM = Transform((0,0,0), (0,0,0,0))

if __name__ == '__main__':
    rospy.init_node('grasp_ctrl')
    tf_pub = rospy.Publisher('lab3/tf', FrameCall, queue_size=3)
    # moveit_pub = rospy.Publisher('lab3/moveit', SOMEMSGTYPE, queue_size=3)

    while not rospy.is_shutdown():
        # parse input
        inval = raw_input("cmd >> ")
        cmd = None
        try:
            inval = inval.split(' ')
            cmd = inval[0]
        except:
            print 'Bad input!'
            continue

        if cmd == 'addframe':
            # publish frame
            """Example input: 
               $ cmd >> addframe (1,2,3) (4,5,6,7) child base
            """
            trans = eval(inval[1])  # XYZ vector
            rot = eval(inval[2])    # quaternion
            name = inval[3]
            base = inval[4]

            tf_pub.publish(Transform(trans, rot), name, base, True)

        elif cmd == 'rmframe':
            # stop publishing frame 
            """Example input: 
               $ cmd >> rmframe child
            """
            name = inval[1]
            tf_pub.publish(DUMMY_TRANSFORM, name, 'blah', False)

        elif cmd == 'moveto':
            # command moveit
            """Example input: 
               $ cmd >> moveto child
            """
            name = inval[1]
            # moveit_pub.publish(name)

        else:
            print 'Bad command: '+inval[0]
