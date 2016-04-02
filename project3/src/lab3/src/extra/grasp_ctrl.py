#!/usr/bin/env python
import sys
import rospkg
import rospy
import tf
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from lab3.msg import FrameCall

PROJECT_PATH = rospkg.RosPack().get_path('lab3')
sys.path.append(PROJECT_PATH+'/src/lab3')
sys.path.append(PROJECT_PATH+'/src/extra')

import transformations

def publish_frame_group(trans, rot, name, base, to_add):
    tf_pub.publish(Transform(trans, rot), name, base, True)

    ### TODO: calculate pre- and post- trans & rot values
    tf_pub.publish(Transform(pre_trans, pre_rot), 'pre'+name, base, True)
    tf_pub.publish(Transform(post_trans, post_rot), 'post'+name, base, True)


if __name__ == '__main__':
    rospy.init_node('grasp_ctrl')
    tf_pub = rospy.Publisher('lab3/tf', FrameCall, queue_size=3)
    moveit_pub = rospy.Publisher('lab3/moveit', String, queue_size=3)

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
            # publish grasp frame
            """Example input: 
               $ cmd >> addframe (1,2,3) (4,5,6,7) child base
            """
            trans = eval(inval[1])  # XYZ vector
            rot = eval(inval[2])    # quaternion
            name = inval[3]
            base = inval[4]

            publish_frame_group(trans, rot, name, base, True)

        elif cmd == 'rmframe':
            # stop publishing grasp frame 
            """Example input: 
               $ cmd >> rmframe child
            """
            name = inval[1]

            # trans and rot values irrelevant
            publish_frame_group((0,0,0), (0,0,0,0), name, 'blah', False)

        elif cmd == 'moveto':
            # command moveit
            """Example input: 
               $ cmd >> moveto child
            """
            name = inval[1]

            ## TODO: figure out ingterface with moveit ctrl. Or just do
            ## moveit control in this file
            moveit_pub.publish(name)

        else:
            print 'Bad command: '+inval[0]