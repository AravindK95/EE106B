#!/usr/bin/env python
from __future__ import division
import sys
import rospy
import roslib
import time
import numpy as np
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int16, Bool, Float64
from geometry_msgs.msg import Transform, Vector3, Twist

import tf
from tf2_msgs.msg import *


def main(name):
    rospy.init_node('tf_plotter')
    listener = tf.TransformListener()
    r = rospy.Rate(10)
    f = open('tf_data'+str(name)+'.txt', 'w')

    while not rospy.is_shutdown():
        try:
            trans, rot = listener.lookupTransform('ar_marker_1', 'ar_marker_0', rospy.Time(0))
        except:
            print "failed to get transform"
            r.sleep()
            continue

        f.write(str(trans))
        f.write('\n')

        r.sleep()



main(sys.argv[1])