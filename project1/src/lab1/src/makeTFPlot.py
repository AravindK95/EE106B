#!/usr/bin/env python
import tf
import sys
import math
import numpy as np
import rospy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import *
from geometry_msgs.msg import Transform, Vector3, Twist

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

def make_plot(name):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    f = open(name+'.txt', 'r')
    data = f.read()
    data = data.split('\n')

    x = np.zeros(len(data))
    y = np.zeros(len(data))
    z = np.zeros(len(data))

    for i in xrange(len(data)-1):
        print data[i]
        data_tuple = eval(data[i])
        if np.linalg.norm(data_tuple) > 10:
            continue
        x[i] = data_tuple[0]
        y[i] = data_tuple[1]
        z[i] = data_tuple[2]

    f.close()

    ax.scatter(x, y, z)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
    rospy.spin()


make_plot(sys.argv[1])