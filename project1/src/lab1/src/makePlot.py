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
import matplotlib.animation as animation
import time

def make_plot(title, setpoint):
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)

    def animate(i):
        data = open(title+".txt","r")
        dataArray = data.read().split('\n')
        xar = []
        yar = []
        for eachLine in dataArray:
            if len(eachLine)>1:
                y,x = eachLine.split(',')
                xar.append(float(x))
                yar.append(float(y))
        ax1.clear()
        ySetpoint = np.ones(len(xar)) * float(setpoint)
        ax1.plot(xar,yar)
        ax1.plot(xar,ySetpoint)
        data.close()

    ani = animation.FuncAnimation(fig, animate, interval=1000)
    plt.show()

def init():
    make_plot(sys.argv[1], sys.argv[2])

if __name__ == '__main__':
    init()