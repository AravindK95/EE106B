#!/usr/bin/env python
import sys
import math
import numpy as np
import rospy
import time
from std_msgs.msg import *
from geometry_msgs.msg import *


def saver(title, msg):
    file = title + ".txt"
    save = open(file, "aw")
    save.write(str(msg.data)+','+str(time.time()))
    save.write('\n')
    save.close()

def saveLeftData(msg):
    saver("left", msg)

def saveRightData(msg):
    saver("right", msg)

def saveLeftVelData(msg):
    saver("leftVel", msg)

def saveRightVelData(msg):
    saver("rightVel", msg)

def main():
    f1 = open('left.txt', 'w')
    f2 = open('right.txt', 'w')
    f3 = open('leftVel.txt', 'w')
    f4 = open('rightVel.txt', 'w')
    f1.close()
    f2.close()
    f3.close()
    f4.close()

    rospy.init_node('plotter')

    rospy.Subscriber('/l_pid/control_effort', Float64, saveLeftData)
    rospy.Subscriber('/r_pid/control_effort', Float64, saveRightData)
    rospy.Subscriber('/zumy7a/cmd_left', Float32, saveLeftVelData)
    rospy.Subscriber('/zumy7a/cmd_right', Float32, saveRightVelData)

    rospy.spin()

if __name__ == '__main__':
    main()