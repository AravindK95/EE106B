#!/usr/bin/env python
import sys
import math
import numpy as np
import rospy
import time
from std_msgs.msg import *
from geometry_msgs.msg import *
from dynamixel_msgs.msg import *


def saver(title, msg):
    file = title + ".csv"
    save = open(file, "aw")
    timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs/float(1e9)
    goal_pos = msg.goal_pos
    curr_pos = msg.current_pos
    vel = msg.velocity
    load = msg.load
    save.write(str(goal_pos)+','+str(curr_pos)+','+str(vel)+','+str(load)+','+str(timestamp))
    save.write('\n')
    save.close()

def saveFinger1(msg):
    saver("f1", msg)

def saveFinger2(msg):
    saver("f2", msg)

def saveFinger3(msg):
    saver("f3", msg)

def main():
    f1 = open('f1.csv', 'w')
    f2 = open('f2.csv', 'w')
    f3 = open('f3.csv', 'w')
    f1.write("goal_position, current_position, velocity, load, timestamp")
    f1.write('\n')
    f2.write("goal_position, current_position, velocity, load, timestamp")
    f2.write('\n')
    f3.write("goal_position, current_position, velocity, load, timestamp")
    f3.write('\n')
    f1.close()
    f2.close()
    f3.close()

    rospy.init_node('plotter')

    rospy.Subscriber('/reflex_sf_f1/state', JointState, saveFinger1)
    rospy.Subscriber('/reflex_sf_f2/state', JointState, saveFinger2)
    rospy.Subscriber('/reflex_sf_f3/state', JointState, saveFinger3)

    rospy.spin()

if __name__ == '__main__':
    main()