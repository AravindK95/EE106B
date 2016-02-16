#!/usr/bin/env python
import sys
import rospy
import roslib
import time

import numpy as np

from std_msgs.msg import *

class RingBuffer():
    def __init__(self, size):
        self.size = size
        self.data = [0.0 for _ in range(size)]
        self.ptr = 0
        self.isFull = False

    def push(self, item):
        self.data[self.ptr] = item
        self.ptr = (self.ptr+1)%self.size
        if not self.isFull and self.ptr == 0:
            self.isFull = True

    # get() does not guarantee FIFO-ordered output
    def get(self):
        if self.isFull:
            return self.data
        else:
            return self.data[:self.ptr]

    def clear(self):
        self.isFull = False
        self.ptr = 0

    def filled(self):
        return self.isFull

    def getAvg(self):
        return np.average(self.get())


def make_differentiator(pub, N, scale):
    posFilter = RingBuffer(N)
    inClock = RingBuffer(N)

    def callback(data):
        #run every time i see a TFMessage on tf_topic.
        metersPerCount = 0.038*np.pi/600 #0.038 m diameter of treads * pi / 1200 encoder counts per revolution
        encoderCount = scale*data.data
        metersTraveled = encoderCount*metersPerCount

        posFilter.push(metersTraveled)
        inClock.push(rospy.Time.now().to_sec())
        if not posFilter.filled():
            return

        velocityEst = 0
        # This could be better filtered + more readable
        for i in range(len(posFilter.get())-1):
            velocityEst = velocityEst + (posFilter.get()[i+1]-posFilter.get()[i])/(inClock.get()[i+1]-inClock.get()[i])
        velocityEst = velocityEst / (len(posFilter.get()) - 1)

        pub.publish(velocityEst)

    return callback

def make_pointsetter(pub):

    def pointsetter(data):


    return pointsetter

# setpoint: Float64 (target velocity + drift correction)
# state: Float64    (zumy velocity - differentiate encoders)
# ctrl_effort: Float64 (pid output)
# pid_enable: Bool     (duh)


def init():
    print("Initializing ZumyReader... ")
    rospy.init_node("zumy_reader")

    print("Initializing ZumyReader publishers... ")
    left_state_pub = rospy.Publisher('l_pid/state', Float64, queue_size=10)
    right_state_pub = rospy.Publisher('r_pid/state', Float64, queue_size=10)

    left_setpt_pub = rospy.Publisher('l_pid/setpoint', Float64, queue_size=10)
    right_setpt_pub = rospy.Publisher('r_pid/setpoint', Float64, queue_size=10)

    print("Initializing ZumyReader subscribers... ")
    rospy.Subscriber('/zumy7a/l_enc', Int16, make_differentiator(left_state_pub, 5, 1))
    rospy.Subscriber('/zumy7a/r_enc', Int16, make_differentiator(right_state_pub, 5, -1))

    rospy.Subscriber('/zumy_ctrl/setpoint', Float64, make_pointsetter())


if __name__ == '__main__':
    init()