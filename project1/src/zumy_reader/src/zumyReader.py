#!/usr/bin/env python
import sys
import rospy
import roslib
import time
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *

global ctrl_output
global encoder_output

ctrl_output = [0.0, 0.0]
encoder_output = [0, 0]

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


def make_differentiator(pub, N, scale, idx):
    posFilter = RingBuffer(N)

    def differentiator(msg):
        global encoder_output
        encoder_output[idx] = msg.data

        metersPerCount = 0.038*np.pi/600 #0.038 m diameter of treads * pi / 600 encoder counts per revolution
        encoderCount = scale*msg.data
        metersTraveled = encoderCount*metersPerCount

        posFilter.push((metersTraveled, rospy.Time.now().to_sec()))
        if not posFilter.filled():
            return

        encoderVals = posFilter.get()
        velocityEst = 0
        # This could be better filtered
        for i in range(len(encoderVals) - 1):
            velocityEst += (encoderVals[i+1][0]-encoderVals[i][0])/(encoderVals[i-1]-encoderVals[i])
        velocityEst /= (len(encoderVals) - 1)

        pub.publish(velocityEst)

    return differentiator

def make_setpoint_listener(l_pub, r_pub):

    def setpoint_listener(msg):
        global drift

        left_setpoint = msg.data
        right_setpoint = msg.data

        l_pub.publish(left_setpoint)
        r_pub.publish(right_setpoint)

    return setpoint_listener

def make_ctrl_listener(idx):
    def ctrl_listener(msg):
        global ctrl_output
        ctrl_output[idx] = msg.data

    return ctrl_listener

def make_enable_listener(l_pub, r_pub):
    
    def enable_listener(msg):
        l_pub.publish(msg.data)
        r_pub.publish(msg.data)

# setpoint: Float64 (target velocity + drift correction)
# state: Float64    (zumy velocity - differentiate encoders)
# ctrl_effort: Float64 (pid output)
# pid_enable: Bool     (duh)

def init():
    global ctrl_output
    global encoder_output

    print("Initializing ZumyReader... ")
    rospy.init_node("zumy_reader")

    print("Initializing ZumyReader publishers... ")
    left_state_pub = rospy.Publisher('/l_pid/state', Float64, queue_size=10)
    right_state_pub = rospy.Publisher('/r_pid/state', Float64, queue_size=10)

    left_setpt_pub = rospy.Publisher('/l_pid/setpoint', Float64, queue_size=10)
    right_setpt_pub = rospy.Publisher('/r_pid/setpoint', Float64, queue_size=10)

    left_enable_pub = rospy.Publisher('/l_pid/enable', Bool, queue_size=10)
    right_enable_pub = rospy.Publisher('/r_pid/enable', Bool, queue_size=10)

    motor_pub = rospy.Publisher('/zumy7a/cmd_vel', Twist, queue_size=10)

    print("Initializing ZumyReader subscribers... ")
    # Convert encoder ticks to velocity, pass to PID
    rospy.Subscriber('/zumy7a/l_enc', Int16, make_differentiator(left_state_pub, 5, 1, 0))
    rospy.Subscriber('/zumy7a/r_enc', Int16, make_differentiator(right_state_pub, 5, -1, 1))

    # Pass enable flag along to PID directly
    rospy.SubscriberI('/zumy_ctrl/enable', Bool, make_enable_listener(left_enable_pub, right_enable_pub))

    # Pass setpoint to PID
    rospy.Subscriber('/zumy_ctrl/setpoint', Float64, make_setpoint_listener(left_setpt_pub, right_setpt_pub))

    # Record controller effort
    rospy.Subscriber('/l_pid/controller_effort', Float64, make_ctrl_listener(0))
    rospy.Subscriber('/r_pid/controller_effort', Float64, make_ctrl_listener(1))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # How to convert motor velocities to twist? Have an idea if we can bypass PID....
        trans = (ctrl_output[0], 0, 0)
        rot = (0, 0, 0)

        motor_pub.publish((trans, rot))
        rate.sleep(rate)

if __name__ == '__main__':
    init()