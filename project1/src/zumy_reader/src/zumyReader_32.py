#!/usr/bin/env python
from __future__ import division
import sys
import rospy
import roslib
import time
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *

zumyStop = Twist(Vector3(0,0,0), Vector3(0,0,0))
metersPerCount = 0.038*np.pi/600 #0.038 m diameter of treads * pi / 600 encoder counts per revolution
DRIFT_SCALE = 0.0000005

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

global ctrl_output
global encoder_output
global is_enabled
global drift_left

ctrl_output = [0.0, 0.0]
encoder_output = [RingBuffer(3), RingBuffer(3)]
is_enabled = False
drift_left = 0.0

def make_differentiator(pub, N, scale, idx):
    posFilter = RingBuffer(N)

    def differentiator(msg):
        global encoder_output
        global drift_left

        # Drift correction
        encoder_output[idx].push(scale*msg.data)
        if encoder_output[0].filled() and encoder_output[1].filled():
        	drift_left = encoder_output[0].getAvg() - encoder_output[1].getAvg()

        # ticks --> meters conversion
        metersTraveled = scale*msg.data*metersPerCount

        posFilter.push((metersTraveled, rospy.Time.now().to_sec()))
        if not posFilter.filled():
            return

        # Velocity calculation
        encoderVals = posFilter.get()
        velocityEst = 0
        # This could be better filtered
        for i in range(len(encoderVals) - 1):
            velocityEst += (encoderVals[i+1][0]-encoderVals[i][0])/(encoderVals[i-1][1]-encoderVals[i][1])
        velocityEst /= (len(encoderVals) - 1)
        pub.publish(velocityEst)

    return differentiator

def make_setpoint_listener(l_pub, r_pub):

    def setpoint_listener(msg):
        print("setpoint is: "+str(msg.data))

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

def make_enable_listener(l_pub, r_pub, motor_pub):
    
    def enable_listener(msg):
        global is_enabled
        print("Robot enable: "+str(msg.data))

        l_pub.publish(msg.data)
        r_pub.publish(msg.data)

        if msg.data:
            is_enabled = True
        else:
            is_enabled = False
            motor_pub.publish(zumyStop)

    return enable_listener

# setpoint: Float64 (target velocity + drift correction)
# state: Float64    (zumy velocity - differentiate encoders)
# ctrl_effort: Float64 (pid output)
# pid_enable: Bool     (duh)

def init():
    global ctrl_output
    global encoder_output
    global is_enabled
    global drift_left

    print("Initializing ZumyReader... ")
    rospy.init_node("zumy_reader")

    print("Initializing ZumyReader publishers... ")
    left_state_pub = rospy.Publisher('/l_pid/state', Float64, queue_size=10)
    right_state_pub = rospy.Publisher('/r_pid/state', Float64, queue_size=10)

    left_setpt_pub = rospy.Publisher('/l_pid/setpoint', Float64, queue_size=10)
    right_setpt_pub = rospy.Publisher('/r_pid/setpoint', Float64, queue_size=10)

    left_enable_pub = rospy.Publisher('/l_pid/pid_enable', Bool, queue_size=10)
    right_enable_pub = rospy.Publisher('/r_pid/pid_enable', Bool, queue_size=10)

    motor_pub = rospy.Publisher('/zumy7a/cmd_vel', Twist, queue_size=10)

    print("Initializing ZumyReader subscribers... ")
    # Convert encoder ticks to velocity, pass to PID
    rospy.Subscriber('/zumy7a/l_enc', Int32, make_differentiator(left_state_pub, 3, 1, 0))
    rospy.Subscriber('/zumy7a/r_enc', Int32, make_differentiator(right_state_pub, 3, -1, 1))

    # Pass enable flag along to PID directly
    rospy.Subscriber('/zumy_ctrl/enable', Bool, make_enable_listener(left_enable_pub, right_enable_pub, motor_pub))

    # Pass setpoint to PID
    rospy.Subscriber('/zumy_ctrl/setpoint', Float64, make_setpoint_listener(left_setpt_pub, right_setpt_pub))

    # Record controller effort
    rospy.Subscriber('/l_pid/control_effort', Float64, make_ctrl_listener(0))
    rospy.Subscriber('/r_pid/control_effort', Float64, make_ctrl_listener(1))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # How to convert motor velocities to twist? Have an idea if we can bypass PID....
        if is_enabled:
            print("ctrl_effort: "+str(ctrl_output))
            # print("drift: "+str(drift_left*DRIFT_SCALE))
            trans = Vector3(ctrl_output[0], 0, 0)
            rot = Vector3(0, 0, drift_left*DRIFT_SCALE)
            motor_pub.publish(Twist(trans, rot))

        rate.sleep()

if __name__ == '__main__':
    init()