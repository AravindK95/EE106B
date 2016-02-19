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

global is_enabled

ctrl_output = [0.0, 0.0]
is_enabled = False

def make_differentiator(pub, N, scale, idx):
    posFilter = RingBuffer(N)

    def differentiator(msg):
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
        print 'velocity =' + str(velocityEst)
        pub.publish(velocityEst)

    return differentiator

def make_setpoint_listener(pub):

    def setpoint_listener(msg):
        setpoint = msg.data
        print("setpoint is: "+str(setpoint))

        pub.publish(setpoint)

    return setpoint_listener

def make_ctrl_listener(pub):

    def ctrl_listener(msg):
        global is_enabled
        if is_enabled:
            effort = msg.data
            print(msg.data)
            pub.publish(effort)

    return ctrl_listener

def make_enable_listener(l_pub, r_pub, l_motor, r_motor):
    
    def enable_listener(msg):
        global is_enabled
        print("Robot enable: "+str(msg.data))

        l_pub.publish(msg.data)
        r_pub.publish(msg.data)

        if msg.data:
            is_enabled = True
        else:
            is_enabled = False
            l_motor.publish(0.0)
            r_motor.publish(0.0)


    return enable_listener

# setpoint: Float64 (target velocity + drift correction)
# state: Float64    (zumy velocity - differentiate encoders)
# ctrl_effort: Float64 (pid output)
# pid_enable: Bool     (duh)

def vel_to_twist(left, right):
    trans_x = (right+left)/1.2
    rot_z =  (right-left)/0.8
    return Twist(Vector3(trans_x,0,0), Vector3(0,0,rot_z))

def init():
    print("Initializing ZumyReader... ")
    rospy.init_node("zumy_reader")

    print("Initializing ZumyReader publishers... ")
    left_state_pub = rospy.Publisher('/l_pid/state', Float64, queue_size=10)
    right_state_pub = rospy.Publisher('/r_pid/state', Float64, queue_size=10)

    left_setpt_pub = rospy.Publisher('/l_pid/setpoint', Float64, queue_size=10)
    right_setpt_pub = rospy.Publisher('/r_pid/setpoint', Float64, queue_size=10)

    left_enable_pub = rospy.Publisher('/l_pid/pid_enable', Bool, queue_size=10)
    right_enable_pub = rospy.Publisher('/r_pid/pid_enable', Bool, queue_size=10)

    motor_pub_left = rospy.Publisher('/zumy7a/cmd_left', Float32, queue_size=10)
    motor_pub_right = rospy.Publisher('/zumy7a/cmd_right', Float32, queue_size=10)
    # twist_pub = rospy.Publisher('/zumy7a/cmd_vel', Twist, queue_size=10)


    print("Initializing ZumyReader subscribers... ")
    # Convert encoder ticks to velocity, pass to PID
    rospy.Subscriber('/zumy7a/l_enc', Int32, make_differentiator(left_state_pub, 5, 1, 0))
    rospy.Subscriber('/zumy7a/r_enc', Int32, make_differentiator(right_state_pub, 5, -1, 1))

    # Pass enable flag along to PID directly
    rospy.Subscriber('/zumy_ctrl/enable', Bool, make_enable_listener(left_enable_pub, right_enable_pub, motor_pub_left, motor_pub_right))

    # Pass setpoint to PID
    rospy.Subscriber('/zumy_ctrl/l_setpoint', Float64, make_setpoint_listener(left_setpt_pub))
    rospy.Subscriber('/zumy_ctrl/r_setpoint', Float64, make_setpoint_listener(right_setpt_pub))

    # Record controller effort
    rospy.Subscriber('/l_pid/control_effort', Float64, make_ctrl_listener(motor_pub_left))
    rospy.Subscriber('/r_pid/control_effort', Float64, make_ctrl_listener(motor_pub_right))

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        # print("ctrl_effort: "+str(ctrl_output))
        # How to convert motor velocities to twist? Have an idea if we can bypass PID....
        # if is_enabled:
        #     cmd = vel_to_twist(ctrl_output[0], ctrl_output[1])
        #     motor_pub.publish(cmd)
        # if is_enabled:
        #      motor_pub_left.publish(ctrl_output[0])
        #      motor_pub_right.publish(ctrl_output[1])
        
        rate.sleep()

if __name__ == '__main__':
    init()