#!/usr/bin/env python
from __future__ import division
import sys
import rospy
import roslib
import time
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

def writer(title, msg):
    file = title + ".txt"
    save = open(file, "aw")
    save.write(str(msg.linear_acceleration.x)+','+str(msg.linear_acceleration.y)+ ',' + str(msg.angular_velocity.z) + ',' +str(rospy.Time.now().to_sec()))
    save.write('\n')
    save.close()

def saveIMU(msg):
    writer('imu_data'+str(sys.argv[1]), msg)

def main(name):
    rospy.init_node('imu_plotter')
    # r = rospy.Rate(10)
    # while not rospy.is_shutdown():
        
    rospy.Subscriber('/zumy7a/imu', Imu, saveIMU)
    rospy.spin()
    # r.sleep()


main(sys.argv[1])