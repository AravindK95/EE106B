#!/usr/bin/env python  
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('object_pose_publisher')

    broadcaster = tf.TransformBroadcaster()
 
    R_ar_obj = np.eye(3)
    t_ar_obj = np.array([-0.4, -0.2, 0.10])
    #t_ar_obj = np.array([-0.4, -0.2, 0.])
    T_ar_obj = np.eye(4)
    T_ar_obj[:3,:3] = R_ar_obj
    T_ar_obj[:3, 3] = t_ar_obj
    q_ar_obj = tf.transformations.quaternion_from_matrix(T_ar_obj)

    r_gripper_obj = np.eye(3)
    t_gripper_obj = np.array([0., 0., 0.05])
    T_gripper_obj = np.eye(4)
    T_gripper_obj[:3,:3] = r_gripper_obj
    T_gripper_obj[:3,3] = t_gripper_obj
    q_gripper_obj = tf.transformations.quaternion_from_matrix(T_gripper_obj)
    
    print 'Publishing object pose'
    
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            broadcaster.sendTransform(t_ar_obj, q_ar_obj, rospy.Time.now(), '/graspable_object', '/ar_marker_3')
            broadcaster.sendTransform(t_gripper_obj, q_gripper_obj, rospy.Time.now(), '/left_gripper_endpoint', '/left_gripper')
        except:
            continue
        rate.sleep()
