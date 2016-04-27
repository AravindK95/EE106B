#!/usr/bin/env python
import sys
import rospkg
import numpy as np
import obj_file
import transformations

def hand_param():


def check_collision(object, hand_param):



def fc_to_hand_pose(contact1, contact2, hand_param):
	c1 = np.array(contact1)
	c2 = np.array(contact2)

	finger_center = (c1+c2)/2

	gripper_y_axis = c2 - c1 # align y axis to connect the two contacts
	
	#check to see if gripper can open wide enough to reach contact points
	contact_distance = np.linalg.norm(gripper_y_axis)
	if contact_distance > hand_param['max_open']:
		return None
	
	gripper_y_axis = gripper_y_axis / np.linalg.norm(gripper_y_axis) #normalize y axis
	
	reachable_grasps = []
	#loop checks if circle of potential grasps around the y axis are valid
	number_of_grasps = 24
	for i in range (0, number_of_grasps-1):
		theta = np.pi / 12 * i
		gripper_z_axis = np.array ([0,0,1])* #some sort of rotation matrix in theta
		gripper_x_axis = np.cross(gripper_y_axis, gripper_z_axis)


if __name__ == '__main__':
	main()
