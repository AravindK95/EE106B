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
	#checks which potential grasp orientations (front, top, back) around the y axis are valid
	thetas = [-np.pi/2, 0, np.pi/2]
	for i in range thetas:
		gripper_z_axis = np.array([gripper_y_axis[1],-gripper_y_axis[0],0]) # z axis starts off parallel to table plane
		gripper_x_axis = np.cross(gripper_y_axis, gripper_z_axis)
		
		#Construct Rigid Body Transform from axis and center
		gripper_RBT = np.concatenate((gripper_x_axis, gripper_y_axis, gripper_z_axis, finger_center), axis=0).T
		gripper_RBT = np.append(gripper_RBT, np.array([0,0,0,1]), axis = 1)

		#Apply appropriate rotation transformation for orientation being tested
		applied_rotation = np.array([[np.cos(i), -np.sin(i), 0, 0], [np.sin(i) -np.cos(i), 0, 0], [0,0,1,0], [0,0,0,1]])
		gripper_RBT = gripper_RBT * applied_rotation

		#Check distance from back of hand to the bounding box



if __name__ == '__main__':
	main()
