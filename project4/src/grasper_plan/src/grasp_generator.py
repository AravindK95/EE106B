#!/usr/bin/env python
import sys
import rospkg
import numpy as np
import obj_file
import transformations

def check_collision(contact_center, verticies, hand_param):
    tol = 0.5
    aligned_verticies_z = []
    
    # create list of mesh points that have same x and y value as contact_center within a tolerance
    for i in verticies:
        if np.abs(i[0]-contact_center[0]) < tol and np.abs(i[1]-contact_center[1]) < tol:
            aligned_verticies_z.append(i[2])

    max_distance = max(aligned_verticies_z)
    if max_distance > hand_param['center_distance']:
        return False
    else
        return True

def fc_to_hand_pose(contact1, contact2, object_mesh, hand_param):
    c1 = np.array(contact1)
    c2 = np.array(contact2)

    finger_center = (c1+c2)/2

    gripper_y_axis = c2 - c1#align y axis to connect the two contacts
    
    #check to see if gripper can open wide enough to reach contact points
    contact_distance = np.linalg.norm(gripper_y_axis)
    if contact_distance > hand_param['max_open']:
        return None
    
    gripper_y_axis = gripper_y_axis / np.linalg.norm(gripper_y_axis) #normalize y axis
    
    reachable_grasps = []
    #checks which potential grasp orientations (front, top, back) around the y axis are valid
    thetas = [0, np.pi/2, np.pi]
    for i in range thetas:
        gripper_z_axis = np.array([gripper_y_axis[1],-gripper_y_axis[0],0]) # z axis starts off parallel to table plane
        gripper_z_axis = gripper_z_axis / np.linalg.norm(gripper_z_axis) #normalize z axis
        gripper_x_axis = np.cross(gripper_y_axis, gripper_z_axis)
        
        #Construct RBT from axis and center
        gripper_RBT = np.concatenate((gripper_x_axis, gripper_y_axis, gripper_z_axis, finger_center), axis=0).T
        gripper_RBT = np.append(gripper_RBT, np.array([0,0,0,1]), axis = 1)

        #Apply appropriate rotation transformation for orientation being tested
        applied_rotation = np.array([[np.cos(i), 0, np.sin(i), 0], [0, 1, 0, 0], [-np.sin(i), 0, np.cos(i), 0], [0,0,0,1]])
        gripper_RBT = gripper_RBT * applied_rotation

        #transform all matrix points to the gripper frame 
        original_verticies = np.array(object_mesh.verticies)
        homogenous_verticies = np.append(original_verticies, np.ones((,original_verticies.shape(0)), axis = 1))
        transformed_verticies = np.dot(homogeneous_verticies, gripper_RBT)

        #Check collisions beetween the hand and the mesh
        if check_collision(finger_center, transformed_verticies,hand_param) = False:
            reachable_grasps.append(gripper_RBT)
    output_list = ()
    
    for i in reachable_grasps:
        output_list.append((i, c1, c2, contact_distance))
    return output_list


def main():
    hand_param = {'max_open' = 4, 'center_distance' = 2}
    # file I/O stuff
    # call fc_to_hand_pose for every FC point
    # save list of all grasps


if __name__ == '__main__':
    main()
