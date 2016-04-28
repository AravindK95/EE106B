#!/usr/bin/env python
import sys
import rospkg
import numpy as np

PROJECT_PATH = rospkg.RosPack().get_path('grasper_plan')
sys.path.append(PROJECT_PATH+'/src')
import obj_file
import transformations

MESH_FILENAME = PROJECT_PATH+'/data/pencil.obj'
FC_DATA_FILENAME = PROJECT_PATH+'/data/points.csv'
GRASP_DATA_FILENAME = PROJECT_PATH+'/data/grasps.csv'

def check_collision(contact_center, vertices, hand_param):
    tol = 0.5
    aligned_vertices_z = []
    
    # create list of mesh points that have same x and y value as contact_center within a tolerance
    for i in vertices:
        if np.abs(i[0]-contact_center[0]) < tol and np.abs(i[1]-contact_center[1]) < tol:
            aligned_vertices_z.append(i[2])

    max_distance = max(aligned_vertices_z)
    if max_distance > hand_param['center_distance']:
        return False
    else:
        return True

def fc_to_hand_pose(contact1, contact2, object_mesh, hand_param):
    c1 = np.array(contact1)
    c2 = np.array(contact2)

    finger_center = (c1+c2)/2

    gripper_y_axis = c2 - c1 #align y axis to connect the two contacts
    
    #check to see if gripper can open wide enough to reach contact points
    contact_distance = np.linalg.norm(gripper_y_axis)
    if contact_distance > hand_param['max_open']:
        return None
    
    gripper_y_axis = gripper_y_axis / np.linalg.norm(gripper_y_axis) #normalize y axis
    
    reachable_grasps = []
    #checks which potential grasp orientations (front, top, back) around the y axis are valid
    thetas = [0, np.pi/2, np.pi, 3*np.pi/2]
    for i in thetas:
        gripper_z_axis = np.array([gripper_y_axis[1],-gripper_y_axis[0],0]) # z axis starts off parallel to table plane
        gripper_z_axis = gripper_z_axis / np.linalg.norm(gripper_z_axis) #normalize z axis
        gripper_x_axis = np.cross(gripper_y_axis, gripper_z_axis)
        
        #Construct RBT from axis and center
        gripper_Rot = np.concatenate((gripper_x_axis, gripper_y_axis, gripper_z_axis),, axis=0)
        gripper_RBT = np.concatenate((gripper_Rot, finger_center.T), axis = 1)
        gripper_RBT = np.append(gripper_RBT, np.array([0,0,0,1]), axis = 1)

        #Apply appropriate rotation transformation for orientation being tested
        applied_rotation = np.array([[np.cos(i), 0, np.sin(i), 0], [0, 1, 0, 0], [-np.sin(i), 0, np.cos(i), 0], [0,0,0,1]])
        gripper_RBT = gripper_RBT * applied_rotation

        # Only calculate possible grasps if Z axis does not point downwards
        if np.vdot(gripper_z_axis, np.array([0,0,1])) > -0.1:
            #transform all matrix points to the gripper frame 
            original_vertices = np.array(object_mesh.vertices)
            homogenous_vertices = np.append(original_vertices, np.ones(1,original_vertices.shape(0)))
            transformed_vertices = np.array()
            for j in homogeneous_vertices:
                transformed_vertices = np.append(transformed_verticies,np.dot(homogeneous_vertices, gripper_RBT))

            #Check collisions beetween the hand and the mesh
            if check_collision(finger_center, transformed_vertices, hand_param) == False:
                reachable_grasps.append(gripper_RBT)
    
    output_list = ()
    for i in reachable_grasps:
        output_list.append((i, c1, c2, contact_distance))
    return output_list


def main():
    hand_param = {'max_open': 4, 'center_distance': 2}
    # read from mesh
    of = obj_file.ObjFile(MESH_FILENAME)
    mesh = of.read()

    # file I/O stuff
    # call fc_to_hand_pose for every FC point
    # save list of all grasps
    in_f = open(FC_DATA_FILENAME, 'r')
    data = in_f.read()
    data = data.split('\n')
    #data = [i.split(';') for i in data]
    print data
    data.pop(0)

    # fc_points structure: list of tuples, with each tuple being one fc pair
    # (i, j, [xi, yi, zi], [xj, yj, zj])
    # i and j are 0 indexed
    fc_points = list()
    for row in data:
    	vals = row.split(';')
    	fc_points.append((vals.split(';')))
    in_f.close()

    ### WESLEY DOES MAGIC HERE ###
    # grasp_list structure: list of tuples, with each tuple being one grasp
    # (RBT, c1, c2, dist(c1, c2))
    grasp_list = list()

    for i in fc_points
        grasp_list.append(fc_to_hand_pose(i[2], i[3], mesh, hand_param))

    out_f = open(GRASP_DATA_FILENAME, 'w')
    out_f.write('rbt; c1; c2; dist\n')
    for g in grasp_list:
    	rbt, c1, c2, d = g
    	out_f.write(str(list(rbt.flatten())) + '; ')
    	out_f.write(str(list(c1.flatten())) + '; ')
    	out_f.write(str(list(c2.flatten())) + '; ')
    	out_f.write(str(d) + '\n')

    out_f.close()

if __name__ == '__main__':
    main()
 