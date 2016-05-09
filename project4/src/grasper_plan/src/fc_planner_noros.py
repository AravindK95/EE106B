#!/usr/bin/env python
import sys
import numpy as np

PROJECT_PATH = '/home/arak/git/ee106b/project4/src/grasper_plan'
MESH_FILENAME = PROJECT_PATH+'/data/pawn_lowpoly.obj'

import obj_file
import transformations
import fc


def contacts_to_baxter_hand_pose(contact1, contact2):
    c1 = np.array(contact1)
    c2 = np.array(contact2)

    # compute gripper center and axis
    center = 0.5 * (c1 + c2)
    y_axis = c2 - c1
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.array([y_axis[1], -y_axis[0], 0]) # the z axis will always be in the table plane for now
    z_axis = z_axis / np.linalg.norm(z_axis)
    x_axis = np.cross(y_axis, z_axis)

    # convert to hand pose
    R_obj_gripper = np.array([x_axis, y_axis, z_axis]).T
    t_obj_gripper = center
    T_obj_gripper = np.eye(4)
    T_obj_gripper[:3,:3] = R_obj_gripper
    T_obj_gripper[:3,3] = t_obj_gripper
    q_obj_gripper = transformations.quaternion_from_matrix(T_obj_gripper)

    return t_obj_gripper, q_obj_gripper 

if __name__ == '__main__':
    of = obj_file.ObjFile(MESH_FILENAME)
    mesh = of.read()
    mesh.tri_normals()

    vertices = mesh.tri_centers()
    triangles = mesh.triangles
    # Manually flip normals based on what gives results
    normals = mesh.normals
    # normals = [[-n[0], -n[1], -n[2]] for n in mesh.normals]

    print 'Num vertices:', len(vertices)
    print 'Num triangles:', len(triangles)
    print 'Num normals:', len(normals)

    # 1. Generate candidate pairs of contact points
    pairs = []
    c = 0
    print "Generating point-pairs"
    for i in range(len(vertices)):
        for j in range(i+1, len(vertices)):
            pairs.append((i, j))
            print c
            c += 1

    print "Total pairs: "+str(len(pairs))

    # 2. Check for force closure
    print "Checking for force closure ..."
    c = np.zeros((3, 2))
    n = np.zeros((3, 2))
    successful = []
    for i, j in pairs:
        print(i, j)
        v1, v2 = vertices[i], vertices[j]
        n1, n2 = normals[i], normals[j]

        c[:, 0] = v1
        c[:, 1] = v2
        n[:, 0] = n1
        n[:, 1] = n2

        retval = fc.force_closure(c, n, 0, 0.6, 0)
        if retval: successful.append((i, j))

    print "Found pairs: "
    print successful
    of = open(PROJECT_PATH+'/data/pawn_points.csv', 'w')
    of.write('i; j; ival; jval\n')
    for i, j in successful:
        of.write(str(i)+'; '+str(j)+'; ')
        of.write(str(vertices[i])+'; '+str(vertices[j])+'\n')
    of.close()

    # 3. Convert each grasp to a hand pose
    # contact1 = vertices[2777]
    # contact2 = vertices[2805]
    # t_obj_gripper, q_obj_gripper = contacts_to_baxter_hand_pose(contact1, contact2)
    # print 'Translation', t_obj_gripper
    # print 'Rotation', q_obj_gripper

    # 4. Execute on the actual robot
