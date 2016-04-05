#!/usr/bin/env python
import sys
import rospkg
import rospy
import tf
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Transform, Pose, Vector3, Quaternion, Point
from lab3.msg import FrameCall

PROJECT_PATH = rospkg.RosPack().get_path('lab3')
sys.path.append(PROJECT_PATH+'/src/lab3')
sys.path.append(PROJECT_PATH+'/src/extra')
SPRAY_BOTTLE_MESH_FILENAME = PROJECT_PATH+'/data/spray.obj'

import obj_file
import transformations
from lab3_starter import contacts_to_baxter_hand_pose

BASE = 'base'
OBJ_BASE = 'graspable_object'

def publish_frame_group(trans, rot, name, base, to_add):
    tf_pub.publish(Transform(Vector3(trans[0], trans[1], trans[2]), 
                             Quaternion(rot[0], rot[1], rot[2], rot[3])), 
                   name, 
                   base, 
                   to_add)
    #One of these is the correct direction to off set grap pos by.
    pre_trans = Vector3(trans[0] - 0.2, trans[1], trans[2])
    pre_rot = Quaternion(rot[0], rot[1], rot[2], rot[3])

    #One of these is the correct direction to lift it straight up. Probably z.
    post_trans = Vector3(trans[0], trans[1], trans[2] + 0.3)
    post_rot = Quaternion(rot[0], rot[1], rot[2], rot[3])
    #We want to the post orientation to be the same as the initial orientation during grasp
    #so we do not need to change orientation of end effector.

    #Publish the pre and post trans
    tf_pub.publish(Transform(pre_trans, pre_rot), 'pre'+name, base, to_add)
    tf_pub.publish(Transform(post_trans, post_rot), 'post'+name, base, to_add)

if __name__ == '__main__':
    of = obj_file.ObjFile(SPRAY_BOTTLE_MESH_FILENAME)
    mesh = of.read()

    vertices = mesh.vertices
    triangles = mesh.triangles
    normals = mesh.normals

    rospy.init_node('grasp_ctrl')

    tf_pub = rospy.Publisher('lab3/tf', FrameCall, queue_size=3)
    moveit_pub = rospy.Publisher('new_position', Pose, queue_size=3)
    claw_pub = rospy.Publisher('gripper_control', Bool, queue_size=3)

    tf_listener = tf.TransformListener()

    while not rospy.is_shutdown():
        # parse input
        inval = raw_input("cmd >> ")
        cmd = None
        try:
            inval = inval.split(' ')
            cmd = inval[0]
        except:
            print 'Bad input!'
            continue

        if cmd == 'addframe':
            # publish grasp frame
            """Example input: 
               $ cmd >> addframe (1,2,3) (4,5,6,7) child base
            """
            trans = eval(inval[1])  # XYZ vector
            rot = eval(inval[2])    # quaternion
            name = inval[3]
            base = inval[4]

            publish_frame_group(trans, rot, name, base, True)

        elif cmd == 'rmframe':
            # stop publishing grasp frame 
            """Example input: 
               $ cmd >> rmframe child
            """
            name = inval[1]

            # trans and rot values irrelevant
            publish_frame_group((0,0,0), (0,0,0,0), name, 'blah', False)

        elif cmd == 'moveto':
            # command moveit
            """Example input: 
               $ cmd >> moveto child
            """
            name = inval[1]
            (trans,rot) = tf_listener.lookupTransform(BASE, name, rospy.Time(0))
            moveit_pub.publish(Pose(Point(trans[0], trans[1], trans[2]),
                                    Quaternion(rot[0], rot[1], rot[2], rot[3])))

        elif cmd == 'setclaw':
            # command the end effector
            """Example input: 
               $ cmd >> setclaw True 
            """
            claw_bool = eval(inval[1])
            claw_pub.publish(claw_bool)

        elif cmd == 'makepose':
            # turn two force closure vertices into a tf frame
            """Example input:
               $ cmd >> makepose name 2473 2035
            """
            name = inval[1]
            idx1 = int(inval[2])
            idx2 = int(inval[3])
            trans,rot = contacts_to_baxter_hand_pose(vertices[idx1], vertices[idx2])
            trans = (trans[0], trans[1], trans[2])
            #rot = (rot[0], rot[1], rot[2], rot[3])
            rot = (0, np.sqrt(2)/2, 0, np.sqrt(2)/2)
            publish_frame_group(trans, rot, name, OBJ_BASE, True)

        else:
            print 'Bad command: '+inval[0]
