#!/usr/bin/env python
import sys
import rospkg
import rospy
import tf
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Transform, Pose, Vector3, Quaternion, Point
from lab3.msg import FrameCall

PROJECT_PATH = rospkg.RosPack().get_path('grasper_plan')
GRASP_DB_FILENAME = PROJECT_PATH+'/data/sorted.csv'
sys.path.append(PROJECT_PATH+'/src')

# import obj_file
import transformations

BASE = 'base'
OBJ_BASE = 'graspable_object'

def publish_frame_group(trans, rot, name, base, to_add):
    tf_pub.publish(Transform(Vector3(trans[0], trans[1], trans[2]), 
                             Quaternion(rot[0], rot[1], rot[2], rot[3])), 
                   name, 
                   base, 
                   to_add)
    #One of these is the correct direction to off set grap pos by.
    # pre_trans = Vector3(trans[0] - 0.2, trans[1], trans[2])
    # pre_rot = Quaternion(rot[0], rot[1], rot[2], rot[3])

    #One of these is the correct direction to lift it straight up. Probably z.
    # post_trans = Vector3(trans[0], trans[1], trans[2] + 0.3)
    # post_rot = Quaternion(rot[0], rot[1], rot[2], rot[3])
    #We want to the post orientation to be the same as the initial orientation during grasp
    #so we do not need to change orientation of end effector.

    #Publish the pre and post trans
    # tf_pub.publish(Transform(pre_trans, pre_rot), 'pre'+name, base, to_add)
    # tf_pub.publish(Transform(post_trans, post_rot), 'post'+name, base, to_add)

def addframe(trans, rot, name, base):
    publish_frame_group(trans, rot, name, base, True)

def rmframe(name):
    # trans and rot values irrelevant
    publish_frame_group((0,0,0), (0,0,0,0), name, 'blah', False)

def moveto(name):
    (trans,rot) = tf_listener.lookupTransform(BASE, name, rospy.Time(0))
    moveit_pub.publish(Pose(Point(trans[0], trans[1], trans[2]),
                            Quaternion(rot[0], rot[1], rot[2], rot[3])))

def setclaw(state):
    claw_pub.publish(state)

def makepose(name, idx1, idx2):
    trans,rot = contacts_to_baxter_hand_pose(vertices[idx1], vertices[idx2])
    trans = (trans[0], trans[1], trans[2])
    #rot = (rot[0], rot[1], rot[2], rot[3])
    rot = (0, np.sqrt(2)/2, 0, np.sqrt(2)/2)
    publish_frame_group(trans, rot, name, OBJ_BASE, True)

if __name__ == '__main__':
    # of = obj_file.ObjFile(SPRAY_BOTTLE_MESH_FILENAME)
    # mesh = of.read()

    # vertices = mesh.vertices
    # triangles = mesh.triangles
    # normals = mesh.normals

    fdata = open(GRASP_DB_FILENAME, 'r')
    data = fdata.read()
    fdata.close()

    data = data.split('\n')[1:-1]
    data = [d.split(';')[:-1] for d in data]

    grasps = [(np.array(eval(grasp_pair[0])).reshape((4,4)),
               np.array(eval(grasp_pair[1])).reshape((4,4))) for grasp_pair in data]

    rospy.init_node('grasp_publisher')

    tf_pub = rospy.Publisher('grasper_ctrl/tf', FrameCall, queue_size=3)
    # moveit_pub = rospy.Publisher('new_position', Pose, queue_size=3)
    # claw_pub = rospy.Publisher('gripper_/control', Bool, queue_size=3)

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
            addframe(trans, rot, name, base)

        elif cmd == 'rmframe':
            # stop publishing grasp frame 
            """Example input: 
               $ cmd >> rmframe child
            """
            name = inval[1]
            rmframe(name)

        elif cmd == 'pubgrasps':
            # publish a pair of grasps from datafile, specify by index
            """Example input:
               $ cmd >> pubgrasps 3 child base
            """
            idx = eval(inval[1])
            q0 = transformations.quaternion_from_matrix(grasps[idx][0])
            t0 = transformations.translation_from_matrix(grasps[idx][0])
            q1 = transformations.quaternion_from_matrix(grasps[idx][1])
            t1 = transformations.translation_from_matrix(grasps[idx][1])

            addframe(t0, q0, inval[2]+'1', inval[3])
            addframe(t0, q0, inval[2]+'2', inval[3])

        # elif cmd == 'moveto':
        #     # command moveit
        #     """Example input: 
        #        $ cmd >> moveto child
        #     """
        #     name = inval[1]
        #     moveto(name)

        # elif cmd == 'setclaw':
        #     # command the end effector
        #     """Example input: 
        #        $ cmd >> setclaw True 
        #     """
        #     claw_bool = eval(inval[1])
        #     setclaw(claw_bool)

        # elif cmd == 'makepose':
        #     # turn two force closure vertices into a tf frame
        #     """Example input:
        #        $ cmd >> makepose name 2473 2035
        #     """
        #     name = inval[1]
        #     idx1 = int(inval[2])
        #     idx2 = int(inval[3])
        #     makepose(name, idx1, idx2)

        # elif cmd == 'test':
        #     # runs repeated tests of a single grasp
        #     """Example input:
        #         $ cmd >> test name
        #     """
        #     name = inval[1]
        #     while not rospy.is_shutdown():
        #         if raw_input("Test again? [y/n] >> ") == 'n':
        #             break

        #         moveto('pre'+name)
        #         rospy.sleep(2)
        #         moveto(name)
        #         rospy.sleep(2)
        #         setclaw(True)
        #         rospy.sleep(2)
        #         moveto('post'+name)
        #         rospy.sleep(4)
        #         moveto(name)
        #         rospy.sleep(2)
        #         setclaw(False)
        #         rospy.sleep(2)
        #         moveto('pre'+name)

        else:
            print 'Bad command: '+inval[0]
