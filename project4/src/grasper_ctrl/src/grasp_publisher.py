#!/usr/bin/env python
import sys
import rospkg
import rospy
import tf
import numpy as np
from baxter_interface import Limb
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Transform, Pose, Vector3, Quaternion, Point
from lab3.msg import FrameCall

PROJECT_PATH = rospkg.RosPack().get_path('grasper_plan')
GRASP_DB_FILENAME = PROJECT_PATH+'/data/pawn_sortedvote.csv'
UNSORTED_DB_FILENAME = PROJECT_PATH+'/data/pawn_grasps.csv'

BASE = 'base'
OBJ_BASE = 'graspable_object'

def publish_frame(trans, rot, name, base, to_add):
    tf_pub.publish(Transform(Vector3(trans[0], trans[1], trans[2]), 
                             Quaternion(rot[0], rot[1], rot[2], rot[3])), 
                   name, 
                   base, 
                   to_add)
    
    rospy.sleep(.25)
    offset_object_frame = np.array([[1, 0, 0, 0], 
                                    [0, 1, 0, 0], 
                                    [0, 0, 1, -0.07], 
                                    [0,0,0,1]])
    offset_trans = tf.transformations.translation_from_matrix(offset_object_frame)
    offset_rot = tf.transformations.quaternion_from_matrix(offset_object_frame)

    tf_pub.publish(Transform(Vector3(offset_trans[0], offset_trans[1], offset_trans[2]), 
                             Quaternion(offset_rot[0], offset_rot[1], offset_rot[2], offset_rot[3])), 
                   'offset'+name, 
                   name, 
                   to_add)

def publish_with_pregrasp(trans, rot, name, base, to_add):
    publish_frame(trans, rot, name, base, to_add)
    
    pregrasp_object_frame = np.array([[1, 0, 0, 0], 
                                      [0, 1, 0, 0], 
                                      [0, 0, 1, -0.15], 
                                      [0,0,0,1]])
    pre_trans = tf.transformations.translation_from_matrix(pregrasp_object_frame)
    pre_rot = tf.transformations.quaternion_from_matrix(pregrasp_object_frame)
    print pre_trans
    print pre_rot


    #One of these is the correct direction to lift it straight up. Probably z.
    post_trans = np.array([trans[0], trans[1], trans[2]+.2])
    post_rot = np.array([rot[0], rot[1], rot[2], rot[3]])
    #We want to the post orientation to be the same as the initial orientation during grasp
    #so we do not need to change orientation of end effector.

    #Publish the pre and post trans
    publish_frame(pre_trans, pre_rot, 'pre'+name, name, to_add)
    rospy.sleep(.25)
    publish_frame(post_trans, post_rot, 'post'+name, base, to_add)

def addframe(trans, rot, name, base):
    publish_frame(trans, rot, name, base, True)

def rmframe(name):
    # trans and rot values irrelevant
    publish_frame((0,0,0), (0,0,0,0), name, 'blah', False)

def moveto(name):
    (trans,rot) = tf_listener.lookupTransform(BASE, "offset"+name, rospy.Time(0))
    moveit_pub.publish(Pose(Point(trans[0], trans[1], trans[2]),
                            Quaternion(rot[0], rot[1], rot[2], rot[3])))

def setclaw(state):
    claw_pub.publish(state)

def makepose(name, idx1, idx2):
    trans,rot = contacts_to_baxter_hand_pose(vertices[idx1], vertices[idx2])
    trans = (trans[0], trans[1], trans[2])
    #rot = (rot[0], rot[1], rot[2], rot[3])
    rot = (0, np.sqrt(2)/2, 0, np.sqrt(2)/2)
    publish_frame(trans, rot, name, OBJ_BASE, True)

if __name__ == '__main__':
    fdata = open(GRASP_DB_FILENAME, 'r')
    data = fdata.read()
    fdata.close()

    data = data.split('\n')[1:-1]
    data = [d.split(';')[:-1] for d in data]

    grasps = [(np.array(eval(grasp_pair[0])).reshape((4,4)),
               np.array(eval(grasp_pair[1])).reshape((4,4))) for grasp_pair in data]

    rospy.init_node('grasp_publisher')

    tf_pub = rospy.Publisher('grasper_ctrl/tf', FrameCall, queue_size=3)
    moveit_pub = rospy.Publisher('new_position', Pose, queue_size=3)
    claw_pub = rospy.Publisher('gripper_control', Bool, queue_size=3)

    tf_listener = tf.TransformListener()

    arm = Limb('left')

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
            q0 = tf.transformations.quaternion_from_matrix(grasps[idx][0])
            t0 = tf.transformations.translation_from_matrix(grasps[idx][0])
            q1 = tf.transformations.quaternion_from_matrix(grasps[idx][1])
            t1 = tf.transformations.translation_from_matrix(grasps[idx][1])

            print t0
            print t1

            addframe(t0, q0, inval[2]+'1', inval[3])
            addframe(t1, q1, inval[2]+'2', inval[3])

        elif cmd == 'pubgraspspre':
            # publish a pair of grasps from datafile with their pregrasps, specify by index
            """Example input:
               $ cmd >> pubgrasps 3 child base
            """
            idx = eval(inval[1])
            q0 = tf.transformations.quaternion_from_matrix(grasps[idx][0])
            t0 = tf.transformations.translation_from_matrix(grasps[idx][0])
            q1 = tf.transformations.quaternion_from_matrix(grasps[idx][1])
            t1 = tf.transformations.translation_from_matrix(grasps[idx][1])

            print t0
            print t1

            publish_with_pregrasp(t0, q0, inval[2]+'1', inval[3], True)
            publish_with_pregrasp(t1, q1, inval[2]+'2', inval[3], True)

        elif cmd == 'rmgrasps':
            # remove published grasp pairs
            """Example input:
               $ cmd >> rmgrasps child
            """
            name = inval[1]
            rmframe(name+'1')
            rmframe(name+'2')

        elif cmd == 'moveto':
            # command moveit
            """Example input: 
               $ cmd >> moveto child
            """
            name = inval[1]
            moveto(name)

        elif cmd == 'movetopost':
            # move effector arm to post grasp position
            """Example input:
               $ cmd >> movetopost
            """
            # trans and rot are relative to base frame
            trans = [0.502, 0.157, 0.235]
            rot = [0.486, 0.535, -0.479, 0.498]
            moveit_pub.publish(Pose(Point(trans[0], trans[1], trans[2]),
                                    Quaternion(rot[0], rot[1], rot[2], rot[3])))

        elif cmd == 'offer':
            # moves arm to 'offer' grasped object
            """Example input:
               $ cmd >> offer
            """
            trans = [0.905, 0.5, 0.316]
            rot = [0.181, 0.690, -0.088, 0.695]
            moveit_pub.publish(Pose(Point(trans[0], trans[1], trans[2]),
                                    Quaternion(rot[0], rot[1], rot[2], rot[3])))
            rospy.sleep(3)
            # Wave object like candy???
            # for offset in [0.5, -0.5, 0.5, -0.5]:
            #     joint_vals = arm.joint_angles()
            #     joint_vals['left_w1'] += offset
            #     arm.move_to_joint_positions(joint_vals, timeout=0.5)

        elif cmd == 'offerpawn':
            # moves arm to 'offer' grasped object
            """Example input:
               $ cmd >> offer
            """
            trans = [0.905, 0.5, 0.316]
            rot = [0.747, -0.101, 0.654, 0.064]
            moveit_pub.publish(Pose(Point(trans[0], trans[1], trans[2]),
                                    Quaternion(rot[0], rot[1], rot[2], rot[3])))
            rospy.sleep(3)
            # Wave object like candy???
            # for offset in [0.5, -0.5, 0.5, -0.5]:
            #     joint_vals = arm.joint_angles()
            #     joint_vals['left_w1'] += offset
            #     arm.move_to_joint_positions(joint_vals, timeout=0.5)

        elif cmd == 'setclaw':
            # command the end effector
            """Example input: 
               $ cmd >> setclaw True 
            """
            claw_bool = eval(inval[1])
            setclaw(claw_bool)

        else:
            print 'Bad command: '+inval[0]
