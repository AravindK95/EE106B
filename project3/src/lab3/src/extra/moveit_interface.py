#!/usr/bin/env python
import sys
import rospkg
import rospy
import tf
import numpy as np
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from baxter_interface import gripper as baxter_gripper
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Transform

PROJECT_PATH = rospkg.RosPack().get_path('lab3')
sys.path.append(PROJECT_PATH+'/src/lab3')
sys.path.append(PROJECT_PATH+'/src/extra')

global left_arm
global left_gripper

def move_arm(data):
    global left_arm
    #define new target pose for arm to execute
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base'
    goal_pose.pose = data
    
    print('Setting Target Pose and Current State')
    #set left arm's target pose to new goal
    left_arm.set_pose_target(goal_pose)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    rospy.sleep(1.0)

    print('Planning Trajectory')
    #generate trajectory for target pose
    left_trajectory = left_arm.plan()

    print('Executing Trajectory')
    #execute trajectory
    left_arm.execute(left_trajectory)


def actuate_gripper(close_bool):
    global left_gripper
    
    if close_bool.data:
        #Close the left gripper
        print('Closing...')
        left_gripper.close(block=True)
        rospy.sleep(1.0)
    else:
        #Open the left gripper
        print('Opening...')
        left_gripper.open(block=True)
        rospy.sleep(1.0)

def main():
    global left_arm
    global left_gripper
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_interface')

    #Set up Baxter Arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)

    #Calibrate the end effector on the left arm
    left_gripper = baxter_gripper.Gripper('left')
    print('Calibrating...')
    left_gripper.calibrate()
    rospy.sleep(2.0)
    print('Ready to go')


    #make subscriber to position topic
    rospy.Subscriber('new_position', Pose, move_arm)

    #make subscriber to gripper topic
    rospy.Subscriber('gripper_control', Bool, actuate_gripper)

    rospy.spin()

if __name__ == '__main__':
    main()