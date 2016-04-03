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

def move_arm(data):
	#define new target pose for arm to execute
	goal_pose = PoseStamped()
	goal_pose.header.frame_id = 'base'
	goal_pose.pose = data
    

    #set left arm's target pose to new goal
    left_arm.set_pose_target(goal_pose)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    #generate trajectory for target pose
    left_trajectory = left_arm.plan()

    #execute trajectory
    left_arm.execute(left_trajectory)


def actuate_gripper(close_bool):
    left_gripper = baxter_gripper.Gripper('left')

	#Calibrate the gripper (other commands won't work unless you do this first)
	print('Calibrating...')
	left_gripper.calibrate()
	rospy.sleep(2.0)

	if close_bool:
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
	#Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_interface')

    #Set up Baxter Arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #make subscriber to position topic
    rospySubscriber("new_position", Pose, move_arm)

    #make subscriber to gripper topic
    rospySubscriber("gripper_control", Bool, actuate_gripper)

    rospy.spin()

if __name__ == '__main__':
	main()