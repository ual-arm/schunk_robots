#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_pose', anonymous=True)

#Instantiate a RobotCommander object. Provides information such as the robot’s kinematic
#model and the robot’s current joint states
robot = moveit_commander.RobotCommander()
#Instantiate a PlanningSceneInterface object. This provides a remote interface for getting,
#setting, and updating the robot’s internal understanding of the surrounding world:
scene = moveit_commander.PlanningSceneInterface()
#Instantiate a MoveGroupCommander object. This object is an interface to a planning group
#(group of joints).
move_group = moveit_commander.MoveGroupCommander('arm')
#Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#move_group.set_end_effector_link('arm_gripper_link')
print move_group.get_end_effector_link()
#With poses
pose_goal = geometry_msgs.msg.Pose()
#pose_goal.position.x = -0.48  #0.5
#pose_goal.position.y = 0.0
#pose_goal.position.z = 0.6  #0.5
#pose_goal.orientation.x = 0.4  #0.983791
#pose_goal.orientation.y = -0.4
#pose_goal.orientation.z = -0.5
#pose_goal.orientation.w = 0.6  #-0.14

#print pose_goal

#move_group.set_joint_value_target(pose_goal, 'arm_6_link', True)

#plan = move_group.go(wait=True)
#move_group.stop()
#move_group.clear_pose_targets()

#rospy.sleep(2)

pose_goal.position.x = 0.35
pose_goal.position.y = -0.17
pose_goal.position.z = 0.6
pose_goal.orientation.x = 0.6
pose_goal.orientation.y = -0.5
pose_goal.orientation.z = 0.0
pose_goal.orientation.w = 0.0

move_group.set_joint_value_target(pose_goal, 'arm_6_link', True)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

rospy.sleep(7)

move_group.set_named_target("home")
plan = move_group.go()
move_group.stop()
move_group.clear_pose_targets()


#--------joints-------------
#joint_goal = move_group.get_current_joint_values()
#joint_goal[0] = 0.0
#joint_goal[3] = 0.4
#joint_goal[4] = -1.1
#print joint_goal

#group_plan = move_group.plan()
#move_group.go(joint_goal, wait=True)
#move_group.stop()
