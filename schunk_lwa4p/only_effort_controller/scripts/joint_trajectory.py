#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_joints', anonymous=True)

#Instantiate a RobotCommander object. Provides information such as the robot’s kinematic
#model and the robot’s current joint states
robot = moveit_commander.RobotCommander()
#Instantiate a PlanningSceneInterface object. This provides a remote interface for getting,
#setting, and updating the robot’s internal understanding of the surrounding world:
scene = moveit_commander.PlanningSceneInterface()
#Instantiate a MoveGroupCommander object. This object is an interface to a planning group
#(group of joints).
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
#Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0.4
joint_goal[3] = 0
joint_goal[4] = 0.2
joint_goal[5] = 0.5

group_plan = move_group.plan()
# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)
# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()
