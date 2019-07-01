#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

def zone1_summit_pos1():
    joint_goal[0] = -0.64
    joint_goal[1] = -0.12
    joint_goal[2] = 0.54
    joint_goal[3] = 0.0
    joint_goal[4] = 0.58
    joint_goal[5] = 0.0
    arm_group.go(joint_goal)

def zone1_summit():
    pose_goal.orientation.x = 0.0    #rot2[0]
    pose_goal.orientation.y = 0.5   #rot2[1]
    pose_goal.orientation.z = -0.6   #rot2[2]
    pose_goal.orientation.w = 0.3    #rot2[3]
    #arm_group.set_pose_target(pose_goal, end_effector_link='arm_gripper_link')
    arm_group.set_joint_value_target(pose_goal, 'arm_gripper_link', True)
    plan = arm_group.go()
    arm_group.stop()
    arm_group.clear_pose_targets()

def zone2_summit_pos1():
    joint_goal[0] = -0.12
    joint_goal[1] = 0.18
    joint_goal[2] = -0.5
    joint_goal[3] = 0.0
    joint_goal[4] = -0.58
    joint_goal[5] = 0.0
    arm_group.go(joint_goal)

def zone2_summit():
    pose_goal.orientation.x = 0.0    #rot2[0]
    pose_goal.orientation.y = 0.2   #rot2[1]
    pose_goal.orientation.z = 0.4   #rot2[2]
    pose_goal.orientation.w = 0.1
    arm_group.set_pose_target(pose_goal, end_effector_link='arm_gripper_link')
    #arm_group.set_joint_value_target(pose_goal, 'arm_gripper_link', True)
    plan = arm_group.go()
    arm_group.stop()
    arm_group.clear_pose_targets()

def home_summit():
    arm_group.set_named_target("home")
    plan = arm_group.go()
    arm_group.stop()
    arm_group.clear_pose_targets()

if __name__ == '__main__':
    rospy.init_node('summit_powerball')
    listener = tf.TransformListener()

    pose_goal = geometry_msgs.msg.Pose()

    moveit_commander.roscpp_initialize(sys.argv)
#Provides information such as the robot’s kinematic model and the robot’s current joint states
    powerball = moveit_commander.RobotCommander()
#Interface to a planning group (group of joints).
    group_name = "arm"
    arm_group = moveit_commander.MoveGroupCommander(group_name)
#Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher("/move_group/dysplay_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
    #arm_group.set_end_effector_link('arm_gripper_link')
    joint_goal = arm_group.get_current_joint_values()

    #Initial states
    zone1 = True
    zone2 = True
    home = True

    home_summit()           #Home position of Powerball (All joints -> 0.0)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            (trans2,rot2) = listener.lookupTransform('/world', '/to_gripper_link', rospy.Time(0))
            #(trans2,rot2) = listener.lookupTransform('/arm_gripper_link', '/to_gripper_link', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            continue

        pose_goal.position.x = trans2[0] #* -1.0
        pose_goal.position.y = trans2[1] #* -1.0
        pose_goal.position.z = 0.60#trans2[2]
        #pose_goal.orientation.x = 0.0    #rot2[0]
        #pose_goal.orientation.y = 0.5   #rot2[1]
        #pose_goal.orientation.z = -0.6   #rot2[2]
        #pose_goal.orientation.w = 0.3    #rot2[3]

        (roll, pitch, yaw) = euler_from_quaternion(rot2)
        #print roll, pitch, yaw

#----------- Zone 1: Closer to the edge of the table----------------------------------
        if (2.18 >= trans[0] >= 2.0) and (1.38 >= trans[1] >= 1.21) and zone1:
            print ('Summit in Zone 1: Closer to the edge of the table')
            zone1_summit_pos1()
            rospy.sleep(2)

            print 'Pos2 Zone 1'
            zone1_summit()

            zone1 = False
            zone2 = True
            home = True

#----------- Zone 2: Between Powerball & ABB ----------------------------------
        if (1.6 >= trans[0] >= 1.1) and (1.8 >= trans[1] >= 1.4) and zone2:
            print('Summit in Zone 2: Between Powerball & ABB')
            print 'Pos1 Zone: between bots'
            zone2_summit_pos1()
            rospy.sleep(2)

            print 'Pos2 Zone: between bots'
            zone2_summit()

            zone2 = False
            zone1 = True
            home = True

#----------- Home of summit  ----------------------------------
        if (0.35 >= trans[0] >= 0.0) and (0.0 >= trans[1] >= -0.08) and home:
            print('Summit out of work zones')
            home_summit()

            home = False
            zone1 = True
            zone2 = True

        rate.sleep()
