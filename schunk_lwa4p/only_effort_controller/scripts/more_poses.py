#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_pose', anonymous=True)

powerball = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("arm")

while not rospy.is_shutdown():
    tecla = raw_input('Ingrese w ó a ó d para ejecutar un movimiento: \n')

# Put the arm in the home position
    if tecla == 'w':
        arm_group.set_named_target("home")
        plan1 = arm_group.go()
        print "Planning frame: %s" %arm_group.get_planning_frame()
        print "End effector link: %s" % arm_group.get_end_effector_link()
        arm_group.stop()

# Put the arm in the pos1 position
    if tecla == 'a':
        arm_group.set_named_target("pos1")
        plan1 = arm_group.go()
        arm_group.stop()

# Put the arm in the pos2 position
    if tecla == 'd':
        arm_group.set_named_target("pos2")
        plan1 = arm_group.go()
        arm_group.stop()
