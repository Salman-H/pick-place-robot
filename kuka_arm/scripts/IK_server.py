#!/usr/bin/env python
"""
ROS node for Inverse Kinematic analyis of the KUKA KR210 robot arm.

Receives end-effector (gripper) poses from the KR210 simulator and performs
Inverse Kinematics, providing a response to the simulator with calculated
joint variable values (joint angles in this case).


Copyright (c) 2017 Electric Movement Inc.

This file is part of Robotic Arm: Pick and Place project for Udacity
Robotics nano-degree program

All Rights Reserved.

"""

__author__ = 'Salman Hashmi, Ryan Keenan, Harsh Pandya'


import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    """Handle request from a CalculateIK type service."""
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # FORWARD KINEMATICS

        # Define DH parameter symbols
        alpha0, alpha1, alpha2, alpha3, \
        alpha4, alpha5, alpha6 = symbols('alpha0:7')  # joint z-axes angles
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link z-axes offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link x-axes offsets
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # joint x-axes angles

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()

            # IK code

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3,
                                                theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("Number of joint trajectory points:" +
                      " %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    """Initialize IK_server ROS node and declare calculate_ik service."""
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
