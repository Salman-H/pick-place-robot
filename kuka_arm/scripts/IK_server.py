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


def get_transform(alpha, a, d, q):
    """Define matrix for homogeneous transforms between adjacent links."""
    tf_matrix = Matrix([
        [           cos(q),            -sin(q),            0,              a],
        [sin(q)*cos(alpha),  cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
        [sin(q)*sin(alpha),  cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
        [                0,                  0,            0,              1]
     ])
    return tf_matrix


def handle_calculate_IK(req):
    """Handle request from a CalculateIK type service."""
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # FORWARD KINEMATICS

        # Define DH parameter symbols
        alpha0, alpha1, alpha2, alpha3, alpha4,\
                alpha5, alpha6 = symbols('alpha0:7')  # joint z-axes angles
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link z-axes offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link x-axes offsets
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # joint x-axes angles

        # Construct DH Table with measurements from 'kr210.urdf.xacro' file
        DH_TABLE = {alpha0:     0,  a0:      0,  d1:  0.75,  q1:      q1,
                    alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: q2-pi/2,
                    alpha2:     0,  a2:   1.25,  d3:     0,  q3:      q3,
                    alpha3: -pi/2,  a3: -0.054,  d4:  1.50,  q4:      q4,
                    alpha4:  pi/2,  a4:      0,  d5:     0,  q5:      q5,
                    alpha5: -pi/2,  a5:      0,  d6:     0,  q6:      q6,
                    alpha6:     0,  a6:      0,  dG: 0.303,  qG:       0}

        # Compute individual transforms between adjacent links
        # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))
        t0_1 = get_transform(alpha0, a0, d1, q1)
        t1_2 = get_transform(alpha1, a1, d2, q2)
        t2_3 = get_transform(alpha2, a2, d3, q3)
        t3_4 = get_transform(alpha3, a3, d4, q4)
        t4_5 = get_transform(alpha4, a4, d5, q5)
        t5_6 = get_transform(alpha5, a5, d6, q6)
        t6_G = get_transform(alpha6, a6, dG, qG)

        # Create overall transform between base frame and gripper(G) by
        # composing the individual link transforms
        t0_G = t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_G
        t0_G = t0_G.subs(DH_TABLE)

        # Initialize service response consisting of a list of
        # joint trajectory positions (joint angles) corresponding
        # to a given gripper pose
        joint_trajectory_list = []

        # For each gripper pose a response of six joint angles is computed
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()
            # INVERSE KINEMATICS

            # Extract gripper pose(position and orientation) from IK request
            # Docs: https://github.com/ros/geometry/blob/indigo-devel/
            # tf/src/tf/transformations.py#L1089
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w]
                )

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
