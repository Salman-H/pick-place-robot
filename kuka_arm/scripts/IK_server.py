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


# Define DH parameter sympy symbols (immutable)
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')     # link z-axes offsets
d1, d2, d3, d4, d5, d6, dG = symbols('d1:7 dG')  # link x-axes offsets
q1, q2, q3, q4, q5, q6, qG = symbols('q1:7 qG')  # joint x-axes angles
alpha0, alpha1, alpha2, alpha3,\
    alpha4, alpha5, alpha6 = symbols('alpha0:7') # joint z-axes angles

# Construct DH Table with measurements from 'kr210.urdf.xacro' file
DH = {alpha0:     0,  a0:      0,  d1:  0.75,  q1:      q1,
      alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: q2-pi/2,
      alpha2:     0,  a2:   1.25,  d3:     0,  q3:      q3,
      alpha3: -pi/2,  a3: -0.054,  d4:  1.50,  q4:      q4,
      alpha4:  pi/2,  a4:      0,  d5:     0,  q5:      q5,
      alpha5: -pi/2,  a5:      0,  d6:     0,  q6:      q6,
      alpha6:     0,  a6:      0,  dG: 0.303,  qG:       0}


def get_Rx(q):
    """Define matrix for rotation (roll) about x axis."""
    Rx = Matrix([[1,      0,       0],
                 [0, cos(q), -sin(q)],
                 [0, sin(q),  cos(q)]])
    return Rx


def get_Ry(q):
    """Define matrix for rotation (pitch) about y axis."""
    Ry = Matrix([[cos(q),  0, sin(q)],
                 [     0,  1,      0],
                 [-sin(q), 0, cos(q)]])
    return Ry


def get_Rz(q):
    """Define matrix for rotation (yaw) about z axis."""
    Rz = Matrix([[cos(q), -sin(q), 0],
                 [sin(q),  cos(q), 0],
                 [     0,       0, 1]])
    return Rz


def get_TF(alpha, a, d, q):
    """Define matrix for homogeneous transforms between adjacent links."""
    Ti_j = Matrix([
        [           cos(q),            -sin(q),            0,              a],
        [sin(q)*cos(alpha),  cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
        [sin(q)*sin(alpha),  cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
        [                0,                  0,            0,              1]
     ])
    return Ti_j


def handle_calculate_IK(req):
    """Handle request from a CalculateIK type service."""
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # FORWARD KINEMATICS

        # Compute individual transforms between adjacent links
        # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))
        T0_1 = get_TF(alpha0, a0, d1, q1).subs(DH)
        T1_2 = get_TF(alpha1, a1, d2, q2).subs(DH)
        T2_3 = get_TF(alpha2, a2, d3, q3).subs(DH)
        T3_4 = get_TF(alpha3, a3, d4, q4).subs(DH)
        T4_5 = get_TF(alpha4, a4, d5, q5).subs(DH)
        T5_6 = get_TF(alpha5, a5, d6, q6).subs(DH)
        T6_G = get_TF(alpha6, a6, dG, qG).subs(DH)

        # Create overall transform between base frame and gripper(G) by
        # composing the individual link transforms
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        # Initialize service response consisting of a list of
        # joint trajectory positions (joint angles) corresponding
        # to a given gripper pose
        joint_trajectory_list = []

        # For each gripper pose a response of six joint angles is computed
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()

            # INVERSE KINEMATICS

            # Extract gripper pose (position and orientation) from IK request
            # Docs: https://github.com/ros/geometry/blob/indigo-devel/
            # tf/src/tf/transformations.py#L1089
            gx = req.poses[x].position.x
            gy = req.poses[x].position.y
            gz = req.poses[x].position.z
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w]
                )
            # Compute gripper pose w.r.t base frame using extrinsic rotations
            Rg = get_Rz(yaw) * get_Ry(pitch) * get_Rx(roll)

            # Align gripper frames in URDF vs DH params through a sequence of
            # intrinsic rotations: 180 deg yaw and -90 deg pitch and account
            # for this frame alignment error in gripper pose
            Rerror = get_Rz(radians(180)) * get_Ry(radians(-90))
            Rg = Rg * Rerror

            # Compute Wrist Center position w.r.t to base frame
            # The displacement from WC to gripper is a translation along zG of
            # magnitude dG w.r.t base frame (Refer to DH Table)
            Zg = Rg[:, 2]  # z-axis orientation (col 3) from gripper pose
            wcx = gx
            wcy = gy
            wcz = gz - DH[dG]*Zg

            # Use a geometric IK method to calculate angles theta 1,2,3 of
            # joints 1,2,3 that control WC position:

            # theta1 is calculated by viewing joint 1 and arm from top-down
            theta1 = atan2(wcy, wcx)

            # theta2,3 are calculated using cosine law on a triangle with edges
            # at joints 1,2 and WC viewed from side
            wcz_2 = wcz - DH[d1]                    # WC z-component from j2
            wcx_2 = sqrt(wcx**2 + wcy**2) - DH[a1]  # WC x-component from j2
            sideA = DH[a2]                          # joints 2-3 link length
            sideB = DH[d4]                          # joints 3-WC link length
            sideC = sqrt(wcx_2**2 + wcz_2**2)       # joints 2-WC link length
            angleB = acos((sideC**2 + sideA**2 - sideB**2) / (2*sideC*sideA))
            angleC = acos((sideA**2 + sideB**2 - sideC**2) / (2*sideA*sideB))

            theta2 = radians(90) - angleB - atan2(wcz_2, wcx_2)
            theta3 = radians(90) - angleC

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
