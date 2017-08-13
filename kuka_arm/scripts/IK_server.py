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


def get_ee_pose(pose_msg):
    """
    Extract EE pose from received trajectory pose in an IK request message.

    NOTE: Pose is position (cartesian coords) and orientation (euler angles)

    Docs: https://github.com/ros/geometry/blob/indigo-devel/
          tf/src/tf/transformations.py#L1089
    """
    ee_x = pose_msg.position.x
    ee_y = pose_msg.position.y
    ee_z = pose_msg.position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [pose_msg.orientation.x, pose_msg.orientation.y,
         pose_msg.orientation.z, pose_msg.orientation.w]
        )
    position = (ee_x, ee_y, ee_z)
    orientation = (roll, pitch, yaw)

    return position, orientation


def get_R_EE(ee_pose):
    """
    Compute EE Rotation matrix w.r.t base frame.

    Computed from EE orientation (roll, pitch, yaw) and describes the
    orientation of each axis of EE w.r.t the base frame

    """
    roll, pitch, yaw = ee_pose[1]
    # Perform extrinsic (fixed-axis) sequence of rotations of EE about
    # x, y, and z axes by roll, pitch, and yaw radians respectively
    R_ee = get_Rz(yaw) * get_Ry(pitch) * get_Rx(roll)
    # Align EE frames in URDF vs DH params through a sequence of
    # intrinsic (body-fixed) rotations: 180 deg yaw and -90 deg pitch
    Rerror = get_Rz(pi) * get_Ry(-pi/2)
    # Account for this frame alignment error in EE pose
    R_ee = R_ee * Rerror

    return R_ee


def get_WC(R_ee, ee_pose):
    """
    Compute Wrist Center position (cartesian coords) w.r.t base frame.

    Keyword arguments:
    R_ee -- EE Rotation matrix w.r.t base frame
    ee_pose -- tuple of cartesian coords and euler angles describing EE

    Return values:
    Wc -- vector of cartesian coords of WC

    """
    ee_x, ee_y, ee_z = ee_pose[0]
    # Define EE position as a vector
    EE_P = Matrix([[ee_x],
                   [ee_y],
                   [ee_z]])
    # Get Col3 vector from R_ee that describes z-axis orientation of EE
    Z_ee = R_ee[:, 2]
    # WC is a displacement from EE equal to a translation along
    # the EE z-axis of magnitude dG w.r.t base frame (Refer to DH Table)
    Wc = EE_P - DH[dG]*Z_ee

    return Wc


def get_joints1_2_3(Wc):
    """
    Calculate joint angles 1,2,3 using geometric IK method.

    NOTE: Joints 1,2,3 control position of WC (joint 5)

    """
    wcx, wcy, wcz = Wc[0], Wc[1], Wc[2]

    # theta1 is calculated by viewing joint 1 and arm from top-down
    theta1 = atan2(wcy, wcx)

    # theta2,3 are calculated using Cosine Law on a triangle with edges
    # at joints 1,2 and WC viewed from side and
    # forming angles A, B and C repectively
    wcz_j2 = wcz - DH[d1]                              # WC z-component from j2
    wcx_j2 = sqrt(wcx**2 + wcy**2) - DH[a1]            # WC x-component from j2

    side_a = round(sqrt((DH[d4])**2 + (DH[a3])**2), 7) # line segment: j3-WC
    side_b = sqrt(wcx_j2**2 + wcz_j2**2)               # line segment: j2-WC
    side_c = DH[a2]                                    # link length:  j2-j3

    angleA = acos((side_b**2 + side_c**2 - side_a**2) / (2*side_b*side_c))
    angleB = acos((side_a**2 + side_c**2 - side_b**2) / (2*side_a*side_c))
    angleC = acos((side_a**2 + side_b**2 - side_c**2) / (2*side_a*side_b))

    # The sag between joint-3 and WC is due to a3 and its angle is formed
    # between y3-axis and side_a
    angle_sag = round(atan2(abs(DH[a3]),DH[d4]), 7)

    theta2 = pi/2 - angleA - atan2(wcz_j2, wcx_j2)
    theta3 = pi/2 - (angleB + angle_sag)

    return theta1, theta2, theta3


def get_joints4_5_6(R_ee, T0_1, T1_2, T2_3, theta1, theta2, theta3):
    """
    Calculate joint Euler angles 4,5,6 using analytical IK method.

    NOTE: Joints 4,5,6 constitute the wrist and control WC orientation

    """
    # Extract rotation components of joints 1,2,3 from their
    # respective individual link Transforms
    R0_1 = T0_1[0:3, 0:3]
    R1_2 = T1_2[0:3, 0:3]
    R2_3 = T2_3[0:3, 0:3]
    # Evaluate the composite rotation matrix fromed by composing
    # these individual rotation matrices
    R0_3 = R0_1 * R1_2 * R2_3
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    # R3_6 is the composite rotation matrix formed from an extrinsic
    # x-y-z (roll-pitch-yaw) rotation sequence that orients WC
    R3_6 = R0_3.inv("LU") * R_ee  # b/c: R0_6 = R_ee = R0_3*R3_6

    r21 = R3_6[1, 0]              # sin(theta5)*cos(theta6)
    r22 = R3_6[1, 1]              # -sin(theta6)*sin(theta6)
    r13 = R3_6[0, 2]              # -sin(theta5)*cos(theta4)
    r23 = R3_6[1, 2]              # cos(theta5)
    r33 = R3_6[2, 2]              # sin(theta4)*sin(theta5)

    # Compute Euler angles theta 4,5,6 from R3_6 by individually
    # isolating and explicitly solving each angle
    theta4 = atan2(r33, -r13)
    theta5 = atan2(sqrt(r13**2 + r33**2), r23)
    theta6 = atan2(-r22, r21)

    return theta4, theta5, theta6


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
            ee_pose = get_ee_pose(req.poses[x])

            R_ee = get_R_EE(ee_pose)
            Wc = get_WC(R_ee, ee_pose)

            theta1, theta2, theta3 = get_joints1_2_3(Wc)
            theta4, theta5, theta6 = get_joints4_5_6(R_ee, T0_1, T1_2, T2_3,
                                                     theta1, theta2, theta3)

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
