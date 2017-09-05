"""
Module for testing a numpy implementation of ROS node IK_server.py

Inverse kinematics is applied to convert a single end-effector position
in cartesian-space to its joint-space. Then Forward kinematics is applied
to recompute the end-effector's position to verify the correctness of the
joint-space.

"""

__author__ = 'Salman Hashmi, Sahil Juneja'


from time import time

from numpy import array, matrix, cos, sin, pi, arccos, arctan2, sqrt
from numpy.linalg import inv
import tf

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt


# Test case format:
# -----------------
# [[[EE position],[EE orientation as quaternions]],
#  [WC location],
#  [joint angles]]
#
# To generate additional test cases:
# ----------------------------------
# Run '$ roslaunch kuka_arm forward_kinematics.launch', adjust the joint angles
# to find thetas. Use the gripper to extract positions and orientation
# (in quaternion xyzw) and use link 5 to find the position of the wrist center.
# These newly generated test cases can be added to the test_cases dictionary.
test_cases = {
    1: [[[2.16135, -1.42635, 1.55109], [0.708611, 0.186356, -0.157931, 0.661967]],
        [1.89451, -1.44302, 1.69366],
        [-0.65, 0.45, -0.36, 0.95, 0.79, 0.49]],
    2: [[[-0.56754, 0.93663, 3.0038], [0.62073, 0.48318, 0.38759, 0.480629]],
        [-0.638, 0.64198, 2.9988],
        [-0.79, -0.11, -2.33, 1.94, 1.14, -3.68]],
    3: [[[-1.3863, 0.02074, 0.90986], [0.01735, -0.2179, 0.9025, 0.371016]],
        [-1.1669, -0.17989, 0.85137],
        [-2.99, -0.12, 0.94, 4.06, 1.29, -4.12]],
    4: [],
    5: []
}

# To store coordinates for plotting
received_ee_points = []
your_ee_points = []
ee_errors = []


def test_code(test_case):
    """
    Calculate end-effector errors in computing Inverse kinematics.

    Keyword arguments:
    test_case -- a dictionary of test cases with following format

    [[[EE position], [EE orientation quaternions]],
      [WC location],
      [joint angles]]

    """
    x = 0

    class Position:
        """Define position of end-effector."""

        def __init__(self, EE_pos):
            """Initialize position object."""
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]

    class Orientation:
        """Define orientation of end-effector."""

        def __init__(self, EE_ori):
            """Initialize position object."""
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        """Define combine object."""

        def __init__(self, position, orientation):
            """Initialize combine object."""
            self.position = position
            self.orientation = orientation

    comb = Combine(position, orientation)

    class Pose:
        """Define end-effector pose."""

        def __init__(self, comb):
            """Initialize end-effector pose object."""
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    # INVERSE KINEMATICS ======================================================
    def get_DH_Table():
        """
        Define DH parameters for Kuka KR10 from its URDF file.

        alphai-1 :  angle b/w z-axes of links i-1 & i along x-axis of link i-1
        ai-1     :  dist b/w z-axes of links i-1 & i along x-axis of link i-1
        di       :  dist b/w x-axes of links i-1 & i along z-axis of link i
        thetai   :  angle b/w x-axes of links i-1 & i along z-axis of link i

        """
        # Define variables for joint angles
        theta1, theta2, theta3, theta4, theta5, theta6 = 0., 0., 0., 0., 0., 0.
        # Construct DH Table with measurements from 'kr210.urdf.xacro' file
        dh = {'alpha0':     0,  'a0':      0,  'd1':  0.75,  'theta1':  theta1,
              'alpha1': -pi/2,  'a1':   0.35,  'd2':     0,  'theta2':  theta2,
              'alpha2':     0,  'a2':   1.25,  'd3':     0,  'theta3':  theta3,
              'alpha3': -pi/2,  'a3': -0.054,  'd4':  1.50,  'theta4':  theta4,
              'alpha4':  pi/2,  'a4':      0,  'd5':     0,  'theta5':  theta5,
              'alpha5': -pi/2,  'a5':      0,  'd6':     0,  'theta6':  theta6,
              'alpha6':     0,  'a6':      0,  'dG': 0.303,  'thetaG':       0}
        return dh

    def get_Rx(theta):
        """Define matrix for rotation (roll) about x axis."""
        Rx = matrix([[1,          0,           0],
                     [0, cos(theta), -sin(theta)],
                     [0, sin(theta),  cos(theta)]])
        return Rx

    def get_Ry(theta):
        """Define matrix for rotation (pitch) about y axis."""
        Ry = matrix([[cos(theta),  0, sin(theta)],
                     [         0,  1,          0],
                     [-sin(theta), 0, cos(theta)]])
        return Ry

    def get_Rz(theta):
        """Define matrix for rotation (yaw) about z axis."""
        Rz = matrix([[cos(theta), -sin(theta), 0],
                     [sin(theta),  cos(theta), 0],
                     [         0,           0, 1]])
        return Rz

    def get_TF(alpha, a, d, theta):
        """Define matrix for homogeneous transforms between adjacent links."""
        Tf = matrix([
            [           cos(theta),            -sin(theta),            0,              a],
            [sin(theta)*cos(alpha),  cos(theta)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
            [sin(theta)*sin(alpha),  cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d],
            [                    0,                      0,            0,              1]
         ])
        return Tf

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

        print('******************************************************')
        print('EE position: {}'.format(position))

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

    def get_WC(dh, R_ee, ee_pose):
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
        EE_P = matrix([[ee_x],
                       [ee_y],
                       [ee_z]])
        # Get Col3 vector from R_ee that describes z-axis orientation of EE
        Z_ee = R_ee[:, 2]
        # WC is a displacement from EE equal to a translation along
        # the EE z-axis of magnitude dG w.r.t base frame (Refer to DH Table)
        Wc = EE_P - dh['dG']*Z_ee
        return Wc

    def get_joints1_2_3(dh, Wc):
        """
        Calculate joint angles 1,2,3 using geometric IK method.

        NOTE: Joints 1,2,3 control position of WC (joint 5)

        """
        wcx, wcy, wcz = Wc[0], Wc[1], Wc[2]
        # theta1 is calculated by viewing joint 1 and arm from top-down
        theta1 = arctan2(wcy, wcx)
        # theta2,3 are calculated using Cosine Law on a triangle with edges
        # at joints 1,2 and WC viewed from side and
        # forming angles A, B and C respectively
        wcz_j2 = wcz - dh['d1']                    # WC z-component from j2
        wcx_j2 = sqrt(wcx**2 + wcy**2) - dh['a1']  # WC x-component from j2
        side_a = round(sqrt((dh['d4'])**2 + (dh['a3'])**2), 7)  # line segment: j3-WC
        side_b = sqrt(wcx_j2**2 + wcz_j2**2)                    # line segment: j2-WC
        side_c = dh['a2']                                       # link length:  j2-j3
        angleA = arccos((side_b**2 + side_c**2 - side_a**2) / (2*side_b*side_c))
        angleB = arccos((side_a**2 + side_c**2 - side_b**2) / (2*side_a*side_c))
        angleC = arccos((side_a**2 + side_b**2 - side_c**2) / (2*side_a*side_b))
        # The sag between joint-3 and WC is due to a3 and its angle is formed
        # between y3-axis and side_a
        angle_sag = round(arctan2(abs(dh['a3']), dh['d4']), 7)
        theta2 = pi/2 - angleA - arctan2(wcz_j2, wcx_j2)
        theta3 = pi/2 - (angleB + angle_sag)
        return theta1, theta2, theta3

    def get_joints4_5_6(dh, R_ee, theta1, theta2, theta3):
        """
        Calculate joint Euler angles 4,5,6 using analytical IK method.

        NOTE: Joints 4,5,6 constitute the wrist and control WC orientation

        """
        # Compute individual transforms between adjacent links
        # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))
        # account for 90 deg constant offset in theta2
        T0_1 = get_TF(dh['alpha0'], dh['a0'], dh['d1'], theta1)
        T1_2 = get_TF(dh['alpha1'], dh['a1'], dh['d2'], theta2-pi/2)
        T2_3 = get_TF(dh['alpha2'], dh['a2'], dh['d3'], theta3)
        # Extract rotation components of joints 1,2,3 from their
        # respective individual link Transforms
        R0_1 = T0_1[0:3, 0:3]
        R1_2 = T1_2[0:3, 0:3]
        R2_3 = T2_3[0:3, 0:3]
        # Evaluate the composite rotation matrix formed by composing
        # these individual rotation matrices
        R0_3 = R0_1 * R1_2 * R2_3
        # R3_6 is the composite rotation matrix formed from an extrinsic
        # x-y-z (roll-pitch-yaw) rotation sequence that orients WC
        # b/c R0_6 = R_ee = R0_3*R3_6
        R3_6 = inv(array(R0_3, dtype='float')) * R_ee
        r21 = R3_6[1, 0]  # sin(theta5)*cos(theta6)
        r22 = R3_6[1, 1]  # -sin(theta6)*sin(theta6)
        r13 = R3_6[0, 2]  # -sin(theta5)*cos(theta4)
        r23 = R3_6[1, 2]  # cos(theta5)
        r33 = R3_6[2, 2]  # sin(theta4)*sin(theta5)
        # Compute Euler angles theta 4,5,6 from R3_6 by individually
        # isolating and explicitly solving each angle
        theta4 = arctan2(r33, -r13)
        theta5 = arctan2(sqrt(r13**2 + r33**2), r23)
        theta6 = arctan2(-r22, r21)
        return theta4, theta5, theta6

    def handle_IK():
        """Simulates handle_calculate_IK()."""
        dh = get_DH_Table()

        # INVERSE KINEMATICS
        ee_pose = get_ee_pose(req.poses[x])
        R_ee = get_R_EE(ee_pose)
        Wc = get_WC(dh, R_ee, ee_pose)
        theta1, theta2, theta3 = get_joints1_2_3(dh, Wc)
        theta4, theta5, theta6 = get_joints4_5_6(dh, R_ee, theta1, theta2, theta3)

        return Wc, theta1, theta2, theta3, theta4, theta5, theta6

    # FORWARD KINEMATICS ======================================================
    Wc, theta1, theta2, theta3, theta4, theta5, theta6 = handle_IK()
    dh = get_DH_Table()
    dh['theta1'] = theta1
    dh['theta2'] = theta2-pi/2  # account for 90 deg constant offset
    dh['theta3'] = theta3
    dh['theta4'] = theta4
    dh['theta5'] = theta5
    dh['theta6'] = theta6
    # Compute individual transforms between adjacent links
    # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))
    T0_1 = get_TF(dh['alpha0'], dh['a0'], dh['d1'], dh['theta1'])
    T1_2 = get_TF(dh['alpha1'], dh['a1'], dh['d2'], dh['theta2'])
    T2_3 = get_TF(dh['alpha2'], dh['a2'], dh['d3'], dh['theta3'])
    T3_4 = get_TF(dh['alpha3'], dh['a3'], dh['d4'], dh['theta4'])
    T4_5 = get_TF(dh['alpha4'], dh['a4'], dh['d5'], dh['theta5'])
    T5_6 = get_TF(dh['alpha5'], dh['a5'], dh['d6'], dh['theta6'])
    T6_ee = get_TF(dh['alpha6'], dh['a6'], dh['dG'], dh['thetaG'])
    # Create overall transform between base frame and EE by
    # composing the individual link transforms
    T0_ee = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_ee
    your_wc = [Wc[0], Wc[1], Wc[2]]                    # Load computed Wc value
    your_ee = [T0_ee[0, 3], T0_ee[1, 3], T0_ee[2, 3]]  # Load computed Ee value

    # ERROR ANALYSIS =========================================================
    print("\nTotal run time to calculate joint angles from pose" +
          " is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc) == 3):
        wc_x_e = abs(your_wc[0] - test_case[1][0])
        wc_y_e = abs(your_wc[1] - test_case[1][1])
        wc_z_e = abs(your_wc[2] - test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print("\nWrist error for x position is: %04.8f" % wc_x_e)
        print("Wrist error for y position is: %04.8f" % wc_y_e)
        print("Wrist error for z position is: %04.8f" % wc_z_e)
        print("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1 - test_case[2][0])
    t_2_e = abs(theta2 - test_case[2][1])
    t_3_e = abs(theta3 - test_case[2][2])
    t_4_e = abs(theta4 - test_case[2][3])
    t_5_e = abs(theta5 - test_case[2][4])
    t_6_e = abs(theta6 - test_case[2][5])
    print("\nTheta 1 error is: %04.8f" % t_1_e)
    print("Theta 2 error is: %04.8f" % t_2_e)
    print("Theta 3 error is: %04.8f" % t_3_e)
    print("Theta 4 error is: %04.8f" % t_4_e)
    print("Theta 5 error is: %04.8f" % t_5_e)
    print("Theta 6 error is: %04.8f" % t_6_e)
    print("\n" +
          "** These theta errors may not be a correct representation of your" +
          "\ncode, due to the fact that the arm can have multiple positions." +
          "\nIt is best to add your forward kinematics to confirm" +
          "\nwhether your code is working or not.**")
    print(" ")

    # Find FK EE error
    if not(sum(your_ee) == 3):
        ee_x_e = abs(your_ee[0] - test_case[0][0][0])
        ee_y_e = abs(your_ee[1] - test_case[0][0][1])
        ee_z_e = abs(your_ee[2] - test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print("End effector error for y position is: %04.8f" % ee_y_e)
        print("End effector error for z position is: %04.8f" % ee_z_e)
        print("Overall end effector offset is: %04.8f units \n" % ee_offset)

    # Add EE points for plotting
    print('')
    received_ee_points.append(test_case[0][0])

    your_ee_points.append(
        [round(your_ee[0].item(0), 8),
         round(your_ee[1].item(0), 8),
         round(your_ee[2].item(0), 8)]
        )

    ee_errors.append(
        [round(ee_x_e.item(0), 8),
         round(ee_y_e.item(0), 8),
         round(ee_z_e.item(0), 8)]
        )


if __name__ == "__main__":
    # Change test case number for different scenarios
    for i in range(1, 4):
        test_case_number = i
        test_code(test_cases[test_case_number])

    #print(received_ee_points)
    #print(your_ee_points)
    #print(ee_errors)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    data1 = array(received_ee_points)
    x1, y1, z1 = data1.T
    ax.scatter(x1, y1, z1, c='blue', s=50, marker='o')
    ax.plot(x1, y1, z1, c='blue', label='rec_ee')

    data2 = array(your_ee_points)
    x2, y2, z2 = data2.T
    ax.scatter(x2, y2, z2, c='orange', s=50, marker='s')
    ax.plot(x2, y2, z2, c='orange', label='fk_ee')

    data3 = array(ee_errors)
    x3, y3, z3 = data3.T
    ax.scatter(x3, y3, z3, c='magenta', s=50, marker='^')
    ax.plot(x3, y3, z3, c='magenta', label='ee_error')

    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    ax.xaxis.label.set_color('red')
    ax.yaxis.label.set_color('green')
    ax.zaxis.label.set_color('blue')

    ax.grid(True, which='both')
    ax.view_init(30,220)

    plt.legend()
    plt.show()
