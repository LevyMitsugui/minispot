#!/usr/bin/env python
# https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot
import numpy as np
from spotmicro.Kinematics.util import RotMatrix3D, point_to_rad
from math import *
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix


class LegIK():
    def __init__(self,
                 legtype="RIGHT",
                 shoulder_length=0.04,
                 elbow_length=0.07,
                 wrist_length=0.11,
                 hip_lim=[-0.548, 0.548],
                 shoulder_lim=[-2.17, 0.97],
                 leg_lim=[-0.1, 2.13]):
        self.legtype = legtype
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length
        self.hip_lim = hip_lim
        self.shoulder_lim = shoulder_lim
        self.leg_lim = leg_lim

    def get_domain(self, x, y, z):
        """
        Calculates the leg's Domain and caps it in case of a breach

        :param x,y,z: hip-to-foot distances in each dimension
        :return: Leg Domain D
        """
        D = (y**2 + (-z)**2 - self.shoulder_length**2 +
             (-x)**2 - self.elbow_length**2 - self.wrist_length**2) / (
                 2 * self.wrist_length * self.elbow_length)
        if D > 1 or D < -1:
            # DOMAIN BREACHED
            print("---------DOMAIN BREACH---------")
            D = np.clip(D, -1.0, 1.0)
            return D
        else:
            return D

    def solve(self, xyz_coord):
        """
        Generic Leg Inverse Kinematics Solver

        :param xyz_coord: hip-to-foot distances in each dimension
        :return: Joint Angles required for desired position
        """
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        D = self.get_domain(x, y, z)
        if self.legtype == "RIGHT":
            return self.RightIK(x, y, z, D)
        else:
            return self.LeftIK(x, y, z, D)

    def RightIK(self, x, y, z, D):
        """
        Right Leg Inverse Kinematics Solver

        :param x,y,z: hip-to-foot distances in each dimension
        :param D: leg domain
        :return: Joint Angles required for desired position
        """
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            print("NEGATIVE SQRT")
            sqrt_component = 0.0
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(sqrt_component), -self.shoulder_length)
        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(wrist_angle),
            self.elbow_length + self.wrist_length * np.cos(wrist_angle))
        joint_angles = np.array([-shoulder_angle, elbow_angle, wrist_angle])
        return joint_angles
    
    def RightIK_(self, x, y, z, D):
        
        #print("Z:",z, " Y:",y ,"X:",x)
        # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
        len_A = norm(np.sqrt(y**2 + z**2))
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            # print("NEGATIVE SQRT")
            sqrt_component = 0.0
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        # a_1 : angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
        # a_2 : angle bewtween len_A and leg's projection line on YZ plane
        # a_3 : angle between link1 and length len_A
        a_1 = point_to_rad(y, z)
        a_2 = asin(1 * self.shoulder_length / len_A)
        a_3 = pi - a_2 + pi/2

        #print("a1",a_1)
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(sqrt_component), -self.shoulder_length)
        
        # angle of link1 about the x-axis
        theta_1 = a_1 - a_3
        j2 = array([0, self.shoulder_length * cos(theta_1), self.shoulder_length * sin(theta_1)]) 
        #print("x:",x," y:",y, " z:",z)
        j4 = array([x,y,z])
        j4_2_vec = j4 - j2  # vector from j2 to j4


        R = theta_1 - pi/2 - pi/2


        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R, 0, 0], is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec, [3, 1]))

        # xyz in the rotated coordinate system + offset due to link_1 removed
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]

        len_B = norm(np.sqrt(x_**2 + z_**2))  # norm(j4-j2)
        # handling mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.elbow_length + self.wrist_length):
            len_B = (self.elbow_length + self.wrist_length) * 0.99999
            # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))

        # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and link_2
        # b_3 : angle between link_2 and link_3
        b_1 = atan(x_ / z_)
        b_2_1 = (self.elbow_length ** 2 + len_B ** 2 - self.wrist_length ** 2)
        b_2_2 = (2 * self.elbow_length * len_B)
        print(b_2_1,b_2_2)
        b_2 = acos(b_2_1 / b_2_2)
        b_3 = acos((self.elbow_length ** 2 + self.wrist_length ** 2 - len_B ** 2) / (2 * self.elbow_length * self.wrist_length))

        # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2 
        theta_3 = pi - b_3
        #print("z: ", z_, " x:", x_)

        #print("t:",b_1)
        #print("theta:",-theta_1)

        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(-theta_3),
            self.elbow_length + self.wrist_length * np.cos(-theta_3))
        #print("elbow:",shoulder_angle)




        #print("theta_1:", theta_1, " theta_2:", -theta_2, " theta_3:", -theta_3)
        #print("shoulder:", -shoulder_angle, " elbow:", elbow_angle, " wrist:", wrist_angle)
        #print("----")
        joint_angles = np.array([-theta_1, theta_2, theta_3])
        return joint_angles

    def LeftIK(self, x, y, z, D):
        """
        Left Leg Inverse Kinematics Solver

        :param x,y,z: hip-to-foot distances in each dimension
        :param D: leg domain
        :return: Joint Angles required for desired position
        """
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            print("NEGATIVE SQRT")
            sqrt_component = 0.0
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(sqrt_component), self.shoulder_length)
        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(wrist_angle),
            self.elbow_length + self.wrist_length * np.cos(wrist_angle))
        joint_angles = np.array([-shoulder_angle, -elbow_angle, -wrist_angle])
        return joint_angles
