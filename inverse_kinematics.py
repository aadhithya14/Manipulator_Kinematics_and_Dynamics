import numpy as np
from robot_parameters import Robot_Parameters
from forward_kinematics import Forward_Kinematics
from sympy import *
import sympy
import math


class Inverse_Kinematics():
    def __init__(self):
        #self.inverse_transformation=inverse_transformation
        self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6=symbols('theta1,theta2,theta3,theta4,theta5,theta6')
        #self.forward_kinematics=Forward_Kinematics(theta=[self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6])
        #self.forward_kinematics.get_transformation_matrix()
        #self.x,self.y,self.z=self.forward_kinematics.get_end_effector_positions()
        #self.x_inverse,self.y_inverse,self.z_inverse=self.inverse_transformation[0][3],self.inverse_transformation[1][3],self.inverse_transformation[2][3]
        #self.x1_inverse,self.x2_inverse,self.x3_inverse,self.y1_inverse,self.y2_inverse,self.y3_inverse,self.z1_inverse, self.z2_inverse, self.z3_inverse=self.inverse_transformation[0][0],self.inverse_transformation[0][1],self.inverse_transformation[0][2],self.inverse_transformation[1][0],self.inverse_transformation[1][1], self.inverse_transformation[1][2], self.inverse_transformation[2][0], self.inverse_transformation[2][1], self.inverse_transformation[2][2]
        #self.forward_transformation=self.forward_kinematics.get_final_transformation_matrix()
        #self.x1,self.x2,self.x3,self.y1,self.y2,self.y3,self.z1, self.z2, self.z3=self.forward_transformation[0][0],self.forward_transformation[0][1],self.forward_transformation[0][2],self.forward_transformation[1][0],self.forward_transformation[1][1], self.forward_transformation[1][2], self.forward_transformation[2][0], self.forward_transformation[2][1], self.forward_transformation[2][2]
        

    def solve_joint_angles(self):
        eq1=self.x-self.x_inverse
        eq2=self.y-self.y_inverse
        eq3=self.z-self.z_inverse
        eq4=self.x1-self.x1_inverse
        eq5=self.y1-self.y1_inverse
        eq6=self.z1-self.z1_inverse
        eq7=self.x2-self.x2_inverse
        eq8=self.y2-self.y2_inverse
        eq9=self.z2-self.z2_inverse
        eq10=self.x3-self.x3_inverse
        eq11=self.y3-self.y3_inverse
        eq12=self.z3-self.z3_inverse
        result=solve([eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq8,eq9,eq10,eq11,eq12],[self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6])
        
        return result

    def get_hypotenuse(self,a, b):
  # calculate the longest side given the two shorter sides 
  # of a right triangle using pythagorean theorem
        return sqrt(a*a + b*b)


    def get_cosine_law_angle(self,a, b, c):    
        # given all sides of a triangle a, b, c
        # calculate angle gamma between sides a and b using cosine law
        cos_gamma = (a*a + b*b - c*c) / (2*a*b)
        sin_gamma = sqrt(1 - cos_gamma * cos_gamma)
        gamma = atan2(sin_gamma, cos_gamma)

        return gamma


    def get_wrist_center(self,gripper_point, R0g, dg = 0.303):
        # get the coordinates of the wrist center wrt to the base frame (xw, yw, zw)
        # given the following info:
        # the coordinates of the gripper (end effector) (x, y, z)
        # the rotation of the gripper in gripper frame wrt to the base frame (R0u)
        # the distance between gripper and wrist center dg which is along common z axis
        # check Report for more info
        xu, yu, zu = gripper_point 
            
        nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
        xw = xu - dg * nx
        yw = yu - dg * ny
        zw = zu - dg * nz 

        return xw, yw, zw


    def get_first_three_angles(self,wrist_center):
        # given the wrist center which a tuple of 3 numbers x, y, z
        # (x, y, z) is the wrist center point wrt base frame
        # return the angles q1, q2, q3 for each respective joint
        # given geometry of the kuka kr210
        # check Report for more info
        x, y, z  = wrist_center
        self.robot_parameters=Robot_Parameters()   
        a = self.robot_parameters.get_a()
        d = self.robot_parameters.get_d()
        a1=a[0]
        d1=d[0]
        a2=a[1]
        d4=d[3]
        a3=a[2]
        l = self.get_hypotenuse(d4, abs(a3))
        phi = atan2(d4, abs(a3))
        
        x_prime = self.get_hypotenuse(x, y)
        mx = x_prime -  a1
        my = z - d1 
        m = self.get_hypotenuse(mx, my)
        alpha = atan2(my, mx)
        
        gamma = self.get_cosine_law_angle(l, a2, m)
        beta = self.get_cosine_law_angle(m, a2, l)
        
        q1 = atan2(y, x)
        q2 = pi/2 - beta - alpha 
        q3 = -(gamma - phi)
            
        return q1, q2, q3 


    def get_last_three_angles(self,R):
        # from our simplification, R36 (R) equals the following:
        #Matrix([
        #[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
        #[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
        #[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
        #From trigonometry we can get q4, q5, q6 if we know numerical values of all cells of matrix R36 (R)   
        sin_q4 = R[2, 2]
        cos_q4 =  -R[0, 2]
            
        sin_q5 = sqrt(R[0, 2]**2 + R[2, 2]**2) 
        cos_q5 = R[1, 2]
            
        sin_q6 = -R[1, 1]
        cos_q6 = R[1, 0] 
        
        q4 = atan2(sin_q4, cos_q4)
        q5 = atan2(sin_q5, cos_q5)
        q6 = atan2(sin_q6, cos_q6)
            
        return q4, q5, q6


    def get_angles(self,x, y, z, roll, pitch, yaw):
        # input: given position and orientation of the gripper_URDF wrt base frame
        # output: angles q1, q2, q3, q4, q5, q6
            
        gripper_point = x, y, z

        ################################################################################
        # All symbolic transformations matrices are declared below 
        ################################################################################

        q1, q2, q3, q4, q5, q6 = symbols('q1:7')
        alpha, beta, gamma = symbols('alpha beta gamma', real = True)
        px, py, pz = symbols('px py pz', real = True)

        # Rotation of joint 3 wrt to the base frame interms the first three angles q1, q2, q3
        R03 = Matrix([
            [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
            [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
            [        cos(q2 + q3),        -sin(q2 + q3),        0]])

        # Transpose of R03 
        R03T = Matrix([
            [sin(q2 + q3)*cos(q1), sin(q1)*sin(q2 + q3),  cos(q2 + q3)],
            [cos(q1)*cos(q2 + q3), sin(q1)*cos(q2 + q3), -sin(q2 + q3)],
            [            -sin(q1),              cos(q1),             0]])

        # Rotation of joint 6 wrt to frame of joint 3 interms of the last three angles q4, q5, q6
        R36 = Matrix([
            [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
            [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
            [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])

        # Rotation of urdf_gripper with respect to the base frame interms of alpha = yaw, beta = pitch, gamma = roll
        R0u = Matrix([
            [1.0*cos(alpha)*cos(beta), -1.0*sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha), 1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)],
            [1.0*sin(alpha)*cos(beta),  sin(alpha)*sin(beta)*sin(gamma) + 1.0*cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha)],
            [          -1.0*sin(beta),                                     1.0*sin(gamma)*cos(beta),                                    1.0*cos(beta)*cos(gamma)]])

        # Total transform of gripper wrt to base frame given orientation yaw (alpha), pitch (beta), roll (beta) and position px, py, pz
        T0g_b = Matrix([
            [1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma),  1.0*sin(alpha)*cos(gamma) - 1.0*sin(beta)*sin(gamma)*cos(alpha), 1.0*cos(alpha)*cos(beta), px],
            [sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha), -1.0*sin(alpha)*sin(beta)*sin(gamma) - 1.0*cos(alpha)*cos(gamma), 1.0*sin(alpha)*cos(beta), py],
            [                                   1.0*cos(beta)*cos(gamma),                                        -1.0*sin(gamma)*cos(beta),           -1.0*sin(beta), pz],
            [                                                          0,                                                                0,                        0,  1]])

        # Total transform of gripper wrt to base frame given angles q1, q2, q3, q4, q5, q6
        T0g_a = Matrix([
            [((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
            [ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
            [                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
            [                                                                                                                                                            0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                              1]])

        # Rotation of urdf_gripper wrt (DH) gripper frame from rotz(pi) * roty(-pi/2) and it's transpose
        Rgu_eval = Matrix([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
        RguT_eval = Matrix([[0, 0, -1], [ -1, 0, 0], [0, 1, 0]])

        # Inverse kinematics transformations starts below

        R0u_eval = R0u.evalf(subs = {alpha: yaw, beta: pitch, gamma: roll})
        R0g_eval = R0u_eval * RguT_eval

        wrist_center = self.get_wrist_center(gripper_point, R0g_eval, dg = 0.303)

        j1, j2, j3 = self.get_first_three_angles(wrist_center)

        R03T_eval = R03T.evalf(subs = {q1: j1.evalf(), q2: j2.evalf(), q3: j3.evalf()})
        R36_eval = R03T_eval * R0g_eval

        j4, j5, j6 = self.get_last_three_angles(R36_eval)

        j1 = j1.evalf()
        j2 = j2.evalf()
        j3 = j3.evalf()
        j4 = j4.evalf()
        j5 = j5.evalf()
        j6 = j6.evalf()

        return j1, j2, j3, j4, j5, j6
        

    
