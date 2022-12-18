import numpy as np
from sympy import *
from forward_kinematics import Forward_Kinematics

class Geometric_Jacobian():
    def __init__(self,theta):
        self.theta=theta
        self.forward_kinematics=Forward_Kinematics(self.theta)
        self.final_transformation=self.forward_kinematics.get_final_transformation_matrix()
        
    def get_jacobian(self):
        self.Z0,self.Z1,self.Z2,self.Z3,self.Z4,self.Z5=self.forward_kinematics.get_Z()
        self.joint_positions=self.forward_kinematics.get_joint_positions()
        #print(self.joint_positions[6]-self.joint_positions[0])
        self.P0=Matrix([self.joint_positions[6]-self.joint_positions[0]])
        self.P1=Matrix([self.joint_positions[6]-self.joint_positions[1]])
        self.P2=Matrix([self.joint_positions[6]-self.joint_positions[2]])
        self.P3=Matrix([self.joint_positions[6]-self.joint_positions[3]])
        self.P4=Matrix([self.joint_positions[6]-self.joint_positions[4]])
        self.P5=Matrix([self.joint_positions[6]-self.joint_positions[5]])

        self.cross1=self.Z0.cross(self.P0)
        self.cross2=self.Z1.cross(self.P1)
        self.cross3=self.Z2.cross(self.P2)
        self.cross4=self.Z3.cross(self.P3)
        self.cross5=self.Z4.cross(self.P4)
        self.cross6=self.Z5.cross(self.P5)

        self.jacobian=Matrix([[self.cross1[0],self.cross2[0],self.cross3[0],self.cross4[0],self.cross5[0],self.cross6[0]],[self.cross1[1],self.cross2[1],self.cross3[1],self.cross4[1],self.cross5[1],self.cross6[1]],[self.cross1[2],self.cross2[2],self.cross3[2],self.cross4[2],self.cross5[2],self.cross6[2]],[self.Z0[0],self.Z1[0],self.Z2[0],self.Z3[0],self.Z4[0],self.Z5[0]],[self.Z0[1],self.Z1[1],self.Z2[1],self.Z3[1],self.Z4[1],self.Z5[1]],[self.Z0[2],self.Z1[2],self.Z2[2],self.Z3[2],self.Z4[2],self.Z5[2]]])

        return self.jacobian