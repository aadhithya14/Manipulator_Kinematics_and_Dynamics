import numpy as np
from forward_kinematics import  Forward_Kinematics
from sympy import *
import sympy

class Jacobian():
    def __init__(self):
        self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6=symbols('theta1,theta2,theta3,theta4,theta5,theta6')
        self.theta=[self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6]
        self.forward_kinematics=Forward_Kinematics(self.theta)
        self.final_transformation=self.forward_kinematics.get_final_transformation_matrix()

    def get_jacobian(self):
        print(self.final_transformation[0][3])
        print(self.final_transformation[1][3])
        print(self.final_transformation[2][3])
        self.dx1=diff(self.final_transformation[0][3],self.theta1)
        self.dx2=diff(self.final_transformation[0][3],self.theta2)
        self.dx3=diff(self.final_transformation[0][3],self.theta3)
        self.dx4=diff(self.final_transformation[0][3],self.theta4)
        self.dx5=diff(self.final_transformation[0][3],self.theta[4])
        self.dx6=diff(self.final_transformation[0][3],self.theta[5])
        self.dy1=diff(self.final_transformation[1][3],self.theta[0])
        self.dy2=diff(self.final_transformation[1][3],self.theta[1])
        self.dy3=diff(self.final_transformation[1][3],self.theta[2])
        self.dy4=diff(self.final_transformation[1][3],self.theta[3])
        self.dy5=diff(self.final_transformation[1][3],self.theta[4])
        self.dy6=diff(self.final_transformation[1][3],self.theta[5])
        self.dz1=diff(self.final_transformation[2][3],self.theta[0])
        self.dz2=diff(self.final_transformation[2][3],self.theta[1])
        self.dz3=diff(self.final_transformation[2][3],self.theta[2])
        self.dz4=diff(self.final_transformation[2][3],self.theta[3])
        self.dz5=diff(self.final_transformation[2][3],self.theta[4])
        self.dz6=diff(self.final_transformation[2][3],self.theta[5])

        self.jacobian_matrix=Matrix([[self.dx1,self.dx2,self.dx3,self.dx4,self.dx5,self.dx6],[self.dy1,self.dy2,self.dy3,self.dy4,self.dy5,self.dy6],[self.dz1,self.dz2,self.dz3,self.dz4,self.dz5,self.dz6]])   
        

    def subs_jacobian(self,theta):
        self.final_jacobian=self.jacobian_matrix.evalf(subs={self.theta1:theta[0],self.theta2:theta[1],self.theta3:theta[2],self.theta4:theta[3],self.theta5:theta[4],self.theta6:theta[5]})
        return self.final_jacobian
