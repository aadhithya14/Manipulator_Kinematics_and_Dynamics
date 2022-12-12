import numpy as np
import pandas
import math
import random

from sympy import *
from robot_parameters import Robot_Parameters

#Initialise transformation matrices

class Forward_Kinematics():

    def __init__(self,theta):
        self.T1=np.zeros((4,4))
        self.T2=np.zeros((4,4))
        self.T3=np.zeros((4,4))
        self.T4=np.zeros((4,4))
        self.T5=np.zeros((4,4))
        self.T6=np.zeros((4,4))
        self.theta=theta #all thetas collectively represented as vector
        #print(self.theta)
        self.dh_parameters=Robot_Parameters()
        self.alpha=self.dh_parameters.get_alpha()#all alphas collectively represented as 
        self.a=self.dh_parameters.get_a()
        self.d=self.dh_parameters.get_d()
        #Input DH parameters (theta1,d,alpha,a) in the same order to avoid mismatch.
        self.dh=np.array([[self.theta[0],self.d[0],self.alpha[0],self.a[0]],[self.theta[1],self.d[1],self.alpha[1],self.a[1]],[self.theta[2],self.d[2],self.alpha[2],self.a[2]],[self.theta[3],self.d[3],self.alpha[3],self.a[3]],[self.theta[4],self.d[4],self.alpha[4],self.a[4]],[self.theta[5],self.d[5],self.alpha[5],self.a[5]]])

    def get_transformation_matrix(self):
        expr=cos(self.theta[0])
        val=cos(self.alpha[0])
        print(expr)
        print(val)
        self.T1=np.array([[cos(self.theta[0]),-cos(self.alpha[0])*(sin(self.theta[0])),sin(self.alpha[0])*sin(self.theta[0]),self.a[0]*cos(self.theta[0])],
                         [sin(self.theta[0]),cos(self.alpha[0])*cos(self.theta[0]),-sin(self.alpha[0])*cos(self.theta[0]),self.a[0]*sin(self.theta[0])],
                         [0, sin(self.alpha[0]), cos(self.alpha[0]),self.d[0]],
                         [0, 0, 0 ,1]])

        self.T2=np.array([[cos(self.theta[1]),-cos(self.alpha[1])*(sin(self.theta[1])),sin(self.alpha[1])*sin(self.theta[1]),self.a[1]*cos(self.theta[1])],
                         [sin(self.theta[1]),cos(self.alpha[1])*cos(self.theta[1]),-sin(self.alpha[1])*cos(self.theta[1]),self.a[1]*sin(self.theta[1])],
                         [0, sin(self.alpha[1]), cos(self.alpha[1]),self.d[1]],
                         [0, 0, 0 ,1]])

        self.T3=np.array([[cos(self.theta[2]),-cos(self.alpha[2])*(sin(self.theta[2])),sin(self.alpha[2])*sin(self.theta[2]),self.a[2]*cos(self.theta[2])],
                         [sin(self.theta[2]),cos(self.alpha[2])*cos(self.theta[2]),-sin(self.alpha[2])*cos(self.theta[2]),self.a[2]*sin(self.theta[2])],
                         [0, sin(self.alpha[2]), cos(self.alpha[2]),self.d[2]],
                         [0, 0, 0 ,1]])
        
        self.T4=np.array([[cos(self.theta[3]),-cos(self.alpha[3])*(sin(self.theta[3])),sin(self.alpha[3])*sin(self.theta[3]),self.a[3]*cos(self.theta[3])],
                         [sin(self.theta[3]),cos(self.alpha[3])*cos(self.theta[3]),-sin(self.alpha[3])*cos(self.theta[3]),self.a[3]*sin(self.theta[3])],
                         [0, sin(self.alpha[3]), cos(self.alpha[3]),self.d[3]],
                         [0, 0, 0 ,1]])

        self.T5=np.array([[cos(self.theta[4]),-cos(self.alpha[4])*(sin(self.theta[4])),sin(self.alpha[4])*sin(self.theta[4]),self.a[4]*cos(self.theta[4])],
                         [sin(self.theta[4]),cos(self.alpha[4])*cos(self.theta[4]),-sin(self.alpha[4])*cos(self.theta[4]),self.a[4]*sin(self.theta[4])],
                         [0, sin(self.alpha[4]), cos(self.alpha[4]),self.d[4]],
                         [0, 0, 0 ,1]])
        
        self.T6=np.array([[cos(self.theta[5]),-cos(self.alpha[5])*(sin(self.theta[5])),sin(self.alpha[5])*sin(self.theta[5]),self.a[5]*cos(self.theta[5])],
                         [sin(self.theta[5]),cos(self.alpha[5])*cos(self.theta[5]),-sin(self.alpha[5])*cos(self.theta[5]),self.a[5]*sin(self.theta[5])],
                         [0, sin(self.alpha[5]), cos(self.alpha[5]),self.d[5]],
                         [0, 0, 0 ,1]])

        self.final_transformation=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3,self.T4,self.T5,self.T6]))

    def get_end_effector_positions(self):
        x= self.final_transformation[0][3]
        y= self.final_transformation[1][3]
        z= self.final_transformation[2][3]
        return x,y,z

    def get_joint_positions(self):
        x1= 0
        y1= 0
        z1= 0
        x2= self.T1[0][3]
        y2= self.T1[1][3]
        z2= self.T1[2][3]
        x3=np.dot(self.T1[0][3],self.T2[0][3])
        y3= np.dot(self.T1[0][3],self.T2[0][3])
        z3= np.dot(self.T1[0][3],self.T2[0][3])
        x4=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3]))[0][3]
        y4=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3]))[1][3]
        z4=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3]))[2][3]
        x5=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3,self.T4]))[0][3]
        y5=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3,self.T4]))[1][3]
        z5=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3,self.T4]))[2][3]
        x6=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3,self.T4,self.T5]))[0][3]
        y6=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3,self.T4,self.T5]))[1][3]
        z6=np.linalg.multi_dot(np.array([self.T1,self.T2,self.T3,self.T4,self.T5]))[2][3]
        self.joint_positions=[[x1,y1,z1],[x2,y2,z2],[x3,y3,z3],[x4,y4,z4],[x5,y5,z5],[x6,y6,z6]]
        return self.joint_positions  

    def get_final_transformation_matrix(self):
        self.get_transformation_matrix()
        self.final_transformation=simplify(self.final_transformation)
        return self.final_transformation      

        







