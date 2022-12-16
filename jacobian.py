import numpy as np
from forward_kinematics import  Forward_Kinematics
from sympy import *
import sympy

class Jacobian():
    def __init__(self):
        self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6=symbols('theta1,theta2,theta3,theta4,theta5,theta6')
        self.forward_kinematics=Forward_Kinematics()
