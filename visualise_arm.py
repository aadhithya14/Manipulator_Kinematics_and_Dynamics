import numpy as np
import pandas
import pybullet
import os
from forward_kinematics import Forward_Kinematics
from controllers.PD import Control
import pybullet_data
import time

class Env():
    def __init__(self,target_position,id,target_torque=np.zeros(6),target_velocity=np.zeros(6),control_type='torque'):
        self.target_position=target_position   
        self.target_velocity=target_velocity
        self.target_torque=target_torque
        self.id=id
        self.control_type=control_type

    def step(self,maxForce=1000):
        #self.visforward_kinematics=Forward_Kinematics()
        self.controller=Control(self.target_position,self.id,self.target_torque,self.target_velocity,self.control_type)
        self.controller.act()
        #time.sleep(10)
        #pybullet.stepSimulation()

    def get_joint_ids(self):
        return [0,1,2,3,4,5]




        





