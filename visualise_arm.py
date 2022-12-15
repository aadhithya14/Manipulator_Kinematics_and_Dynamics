import numpy as np
import pandas
import pybullet
import os
from forward_kinematics import Forward_Kinematics
from controllers.PD import PDControl
import pybullet_data
import time

class Env():
    def __init__(self,target_position,id):
        self.target_position=target_position   
        self.id=id

    def step(self,maxForce=1000):
        #self.visforward_kinematics=Forward_Kinematics()
        self.controller=PDControl(self.target_position,self.id)
        self.controller.act()
        #time.sleep(10)
        #pybullet.stepSimulation()

    def get_joint_ids(self):
        return [0,1,2,3,4,5]




        





