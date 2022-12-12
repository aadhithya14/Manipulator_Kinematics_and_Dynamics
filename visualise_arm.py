import numpy as np
import pandas
import pybullet
import os
from forward_kinematics import Forward_Kinematics
from controllers.PD import PDControl
import pybullet_data
import time

class Env():
    def __init__(self,target_position):
        self.physicsClient = pybullet.connect(pybullet.GUI)
        self.target_position=target_position
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.urdf_path=str(os.path.dirname(os.path.abspath(__file__)))+'/ele_robot_s_1/urdf/'
        print(self.urdf_path)
        self.base_position=np.zeros(3)
        self.base_orientation=np.array([0,0,0,1])
        #or p.DIRECT for non-graphical version p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally p.setGravity(0,0,-10) planeId = p.loadURDF("plane.urdf")
        pybullet.setGravity(0,0,-10)
        self.id=pybullet.loadURDF(self.urdf_path+'/ele_robot_s_1.urdf',self.base_position,self.base_orientation)
        print(pybullet.getNumJoints(self.id))
        self.joint_ids=self.get_joint_ids()
        
    def reset(self):
        pybullet.resetSimulation()

    
    def step(self,maxForce=1000):
        #self.visforward_kinematics=Forward_Kinematics()
        self.controller=PDControl(self.target_position,self.id,)
        self.controller.act()

    def get_joint_ids(self):
        return [0,1,2,3,4,5]




        





