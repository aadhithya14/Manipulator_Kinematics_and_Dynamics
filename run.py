from inverse_kinematics import Inverse_Kinematics
from turtle import forward
import numpy as np
import argparse

from sympy import Inverse
from forward_kinematics import Forward_Kinematics
from visualise_arm import Env

if __name__=='__main__':
    #theta = list(map(float,input("\nEnter the joint angles : ").strip().split()))[:6]
    #forward_kinematics=Forward_Kinematics(theta)
    px, py, pz = list(map(float,input("\nEnter px, py, pz : ").strip().split()))[:3]
    roll, pitch , yaw= list(map(float,input("\nEnter roll, pitch, yaw : ").strip().split()))[:3]
    px=0.49792 
    py=1.3673
    pz=2.4988
    roll=0.361
    pitch=-0.078
    yaw=2.561
    inverse_kinematics=Inverse_Kinematics()
    joint_angles=inverse_kinematics.get_angles(px,py,pz,roll,pitch,yaw)
    print(joint_angles)
    env=Env(joint_angles)
    env.reset()
    while True:
        env.step()   