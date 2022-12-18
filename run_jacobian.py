from controllers.PD import Control
from inverse_kinematics import Inverse_Kinematics
from turtle import forward
import numpy as np
import argparse
import os
from sympy import Inverse
from geometric_jacobian import Geometric_Jacobian
from visualise_arm import Env
import pybullet
import pybullet_data
from sympy import *

if __name__=='__main__':

    physicsClient = pybullet.connect(pybullet.GUI)
    #done=False
    #px, py, pz = list(map(float,input("\nEnter px, py, pz : ").strip().split()))[:3]
    #roll, pitch , yaw= list(map(float,input("\nEnter roll, pitch, yaw : ").strip().split()))[:3]
    
    #joint_angles=inverse_kinematics.get_angles(px,py,pz,roll,pitch,yaw)
    #print(joint_angles)
    pybullet.resetDebugVisualizerCamera(2., 180, 0., [0.52, 0.2, np.pi/4.])
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    urdf_path=str(os.path.dirname(os.path.abspath(__file__)))+'/ele_robot_s_1/urdf/'
    base_position=np.zeros(3)
    base_orientation=np.array([0,0,0,1])
    pybullet.loadURDF(urdf_path+"/plane_with_restitution.urdf", [0, 0, 0], useFixedBase=True)

        #or p.DIRECT for non-graphical version p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally p.setGravity(0,0,-10) planeId = p.loadURDF("plane.urdf")
    id=pybullet.loadURDF(urdf_path+'/ele_robot_s_1.urdf',base_position,base_orientation,useFixedBase=True, flags=pybullet.URDF_USE_INERTIA_FROM_FILE)
    pybullet.getNumJoints(id)
    joint_angle=np.zeros(6)
    joint_ids=[0,1,2,3,4,5]
    #pybullet.setGravity(0,0,-10)
    pybullet.setGravity(0., 0., -10.)

    maxIters=1000
    joint_angle1_id = pybullet.addUserDebugParameter("JointAngle1",-3.14,3.14,0.1)
    joint_angle2_id = pybullet.addUserDebugParameter("JointAngle2",-3.14,3.14,0.1)
    joint_angle3_id = pybullet.addUserDebugParameter("JointAngle3",-3.14,3.14,0.1)
    joint_angle4_id = pybullet.addUserDebugParameter("JointAngle4",-3.14,3.14,0.1)
    joint_angle5_id = pybullet.addUserDebugParameter("JointAngle5",-3.14,3.14,0.1)
    joint_angle6_id = pybullet.addUserDebugParameter("JointAngle6",-3.14,3.14,0.1)
    joint_vel1_id = pybullet.addUserDebugParameter("JointVelocity1",-3.14,3.14,0.1)
    joint_vel2_id = pybullet.addUserDebugParameter("JointVelocity2",-3.14,3.14,0.1)
    joint_vel3_id = pybullet.addUserDebugParameter("JointVelocity3",-3.14,3.14,0.1)
    joint_vel4_id = pybullet.addUserDebugParameter("JointVelocity4",-3.14,3.14,0.1)
    joint_vel5_id = pybullet.addUserDebugParameter("JointVelocity5",-3.14,3.14,0.1)
    joint_vel6_id = pybullet.addUserDebugParameter("JointVelocity6",-3.14,3.14,0.1)
    #video_id=pybullet.startStateLogging(loggingType=pybullet.STATE_LOGGING_VIDEO_MP4,fileName="forward.mp4")
    
    #print("***")
    for _ in range(10000):
      pybullet.stepSimulation()
      joint_angle1 = pybullet.readUserDebugParameter(joint_angle1_id)
      joint_angle2 = pybullet.readUserDebugParameter(joint_angle2_id)
      joint_angle3 = pybullet.readUserDebugParameter(joint_angle3_id)
      joint_angle4 = pybullet.readUserDebugParameter(joint_angle4_id)
      joint_angle5 = pybullet.readUserDebugParameter(joint_angle5_id)
      joint_angle6 = pybullet.readUserDebugParameter(joint_angle6_id)
      joint_vel1 = pybullet.readUserDebugParameter(joint_vel1_id)
      joint_vel2 = pybullet.readUserDebugParameter(joint_vel2_id)
      joint_vel3 = pybullet.readUserDebugParameter(joint_vel3_id)
      joint_vel4 = pybullet.readUserDebugParameter(joint_vel4_id)
      joint_vel5 = pybullet.readUserDebugParameter(joint_vel5_id)
      joint_vel6 = pybullet.readUserDebugParameter(joint_vel6_id)
      theta= np.array([joint_angle1,joint_angle2,joint_angle3,joint_angle4,joint_angle5,joint_angle6])
      joint_velocity= np.array([joint_vel1,joint_vel2,joint_vel3,joint_vel4,joint_vel5,joint_vel6])
      jacobian=Geometric_Jacobian(theta)
      jacobian_matrix=jacobian.get_jacobian()
      
      print("The Jacobian Matrix is",jacobian_matrix)   
      endeff_velocities=jacobian_matrix*Matrix([joint_vel1,joint_vel2,joint_vel3,joint_vel4,joint_vel5,joint_vel6])
      print("The endeffector velocities are",endeff_velocities)
      env=Env(theta,id,joint_velocity)
      env.step()
     