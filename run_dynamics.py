from controllers.PD import PDControl
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
from sympy import*

if __name__=='__main__':

    physicsClient = pybullet.connect(pybullet.GUI)
    #done=False
    #px, py, pz = list(map(float,input("\nEnter px, py, pz : ").strip().split()))[:3]
    #roll, pitch , yaw= list(map(float,input("\nEnter roll, pitch, yaw : ").strip().split()))[:3]
    
    #joint_angles=inverse_kinematics.get_angles(px,py,pz,roll,pitch,yaw)
    #print(joint_angles)
    inverse_kinematics=Inverse_Kinematics()
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
    targetPosXId =pybullet.addUserDebugParameter("targetPosX",-50,50,10)
    targetPosYId = pybullet.addUserDebugParameter("targetPosY",-300,300,10)
    targetPosZId = pybullet.addUserDebugParameter("targetPosZ",0,180,10)
    Roll = pybullet.addUserDebugParameter("Roll",-10,10,-0.1)
    Pitch = pybullet.addUserDebugParameter("Pitch",-10,10,-0.1)
    Yaw = pybullet.addUserDebugParameter("Yaw",-10,10,-0.1)
    targetForceX = pybullet.addUserDebugParameter("targetForceX",-5,5,1)
    targetForceY = pybullet.addUserDebugParameter("targetForceY",-5,5,1)
    targetForceZ = pybullet.addUserDebugParameter("targetForceZ",0,10,2)
    targetMomentx = pybullet.addUserDebugParameter("targetMomentx",-5,5,1)
    targetMomenty = pybullet.addUserDebugParameter("targetMomenty",-5,5,1)
    targetMomentz = pybullet.addUserDebugParameter("targetMomentz",-5,5,1)
    #video_id=pybullet.startStateLogging(loggingType=pybullet.STATE_LOGGING_VIDEO_MP4,fileName="forward.mp4")
    
    #print("***")
    for _ in range(10000):
      pybullet.stepSimulation()
      targetPosX = pybullet.readUserDebugParameter(targetPosXId)
      targetPosY = pybullet.readUserDebugParameter(targetPosYId)
      targetPosZ = pybullet.readUserDebugParameter(targetPosZId)
      targetRoll = pybullet.readUserDebugParameter(Roll)
      targetPitch = pybullet.readUserDebugParameter(Pitch)
      targetYaw = pybullet.readUserDebugParameter(Yaw)
      targetForce_X = pybullet.readUserDebugParameter(targetForceX)
      targetForce_Y = pybullet.readUserDebugParameter(targetForceY)
      targetForce_Z = pybullet.readUserDebugParameter(targetForceZ)
      targetMoment_X = pybullet.readUserDebugParameter(targetMomentx)
      targetMoment_Y = pybullet.readUserDebugParameter(targetMomenty)
      targetMoment_Z = pybullet.readUserDebugParameter(targetMomentz)
      #theta= np.array([joint_angle1,joint_angle2,joint_angle3,joint_angle4,joint_angle5,joint_angle6])
      #theta =inverse_kinematics.get_angles(targetForce_X,targetForceY,targetForce_Z,targetMoment_X,targetMoment_Y,targetMoment_Z)
      theta =inverse_kinematics.get_angles(targetPosX,targetPosY,targetPosZ,targetRoll,targetPitch,targetYaw)
      jacobian=Geometric_Jacobian(theta)
      jacobian_matrix=jacobian.get_jacobian()
      
      transpose_jacobian=jacobian_matrix.transpose()
      Torques=transpose_jacobian*Matrix([targetForceX,targetForceY,targetForceZ,targetMoment_X,targetMoment_Y,targetMoment_Z])

      env=Env(Torques,id,control_type='torque')
      env.step()