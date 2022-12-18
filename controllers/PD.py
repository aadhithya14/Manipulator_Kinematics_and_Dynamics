import numpy as np
from robot_parameters import Robot_Parameters
import pybullet

class Control():
    def __init__(self,action,robot_id,des_velocity=np.zeros(6),control_type='PD'):
        self.des_velocity=des_velocity
        self.control_type=control_type
        self.robot_id=robot_id
        self.ndof=6
        self.action=action
        self.max_torque=2.5
        self.q, self.qdot    = self.get_cont_joint_state()
        self.cont_joint_ids= self.get_cont_joint_ids()
        # kp_var (qdes - q) - kd_var qdot

        # example parametrization
        # kp_var_range: [  1.00, 10.00 ]
        # kd_var_range: [  0.05,  0.14 ]

       
        #action_gain = action[self.ndof:]
        
        self.robot_parameters=Robot_Parameters()
        self.kp,self.kd = self.robot_parameters.get_kpkd()
        self.joint_type='circular'

    def get_cont_joint_ids(self):
        return [0,1,2,3,4,5]
        
    def act(self):
        #q_diff     = self.pos_diff(self.q,self.action,self.joint_type)
        #torque = self.kp * q_diff - self.kd * self.qdot
        if self.control_type=='PD':
            self.position_control(self.action,self.des_velocity)
        else:
            self.torque_control(self.action)

    def get_cont_joint_state(self):
        joint_pos = np.zeros(6)
        joint_vel = np.zeros(6)
        for i in range(6):
            joint_pos[i], joint_vel[i], _, _ = pybullet.getJointState(self.robot_id, i)
        return joint_pos, joint_vel
    

    def position_control(self, des_action, des_vel=np.zeros(6), no_clipping=False):
        self.des_action = des_action
        self.des_vel= des_vel
        for i in range(6):
            pybullet.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=self.cont_joint_ids[i],
                controlMode=pybullet.POSITION_CONTROL,
                targetPosition=self.des_action[i],
                targetVelocity=self.des_vel[i],
                force=500,
                positionGain=0.03,
                velocityGain=1)
            
            """pybullet.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=self.cont_joint_ids[i],
                controlMode=pybullet.TORQUE_CONTROL,
                force=self.des_action[i])
            """
        pybullet.stepSimulation()
        
    def torque_control(self,des_torque):
        self.des_torque=des_torque
        for i in range(6):
            pybullet.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=self.cont_joint_ids[i],
                controlMode=pybullet.TORQUE_CONTROL,
                force=self.des_torque[i])
        pybullet.stepSimulation()
