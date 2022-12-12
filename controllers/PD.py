import numpy as np
from robot_parameters import Robot_Parameters
import pybullet

class PDControl():
    def __init__(self,action,robot_id,control_type='PD'):
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

    def get_cont_joint_ids():
        return [0,1,2,3,4,5]
        
    def act(self):
        q_diff     = self.pos_diff(self.q,self.action,self.joint_type)
        torque = self.kp * q_diff - self.kd * self.qdot
        self.torque_control(torque)

    def get_cont_joint_state(self):
        joint_pos = np.zeros(6)
        joint_vel = np.zeros(6)
        for i in range(6):
            joint_pos[i], joint_vel[i], _, _ = pybullet.getJointState(self.robot_id, i)
            print(joint_pos)
        return joint_pos, joint_vel
    
    def pos_diff(self, pos, des_pos, joint_range_type):
        pos_diff = np.zeros(pos.shape)
        for i in range(pos.shape[0]):
            if joint_range_type[i] == 'limited':
                pos_diff[i] = des_pos[i] - pos[i]
            else:
                pos_diff[i] = self.angle_diff(pos[i], des_pos[i])
        return pos_diff

    def torque_control(self, des_torque, no_clipping=False):
        self.des_torque = des_torque
        for i in range(6):
            pybullet.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=self.cont_joint_ids[i],
                controlMode=pybullet.TORQUE_CONTROL,
                force=self.des_torque[i])
        pybullet.stepSimulation()

    def angle_diff(pos, des_pos):
        if des_pos > pos:
            if des_pos - pos < np.pi:
                return des_pos - pos
            else:
                return des_pos - 2 * np.pi - pos
        else:
            if pos - des_pos < np.pi:
                return des_pos - pos
            else:
                return des_pos + 2 * np.pi - pos