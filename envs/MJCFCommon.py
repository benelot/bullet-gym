'''
Created on Apr 4, 2017

@author: benelot
'''

import gym
from gym import spaces
import numpy as np
import pybullet as p

class PybulletMujocoEnv(gym.Env):
    def __init__(self, model_xml, robot_name, timestep, frame_skip, action_dim, obs_dim, repeats):
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(action_dim,))
        float_max = np.finfo(np.float32).max
        
        # obs space for problem is (R, obs_dim)
        #  R = number of repeats
        #  obs_dim d tuple
        self.state_shape = (repeats, obs_dim)
        self.observation_space = gym.spaces.Box(-float_max, float_max, shape=self.state_shape)
        # no state until reset.
        self.state = np.empty(self.state_shape, dtype=np.float32)
        self.frame_skip = frame_skip
        self.timestep = timestep
        self.model_xml = model_xml
        self.bodies, self.joints, = self.getScene(p.loadMJCF(model_xml))
        self.robot_name = robot_name
        self.dt = timestep * frame_skip
        self.metadata = {
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second': int(np.round(1.0 / timestep / frame_skip))
            }
        self._seed()
        
    def getScene(self, bodies):
        joints = {}
        for i in range(len(bodies)):
            for j in range(p.getNumJoints(bodies[i])):
                _,name,_,_,_,_,_,_,_,_,_,_ = p.getJointInfo(bodies[i],j)
                joints[name] = Joint(bodies,i,j)
                joints[name].disable_motor()
        return bodies, joints

    def _seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]



class BodyPart:
    def __init__(self):
        pass
    
class Joint:
    def __init__(self, bodies, bodyIndex, jointIndex):
        self.bodies = bodies
        self.bodyIndex = bodyIndex
        self.jointIndex = jointIndex
        
    def set_state(self, x, vx):
        p.resetJointState(self.bodies[self.bodyIndex], self.jointIndex, x, vx)
        
    def get_state(self):
        x, vx,_,_ = p.getJointState(self.bodies[self.bodyIndex],self.jointIndex)
        return x, vx
    
    def set_position(self, position):
        p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,p.POSITION_CONTROL, targetPosition=position)
    
    def set_velocity(self, velocity):
        p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,p.VELOCITY_CONTROL, targetVelocity=velocity)
    
    def set_torque(self, torque):
        p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,p.TORQUE_CONTROL, force=torque)
        
    def disable_motor(self):
        p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,controlMode=p.VELOCITY_CONTROL, force=0)