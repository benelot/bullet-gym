'''
Created on Apr 4, 2017

@author: benelot
'''

import gym
import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import pybullet as p

def state_fields_of_pose_of(body_id, link_id=-1): # a method you will most probably need a lot to get pose and orientation
    if link_id == -1:
        (x,y,z), (a,b,c,d) = p.getBasePositionAndOrientation(body_id)
    else:
        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(body_id, link_id)
    return np.array([x,y,z,a,b,c,d])

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
        self.parts, self.joints, = self.getScene(p.loadMJCF(model_xml))
        self.robot_name = robot_name
        self.dt = timestep * frame_skip
        self.metadata = {
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second': int(np.round(1.0 / timestep / frame_skip))
            }
        self._seed()
        
    def getScene(self, bodies):
        parts = {}
        joints = {}
        for i in range(len(bodies)):
            for j in range(p.getNumJoints(bodies[i])):
                _,joint_name,_,_,_,_,_,_,_,_,_,_,part_name = p.getJointInfo(bodies[i],j)
                joints[joint_name] = Joint(bodies,i,j)
                joints[joint_name].disable_motor()
                
                parts[part_name] = BodyPart(bodies,i,j)
        return parts, joints

    def _seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]



class BodyPart:
    def __init__(self, bodies, bodyIndex, bodyPartIndex):
        self.bodies = bodies
        self.bodyIndex = bodyIndex
        self.bodyPartIndex = bodyPartIndex
        self.initialPosition = self.current_position()
        self.initialOrientation = self.current_orientation()
    
    def get_pose(self):
        return state_fields_of_pose_of(self.bodies[self.bodyIndex], self.bodyPartIndex)
    
    def current_position(self):
        return self.get_pose()[:3]
    
    def current_orientation(self):
        return self.get_pose()[3:]
    
    def reset_position(self, position):
        p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, self.get_orientation())
        
    def reset_orientation(self, orientation):
        p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], self.get_position(), orientation)
        
    def reset_pose(self, position, orientation):
        p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, orientation)
    
class Joint:
    def __init__(self, bodies, bodyIndex, jointIndex):
        self.bodies = bodies
        self.bodyIndex = bodyIndex
        self.jointIndex = jointIndex
        _,_,_,_,_,_,_,_,self.lowerLimit, self.upperLimit,_,_,_ = p.getJointInfo(self.bodies[self.bodyIndex], self.jointIndex)
        
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
    
    def reset_position(self, position, velocity):
        self.set_position(position)
        self.set_velocity(velocity)
        self.disable_motor()

    def disable_motor(self):
        p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,controlMode=p.VELOCITY_CONTROL, force=0)