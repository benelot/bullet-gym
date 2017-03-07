#!/usr/bin/env python

# Environment to learn to use a certain morphology to succed within a certain reward function
# Rewards can be based on the achieved speed of the morphology, the height of jumping etc.

import gym
from gym import spaces
import numpy as np
import pybullet as p
import sys
import time
from RewardFunction import RewardFunction
from VelocityHelper import VelocityHelper

np.set_printoptions(precision=3, suppress=True, linewidth=10000)

def add_opts(parser):
    # add some parser arguments such as the ones below
    parser.add_argument('--delay', type=float, default=0.0)
    parser.add_argument('--action-gain', type=float, default=-1,
                      help="magnitude of action gain applied per step")
    parser.add_argument('--action-force', type=float, default=1.25,
                      help="magnitude of action gain applied per step")
    parser.add_argument('--gravity-force', type=float, default=-9.81,
                        help="amount of gravity")
    parser.add_argument('--control-type', type=str, default='position-control',
                        help="the type of control to move the morphology (position-control, velocity-control, torque-control)")
    parser.add_argument('--morphology-type', type=int, default=3,
                        help="Type of morphology; 1 = snake/2 = springy snake/3 = phantomx")
    parser.add_argument('--action-repeats', type=int, default=2,
                      help="number of action repeats")
    parser.add_argument('--steps-per-repeat', type=int, default=5,
                      help="number of sim steps per repeat")
    parser.add_argument('--max-episode-len', type=int, default=200,
                      help="maximum episode length for motion")
    parser.add_argument('--random-initial-position', action='store_true',
                        help="Should the morphology start in random initial position?")

def state_fields_of_pose_of(body_id, link_id=-1): # a method you will most probably need a lot to get pose and orientation
    if link_id == -1:
        (x,y,z), (a,b,c,d) = p.getBasePositionAndOrientation(body_id)
    else:
        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(body_id, link_id)
    return np.array([x,y,z,a,b,c,d])

def state_fields_of_pv_of(body_id, vHelper, link_id=-1):
    if link_id == -1:
        (x,y,z), (a,b,c,d) = p.getBasePositionAndOrientation(body_id)
        (vx,vy,vz), (va,vb,vc) = p.getBaseVelocity(body_id, 0)
    else:
        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(body_id, link_id)
        o = vHelper.getVelocities()
        (vx,vy,vz), (va,vb,vc) = (x-o[link_id+1][0],y-o[link_id+1][1],z-o[link_id+1][2]), (a-o[link_id+1][3],b-o[link_id+1][4],c-o[link_id+1][5])
    
    return np.array([x,y,z,a,b,c,d,vx,vy,vz,va,vb,vc])

def get_discrete_action_space_of(body_id):
    num_joints = p.getNumJoints(body_id)
    return spaces.Discrete(3^num_joints) # here is the state explosion!

def get_continuous_action_space_of(body_id): # the continuous version avoids the state explosion!
    num_joints = p.getNumJoints(body_id)
    return spaces.Box(-1.0, 1.0, shape=(num_joints,))

class Motionv0Env(gym.Env):
    
    def __init__(self, opts):
        self.gui = opts.gui
        self.max_episode_len = opts.max_episode_len
        self.delay = opts.delay if self.gui else 0.0
        
        self.metadata = {
            'discrete_actions' : True,
            'continuous_actions' : True,
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second' : int(np.round(1.0 / 25.0))
        }
        
        # do some parameter setting from your parser arguments and add other configurations
        
        self.control_type = opts.control_type
          
        # force to apply per action simulation step.
        # in the discrete case this is the fixed gain applied
        # in the continuous case each x/y is in range (-G, G)
        self.action_gain = opts.action_gain # scale of control (position, velocity)
        self.action_force = opts.action_force # Newton
        
        # how many time to repeat each action per step().
        # and how many sim steps to do per state capture
        # (total number of sim steps = action_repeats * steps_per_repeat
        self.repeats = opts.action_repeats
        self.steps_per_repeat = opts.steps_per_repeat
        
        self.random_initial_position = opts.random_initial_position
        
        # setup bullet
        p.connect(p.GUI if self.gui else p.DIRECT)
        p.setGravity(0, 0, opts.gravity_force)  # maybe you need gravity?
        
        p.loadURDF("envs/models/ground.urdf", 0,0,0, 0,0,0,1)
        
        # load your models
        if opts.morphology_type == 1:
            self.body = p.loadURDF("envs/models/simple-snakev0.urdf",0,0,2, 0,0,0,1)
        elif opts.morphology_type == 2:
            self.body = p.loadURDF("envs/models/springy-snakev0.urdf",0,0,2, 0,0,0,1)
        elif opts.morphology_type == 3:
            self.body = p.loadURDF("envs/models/phantomx/phantomx.urdf",0,0,2, 0,0,0,1)
            
        self.initPosition, self.initOrientation = p.getBasePositionAndOrientation(self.body)
            
        self.num_joints = p.getNumJoints(self.body)
        
        self.velocityHelper = VelocityHelper(self.body)
        
        self.reward = RewardFunction(self.body, RewardFunction.VelocityReward, RewardFunction.XAxis) # velocity in X axis dimension gets rewarded
        

        # in the low dimensional case obs space for problem is (R, num_links, 13)
        #  R = number of repeats
        #  num joints + 1 = number of links of snake
        #  13d tuple for pos + orientation + velocities (angular velocity in euler)
        self.state_shape = (self.repeats, p.getNumJoints(self.body)+1, 13)
        
        # no state until reset.
        self.state = np.empty(self.state_shape, dtype=np.float32)
        
    def configureActions(self, discrete_actions):
        
        # if it is possible to switch actions, do this here
         
        # true if action space is discrete
        # false if action space is continuous
        self.discrete_actions = discrete_actions
         
        if self.discrete_actions:
            self.action_space = get_discrete_action_space_of(self.body)
        else:
            self.action_space = get_continuous_action_space_of(self.body)
         
        # Our observations can be within this box
        float_max = np.finfo(np.float32).max
        self.observation_space = gym.spaces.Box(-float_max, float_max, self.state_shape)
        
        if self.discrete_actions:
            if self.control_type == "position-control":
                if self.action_gain == -1:
                    self.action_gain = 0.5 * np.pi
            elif self.control_type == "velocity-control":
                if self.action_gain == -1:
                    self.action_gain = 1;
            elif self.control_type == "torque-control":
                if self.action_gain == -1:
                    self.action_gain = 1.5; # torque control is even weirder
        else:
            if self.control_type == "position-control":
                if self.action_gain == -1:
                    self.action_gain = np.pi
            elif self.control_type == "velocity-control":
                if self.action_gain == -1:
                    self.action_gain = 1;
            elif self.control_type == "torque-control":
                if self.action_gain == -1:
                    self.action_gain = 0; # torque control is weird
        
        
    def _configure(self, display=None):
        pass

    def _seed(self, seed=None):
        pass

    def _render(self, mode='human', close=False):
        pass
        
    def _step(self, action):
        if self.done:
            print >>sys.stderr, "Why is step called when the episode is done?"
            return np.copy(self.state), 0, True, {}
    # choose your next action
    # do some out of bounds checks to reset the environment and agent
    # calculate the reward for your agent
 
        info = {}
         
        # check if action is NaN
        if np.isnan(action).any():
            info['done_reason'] = 'action is NaN'
            reward = 0
            self.done = True
        else:
            # based on action decide the actions for each joint
            joint_actions = -np.ones(self.num_joints)
            if self.discrete_actions: # the actions come out of one number encoding the actions to apply
                # example: (state explosion 3^2)
                # action 0 => (-1,-1) # action 1 => ( 0,-1) # action 2 => ( 1,-1)
                # action 3 => (-1, 0) # action 4 => ( 0, 0) # action 5 => ( 1, 0)
                # action 6 => (-1, 1) # action 7 => ( 0, 1) # action 8 => ( 1, 1)
                for joint in xrange(self.num_joints): # finds out the action to apply for each joint
                    if(action == 0):
                        break
                    action_sign = np.mod(action,3) - 1
                    action = np.floor_divide(action, 3)
            
                    joint_actions[joint] = action_sign * action * self.action_gain
            else: # continuous actions
                joint_actions = action * self.action_gain
      
            # step simulation forward. at the end of each repeat we set part of the step's
            # state by capturing the cart & pole state in some form.
            for r in xrange(self.repeats):
                for _ in xrange(self.steps_per_repeat):
                    p.stepSimulation()
                    for joint in xrange(self.num_joints):
                        if self.control_type == "position-control":
                            p.setJointMotorControl2(self.body,joint,p.POSITION_CONTROL, targetPosition = joint_actions[joint], force = self.action_force)
                        elif self.control_type == "velocity-control":
                            p.setJointMotorControl2(self.body,joint,p.VELOCITY_CONTROL, targetVelocity = joint_actions[joint], force = self.action_force)
                        elif self.control_type == "torque-control":
                            p.setJointMotorControl2(self.body,joint,p.TORQUE_CONTROL, force = joint_actions[joint])
                    if self.delay > 0:
                        time.sleep(self.delay)
                self.set_state_element_for_repeat(r)
            self.steps += 1
     
            # check for end of episode (by length)
            if self.steps >= self.max_episode_len:
                info['done_reason'] = 'episode length'
                self.done = True
 
            # calc reward
            reward = self.reward.getReward()
        
        # check if reward is NaN
        if np.isnan(reward):
            info['done_reason'] = 'reward is NaN'
            reward = 0
            self.done = True
 
        # return observation
        return np.copy(self.state), reward, self.done, info
        
    def set_state_element_for_repeat(self, repeat):
        # in low dim case state is (R, num_links, 10)
        # R -> repeat, num_links -> N objects (all links), 13 -> 14d pos+orientation+velocity
        for link in xrange(-1, self.num_joints):
            self.state[repeat][link] = state_fields_of_pv_of(self.body, self.velocityHelper, link) # get position, orientation and velocity of link
        
    def _reset(self):
        # reset your environment
        
        # reset state
        self.steps = 0
        self.done = False

        # reset morphology
        p.resetBasePositionAndOrientation(self.body, self.initPosition, self.initOrientation) # reset body position and orientation
        
        resetPosition = 0      
        for joint in xrange(self.num_joints):
            if self.random_initial_position:  
                resetPosition = np.random.random() * 2 * np.pi - np.pi
            p.resetJointState(self.body, joint, resetPosition) # reset joint position of joints
        for _ in xrange(100): p.stepSimulation()

        # bootstrap state by running for all repeats
        for i in xrange(self.repeats):
            self.set_state_element_for_repeat(i)
 
        # return this state
        return np.copy(self.state)
