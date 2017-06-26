#!/usr/bin/env python

# Hopper environment originally built for the MuJoCo physics engine

import gym
from gym import spaces
import numpy as np
import pybullet as p
import sys
import time
from MJCFCommon import PybulletMujocoEnv

np.set_printoptions(precision=3, suppress=True, linewidth=10000)

def add_opts(parser):
    pass
    # add some parser arguments such as the ones below
    parser.add_argument('--delay', type=float, default=0.0)
    parser.add_argument('--action-repeats', type=int, default=2,
                      help="number of action repeats")
    parser.add_argument('--steps-per-repeat', type=int, default=5,
                      help="number of sim steps per repeat")
    parser.add_argument('--max-episode-len', type=int, default=200,
                      help="maximum episode length")

class MJCFHopperv0Env(PybulletMujocoEnv):
    
    def __init__(self, opts):
        self.gui = opts.gui
        self.max_episode_len = opts.max_episode_len
        self.delay = opts.delay if self.gui else 0.0
        
        # do some parameter setting from your parser arguments and add other configurations
 
        # how many time to repeat each action per step().
        # and how many sim steps to do per state capture
        # (total number of sim steps = action_repeats * steps_per_repeat
        self.repeats = opts.action_repeats
        self.steps_per_repeat = opts.steps_per_repeat

        # setup bullet
        p.connect(p.GUI if self.gui else p.DIRECT)
        p.setGravity(0,0,-9.81)
        
        PybulletMujocoEnv.__init__(self, "envs/models/mjcf/hopper.xml", "walker", 0.02, frame_skip=2, action_dim=3, obs_dim=8, repeats=self.repeats)

        self.torso = self.parts["torso"]
        self.foot = self.parts["foot"]
        self.thigh_joint = self.joints["thigh_joint"]
        self.leg_joint = self.joints["leg_joint"]
        self.foot_joint = self.joints["foot_joint"]
        
        self.metadata = {
            'discrete_actions' : False,
            'continuous_actions': True,
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second': int(np.round(1.0 / self.timestep / self.frame_skip))
        }
        
    def configureActions(self, discrete_actions):
        pass        
        
    def _configure(self, display=None):
        pass

    def _render(self, mode='human', close=False):
        pass
        
    def _step(self, action):
        assert( np.isfinite(action).all() )
        
        if self.done:
            print >>sys.stderr, "Why is step called when the episode is done?"
            return np.copy(self.state), 0, True, {}
        # choose your next action
        # do some out of bounds checks to reset the environment and agent
        # calculate the reward for your agent
 
        info = {}
 
        # step simulation forward. at the end of each repeat we set part of the step's
        # state by capture parts' states in some form.
        for r in xrange(self.repeats):
            for _ in xrange(self.steps_per_repeat):
                p.stepSimulation()
                self.thigh_joint.set_torque( 75*float(np.clip(action[0], -1, +1)) )
                self.leg_joint.set_torque( 75*float(np.clip(action[1], -1, +1)) )
                self.foot_joint.set_torque(75*float(np.clip(action[2],-1, +1)))
                if self.delay > 0:
                    time.sleep(self.delay)
            self.set_state_element_for_repeat(r)
        self.steps += 1

        # check for end of episode (by length)
        if self.steps >= self.max_episode_len:
            info['done_reason'] = 'episode length'
            self.done = True
        
        self.speed = self.foot.current_position()[0]-self.last_position
        self.last_position = self.foot.current_position()[0]
        self.rewards = [self.speed]
        
        # return observation
        return self.state, self.speed, self.done, info
        
    def set_state_element_for_repeat(self, repeat):
        # in low dim case state is (R, obs_dim)
        # R -> repeat, obs_dim -> obs_dim d pose
        height = self.foot.current_position()[2]
        theta, self.theta_dot = self.thigh_joint.get_state()
        gamma, self.gamma_dot = self.leg_joint.get_state()
        phi, self.phi_dot = self.leg_joint.get_state()
        self.state[repeat] = np.array([
            height,
            theta,
            self.theta_dot,
            gamma,
            self.gamma_dot,
            phi,
            self.phi_dot,
            self.speed
            ])
        
    def _reset(self):
        # reset your environment
              
        # reset state
        self.steps = 0
        self.done = False
        self.speed = 0

        self.torso.reset_pose(self.torso.initialPosition, self.torso.initialOrientation)
        self.thigh_joint.reset_position(self.np_random.uniform( low=self.thigh_joint.lowerLimit, high=self.thigh_joint.upperLimit ))
        self.leg_joint.reset_position(self.np_random.uniform( low=self.leg_joint.lowerLimit, high=self.leg_joint.upperLimit ))
        self.foot_joint.reset_position(self.np_random.uniform( low=self.foot_joint.lowerLimit, high=self.foot_joint.upperLimit ))
        
        self.last_position = self.foot.current_position()[0]
        # bootstrap state by running for all repeats
        for i in xrange(self.repeats):
            self.set_state_element_for_repeat(i)
 
        # return this state
        return np.copy(self.state)


###
