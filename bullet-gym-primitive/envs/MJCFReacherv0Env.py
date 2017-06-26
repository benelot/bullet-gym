#!/usr/bin/env python

# Reacher environment originally built for the MuJoCo physics engine

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

class MJCFReacherv0Env(PybulletMujocoEnv):
    
    TARG_LIMIT = 0.27
    
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
        
        PybulletMujocoEnv.__init__(self, "envs/models/mjcf/reacher.xml", "walker", 0.02, frame_skip=2, action_dim=2, obs_dim=9, repeats=self.repeats)

        self.target_x = self.joints["target_x"]
        self.target_y = self.joints["target_y"]
        self.fingertip = self.parts["fingertip"]
        self.target    = self.parts["target"]
        self.central_joint = self.joints["joint0"]
        self.elbow_joint   = self.joints["joint1"]
        
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
                self.central_joint.set_torque( 0.07*float(np.clip(action[0], -1, +1)) )
                self.elbow_joint.set_torque( 0.07*float(np.clip(action[1], -1, +1)) )
                if self.delay > 0:
                    time.sleep(self.delay)
            self.set_state_element_for_repeat(r)
        self.steps += 1

        # check for end of episode (by length)
        if self.steps >= self.max_episode_len:
            info['done_reason'] = 'episode length'
            self.done = True
        
        self.to_target_vec = np.array(self.fingertip.current_position()) - np.array(self.target.current_position())
        reward_dist = - np.linalg.norm(self.to_target_vec)
        reward_ctrl = - np.square(action).sum()
        reward_rotation = - np.square(self.theta_dot*self.dt) - np.square(self.gamma_dot*self.dt)
        self.rewards = [reward_dist, reward_ctrl, reward_rotation]
        
        # return observation
        return self.state, reward_dist + reward_ctrl + reward_rotation, self.done, info
        
    def set_state_element_for_repeat(self, repeat):
        # in low dim case state is (R, obs_dim)
        # R -> repeat, obs_dim -> obs_dim d pose
        theta, self.theta_dot = self.central_joint.get_state()
        gamma, self.gamma_dot = self.elbow_joint.get_state()
        target_x, _ = self.target_x.get_state()
        target_y, _ = self.target_y.get_state()
        self.to_target_vec = np.array(self.fingertip.current_position()) - np.array(self.target.current_position())
        self.state[repeat] = np.array([
            target_x,
            target_y,
            np.cos(theta),
            np.sin(theta),
            self.theta_dot,
            gamma,
            self.gamma_dot,
            self.to_target_vec[0],
            self.to_target_vec[1]
            ])
        
    def _reset(self):
        # reset your environment
        
        # reset state
        self.steps = 0
        self.done = False
 
        # reset target to another location
        self.target_x.set_state(self.np_random.uniform( low=-self.TARG_LIMIT, high=self.TARG_LIMIT ),0)
        self.target_y.set_state(self.np_random.uniform( low=-self.TARG_LIMIT, high=self.TARG_LIMIT ),0)
        
        # reset joints to another location
        self.central_joint.reset_position(self.np_random.uniform( low=-np.pi, high=np.pi ))
        self.elbow_joint.reset_position(self.np_random.uniform( low=-np.pi, high=np.pi ))
        
        # bootstrap state by running for all repeats
        for i in xrange(self.repeats):
            self.set_state_element_for_repeat(i)
 
        # return this state
        return np.copy(self.state)


###
