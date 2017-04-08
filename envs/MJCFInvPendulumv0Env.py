#!/usr/bin/env python

# Inverted pendulum environment originally built for the MuJoCo physics engine

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
#     parser.add_argument('--action-force', type=float, default=50.0,
#                       help="magnitude of action force applied per step")
    parser.add_argument('--initial-force', type=float, default=55.0,
                      help="magnitude of initial push, in random direction")
    parser.add_argument('--no-random-theta', action='store_true')
    parser.add_argument('--swingup', action='store_true')
    parser.add_argument('--action-repeats', type=int, default=2,
                      help="number of action repeats")
    parser.add_argument('--steps-per-repeat', type=int, default=5,
                      help="number of sim steps per repeat")
    parser.add_argument('--max-episode-len', type=int, default=200,
                      help="maximum episode length for cartpole")
    parser.add_argument('--reward-calc', type=str, default='fixed',
                      help="'fixed': 1 per step. 'angle': 2*max_angle - ox - oy. 'action': 1.5 - |action|. 'angle_action': both angle and action")
def state_fields_of_pose_of(body_id, link_id=-1): # a method you will most probably need a lot to get pose and orientation
    if link_id == -1:
        (x,y,z), (a,b,c,d) = p.getBasePositionAndOrientation(body_id)
    else:
        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(body_id, link_id)
    return np.array([x,y,z,a,b,c,d])

class MJCFInvPendulumv0Env(PybulletMujocoEnv):
    
    def __init__(self, opts):
        self.gui = opts.gui
        self.max_episode_len = opts.max_episode_len
        self.delay = opts.delay if self.gui else 0.0
        
        # do some parameter setting from your parser arguments and add other configurations

        # threshold for angle from z-axis.
        # if x or y > this value we finish episode.
        self.angle_threshold = 0.26  # radians; ~= 15deg
        

        # initial push force. this should be enough that taking no action will always
        # result in pole falling after initial_force_steps but not so much that you
        # can't recover. see also initial_force_steps.
        self.initial_force = opts.initial_force
         
        # number of sim steps initial force is applied.
        # (see initial_force)
        self.initial_force_steps = 30
         
        # whether we do initial push in a random direction
        # if false we always push with along x-axis (simplee problem, useful for debugging)
        self.random_theta = not opts.no_random_theta
             
        # how many time to repeat each action per step().
        # and how many sim steps to do per state capture
        # (total number of sim steps = action_repeats * steps_per_repeat
        self.repeats = opts.action_repeats
        self.steps_per_repeat = opts.steps_per_repeat
        
        self.swingup = opts.swingup
        
        self.reward_calc = opts.reward_calc
        
        # setup bullet
        p.connect(p.GUI if self.gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)  # maybe you need gravity?
        
        PybulletMujocoEnv.__init__(self, "envs/models/mjcf/inverted_pendulum.xml", "walker", 0.02, frame_skip=2, action_dim=1, obs_dim=5, repeats=self.repeats)
        
        self.slider = self.joints["slider"]
        self.polejoint = self.joints["hinge"]
        
        self.metadata = {
            'discrete_actions' : False,
            'continuous_actions': True,
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second': int(np.round(1.0 / self.timestep / self.frame_skip))
        }
        
    def configureActions(self, discrete_actions):
        pass
        # if it is possible to switch actions, do this here
        
        # true if action space is discrete
        # false if action space is continuous
#         self.discrete_actions = discrete_actions
        
#         if self.discrete_actions:
#             self.action_space = spaces.Discrete(3)
#         else:
#             self.action_space = spaces.Box(-1.0, 1.0, shape=(1, 1))
        
#         # Our observations can be within this box
#         float_max = np.finfo(np.float32).max
#         self.observation_space = gym.spaces.Box(-float_max, float_max, self.state_shape)
        
        
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
 
        # based on action decide the x forces
        # continuous actions
        force = action[0]
 
        # step simulation forward. at the end of each repeat we set part of the step's
        # state by capture the cart & pole state in some form.
        for r in xrange(self.repeats):
            for _ in xrange(self.steps_per_repeat):
                p.stepSimulation()
                
                self.slider.set_torque(force)
                if self.delay > 0:
                    time.sleep(self.delay)
            self.set_state_element_for_repeat(r)
        self.steps += 1
 
        # Check for out of bounds by position or orientation on pole.
        # we (re)fetch pose explicitly rather than depending on fields in state.
        x1,vx1 = self.polejoint.get_state()
        
        if abs(x1) > self.angle_threshold:
            info['done_reason'] = 'out of orientation bounds'
            self.done = True
            reward = 0.0
        # check for end of episode (by length)
        if self.steps >= self.max_episode_len:
            info['done_reason'] = 'episode length'
            self.done = True
        
        if self.swingup:
            reward = np.cos(self.theta)
        else:
            reward = 1.0
            if self.reward_calc == "angle" or self.reward_calc == "angle_action":
                # clip to zero since angles can be past threshold
                reward += max(0, np.pi/2 - np.abs(x1))
            if self.reward_calc == "action" or self.reward_calc == "angle_action":
                # max norm will be sqr(2) ~= 1.4.
                # reward is already 1.0 to add another 0.5 as o0.1 buffer from zero
                reward += 0.5 - np.linalg.norm(action[0])
 
        # return observation
        return np.copy(self.state), reward, self.done, info
        
    def set_state_element_for_repeat(self, repeat):
        # in low dim case state is (R, 5)
        # R -> repeat, 5 -> 5d pose
        x1, vx1 = self.polejoint.get_state()
        x2, vx2 = self.slider.get_state()
        self.state[repeat] = np.array([
            x2, vx2,
            np.cos(x1), np.sin(x1), vx1
            ])
        
    def _reset(self):
        # reset your environment
        
        # reset state
        self.steps = 0
        self.done = False
 
        # reset pole on cart in starting poses
        self.slider.set_state(0, 0) # reset joint position of cart
        self.polejoint.set_state(0 if not self.swingup else 3.1415,0) # reset joint position of pole
        for _ in xrange(100): p.stepSimulation()
 
        # give a fixed force push in a random direction to get things going...
        theta = (np.random.random()*2-1) if self.random_theta else 0.0
        for _ in xrange(self.initial_force_steps):
            p.stepSimulation()
            self.polejoint.set_torque(theta)
            if self.delay > 0:
                time.sleep(self.delay)
 
        self.polejoint.disable_motor() # after having applied some initial torque, turn off motor

        
        # bootstrap state by running for all repeats
        for i in xrange(self.repeats):
            self.set_state_element_for_repeat(i)
 
        # return this state
        return np.copy(self.state)
