#!/usr/bin/env python

# some initial description of the environment

import gym
from gym import spaces
import numpy as np
import pybullet as p
import sys
import time

np.set_printoptions(precision=3, suppress=True, linewidth=10000)

def add_opts(parser):
    pass
    # add some parser arguments such as the ones below
#     parser.add_argument('--delay', type=float, default=0.0)
#     parser.add_argument('--action-force', type=float, default=50.0,
#                       help="magnitude of action force applied per step")
#     parser.add_argument('--initial-force', type=float, default=55.0,
#                       help="magnitude of initial push, in random direction")
def state_fields_of_pose_of(body_id, link_id=-1): # a method you will most probably need a lot to get pose and orientation
    if link_id == -1:
        (x,y,z), (a,b,c,d) = p.getBasePositionAndOrientation(body_id)
    else:
        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(body_id, link_id)
    return np.array([x,y,z,a,b,c,d])

class TemplateEnv(gym.Env):
    
    def __init__(self, opts):
        self.gui = opts.gui
        self.max_episode_len = opts.max_episode_len
        self.delay = opts.delay if self.gui else 0.0
        
        self.metadata = {
            'discrete_actions' : True,
            'continuous_actions': True,
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second' : int(np.round(1.0 / 25.0))
        }
        
        # do some parameter setting from your parser arguments and add other configurations
        # setup state space
        
#         # threshold for pole position.
#         # if absolute x or y moves outside this we finish episode
#         self.pos_threshold = 2.4
# 
#         # threshold for angle from z-axis.
#         # if x or y > this value we finish episode.
#         self.angle_threshold = 0.26  # radians; ~= 15deg
#         
#         # force to apply per action simulation step.
#         # in the discrete case this is the fixed force applied
#         # in the continuous case each x/y is in range (-F, F)
#         self.action_force = opts.action_force
# 
#         # initial push force. this should be enough that taking no action will always
#         # result in pole falling after initial_force_steps but not so much that you
#         # can't recover. see also initial_force_steps.
#         self.initial_force = opts.initial_force
#         
#         # number of sim steps initial force is applied.
#         # (see initial_force)
#         self.initial_force_steps = 30
#         
#         # whether we do initial push in a random direction
#         # if false we always push with along x-axis (simplee problem, useful for debugging)
#         self.random_theta = not opts.no_random_theta
#             
#         # how many time to repeat each action per step().
#         # and how many sim steps to do per state capture
#         # (total number of sim steps = action_repeats * steps_per_repeat
#         self.repeats = opts.action_repeats
#         self.steps_per_repeat = opts.steps_per_repeat
#         
#         # in the low dimensional case obs space for problem is (R, 2, 7)
#         #  R = number of repeats
#         #  2 = two items; cart & pole
#         #  7d tuple for pos + orientation pose
#         self.state_shape = (self.repeats, 2, 7)
#         
#         # check reward type
#         assert opts.reward_calc in ['fixed', 'angle', 'action', 'angle_action']
#         self.reward_calc = opts.reward_calc
#         
#         # no state until reset.
#         self.state = np.empty(self.state_shape, dtype=np.float32)
        
        # setup bullet
        p.connect(p.GUI if self.gui else p.DIRECT)
#         p.setGravity(0, 0, -9.81)  # maybe you need gravity?
        
        # load your models
        
    def configureActions(self, discrete_actions):
        
        # if it is possible to switch actions, do this here
        
        # true if action space is discrete
        # false if action space is continuous
        self.discrete_actions = discrete_actions
        
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
# 
#         info = {}
# 
#         # based on action decide the x forces
#         fx = 0
#         if self.discrete_actions:
#             if action == 0:
#                 pass
#             elif action == 1:
#                 fx = self.action_force
#             elif action == 2:
#                 fx = -self.action_force
#             else:
#                 raise Exception("unknown discrete action [%s]" % action)
#         else: # continuous actions
#             fx = action[0] * self.action_force
# 
#         # step simulation forward. at the end of each repeat we set part of the step's
#         # state by capture the cart & pole state in some form.
#         for r in xrange(self.repeats):
#             for _ in xrange(self.steps_per_repeat):
#                 p.stepSimulation()
#                 p.applyExternalForce(self.cartpole, -1, (fx,0,0), (0,0,0), p.WORLD_FRAME)
#                 if self.delay > 0:
#                     time.sleep(self.delay)
#             self.set_state_element_for_repeat(r)
#         self.steps += 1
# 
#         # Check for out of bounds by position or orientation on pole.
#         # we (re)fetch pose explicitly rather than depending on fields in state.
#         (x, y, _z), orient,_,_,_,_ = p.getLinkState(self.cartpole, 0)
#         ox, oy, _oz = p.getEulerFromQuaternion(orient)  # roll / pitch / yaw
#         if abs(x) > self.pos_threshold or abs(y) > self.pos_threshold:
#             info['done_reason'] = 'out of position bounds'
#             self.done = True
#             reward = 0.0
#         elif abs(oy) > self.angle_threshold:
#             # TODO: probably better to do explicit angle from z?
#             info['done_reason'] = 'out of orientation bounds'
#             self.done = True
#             reward = 0.0
#         # check for end of episode (by length)
#         if self.steps >= self.max_episode_len:
#             info['done_reason'] = 'episode length'
#             self.done = True
# 
#         # calc reward, fixed base of 1.0
#         reward = 1.0
#         if self.reward_calc == "angle" or self.reward_calc == "angle_action":
#             # clip to zero since angles can be past threshold
#             reward += max(0, 2 * self.angle_threshold - np.abs(ox))
#         if self.reward_calc == "action" or self.reward_calc == "angle_action":
#             # max norm will be sqr(2) ~= 1.4.
#             # reward is already 1.0 to add another 0.5 as o0.1 buffer from zero
#             reward += 0.5 - np.linalg.norm(action[0])
# 
#         # return observation
#         return np.copy(self.state), reward, self.done, info
        
    def set_state_element_for_repeat(self, repeat):
        # in low dim case state is (R, 2, 7)
        # R -> repeat, 2 -> 2 objects (cart & pole), 7 -> 7d pose
        self.state[repeat][0] = state_fields_of_pose_of(self.cartpole, -1) # get position and orientation of cart
        self.state[repeat][1] = state_fields_of_pose_of(self.cartpole, 0) #  get position and orientation of pole
        
    def _reset(self):
        pass
        # reset your environment
        
#         # reset state
#         self.steps = 0
#         self.done = False
# 
#         # reset pole on cart in starting poses
#         p.resetBasePositionAndOrientation(self.cartpole, (0,0,0.08), (0,0,0,1)) # reset cart position and orientation
#         p.resetJointState(self.cartpole, 0, 0) # reset joint position of pole
#         for _ in xrange(100): p.stepSimulation()
# 
#         # give a fixed force push in a random direction to get things going...
#         theta = (np.random.random()*2-1) if self.random_theta else 0.0
#         for _ in xrange(self.initial_force_steps):
#             p.stepSimulation()
#             p.applyExternalForce(self.cartpole, 0, (theta, 0 , 0), (0, 0, 0), p.WORLD_FRAME)
#             if self.delay > 0:
#                 time.sleep(self.delay)
# 
#         # bootstrap state by running for all repeats
#         for i in xrange(self.repeats):
#             self.set_state_element_for_repeat(i)
# 
#         # return this state
#         return np.copy(self.state)
