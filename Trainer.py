#!/usr/bin/env python

import argparse # parse input arguments
import numpy as np # arithmetic library
import time

np.set_printoptions(precision=3, suppress=True, linewidth=10000)

def add_opts(parser):
	parser.add_argument('--agent', type=str, default="KerasDQNAgent", help="Agent to be trained with")
	parser.add_argument('--env', type=str, default="2DDetachedCartPolev0Env", help="Environment to be trained in")
	parser.add_argument('--train-for', type=int, default=100)
	parser.add_argument('--test-for', type=int, default=0)
	parser.add_argument('--show-train', action='store_true')
	parser.add_argument('--show-test', action='store_true')
	parser.add_argument('--load-file', type=str, default=None)
	parser.add_argument('--save-file', type=str, default=None)
	parser.add_argument('--gui', action='store_true', help="Should GUI be shown during this session?")
	
	opts, unknown = parser.parse_known_args() # parse agent and environment to add their opts

	exec "from agents import %s" % opts.agent # import agent type
	exec "from envs import %s" % opts.env # import env type
	exec "%s.add_opts(parser)" % opts.agent
	exec "%s.add_opts(parser)" % opts.env 
	

class Trainer:

	def __init__(self):
		pass
		
	def setup_exercise(self, opts):
		
		exec "from agents import %s" % opts.agent # import agent type
		exec "from envs import %s" % opts.env # import env type
		
		# setup agent
		agent = 0 # just to mute the ide
		exec "agent = %s.%s(opts)" % (opts.agent, opts.agent)

		# setup environment
		env = 0 # just to mute the ide
		exec "env = %s.%s(opts)" % (opts.env, opts.env)
		
		if agent.metadata['discrete_actions'] != env.metadata['discrete_actions']:
			print "Incompatible agent/environment pair!"
		
		# configurations
		env.seed(int(time.time()))
		env.configureActions(agent.metadata['discrete_actions']) # configure environment to accepts discrete actions
		if agent.metadata['discrete_actions'] == True:
			agent.configure(env.observation_space.shape, env.action_space.n) # configure agent to use the environment properties
		else:
			agent.configure(env.observation_space.shape, env.action_space.shape[0]) # configure agent to use the environment properties
		
		if opts.load_file is not None:
			print "loading weights from [%s]" % opts.load_file
			agent.load_weights(opts.load_file)
		
		# Okay, now it's time to learn something! We visualize the training here for show, but this
		# slows down training quite a lot. You can always safely abort the training prematurely using
		# Ctrl + C.
		agent.train(env, nb_steps=opts.train_for, visualize=opts.show_train, verbosity=1)
		
		# After training is done, we save the final weights.
		if opts.save_file is not None:
			print "saving weights to [%s]" % opts.save_file
			agent.save_weights(opts.save_file, overwrite=True)
			
		
		# Finally, evaluate our algorithm.
		agent.test(env, nb_episodes=opts.test_for, visualize=opts.show_test)
