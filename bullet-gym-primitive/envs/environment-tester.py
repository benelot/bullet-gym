#!/usr/bin/env python


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

fixedTimeStep = 0.001

import argparse # parse input arguments
import numpy as np

parser = argparse.ArgumentParser()

opts_env = "Motionv0Env"

exec "import %s" % opts_env # import env type

exec "%s.add_opts(parser)" % opts_env

add_opts(parser)

opts, unknown = parser.parse_known_args() # parse agent and environment to add their opts

opts.gui = True

# setup environment
env = 0 # just to mute the ide
exec "env = %s.%s(opts)" % (opts_env, opts_env)

env._reset()

env.configureActions(False)

while True:
	env._step(np.zeros(30))
