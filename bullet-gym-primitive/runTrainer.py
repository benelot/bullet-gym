#!/usr/bin/env python

import Trainer

import argparse # parse input arguments
import numpy as np
import time

trainer = Trainer.Trainer()

# initialize random seed
np.random.seed(int(time.time()))

parser = argparse.ArgumentParser()

# add all parsing options
Trainer.add_opts(parser)

# parse arguments
opts, unknown = parser.parse_known_args()
print "OPTS", opts
print "UNKNOWN", unknown

    
trainer.setup_exercise(opts)
