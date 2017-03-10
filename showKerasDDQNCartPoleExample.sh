#!/bin/sh
python runTrainer.py --agent=KerasDDQNAgent --env=CartPolev0Env --train-for=0 --test-for=10000000 --delay=0.005 --gui --show-test --load-file=checkpoints/KerasDDQN-CartPolev0-chkpt-2.h5

