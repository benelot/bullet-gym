#!/bin/sh
python runTrainer.py --agent=KerasDDQNAgent --env=CartPolev0Env --train-for=10000000 --test-for=0 --gui --save-file=KerasDDQN-CartPolev0-$(date +%y%m%d%H%M%S).h5

