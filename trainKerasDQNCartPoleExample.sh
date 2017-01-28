#!/bin/sh
python runTrainer.py --agent=KerasDQNAgent --env=CartPolev0Env --train-for=10000000 --test-for=0 --gui --save-file=KerasDQN-CartPolev0-$(date +%y%m%d%H%M%S).h5

