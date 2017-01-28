#!/bin/sh
python runTrainer.py --agent=KerasDDQNAgent --env=Detached2DCartPolev0Env --train-for=10000000 --test-for=0 --gui --save-file=KerasDDQN-D2DCartPolev0-$(date +%y%m%d%H%M%S).h5

