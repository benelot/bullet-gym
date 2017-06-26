#!/usr/bin/python
import os 
os.system('python runTrainer.py --agent=KerasDDQNAgent --env=Detached2DCartPolev0Env --train-for=10000000 --test-for=0 --gui --save-file=checkpoints/KerasDDQN-D2DCartPolev0-$(date +%y%m%d%H%M%S).h5')

