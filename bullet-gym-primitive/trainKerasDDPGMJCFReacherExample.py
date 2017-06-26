#!/usr/bin/python
import os 
os.system('python runTrainer.py --agent=KerasDDPGAgent --env=MJCFReacherv0Env --train-for=10000000 --test-for=0 --gui --save-file=checkpoints/KerasDDPG-MJCFReacherv0-$(date +%y%m%d%H%M%S).h5')

