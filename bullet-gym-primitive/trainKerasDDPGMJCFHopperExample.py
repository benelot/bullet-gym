#!/usr/bin/python
import os 
os.system('python runTrainer.py --agent=KerasDDPGAgent --env=MJCFHopperv0Env --train-for=10000000 --test-for=0 --gui --save-file=checkpoints/KerasDDPG-MJCFHopperv0-$(date +%y%m%d%H%M%S).h5')

