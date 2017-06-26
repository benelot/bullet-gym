#!/usr/bin/python
import os 
os.system('python runTrainer.py --agent=KerasDDPGAgent --env=Motionv0Env --train-for=10000000 --test-for=0 --random-initial-position --save-file=checkpoints/KerasDDPG-Motionv0-$(date +%y%m%d%H%M%S).h5')

