#!/usr/bin/python
import os 
os.system('python runTrainer.py --agent=KerasDDPGAgent --env=CartPolev0Env --train-for=10000000 --test-for=0 --save-file=checkpoints/KerasDDPG-CartPolev0-$(date +%y%m%d%H%M%S).h5')

