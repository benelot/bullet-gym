#!/usr/bin/python
import os 
os.system('python runTrainer.py --agent=KerasCEMAgent --env=CartPolev0Env --train-for=0 --test-for=10000000 --delay=0.005 --gui --show-test --load-file=checkpoints/KerasCEM-CartPolev0-chkpt-1.h5')

