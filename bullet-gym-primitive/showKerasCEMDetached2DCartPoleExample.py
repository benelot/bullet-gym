#!/usr/bin/python
import os 
os.system('python runTrainer.py --agent=KerasCEMAgent --env=Detached2DCartPolev0Env --train-for=0 --test-for=10000000 --delay=0.005 --gui --show-test --load-file=checkpoints/KerasCEM-D2DCartPolev0-chkpt-1.h5')

