#!/usr/bin/python
import os 
os.system('python runTrainer.py --agent=KerasNAFAgent --env=Motionv0Env --train-for=0 --test-for=10000000 --random-initial-position --delay=0.005 --gui --show-test --load-file=checkpoints/KerasNAF-Motionv0-chkpt-1.h5')
