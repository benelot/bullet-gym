#!/bin/sh
python runTrainer.py --agent=KerasDDPGAgent --env=Motionv0Env --train-for=10000000 --test-for=0 --random-initial-position --gui --save-file=checkpoints/KerasDDPG-Motionv0-$(date +%y%m%d%H%M%S).h5

