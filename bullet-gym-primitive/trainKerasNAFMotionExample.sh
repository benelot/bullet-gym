#!/bin/sh
python runTrainer.py --agent=KerasNAFAgent --env=Motionv0Env --train-for=10000000 --test-for=0 --random-initial-position --gui --save-file=checkpoints/KerasNAF-Motionv0-$(date +%y%m%d%H%M%S).h5

