#!/bin/sh
python runTrainer.py --agent=KerasDDPGAgent --env=Motionv0Env --train-for=0 --test-for=10000000 --random-initial-position --delay=0.005 --gui --show-test --load-file=checkpoints/KerasDDPG-Motionv0-chkpt-1.h5
