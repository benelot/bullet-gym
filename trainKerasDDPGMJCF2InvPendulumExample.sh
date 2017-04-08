#!/bin/sh
python runTrainer.py --agent=KerasDDPGAgent --env=MJCF2InvPendulumv0Env --train-for=10000000 --test-for=0 --gui --save-file=checkpoints/KerasDDPG-MJCF2InvPendulumv0-$(date +%y%m%d%H%M%S).h5

