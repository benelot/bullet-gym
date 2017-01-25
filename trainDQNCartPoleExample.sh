#!/bin/sh
python runTrainer.py --agent=KerasDQNAgent --env=CartPolev0Env --train-for=10000000 --test-for=0 --gui --save-file=DQNCP-$(date +%y%m%d%H%M%S).h5

