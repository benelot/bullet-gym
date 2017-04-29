#!/bin/sh
python runTrainer.py --agent=KerasCEMAgent --env=CartPolev0Env --train-for=10000000 --test-for=0 --save-file=checkpoints/KerasCEM-CartPolev0-$(date +%y%m%d%H%M%S).h5

