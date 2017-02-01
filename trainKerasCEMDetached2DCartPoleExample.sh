#!/bin/sh
python runTrainer.py --agent=KerasCEMAgent --env=Detached2DCartPolev0Env --train-for=10000000 --test-for=0 --gui --save-file=checkpoints/KerasCEM-D2DCartPolev0-$(date +%y%m%d%H%M%S).h5

