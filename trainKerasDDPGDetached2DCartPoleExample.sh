#!/bin/sh
python runTrainer.py --agent=KerasDDPGAgent --env=Detached2DCartPolev0Env --train-for=10000000 --test-for=0 --gui --save-file=KerasDDPG-D2DCartPolev0-$(date +%y%m%d%H%M%S).h5

