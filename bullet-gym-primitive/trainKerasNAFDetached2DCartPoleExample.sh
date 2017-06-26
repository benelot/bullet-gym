#!/bin/sh
python runTrainer.py --agent=KerasNAFAgent --env=Detached2DCartPolev0Env --train-for=10000000 --test-for=0 --gui --save-file=checkpoints/KerasNAF-D2DCartPolev0-$(date +%y%m%d%H%M%S).h5

