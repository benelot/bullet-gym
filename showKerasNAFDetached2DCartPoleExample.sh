#!/bin/sh
python runTrainer.py --agent=KerasNAFAgent --env=Detached2DCartPolev0Env --train-for=0 --test-for=10000000 --gui --show-test --load-file=KerasNAF-D2DCartPolev0-chkpt-1.h5

