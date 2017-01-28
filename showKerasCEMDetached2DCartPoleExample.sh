#!/bin/sh
python runTrainer.py --agent=KerasCEMAgent --env=Detached2DCartPolev0Env --train-for=0 --test-for=10000000 --gui --show-test --load-file=KerasCEM-D2DCartPolev0-chkpt-1.h5

