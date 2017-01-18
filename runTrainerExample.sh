#!/bin/sh
python runTrainer.py --agent=KerasDQNAgent --env=Detached2DCartPolev0Env --load-file=chkpt-2.h5 --train-for=100000 --test-for=10000000 --gui --show-test

