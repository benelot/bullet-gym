#!/bin/sh
python runTrainer.py --agent=KerasDQNAgent --env=Detached2DCartPolev0Env --train-for=0 --test-for=10000000 --gui --show-test --load-file=DQND2DCP-chkpt-1.h5

