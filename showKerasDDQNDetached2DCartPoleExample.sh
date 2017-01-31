#!/bin/sh
python runTrainer.py --agent=KerasDDQNAgent --env=Detached2DCartPolev0Env --train-for=0 --test-for=10000000 --delay=0.005 --gui --show-test --load-file=KerasDDQN-D2DCartPolev0-chkpt-1.h5

