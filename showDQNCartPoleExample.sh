#!/bin/sh
python runTrainer.py --agent=KerasDQNAgent --env=CartPolev0Env --train-for=0 --test-for=10000000 --gui --show-test --load-file=DQNCP-chkpt-1.h5

