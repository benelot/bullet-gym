#!/bin/sh
python runTrainer.py --agent=KerasDDPGAgent --env=CartPolev0Env --train-for=0 --test-for=10000000 --delay=0.005 --gui --show-test --load-file=KerasDDPG-CartPolev0-chkpt-1.h5

