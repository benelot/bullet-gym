# bullet-gym - The pybullet environment for use with the OpenAI Gym Reinforcement Learning Research Platform
bullet-gym is a prototyping repository for OpenAI Gym environments implemented with [BulletPhysics](https://github.com/bulletphysics/bullet3) pybullet. Several Reinforcement Learning agents are implemented within the Agent interface defined by the [Agents of Keras-rl](https://github.com/matthiasplappert/keras-rl/wiki/Agent-Overview). They can be easily set up to be trained and their progress be shown using a Trainer class accepting commandline arguments. Some standard arguments are defined by the Trainer, but additional arguments can be defined by the agent and the environment to enable the researcher to provide special parameters to either one.

Thanks to matpalm for a [beautiful example](https://github.com/benelot/cartpoleplusplus) on how to use pybullet with OpenAI Gym.

## Repository Contents

The following agents are available to be run with the environments:
- Keras DQN Agent [KerasDQNAgent]
- Keras DDQN Agent [KerasDDQNAgent]
- Keras DDPG Agent[KerasDDPGAgent]
- Keras NAF Agent [KerasNAFAgent]
- Keras CEM Agent [KerasCEMAgent]

The following environments are available to be run with the agents:
- CartPole Environment [CartPolev0Env]
- CartPole Environment with Detached Pole (2D control problem) [Detached2DCartPolev0Env]
- Motion Environment with different morphologies [Motionv0Env]

## Dependencies

- BulletPhysics library with pybullet installed on the python path: [Pybullet quickstart guide [pdf]](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstartguide.pdf)
- keras
- tensorflow
- python with numpy and h5py
- Keras-rl [Keras-rl repository](https://github.com/matthiasplappert/keras-rl)
- OpenAI Gym

## Quick-start

Some examples running out of the box can be found at the root of this repository. The show*.sh scripts run an agent-environment combination with a pretrained agent showing its performance on the environment.

Currently available are the following scripts:

- showDQNDetached2DCartPoleExample.sh
- showDQNCartPoleExample.sh [Checkpoint missing]

To train an agent from scratch, the train*.sh scripts can be run. They run the agent-environment combination in training-mode. Removing the --gui flag in the scripts turns off the GUI output and makes the training headless.

Currently available are the following scripts:

- trainDQNCartPoleExample.sh
- trainDQNDetached2DCartpoleExample.sh

## Environments


### CartPole-v0 environment

TODO

### Detached 2D CartPole-v0 environment

## Agents


### Keras DQN Agent

TODO

[DQN Agent on Keras-rl](https://github.com/matthiasplappert/keras-rl/wiki/DQNAgent)
