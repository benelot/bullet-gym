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

### Environments

#### CartPole Example
A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track. The system is controlled by applying a force of +action_force or -action_force to the cart. The pendulum starts upright and gets a small, initial hit, and the goal is to prevent it from falling over. 
- showKerasDDQNCartPoleExample.sh 
- showKerasDDPGCartPoleExample.sh
- showKerasNAFCartPoleExample.sh [checkpoint missing]
- showKerasDQNCartPoleExample.sh [checkpoint missing]
- showKerasCEMCartPoleExample.sh [checkpoint missing]

#### Detached CartPole Example
A pole is standing unattached on top of a cart, which moves along a frictionless track. The system is controlled by applying a force of +action_force or -action_force to the cart. The pendulum starts upright and gets a small, initial hit, and the goal is to prevent it from falling over.
- showKerasDDQNDetached2DCartPoleExample.sh
- showKerasNAFDetached2DCartPoleExample.sh
- showKerasDDPGDetached2DCartPoleExample.sh [checkpoint missing]
- showKerasCEMDetached2DCartPoleExample.sh [checkpoint missing]
- showKerasDQNDetached2DCartPoleExample.sh [checkpoint missing]

#### Motion Example
A robot morphology (currently a snake and a phantomx robot morphology) is set out to learn locomotion patterns to perform according to the reward function. The reward function can be something like moving along or reaching a certain speed in the direction of a certain axis. The morphologies can be controlled using position,velocity or torque control. The environment must be diversified into multiple environments, maybe one for each morphology.
- showKerasDDPGMotionExample.sh [checkpoint missing]
- showKerasNAFMotionExample.sh [checkpoint missing]

To train an agent from scratch, the train*.sh scripts can be run. They run the agent-environment combination in training-mode. Removing the --gui flag in the scripts turns off the GUI output and makes the training headless.

Currently available are the following scripts:

### Environments

#### CartPole Example
A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track. The system is controlled by applying a force of +action_force or -action_force to the cart. The pendulum starts upright and gets a small, initial hit, and the goal is to prevent it from falling over. 
- trainKerasDQNCartPoleExample.sh
- trainKerasCEMCartPoleExample.sh
- trainKerasDDPGCartPoleExample.sh
- trainKerasDDQNCartPoleExample.sh
- trainKerasDQNCartPoleExample.sh
- trainKerasNAFCartPoleExample.sh

#### Detached CartPole Example
A pole is standing unattached on top of a cart, which moves along a frictionless track. The system is controlled by applying a force of +action_force or -action_force to the cart. The pendulum starts upright and gets a small, initial hit, and the goal is to prevent it from falling over.
- trainKerasDQNDetached2DCartpoleExample.sh
- trainKerasCEMDetached2DCartPoleExample.sh
- trainKerasDDPGDetached2DCartPoleExample.sh
- trainKerasNAFDetached2DCartPoleExample.sh
- trainKerasDDQNDetached2DCartPoleExample.sh

#### Motion Example
- trainKerasDDPGMotionExample.sh
- trainKerasNAFMotionExample.sh
