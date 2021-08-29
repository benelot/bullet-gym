# [DEPRECATED]: For the stable release use https://github.com/benelot/pybullet-gym.

### bullet-gym - The pybullet environment for use with the OpenAI Gym Reinforcement Learning Research Platform

OpenAI gym is currently one of the most widely used toolkit for developing and comparing reinforcement learning algorithms. Unfortunately, for several challenging continuous control environments it requires the user to install MuJoCo, a commercial physics engine which requires a license to run for longer than 30 days. Such a commercial barrier hinders open research, especially in the perspective that other appropriate physics engines exist. This repository provides alternative implementations of the original MuJoCo environments which can be used for free. The environments have been reimplemented using [BulletPhysics'](https://github.com/bulletphysics/bullet3) python wrapper pybullet, such that they seamlessly integrate into the OpenAI gym framework. In order to show the usability of the new environments, several RL agents from the [Keras-RL](https://github.com/matthiasplappert/keras-rl/wiki/Agent-Overview) are configured to be trained out of the box. To further simplify the training of agents, a Trainer class was implemented which helps to capture commandline arguments in a unified fashion. The trainer provides a set of standard arguments, but additional arguments can be defined by the agent and the environment to enable the researcher to provide special parameters to either one.

Thanks to matpalm for a [beautiful example](https://github.com/benelot/cartpoleplusplus) on how to use pybullet with OpenAI Gym.

## Repository Contents

The following environments are available to be run with the agents:
- CartPole Environment [[CartPolev0Env](https://github.com/benelot/bullet-gym/blob/master/envs/CartPolev0Env.py)]
- CartPole Environment with Detached Pole (2D control problem) [[Detached2DCartPolev0Env](https://github.com/benelot/bullet-gym/blob/master/envs/Detached2DCartPolev0Env.py)]
- Motion Environment with different morphologies [[Motionv0Env](https://github.com/benelot/bullet-gym/blob/master/envs/Motionv0Env.py)]
- Inverted Double Pendulum [[InvertedDoublePendulum-v1](https://gym.openai.com/envs/InvertedDoublePendulum-v1)]
- Reacher [[Reacher-v1](https://gym.openai.com/envs/Reacher-v1)]
- Half Cheetah [[HalfCheetah-v1](https://gym.openai.com/envs/HalfCheetah-v1)]
- Swimmer [[Swimmer-v1](https://gym.openai.com/envs/Swimmer-v1)]
- Hopper [[Hopper-v1](https://gym.openai.com/envs/Hopper-v1)]
- Walker 2D [[Walker-2d-v1](https://gym.openai.com/envs/Walker2d-v1)]
- Ant [[Ant-v1](https://gym.openai.com/envs/Ant-v1)]
- Humanoid [[Humanoid-v1](https://gym.openai.com/envs/Humanoid-v1)]
- Humanoid Standup [[HumanoidStandup-v1](https://gym.openai.com/envs/HumanoidStandup-v1)]

The following agents are available to be run with the environments:
- Keras DQN Agent [[KerasDQNAgent](https://github.com/benelot/bullet-gym/blob/master/agents/KerasDQNAgent.py)]
- Keras DDQN Agent [[KerasDDQNAgent](https://github.com/benelot/bullet-gym/blob/master/agents/KerasDDQNAgent.py)]
- Keras DDPG Agent [[KerasDDPGAgent](https://github.com/benelot/bullet-gym/blob/master/agents/KerasDDPGAgent.py)]
- Keras NAF Agent [[KerasNAFAgent](https://github.com/benelot/bullet-gym/blob/master/agents/KerasNAFAgent.py)]
- Keras CEM Agent [[KerasCEMAgent](https://github.com/benelot/bullet-gym/blob/master/agents/KerasCEMAgent.py)]

## Dependencies

- BulletPhysics library with pybullet installed on the python path: [Pybullet quickstart guide [pdf]](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstartguide.pdf)
- python with numpy and h5py - pip install numpy h5py
- [Keras](https://keras.io/) - pip install keras
- [TensorFlow](https://www.tensorflow.org/) - installation according to page
- [Keras-RL](https://github.com/matthiasplappert/keras-rl) - installation according to page
- [OpenAI Gym](https://github.com/openai/gym) - installation according to page


<!-- 
## Quick-start

Some examples running out of the box can be found at the root of this repository. The show*.sh scripts run an agent-environment combination with a pretrained agent showing its performance on the environment. To train an agent from scratch, the train*.sh scripts can be run. They run the agent-environment combination in training-mode. Removing the --gui flag in the scripts turns off the GUI output and makes the training headless.

Currently available are the following scripts:

### Environments

#### CartPole Example
A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track. The system is controlled by applying a force of +action_force or -action_force to the cart. The pendulum starts upright and gets a small, initial hit, and the goal is to prevent it from falling over. 
- [showKerasDQNCartPoleExample.sh][trainKerasDQNCartPoleExample.sh]
- [showKerasDDQNCartPoleExample.sh][trainKerasDDQNCartPoleExample.sh]
- [showKerasDDPGCartPoleExample.sh][trainKerasDDPGCartPoleExample.sh]
- [showKerasNAFCartPoleExample.sh (checkpoint missing)][trainKerasNAFCartPoleExample.sh]
- [showKerasCEMCartPoleExample.sh (checkpoint missing)][trainKerasCEMCartPoleExample.sh]

#### Detached CartPole Example
A pole is standing unattached on top of a cart, which moves along a frictionless track. The system is controlled by applying a force of +action_force or -action_force to the cart. The pendulum starts upright and gets a small, initial hit, and the goal is to prevent it from falling over.
- [showKerasDQNDetached2DCartPoleExample.sh][trainKerasDQNDetached2DCartpoleExample.sh]
- [showKerasDDQNDetached2DCartPoleExample.sh][trainKerasDDQNDetached2DCartPoleExample.sh]
- [showKerasNAFDetached2DCartPoleExample.sh][trainKerasNAFDetached2DCartPoleExample.sh]
- [showKerasDDPGDetached2DCartPoleExample.sh (checkpoint missing)][trainKerasDDPGDetached2DCartPoleExample.sh]
- [showKerasCEMDetached2DCartPoleExample.sh (checkpoint missing)][trainKerasCEMDetached2DCartPoleExample.sh]

#### Motion Example
A robot morphology (currently a snake and a phantomx robot morphology) is set out to learn locomotion patterns to perform according to the reward function. The reward function can be something like moving along or reaching a certain speed in the direction of a certain axis. The morphologies can be controlled using position,velocity or torque control. The environment must be diversified into multiple environments, maybe one for each morphology.
- [showKerasDDPGMotionExample.sh (checkpoint missing)][trainKerasDDPGMotionExample.sh]
- [showKerasNAFMotionExample.sh (checkpoint missing)][trainKerasNAFMotionExample.sh]

 -->
