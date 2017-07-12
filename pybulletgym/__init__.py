from gym.envs.registration import register

register(
	id='PybulletInvertedPendulum-v0',
	entry_point='envs.gym_pendula:PybulletInvertedPendulum',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

register(
	id='PybulletInvertedDoublePendulum-v0',
	entry_point='envs.gym_pendula:PybulletInvertedDoublePendulum',
	max_episode_steps=1000,
	reward_threshold=9100.0,
	)

register(
	id='PybulletInvertedPendulumSwingup-v0',
	entry_point='envs.gym_pendula:PybulletInvertedPendulumSwingup',
	max_episode_steps=1000,
	reward_threshold=800.0,
	)

register(
	id='PybulletReacher-v0',
	entry_point='envs.gym_manipulators:PybulletReacher',
	max_episode_steps=150,
	reward_threshold=-3.75,
	)

register(
	id='PybulletPusher-v0',
	entry_point='envs.gym_manipulators:PybulletPusher',
	max_episode_steps=100,
	reward_threshold=0.0,
)

register(
	id='PybulletThrower-v0',
	entry_point='envs.gym_manipulators:PybulletThrower',
	max_episode_steps=100,
	reward_threshold=0.0,
)

register(
	id='PybulletStriker-v0',
	entry_point='envs.gym_manipulators:PybulletStriker',
	max_episode_steps=100,
	reward_threshold=0.0,
)

register(
	id='PybulletHopper-v0',
	entry_point='envs.gym_forward_walkers:PybulletHopper',
	max_episode_steps=1000,
	reward_threshold=3800.0	#~12 jumps
	)
register(
	id='PybulletWalker2d-v0',
	entry_point='envs.gym_forward_walkers:PybulletWalker2d',
	max_episode_steps=1000,
	)
register(
	id='PybulletHalfCheetah-v0',
	entry_point='envs.gym_forward_walkers:PybulletHalfCheetah',
	max_episode_steps=1000,
	reward_threshold=4800.0
	)

register(
	id='PybulletAnt-v0',
	entry_point='envs.gym_forward_walkers:PybulletAnt',
	max_episode_steps=1000,
	reward_threshold=6000.0
	)

register(
	id='PybulletHumanoid-v0',
	entry_point='envs.gym_forward_walkers:PybulletHumanoid',
	max_episode_steps=1000
	)
#register(
#	id='RoboschoolHumanoidHarder-v0',
#	entry_point='roboschool:RoboschoolHumanoidHarder',
#	max_episode_steps=1000
#	)
#
#register(
#	id='RoboschoolPong-v0',
#	entry_point='roboschool:RoboschoolPong',
#	max_episode_steps=1000
#	)

from pybulletgym.envs.gym_pendula import PybulletInvertedPendulum
from pybulletgym.envs.gym_pendula import PybulletInvertedDoublePendulum
from pybulletgym.envs.gym_pendula import PybulletInvertedPendulumSwingup
from pybulletgym.envs.gym_manipulators import PybulletReacher

from pybulletgym.agents import agent_register

agent_register.register(
	id='BaselinesDQNAgent-v0',
	entry_point='agents.BaselinesDQNAgent:BaseLinesDQNAgent'
)

agent_register.register(
	id='KerasCEMAgent-v0',
	entry_point='agents.KerasCEMAgent:KerasCEMAgent'
)

agent_register.register(
	id='KerasDDPGAgent-v0',
	entry_point='agents.KerasDDPGAgent:KerasDDPGAgent'
)

agent_register.register(
	id='KerasDDQNAgent-v0',
	entry_point='agents.KerasDDQNAgent:KerasDDQNAgent'
)

agent_register.register(
	id='KerasDQNAgent-v0',
	entry_point='agents.KerasDQNAgent:KerasDQNAgent'
)

agent_register.register(
	id='KerasNAFAgent-v0',
	entry_point='agents.KerasNAFAgent:KerasNAFAgent'
)

from pybulletgym.agents.BaselinesDQNAgent import BaselinesDQNAgent

from pybulletgym.agents.KerasCEMAgent import KerasCEMAgent
from pybulletgym.agents.KerasDDPGAgent import KerasDDPGAgent
from pybulletgym.agents.KerasDDQNAgent import KerasDDQNAgent
from pybulletgym.agents.KerasDQNAgent import KerasDQNAgent
from pybulletgym.agents.KerasNAFAgent import KerasNAFAgent
