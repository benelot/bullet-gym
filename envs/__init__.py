from gym.envs.registration import registry, register, make, spec

# ------------bullet-------------

register(
    id='CartPoleEnv-v0',
    entry_point='envs:CartPolev0Env',
    timestep_limit=1000,
    reward_threshold=1000.0,
)

register(
    id='Detached2DCartPoleEnv-v0',
    entry_point='envs:Detached2DCartPolev0Env',
    timestep_limit=1000,
    reward_threshold=1000.0,
)

#register(
#    id='MotionEnv-v0',
#    entry_point='envs:Motionv0Env',
#    timestep_limit=1000,
#    reward_threshold=1000.0,
#)

#register(
#    id='MJCFAntEnv-v0',
#    entry_point='envs:MJCFAntv0Env',
#    timestep_limit=1000,
#    reward_threshold=1000.0,
#)

#register(
#    id='MJCFHopperEnv-v0',
#    entry_point='envs:MJCFHopperv0Env',
#    timestep_limit=1000,
#    reward_threshold=1000.0,
#)

#register(
#    id='MJCFHumanoidEnv-v0',
#    entry_point='envs:MJCFHumanoidv0Env',
#    timestep_limit=1000,
#    reward_threshold=1000.0,
#)

#register(
#    id='MJCFInvPendulumEnv-v0',
#    entry_point='envs:MJCFInvPendulumv0Env',
#    timestep_limit=1000,
#    reward_threshold=1000.0,
#)

#register(
#    id='MJCF2InvPendulumEnv-v0',
#    entry_point='envs:MJCF2InvPendulumv0Env',
#    timestep_limit=1000,
#    reward_threshold=1000.0,
#)

#register(
#    id='MJCFReacherEnv-v0',
#    entry_point='envs:MJCFReacherv0Env',
#    timestep_limit=1000,
#    reward_threshold=1000.0,
#)

#register(
#    id='MJCFSwimmerEnv-v0',
#    entry_point='envs:MJCFSwimmerv0Env',
#    timestep_limit=1000,
#    reward_threshold=1000.0,
#)
