from gym.envs.registration import register

register(
    id='gym_unscrewing_env-v0',
    entry_point='rl_gym.envs:UnscrewingEnv',
    timestep_limit=1000
)
