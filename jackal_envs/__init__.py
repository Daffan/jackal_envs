from gym.envs.registration import register
from jackal_envs import jackal_sim_wrapper

register(
    id='jackal_navigation-v0',
    entry_point='jackal_envs.envs:GazeboJackalNavigationEnv',
)
