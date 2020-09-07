from gym.envs.registration import register
from jackal_envs import jackal_sim_wrapper
from jackal_envs.parallel_jackal_navigation_env import ParallelGazeboJackalNavigationEnv

register(
    id='jackal_navigation-v0',
    entry_point='jackal_envs.envs:GazeboJackalNavigationEnv',
)

register(
    id='jackal_navigation_parallel-v0',
    entry_point='jackal_envs:ParallelGazeboJackalNavigationEnv',
)
