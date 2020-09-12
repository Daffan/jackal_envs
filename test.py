import time
import json

import gym
import jackal_envs
from jackal_envs.jackal_sim_wrapper import *

from tianshou.env import SubprocVectorEnv
from tianshou.policy import DQNPolicy
from tianshou.utils.net.common import Net
from tianshou.data import Collector, ReplayBuffer, PrioritizedReplayBuffer

# from jackal_sim_wrapper import RandomStartGoalPosition, ReducedObservation, RewardShaping

# env = gym.make('jackal_navigation-v0', world_name = '75cm_split.world', gui = 'false', VLP16 = 'false', init_position = [-2, -2, 0], goal_position = [2, 2, 0], verbose = 0)
# env = ReducedObservation(gym.make('jackal_navigation-v0', world_name = '85cm_split.world', gui = 'true', VLP16 = 'false'))
# env = RandomStartGoalPosition(gym.make('jackal_navigation-v0', world_name = '85cm_split.world', gui = 'true', VLP16 = 'false'))
# env = RewardShaping(gym.make('jackal_navigation-v0', world_name = 'sequential_applr_testbed.world', gui = 'true', VLP16 = 'false', init_position = [-8, 0, 0], goal_position = [54, 0, 0]))
# env = gym.make('jackal_navigation_parallel-v0', config_path = 'configs/dqn.json')
'''
env = SubprocVectorEnv([lambda: gym.make('jackal_navigation_parallel-v0', config_path='configs/dqn.json') \
                                for _ in range(1)])
'''
config_path='configs/dqn.json'
with open(config_path, 'rb') as f:
    config = json.load(f)

env_config = config['env_config']
wrapper_config = config['wrapper_config']
training_config = config['training_config']

env = wrapper_dict[wrapper_config['wrapper']](gym.make('jackal_navigation-v0', **env_config), wrapper_config['wrapper_args'])

env.reset()
done = False
count = 0
t1 = time.time()
ep_return = 0
while not done:
    count += 1
    obs_next, rew, done, info = env.step(64)
    X = info['X']
    ep_return += rew
    print('current step %d, rew %f, X %f' %(count, rew, X))
    print(info['params'])
print('done! ep_return: %f' %(ep_return))
t2 = time.time()
print(t2-t1)
env.close()
