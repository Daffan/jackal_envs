import time

import gym
import jackal_envs
# from jackal_sim_wrapper import RandomStartGoalPosition, ReducedObservation, RewardShaping

# env = gym.make('jackal_navigation-v0', world_name = '75cm_split.world', gui = 'false', VLP16 = 'false', init_position = [-2, -2, 0], goal_position = [2, 2, 0], verbose = 0)
# env = ReducedObservation(gym.make('jackal_navigation-v0', world_name = '85cm_split.world', gui = 'true', VLP16 = 'false'))
# env = RandomStartGoalPosition(gym.make('jackal_navigation-v0', world_name = '85cm_split.world', gui = 'true', VLP16 = 'false'))
# env = RewardShaping(gym.make('jackal_navigation-v0', world_name = 'sequential_applr_testbed.world', gui = 'true', VLP16 = 'false', init_position = [-8, 0, 0], goal_position = [54, 0, 0]))
env = gym.make('jackal_navigation_parallel-v0', config_path = 'configs/dqn.json')

done = False
count = 0
while not done:
    count += 1
    obs, rew, done, _ = env.step(64)
    print('current step %d' %(count))
print('done!')

env.close()
