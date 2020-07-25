from jackal_envs.envs import GazeboJackalNavigationEnv
import time


env = GazeboJackalNavigationEnv(gui = True)

done = False
count = 0
while not done:
    count += 1
    obs, rew, done, _ = env.step(0)
    print('current step %d' %(count))
    if count > 40:
        break

print(env.gazebo_sim.get_model_state())

env.close()

import gym
import jackal_envs
from jackal_sim_wrapper import RandomStartGoalPosition, ReducedObservation
from helper import visual_laserscan, obs_reduction

env = gym.make('jackal_navigation-v0', world_name = 'sequential_applr_testbed.world', gui = 'true', VLP16 = 'false')
env = ReducedObservation(gym.make('jackal_navigation-v0', world_name = '85cm_split.world', gui = 'true', VLP16 = 'false'))
env = RandomStartGoalPosition(gym.make('jackal_navigation-v0', world_name = '85cm_split.world', gui = 'true', VLP16 = 'false'))
