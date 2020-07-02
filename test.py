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
