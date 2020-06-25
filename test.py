from jackal_envs.envs import GazeboJackalNavigationEnv
env = GazeboJackalNavigationEnv()
import time


env = GazeboJackalNavigationEnv()

for _ in range(100):
    env.reset()
    time.sleep(1)
# done = False
# while not done:
#     obs, rew, done, _ = env.step(0)


env.close()
