import gym
import docker
from os.path import dirname, basename
import numpy
import re
from gym import utils, spaces

class ParallelGazeboJackalNavigationEnv(gym.Env):

    def __init__(self, config_path):
        self.dirname = dirname(config_path)
        self.basename = basename(config_path)
        client = docker.from_env()
        container = self.client.containers.run('zifanxu/ros-jackal', detach = True, tty=True,
                                                volumes={dirname:{'bind': '/home/configs/', 'mode': 'rw'}})
        out = container.exec_run("/bin/bash -c 'source /home/jackal_ws/devel/setup.bash; \
                                python3 /home/jackal_envs/contrainer_script/init.py \
                                --config /home/configs/%s'" %(self.basename))
        self.container = container

        state_shape = re.search("\[state_shape:(.*)\]", out).group(1)
        state_shape = int(state_shape)

        action_shape = re.search("\[action_shape:(.*)\]", out).group(1)
        action_shape = int(action_shape)

        self.action_space = spaces.Discrete(action_shape)
        self.observation_space = spaces.Box(low=np.array([-1]*state_shape), # a hard coding here
                                            high=np.array([1]*state_shape),
                                            dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)

    def reset(self):
        out = self.container.exec_run("/bin/bash -c 'source /home/jackal_ws/devel/setup.bash; \
                                    python3 /home/jackal_envs/contrainer_script/reset.py \
                                    --config /home/configs/%s'" %(self.basename))
        out = out.replace('\n', '')
        obs = re.search("Observation:\[(.*)\]", out).group(1)
        obs = obs.split(' ')
        obs = [float(s) for s in obs if s]
        return np.array(obs)

    def step(self, action):
        out = self.container.exec_run("/bin/bash -c 'source /home/jackal_ws/devel/setup.bash; \
                                    python3 /home/jackal_envs/contrainer_script/step.py \
                                    --config /home/configs/%s \
                                    --action %d'" %(self.basename, action))
        out = out.replace('\n', '')
        # pharse observation
        obs = re.search("\[Observation\]\[(.*)\]\[Reward\]", out).group(1)
        obs = obs.split(' ')
        obs = [float(s) for s in obs if s]
        # pharse reward
        rew = re.search("\[Reward\](.*)\[Done\]", out).group(1)
        rew = float(rew)
        # pharse done
        done = re.search("\[Done\](.*)\[Information\]", out).group(1)
        done = True if done == 'True' else False
        # pharse info
        params = re.search("\[Information\]\[(.*)\]", out).group(1)
        params = params.split(', ')
        params = [float(s) for s in params if s]
        info = {'params': parmas}

        return obs, rew, done, info

    def close(self):
        self.container.stop()
        self.container.remove()

