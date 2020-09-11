import gym
import docker
from os.path import dirname, basename, abspath
import numpy as np
import re
from gym import utils, spaces
gym.logger.set_level(40)

class ParallelGazeboJackalNavigationEnv(gym.Env):

    def __init__(self, config_path):
        config_path = abspath(config_path)
        self.dirname = dirname(config_path)
        self.basename = basename(config_path)
        self.client = docker.from_env()
        container = self.client.containers.run('zifanxu/ros-jackal', detach = True, tty=True,
                                                volumes={self.dirname:{'bind': '/home/mnt/', 'mode': 'rw'}})
        exit_code, out = container.exec_run("/bin/bash -c 'source /home/jackal_ws/devel/setup.bash; \
                                    python3 /home/jackal_envs/contrainer_script/init.py \
                                    --config /home/mnt/%s'" %(self.basename))
        self.container = container
        assert exit_code == 0, out.decode('utf-8')
        out = out.decode('utf-8')
        state_shape = re.search("\[state_shape:(.*)\]", out).group(1)
        state_shape = int(state_shape)

        action_shape = re.search("\[action_shape:(.*)\]", out).group(1)
        action_shape = int(action_shape)

        self.action_space = spaces.Discrete(action_shape)
        self.observation_space = spaces.Box(low=np.array([-1]*state_shape), # a hard coding here
                                            high=np.array([1]*state_shape),
                                            dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)

    def seed(self, seed):
        np.random.seed(seed)

    def reset(self):
        exit_code, out = self.container.exec_run("/bin/bash -c 'source /home/jackal_ws/devel/setup.bash; \
                                    python3 /home/mnt/contrainer_script/reset.py \
                                    --config /home/mnt/%s'" %(self.basename))
        assert exit_code == 0, out.decode('utf-8')
        out = out.decode('utf-8').replace('\n', '')
        obs = re.search("Observation:\[(.*)\]", out).group(1)
        obs = obs.split(' ')
        obs = [float(s) for s in obs if s]

        self.xl = [] # list of x coordinate
        return np.array(obs)

    def step(self, action):
        exit_code, out = self.container.exec_run("/bin/bash -c 'source /home/jackal_ws/devel/setup.bash; \
                                    python3 /home/mnt/contrainer_script/step.py \
                                    --config /home/mnt/%s \
                                    --action %d'" %(self.basename, action))
        assert exit_code == 0, out.decode('utf-8')
        out = out.decode('utf-8').replace('\n', '')
        # pharse observation
        obs = re.search("\[Observation\]\[(.*)\]\[Reward\]", out).group(1)
        obs = obs.split(' ')
        obs = [float(s) for s in obs if s]
        obs = np.array(obs)
        # pharse reward
        rew = re.search("\[Reward\](.*)\[Done\]", out).group(1)
        rew = float(rew)
        # pharse done
        done = re.search("\[Done\](.*)\[Params\]", out).group(1)
        done = True if done == 'True' else False
        # pharse info
        params = re.search("\[Params\]\[(.*)\]", out).group(1)
        params = params.split(', ')
        params = [float(s) for s in params if s]
        X = re.search("\[X\](.*)\[Y\]", out).group(1)
        X = float(X)
        Y = re.search("\[Y\](.*)\[Over\]", out).group(1)
        Y = float(X)

        info = {'params': params, 'X': X, 'Y':Y}

        # To decide stuck condition
        self.xl.append(X)
        if len(xl) > 100:
            if xl[-1] <= xl[-100]:
                rew = -300
                done = True

        return obs, rew, done, info

    def close(self):
        self.container.stop()
        self.container.remove()

