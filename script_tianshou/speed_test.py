import sys
import jackal_envs
import gym
from jackal_envs.jackal_sim_wrapper import *

import numpy
try:
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
except:
    pass
import torch
from torch.utils.tensorboard import SummaryWriter

from tianshou.env import DummyVectorEnv
from tianshou.policy import DQNPolicy
from tianshou.utils.net.common import Net
from tianshou.data import Collector, ReplayBuffer, PrioritizedReplayBuffer
from offpolicy import offpolicy_trainer

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

import pickle
import argparse
import json
from datetime import datetime
import os

parser = argparse.ArgumentParser(description = 'Jackal navigation simulation')
parser.add_argument('--config', dest = 'config_path', type = str, default = '../configs/dqn.json', help = 'path to the configuration file')
parser.add_argument('--save', dest = 'save_path', type = str, default = '../results/', help = 'path to the saving folder')

args = parser.parse_args()
config_path = args.config_path
save_path = args.save_path

# Load the config files
if not config_path.startswith('../'):
    config_path = '../configs/' + config_path

with open(config_path, 'rb') as f:
    config = json.load(f)

env_config = config['env_config']
wrapper_config = config['wrapper_config']
training_config = config['training_config']

# initialize the env --> num_env can only be one right now
if not config['use_container']:
    env = wrapper_dict[wrapper_config['wrapper']](gym.make('jackal_navigation-v0', **env_config), wrapper_config['wrapper_args'])
    train_envs = DummyVectorEnv([lambda: env for _ in range(1)])
    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n
else:
    env = gym.make('jackal_navigation_parallel-v0', config_path=config_path)
    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n
    env.close()
    train_envs = DummyVectorEnv([lambda: gym.make('jackal_navigation_parallel-v0', config_path=config_path) \
                                for _ in range(training_config['num_envs'])])

# config random seed
np.random.seed(config['seed'])
torch.manual_seed(config['seed'])
train_envs.seed(config['seed'])

net = Net(training_config['layer_num'], state_shape, action_shape, config['device']).to(config['device'])
optim = torch.optim.Adam(net.parameters(), lr=training_config['learning_rate'])
policy = DQNPolicy(
        net, optim, training_config['gamma'], training_config['n_step'],
        target_update_freq=training_config['target_update_freq'])

if training_config['prioritized_replay']:
    buf = PrioritizedReplayBuffer(
            training_config['buffer_size'],
            alpha=training_config['alpha'], beta=training_config['beta'])
else:
    buf = ReplayBuffer(training_config['buffer_size'])
policy.set_eps(1)
train_collector = Collector(policy, train_envs, buf)

t1 = time.time()
train_collector.collect(n_step=1000)
t2 = time.time()

print('run 1000 step with %d envs using time %f' %(training_config['num_envs'], t2-t1))

