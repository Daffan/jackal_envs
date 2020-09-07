import jackal_envs
import json
import argparse
import gym

parser = argparse.ArgumentParser(description = 'step the existing environment')
parser.add_argument('--config', dest = 'config_path', type = str, default = '../configs/ppo.json', help = 'path to the configuration file')
args = parser.parse_args()

with open(args.config_path, 'rb') as f:
    config = json.load(f)

env_config = config['env_config']
env_config['init_world'] = True
wrapper_config = config['wrapper_config']

wrapper_dict = jackal_envs.jackal_sim_wrapper.wrapper_dict
env = wrapper_dict[wrapper_config['wrapper']](gym.make('jackal_navigation-v0', **env_config), wrapper_config['wrapper_args'])

state_shape = env.observation_space.shape or env.observation_space.n
action_shape = env.action_space.shape or env.action_space.n

print('[state_shape:%d]' %(state_shape))
print('[action_shape:%d]' %(action_shape))
