import jackal_envs
import json
import argparse

parser = argparse.ArgumentParser(description = 'step the existing environment')
parser.add_argument('--config', dest = 'config_path', type = str, default = '../configs/ppo.json', help = 'path to the configuration file')
with open(parser.config, 'rb') as f:
    config = json.load(f)

env_config = config['env_config']
env_config['init_world'] = False
wrapper_config = config['wrapper_config']

wrapper_dict = jackal_envs.jackal_sim_wrapper.wrapper_dict
env = wrapper_dict[wrapper_config['wrapper']](gym.make('jackal_navigation-v0', **env_config), wrapper_config['wrapper_args'], init_world = False)

obs = env.close()
