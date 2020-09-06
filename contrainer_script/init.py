import jackal_envs

# Load config
config_path = '/home/mnt/config.json'
import json
with open(config_path, 'rb') as f:
    config = json.load(f)

env_config = config['env_config']
env_config['init_world'] = True
wrapper_config = config['wrapper_config']

wrapper_dict = jackal_envs.jackal_sim_wrapper.wrapper_dict
env = wrapper_dict[wrapper_config['wrapper']](gym.make('jackal_navigation-v0', **env_config), wrapper_config['wrapper_args'])
