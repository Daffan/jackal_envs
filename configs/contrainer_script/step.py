import jackal_envs
import argparse
import json
import gym

parser = argparse.ArgumentParser(description = 'step the existing environment')
parser.add_argument('--config', dest = 'config_path', type = str, default = '../configs/ppo.json', help = 'path to the configuration file')
parser.add_argument('--action', dest = 'action', type = int, default = 0, help = 'action for the next step')
args = parser.parse_args()

with open(args.config_path, 'rb') as f:
    config = json.load(f)

env_config = config['env_config']
env_config['init_world'] = False
wrapper_config = config['wrapper_config']

wrapper_dict = jackal_envs.jackal_sim_wrapper.wrapper_dict
env = wrapper_dict[wrapper_config['wrapper']](gym.make('jackal_navigation-v0', **env_config), wrapper_config['wrapper_args'])
msg = env.env.gazebo_sim.get_model_state()
X = msg.pose.position.x
Y = msg.pose.position.y
obs, rew, done, info = env.step(args.action)
print('[Observation]')
print(obs)
print('[Reward]')
print(rew)
print('[Done]')
print(done)
print('[Params]')
print(info['params'])
print('[X]')
print(X)
print('[Y]')
print(Y)
print('[Over]')
