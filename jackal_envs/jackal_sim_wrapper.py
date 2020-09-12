import gym
import numpy as np
import matplotlib.pyplot as plt
gym.logger.set_level(40)

args = {
    "start_range": [[-1.5, -0.5], [-1.5, 1.5]],
    "goal_range": [[0.5, 1.5], [-1.5, 1.5]],
    "seed": 43,

    "reduction": 10,
    "polar_goal": 'true',
    "centered_bin": "",
    "reward_shaping": 'false',

    "goal_distance_reward": 'true',
    "stuck_punishment": 0.1
}

class RandomStartGoalPosition(gym.Wrapper):

    def __init__(self, env, args = args):
        '''A wrapper that will randomly sample the start and goal position in a
        specific range
        args:
            env -- GazeboJackalNavigationEnv
            start_range -- [[x_min, x_max], [y_min, y_max]]
            goal_range -- [[x_min, x_max], [y_min, y_max]]
        '''
        super(RandomStartGoalPosition, self).__init__(env)
        np.random.seed(args['seed'])
        self.start_range = args['start_range']
        self.goal_range = args['goal_range']

    def reset(self):

        start_x = np.random.uniform(self.start_range[0][0], self.start_range[0][1])
        start_y = np.random.uniform(self.start_range[1][0], self.start_range[1][1])
        start_rotation = np.random.uniform(0, 2*np.pi)

        goal_x = np.random.uniform(self.goal_range[0][0], self.goal_range[0][1]) - start_x
        goal_y = np.random.uniform(self.goal_range[1][0], self.goal_range[1][1]) - start_y
        goal_rotation = np.random.uniform(0, 2*np.pi)


        self.env.gazebo_sim.reset_init_model_state(init_position = [start_x, start_y, start_rotation])
        self.env.navi_stack.reset_global_goal(goal_position = [goal_x, goal_y, goal_rotation])
        obs = self.env.reset()

        return obs

class ReducedObservation(gym.Wrapper):

    def __init__(self, env, args = args):
        '''A wrapper that will reduce the dimension of the observation
        args:
            env -- GazeboJackalNavigationEnv
            arg -- arguments of the wrapper
        '''
        super(ReducedObservation, self).__init__(env)
        self.reduction = args['reduction']
        self.polar_goal = True if args['polar_goal'] == 'true' else False
        self.centered_bin = args['centered_bin']
        self.reward_shaping = True if args['reward_shaping'] == 'true' else False
        self.observation_space = gym.spaces.Box(low=np.array([0.2]*(74)), # a hard coding here
                                            high=np.array([100]*(74)),
                                            dtype=np.float32)

    def obs_reduction(self, obs):
        a = obs[:-3][::self.reduction]
        l = len(obs)
        if not self.polar_goal:
            a = np.concatenate([a, obs[-3:]])
        else:
            theta = (np.arctan(obs[-2]/obs[-3])/(2*2.3561899662)*l+l/2)//self.reduction
            if self.centered_bin:
                a = a[theta-self.centered_bin:theta+self.centered_bin]
                a = np.concatenate([a, np.array([obs[-1]])])
            else:
                a = np.concatenate([a, np.array([theta, obs[-1]])])
        return a

    def reset(self):
        obs = self.env.reset()
        current_position = np.array([self.env.navi_stack.robot_config.X, \
                                    self.env.navi_stack.robot_config.Y])
        self.local_goal = obs[-3:-1] + current_position
        return self.obs_reduction(obs)

    def step(self, action):
        obs, rew, done, info = self.env.step(action)
        if self.reward_shaping:
            current_position = np.array([self.env.navi_stack.robot_config.X, \
                                        self.env.navi_stack.robot_config.Y])
            rew = -np.sum((self.local_goal-current_position)**2)**0.5
            self.local_goal = obs[-3:-1] + current_position
        obs = self.obs_reduction(obs)
        return obs, rew, done, info

class RewardShaping(gym.Wrapper):

    def __init__(self, env, args = args):
        '''A wrapper that will shape the reward by the length of the globle path
        args:
            env -- GazeboJackalNavigationEnv
            start_range -- [[x_min, x_max], [y_min, y_max]]
            goal_range -- [[x_min, x_max], [y_min, y_max]]
        '''
        super(RewardShaping, self).__init__(env)
        self.goal_distance_reward = args['goal_distance_reward']
        self.stuck_punishment = args['stuck_punishment']
        self.punishment_reward = args['punishment_reward']
        self.global_path = self.env.navi_stack.robot_config.global_path
        self.gp_len = sum([self.distance(self.global_path[i+1], self.global_path[i]) for i in range(len(self.global_path)-1)])

    def distance(self, p1, p2):
        return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**0.5

    def reset(self):
        obs = self.env.reset()
        self.env._set_param('/X', self.env.gazebo_sim.get_model_state().x)
        self.rp = []
        return obs

    def visual_path(self):
        plt.scatter(self.global_path[:,0], self.global_path[:,1])
        plt.scatter(self.env.navi_stack.robot_config.X, self.env.navi_stack.robot_config.Y)
        plt.show()

    def step(self, action):
        # take one step
        obs, rew, done, info = self.env.step(action)
        # reward is the decrease of the distance
        position = self.env.gazebo_sim.get_model_state().pose.position
        rew += (position.x - self.env._get_param('/X')) * goal_distance_reward
        self.env._set_param('/X', position.x)
        rew += self.env.navi_stack.punish_rewrad()*self.stuck_punishment
        rp = np.array([position.x, position.y])
        self.rp.append(rp)

        if len(self.rp) > 100:
            if self.rp[-1] < self.rp[-100]:
                done = True
                rew = -self.punishment_reward
        if position.z > 0.1: # or
            done = True
            rew = -self.punishment_reward
        if position.x > 42: # or
            done = True



        return obs, rew, done, info


wrapper_dict = {
    'random_start_goal': RandomStartGoalPosition,
    'reduced_observation': ReducedObservation,
    'reward_shaping': RewardShaping,
    'default': lambda env: env
}
