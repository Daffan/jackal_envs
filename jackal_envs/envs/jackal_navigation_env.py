import gym
import rospy
import rospkg
import roslaunch
import time
import numpy as np
import os
import subprocess

from gym import utils, spaces
from std_srvs.srv import Empty
import actionlib
from gym.utils import seeding

from .gazebo_simulation import GazeboSimulation
from .navigation_stack import  NavigationStack

class GazeboJackalNavigationEnv(gym.Env):

    def __init__(self, world_name = 'jackal_race.world', VLP16 = 'true', gui = 'false',
                init_position = [-8, 0, 0], goal_position = [46, 0, 0], max_step = 50, time_step = 1,
                max_vel_x_delta = 0.2, init_max_vel_x = 0.5):
        gym.Env.__init__(self)

        self.world_name = world_name
        self.VLP16 = True if VLP16=='true' else False
        self.gui = True if gui=='true' else False
        self.max_step = max_step
        self.time_step = time_step
        self.max_vel_x_delta = max_vel_x_delta
        self.init_max_vel_x = init_max_vel_x

        # Launch gazebo and navigation demo
        # Should have the system enviroment source to jackal_helper
        rospack = rospkg.RosPack()
        BASE_PATH = rospack.get_path('jackal_helper')
        self.gazebo_process = subprocess.Popen(['roslaunch', \
                                                os.path.join(BASE_PATH, 'launch', 'jackal_world_navigation.launch'),
                                                'world_name:=' + world_name,
                                                'gui:=' + gui,
                                                'VLP16:=' + VLP16
                                                ])

        time.sleep(10)
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('gym', anonymous=True)

        self.gazebo_sim = GazeboSimulation(init_position = init_position)
        self.navi_stack = NavigationStack(goal_position = goal_position)

        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)
        if VLP16 == 'true':
            self.observation_space = spaces.Box(low=np.array([0.2]*(2095)), # a hard coding here
                                                high=np.array([30]*(2095)),
                                                dtype=np.float)
        elif VLP16 == 'false':
            self.observation_space = spaces.Box(low=np.array([0.2]*(720)), # a hard coding here
                                                high=np.array([30]*(720)),
                                                dtype=np.float)

        self._seed()

        self.navi_stack.set_global_goal()
        self.reset()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _observation_builder(self, laser_scan, local_goal):
        '''
        Observation is the laser scan plus local goal. Episode ends when the
        between gobal goal and robot positon is less than 0.4m. Reward is set
        to -1 for each step
        '''
        scan_ranges = np.array(laser_scan.ranges)[1:-2]
        scan_ranges[scan_ranges == np.inf] = 30
        local_goal_position = np.array([local_goal.position.x, local_goal.position.y])
        max_vel = rospy.get_param('/move_base/TrajectoryPlannerROS/max_vel_x')
        state = np.concatenate([scan_ranges, local_goal_position])
        state = np.append(state, max_vel)

        distance = np.sqrt(np.sum((local_goal_position)**2))
        if distance < 0.4 or self.step_count >= self.max_step:
            done = True
        else:
            done = False

        return state, -1, done, {}

    def step(self, action):
        self.step_count += 1
        params = rospy.get_param('/move_base/TrajectoryPlannerROS/max_vel_x')
        if action == 0:
            params = params
        elif action == 1:
            # params = min(2, params + self.max_vel_x_delta)
            # self.navi_stack.set_max_vel_x(params)
            pass
        elif action == 2:
            # params = max(0.1, params - self.max_vel_x_delta)
            # self.navi_stack.set_max_vel_x(params)
            pass
        else:
            raise Exception('Action does not exist')

        # Unpause the world
        self.gazebo_sim.unpause()

        # Sleep for 5s (a hyperparameter that can be tuned)
        rospy.sleep(self.time_step)

        # Collect the laser scan data
        laser_scan = self.gazebo_sim.get_laser_scan()
        local_goal = self.navi_stack.get_local_goal()

        # Pause the simulation world
        self.gazebo_sim.pause()

        return self._observation_builder(laser_scan, local_goal)

    def reset(self):

        self.step_count = 0
        # reset robot in odom frame clear_costmap
        self.navi_stack.reset_robot_in_odom()
        # Resets the state of the environment and returns an initial observation.
        self.gazebo_sim.reset()
        # reset max_vel_x value
        self.navi_stack.set_max_vel_x(self.init_max_vel_x)

        # Unpause simulation to make observation
        self.gazebo_sim.unpause()

        #read laser data
        self.navi_stack.clear_costmap()
        rospy.sleep(0.1)
        self.navi_stack.clear_costmap()

        laser_scan = self.gazebo_sim.get_laser_scan()
        local_goal = self.navi_stack.get_local_goal()
        self.navi_stack.set_global_goal()

        self.gazebo_sim.pause()

        state, _, _, _ = self._observation_builder(laser_scan, local_goal)

        return state

    def close(self):
        os.system("killall -9 rosmaster")
        os.system("killall -9 gzclient")
        os.system("killall -9 gzserver")
        os.system("killall -9 roscore")
        # Kill gzclient, gzserver and roscore
        tmp = os.popen("ps -Af").read()
        gzclient_count = tmp.count('gzclient')
        gzserver_count = tmp.count('gzserver')
        roscore_count = tmp.count('roscore')
        rosmaster_count = tmp.count('rosmaster')

        if gzclient_count > 0:
            os.system("killall -9 gzclient")
        if gzserver_count > 0:
            os.system("killall -9 gzserver")
        if rosmaster_count > 0:
            os.system("killall -9 rosmaster")
        if roscore_count > 0:
            os.system("killall -9 roscore")

        if (gzclient_count or gzserver_count or roscore_count or rosmaster_count >0):
            os.wait()

if __name__ == '__main__':
    env = GazeboJackalNavigationEnv()
    env.reset()
    print(env.step(0))
    env.unpause()
    time.sleep(30)
    env.close()
