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

from gazebo_simulation import GazeboSimulation
from navigation_stack import  NavigationStack

class GazeboJackalNavigationEnv(gym.Env):

    def __init__(self, config = 'demo_jackal_world_navigation'):
        gym.Env.__init__(self)

        # Launch gazebo and navigation demo
        # Should have the system enviroment source to test_pkg
        rospack = rospkg.RosPack()
        BASE_PATH = rospack.get_path('jackal_helper')
        self.gazebo_process = subprocess.Popen(['roslaunch', os.path.join(BASE_PATH, 'launch', 'jackal_world_navigation.launch')])

        time.sleep(5)
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('gym', anonymous=True)

        self.gazebo_sim = GazeboSimulation()
        self.navi_stack = NavigationStack()

        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()

        self.navi_stack.set_global_goal()
        self.gazebo_sim.pause()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _observation_builder(self, laser_scan, local_goal):
        '''
        Observation is the laser scan plus local goal. Episode ends when the
        between gobal goal and robot positon is less than 0.4m. Reward is set
        to -1 for each step
        '''
        scan_ranges = np.array(laser_scan.ranges)[1:-1]
        scan_ranges[scan_ranges == np.inf] = 30
        local_goal_position = np.array([local_goal.position.x, local_goal.position.y])
        state = np.concatenate([scan_ranges, local_goal_position])

        distance = np.sqrt(np.sum((local_goal_position)**2))
        if distance < 0.4:
            done = True
        else:
            done = False

        return state, -1, done, {}

    def step(self, action):
        params = rospy.get_param('/move_base/TrajectoryPlannerROS/max_vel_x')
        if action == 0:
            params = params
        elif action == 1:
            params = params + 0.1
            self.navi_stack.set_max_vel_x(params)
        elif action == 2:
            params = params - 0.1
            self.navi_stack.set_max_vel_x(params)
        else:
            raise Exception('Action does not exist')

        # Unpause the world
        self.gazebo_sim.unpause()

        # Sleep for 5s (a hyperparameter that can be tuned)
        rospy.sleep(1)

        # Collect the laser scan data
        laser_scan = self.gazebo_sim.get_laser_scan()
        local_goal = self.navi_stack.get_local_goal()

        # Pause the simulation world
        self.gazebo_sim.pause()

        return self._observation_builder(laser_scan, local_goal)

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        self.gazebo_sim.reset()

        # Unpause simulation to make observation
        self.gazebo_sim.unpause()

        #read laser data
        laser_scan = self.gazebo_sim.get_laser_scan()
        local_goal = self.navi_stack.get_local_goal()

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
