import rospy
import actionlib
from math import radians

import dynamic_reconfigure.client
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, Pose
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from tf.transformations import quaternion_from_euler

def _create_global_goal(x, y, yaw):
    """
    Create a MoveBaseGoal with x, y position and yaw rotation (in degrees).
    Returns a MoveBaseGoal
    """
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'odom' # Note: the frame_id must be map
    mb_goal.target_pose.pose.position.x = x
    mb_goal.target_pose.pose.position.y = y
    mb_goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)

    # Orientation of the robot is expressed in the yaw value of euler angles
    angle = radians(yaw) # angles are expressed in radians
    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

    return mb_goal

class NavigationStack():

    def __init__(self):
        self.client = dynamic_reconfigure.client.Client('move_base/TrajectoryPlannerROS')
        self.nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.global_goal = _create_global_goal(6, 6, 0)

    def set_max_vel_x(self, params):

        self.client.update_configuration({'max_vel_x': params})
        rospy.set_param('/move_base/TrajectoryPlannerROS/max_vel_x', params)

    def get_local_goal(self):

        local_goal = None
        while local_goal is None:
            try:
                local_goal = rospy.wait_for_message('local_goal', Pose, timeout=5)
            except:
                pass
        return local_goal

    def set_global_goal(self, x = 6, y = 6, yaw = 0):
        self.nav_as.wait_for_server()
        try:
            self.nav_as.send_goal(self.global_goal)
            print("Published globe goal position!")
        except (rospy.ServiceException) as e:
            print ("/move_base service call failed")
