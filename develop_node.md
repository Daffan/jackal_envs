A resource for understanding the sensor_msgs/LaserScan

jackal installation instruction:
 https://gist.github.com/vfdev-5/57a0171d8f5697831dc8d374839bca12

topic: front/scan would be used to generate the observation

tutorial for sending goal to the navigation: \
http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals



Launch the jackal navigation:

source /opt/ros/melodic/setup.bash
roslaunch jackal_gazebo jackal_world.launch gui:=false config:=front_laser
roslaunch jackal_navigation odom_navigation_demo.launch

No idea but the simulation time runs slower compared to the real time:
quick experiment:
>>> a = rospy.get_time(); time.sleep(60); b = rospy.get_time(); print(b-a)
46.545
60s real time, but the simulation world only goes 46s

GAZEBO_RESOURCE_PATH: /usr/share/gazebo-9/worlds

empty world is literally just an empty world, it has nothing in it

things in /opt/ros/melodic/share/jackal_description/urdf/configs/front_laser
# The front_laser configuration of Jackal is sufficient for
# basic gmapping and navigation. It is mostly the default
# config, but with a SICK LMS100 series LIDAR on the front,
# pointing forward.

JACKAL_LASER=1

file /opt/ros/melodic/share/jackal_description/urdf/jackal.urdf.xacro
file /opt/ros/melodic/share/jackal_description/urdf/jackal.gazebo
file /opt/ros/melodic/share/jackal_description/urdf/accessories.urdf.xacro
has mostly all the description of the robot

Where to put the assets files? (choose the first solution for now)
(1) put them directly under envs folders. Benefit: don't have to source the workspace
Disadvantage: have to write globe variables in the .bashrc files
Example: export GAZEBO_ASSETS_PATH=/home/zifan/gym-gazebo/gym_gazebo/envs/assets/
(2) build a package in a workspace: has to source the setup every time


Mount a VLP-16 Liader:
https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/
NOt enough blocks issues: repalce the origin blocks in the first link with the normal one

current problem:
(1) /front/scan publish pointcloud2 instead of LaserScan
(2) rivz cannot recogonize the robot
