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
