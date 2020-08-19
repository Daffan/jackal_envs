# jackal_envs
Jakcal environment for navigation: 
## Installation 
Assume the installation in ros-melodic with Ubuntu 18.04

1. Clone this github repository 
   ```bash
   git clone --branch stable https://github.com/Daffan/jackal_envs.git
   cd jackal_envs
   pip install -e .
   pip install -r requirements.txt
   ```
2. Install dependencies
   ```bash
   sudo apt-get install ros-melodic-robot-localization ros-melodic-controller-manager ros-melodic-joint-state-controller ros-melodic-diff-drive-controller ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-gazebo-plugins             ros-melodic-lms1xx ros-melodic-pointgrey-camera-description ros-melodic-roslint ros-melodic-amcl ros-melodic-gmapping      ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-message-runtime ros-melodic-topic-tools ros-melodic-teleop-twist-joy
   sudo apt-get install ros-melodic-velodyne-simulator
   ```
2. Prepare workspace 
  ```bash
  mkdir -p jackal_ws/src
  cd jackal_ws/src
  git clone https://github.com/Daffan/jackal_helper.git
  git clone https://github.com/jackal/jackal.git
  git clone https://github.com/jackal/jackal_simulator.git
  git clone https://github.com/jackal/jackal_desktop.git
  git clone https://github.com/ros-visualization/interactive_marker_twist_server.git
  cd ../
  catkin_make
  ```
3. Test
  ```bash
  source /devel/setup.bash
  python ../jackal_envs/test.py
  ```
  This will run an eqisode of navigation to a goal position x = 6, y = 6, with each step 1s duration. 
