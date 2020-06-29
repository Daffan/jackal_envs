# jackal_envs
Jakcal environment for navigation: 
## Installation 
1. Clone this github repository 
   ```bash
   # Checkout the latest stable release
   git clone --branch stable https://github.com/Daffan/jackal_envs.git
   cd jackal_envs
   pip install -e .
   ```
2. Prepare workspace 
  ```bash
  mkdir -p jackal_ws/src
  cd jackal_ws/src
  git clone https://github.com/Daffan/jackal_helper.git
  cd ../
  catkin_make
  ```
3. Test
  ```bash
  source ~/jackal_ws/devel/setup.bash
  python ~/test.py
  ```
  This will run an eqisode of navigation to a goal position x = 6, y = 6, with each step 1s duration. 
