# Roboy in pybullet

## Setup 
*Tested on Ubuntu 18.04*
- install python>=3.6
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/)
```bash
# Creating ROS2 workspace
mkdir -p roboy_ws/src

# Cloning repos to src directory
cd roboy_ws/src
git clone https://github.com/Roboy/roboy3_models.git -b bullet
git clone https://github.com/Roboy/roboy_communication.git -b dashing
git clone https://github.com/Roboy/bulletroboy.git
cd bulletroboy/
git checkout retargeting

# Building packages in workspace
cd roboy_ws
source /opt/ros/eloquent/setup.bash
colcon build --symlink-install --packages-select bulletroboy roboy_simulation_msgs roboy_control_msgs

# Sourcing workspace overlay (ROS2 Wiki recommends opening a new terminal before this step)
source /opt/ros/eloquent/setup.bash
cd roboy_ws
. install/setup.bash

ros2 run bulletroboy retargeting
