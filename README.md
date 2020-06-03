# Roboy in pybullet

## Setup 
*Tested on Ubuntu 18.04 and Windows 10*
- install python>=3.6
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/)
- get Roboy3 models, this repo and roboy communication repo:
```bash
git clone  https://github.com/Roboy/roboy3_models.git -b bullet
git clone  https://github.com/Roboy/bulletroboy.git
git clone https://github.com/Roboy/roboy_communication.git -b dashing-ss20-EFC57-Collision_message
```

## Example
Always  make sure ros2 is sourced in every new terminal!!

Build the roboy_simulation_msgs package 
```bash
cd roboy_communication/roboy_simulation_msgs

python colcon build
```
Run the simulation example, where 
- Roboy draws a predefined circle using inverse kinematics 
- the joint states and collisions are published with ROS2.
```bash
# source roboy_simulation_msgs package
. /path/to/roboy_communication/roboy_simulation_msgs/install/setup.bash
# navigate to bullet roboy repo
cd bulletroboy/src
# use python or python3 depending on your system setup
python ros2_roboy.py  --model-path /path/to/roboy3_models/upper_body/bullet.urdf
# i.e. python ros2_roboy.py  --model-path ../../roboy3_models/upper_body/bullet.urdf

# in a new terminal, to see current joint states
ros2 topic echo /roboy/simulation/joint_state

# in a new terminal, to see collisions (source roboy_simulation_msgs first)
. /path/to/roboy_communication/roboy_simulation_msgs/install/setup.bash
ros2 topic echo /roboy/simulation/collision
```

Run the cage simulation of a human figure with attached tendons.
```bash
cd bulletroboy/src
# use python or python3 depending on your system setup
python cage_simulation.py
```
