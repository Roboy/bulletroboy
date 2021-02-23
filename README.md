# ExoForce

## Setup 
*Tested on Ubuntu 20.04*
- install python>=3.6
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS2 Foxy Fitzroy](https://index.ros.org/doc/ros2/Installation/Foxy/)
```bash
# Creating ROS2 workspace
mkdir -p roboy_ws/src

# Cloning repos to src directory
cd roboy_ws/src
git clone https://github.com/Roboy/roboy3_models.git -b bullet
git clone https://github.com/Roboy/roboy_communication.git -b dashing
git clone https://github.com/Roboy/bulletroboy.git

# Building packages in workspace
cd roboy_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select bulletroboy roboy_simulation_msgs roboy_control_msgs roboy_middleware_msgs

# Sourcing workspace overlay (ROS2 Wiki recommends opening a new terminal before this step)
source /opt/ros/foxy/setup.bash
cd roboy_ws
. install/setup.bash
```

## Usage
To be able to run any node after the package is built, you need to source the workspace overlay.

Run the system with the operator simulation.
```bash
ros2 launch bulletroboy cage_sim.launch.py  # use -s to list arguments
```

Run the system with the real exoforce cage. 
(Each command must be executed in a new terminal. Operator and exoforce nodes require interaction through the terminal and cannot be added to the launch file yet.)
```bash
ros2 launch bulletroboy exoforce.launch.py # use -s to list arguments
ros2 run bulletroboy operator
ros2 run bulletroboy exoforce
```
