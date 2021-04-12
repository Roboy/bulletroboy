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
git clone https://github.com/Roboy/bulletroboy.git -b exoforce

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

1. [ros2-web-bridge](https://github.com/RobotWebTools/ros2-web-bridge) needs to be compiled and running to comunicate with the construct app.
2. [ros1_bridge](https://github.com/ros2/ros1_bridge) needs to be compiled and running to comunication with the force control node.
3. [roboy_plexus](https://github.com/Roboy/roboy_plexus) needs to be configured and running to comunicate with the motors.
4. [force_control](https://github.com/Roboy/exoforce_force_control) needs to be running.

Run the system using the launch file.
```bash
ros2 launch bulletroboy exoforce.launch.py # use -s to list arguments
```
