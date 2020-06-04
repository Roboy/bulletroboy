# Roboy in pybullet

## Setup 
*Tested on Ubuntu 18.04 and Windows 10*
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

# Building packages in workspace
cd roboy_ws
source /opt/ros/eloquent/setup.bash
colcon build --symlink-install --packages-select bulletroboy roboy_simulation_msgs

# Sourcing workspace overlay (ROS2 Wiki recommends opening a new terminal before this step)
source /opt/ros/eloquent/setup.bash
cd roboy_ws
. install/setup.bash
```

## Examples
To be able to run any node after the package is built, you need to source the workspace overlay.

Run the simulation example, where 
- Roboy draws a predefined circle using inverse kinematics 
- the joint states are published with ROS2.
```bash
# run ros2 node
ros2 run bulletroboy ros2_roboy

# in a new terminal, to see current joint states
ros2 topic echo /roboy/simulation/joint_state
```

Run the cage simulation of a human figure with attached tendons.
```bash
ros2 run bulletroboy cage_simulation  # see node help for execution modes
```
