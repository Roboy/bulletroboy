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
colcon build --symlink-install --packages-select bulletroboy roboy_simulation_msgs roboy_control_msgs

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
ros2 run bulletroboy roboy_simulation

# in a new terminal, to see current joint states
ros2 topic echo /roboy/simulation/joint_state

# in a new terminal, to see collisions (need to source roboy_simulation_msgs package first)
. /install/setup.bash
ros2 topic echo /roboy/simulation/collision
```

Run the cage simulation of a human figure with attached tendons.
```bash
ros2 run bulletroboy cage_simulation  # see node help for execution modes
```

## Running the loop.

# The Operator:
To test the operator reacting to the collisions message follow these instructions:

- open three terminals and in each of them run the following lines of code:

```bash
cd roboy_ws
source /opt/ros/eloquent/setup.bash
. install/setup.bash
cd src/bulletroboy/bulletroboy
```
- Now in the first terminal run this line to initiate the collisions message (test) publisher:

```bash
python3 test_publisher.py
```

- In the second terminal run this line to check the running ros2 collisions message:

```bash
ros2 topic echo /roboy/exoforce/collisions
```

- In the third terminal run this line. (Curretly, the operator follows the collisions message from test_publisher. Once its merged with the other loop components this will change.)

```bash
python3 cage_simulation.py -m forces
```

