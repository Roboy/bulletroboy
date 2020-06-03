# Roboy in pybullet

## Setup 
*Tested on Ubuntu 18.04 and Windows 10*
- install python>=3.6
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/)
  ** Always make sure ros2 is sourced in every new terminal!!
  ```bash
#if you installed ros2 from binaries
. /opt/ros/eloquent/setup.bash
#if you installed ros2 from source
. ~/ros2_eloquent/install/setup.bash
```
- create a ros2 workspace
```bash
#create a directory(e.g dev_ws) 
mkdir dev_ws
#create an src directory: this is where all packages should be
mkdir dev_ws/src
cd dev_ws/src
```

- get Roboy3 models, this repo and roboy communication repo:
```bash
git clone  https://github.com/Roboy/roboy3_models.git -b bullet
git clone  https://github.com/Roboy/bulletroboy.git
git clone https://github.com/Roboy/roboy_communication.git -b dashing-ss20-EFC57-Collision_message
```

- resolve dependencies
```bash
sudo rosdep install -i --from-path src --rosdistro eloquent -y
```
-  navigate to dev_ws and build the workspace

```bash
#to build ALL packages in workspace
colcon build
#to build specific package in workspace
colcon build --packages-select <name-of-pkg>
```

## Example

Build the roboy_simulation_msgs package and bulletroboy package
```bash
cd dev_ws
colcon build --packages-select roboy_simulation_msgs bulletroboy
```
Run the simulation example, where 
- Roboy draws a predefined circle using inverse kinematics 
- the joint states and collisions are published with ROS2.
```bash
# source the built packages (from the dev_ws directory)
. /install/setup.bash
# run ros2 node
ros2 run bulletroboy ros2_roboy  --model-path /path/to/roboy3_models/upper_body/bullet.urdf
# i.e. python ros2_roboy.py  --model-path ../../roboy3_models/upper_body/bullet.urdf

# in a new terminal, to see current joint states
ros2 topic echo /roboy/simulation/joint_state

# in a new terminal, to see collisions (need to source roboy_simulation_msgs package first)
. /install/setup.bash
ros2 topic echo /roboy/simulation/collision
```

Run the cage simulation of a human figure with attached tendons.
```bash
cd bulletroboy/src
# use python or python3 depending on your system setup
python cage_simulation.py
```
