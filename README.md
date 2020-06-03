# Roboy in pybullet

## Setup 
*Tested on Ubuntu 18.04 and Windows 10*
- install python>=3.6
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/)

  ** *Always make sure ros2 is sourced in every new terminal as fllows*
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
```

- clone Roboy3 models and this repo in dev_ws/src:
```bash
cd dev_ws/src
git clone  https://github.com/Roboy/roboy3_models.git -b bullet
git clone  https://github.com/Roboy/bulletroboy.git
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
Make sure you build bulletroboy as mentioned in previous section.

Run the simulation example, where 
- Roboy draws a predefined circle using inverse kinematics 
- the joint states are published with ROS2.
```bash
# source the built packages (from the dev_ws directory)
. /install/setup.bash
# run ros2 node
ros2 run bulletroboy ros2_roboy  --model-path /path/to/roboy3_models/upper_body/bullet.urdf
# i.e. ros2 run bulletroboy ros2_roboy  --model-path src/roboy3_models/upper_body/bullet.urdf

# in a new terminal, to see current joint states
ros2 topic echo /roboy/simulation/joint_state
```

Run the cage simulation of a human figure with attached tendons.
```bash
cd bulletroboy/src
# use python or python3 depending on your system setup
python cage_simulation.py
```
