# Roboy in pybullet

## Setup 
*Tested on Ubuntu 18.04 and Windows 10*
- install python>=3.6
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/)
- get Roboy3 models and this repo:
```bash
git clone  https://github.com/Roboy/roboy3_models.git -b bullet
git clone  https://github.com/Roboy/bulletroboy.git
```

## Example
Run the simulation example, where 
- Roboy draws a predefined circle using inverse kinematics 
- the joint states are published with ROS2.
```bash
cd bulletroboy/src
# use python or python3 depending on your system setup
python ros2_roboy.py  --model-path /path/to/roboy3_models/upper_body/bullet.urdf
# i.e. python ros2_roboy.py  --model-path ../../roboy3_models/upper_body/bullet.urdf

# in a new terminal, to see current joint states
ros2 topic echo /roboy/simulation/joint_state
```

Run the cage simulation of a human figure with attached tendons.
```bash
cd bulletroboy/src
# use python or python3 depending on your system setup
python cage_simulation.py
```
