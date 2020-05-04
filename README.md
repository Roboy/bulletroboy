# Roboy in pybullet

## Setup
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/)
- get Roboy3 models and this repo:
```
git clone  https://github.com/Roboy/roboy3_models.git -b bullet
git clone  https://github.com/Roboy/bulletroboy.git
```

## Example
Run the simulation example, where Roboy draws a predefined circle using inverse kinematics and joint states are published with ROS2.
```
cd bulletroboy/src
python ros2_roboy.py  --model-path /path/to/roboy3_models/upper_body/bullet.urdf
# i.e. python ros2_roboy.py  --model-path ../../roboy3_models/upper_body/bullet.urdf

# in a new terminal, to see current joint states
ros2 topic echo /roboy/simulation/joint_state
```
