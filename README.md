# Roboy in pybullet

## Setup
*Tested on Ubuntu 18.04 and 20.04*
- install python>=3.6
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu) or [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation)

```bash
# setup ROS for python3
sudo pip3 install rospkg catkin_pkg

# install other requirements
pip3 install --user pyquaternion
pip3 install pyyaml

# Create ROS workspace
mkdir -p roboy_ws/src

# Clone repos to src directory
cd roboy_ws/src
git clone https://github.com/Roboy/roboy3_models.git
git clone https://github.com/Roboy/roboy_communication.git
git clone https://github.com/Roboy/bulletroboy.git

cd roboy_ws
catkin_make
source devel/setup.bash
rosrun bulletroboy ik.py
```
