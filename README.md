# Roboy in pybullet

## Setup 
*Tested on Ubuntu 18.04*
- install python>=3.6
- install [pybullet](https://github.com/bulletphysics/bullet3/blob/master/README.md#pybullet)
- install [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)

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
git clone https://github.com/Roboy/roboy3_models.git -b caspr
git clone https://github.com/Roboy/roboy_communication.git
git clone https://github.com/Roboy/bulletroboy.git -b simulated-animus

cd roboy_ws
catkin_make
source devel/setup.bash
rosrun bulletroboy ik.py
```
One can move the robot with the mouse pointer or sending inverse kinematics targets on the ROS topic `/bullet_ik`
