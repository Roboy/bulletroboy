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
One can move the robot with the mouse pointer or sending inverse kinematics targets on the ROS topic `/bullet_ik`.


One can also simulate collisions by using the Pybullet Gui. You can add cubes of different sizes and weights. You can also add objects from the pybullet_data urdf files by clicking on "Add model from pybullet_data" button and writing in the terminal the urdf file of your choice. Also, you can try already predefined tests: 
- drop Cubes of weights 1, 3 and 5kg into the hands of Roboy 
- simulate table
- simulate wall 
- drom a random object on the table
