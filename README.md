# Simulated Roboy 3.0 using pyBullet
- Python >= 3.6
- ROS melodic and later

## Dependencies
  - `pybullet`
  - `pyquaternion`
  - `cv2`
  - `numpy`
  - `cv_bridge` ROS package
  - `sensor_msgs` ROS package
  - `visualization_msgs` ROS package
  - `geometry_msgs` ROS package
  - [`roboy3_models`](https://github.com/Roboy/roboy3_models/tree/bullet) ROS package (branch *bullet*)

## Run
1. Build ROS workspace with `roboy3_models` and `bulletroboy` packages
2. Start ROS master with `roscore`
3. Start the simulation ROS node 
```rosrun bulletroboy ik.py```








