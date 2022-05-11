#Simulated Roboy 3.0 using pyBullet
- Python >= 3.6
- ROS melodic and later
- Depends on
  - `pybullet`
  - `pyquaternion`
  - `cv2`
  - `numpy`
  - `cv_bridge` ROS package
  - `sensor_msgs` ROS package
  - `visualization_msgs` ROS package
  - `geometry_msgs` ROS package
  - [`roboy3_models`](https://github.com/Roboy/roboy3_models/tree/bullet) ROS package (branch *bullet*)

To start the ROS node, start `roscore` and use:
```rosrun bulletroboy ik.py```







