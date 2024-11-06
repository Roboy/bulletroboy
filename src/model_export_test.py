import pybullet as p
import pybullet_data
import time

# Initialize the PyBullet simulation
p.connect(p.GUI)

# Set additional search path to find URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the URDF model
# model_path = "/home/roboy/workspace/alice_models/alice-head-v1/model.urdf"
model_path = "/home/roboy/workspace/alice_models/viper-arms-v2/model.urdf"
robot_id = p.loadURDF(model_path)
p.changeDynamics(robot_id, -1, mass=0)  # Use -1 to target the base link


# Enable real-time simulation
p.setRealTimeSimulation(1)

# Get the number of joints
num_joints = p.getNumJoints(robot_id)

# Create sliders for each joint
joint_sliders = []
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robot_id, joint_index)
    joint_name = joint_info[1].decode("utf-8")  # Joint name is in bytes, so we decode it
    slider = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0)  # Slider range set to [-π, π]
    joint_sliders.append(slider)

# Run the simulation and set joint positions according to slider values
try:
    while True:
        # Update joint positions based on slider values
        for joint_index in range(num_joints):
            slider_value = p.readUserDebugParameter(joint_sliders[joint_index])
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=slider_value
            )
        time.sleep(0.01)  # Small delay for stability
except KeyboardInterrupt:
    pass

# Disconnect from the simulation
p.disconnect()
