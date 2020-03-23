import pybullet as p
import pybullet_data
import time
import numpy as np

import rospy
from sensor_msgs.msg import JointState

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

rospy.init_node("bulletroboy")

p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0,0,-9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.loadURDF("samurai.urdf", 0,2,0)
p.loadURDF("teddy_vhacd.urdf", 0,-1,2)
p.loadMJCF("mjcf/humanoid_fixed.xml")
# p.loadURDF("humanoid/humanoid.urdf", 0,-2,0)
roboy = p.loadURDF('/home/roboy/workspace/roboy3/src/robots/upper_body/model.urdf',basePosition=(0,1,0.5), useFixedBase=1)
joints = []
for i in range(p.getNumJoints(roboy)):
    joints.append(p.getJointInfo(roboy, i)[1].decode("utf-8"))
    p.setJointMotorControl2(roboy, i, p.POSITION_CONTROL, targetPosition=0, force=500)

def joint_target_cb(msg):
    for i in range(len(msg.name)):
        j = msg.name[i]
        try:
            idx = joints.index(j)
            p.setJointMotorControl2(roboy,
                                    idx,
                                    p.POSITION_CONTROL,
                                    targetPosition=msg.position[i],
                                    force=100)
        except:
            rospy.logwarn("Joint %s was not found in pybullet model"%j)



joint_target_sub = rospy.Subscriber("/cardsflow_joint_states", JointState, joint_target_cb)


fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

init_up_vector = (0, 0, 1) 
def kuka_camera():
    com_p, com_o, _, _, _, _ = p.getLinkState(roboy, 11)
    com_t = list(com_p)
    com_p = list(com_p)

    com_t[1] = com_p[1] - 5

    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    init_camera_vector = com_t
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)

    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
    img = p.getCameraImage(50, 50, view_matrix, projection_matrix)
    p.addUserDebugLine(lineFromXYZ=com_p, lineToXYZ=camera_vector, lifeTime=0)
    return img

rate = rospy.Rate(100)

while (not rospy.is_shutdown()):
    kuka_camera()
    # rate.sleep()
    p.stepSimulation()