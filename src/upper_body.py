import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
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
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)

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

height = 640
width = 640
aspect = width/height

fov, nearplane, farplane = 60, 0.01, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

init_up_vector = (0, 0, 1)
def camera(link_id):
    com_p, com_o, _, _, _, _ = p.getLinkState(roboy, link_id)
    com_t = list(com_p)
    com_p = list(com_p)

    com_t[1] = com_p[1] - 5

    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    init_camera_vector = com_t
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)

    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
    w, h, img, depth, mask = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # p.addUserDebugLine(lineFromXYZ=com_p, lineToXYZ=camera_vector, lifeTime=0)
    return w,h,img

def to_cv2(w,h,img,right):
    rgba_pic = np.array(img, np.uint8).reshape((h, w, 4))
    if right:
        rgba_pic = np.roll(rgba_pic, 300)
        pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
        # pic = pic[0:height,300:width]
    else:
        rgba_pic = np.roll(rgba_pic, -300)
        pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
        # pic = pic[0:height,0:width-300]

    return pic
rate = rospy.Rate(100)

cv2.namedWindow("window", cv2.WINDOW_NORMAL)
cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
# cv2.namedWindow("window", cv2.WINDOW_AUTOSIZE)
# cv2.setWindowProperty("window",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

while (not rospy.is_shutdown()):
    left = camera(11)
    right = camera(12)
    left_pic = to_cv2(left[0],left[1],left[2],0)
    right_pic = to_cv2(right[0],right[1],right[2],1)
    vis = np.concatenate((right_pic, left_pic), axis=1)
    cv2.imshow('window', vis)
    cv2.waitKey(1)
    # rate.sleep()
    p.stepSimulation()