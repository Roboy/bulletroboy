import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

rospy.init_node("bulletroboy")
caml_pub = rospy.Publisher('roboy/sensors/caml/compressed', CompressedImage)
camr_pub = rospy.Publisher('roboy/sensors/camr/compressed', CompressedImage)
bridge = CvBridge()
publish_pics = True

p.connect(p.GUI)
p.setRealTimeSimulation(1)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)

p.resetSimulation()
p.setGravity(0,0,-9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setAdditionalSearchPath("/home/roboy/workspace/avatar_ws/src/icub-gazebo/icub/l_hand")
plane = p.loadURDF("plane.urdf")
# p.loadURDF("samurai.urdf", 0,2,0)
# p.loadURDF("teddy_vhacd.urdf", 0,-1,2)
# p.loadMJCF("mjcf/half_cheetah.xml")
# hand = p.loadSDF("/home/roboy/workspace/avatar_ws/src/icub-gazebo/icub/l_hand/l_hand.sdf")
chessboard = p.loadSDF("/home/roboy/workspace/roboy3/src/robots/chessboard/chessboard.sdf", p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
# hand = p.loadURDF("/home/roboy/workspace/avatar_ws/src/simox_ros/sr_grasp_description/urdf/shadowhand.urdf",basePosition=(0.2,0,0.15), baseOrientation=(-1.57, 0,0,1.57))
path = "/home/roboy/workspace/avatar_ws/src/sr_common/sr_description/robots/model.urdf"
# left_hand = p.loadURDF(path)
p.resetBasePositionAndOrientation(chessboard[0], posObj=(0,0,0.7),  ornObj=(0,0,0,1))
# hand_joint_names = ["FFJ4", "FFJ3", "FFJ2", "FFJ1"]
# hand_joints = []
#
# for i in range(p.getNumJoints(hand)):
#     hand_joints.append(p.getJointInfo(hand, i)[1].decode("utf-8"))
#     p.setJointMotorControl2(hand, i, p.POSITION_CONTROL, targetPosition=0, force=100)

# p.loadURDF("half_cheetah.urdf")
# p.loadURDF("humanoid/humanoid.urdf", 0,-2,0)
# roboy = p.loadURDF('/home/roboy/workspace/roboy3/src/robots/upper_body/model.urdf',basePosition=(0.75,0,0.2), baseOrientation=(0,0,-0.7071,0.7071),useFixedBase=1)
roboy = p.loadURDF('/home/roboy/workspace/roboy3/src/robots/upper_body/model.urdf', basePosition=(0.0,0.7,0.4), useFixedBase=1)
table = p.loadURDF('/home/roboy/.local/lib/python3.6/site-packages/pybullet_data/table/table.urdf')
p.loadURDF('/home/roboy/.local/lib/python3.6/site-packages/pybullet_data/jenga/jenga.urdf', basePosition=(0.0,0.4,1))
p.loadURDF('/home/roboy/.local/lib/python3.6/site-packages/pybullet_data/domino/domino.urdf',basePosition=(0.2,0.4,1))
p.loadURDF('/home/roboy/.local/lib/python3.6/site-packages/pybullet_data/lego/lego.urdf',basePosition=(0.0,0.3,1))
joints = []
for i in range(p.getNumJoints(roboy)):
    joints.append(p.getJointInfo(roboy, i)[1].decode("utf-8"))
    p.setJointMotorControl2(roboy, i, p.POSITION_CONTROL, targetPosition=0, force=500)

def hand_joints_cb(msg):
    active = []
    for i in range(len(msg.name)):
        j = msg.name[i]
        try:
            idx = joints.index(j)
            active.append(idx)
        except:
            rospy.logwarn("Joint %s was not found in pybullet model"%j)
    try:
        p.setJointMotorControlArray(roboy,
                                active,
                                p.POSITION_CONTROL,
                                targetPositions=msg.position)
    except:
        rospy.logwarn_throttle(1, "oops, could not set joint motor control cmd")

def joint_target_cb(msg):
    active = []
    for i in range(len(msg.name)):
        j = msg.name[i]
        try:
            idx = joints.index(j)
            active.append(idx)
            # p.setJointMotorControl2(roboy,
            #                         idx,
            #                         p.POSITION_CONTROL,
            #                         targetPosition=msg.position[i],
            #                         force=1000)
        except:
            rospy.logwarn("Joint %s was not found in pybullet model"%j)
    p.setJointMotorControlArray(roboy,
                                active,
                                p.POSITION_CONTROL,
                                targetPositions=msg.position)
                                # positionGains=kps,
                                # velocityGains=kds)

def fingertip_cb(msg):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius = 0.02)
    col = p.createCollisionShape(p.GEOM_SPHERE, radius = 0.02)
    mass = 0.01

    link_Masses = [1]
    linkCollisionShapeIndices = [col]
    linkVisualShapeIndices = [vis]
    linkPositions = [[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]]
    linkOrientations = [[0, 0, 0, 1]]
    linkInertialFramePositions = [[0, 0, 0]]
    linkInertialFrameOrientations = [[0, 0, 0, 1]]
    indices = [0]
    jointTypes = [p.JOINT_FIXED]
    axis = [[0, 0, 1]]
    basePosition = p.getLinkState(roboy, 20)[0]
    baseOrientation = [0, 0, 0, 1]
    p.createMultiBody(mass,
                      col,
                      vis,
                      basePosition,
                      baseOrientation,
                      linkMasses=link_Masses,
                      linkCollisionShapeIndices=linkCollisionShapeIndices,
                      linkVisualShapeIndices=linkVisualShapeIndices,
                      linkPositions=linkPositions,
                      linkOrientations=linkOrientations,
                      linkInertialFramePositions=linkInertialFramePositions,
                      linkInertialFrameOrientations=linkInertialFrameOrientations,
                      linkParentIndices=indices,
                      linkJointTypes=jointTypes,
                      linkJointAxis=axis)

# joint_target_sub = rospy.Subscriber("/cardsflow_joint_states", JointState, joint_target_cb)
joint_target_sub = rospy.Subscriber("/cardsflow_joint_states", JointState, joint_target_cb)
# joint_target_sub = rospy.Subscriber("/joint_targets", JointState, joint_target_cb)
hand_joints_sub = rospy.Subscriber("/senseglove/joint_states", JointState, hand_joints_cb)
# fingertip_sub = rospy.Subscriber("/senseglove/fingertip", PoseStamped, fingertip_cb)

height = 720
width = 640
aspect = width/height

fov, nearplane, farplane = 100, 0.01, 100
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
    w, h, img, depth, mask = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=0, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags=p.ER_NO_SEGMENTATION_MASK)
    # p.addUserDebugLine(lineFromXYZ=com_p, lineToXYZ=camera_vector, lifeTime=0)
    return w,h,img

def to_cv2(w,h,img,right):
    rgba_pic = np.array(img, np.uint8).reshape((h, w, 4))
    if right:
        # rgba_pic = np.roll(rgba_pic, 300)
        pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
        # pic = pic[0:height,300:width]
    else:
        # rgba_pic = np.roll(rgba_pic, -300)
        pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
        # pic = pic[0:height,0:width-300]

    return pic
rate = rospy.Rate(100)

cv2.namedWindow("window", cv2.WINDOW_NORMAL)
cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
# cv2.namedWindow("window", cv2.WINDOW_AUTOSIZE)
# cv2.setWindowProperty("window",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

while (not rospy.is_shutdown()):
    left = camera(40)
    right = camera(41)
    left_pic = to_cv2(left[0],left[1],left[2],0)
    right_pic = to_cv2(right[0],right[1],right[2],1)
    if (publish_pics):
        caml_pub.publish(bridge.cv2_to_compressed_imgmsg(left_pic))
        camr_pub.publish(bridge.cv2_to_compressed_imgmsg(right_pic))
    vis = np.concatenate((left_pic, right_pic), axis=1)
    cv2.imshow('window', vis)
    cv2.waitKey(1)
    # rate.sleep()
    # p.stepSimulation()