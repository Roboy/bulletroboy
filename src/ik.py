import pybullet as p
import math
import time
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerUpdate

rospy.init_node("bullet_joints")
joint_target_pub = rospy.Publisher("/joint_targets", JointState, queue_size=1)


msg = JointState()

p.connect(p.GUI)
ob = p.loadURDF("/home/roboy/workspace/roboy3/src/robots/upper_body/model.urdf", useFixedBase=1)
p.setGravity(0,0,-10)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

useOrientation = 0

#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 1
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15
markerVisualId = p.addUserDebugText("X", [0,0,0])
numJoints = p.getNumJoints(ob)
global freeJoints
freeJoints = []
joint_names = {}
efs = {}
for i in range(numJoints):
    info = p.getJointInfo(ob,i)
    if info[2] == p.JOINT_REVOLUTE:
        if info[1] == b'head_axis0' or info[1] == b'head_axis1' or info[1] == b'head_axis2':
            continue
        freeJoints.append(i)
        joint_names[i] = (p.getJointInfo(0, i)[1].decode("utf-8"))
    if info[12] == b'hand_right':
        endEffectorId = i
        print("EF id: " + str(i))
        efs["hand_right"] = i
    if info[12] == b'hand_left':
        efs["hand_left"] = i



def accurateCalculateInverseKinematics(ob, endEffectorId, targetPos, threshold, maxIter, targetOrn=None):
    closeEnough = False
    iter = 0
    dist2 = 1e30
    while (not closeEnough and iter < maxIter):
        if targetOrn is None:
            jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos)
        else:
            jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos, targetOrientation=targetOrn)
        #import pdb; pdb.set_trace()
        for i in range(len(freeJoints)):
            p.resetJointState(ob, freeJoints[i], jointPoses[i])
        ls = p.getLinkState(ob, endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
    #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
    return jointPoses

def ik(msg):
    global freeJoints
    endEffectorId = efs[msg.header.frame_id]
    threshold = 0.001
    maxIter = 100
    pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    orn = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    jointPoses = accurateCalculateInverseKinematics(0, endEffectorId, pos,
                                                    threshold, maxIter, orn)
    if (useSimulation):
        for i in range(len(freeJoints)):
            p.setJointMotorControl2(bodyIndex=0,
                                    jointIndex=freeJoints[i],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=100,
                                    positionGain=1,
                                    velocityGain=0.1)

def marker(msg):
    global freeJoints
    global markerVisualId
    threshold = 0.001
    maxIter = 100
    for i in range(len(msg.poses)):
        pose_msg = msg.poses[i]
        endEffectorId = efs[pose_msg.name]

        pos = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
        orn = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
        markerVisualId = [0,0.7,0,0.7] #p.addUserDebugText("X",pos, replaceItemUniqueId=markerVisualId)
        jointPoses = accurateCalculateInverseKinematics(0, endEffectorId, pos, orn,
                                                        threshold, maxIter)
        if (useSimulation):
            for i in range(len(freeJoints)):
                p.setJointMotorControl2(bodyIndex=0,
                                        jointIndex=freeJoints[i],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=jointPoses[i],
                                        targetVelocity=0,
                                        force=100,
                                        positionGain=1,
                                        velocityGain=0.1)


ik_sub = rospy.Subscriber("/bullet_ik", PoseStamped, ik)

marker_sub = rospy.Subscriber("/interactive_markers/update", InteractiveMarkerUpdate, marker)
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    msg.position = []
    msg.velocity = []
    msg.effort = []
    msg.name = []
    for i in freeJoints:
        js = p.getJointState(0, i)
        msg.position.append(js[0])
        msg.velocity.append(0)
        msg.effort.append(0)
        msg.name.append(joint_names[i])
    joint_target_pub.publish(msg)
    rate.sleep()
p.disconnect()

    # ls = p.getLinkState(ob, endEffectorId)
    # if (hasPrevPose):
    #     p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
    #     p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    # prevPose = pos
    # prevPose1 = ls[4]
    # hasPrevPose = 1

while True:
    if (useRealTimeSimulation):
        t = time.time()  #(dt, micro) = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f').split('.')
        #t = (dt.second/60.)*2.*math.pi
    else:
        t = t + 0.001

    if (useSimulation and useRealTimeSimulation == 0):
        p.stepSimulation()

    pos = [0.2 * math.cos(t)-0.4, -0.4, 0. + 0.2 * math.sin(t) + 0.7]
    threshold = 0.001
    maxIter = 100
    jointPoses = accurateCalculateInverseKinematics(ob, endEffectorId, pos,
                                                    threshold, maxIter)
    if (useSimulation):
        for i in range(len(freeJoints)):
            p.setJointMotorControl2(bodyIndex=ob,
                                    jointIndex=freeJoints[i],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=100,
                                    positionGain=1,
                                    velocityGain=0.1)
    ls = p.getLinkState(ob, endEffectorId)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1