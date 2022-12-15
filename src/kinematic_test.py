#!/usr/bin/python3
import pybullet as p
import math
import time
import rospy
from sensor_msgs.msg import JointState

import numpy as np

from pyquaternion import Quaternion

import rospkg
rospack = rospkg.RosPack()
robots_path = rospack.get_path('robots')

import argparse
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--wait', dest='wait', action='store_const',
                    const=True, default=False,
                    help='wait 3s before autocalibration')

args = parser.parse_args()

if args.wait:
    rospy.logwarn("Waiting 3 seconds before launching IK node...")
    rospy.sleep(3)



def quaternion_multiply(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                     -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)


rospy.init_node("bullet_joints")
topic_root = "/roboy/oxford"
joint_target_pub = rospy.Publisher(topic_root+"/control/joint_targets", JointState, queue_size=1)

msg = JointState()

p.connect(p.GUI) # (p.GUI)
roboy = ob = p.loadURDF(robots_path+"/musc_shoulder/model.urdf", useFixedBase=1, basePosition=(0,0,1), baseOrientation=(0,0,0.7071,0.7071))

p.setGravity(0,0,-10)
t = 0.


useSimulation = 1
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)


global freeJoints, numJoints, init_orn,  head_initialized, right_initialized, right_orn_offset
numJoints = p.getNumJoints(ob)


if not rospy.has_param('publish_cardsflow'):
    rospy.logwarn("Set param /publish_cardsflow true to forward joint poses")
    rospy.set_param('publish_cardsflow', False)

init_orn = (0,0,0,1)
freeJoints = []
joint_names = {}
efs = {}
for i in range(numJoints):
    info = p.getJointInfo(ob,i)
    if info[2] == p.JOINT_REVOLUTE:
        # if info[1] == b'head_axis0' or info[1] == b'head_axis1' or info[1] == b'head_axis2':
        #     continue
        freeJoints.append(i)
        joint_names[i] = (p.getJointInfo(0, i)[1].decode("utf-8"))
        rospy.loginfo("Added joint %s to freeJoints"%joint_names[i])
    if info[12] == b'hand_right':
        endEffectorId = i
        print("EF id: " + str(i))
        efs["hand_right"] = i
    if info[12] == b'hand_left':
        efs["hand_left"] = i


init_up_vector = (0, 0, 1)


def isRight(id):
    info = p.getJointInfo(ob,id)
    return "right" in str(info[12])

def idFromName(joint_name):
    for i in range(numJoints):
        if p.getJointInfo(0,i)[1].decode("utf-8") == joint_name:
            return i
    return None

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
        for idx,pos in zip(freeJoints,jointPoses): #range(len(freeJoints)):#p.getNumJoints(ob)):#
            # p.resetJointState(ob, freeJoints[i], jointPoses[i])
            if (isRight(idx) and isRight(endEffectorId)) or (not isRight(idx) and not isRight(endEffectorId)):
                p.setJointMotorControl2(bodyIndex=ob,
                                            jointIndex=idx,#freeJoints[i],
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=pos,
                                            maxVelocity=0.5)#jointPoses[i])
                                            # targetVelocity=0,
                                            # force=1,
                                            # positionGain=5,
                                            # velocityGain=0.1)
        ls = p.getLinkState(ob, endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
    #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
    return jointPoses

def ik(msg):

    global freeJoints, markerVisualId, head_initialized, right_initialized, orn_offset, left_initialized
    pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    orn = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]


    if msg.header.frame_id == "head":
        if not head_initialized:
            pos = list(pos)
            pos[2] -= 0.4
            rospy.logwarn_throttle(1, "head: " +  str(pos))
            p.resetBasePositionAndOrientation(0, pos, (0,0,0.7071,0.7071))
            head_initialized = True

    else:#if msg.header.frame_id == "hand_left":
        if not right_initialized and msg.header.frame_id == "hand_right":
            orn = list(orn)
            ctlr_orn = Quaternion(orn[3], orn[0], orn[1], orn[2])
            ctlr_orn = ctlr_orn.inverse
            orn_offset["hand_right"] = [ctlr_orn[1], ctlr_orn[2], ctlr_orn[3], ctlr_orn[0]]
            rospy.logwarn("Right controller orientation offset: " + str(orn_offset["hand_right"]))
            # rospy.logwarn_throttle(1,right_orn_offset)
            right_initialized = True

        if not left_initialized and msg.header.frame_id == "hand_left":
            orn = list(orn)
            ctlr_orn = Quaternion(orn[3], orn[0], orn[1], orn[2])
            ctlr_orn = ctlr_orn.inverse
            orn_offset["hand_left"] = [ctlr_orn[1], ctlr_orn[2], ctlr_orn[3], ctlr_orn[0]]
            rospy.logwarn("Left controller orientation offset: " + str(orn_offset["hand_left"]))
            # rospy.logwarn_throttle(1,left_orn_offset)
            left_initialized = True

        else:
            endEffectorId = efs[msg.header.frame_id]
            threshold = 0.03
            maxIter = 80

            # if first:
            #     init_orn = orn
            # else:
            rospy.loginfo_throttle(1, orn)
            corrected_orn = quaternion_multiply(orn, orn_offset[msg.header.frame_id]) #(0,0,0.7071,0.7071))
            # corrected_orn = quaternion_multiply(corrected_orn, (0,0,-0.7071,0.7071))

            # jointPoses = p.calculateInverseKinematics(0, endEffectorId, pos, corrected_orn)#(0,0,0,1)) #orn)  #accurateCalculateInverseKinematics(0, endEffectorId, pos,
            #                                                 # threshold, maxIter, corrected_orn)

            jointPoses = accurateCalculateInverseKinematics(0, endEffectorId, pos,
                                                            threshold, maxIter)#, corrected_orn)
            # if (useSimulation):
            # for i in range(len(freeJoints)):
            #     p.setJointMotorControl2(bodyIndex=0,
            #                             jointIndex=freeJoints[i],
            #                             controlMode=p.POSITION_CONTROL,
            #                             targetPosition=jointPoses[i],
            #                             targetVelocity=0,
            #                             force=1,
            #                             positionGain=5,
            #                             velocityGain=0.1)

    markerVisualId[msg.header.frame_id] = p.addUserDebugText(msg.header.frame_id, pos, replaceItemUniqueId=markerVisualId[msg.header.frame_id])

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

def joint_targets_cb(msg):
    for i in range(len(msg.name)):
        id = idFromName(msg.name[i])
        if id is not None:
            p.setJointMotorControl2(bodyIndex=ob,
                                        jointIndex=id,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=msg.position[i],
                                        maxVelocity=0.3)



joint_target_sub = rospy.Subscriber(topic_root+"/simulation/joint_targets", JointState, joint_targets_cb, queue_size=1)


rate = rospy.Rate(600)
while not rospy.is_shutdown():
    p.stepSimulation()
    msg.header.stamp = rospy.Time.now()
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
    # if rospy.get_param('publish_cardsflow'):
    #     rospy.loginfo_throttle(1, msg.position)
    #     joint_target_pub.publish(msg)
    rate.sleep()
p.disconnect()
