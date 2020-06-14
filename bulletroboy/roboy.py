import pybullet as p
import time

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision
from geometry_msgs.msg import PoseStamped

class BulletRoboy(Node):
    def __init__(self, body_id):
        super().__init__("bullet_roboy")
        self.body_id = body_id
        self.t = 0.
        self.prevPose = [0, 0, 0]
        self.prevPose1 = [0, 0, 0]
        self.hasPrevPose = 0
        self.ikSolver = 0
        #trailDuration is duration (in seconds) after debug lines will be removed automatically
        #use 0 for no-removal
        self.trailDuration = 15
        numJoints = p.getNumJoints(self.body_id)
        self.freeJoints = []

        for i in range(numJoints):
            info = p.getJointInfo(self.body_id,i)
            if info[2] == p.JOINT_REVOLUTE:
                self.freeJoints.append(i)
            if info[12] == b'hand_left':
                self.endEffectorId = i
                print("EF id: " + str(i))

        timer_period = 0.1 # seconds

        #Publishers and subscribers
        self.joint_publisher = JointPublisher(body_id, self.create_publisher(JointState, '/roboy/simulation/joint_state', 1))
        self.timer = self.create_timer(timer_period, self.joint_publisher.timer_callback)
        self.collision_publisher = CollisionPublisher(body_id, self.create_publisher(Collision, '/roboy/simulation/collision', 1))
        self.create_subscription(PoseStamped, '/roboy/simulation/operator/pose/endeffector', self.move,10)

    def move(self, link_info):
        link_pos= [link_info.pose.position.x, link_info.pose.position.y, link_info.pose.position.z]
        link_orn= [link_info.pose.orientation.x, link_info.pose.orientation.y, link_info.pose.orientation.z, link_info.pose.orientation.w]
        


    def accurateCalculateInverseKinematics(self, targetPos, threshold, maxIter):
      closeEnough = False
      iter = 0
      dist2 = 1e30
      while (not closeEnough and iter < maxIter):
        jointPoses = p.calculateInverseKinematics(self.body_id, self.endEffectorId, targetPos)
        #import pdb; pdb.set_trace()
        # rclpy.logging._root_logger.info("resetting joints states")

        for i in range(len(self.freeJoints)):
          p.resetJointState(self.body_id, self.freeJoints[i], jointPoses[i])
        ls = p.getLinkState(self.body_id, self.endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
      return jointPoses

    def drawDebugLines(self, targetPos):
        # drawing debug lines
        ls = p.getLinkState(self.body_id, self.endEffectorId)
        if (self.hasPrevPose):
            p.addUserDebugLine(self.prevPose, targetPos, [0, 0, 0.3], 1, self.trailDuration)
            p.addUserDebugLine(self.prevPose1, ls[4], [1, 0, 0], 1, self.trailDuration)
        self.prevPose = targetPos
        self.prevPose1 = ls[4]
        self.hasPrevPose = 1
        
class JointPublisher():

    def __init__(self, bulletBodyId, publisher):
        self.body_id = bulletBodyId
        self.publisher = publisher
        self.joint_names = []
        for i in range(p.getNumJoints(self.body_id)):
            ji = p.getJointInfo(self.body_id,i)
            self.joint_names.append(ji[1].decode("utf-8"))

    def timer_callback(self):
        msg = JointState()
        msg.effort = [0.0]*len(self.joint_names)
        msg.velocity = [0.0]*len(self.joint_names)
        for i in range(p.getNumJoints(self.body_id)):
            js = p.getJointState(self.body_id, i)
            msg.position.append(js[0])
            msg.name.append(self.joint_names[i])
        self.publisher.publish(msg)

class CollisionPublisher():
    def __init__(self, bulletBodyId, publisher):
        self.body_id = bulletBodyId
        self.publisher = publisher

    def send(self, collision):
        msg = Collision()
        
        msg.linkid = collision[3]

        pos_in_lf = self.get_pos_in_link_frame(collision[3], collision[5])
        msg.position.x = pos_in_lf[0]
        msg.position.y = pos_in_lf[1]
        msg.position.z = pos_in_lf[2]

        msg.contactnormal.x = collision[7][0]
        msg.contactnormal.y = collision[7][1]
        msg.contactnormal.z = collision[7][2]

        msg.contactdistance = collision[8]

        msg.normalforce = collision[9]

        rclpy.logging._root_logger.info("Publishing collision in link %i" % msg.linkid)

        self.publisher.publish(msg)
    
    def get_pos_in_link_frame(self, link, position):
        frame_pos = (p.getLinkState(self.body_id, link))[4]
        frame_orn = (p.getLinkState(self.body_id, link))[5]

        _, inv_frame_orn = p.invertTransform(frame_pos, frame_orn)

        rotation = np.array(p.getMatrixFromQuaternion(inv_frame_orn))
        rotation = rotation.reshape(3,3)
        translation = np.array(frame_pos)

        return rotation.dot(position) + translation

