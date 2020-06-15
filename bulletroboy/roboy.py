import pybullet as p
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision
from geometry_msgs.msg import PoseStamped
from bulletroboy.link_mapping import OPERATOR_TO_ROBOY_NAMES

class BulletRoboy(Node):
    """
    This class represents the Roboy in simulation. It handles movements of the roboy as well as publishing and receiving ros messages.
    """
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
        self.endEffectors = {}
        print(p.getBasePositionAndOrientation(body_id))

        for i in range(numJoints):
            info = p.getJointInfo(self.body_id,i)
            if info[2] == p.JOINT_REVOLUTE:
                self.freeJoints.append(i)
            if info[12] == b'hand_left':
                self.endEffectors['hand_left'] = i
                rclpy.logging._root_logger.info("EF hand_left id: " + str(i))
            if info[12] == b'hand_right':
                self.endEffectors['hand_right'] = i
                rclpy.logging._root_logger.info("EF hand_right id: " + str(i))

        #Publishers and subscribers
        timer_period = 0.1 # seconds

        self.joint_publisher = JointPublisher(body_id, self.create_publisher(JointState, '/roboy/simulation/joint_state', 1))
        self.timer = self.create_timer(timer_period, self.joint_publisher.timer_callback)
        self.collision_publisher = CollisionPublisher(body_id, self.create_publisher(Collision, '/roboy/simulation/collision', 1))
        self.create_subscription(PoseStamped, '/roboy/simulation/operator/pose/endeffector', self.move,10)
        self.create_subscription(PoseStamped, '/roboy/simulation/operator/pose/endeffector', self.drawDebugLines,10)
    
    def move(self, link_info):
        print('RECEIVED')
        #process message
        ef_id = self.endEffectors[OPERATOR_TO_ROBOY_NAMES[link_info.header.frame_id]]
        link_pos= [link_info.pose.position.x, link_info.pose.position.y, link_info.pose.position.z - 0.1]
        link_orn= [link_info.pose.orientation.x, link_info.pose.orientation.y, link_info.pose.orientation.z, link_info.pose.orientation.w]
        link_pos, link_orn = self.get_pos_in_world(link_pos, link_orn)
        print(link_info.header.frame_id)
        print(link_pos, link_orn)
        print(p.getLinkState(self.body_id, ef_id)[:2])
        
        #move
        maxIter = 100
        iter = 0
        while(iter <= maxIter):
            jointPoses = p.calculateInverseKinematics(self.body_id, ef_id, link_pos, link_orn)
            for i in range(len(self.freeJoints)):
                p.resetJointState(self.body_id, self.freeJoints[i], jointPoses[i])
            iter = iter + 1
        print(p.getLinkState(self.body_id, ef_id)[:2])

    # def accurateCalculateInverseKinematics(self, targetPos, threshold, maxIter):
    #   closeEnough = False
    #   iter = 0
    #   dist2 = 1e30
    #   while (not closeEnough and iter < maxIter):
    #     jointPoses = p.calculateInverseKinematics(self.body_id, self.endEffectorId, targetPos)
    #     #import pdb; pdb.set_trace()
    #     # rclpy.logging._root_logger.info("resetting joints states")

    #     for i in range(len(self.freeJoints)):
    #       p.resetJointState(self.body_id, self.freeJoints[i], jointPoses[i])
    #     ls = p.getLinkState(self.body_id, self.endEffectorId)
    #     newPos = ls[4]
    #     diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
    #     dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
    #     closeEnough = (dist2 < threshold)
    #     iter = iter + 1
    #   return jointPoses

    def get_pos_in_world(self, pos, orn):
        # torso_pos, torso_orn = p.getBasePositionAndOrientation(self.body_id)
        # new_pos, new_orn = p.multiplyTransforms(torso_pos, torso_orn, pos, orn)

        # new_pos, new_orn = p.multiplyTransforms(new_pos, new_orn, [0,0,0], p.getQuaternionFromEuler([0, 0, -1.5708]))
        new_pos, new_orn = pos, orn
        rotation = np.array([[0,1,0],[-1,0,0],[0,0,1]])
        new_pos = rotation.dot(np.array(new_pos))
        
        _, new_orn = p.multiplyTransforms(new_pos, new_orn, [0,0,0],self.axis_angle_to_quaternion(0, 0, 1, 90))

        return new_pos, new_orn

    def axis_angle_to_quaternion(self, ax, ay, az, angle):
        qx = ax * math.sin(angle/2)
        qy = ay * math.sin(angle/2)
        qz = az * math.sin(angle/2)
        qw = math.cos(angle/2)
        return qx, qy, qz, qw

    # def drawDebugLines(self, link_info):
    #     # drawing debug lines
    #     link_pos= [link_info.pose.position.x, link_info.pose.position.y, link_info.pose.position.z]
    #     link_orn= [link_info.pose.orientation.x, link_info.pose.orientation.y, link_info.pose.orientation.z, link_info.pose.orientation.w]
        
    #     targetPos, _ = self.get_pos_in_world(link_pos, link_orn)
    #     for ef in self.endEffectors:
    #         ls = p.getLinkState(self.body_id, self.endEffectors[ef])
    #         if (self.hasPrevPose):
    #             p.addUserDebugLine(self.prevPose, targetPos, [0, 0, 0.3], 1, self.trailDuration)
    #             p.addUserDebugLine(self.prevPose1, ls[4], [1, 0, 0], 1, self.trailDuration)
    #         print(targetPos)
    #         self.prevPose = targetPos
    #         self.prevPose1 = ls[4]
    #         self.hasPrevPose = 1
        
    def drawDebugLines(self, link_info):
        # drawing debug lines
        link_pos= [link_info.pose.position.x, link_info.pose.position.y, link_info.pose.position.z]
        link_orn= [link_info.pose.orientation.x, link_info.pose.orientation.y, link_info.pose.orientation.z, link_info.pose.orientation.w]
        targetPos, _ = self.get_pos_in_world(link_pos, link_orn)
        for ef in self.endEffectors:
            ls = p.getLinkState(self.body_id, self.endEffectors[ef])
            if (self.hasPrevPose):
                p.addUserDebugLine(self.prevPose, targetPos, [0, 0, 0.3], 1, self.trailDuration)
                p.addUserDebugLine(self.prevPose1, link_pos, [1, 0, 0], 1, self.trailDuration)
            #print(targetPos)
            self.prevPose = targetPos
            self.prevPose1 = link_pos
            self.hasPrevPose = 1
            
class JointPublisher():
    """
    This class handles publishing joint state messages.
    """
    def __init__(self, bulletBodyId, publisher):
        self.body_id = bulletBodyId
        self.publisher = publisher
        self.joint_names = []
        for i in range(p.getNumJoints(self.body_id)):
            ji = p.getJointInfo(self.body_id,i)
            self.joint_names.append(ji[1].decode("utf-8"))

    def timer_callback(self):
        """Callback function for the timer, publishes joint message every time it gets triggered by timer. 
        """
        msg = JointState()
        msg.effort = [0.0]*len(self.joint_names)
        msg.velocity = [0.0]*len(self.joint_names)
        for i in range(p.getNumJoints(self.body_id)):
            js = p.getJointState(self.body_id, i)
            msg.position.append(js[0])
            msg.name.append(self.joint_names[i])
        self.publisher.publish(msg)

class CollisionPublisher():
    """
    This class handles publishing collision messages.
    """
    def __init__(self, bulletBodyId, publisher):
        self.body_id = bulletBodyId
        self.publisher = publisher

    def publish(self, collision):
        """Processes the collision message and publishes it.

        Args:
            collision: item of the contact points list output of pybullet getcontactPoints.
        """
        msg = Collision()

        #collision[3] == linkIndexA in PyBullet docu
        msg.linkid = collision[3]

        #collision[5] == positionOnA in PyBullet docu
        pos_in_lf = self.get_pos_in_link_frame(collision[3], collision[5])
        msg.position.x = pos_in_lf[0]
        msg.position.y = pos_in_lf[1]
        msg.position.z = pos_in_lf[2]

        #collision[7] == contactNormalOnB in PyBullet docu
        msg.contactnormal.x = collision[7][0]
        msg.contactnormal.y = collision[7][1]
        msg.contactnormal.z = collision[7][2]

        #collision[8] == contactDistance in PyBullet docu
        msg.contactdistance = collision[8]

        #collision[9] == normalForce in PyBullet docu
        msg.normalforce = collision[9]

        rclpy.logging._root_logger.info("Publishing collision in link %i" % msg.linkid)

        self.publisher.publish(msg)
    
    def get_pos_in_link_frame(self, link, position):
        """Changes the position of the collision from world's coordinates system to link frame.

        Args:
            link: link id in which a collision happened.
            position: vector that would be changed.

        Returns:
            The position in link frame.
        """

        #[0] == linkWorldPosition in PyBullet docu
        #[1] == linkWorldOrientation in PyBullet docu
        frame_pos = (p.getLinkState(self.body_id, link))[0]
        frame_orn = (p.getLinkState(self.body_id, link))[1]

        _, inv_frame_orn = p.invertTransform(frame_pos, frame_orn)

        rotation = np.array(p.getMatrixFromQuaternion(inv_frame_orn))
        rotation = rotation.reshape(3,3)
        translation = np.array(frame_pos)

        return rotation.dot(position) + translation
