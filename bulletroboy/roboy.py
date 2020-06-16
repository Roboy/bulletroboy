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

        # DEBUGGING VARIABLES
        self.prevPose = [0, 0, 0]
        self.prevPose1 = [0, 0, 0]
        #trailDuration is duration (in seconds) after debug lines will be removed automatically
        #use 0 for no-removal
        self.trailDuration = 5
        self.black_line = p.addUserDebugLine([0,0,0], [0,0,0] + np.array([0,0,0.1]), [0, 0, 0.3], 1, self.trailDuration)
        self.red_line = p.addUserDebugLine([0,0,0], [0,0,0] + np.array([0,0,0.1]), [0, 0, 0.3], 1, self.trailDuration)

        numJoints = p.getNumJoints(self.body_id)
        self.freeJoints = []
        self.endEffectors = {}

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
        self.collision_publisher = CollisionPublisher(body_id, self.create_publisher(Collision, '/roboy/simulation/collisions', 1))
        self.create_subscription(PoseStamped, '/roboy/simulation/operator/pose/endeffector', self.move, 10)
        
    def move(self, link_info):
        rclpy.logging._root_logger.info('Endeffector pose received')

        #process message
        ef_name = OPERATOR_TO_ROBOY_NAMES[link_info.header.frame_id]
        ef_id = self.endEffectors[ef_name]
        
        link_pos = [link_info.pose.position.x, link_info.pose.position.y, link_info.pose.position.z]
        link_orn= [link_info.pose.orientation.x, link_info.pose.orientation.y, link_info.pose.orientation.z, link_info.pose.orientation.w]
        link_pos, link_orn = self.adapt_pos_to_roboy(link_pos, link_orn)
        self.drawDebugLine(ef_name, link_pos)

        #move
        threshold = 0.001
        maxIter = 100
        self.accurateCalculateInverseKinematics(ef_id, link_pos, link_orn, threshold, maxIter)


    def accurateCalculateInverseKinematics(self, endEffectorId, targetPos, targetOrn, threshold, maxIter):
      closeEnough = False
      iter = 0
      dist2 = 1e30
      while (not closeEnough and iter < maxIter):
        jointPoses = p.calculateInverseKinematics(self.body_id, endEffectorId, targetPos, targetOrn)
        #import pdb; pdb.set_trace()
        # rclpy.logging._root_logger.info("resetting joints states")

        for i in range(len(self.freeJoints)):
          p.resetJointState(self.body_id, self.freeJoints[i], jointPoses[i])
        ls = p.getLinkState(self.body_id, endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
      return jointPoses

    def get_pos_in_world(self, pos, orn):
        torso_pos, torso_orn = p.getBasePositionAndOrientation(self.body_id)
        return p.multiplyTransforms(torso_pos, torso_orn, pos, orn)

    def adapt_pos_to_roboy(self, pos, orn):
        new_pos = np.array(pos) - [0,0,0.2]
        new_orn = orn
        # pos, orn = self.get_pos_in_world(pos, orn)

        # rotation = np.array([[0,1,0],[-1,0,0],[0,0,1]])
        # new_pos = rotation.dot(np.array(pos)) - [0,0.05,0.2]

        # _, new_orn = p.multiplyTransforms(new_pos, new_orn, [0,0,0],self.axis_angle_to_quaternion(0, 0, 1, 90))
        # new_orn = self.quaternion_multiply(orn, self.axis_angle_to_quaternion(0, 0, 1, -90))
        # new_orn = self.quaternion_multiply(new_orn, p.getQuaternionFromEuler([0, 0, 1.5708]))

        return new_pos, new_orn

    def axis_angle_to_quaternion(self, ax, ay, az, angle):
        qx = ax * math.sin(angle/2)
        qy = ay * math.sin(angle/2)
        qz = az * math.sin(angle/2)
        qw = math.cos(angle/2)
        return qx, qy, qz, qw
    
    def quaternion_multiply(self, quaternion0, quaternion1):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    # TODO REMOVE THIS METHOD
    def debug_orn(self, link_info):
        link_pos= [link_info.pose.position.x, link_info.pose.position.y, link_info.pose.position.z]
        link_orn= [link_info.pose.orientation.x, link_info.pose.orientation.y, link_info.pose.orientation.z, link_info.pose.orientation.w]
        p.resetBasePositionAndOrientation(self.original, link_pos, link_orn)

        targetPos, targetOrn = self.get_pos_in_world(link_pos, link_orn)
        p.resetBasePositionAndOrientation(self.target, targetPos, targetOrn)
           
    def drawDebugLine(self, ef, link_pos):
        # drawing debug lines

        # targetPos, targetOrn = self.adapt_pos_to_roboy(link_pos, link_orn)
        # link_rot_mat = p.getMatrixFromQuaternion(link_orn)
        # link_rot_mat = np.array(link_rot_mat).reshape((3,3))
        # target_rot_mat = p.getMatrixFromQuaternion(targetOrn)
        # target_rot_mat = np.array(target_rot_mat).reshape((3,3))

        # self.black_line = p.addUserDebugLine(targetPos, 
        #                             targetPos + target_rot_mat.dot(np.array([0,0,0.1])), 
        #                             [0, 0, 0.3], 2, 
        #                             self.trailDuration, 
        #                             replaceItemUniqueId=self.black_line)
        # self.red_line = p.addUserDebugLine(link_pos, 
        #                                 link_pos + link_rot_mat.dot(np.array([0,0,0.1])), 
        #                                 [1, 0, 0], 2, 
        #                                 self.trailDuration, 
        #                                 replaceItemUniqueId=self.red_line)
        if(ef == 'hand_left'):
            # p.addUserDebugLine(self.prevPose, targetPos, [0, 1, 0], 1, self.trailDuration)
            p.addUserDebugLine(self.prevPose1, link_pos, [0, 0, 1], 1, self.trailDuration)
            # self.prevPose = targetPos
            self.prevPose1 = link_pos

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
