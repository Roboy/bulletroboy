import pybullet as p
import time

import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision
from roboy_simulation_msgs.srv import LinkNameFromId

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
        
        for i in range(numJoints):
            info = p.getJointInfo(self.body_id,i)
            if info[2] == p.JOINT_REVOLUTE:
                self.freeJoints.append(i)
            if info[12] == b'hand_left':
                self.endEffectorId = i
                self.get_logger().info("EF id: " + str(i))

        timer_period = 0.1 # seconds

        #Joint state publisher
        self.joint_names = []
        for i in range(p.getNumJoints(self.body_id)):
            ji = p.getJointInfo(self.body_id,i)
            self.joint_names.append(ji[1].decode("utf-8"))
        self.joint_publisher = self.create_publisher(JointState, '/roboy/simulation/joint_state', 1)
        self.timer = self.create_timer(timer_period, self.joint_state_timer_callback)
        
        #Collision publisher
        self.collision_publisher = self.create_publisher(Collision, '/roboy/simulation/collision', 1)

        #Services

        #LinkNameFromId service
        self.link_name_service = self.create_service(LinkNameFromId, '/roboy/simulation/roboy/link_name_from_link_id', self.get_link_name_from_id)
    
    def get_links(self):
        links = []
        for i in range(p.getNumJoints(self.body_id)):
            link = {}
            link['name'] = str(p.getJointInfo(self.body_id,i)[12], 'utf-8')
            link['id'] = i
            links.append(link)
        return links

    def get_link_name_from_id(self, request, response):
        self.get_logger().info("Service LinkNameFromLinkId: request received")
        response.link_name = ""
        for link in self.get_links():
            if link['id'] == request.link_id:
                response.link_name = link['name']
                break
        return response

    def accurateCalculateInverseKinematics(self, targetPos, threshold, maxIter):
      closeEnough = False
      iter = 0
      dist2 = 1e30
      while (not closeEnough and iter < maxIter):
        jointPoses = p.calculateInverseKinematics(self.body_id, self.endEffectorId, targetPos)
        #import pdb; pdb.set_trace()
        # self.get_logger().info("resetting joints states")

        for i in range(len(self.freeJoints)):
          p.resetJointState(self.body_id, self.freeJoints[i], jointPoses[i])
        ls = p.getLinkState(self.body_id, self.endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
      #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
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
        
    def joint_state_timer_callback(self):
        """Callback function for the timer, publishes joint message every time it gets triggered by timer. 
        """
        msg = JointState()
        msg.effort = [0.0]*len(self.joint_names)
        msg.velocity = [0.0]*len(self.joint_names)
        for i in range(p.getNumJoints(self.body_id)):
            js = p.getJointState(self.body_id, i)
            msg.position.append(js[0])
            msg.name.append(self.joint_names[i])
        self.joint_publisher.publish(msg)

    def publish_collision(self, collision):
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

        self.get_logger().info("Publishing collision in link %i" % msg.linkid)

        self.collision_publisher.publish(msg)
    
    def get_pos_in_link_frame(self, link, position):
        """Changes the position of the collision from world's coordinates system to link frame.

        Args:
            link: link id in which a collision happened.
            position: vector that would be changed.

        Returns:
            The position in link frame.
        """

        frame_pos = [0,0,0]
        frame_orn = [0,0,0,0]

        if(link == -1):
            frame_pos, frame_orn = (p.getBasePositionAndOrientation(self.body_id))[:2]
        else:
            #[0] == linkWorldPosition in PyBullet docu
            #[1] == linkWorldOrientation in PyBullet docu
            frame_pos, frame_orn = (p.getLinkState(self.body_id, link))[:2]

        _, inv_frame_orn = p.invertTransform(frame_pos, frame_orn)

        rotation = np.array(p.getMatrixFromQuaternion(inv_frame_orn))
        rotation = rotation.reshape(3,3)
        translation = np.array(frame_pos)

        return rotation.dot(position) + translation
