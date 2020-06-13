import pybullet as p
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision

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
        self.joint_publisher = JointPublisher(body_id, self.create_publisher(JointState, '/roboy/simulation/joint_state', 1))
        self.timer = self.create_timer(timer_period, self.joint_publisher.timer_callback)
        self.collision_publisher = CollisionPublisher(body_id, self.create_publisher(Collision, '/roboy/simulation/collision', 1))



    def accurateCalculateInverseKinematics(self, targetPos, threshold, maxIter):
      closeEnough = False
      iter = 0
      dist2 = 1e30
      while (not closeEnough and iter < maxIter):
        jointPoses = p.calculateInverseKinematics(self.body_id, self.endEffectorId, targetPos)
        #import pdb; pdb.set_trace()
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
        # msg.externalbody = collision[2]
        # msg.linkroboy = collision[3]
        # msg.linkexternalbody = collision[4]
        # msg.positiononroboy.x = collision[5][0]
        # msg.positiononroboy.y = collision[5][1]
        # msg.positiononroboy.z = collision[5][2]
        # msg.positiononexternalbody.x = collision[6][0]
        # msg.positiononexternalbody.y = collision[6][1]
        # msg.positiononexternalbody.z = collision[6][2]
        # msg.contactnormalonexternalbody.x = collision[7][0]
        # msg.contactnormalonexternalbody.y = collision[7][1]
        # msg.contactnormalonexternalbody.z = collision[7][2]
        # msg.contactdistance = collision[8]
        # msg.normalforce = collision[9]
        # msg.lateralfriction1 = collision[10]
        # msg.lateralfrictiondir1.x = collision[11][0]
        # msg.lateralfrictiondir1.y = collision[11][1]
        # msg.lateralfrictiondir1.z = collision[11][2]
        # msg.lateralfriction2 = collision[12]
        # msg.lateralfrictiondir2.x = collision[13][0]
        # msg.lateralfrictiondir2.y = collision[13][1]
        # msg.lateralfrictiondir2.z = collision[13][2]
        # self.publisher.publish(msg)
    