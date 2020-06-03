import argparse
import os

import pybullet as p
import math
import time

from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision

def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)
    else:
        return arg #return open(arg, 'r')  # return an open file handle

parser = argparse.ArgumentParser()
parser.add_argument("--model-path", dest="filename", required=True,  metavar="FILE", help="path to the model URDF description", type=lambda x: is_valid_file(parser, x) )
args = parser.parse_args()


class JointPublisher(Node):

    def __init__(self, bulletBodyId):
        super().__init__("bullet_joint_publisher")
        self.body_id = bulletBodyId
        self.publisher = self.create_publisher(JointState, '/roboy/simulation/joint_state', 1)
        timer_period = 0.1 # seconds
        self.time = self.create_timer(timer_period, self.timer_callback)

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

class CollisionPublisher(Node):
    def __init__(self, bulletBodyId):
        super().__init__("bullet_collision_publisher")
        self.body_id = bulletBodyId
        self.publisher = self.create_publisher(Collision, '/roboy/simulation/collision', 1)

    def send(self, collision):
        msg = Collision()
        msg.externalbody = collision[2]
        msg.linkroboy = collision[3]
        msg.linkexternalbody = collision[4]
        msg.positiononroboy.x = collision[5][0]
        msg.positiononroboy.y = collision[5][1]
        msg.positiononroboy.z = collision[5][2]
        msg.positiononexternalbody.x = collision[6][0]
        msg.positiononexternalbody.y = collision[6][1]
        msg.positiononexternalbody.z = collision[6][2]
        msg.contactnormalonexternalbody.x = collision[7][0]
        msg.contactnormalonexternalbody.y = collision[7][1]
        msg.contactnormalonexternalbody.z = collision[7][2]
        msg.contactdistance = collision[8]
        msg.normalforce = collision[9]
        msg.lateralfriction1 = collision[10]
        msg.lateralfrictiondir1.x = collision[11][0]
        msg.lateralfrictiondir1.y = collision[11][1]
        msg.lateralfrictiondir1.z = collision[11][2]
        msg.lateralfriction2 = collision[12]
        msg.lateralfrictiondir2.x = collision[13][0]
        msg.lateralfrictiondir2.y = collision[13][1]
        msg.lateralfrictiondir2.z = collision[13][2]
        self.publisher.publish(msg)

class BulletRoboy():
    def __init__(self, body_id):
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
                self.endEffectorId = i;
                print("EF id: " + str(i))

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

def main():
    p.connect(p.GUI)
    body = p.loadURDF(args.filename, useFixedBase=1)
    cube = p.loadURDF("../models/cube.urdf",[-0.5, -0.80282, 0.2], useFixedBase=1)  
    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(1)

    bb = BulletRoboy(body)

    rclpy.init()
    publisher_node = JointPublisher(body)
    collision_publisher_node = CollisionPublisher(body)
    spin_thread = Thread(target=rclpy.spin, args=(publisher_node,))
    spin_thread.start()

    while rclpy.ok():
        try:
            t = time.time()
            pos = [0.2 * math.cos(t)+0.2, -0.4, 0. + 0.2 * math.sin(t) + 0.7]
            threshold = 0.001
            maxIter = 100
            bb.accurateCalculateInverseKinematics(pos, threshold, maxIter)

            contactPts = p.getContactPoints(body)

            for  point in contactPts:
                print("Collision at ", point[2])
                collision_publisher_node.send(point)

            bb.drawDebugLines(pos)
        except KeyboardInterrupt:
            publisher_node.destroy_node()
            rclpy.shutdown()

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
