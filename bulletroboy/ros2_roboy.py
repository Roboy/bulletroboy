import argparse
import os

import pybullet as p
import math
import time

import rclpy

from threading import Thread
from bulletroboy.msg_publisher import BulletRoboyNode


def is_valid_file(parser, arg):
    file_path = os.path.dirname(os.path.realpath(__file__))
    file_path += "/" + arg
    if not os.path.exists(file_path):
        parser.error("The file %s does not exist!" % file_path)
    else:
        return file_path #return open(arg, 'r')  # return an open file handle

parser = argparse.ArgumentParser()
parser.add_argument("--model-path", dest="filename", default="../../roboy3_models/upper_body/bullet.urdf", metavar="FILE", help="path to the model URDF description", type=lambda x: is_valid_file(parser, x) )
args = parser.parse_args()



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
                self.endEffectorId = i
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
    p.loadURDF(is_valid_file(parser, "../models/cube.urdf"), [-0.5, -0.8, 0.5], useFixedBase=1)
    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(1)

    bb = BulletRoboy(body)

    rclpy.init()
    bullet_node = BulletRoboyNode(body)

    spin_thread = Thread(target=rclpy.spin, args=(bullet_node,))
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
                print("Collision at link ", point[3])
                bullet_node.send_collision(point)

            bb.drawDebugLines(pos)
        except KeyboardInterrupt:
            bullet_node.destroy_node()
            rclpy.shutdown()

    bullet_node.destroy_node()
    rclpy.shutdown()
    p.disconnect()

if __name__ == '__main__':
    main()
