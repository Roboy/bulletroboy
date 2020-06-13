import argparse
import os

import pybullet as p
import math
import time

import rclpy

from threading import Thread
from bulletroboy.roboy import BulletRoboy

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


def main():
    p.connect(p.GUI)

    body = p.loadURDF(args.filename, useFixedBase=1)
    p.loadURDF(is_valid_file(parser, "../models/cube.urdf"), [-0.5, -0.8, 0.5], useFixedBase=1)
    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(1)

    rclpy.init()

    bb = BulletRoboy(body)

    spin_thread = Thread(target=rclpy.spin, args=(bb,))
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
                bb.collision_publisher.send(point)

            bb.drawDebugLines(pos)
        except KeyboardInterrupt:
            bb.destroy_node()
            rclpy.shutdown()

    bb.destroy_node()
    rclpy.shutdown()
    p.disconnect()

if __name__ == '__main__':
    main()
