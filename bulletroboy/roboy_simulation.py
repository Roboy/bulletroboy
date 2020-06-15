import argparse
import os

import pybullet as p
import math
import time

import rclpy

from threading import Thread
from bulletroboy.roboy import BulletRoboy
from bulletroboy.environment_control import EnvironmentCtrl

def is_valid_file(parser, arg):
    file_path = os.path.dirname(os.path.realpath(__file__))
    file_path += "/" + arg
    if not os.path.exists(file_path):
        rclpy.logging._root_logger.error("The file %s does not exist!" % file_path)
    else:
        return file_path #return open(arg, 'r')  # return an open file handle

parser = argparse.ArgumentParser()
parser.add_argument("--model-path", dest="filename", default="../../roboy3_models/upper_body/bullet.urdf", metavar="FILE", help="path to the model URDF description", type=lambda x: is_valid_file(parser, x) )
args = parser.parse_args()


def main():
    """Sets up pybullet environment and runs simulation.
    """
    p.connect(p.GUI)

    body = p.loadURDF(args.filename, [0, 0, 0.2], useFixedBase=1)
    env = EnvironmentCtrl()

    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(1)

    rclpy.init()

    rclpy.logging._root_logger.info("Starting Bullet Roboy node ")

    bb = BulletRoboy(body)

    spin_thread = Thread(target=rclpy.spin, args=(bb,))
    spin_thread.start()

    while rclpy.ok():
        try:
            #update the environement parameters with each step
            env.update()
            t = time.time()
            pos = [0.2 * math.cos(t)+0.2, -0.4, 0. + 0.2 * math.sin(t) + 0.7]
            threshold = 0.001
            maxIter = 100
            # rclpy.logging._root_logger.info("Moving roboy")
            # bb.accurateCalculateInverseKinematics(pos, threshold, maxIter)

            contactPts = p.getContactPoints(body)

            for  point in contactPts:
                rclpy.logging._root_logger.info("Collision at link %i" % point[3])
                bb.collision_publisher.publish(point)

            # bb.drawDebugLines(pos)
        except KeyboardInterrupt:
            env.stop()
            bb.destroy_node()
            rclpy.shutdown()
            p.disconnect()

    env.stop()
    bb.destroy_node()
    rclpy.shutdown()
    p.disconnect()

if __name__ == '__main__':
    main()
