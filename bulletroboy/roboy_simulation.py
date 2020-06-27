import argparse
import os

import pybullet as p
import math

import rclpy

from threading import Thread
from bulletroboy.roboy import BulletRoboy
from bulletroboy.environment_control import EnvironmentCtrl

MODEL_DEFAULT_PATH =  os.path.dirname(os.path.realpath(__file__)) + "/" + "../../roboy3_models/upper_body/bullet.urdf"

def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        rclpy.logging._root_logger.error("The file %s does not exist!" % arg)
    else:
        return arg #return open(arg, 'r')  # return an open file handle


def main():
    """Sets up pybullet environment and runs simulation.
    """
    # PARSING ARGUMENTS
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-path", dest="filename", default=MODEL_DEFAULT_PATH, metavar="FILE", help="path to the model URDF description", type=lambda x: is_valid_file(parser, x))
    args = parser.parse_args()
    
    # SETTING UP WORLD
    p.connect(p.GUI)
    #flags= p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
    body = p.loadURDF(args.filename, [0, 0, 0.8], p.getQuaternionFromEuler([0, 0, 1.5708]), useFixedBase=1)#, flags=flags)
    env = EnvironmentCtrl()

    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(0)

    #INITIALISING ROS AND NODE
    rclpy.init()
    rclpy.logging._root_logger.info("Starting Bullet Roboy node ")

    bb = BulletRoboy(body)

    spin_thread = Thread(target=rclpy.spin, args=(bb,))
    spin_thread.start()

    while rclpy.ok():
        try:
            #update the environement parameters with each step
            env.update()
            p.stepSimulation()
            contactPts = p.getContactPoints(body)

            for  point in contactPts:
                bb.publish_collision(point)

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
