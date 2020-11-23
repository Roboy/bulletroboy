import sys, os
import pybullet as p

import rclpy

from threading import Thread
from ..roboy.roboy import BulletRoboy
from ..roboy.environment_control import EnvironmentCtrl
from ..utils.utils import parse_launch_arg

MODEL_DEFAULT_PATH =  os.path.dirname(os.path.realpath(__file__)) + "/" + "../../../roboy3_models/upper_body/bullet.urdf"

def main():
    """Sets up pybullet environment and runs simulation.
    """
    # PARSING ARGUMENTS
    model_path = parse_launch_arg(sys.argv[1], MODEL_DEFAULT_PATH, rclpy.logging._root_logger.info)

    # SETTING UP SIMULATION
    p.connect(p.GUI)
    #flags= p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
    body = p.loadURDF(model_path, useFixedBase=1)
    env = EnvironmentCtrl()

    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(0)
    
    #INITIALISING ROS AND NODE
    rclpy.init()
    
    bb = BulletRoboy(body)
    spin_thread = Thread(target=rclpy.spin, args=(bb,))
    spin_thread.start()

    while rclpy.ok():
        try:
            if bb.ready:
                #update the environement parameters with each step
                env.update()
                p.stepSimulation()
                contactPts = p.getContactPoints(body)

                for point in contactPts:
                    bb.publish_collision(point)
                    bb.publish_collision_to_decomposer(point)

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
