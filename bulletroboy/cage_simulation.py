import argparse
import os, math
import pybullet as p
import pybullet_data
import rclpy
from threading import Thread

from bulletroboy.operator import Operator, Moves
from bulletroboy.exoforce import CageConfiguration
from bulletroboy.exoforce_simulation import ExoForceSim

CONFIG_DEFAULT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "../config/cageConfiguration.xml"

def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)
    else:
        return arg #return open(arg, 'r')  # return an open file handle


def main():

    # PARSING ARGUMENTS
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", dest="mode", default="debug", help="execution mode: [debug]: uses pybullet debug forces [tendon]: uses tendon forces [forces]: uses link forces")
    parser.add_argument("--config-path", dest="config_path", default=CONFIG_DEFAULT_PATH, metavar="FILE", help="path to the cage configuration XML file", type=lambda x: is_valid_file(parser, x) )
    parser.add_argument("--model-path", dest="model_path", metavar="FILE", help="path to the human model URDF description", type=lambda x: is_valid_file(parser, x) )
    args = parser.parse_args()

    # SIMULATION SETUP
    p.connect(p.GUI)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)     # Don't change. Else p.applyExternalForce() won't work.

    # LOADING SIM BODIES
    p.loadURDF("plane.urdf")
    
    human_urdf = args.model_path if args.model_path else "humanoid/humanoid.urdf"
    human_model = p.loadURDF(human_urdf, [0, 0, 1], p.getQuaternionFromEuler([math.pi/2, 0, 0]), globalScaling=0.284, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

    # EXOFORCE SETUP
    initial_cage_conf = CageConfiguration(args.config_path)

    rclpy.init()
    node = rclpy.create_node('exoforce_operator')

    operator = Operator(human_model,node=node)
    exoforce = ExoForceSim(initial_cage_conf, operator, args.mode, node=node)

    spin_thread = Thread(target=rclpy.spin, args=(node,))

    spin_thread.start()

    # RUN SIM
    try:
        while True:
            exoforce.operator.publish_state()
            exoforce.operator.move(Moves.ARM_ROLL)
            exoforce.update()
            p.stepSimulation()

    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

    # exoforce.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
