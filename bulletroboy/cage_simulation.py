import argparse
import os
import pybullet as p
import pybullet_data
import rclpy
from threading import Thread

from bulletroboy.operator import Operator
from bulletroboy.exoforce import CageConfiguration
from bulletroboy.exoforce_simulation import ExoForceSim
from bulletroboy.constants import FOREARM_ROLL

def is_valid_file(parser, arg):
    file_path = os.path.dirname(os.path.realpath(__file__))
    file_path += "/" + arg
    if not os.path.exists(file_path):
        parser.error("The file %s does not exist!" % file_path)
    else:
        return file_path #return open(arg, 'r')  # return an open file handle


def main():

    # PARSING ARGUMENTS
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", dest="mode", default="debug", help="execution mode: [debug]: uses pybullet debug forces [tendon]: uses tendon forces [forces]: uses link forces")
    parser.add_argument("--config-path", dest="config_path", default="../config/cageConfiguration.xml", metavar="FILE", help="path to the cage configuration XML file", type=lambda x: is_valid_file(parser, x) )
    parser.add_argument("--model-path", dest="model_path", default="../models/human.urdf", metavar="FILE", help="path to the human model URDF description", type=lambda x: is_valid_file(parser, x) )
    args = parser.parse_args()

    # SIMULATION SETUP
    p.connect(p.GUI)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)     # Don't change. Else p.applyExternalForce() won't work.

    # LOADING SIM BODIES
    p.loadURDF("plane.urdf")
    human_model = p.loadURDF(args.model_path, [0, 0, 1], useFixedBase=1)

    # EXOFORCE SETUP
    initial_cage_conf = CageConfiguration(args.config_path)

    rclpy.init()
    exoforce = ExoForceSim(initial_cage_conf, human_model, args.mode)

    spin_thread = Thread(target=rclpy.spin, args=(exoforce,))
    spin_thread.start()

    # RUN SIM
    try:
        while True:
            exoforce.operator.publish_state()
            exoforce.operator.move(FOREARM_ROLL)
            exoforce.update()
            p.stepSimulation()

    except KeyboardInterrupt:
        exoforce.destroy_node()
        rclpy.shutdown()

    exoforce.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
