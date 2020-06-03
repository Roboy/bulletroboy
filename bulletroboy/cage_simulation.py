import argparse
import os
import pybullet as p
import pybullet_data

import rclpy
from threading import Thread

from bulletroboy.exoforce import CageConfiguration, Operator, ExoForce
from roboy_simulation_msgs.msg import TendonUpdate
from std_msgs.msg import Float32

def is_valid_file(parser, arg):
    file_path = os.path.dirname(os.path.realpath(__file__))
    file_path += arg
    if not os.path.exists(file_path):
        parser.error("The file %s does not exist!" % file_path)
    else:
        return file_path #return open(arg, 'r')  # return an open file handle

def main():

    # PARSING ARGUMENTS
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", dest="mode", default="debug", help="execution mode: [debug]: uses pybullet debug forces [tendon]: uses tendon forces [forces]: uses link forces")
    parser.add_argument("--config-path", dest="config_path", default="/../config/cageConfiguration.xml", metavar="FILE", help="path to the cage configuration XML file", type=lambda x: is_valid_file(parser, x) )
    parser.add_argument("--model-path", dest="model_path", default="/../models/human.urdf", metavar="FILE", help="path to the human model URDF description", type=lambda x: is_valid_file(parser, x) )
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
    operator = Operator(human_model)

    exoforce = ExoForce(initial_cage_conf, operator)

    # ROS2 LISTENERS DEFINITION
    def tendon_update_listener(tendon_force):
        exoforce.tendon_update(tendon_force.tendon_id, tendon_force.force)

    def cage_rotation_listener(angle):
        exoforce.rotate_cage(angle.data)

    def forces_update_listener(forces):
        # TODO: implement force update
        pass

    # SETIING UP DATA INPUT ACCORDING TO MODE
    if args.mode == "debug":
        for tendon in exoforce.get_tendons():
            tendon.forceId = p.addUserDebugParameter("Force in " + tendon.name, 0, 200, 0)

        cage_angle_id = p.addUserDebugParameter("Cage Angle", -180, 180, 0)
    else:
        rclpy.init()
        node = rclpy.create_node("cage_simulation")
        if args.mode == "tendon":
            node.create_subscription(TendonUpdate, '/roboy/simulation/tendon_force', tendon_update_listener, 10)
        elif args.mode == "forces":
            # TODO: update subscriber with correct msg type
            node.create_subscription(TendonUpdate, '/roboy/simulation/operator_forces', forces_update_listener, 10)

        node.create_subscription(Float32, 'roboy/simulation/cage_rotation', cage_rotation_listener, 10)
        
        spin_thread = Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()

    # RUN SIM
    try:
        while True:

            for tendon in exoforce.get_tendons():
                tendon.update_lines()

            if args.mode == "debug":
                cage_angle = p.readUserDebugParameter(cage_angle_id)
                exoforce.rotate_cage(cage_angle)

                for tendon in exoforce.get_tendons():
                    force = p.readUserDebugParameter(tendon.forceId)
                    if force > 0: tendon.apply_force(force)

            p.stepSimulation()

    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
