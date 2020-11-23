import sys, os, math
import pybullet as p
import pybullet_data
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ..exoforce.exoforce import CageConfiguration
from ..exoforce.exoforce_simulation import ExoForceSim
from ..operator.operator_simulation import OperatorSim
from ..utils.utils import parse_launch_arg

OPERATOR_DEFAULT_URFD = "humanoid/humanoid.urdf"
CAGE_CONF_DEFAULT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "../../config/cageConfiguration.xml"

def main():

	# PARSING ARGUMENTS
	operator_urdf = parse_launch_arg(sys.argv[1], OPERATOR_DEFAULT_URFD, rclpy.logging._root_logger.info)
	cage_conf_path = parse_launch_arg(sys.argv[2], CAGE_CONF_DEFAULT_PATH, rclpy.logging._root_logger.info)
	cage_mode = sys.argv[3]

	# SIMULATION SETUP
	p.connect(p.GUI)
	p.resetSimulation()
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.setGravity(0, 0, -9.81)
	p.setRealTimeSimulation(0)     # Don't change. Else p.applyExternalForce() won't work.

	# LOADING SIM BODIES
	p.loadURDF("plane.urdf")
	
	operator_model = p.loadURDF(operator_urdf, [0, 0, 1], p.getQuaternionFromEuler([math.pi/2, 0, 0]), globalScaling=0.284, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

	# EXOFORCE SETUP
	initial_cage_conf = CageConfiguration(cage_conf_path)

	rclpy.init()
	operator = OperatorSim(operator_model)
	exoforce = ExoForceSim(initial_cage_conf, operator, cage_mode)

	executor = MultiThreadedExecutor()
	executor.add_node(operator)
	executor.add_node(exoforce)
	spin_thread = Thread(target=executor.spin)

	spin_thread.start()
	# input("Start position control?")
	# exoforce.init_pos_control()

	# RUN SIM
	try:
		while True:
			operator.start_publishing()
			exoforce.move_operator_sim()
			exoforce.update()
			p.stepSimulation()

	except KeyboardInterrupt:
		operator.destroy_node()
		exoforce.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
