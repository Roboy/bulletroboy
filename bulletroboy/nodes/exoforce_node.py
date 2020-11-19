import os
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ..exoforce.exoforce import CageConfiguration
from ..exoforce.exoforce_hardware import ExoforceHW

CONFIG_DEFAULT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "../../config/realCageConfiguration.xml"

def main(args=None):
	rclpy.init(args=args)

	# EXOFORCE SETUP
	initial_cage_conf = CageConfiguration(CONFIG_DEFAULT_PATH)
	exoforce = ExoforceHW(initial_cage_conf)
	executor = MultiThreadedExecutor()
	rclpy.spin(exoforce, executor)

	exoforce.destroy_node()
	executor.shutdown()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
