import os
import rclpy
from ..exoforce.exoforce import CageConfiguration
from ..exoforce.exoforce_hardware import ExoforceHW

CONFIG_DEFAULT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "../../config/realCageConfiguration.xml"

def main(args=None):
	rclpy.init(args=args)

	# EXOFORCE SETUP
	initial_cage_conf = CageConfiguration(CONFIG_DEFAULT_PATH)
	exoforce = ExoforceHW(initial_cage_conf)
	rclpy.spin(exoforce)

	exoforce.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
