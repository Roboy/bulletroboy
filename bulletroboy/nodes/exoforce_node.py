import os, sys
import rclpy
from ..exoforce.exoforce import CageConfiguration
from ..exoforce.exoforce_hardware import ExoforceHW
from ..utils.utils import parse_launch_arg

CAGE_CONF_DEFAULT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "../../config/realCageConfiguration.xml"

def main(args=None):
	try:
		rclpy.init(args=args)

		cage_conf_path = parse_launch_arg(sys.argv[1], CAGE_CONF_DEFAULT_PATH, rclpy.logging._root_logger.info)

		# EXOFORCE SETUP
		initial_cage_conf = CageConfiguration(cage_conf_path)
		exoforce = ExoforceHW(initial_cage_conf)
		rclpy.spin(exoforce)

	except KeyboardInterrupt:
		rclpy.shutdown()

if __name__ == "__main__":
	main()
