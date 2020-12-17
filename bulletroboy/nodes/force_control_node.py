import rclpy
from ..force_control.force_control import ForceControl

def main(args=None):
    rclpy.init(args=args)

    force_control = ForceControl()
    rclpy.spin(force_control)

    force_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
	main()
