import rclpy
from ..operator.operator_hardware import OperatorHW

def main(args=None):
    try:
        rclpy.init(args=args)

        operator = OperatorHW()
        rclpy.spin(operator)

    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
	main()
