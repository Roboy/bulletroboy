import rclpy
from rclpy.executors import MultiThreadedExecutor
from ..operator.operator_hardware import OperatorHW

def main(args=None):
    rclpy.init(args=args)

    operator = OperatorHW()
    rclpy.spin(operator, MultiThreadedExecutor())

    operator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
	main()