import rclpy
from rclpy.executors import MultiThreadedExecutor
from ..state_mapper.state_mapper import StateMapper

def main(args=None):
    try:
        rclpy.init(args=args)

        forcesMapper = StateMapper()
        executor = MultiThreadedExecutor()
        rclpy.spin(forcesMapper, executor)

    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
