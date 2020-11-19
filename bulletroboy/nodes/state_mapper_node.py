import rclpy
from rclpy.executors import MultiThreadedExecutor
from ..state_mapper.state_mapper import StateMapper

def main(args=None):
    rclpy.init(args=args)

    forcesMapper = StateMapper()
    executor = MultiThreadedExecutor()
    rclpy.spin(forcesMapper, executor)

    forcesMapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
