import rclpy
from ..state_mapper.state_mapper import StateMapper

def main(args=None):
    try:
        rclpy.init(args=args)

        forcesMapper = StateMapper()
        rclpy.spin(forcesMapper)

    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
