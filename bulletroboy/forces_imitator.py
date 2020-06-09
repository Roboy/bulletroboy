import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision

class ForcesImitator(Node):
    def __init__(self):
        super().__init__('forces_imitator')

        self.result = None

        self.environmentSubscription = self.create_subscription(
            Collision,
            'not/yet/defined',
            self.listener_environnment_callback,
            10)

        self.robotCollisionSubscription = self.create_subscription(
            Collision,
            '/roboy/simulation/collision',
            self.listener_robot_callback,
            10)

        self.publisher = self.create_publisher(Collision, '/roboy/kinematic/forces_imitator', 10)
        timer_period = 0.1 # seconds
        self.time = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.result:
            return;
        self.publisher.publish(self.result)
        self.result = None


    # def listener_joints_callback(self, msg):
    #     self.jointState = msg
    #     self.update()
    #     self.get_logger().info('I heard: "%s"' % msg.data)

    def listener_environnment_callback(self, msg):
        self.updateToHumanCollision(msg)

    def listener_robot_callback(self, msg):
        self.updateToHumanCollision(msg)

    def robotToHumanLinkRatio(self, linkName):
        return 1
    
    def proportionalHumanForce(self, collision, ratio):
        return collision

    def updateToHumanCollision(self, collision):
        linkName = collision.linkname
        forceRatio = self.robotToHumanLinkRatio(linkName)
        self.result = self.proportionalHumanForce(collision, forceRatio)


def main(args=None):
    rclpy.init(args=args)

    forcesImitator = ForcesImitator()
    rclpy.spin(forcesImitator)
    forcesImitator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()