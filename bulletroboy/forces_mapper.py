import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision
import yaml

class ForcesMapper(Node):
    def __init__(self):
        super().__init__('forces_imitator')

        self.result = None

        self.robotCollisionSubscription = self.create_subscription(
            Collision,
            '/roboy/simulation/collision',
            self.collision_listener,
            10)

        self.exoforceCollisionPublisher = self.create_publisher(Collision, 'roboy/exoforce/collisions', 10)
        timer_period = 0.005 # seconds
        self.time = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Publishes collision to exoforce if any."""

        if not self.result:
            return;
        self.exoforceCollisionPublisher.publish(self.result)
        self.result = None


    # def listener_joints_callback(self, msg):
    #     self.jointState = msg
    #     self.update()
    #     self.get_logger().info('I heard: "%s"' % msg.data)

    def collision_listener(self, msg):
        """Collision subscriber handler."""

        self.transformFromRobotToOperator(msg)

    def robotToHumanLinkRatio(self, linkId):
        """Calculates the ratio between robot link and human link.

        Parameters:
            linkId (str):The id of the link whose ratio needs to be calculated.

        Returns:
            int:ratio (between 0 and 1)
        """

        return 1
    
    def scaleToOperator(self, collision):
        """Scales down the collision from robot to human.

        Parameters:
            collision (Collision):The collision that happened on the robot side.

        Returns:
            Collision:Collision scaled to human
        """

        return collision

    def transformFromRobotToOperator(self, collision):
        """Calculates the collision on the human and assign it to the 'result' var.

        Parameters:
            collision (Collision):The collision that happened on the robot side.
        """

        self.result = self.scaleToOperator(collision)

    def load_roboy_to_human_link_name_map(self):
        with open('../resources/roboy_to_human_linknam_map.yaml') as f:
            linkNameMaps = yaml.safe_load(f)
            return linkNameMaps.get("roboyToHumanLinkNameMap")



def main(args=None):
    rclpy.init(args=args)

    forcesMapper = ForcesMapper()
    rclpy.spin(forcesMapper)
    forcesMapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()