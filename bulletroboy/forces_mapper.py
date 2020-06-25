import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision
import yaml
from roboy_simulation_msgs.srv import LinkIdFromName
from roboy_simulation_msgs.srv import LinkNameFromId
from asgiref.sync import async_to_sync

class ForcesMapper(Node):
    def __init__(self):
        super().__init__('forces_imitator')

        self.result = None

        self.robotCollisionSubscription = self.create_subscription(
            Collision,
            '/roboy/simulation/collision',
            self.collision_listener,
            10)

        self.link_names_map = self.load_roboy_to_human_link_name_map()

        # Define clients
        self.roboy_link_name_from_id_client = self.create_client(LinkNameFromId, 'add_two_ints')
        self.operator_link_id_from_name_client = self.create_client(LinkIdFromName, '/roboy/simulation/operator/link_id_from_link_name')

        # Connect to services
        while not self.roboy_link_name_from_id_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        while not self.operator_link_id_from_name_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.exoforceCollisionPublisher = self.create_publisher(Collision, 'roboy/exoforce/collisions', 10)
        timer_period = 0.005 # seconds
        self.time = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Publishes collision to exoforce if any."""

        if not self.result:
            return
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
    
    def scale_to_operator(self, collision):
        """Scales down the collision from robot to human.

        Parameters:
            collision (Collision):The collision that happened on the robot side.

        Returns:
            Collision:Collision scaled to human
        """
        return collision
    
    def map_collision_to_operator(self, roboy_collision):
        roboy_link_name = self.get_roboy_link_name(roboy_collision.linkid)
        operator_link_name = self.link_names_map[roboy_link_name]
        operator_link_id = self.get_operator_link_id(operator_link_name)

        roboy_collision.linkid = operator_link_id

        return roboy_collision

    def get_roboy_link_name(self, roboy_link_id):
        roboy_link_name_from_id_req = LinkNameFromId.Request()
        roboy_link_name_from_id_req.link_id = roboy_link_id
        response = self.roboy_link_name_from_id_client.call(roboy_link_name_from_id_req)
        return response

    def get_operator_link_id(self, operator_link_name):
        operator_link_name_from_id_req = LinkIdFromName.Request()
        operator_link_name_from_id_req.link_name = operator_link_name
        response = self.roboy_link_name_from_id_client.call(operator_link_name_from_id_req)
        return response

    def transformFromRobotToOperator(self, collision):
        """Calculates the collision on the human and assign it to the 'result' var.

        Parameters:
            collision (Collision):The collision that happened on the robot side.
        """

        scaled_collision = self.scale_to_operator(collision)
        operator_collision = self.map_collision_to_operator(scaled_collision)
        self.result = operator_collision

    def load_roboy_to_human_link_name_map(self):
        """Fetches the link name map"""
        with open('../resources/roboy_to_human_linkname_map.yaml') as f:
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