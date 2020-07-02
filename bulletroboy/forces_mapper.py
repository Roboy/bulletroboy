import os
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision
import yaml
from roboy_simulation_msgs.srv import LinkInfoFromName
from roboy_simulation_msgs.srv import LinkInfoFromId

class ForcesMapper(Node):
    def __init__(self):
        super().__init__('forces_imitator')

        self.action_done_event = Event()

        self.callback_group = ReentrantCallbackGroup()
        self.link_names_map = self.load_roboy_to_human_link_name_map()

        self.robotCollisionSubscription = self.create_subscription(
            Collision,
            '/roboy/simulation/collision',
            self.collision_listener,
            10)

        # Define clients
        self.roboy_link_info_from_id_client = self.create_client(LinkInfoFromId, '/roboy/simulation/roboy/link_info_from_id', callback_group=self.callback_group)
        self.operator_link_info_from_name_client = self.create_client(LinkInfoFromName, '/roboy/simulation/operator/link_info_from_name', callback_group=self.callback_group)
        
        # Define publishers
        self.exoforceCollisionPublisher = self.create_publisher(Collision, '/roboy/exoforce/collisions', 10)

    def call_service(self, client, msg):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        response = client.call(msg)
        return response

    def collision_listener(self, msg):
        """Collision subscriber handler.
        """
        self.get_logger().info('Received collision')
        self.transform_from_roboy_to_operator(msg)

    def transform_from_roboy_to_operator(self, collision):
        """Calculates the collision on the human and assign it to the 'result' var.

        Parameters:
            collision (Collision):The collision that happened on the robot side.
        """
        operator_collision = self.map_collision_to_operator(collision)
        self.get_logger().info("publishing")
        self.exoforceCollisionPublisher.publish(operator_collision)
    
    def map_collision_to_operator(self, roboy_collision):
        """Maps roboy link id in collision to the operator corresponding link id

        Parameters:
            roboy_collision (Collision): The roboy collision

        Returns:
            Collision: The collision with mapped linkid
        """
        self.get_logger().info('mapping start')
        roboy_link_info = self.get_roboy_link_info(roboy_collision.linkid)
        operator_link_name = self.link_names_map[roboy_link_info.link_name]
        operator_link_info = self.get_operator_link_info(operator_link_name)
        self.get_logger().info('responses')

        operator_collision = Collision()
        operator_collision = roboy_collision
        operator_collision.linkid = operator_link_info.link_id
        self.get_logger().info('responses2')

        position_scale = self.roboy_to_operator_link_ratio(roboy_link_info.dimensions, operator_link_info.dimensions)
        self.get_logger().info('responses3')
        #operator_collision.collision = self.scale_to_operator(operator_collision, position_scale)
        self.get_logger().info("mapping done")
        return operator_collision

    def get_roboy_link_info(self, roboy_link_id):
        """Gets the roboy link name from roboy link id by calling the corresponding servicce synchronously

        Parameters:
            roboy_link_id (uint8): The link id of roboy

        Returns:
            LinkInfoFromId: The link info of the passed link id
        """
        self.get_logger().info('Getting roboy link info')
        roboy_link_info_from_id_req = LinkInfoFromId.Request()
        roboy_link_info_from_id_req.link_id = roboy_link_id
        response = self.call_service(self.roboy_link_info_from_id_client, roboy_link_info_from_id_req)
        self.get_logger().info('Got roboy link info')
        return response

    def get_operator_link_info(self, operator_link_name):
        """Gets the operator link id from the operator link name by calling the corresponding servicce synchronously

        Parameters:
            operator_link_name (str): The link name of the operator

        Returns:
            LinkInfoFromName: The link info of the passed link name
        """
        self.get_logger().info('Getting operator link info')
        operator_link_info_from_id_req = LinkInfoFromName.Request()
        operator_link_info_from_id_req.link_name = operator_link_name
        response = self.call_service(self.operator_link_info_from_name_client, operator_link_info_from_id_req)
        self.get_logger().info('Got operator link info')
        return response

    def roboy_to_operator_link_ratio(self, roboy_dimensions, operator_dimensions):
        """Calculates the ratio between robot link and human link.

        Parameters:
            roboy_dimensions(Vector3):The bounding box of the link whose ratio needs to be calculated.
            operator_dimensions(Vector3): The bounding box of the link whose ratio is needed
        Returns:
            Vector3:ratio on the three dimensions
        """

        x_scale = roboy_dimensions.x / operator_dimensions.x
        y_scale = roboy_dimensions.y / operator_dimensions.y
        z_scale = roboy_dimensions.z / operator_dimensions.z

        return [x_scale, y_scale, z_scale]
    
    def scale_to_operator(self, collision, scale):
        """Scales down the collision from robot to human.

        Parameters:
            collision (Collision):The collision that happened on the robot side.
            scale (Vector3): The position scale from roboy to operator

        Returns:
            Collision:Collision scaled to human
        """
        collision.position.x = collision.position.x / scale[0]
        collision.position.y = collision.position.y / scale[1]
        collision.position.z = collision.position.z / scale[2]

        return collision

    def load_roboy_to_human_link_name_map(self):
        """Fetches the link name map"""
        script_dir = os.path.dirname(__file__)
        rel_path = "../resource/roboy_to_human_linkname_map.yaml"
        abs_file_path = os.path.join(script_dir, rel_path)
        with open(abs_file_path) as f:
            linkNameMaps = yaml.safe_load(f)
            return linkNameMaps.get("roboyToHumanLinkNameMap")



def main(args=None):
    rclpy.init(args=args)

    forcesMapper = ForcesMapper()
    executor = MultiThreadedExecutor()
    rclpy.spin(forcesMapper, executor)

    forcesMapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()