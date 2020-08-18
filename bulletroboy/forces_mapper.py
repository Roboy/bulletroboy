import os
from threading import Event

import numpy as np
from scipy.spatial.transform import Rotation as R

from pyquaternion import Quaternion

import yaml 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# from sensor_msgs.msg import JointState
import bulletroboy.utils as utils
from roboy_simulation_msgs.msg import Collision
from roboy_simulation_msgs.srv import LinkInfoFromName
from roboy_simulation_msgs.srv import LinkInfoFromId
from roboy_control_msgs.srv import GetLinkPose
from geometry_msgs.msg import PoseStamped

class ForcesMapper(Node):
    def __init__(self):
        super().__init__('forces_imitator')

        self.action_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()
        self.roboy_to_human_link_names_map = utils.load_roboy_to_human_link_name_map()

        self.human_to_roboy_link_names_map = {v: k for k, v in self.roboy_to_human_link_names_map.items()}

        self.roboy_initial_link_poses = {}
        self.operator_initial_link_poses = {}
        
        # Define clients
        self.roboy_link_info_from_id_client = self.create_client(LinkInfoFromId, '/roboy/simulation/roboy/link_info_from_id', callback_group=self.callback_group)
        self.operator_link_info_from_name_client = self.create_client(LinkInfoFromName, '/roboy/simulation/operator/link_info_from_name', callback_group=self.callback_group)
        self.roboy_initial_link_pose_client = self.create_client(GetLinkPose, '/roboy/simulation/roboy/initial_link_pose', callback_group=self.callback_group)
        self.operator_initial_link_pose_client = self.create_client(GetLinkPose, '/roboy/simulation/operator/initial_link_pose', callback_group=self.callback_group)

        # Define subscriptions
        self.robot_collision_subscription = self.create_subscription(
            Collision,
            'roboy/simulation/roboy/collision',
            self.collision_listener,
            1)
        # Operator EF pose subscriber
        self.operator_movement_subscription = self.create_subscription(
            PoseStamped, 
            '/roboy/simulation/operator/pose/endeffector', 
            self.operator_movement_listener, 
            1)
        # Define publishers
        self.exoforce_collision_publisher = self.create_publisher(Collision, '/roboy/simulation/exoforce/operator/collisions', 1)
        self.right_ef_publisher = self.create_publisher(PoseStamped, '/roboy/exoforce/pose/endeffector/right', 1)
        self.left_ef_publisher = self.create_publisher(PoseStamped, '/roboy/exoforce/pose/endeffector/left', 1)

    def operator_movement_listener(self, ef_pose):
        """Callback function of the endeffector subscription. Processes the msg received and moves the link accordingly.

        Args:
            ef_pose: end effector pose received from the operator.
        """
        self.get_logger().debug('Endeffector pose received: ' + ef_pose.header.frame_id)

        #process message
        self.get_logger().debug('got frame-id' + ef_pose.header.frame_id)

        ef_pose.header.frame_id = self.human_to_roboy_link_names_map[ef_pose.header.frame_id]
        
        orn = [ef_pose.pose.orientation.x, 
                ef_pose.pose.orientation.y, 
                ef_pose.pose.orientation.z, 
                ef_pose.pose.orientation.w]
        
        link_orn = self.adapt_orientation_to_roboy(ef_pose.header.frame_id, orn)
        ef_pose.pose.orientation.x = link_orn[0]
        ef_pose.pose.orientation.y = link_orn[1]
        ef_pose.pose.orientation.z = link_orn[2]
        ef_pose.pose.orientation.w = link_orn[3]
        
        self.get_logger().debug('Publishing EF-Pose')

        if ef_pose.header.frame_id.find("right") != -1:
            self.right_ef_publisher.publish(ef_pose)
        else:
            self.left_ef_publisher.publish(ef_pose)
          
    def call_service(self, client, msg):
        """Calls a client synchnrnously passing a msg and returns the response

        Parameters:
            client (RosClient): The client used to communicate with the server
            msg (Object): A server msg to send to the server as a request

        Returns:
            response (Object): A response msg from the server
        """
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        response = client.call(msg)
        return response

    def collision_listener(self, msg):
        """Collision subscriber handler.
        """
        self.get_logger().info("got collision")
        operator_collision = self.map_collision_to_operator(msg)
        self.get_logger().info("publishing")
        self.exoforce_collision_publisher.publish(operator_collision)

    def map_collision_to_operator(self, roboy_collision):
        """Maps roboy link id in collision to the operator corresponding link id

        Parameters:
            roboy_collision (Collision): The roboy collision

        Returns:
            Collision: The collision with mapped linkid
        """
        self.get_logger().debug('mapping start')
        roboy_link_info = self.get_roboy_link_info(roboy_collision.linkid)
        operator_link_name = self.roboy_to_human_link_names_map[roboy_link_info.link_name]
        operator_link_info = self.get_operator_link_info(operator_link_name)
        self.get_logger().debug('responses')

        operator_collision = Collision()
        operator_collision = roboy_collision
        operator_collision.linkid = operator_link_info.link_id
        self.get_logger().debug('responses2')

        position_scale = self.roboy_to_operator_link_ratio(roboy_link_info.dimensions, operator_link_info.dimensions)
        operator_collision = self.scale_to_operator(operator_collision, position_scale)
        new_position = self.adapt_vector_to_orientation(roboy_link_info.link_name, [operator_collision.position.x, operator_collision.position.y, operator_collision.position.z])
        operator_collision.position.x = new_position[0]
        operator_collision.position.y = new_position[1]
        operator_collision.position.z = new_position[2]
        self.get_logger().debug("mapping done")

        new_contact_normal = self.adapt_vector_to_orientation(roboy_link_info.link_name, [roboy_collision.contactnormal.x, roboy_collision.contactnormal.y, roboy_collision.contactnormal.z])
        operator_collision.contactnormal.x = new_contact_normal[0]
        operator_collision.contactnormal.y = new_contact_normal[1]
        operator_collision.contactnormal.z = new_contact_normal[2]

        return operator_collision

    def get_roboy_link_info(self, roboy_link_id):
        """Gets the roboy link name from roboy link id by calling the corresponding servicce synchronously

        Parameters:
            roboy_link_id (uint8): The link id of roboy

        Returns:
            LinkInfoFromId: The link info of the passed link id
        """
        self.get_logger().debug('Getting roboy link info')
        roboy_link_info_from_id_req = LinkInfoFromId.Request()
        roboy_link_info_from_id_req.link_id = roboy_link_id
        response = self.call_service(self.roboy_link_info_from_id_client, roboy_link_info_from_id_req)
        self.get_logger().info("received response")
        return response

    def get_operator_link_info(self, operator_link_name):
        """Gets the operator link id from the operator link name by calling the corresponding servicce synchronously

        Parameters:
            operator_link_name (str): The link name of the operator

        Returns:
            LinkInfoFromName: The link info of the passed link name
        """
        self.get_logger().debug('Getting operator link info')
        operator_link_info_from_id_req = LinkInfoFromName.Request()
        operator_link_info_from_id_req.link_name = operator_link_name
        response = self.call_service(self.operator_link_info_from_name_client, operator_link_info_from_id_req)
        self.get_logger().debug('Got operator link info')
        return response

    def roboy_to_operator_link_ratio(self, roboy_dimensions, operator_dimensions):
        """Calculates the ratio between robot link and human link.

        Parameters:
            roboy_dimensions(Position):The bounding box of the link whose ratio needs to be calculated.
            operator_dimensions(Position): The bounding box of the link whose ratio is needed
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

    def get_initial_link_pose(self, link_name, client):
        """Gets initial link pose for a link using its name

        Parameters: 
            link_name (String): The name of the link
            client (RosClient): The client used to pass a request and get a response from service

        Returns:
            A array containing two vectors, first is position and second is orientations
        """
        self.get_logger().debug('Getting initial' + link_name + ' link pose')
        initial_link_pose_req = GetLinkPose.Request()
        initial_link_pose_req.link_name = link_name
        response = self.call_service(client, initial_link_pose_req)
        self.get_logger().debug('service called')

        return [[response.pose.position.x, 
            response.pose.position.y, 
            response.pose.position.z], 
            [response.pose.orientation.x, 
            response.pose.orientation.y, 
            response.pose.orientation.z, 
            response.pose.orientation.w]]

    def adapt_orientation_to_roboy(self, roboy_link_name, orientation):
        """Adapts the orientation received to roboy's link according to roboy's and operators initial links.

        Parameters:
            roboy_link_name (str): link name according to roboy's urfd.
            orientation (Vector4): received target orientation from operator.
        
        Returns:
            Vector4: The adapted target orientation.
        """
        
        diff = self.roboy_to_operator_orientation_diff(roboy_link_name)
        quat = diff * orientation 

        return [quat[0], quat[1], quat[2], quat[3]]

    def adapt_vector_to_orientation(self, roboy_link_name, contact_normal):
        """Adapts the vector's direction received to roboy's link according to roboy's and operators initial links.

        Parameters:
            roboy_link_name (str): link name according to roboy's urfd.
            contact_normal (Vector3): received target orientation from operator.

        Returns:
            Vector4: The adapted target orientation.
        """

        diff = self.roboy_to_operator_orientation_diff(roboy_link_name)
        R = diff.rotation_matrix

        return R.dot(contact_normal)

    def roboy_to_operator_orientation_diff(self, roboy_link_name):
        """Calculated the orientation difference between the roboy link frame and the operator link frame

        Parameters:
            roboy_link_name (str): The name of the roboy link

        Returns:
            diff (Quaternion): The difference of quaternions between roboy and operator links
        """
        if self.roboy_initial_link_poses.get(roboy_link_name) == None :
            self.roboy_initial_link_poses[roboy_link_name] = self.get_initial_link_pose(roboy_link_name, self.roboy_initial_link_pose_client)
            self.get_logger().info("Got roboy initial pose for link " + roboy_link_name + " : " + str(self.roboy_initial_link_poses[roboy_link_name]))
        roboy_init_orn = Quaternion(np.array(self.roboy_initial_link_poses[roboy_link_name][1]))
        operator_link_name = self.roboy_to_human_link_names_map[roboy_link_name]
        if self.operator_initial_link_poses.get(operator_link_name) == None :
            self.operator_initial_link_poses[operator_link_name] = self.get_initial_link_pose(operator_link_name, self.operator_initial_link_pose_client)        
            self.get_logger().info("Got operator initial pose for link " + operator_link_name + " : " + str(self.operator_initial_link_poses[operator_link_name]))
        op_init_orn = Quaternion(np.array(self.operator_initial_link_poses[operator_link_name][1]))

        diff = roboy_init_orn / op_init_orn

        return diff

def main(args=None):
    rclpy.init(args=args)

    forcesMapper = ForcesMapper()
    executor = MultiThreadedExecutor()
    rclpy.spin(forcesMapper, executor)

    forcesMapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()