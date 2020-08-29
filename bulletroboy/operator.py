from abc import ABC, abstractmethod
import numpy as np
import rclpy

from rclpy.node import Node
from roboy_control_msgs.srv import GetLinkPose
from geometry_msgs.msg import PoseStamped
from roboy_simulation_msgs.srv import LinkInfoFromName

class Operator(Node, ABC):
	"""This class handles the operator's ROS node.

	"""
	def __init__(self):
		"""
		Args:
			body_id (int): Pybullet body indentifier.

		"""
		super().__init__("operator_node")
		# TODO delete from moves and add to init_links 
		self.links = []
		self.end_effectors = {}
		self.init_links()
		self.ef_publisher = self.create_publisher(PoseStamped, '/roboy/simulation/operator/pose/endeffector', 1)
		self.link_info_service = self.create_service(LinkInfoFromName, '/roboy/simulation/operator/link_info_from_name', self.link_info_from_name_callback)
		self.initial_pose_service = self.create_service(GetLinkPose, '/roboy/simulation/operator/initial_link_pose', self.initial_link_pose_callback)

	def get_link_center(self, link_name):
		"""Gets link's center point in world frame.
		
		Args:
			link_name (string): Name of the link to search.

		Returns:
		   	List[dict]: 'name' and 'id' for each link in the operator's pybullet body.

		"""
		center = None
		link = self.get_link_info_from_name(link_name)
		if link:
			pose = self.get_link_pose(link['id'])
			if pose: 
				center = np.asarray(pose)[0]
		return center
		
	@abstractmethod
	def init_links(self):
		"""Initiates the links in operator body.
		
		Args:
			-

		Returns:
		   	-

		"""
		pass

	@abstractmethod
	def get_link_pose(self, link_id):
		"""Gets the pose with the given id.

		Args:
			link_id: id of the link wanted.
		
		Returns:
			An array containig the position vector and orientation quaternion.
		"""
		pass

	def get_link_info_from_name(self, link_name):
		"""Gets the information about the link with the given name from the links dictionary.

		Args:
			link_name: name of the link wanted.
		"""
		link = list(filter(lambda link: link['name'] == link_name, self.links))
		assert len(link) == 1
		return link[0]

	def publish_ef_state(self):
		"""Publishes the end effectors' state as a ROS message.
		
		Args:
			-

		Returns:
		   	-

		"""
		for ef in self.end_effectors.keys():
		   self.get_logger().debug('Sending Endeffector pose: ' + ef)
		   ef_pos,ef_orn = self.get_link_pose(self.end_effectors[ef])
		   msg = PoseStamped()
		   msg.header.frame_id = ef

		   msg.pose.position.x = ef_pos[0]
		   msg.pose.position.y = ef_pos[1]
		   msg.pose.position.z = ef_pos[2]

		   msg.pose.orientation.x = ef_orn[0]
		   msg.pose.orientation.y = ef_orn[1]
		   msg.pose.orientation.z = ef_orn[2]
		   msg.pose.orientation.w = ef_orn[3]

		   self.ef_publisher.publish(msg)

	def link_info_from_name_callback(self, request, response):
		"""ROS service callback to get link info from link name.
		"""
		#self.get_logger().info("received call")
		link = self.get_link_info_from_name(request.link_name)
		response.link_id = link['id']
		response.dimensions.x, response.dimensions.y, response.dimensions.z = link['dims']
		#self.get_logger().info("responding")
		return response 

	def initial_link_pose_callback(self, request, response):
		'''ROS service callback to get initial link position from link name.
		'''
		self.get_logger().info(f"Service Initial Link Pose: request received for {request.link_name}")

		link = self.get_link_info_from_name(request.link_name)
		
		link_pos = link['init_pose'][0]
		link_orn = link['init_pose'][1]
		response.pose.position.x = link_pos[0]
		response.pose.position.y = link_pos[1]
		response.pose.position.z = link_pos[2]

		response.pose.orientation.x = link_orn[0]
		response.pose.orientation.y = link_orn[1]
		response.pose.orientation.z = link_orn[2]
		response.pose.orientation.w = link_orn[3]
		self.get_logger().info(f"Responding")

		return response

