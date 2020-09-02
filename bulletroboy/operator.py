from abc import ABC, abstractmethod
import numpy as np
import rclpy

from rclpy.node import Node
from roboy_control_msgs.srv import GetLinkPose
from geometry_msgs.msg import PoseStamped
from roboy_simulation_msgs.srv import LinkInfoFromName

from .utils import load_roboy_to_human_link_name_map

class Link():
	def __init__(self, id, human_name, roboy_name, dims, init_pose=None):
		self.human_name = human_name
		self.roboy_name = roboy_name
		self.id = id
		self.dims = dims
		self.init_pose = init_pose
		self.pose = init_pose

	def set_pose(self, pos, orn):
		if self.init_pose is None:
			self.init_pose = [pos, orn]
		self.pose = [pos, orn]

	def get_center(self):
		return np.array(self.pose[0]) if self.pose is not None else None

class Operator(Node, ABC):
	"""This class handles the operator's ROS node.

	"""
	def __init__(self):
		"""
		Args:
			-

		"""
		super().__init__("operator_node")
		# TODO delete from moves and add to init_links
		self.link_map = load_roboy_to_human_link_name_map()
		self.init_links()
		self.init_end_effectors(["left_wrist", "right_wrist"])

		self.ef_publisher = self.create_publisher(PoseStamped, '/roboy/simulation/operator/pose/endeffector', 1)
		self.link_info_service = self.create_service(LinkInfoFromName, '/roboy/simulation/operator/link_info_from_name', self.link_info_from_name_callback)
		self.initial_pose_service = self.create_service(GetLinkPose, '/roboy/simulation/operator/initial_link_pose', self.initial_link_pose_callback)

	def start_publishing(self, period=0.1):
		self.timer = self.create_timer(period, self.publish_ef_state)

	def init_end_effectors(self, efs):
		self.end_effectors = []
		for link in self.links:
			if link.human_name in efs:
				self.end_effectors.append(link)
				self.get_logger().info("EF " + link.human_name + ": " + str(link.id))
		

	def get_human_link(self, roboy_link_name):
		for link in self.links:
			if link.roboy_name == roboy_link_name:
				return link
		return None

	def get_link(self, human_link_name):
		for link in self.links:
			if link.human_name == human_link_name:
				return link
		return None

	@abstractmethod
	def init_links(self):
		"""Initiates the links in operator body.
		
		Args:
			-

		Returns:
		   	-

		"""
		pass

	def publish_ef_state(self):
		"""Publishes the end effectors' state as a ROS message.
		
		Args:
			-

		Returns:
		   	-

		"""
		for ef_link in self.end_effectors:
		   if ef_link.pose is None:
			   continue
		#    self.get_logger().info('Sending Endeffector pose: ' + ef_link.human_name)
		   ef_pos, ef_orn = ef_link.pose
		   msg = PoseStamped()
		   msg.header.frame_id = ef_link.human_name

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
		link = self.get_link(request.link_name)
		response.link_id = link.id
		response.dimensions.x, response.dimensions.y, response.dimensions.z = link.dims
		#self.get_logger().info("responding")
		return response 

	def initial_link_pose_callback(self, request, response):
		'''ROS service callback to get initial link position from link name.
		'''
		self.get_logger().info(f"Service Initial Link Pose: request received for {request.link_name}")

		link = self.get_link(request.link_name)
		
		link_pos = link.init_pose[0]
		link_orn = link.init_pose[1]
		response.pose.position.x = link_pos[0]
		response.pose.position.y = link_pos[1]
		response.pose.position.z = link_pos[2]

		response.pose.orientation.x = link_orn[0]
		response.pose.orientation.y = link_orn[1]
		response.pose.orientation.z = link_orn[2]
		response.pose.orientation.w = link_orn[3]
		self.get_logger().info(f"Responding")

		return response
