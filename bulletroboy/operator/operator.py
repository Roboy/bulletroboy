from abc import ABC, abstractmethod

import numpy as np
import rclpy
from roboy_middleware_msgs.msg import EFPose
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from roboy_control_msgs.srv import GetLinkPose
from roboy_simulation_msgs.srv import LinkInfoFromName

from ..utils.utils import Services, Topics, call_service_async


class Link():
	"""This class handles one operator link.

	"""
	def __init__(self, id, human_name, roboy_name, dims, init_pose=None):
		"""
		Args:
			id (int): Link id.
			human_name (string): Human link name.
			roboy_name (string): Name of roboy's link that maps to this human link.
			dims (3darray[float]): x, y, and z dimensions of the link.
			init_pose ([3darray[float], 4darray[float]]): Intial pose (pos, orn) of the link.

		"""
		self.human_name = human_name
		self.roboy_name = roboy_name
		self.id = id
		self.dims = dims
		self.init_pose = init_pose
		self.pose = init_pose

	def set_pose(self, pos, orn):
		"""Updates link object pose.
		
		Args:
			pos (3darray[float]): Position of the link.
			orn (4darray[float]): Orientation of the link.

		Returns:
		   	-

		"""
		if self.init_pose is None:
			self.init_pose = [pos, orn]
		self.pose = [pos, orn]

	def get_center(self):
		"""Gets link center point.
		
		Args:
			-

		Returns:
		   	-

		"""
		return np.array(self.pose[0]) if self.pose is not None else None

class Operator(Node, ABC):
	"""This class handles the operator's ROS node.

	"""
	def __init__(self, node_name="operator"):
		"""
		Args:
			-

		"""
		super().__init__(node_name)
		self.ready = False
		self.links = None

		# State Mapper node parameters client
		self.state_mapper_parameters_client = self.create_client(GetParameters, Services.STATE_MAPPER_GET)
		request = GetParameters.Request()
		request.names = ['roboy_link_names','operator_link_names']
		call_service_async(self.state_mapper_parameters_client, request, self.initialize, self.get_logger())

	def initialize(self, future):
		try:
			result = future.result()
		except Exception as e:
			self.get_logger().error("mapped_links service call failed %r" % (e,))
		else:
			roboy_link_names = result.values[0].string_array_value
			operator_link_names = result.values[1].string_array_value
			self.link_map = {k: v for k, v in zip(roboy_link_names, operator_link_names)}

			self.init_node()
			self.start_node()
			self.ready = True

	def init_node(self):
		self.init_links()
		
		self.ef_publisher = self.create_publisher(EFPose, Topics.OP_EF_POSES, 1)
		self.link_info_service = self.create_service(LinkInfoFromName, Services.LINK_INFO_FROM_NAME, self.link_info_from_name_callback)
		self.initial_pose_service = self.create_service(GetLinkPose, Services.OP_INITIAL_LINK_POSE, self.initial_link_pose_callback)

	@abstractmethod
	def start_node(self):
		pass

	def start_publishing(self, period=0.02):
		self.timer = self.create_timer(period, self.publish_ef_state)

	def init_end_effectors(self, efs, init_pos=[None], init_orn=[None]):
		"""Initilizes operator's end effector.
		
		Args:
			efs (list[string]): Names of the end effectors link.
			init_pos (list[float[3]]): Initial positions of the end effectors.
			init_orn (list[float[4]]): Initial orientations of the end effectors.

		Returns:
		   	-

		"""
		self.end_effectors = []

		if len(efs) != len(init_pos): init_pos *= len(efs)
		if len(efs) != len(init_orn): init_orn *= len(efs)
		
		for ef, pos, orn in zip(efs, init_pos, init_orn):
			ef_link = self.get_link(ef)
			if ef_link is None:
				self.get_logger().error(f"EF {ef} is not a link of the operator!")
				return False
			
			if pos is not None and orn is not None:
				ef_link.init_pose = [pos, orn]

			self.end_effectors.append(ef_link)
			self.get_logger().info("EF " + ef_link.human_name + ": " + str(ef_link.id))
		
		return True

	def get_human_link(self, roboy_link_name):
		"""Gets link object given the roboy's link name that maps to it.
		
		Args:
			roboy_link_name (string): Roboy's link name to look for.

		Returns:
		   	Link: Link object.

		"""
		for link in self.links:
			if link.roboy_name == roboy_link_name:
				return link
		return None

	def get_link(self, human_link_name):
		"""Gets link object given a human link name.
		
		Args:
			human_link_name (string): Link name to look for.

		Returns:
		   	Link: Link object.

		"""
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

			msg = EFPose()
			msg.ef_name = ef_link.human_name

			msg.ef_pose.position.x = ef_pos[0]
			msg.ef_pose.position.y = ef_pos[1]
			msg.ef_pose.position.z = ef_pos[2]

			msg.ef_pose.orientation.x = ef_orn[0]
			msg.ef_pose.orientation.y = ef_orn[1]
			msg.ef_pose.orientation.z = ef_orn[2]
			msg.ef_pose.orientation.w = ef_orn[3]

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
		"""ROS service callback to get initial link position from link name.

		"""
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
		self.get_logger().debug(f"Responding")

		return response
