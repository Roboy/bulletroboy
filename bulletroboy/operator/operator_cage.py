from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from roboy_middleware_msgs.msg import MotorCommand
from .operator import Operator, Link
from ..utils.utils import load_op_link_dims, Topics

import time

PULL_MOTOR = 16

class OperatorCage(Operator):
	"""This class represents the "real" Operator and communicates directly with the animus_server.

	"""
	def __init__(self):
		super().__init__()

	def start_node(self):
		self.pose_subscriber = self.create_subscription(PoseStamped, Topics.VR_HEADSET_POSES, self.vr_pose_listener, 1, callback_group=ReentrantCallbackGroup())
		self.motor_command_publisher = self.create_publisher(MotorCommand, Topics.MOTOR_COMMAND, 1)
		input("\nStand in initial position and press enter...")
		self.pull()
		self.start_publishing()

	def init_links(self):
		"""Initiates the links in operator class.

		Args:
			-
		Returns:
			-
		"""
		self.links = []
		dims_dict = load_op_link_dims()
		for i, key in enumerate(self.link_map):
			human_name = self.link_map[key]
			roboy_name = key
			dims = dims_dict[human_name]
			self.links.append(Link(i, human_name, roboy_name, dims))

	def pull(self):
		msg = MotorCommand()

		msg.legacy = False
		msg.motor = [PULL_MOTOR]
		msg.setpoint = [800.0]
		self.motor_command_publisher.publish(msg)
		time.sleep(1)
		msg.setpoint = [150.0]
		self.motor_command_publisher.publish(msg)

	def vr_pose_listener(self, ef_pose):
		"""Callback of the pose subscriber. Sets the pose of the link given in the msg.

		Args:
			ef_pose: received PoseStamped msg.
		
		Returns:
			-

		"""
		self.get_logger().info("Received pose for " + ef_pose.header.frame_id)
		link = self.get_human_link(ef_pose.header.frame_id)
		if link is None:
			self.get_logger().warn(ef_pose.header.frame_id + "has no mapping!")
			return
		offset = 0 if link.human_name == "neck" else 0.4
		link_pos = [ef_pose.pose.position.x + offset, ef_pose.pose.position.y, ef_pose.pose.position.z + 1.5]
		link_orn = [ef_pose.pose.orientation.x, 
						ef_pose.pose.orientation.y, 
						ef_pose.pose.orientation.z, 
						ef_pose.pose.orientation.w]
		link.set_pose(link_pos, link_orn)
