from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from roboy_simulation_msgs.msg import TendonUpdate
from .operator import Operator, Link
from ..utils.utils import Topics, Services

import time

# This constants need to be defined in a conf file
PULL_TIME = 1 # in seconds
PULL_MOTOR = 16
PULL_FORCE = 40 # in Newtons
MIN_FORCE = 4 # in Newtons

class OperatorHW(Operator):
	"""This class handles operator related functions and communicates directly with the TeleportVR app.

	"""

	def start_node(self):
		self.target_force_publisher = self.create_publisher(TendonUpdate, Topics.TARGET_FORCE, 1)		
		self.create_subscription(PoseStamped, Topics.VR_HEADSET_POSES, self.vr_pose_listener, 1, callback_group=ReentrantCallbackGroup())

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
		for i, key in enumerate(self.link_map):
			human_name = self.link_map[key]
			roboy_name = key
			dims = self.get_parameter("operator_link_dimentions." + human_name).get_parameter_value().double_array_value
			if human_name == "neck":
				self.links.append(Link(i, human_name, roboy_name, dims, [[.0,.0,1.5],[.0,.0,.0,.0]]))
			else:
				self.links.append(Link(i, human_name, roboy_name, dims))

	def pull(self):
		"""Pulls operator to simulate connection to the roboy.

		Args:
			-

		Returns:
			-
			
		"""
		msg = TendonUpdate()
		msg.tendon_id = PULL_MOTOR
		msg.force = float(PULL_FORCE)
		self.target_force_publisher.publish(msg)
		time.sleep(PULL_TIME)
		msg.force = float(MIN_FORCE)
		self.target_force_publisher.publish(msg)

	def vr_pose_listener(self, link_pose):
		"""Callback of the pose subscriber. Sets the pose of the link given in the msg.

		Args:
			link_pose: received PoseStamped msg.
		
		Returns:
			-

		"""
		self.get_logger().debug("Received pose for " + link_pose.header.frame_id)
		link = self.get_human_link(link_pose.header.frame_id)
		if link is None:
			self.get_logger().warn(link_pose.header.frame_id + "has no mapping!")
			return
		offset = 0 if link.human_name == "neck" else 0.4
		link_pos = [link_pose.pose.position.x + offset, link_pose.pose.position.y, link_pose.pose.position.z + 1.5]
		link_orn = [link_pose.pose.orientation.x, 
						link_pose.pose.orientation.y, 
						link_pose.pose.orientation.z, 
						link_pose.pose.orientation.w]
		link.set_pose(link_pos, link_orn)
