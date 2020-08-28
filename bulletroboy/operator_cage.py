import rclpy
from bulletroboy.operator import Operator
from geometry_msgs.msg import PoseStamped
import bulletroboy.utils as utils

class Operator_Cage(Operator):
	"""This class represents the "real" Operator and communicates directly with the animus_server.

	"""
	def __init__(self):
		"""
		Args:
			body_id (int): Pybullet body indentifier.

		"""
		self.link_names_map = utils.load_roboy_to_human_link_name_map()
		super().__init__()
		self.pose_subscriber = self.create_subscription(PoseStamped, '/bullet_ik', self.set_link_pose_cb, 10)

	def init_links(self):
		"""Initiates the links in operator body.
		Args:
			-
		Returns:
		   	-
		"""
		link = None
		for i, link_name in link_names_map.keys():
			link['name'] = link_name
			link['id'] = i
			link['dims'] = utils.load_op_link_dims
	def get_link_pose(self, link_id):
		"""Gets the pose with the given id.

		Args:
			link_id: id of the link wanted.
		
		Returns:
			An array containig the position vector and orientation quaternion.
		"""
		pass

	def set_link_pose_cb(self, msg):
		"""Callback of the pose subscriber. Sets the pose of the link given in the msg.

		Args:
			msg: received PoseStamped msg.
		
		Returns:
			-
		"""
		pass

