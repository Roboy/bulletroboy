import rclpy
from bulletroboy.operator import Operator
from geometry_msgs.msg import PoseStamped

class Operator_Cage(Operator):
	"""This class represents the "real" Operator and communicates directly with the animus_server.

	"""
	def __init__(self):
		"""
		Args:
			body_id (int): Pybullet body indentifier.

		"""
		super().__init__()
		self.pose_subscriber = self.create_subscription(PoseStamped, '/bullet_ik', self.ef_pose_callback, 10)
 
		"""Initiates the links in operator body.
		Args:
			-
		Returns:
		   	-
		"""
		pass

	def get_link_pose(self, link_id):
		"""Gets the pose with the given id.

		Args:
			link_id: id of the link wanted.
		
		Returns:
			An array containig the position vector and orientation quaternion.
		"""
		pass

