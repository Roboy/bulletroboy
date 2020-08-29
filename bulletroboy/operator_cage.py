import rclpy
from bulletroboy.operator import Operator
from geometry_msgs.msg import PoseStamped
import bulletroboy.utils as utils

class OperatorCage(Operator):
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
		dims_dict = utils.load_op_link_dims()
		id = 0
		for link_name in self.link_names_map.values():
			link = {}
			link['name'] = link_name
			# link['id'] = i
			link['id'] = id
			link['dims'] = dims_dict[link_name]
			link['init_pose'] = None
			link['pose'] = None
			self.links.append(link)
			if link_name == 'left_wrist':
				self.end_effectors[link_name] = id
				self.get_logger().info("EF hand_left id: " + str(id))
			if link_name == 'right_wrist':
				self.end_effectors[link_name] = id
				self.get_logger().info("EF hand_right id: " + str(id))
			if link_name == 'neck':
				self.get_logger().info("Neck id: " + str(id))
			id += 1

	def get_link_pose(self, link_id):
		"""Gets the pose with the given id.

		Args:
			link_id: id of the link wanted.
		
		Returns:
			An array containig the position vector and orientation quaternion.
		"""
		link = list(filter(lambda link: link['id'] == link_id, self.links))
		assert len(link) == 1
		if link[0]['pose']:
			return link[0]['pose']


	def set_link_pose_cb(self, ef_pose):
		"""Callback of the pose subscriber. Sets the pose of the link given in the msg.

		Args:
			ef_pose: received PoseStamped msg.
		
		Returns:
			-
		"""
		self.get_logger().info("Received pose for " + ef_pose.header.frame_id)
		op_link_name = self.link_names_map[ef_pose.header.frame_id] 
		link_pos = [ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z]
		link_orn = [ef_pose.pose.orientation.x, 
						ef_pose.pose.orientation.y, 
						ef_pose.pose.orientation.z, 
						ef_pose.pose.orientation.w]
		link = list(filter(lambda link: link['name'] == op_link_name, self.links))
		if len(link) == 1:
			if not link[0]['init_pose']:
				self.get_logger().info('not initialized')
				link[0]['init_pose'] = [link_pos, link_orn]
			link[0]['pose'] = [link_pos, link_orn]
			self.publish_ef_state()
		
		

