from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from .operator import Operator, Link
from .utils import load_op_link_dims


class OperatorCage(Operator):
	"""This class represents the "real" Operator and communicates directly with the animus_server.

	"""
	def __init__(self):
		super().__init__()
		self.pose_subscriber = self.create_subscription(PoseStamped, '/bullet_ik', self.vr_pose_listener, 1, callback_group=ReentrantCallbackGroup())
		input("\nStand in initial position and press enter...")
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
		link_pos = [ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z + 1.5]
		link_orn = [ef_pose.pose.orientation.x, 
						ef_pose.pose.orientation.y, 
						ef_pose.pose.orientation.z, 
						ef_pose.pose.orientation.w]
		link.set_pose(link_pos, link_orn)


import rclpy

def main(args=None):
    rclpy.init(args=args)

    operator = OperatorCage()
    rclpy.spin(operator, MultiThreadedExecutor())

    operator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
	main()
