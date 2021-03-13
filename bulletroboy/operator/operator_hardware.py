import time

from roboy_middleware_msgs.msg import EFPose
from roboy_middleware_msgs.srv import InitExoforce
from roboy_simulation_msgs.msg import TendonUpdate

from ..utils.utils import Services, Topics
from .operator import Link, Operator


class OperatorHW(Operator):
	"""This class handles operator related functions and communicates directly with the TeleportVR app.

	"""

	def start_node(self):
		self.declare_parameters(
			namespace='',
			parameters=[('pull_motor', -1),
						('pull_force', 30.0),
						('pull_time', 1.0)]
			)
		
		self.pull_motor = self.get_parameter("pull_motor").get_parameter_value().integer_value
		self.pull_force = self.get_parameter("pull_force").get_parameter_value().double_value
		self.pull_time = self.get_parameter("pull_time").get_parameter_value().double_value

		self.target_force_publisher = self.create_publisher(TendonUpdate, Topics.TARGET_FORCE, 1)		
		self.create_subscription(EFPose, Topics.VR_HEADSET_POSES, self.vr_pose_listener, 1)
		self.init_service = self.create_service(InitExoforce, Services.INIT_OPERATOR, self.init_callback)

	def init_links(self):
		"""Initiates the links in operator class.

		Args:
			-

		Returns:
			-

		"""
		self.declare_parameters(
			namespace='',
			parameters=[('operator_link_dimentions.' + link_name, None) for link_name in self.link_map.values()]
			)
				
		self.links = []
		for i, key in enumerate(self.link_map):
			human_name = self.link_map[key]
			roboy_name = key
			dims = self.get_parameter("operator_link_dimentions." + human_name).get_parameter_value().double_array_value
			if human_name == "neck":
				self.links.append(Link(i, human_name, roboy_name, dims, [[.0,.0,1.5],[.0,.0,.0,.0]]))
			else:
				self.links.append(Link(i, human_name, roboy_name, dims))

	def init_callback(self, request, response):
		"""Callback for ROS service to init the operator.

		"""
		self.get_logger().info("Initializing...")

		ef_names = [name for name in request.ef_name]
		init_poses = [([pose.position.x, pose.position.y, pose.position.z],[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]) for pose in request.ef_init_pose]
		init_poses = list(map(lambda pose: self.transform_pose_vr_to_cage(pose[0], pose[1]), init_poses))

		init_positions = [pose[0] for pose in init_poses]
		init_orientations = [pose[1] for pose in init_poses]

		if not self.init_end_effectors(ef_names, init_positions, init_orientations):
			err_msg = "Failed to initialize end effectors!"
			self.get_logger().error(f"Could not be initialized: {err_msg}")
			response.success = False
			response.message = err_msg
			return response
		
		self.pull()
		self.start_publishing()

		self.get_logger().info("Successfully initialized.")
		response.success = True

		return response

	def pull(self):
		"""Pulls operator to simulate connection to the roboy.

		Args:
			-

		Returns:
			-
			
		"""
		if self.pull_motor < 0:
			self.get_logger().warn("Cannot pull, pull_motor not set!")
		else:
			msg = TendonUpdate()
			msg.tendon_id = self.pull_motor
			msg.force = float(self.pull_force)
			self.target_force_publisher.publish(msg)
			time.sleep(self.pull_time)
			msg.force = 0.0
			self.target_force_publisher.publish(msg)

	def transform_pose_vr_to_cage(self, position, orientation):
		"""Transforms pose from vr frame to cage frame.

		Args:
			position (3darray[float]): Position in vr frame.
			orientation (4darray[float]): Orientation in vr frame.
		
		Returns:
			3darray[float]: Position in cage frame.
			4darray[float]: Orientation in cage frame.

		"""
		x_offset = 0.2 # x axis offset, because roboy arms are longer
		z_offset = 1.5 # height offset, because of how we define the cage frame last semester, this will be address with the frames calibration

		position[0] += x_offset
		position[2] -= z_offset

		return position, orientation

	def vr_pose_listener(self, ef_pose):
		"""Callback of the pose subscriber. Sets the pose of the link given in the msg.

		Args:
			ef_pose: received EFPose msg.
		
		Returns:
			-

		"""
		self.get_logger().debug("Received pose for " + ef_pose.ef_name)

		link = self.get_link(ef_pose.ef_name)

		if link is None:
			self.get_logger().warn(ef_pose.ef_name + " is not an operator link.")
			return

		link_pos = [ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z]
		link_orn = [ef_pose.pose.orientation.x, ef_pose.pose.orientation.y, ef_pose.pose.orientation.z, ef_pose.pose.orientation.w]

		link.set_pose(self.transform_pose_vr_to_cage(link_pos, link_orn))
