import numpy as np
from scipy.spatial.transform import Rotation as R

from pyquaternion import Quaternion

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from ..utils.utils import Topics, Services
from roboy_simulation_msgs.msg import Collision
from roboy_control_msgs.msg import EFPose
from roboy_simulation_msgs.srv import LinkInfoFromName
from roboy_simulation_msgs.srv import LinkInfoFromId
from roboy_control_msgs.srv import GetLinkPose

class StateMapper(Node):
	def __init__(self):
		super().__init__('state_mapper')
		self.declare_parameters(
			namespace='',
			parameters=[
				('roboy_link_names', None),
				('operator_link_names', None)
			])
		
		self.callback_group = ReentrantCallbackGroup()

		self.roboy_link_names = self.get_parameter("roboy_link_names").get_parameter_value().string_array_value
		self.operator_link_names = self.get_parameter("operator_link_names").get_parameter_value().string_array_value
		
		assert len(self.roboy_link_names) == len(self.operator_link_names), "Roboy and operator links list have different lenghts"

		self.roboy_initial_link_poses = {}
		self.operator_initial_link_poses = {}
		
		#to avoid deadlocks in services
		self.roboy_is_ready = False

		# Define clients
		self.roboy_link_info_from_id_client = self.create_client(LinkInfoFromId, Services.LINK_INFO_FROM_ID, callback_group=self.callback_group)
		self.operator_link_info_from_name_client = self.create_client(LinkInfoFromName, Services.LINK_INFO_FROM_NAME, callback_group=self.callback_group)
		self.roboy_initial_link_pose_client = self.create_client(GetLinkPose, Services.ROBOY_INITIAL_LINK_POSE, callback_group=self.callback_group)
		self.operator_initial_link_pose_client = self.create_client(GetLinkPose, Services.OP_INITIAL_LINK_POSE, callback_group=self.callback_group)

		# Define subscriptions
		self.robot_collision_subscription = self.create_subscription(
			Collision,
			Topics.ROBOY_COLLISIONS,
			self.collision_listener,
			1)
		# Operator EF pose subscriber
		self.operator_movement_subscription = self.create_subscription(
			EFPose, 
			Topics.OP_EF_POSES, 
			self.operator_movement_listener, 
			1)
		# Define publishers
		self.exoforce_collision_publisher = self.create_publisher(Collision, Topics.MAPPED_COLLISIONS, 1)
		self.right_ef_publisher = self.create_publisher(EFPose, Topics.MAPPED_OP_REF_POSE, 1)
		self.left_ef_publisher = self.create_publisher(EFPose, Topics.MAPPED_OP_LEF_POSE, 1)

		# Define service
		self.operator_initial_head_pose = self.create_service(GetLinkPose, Services.INITIAL_HEAD_POSE, self.operator_initial_head_pose_callback)
	
	def get_roboy_link_name(self, operator_link):
		"""Gets corresponding roboy link's name to the given operator link's name.
		
		Args:
			operator_link (string): Operator link's name.

		Returns:
		   	string: Corresponding roboy link's name.

		"""
		roboy_link = None
		for roboy, oper in zip(self.roboy_link_names, self.operator_link_names):
			if oper == operator_link:
				roboy_link = roboy
				break
		return roboy_link

	def get_operator_link_name(self, roboy_link):
		"""Gets corresponding operator link's name to the given roboy link's name.
		
		Args:
			roboy_link (string): Roboy link's name.

		Returns:
		   	string: Corresponding operator link's name.

		"""
		operator_link = None
		for roboy, oper in zip(self.roboy_link_names, self.operator_link_names):
			if roboy == roboy_link:
				operator_link = oper
				break
		return operator_link

	def operator_initial_head_pose_callback(self, request, response, link_name="neck"):
		"""Callback for ROS service for initial head pose of the operator.

		Args:
			request: the GetLinkPose service request contains a link name,
					but we do not need it for this service.
			response: the response that would be sent back.
			link_name: name of the head link for the current operator.

		Returns:
			the response.


		"""
		self.get_logger().info("Service operator initial head pose: request received")

		head_pos, head_orn = self.get_initial_link_pose(link_name, self.operator_initial_link_pose_client)[:2]

		response.pose.position.x = head_pos[0]
		response.pose.position.y = head_pos[1]
		response.pose.position.z = head_pos[2]

		response.pose.orientation.x = head_orn[0]
		response.pose.orientation.y = head_orn[1]
		response.pose.orientation.z = head_orn[2]
		response.pose.orientation.w = head_orn[3]
		self.get_logger().debug("Responding")

		self.roboy_is_ready = True

		return response

	def operator_movement_listener(self, ef_pose):
		"""Callback function of the endeffector subscription. Processes the msg received and moves the link accordingly.

		Args:
			ef_pose: end effector pose received from the operator.

		"""
		self.get_logger().debug('Endeffector pose received: ' + ef_pose.ef_name)

		if not self.roboy_is_ready:
			#self.get_logger().info('Roboy simulation is not ready.')

			return

		#process message
		self.get_logger().debug('got frame-id' + ef_pose.ef_name)
		
		ef_pose.ef_name = self.get_roboy_link_name(ef_pose.ef_name)
		
		orn = [ef_pose.ef_pose.orientation.x, 
				ef_pose.ef_pose.orientation.y, 
				ef_pose.ef_pose.orientation.z, 
				ef_pose.ef_pose.orientation.w]
		
		link_orn = self.adapt_orientation_to_roboy(ef_pose.ef_name, orn)
		ef_pose.ef_pose.orientation.x = link_orn[0]
		ef_pose.ef_pose.orientation.y = link_orn[1]
		ef_pose.ef_pose.orientation.z = link_orn[2]
		ef_pose.ef_pose.orientation.w = link_orn[3]
		
		self.get_logger().debug('Publishing EF-Pose')

		if ef_pose.ef_name.find("right") != -1:
			self.right_ef_publisher.publish(ef_pose)
		else:
			self.left_ef_publisher.publish(ef_pose)
		  
	def call_service(self, client, msg):
		"""Calls a client synchnrnously passing a msg and returns the response

		Args:
			client (RosClient): The client used to communicate with the server
			msg (Object): A server msg to send to the server as a request

		Returns:
			response (Object): A response msg from the server

		"""
		while not client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info(f"{client.srv_name} service not available, waiting again...")
		response = client.call(msg)
		return response

	def collision_listener(self, msg):
		"""Collision subscriber handler.

		"""
		self.get_logger().info(f"Mapping collision: {msg.normalforce} N dir: {msg.contactnormal}")
		operator_collision = self.map_collision_to_operator(msg)
		if operator_collision is None:
			return
		self.get_logger().debug("publishing")
		self.exoforce_collision_publisher.publish(operator_collision)

	def map_collision_to_operator(self, roboy_collision):
		"""Maps roboy link id in collision to the operator corresponding link id.

		Args:
			roboy_collision (Collision): The roboy collision.

		Returns:
			Collision: The collision with mapped linkid.

		"""
		self.get_logger().debug('mapping start')
		roboy_link_info = self.get_roboy_link_info(roboy_collision.linkid)
		operator_link_name = self.get_operator_link_name(roboy_link_info.link_name)
		if operator_link_name is None:
			return None
		operator_link_info = self.get_operator_link_info(operator_link_name)
		self.get_logger().debug('responses')

		operator_collision = Collision()
		operator_collision = roboy_collision
		operator_collision.linkid = operator_link_info.link_id
		self.get_logger().debug('responses2')

		position_scale = self.roboy_to_operator_link_ratio(roboy_link_info.dimensions, operator_link_info.dimensions)
		operator_collision = self.scale_to_operator(operator_collision, position_scale)
		new_position = self.adapt_vector_to_orientation(roboy_link_info.link_name, [operator_collision.position.x, operator_collision.position.y, operator_collision.position.z])
		operator_collision.position.x = new_position[0]
		operator_collision.position.y = new_position[1]
		operator_collision.position.z = new_position[2]
		self.get_logger().debug("mapping done")

		new_contact_normal = self.adapt_vector_to_orientation(roboy_link_info.link_name, [roboy_collision.contactnormal.x, roboy_collision.contactnormal.y, roboy_collision.contactnormal.z])
		operator_collision.contactnormal.x = new_contact_normal[0]
		operator_collision.contactnormal.y = new_contact_normal[1]
		operator_collision.contactnormal.z = new_contact_normal[2]

		return operator_collision

	def get_roboy_link_info(self, roboy_link_id):
		"""Gets the roboy link name from roboy link id by calling the corresponding servicce synchronously.

		Args:
			roboy_link_id (uint8): The link id of roboy.

		Returns:
			LinkInfoFromId: The link info of the passed link id.

		"""
		self.get_logger().debug('Getting roboy link info')
		roboy_link_info_from_id_req = LinkInfoFromId.Request()
		roboy_link_info_from_id_req.link_id = roboy_link_id
		response = self.call_service(self.roboy_link_info_from_id_client, roboy_link_info_from_id_req)
		self.get_logger().debug("received response")
		return response

	def get_operator_link_info(self, operator_link_name):
		"""Gets the operator link id from the operator link name by calling the corresponding servicce synchronously.

		Args:
			operator_link_name (string): The link name of the operator.

		Returns:
			LinkInfoFromName: The link info of the passed link name.

		"""
		self.get_logger().debug('Getting operator link info')
		operator_link_info_from_id_req = LinkInfoFromName.Request()
		operator_link_info_from_id_req.link_name = operator_link_name
		response = self.call_service(self.operator_link_info_from_name_client, operator_link_info_from_id_req)
		self.get_logger().debug('Got operator link info')
		return response

	def roboy_to_operator_link_ratio(self, roboy_dimensions, operator_dimensions):
		"""Calculates the ratio between robot link and human link.

		Args:
			roboy_dimensions (Vector3): The bounding box of the link whose ratio needs to be calculated.
			operator_dimensions (Vector3): The bounding box of the link whose ratio is needed

		Returns:
			3darray(float): ratio on the three dimensions

		"""
		x_scale = roboy_dimensions.x / operator_dimensions.x
		y_scale = roboy_dimensions.y / operator_dimensions.y
		z_scale = roboy_dimensions.z / operator_dimensions.z

		return [x_scale, y_scale, z_scale]
	
	def scale_to_operator(self, collision, scale):
		"""Scales down the collision from robot to human.

		Args:
			collision (Collision):The collision that happened on the robot side.
			scale (3darray(float)): The position scale from roboy to operator

		Returns:
			Collision: Collision scaled to human

		"""
		collision.position.x = collision.position.x / scale[0]
		collision.position.y = collision.position.y / scale[1]
		collision.position.z = collision.position.z / scale[2]

		return collision

	def get_initial_link_pose(self, link_name, client):
		"""Gets initial link pose for a link using its name.

		Args:
			link_name (string): The name of the link.
			client (RosClient): The client used to pass a request and get a response from service.

		Returns:
			[3darray(float), 4darray(float)]: A array containing two vectors, first is position and second is orientation.

		"""
		self.get_logger().info('Getting initial ' + link_name + ' link pose...')
		initial_link_pose_req = GetLinkPose.Request()
		initial_link_pose_req.link_name = link_name
		response = self.call_service(client, initial_link_pose_req)
		self.get_logger().debug('service called')

		return [[response.pose.position.x, 
			response.pose.position.y, 
			response.pose.position.z], 
			[response.pose.orientation.x, 
			response.pose.orientation.y, 
			response.pose.orientation.z, 
			response.pose.orientation.w]]

	def adapt_orientation_to_roboy(self, roboy_link_name, orientation):
		"""Adapts the orientation received to roboy's link according to roboy's and operators initial links.

		Args:
			roboy_link_name (string): link name according to roboy's urfd.
			orientation (4darray(float)): received target orientation from operator.
		
		Returns:
			4darray(float): The adapted target orientation.

		"""
		
		diff = self.roboy_to_operator_orientation_diff(roboy_link_name)
		quat = diff * orientation 

		return [quat[0], quat[1], quat[2], quat[3]]

	def adapt_vector_to_orientation(self, roboy_link_name, vector):
		"""Adapts the vector's direction received to roboy's link according to roboy's and operators initial links.

		Args:
			roboy_link_name (string): link name according to roboy's urfd.
			vector (3darray(float)): received target orientation from operator.

		Returns:
			4darray(float): The adapted target orientation.

		"""

		diff = self.roboy_to_operator_orientation_diff(roboy_link_name)
		R = diff.rotation_matrix

		return R.dot(vector)

	def roboy_to_operator_orientation_diff(self, roboy_link_name):
		"""Calculates the orientation difference between the roboy link frame and the operator link frame

		Args:
			roboy_link_name (string): The name of the roboy link

		Returns:
			Quaternion: The difference of quaternions between roboy and operator links

		"""
		if self.roboy_initial_link_poses.get(roboy_link_name) == None :
			self.roboy_initial_link_poses[roboy_link_name] = self.get_initial_link_pose(roboy_link_name, self.roboy_initial_link_pose_client)
			self.get_logger().debug("Got roboy initial pose for link " + roboy_link_name + " : " + str(self.roboy_initial_link_poses[roboy_link_name]))
		roboy_init_orn = Quaternion(np.array(self.roboy_initial_link_poses[roboy_link_name][1]))
		operator_link_name = self.get_operator_link_name(roboy_link_name)
		if self.operator_initial_link_poses.get(operator_link_name) == None :
			self.operator_initial_link_poses[operator_link_name] = self.get_initial_link_pose(operator_link_name, self.operator_initial_link_pose_client)        
			self.get_logger().debug("Got operator initial pose for link " + operator_link_name + " : " + str(self.operator_initial_link_poses[operator_link_name]))
		if np.sum(self.operator_initial_link_poses[operator_link_name][0]) + np.sum(self.operator_initial_link_poses[operator_link_name][1]) == 0:
			op_init_orn = roboy_init_orn
		else:
			op_init_orn = Quaternion(np.array(self.operator_initial_link_poses[operator_link_name][1]))

		diff = roboy_init_orn / op_init_orn

		return diff
