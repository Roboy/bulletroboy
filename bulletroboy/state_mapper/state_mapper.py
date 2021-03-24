import numpy as np
from scipy.spatial.transform import Rotation as R

from pyquaternion import Quaternion

from rclpy.node import Node

from ..utils.utils import Topics, Services
from roboy_simulation_msgs.msg import Collision, ContactPoint
from roboy_middleware_msgs.msg import EFPose
from roboy_middleware_msgs.srv import InitStateMapper
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

		self.ready = False

		self.roboy_link_names = self.get_parameter("roboy_link_names").get_parameter_value().string_array_value
		self.operator_link_names = self.get_parameter("operator_link_names").get_parameter_value().string_array_value
		
		assert len(self.roboy_link_names) == len(self.operator_link_names), "Roboy and operator links list have different lenghts"

		self.roboy_link_info = []
		self.operator_link_info = []

		# Define subscriptions
		self.robot_collision_subscription = self.create_subscription(Collision,	Topics.ROBOY_COLLISIONS, self.collision_listener, 1)

		# Operator EF pose subscriber
		# self.operator_movement_subscription = self.create_subscription(EFPose, Topics.OP_EF_POSES, self.operator_movement_listener, 1)

		# Define publishers
		self.exoforce_collision_publisher = self.create_publisher(Collision, Topics.MAPPED_COLLISIONS, 1)
		# self.right_ef_publisher = self.create_publisher(EFPose, Topics.MAPPED_OP_REF_POSE, 1)
		# self.left_ef_publisher = self.create_publisher(EFPose, Topics.MAPPED_OP_LEF_POSE, 1)

		# Define service
		# self.operator_initial_head_pose = self.create_service(GetLinkPose, Services.INITIAL_HEAD_POSE, self.operator_initial_head_pose_callback)
		
		# Init service
		self.init_service = self.create_service(InitStateMapper, Services.INIT_STATE_MAPPER, self.init_callback)
	
	def init_callback(self, request, response):
		"""Callback for ROS service to init the state mapper.

		"""
		self.get_logger().info("Initializing...")
		err_msg = ""
		response.success = True

		self.get_logger().debug("Roboy link info:")
		self.roboy_link_info = self.get_link_info_dict(request.roboy_link_information)
		self.get_logger().debug("Operator link info:")
		self.operator_link_info = self.get_link_info_dict(request.operator_link_information)

		roboy_link_without_info = [link_name for link_name in self.roboy_link_names if self.get_roboy_link_info_name(link_name) is None]
		operator_link_without_info = [link_name for link_name in self.operator_link_names if self.get_operator_link_info_name(link_name) is None]

		if roboy_link_without_info:
			err_msg = f"The following roboy links have no info {roboy_link_without_info}"
		elif operator_link_without_info:
			err_msg = f"The following operator links have no info {operator_link_without_info}"

		if err_msg:
			self.get_logger().error(f"Cannot start the state mapper node [{err_msg}]")
			response.success = False
			response.message = err_msg
		else:
			self.get_logger().info("Successfully initialized.")
			self.ready = True

		return response

	def get_link_info_dict(self, link_info_list):
		"""Converts link info msg to dict.
		
		Args:
			link_info_list (LinkInformation[]): Link Information msg object.

		Returns:
		   	list: List of link information in dict format.

		"""
		links_info = []
		for link_info in link_info_list:
			link_id = link_info.id
			link_name = link_info.name
			link_dimensions = np.array([link_info.dimensions.x, link_info.dimensions.y, link_info.dimensions.z])
			link_init_pos = np.array([link_info.init_pose.position.x, link_info.init_pose.position.y, link_info.init_pose.position.z])
			link_init_orn = np.array([link_info.init_pose.orientation.x, link_info.init_pose.orientation.y, link_info.init_pose.orientation.z, link_info.init_pose.orientation.w])

			links_info.append({'id': link_id, 'name': link_name, 'dimensions': link_dimensions, 'init_pose': {'position': link_init_pos, 'orientation': link_init_orn}})

			self.get_logger().debug(f"Received link info for {link_name}")

		return links_info

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

	def get_roboy_link_info_id(self, roboy_link_id):
		"""Return roboy links info.

		Args:
			roboy_link_id (int): Roboy link id

		Returns:
			dict: Roboy link info.

		"""
		roboy_link_info = list(filter(lambda link_info: link_info['id'] == roboy_link_id, self.roboy_link_info))
		
		return None if len(roboy_link_info) == 0 else roboy_link_info[0]

	def get_roboy_link_info_name(self, roboy_link_name):
		"""Return roboy links info.

		Args:
			roboy_link_name (string): Roboy link name

		Returns:
			dict: Roboy link info.

		"""
		roboy_link_info = list(filter(lambda link_info: link_info['name'] == roboy_link_name, self.roboy_link_info))
		
		return None if len(roboy_link_info) == 0 else roboy_link_info[0]

	def get_operator_link_info_name(self, operator_link_name):
		"""Return operator links info.

		Args:
			operator_link_name (string): Operator link name

		Returns:
			dict: Operator link info.

		"""
		operator_link_info = list(filter(lambda link_info: link_info['name'] == operator_link_name, self.operator_link_info))
		
		return None if len(operator_link_info) == 0 else operator_link_info[0]

	def collision_listener(self, msg):
		"""Collision subscriber handler.

		"""

		if not self.ready:
			return
			
		self.get_logger().info(f"Mapping collision with {len(msg.contact_points)} contact points")

		op_contact_pts = []
		for roboy_contact_point in msg.contact_points:
			op_contact_point = self.map_contact_point_to_operator(roboy_contact_point)

			if op_contact_point is None:
				continue

			op_contact_pts.append(op_contact_point)

		if not op_contact_pts:
			return

		operator_collision = Collision()
		operator_collision.contact_points = op_contact_pts

		self.exoforce_collision_publisher.publish(operator_collision)

	def map_contact_point_to_operator(self, roboy_contact_pt):
		"""Maps roboy link id in contact_pt to the operator corresponding link id.

		Args:
			roboy_contact_pt (ContactPoint): The roboy contact_pt.

		Returns:
			ContactPoint: The contact_pt with mapped linkid.

		"""

		# Getting roboy link info
		roboy_link_info = self.get_roboy_link_info_id(roboy_contact_pt.linkid)
		if roboy_link_info is None:
			self.get_logger().error(f"No link info found for roboy link {roboy_contact_pt.linkid}")
			return None

		# Getting mapped operator link name
		operator_link_name = self.get_operator_link_name(roboy_link_info['name'])
		if operator_link_name is None:
			self.get_logger().error(f"No mapping found for roboy link {roboy_link_info['name']}")
			return None

		# Getting mapped operator link info
		operator_link_info = self.get_operator_link_info_name(operator_link_name)
		if operator_link_info is None:
			self.get_logger().error(f"No link info found for operator link {operator_link_name}")
			return None

		# Getting ratio between link sizes
		link_size_ratio = np.divide(roboy_link_info['dimensions'], operator_link_info['dimensions'])
		scaled_collision_position = np.divide(np.array([roboy_contact_pt.position.x, roboy_contact_pt.position.y, roboy_contact_pt.position.z]), link_size_ratio)

		# Transforming collision between link frames
		new_position = self.rotate_vector(roboy_link_info, operator_link_info, scaled_collision_position)
		new_contact_normal = self.rotate_vector(roboy_link_info, operator_link_info, np.array([roboy_contact_pt.contactnormal.x, roboy_contact_pt.contactnormal.y, roboy_contact_pt.contactnormal.z]))

		# Building operator collision msg
		operator_contact_pt = roboy_contact_pt
		operator_contact_pt.linkid = operator_link_info['id']

		operator_contact_pt.position.x = new_position[0]
		operator_contact_pt.position.y = new_position[1]
		operator_contact_pt.position.z = new_position[2]

		operator_contact_pt.contactnormal.x = new_contact_normal[0]
		operator_contact_pt.contactnormal.y = new_contact_normal[1]
		operator_contact_pt.contactnormal.z = new_contact_normal[2]

		return operator_contact_pt

	def rotate_vector(self, roboy_link_info, operator_link_info, vector):
		"""Adapts the vector's direction received to roboy's link according to roboy's and operators initial links.

		Args:
			roboy_link_info (dict): Roboy link information
			operator_link_info (dict): Operator link information
			vector (3darray(float)): Vector in roboy link frame.

		Returns:
			3darray(float): Rotated vector.

		"""
		diff = self.roboy_to_operator_orientation_diff(roboy_link_info, operator_link_info)
		R = diff.rotation_matrix

		return R.dot(vector)

	def roboy_to_operator_orientation_diff(self, roboy_link_info, operator_link_info):
		"""Calculates the orientation difference between the roboy link frame and the operator link frame

		Args:
			roboy_link_info (dict): Roboy link information
			operator_link_info (dict): Operator link information

		Returns:
			Quaternion: The difference of quaternions between roboy and operator links

		"""
		roboy_link_init_orn = Quaternion(roboy_link_info['init_pose']['orientation'])
		if np.sum(operator_link_info['init_pose']['orientation']) == 0:
			op_link_init_orn = roboy_link_init_orn
		else:
			op_link_init_orn = Quaternion(operator_link_info['init_pose']['orientation'])

		return roboy_link_init_orn / op_link_init_orn

	# def operator_initial_head_pose_callback(self, request, response):
	# 	"""Callback for ROS service for initial head pose of the operator.

	# 	Args:
	# 		request: the GetLinkPose service request contains a link name
	# 		response: the response that would be sent back.

	# 	Returns:
	# 		response

	# 	"""
	# 	self.get_logger().info("Service operator initial head pose: request received")

	# 	head_pos, head_orn = self.get_initial_link_pose(link_name, self.operator_initial_link_pose_client)[:2]

	# 	response.pose.position.x = head_pos[0]
	# 	response.pose.position.y = head_pos[1]
	# 	response.pose.position.z = head_pos[2]

	# 	response.pose.orientation.x = head_orn[0]
	# 	response.pose.orientation.y = head_orn[1]
	# 	response.pose.orientation.z = head_orn[2]
	# 	response.pose.orientation.w = head_orn[3]
	# 	self.get_logger().debug("Responding")

	# 	self.roboy_is_ready = True

	# 	return response

	# def operator_movement_listener(self, ef_pose):
	# 	"""Callback function of the endeffector subscription. Processes the msg received and moves the link accordingly.

	# 	Args:
	# 		ef_pose: end effector pose received from the operator.

	# 	"""
	# 	self.get_logger().debug('Endeffector pose received: ' + ef_pose.ef_name)

	# 	if not self.roboy_is_ready:
	# 		#self.get_logger().info('Roboy simulation is not ready.')

	# 		return

	# 	#process message
	# 	self.get_logger().debug('got frame-id' + ef_pose.ef_name)
		
	# 	ef_pose.ef_name = self.get_roboy_link_name(ef_pose.ef_name)
		
	# 	orn = [ef_pose.ef_pose.orientation.x, 
	# 			ef_pose.ef_pose.orientation.y, 
	# 			ef_pose.ef_pose.orientation.z, 
	# 			ef_pose.ef_pose.orientation.w]
		
	# 	link_orn = self.adapt_orientation_to_roboy(ef_pose.ef_name, orn)
	# 	ef_pose.ef_pose.orientation.x = link_orn[0]
	# 	ef_pose.ef_pose.orientation.y = link_orn[1]
	# 	ef_pose.ef_pose.orientation.z = link_orn[2]
	# 	ef_pose.ef_pose.orientation.w = link_orn[3]
		
	# 	self.get_logger().debug('Publishing EF-Pose')

	# 	if ef_pose.ef_name.find("right") != -1:
	# 		self.right_ef_publisher.publish(ef_pose)
	# 	else:
	# 		self.left_ef_publisher.publish(ef_pose)

	# def get_initial_link_pose(self, link_name, client):
	# 	"""Gets initial link pose for a link using its name.

	# 	Args:
	# 		link_name (string): The name of the link.
	# 		client (RosClient): The client used to pass a request and get a response from service.

	# 	Returns:
	# 		[3darray(float), 4darray(float)]: A array containing two vectors, first is position and second is orientation.

	# 	"""
	# 	self.get_logger().info('Getting initial ' + link_name + ' link pose...')
	# 	initial_link_pose_req = GetLinkPose.Request()
	# 	initial_link_pose_req.link_name = link_name
	# 	response = self.call_service(client, initial_link_pose_req)
	# 	self.get_logger().debug('service called')

	# 	return [[response.pose.position.x, 
	# 		response.pose.position.y, 
	# 		response.pose.position.z], 
	# 		[response.pose.orientation.x, 
	# 		response.pose.orientation.y, 
	# 		response.pose.orientation.z, 
	# 		response.pose.orientation.w]]

	# def adapt_orientation_to_roboy(self, roboy_link_name, orientation):
	# 	"""Adapts the orientation received to roboy's link according to roboy's and operators initial links.

	# 	Args:
	# 		roboy_link_name (string): link name according to roboy's urfd.
	# 		orientation (4darray(float)): received target orientation from operator.
		
	# 	Returns:
	# 		4darray(float): The adapted target orientation.

	# 	"""
		
	# 	diff = self.roboy_to_operator_orientation_diff(roboy_link_name)
	# 	quat = diff * orientation 

	# 	return [quat[0], quat[1], quat[2], quat[3]]
		  
	# def call_service(self, client, msg):
	# 	"""Calls a client synchnrnously passing a msg and returns the response

	# 	Args:
	# 		client (RosClient): The client used to communicate with the server
	# 		msg (Object): A server msg to send to the server as a request

	# 	Returns:
	# 		response (Object): A response msg from the server

	# 	"""
	# 	while not client.wait_for_service(timeout_sec=1.0):
	# 		self.get_logger().info(f"{client.srv_name} service not available, waiting again...")
	# 	response = client.call(msg)
	# 	return response
