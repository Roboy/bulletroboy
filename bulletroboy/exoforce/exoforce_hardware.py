import time
from functools import partial

import numpy as np
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from roboy_middleware_msgs.msg import ExoforceResponse, InitExoforceRequest
from roboy_middleware_msgs.srv import InitExoforce
from roboy_simulation_msgs.msg import Collision, TendonUpdate
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from ..utils.utils import Services, Topics, call_service_async
from .exoforce import ExoForce


class ExoforceHW(ExoForce):
	def __init__(self, cage_conf):
		"""
		Args:
			cage_conf (CageConfiguration): Cage configuration defined in the configuration file.
		
		"""
		super().__init__(cage_conf, "exoforce")
		self.callback_group = ReentrantCallbackGroup()

		self.declare_parameters(
			namespace='',
			parameters=[
				('no_slack_force', 4.0),
				('tendon_init_time', 5.0),
				('max_force_apply_time', 1.0)]
			)
		
		# state variables
		self.active = False
		self.starting = False

		# Initial parameters
		self.no_slack_force = self.get_parameter("no_slack_force").get_parameter_value().double_value
		self.tendon_init_time = self.get_parameter("tendon_init_time").get_parameter_value().double_value
		self.max_force_apply_time = self.get_parameter("max_force_apply_time").get_parameter_value().double_value

		# Force timer
		self.apply_min_force_timer = self.create_timer(self.max_force_apply_time, lambda: self.set_target_force(self.no_slack_force))
		self.apply_min_force_timer.cancel()

		# Collision subscription
		self.create_subscription(Collision, Topics.MAPPED_COLLISIONS, self.collision_listener, 1)
		self.create_subscription(Collision, 'roboy/simulation/roboy/collision_hw', self.collision_listener, 1)

		# Operator EF pose subscription
		# self.create_subscription(PoseStamped, Topics.OP_EF_POSES, self.ef_pos_listener, 1)
		self.create_subscription(PoseStamped, Topics.VR_HEADSET_POSES, self.operator_ef_pos_listener, 10, callback_group=self.callback_group)
		
		# Target force publisher
		self.target_force_publisher = self.create_publisher(TendonUpdate, Topics.TARGET_FORCE, 1)

		# Force Control node start/stop clients
		self.start_force_control_client = self.create_client(Trigger, Services.START_FORCE_CONTROL)
		self.stop_force_control_client = self.create_client(Trigger, Services.STOP_FORCE_CONTROL)

		# Init operator client
		self.init_operator_client = self.create_client(InitExoforce, Services.INIT_OPERATOR)

		# Start exoforce topics
		self.create_subscription(InitExoforceRequest, Topics.INIT_EXOFORCE_REQ, self.start_exoforce, 1)
		self.init_response_publisher = self.create_publisher(ExoforceResponse, Topics.INIT_EXOFORCE_RES, 1)

		# Stop exoforce topics
		self.create_subscription(Empty, Topics.STOP_EXOFORCE_REQ, self.start_exoforce, 1)
		self.stop_response_publisher = self.create_publisher(ExoforceResponse, Topics.STOP_EXOFORCE_RES, 1)

	def send_exoforce_response(self, publisher, success, message=""):
		"""Sends ExoforceResponse msg through response topic.
		
		Args:
			publisher (Publisher): Response publisher.
			success (bool): Success msg data.
			message (string): Message msg data.

		Returns:
			-

		"""
		msg = ExoforceResponse()
		msg.success = success
		msg.message = message
		publisher.publish(msg)
		self.starting = False

	def start_exoforce(self, init_msg):
		"""Starts exoforce.
		
		Args:
			init_msg (InitExoforceRequest): Initialization message.

		Returns:
			-

		"""
		if self.starting:
			return
		self.starting = True
		self.get_logger().info("Starting Exoforce...")

		for ef_name in init_msg.ef_name:
			if ef_name not in [ef.name for ef in self.end_effectors]:
				err_msg = f"{ef_name} is not a cage end effector!"
				self.get_logger().error(f"Cannot start the Exoforce: {err_msg}")
				self.send_exoforce_response(self.init_response_publisher, False, err_msg)
				return

		self.get_logger().info("Starting Force Control node...")
		call_service_async(self.start_force_control_client, Trigger.Request(), partial(self.start_exoforce_callback, init_msg), self.get_logger())

	def start_exoforce_callback(self, init_msg, future):
		"""Start exoforce callback.
		
		Args:
			init_msg (InitExoforceRequest): Initialization message.
			future: Service future var.

		Returns:
			-

		"""
		try:
			result = future.result()
		except Exception as e:
			self.get_logger().error(f"{self.start_force_control_client.srv_name} service call failed {e}")
		else:
			if not result.success:
				err_msg = f"Failed to start force control node: {result.message}"
				self.get_logger().error(f"Failed to start Exoforce: {err_msg}")
				self.send_exoforce_response(self.init_response_publisher, False, err_msg)
			else:
				self.get_logger().info("Force Control node started.")
				self.set_target_force(self.no_slack_force)
				time.sleep(self.tendon_init_time)
				self.active = True
				self.get_logger().info("Exoforce succesfully started.")

				self.init_operator(init_msg)

				self.send_exoforce_response(self.init_response_publisher, True)

	def init_operator(self, init_msg):
		"""Inits operator node.
		
		Args:
			init_msg (InitExoforceRequest): Initialization message.

		Returns:
			-

		"""
		self.get_logger().info("Initializing operator node...")

		request = InitExoforce.Request()
		request.ef_name = init_msg.ef_name
		request.ef_enabled = init_msg.ef_enabled
		request.ef_init_pose = init_msg.ef_init_pose

		call_service_async(self.init_operator_client, request, self.init_operator_callback, self.get_logger())

	def init_operator_callback(self, future):
		"""Init operator callback.
		
		Args:
			future: Service future var.

		Returns:
			-

		"""
		try:
			result = future.result()
		except Exception as e:
			self.get_logger().error(f"{self.init_operator_client.srv_name} service call failed {e}")
		else:
			if result.success:
				self.get_logger().info("Operator node succesfully initialized.")
			else:
				self.get_logger().error(f"Operator node could not be initialized: {result.message}")

	def stop_exoforce(self):
		"""Stops exoforce.
		
		Args:
			-

		Returns:
			-

		"""
		self.active = False
		self.apply_min_force_timer.cancel()
		call_service_async(self.stop_force_control_client, Trigger.Request(), self.stop_force_control_callback, self.get_logger())

	def stop_force_control_callback(self, future):
		"""Requests force control node to stop.
		
		Args:
			-

		Returns:
			bool success

		"""
		try:
			result = future.result()
		except Exception as e:
			self.get_logger().error(f"{self.stop_force_control_client.srv_name} service call failed {e}")
		else:
			if not result.success:
				self.get_logger().error(f"Failed to stop force control node: {result.message}")
			else:
				self.get_logger().info("Force Control node stopped.")

	def set_target_force(self, force):
		"""Sets a target force to all the tendons.
		
		Args:
			force (float): Target force to be set in Newtons.

		Returns:
			-

		"""
		for muscle in self.muscle_units:
			self.set_tendon_target_force(muscle.id, force)

	def set_tendon_target_force(self, tendon_id, force):
		"""Set a tendon target force.
		
		Args:
			tendon_id (int): Id of the tendon.
			force (float): Target force to be set in Newtons.

		Returns:
			-

		"""
		force_msg = TendonUpdate()
		force_msg.tendon_id = tendon_id
		force_msg.force = float(force)
		self.target_force_publisher.publish(force_msg)
	
	def collision_listener(self, collision_msg):
		"""Collision listener.

		Args:
			collision_msg (Collision): Collision message.

		Returns:
			-
		
		"""
		if not self.active:
			return
		
		self.get_logger().info(f"Received collision: link: {collision_msg.linkid} force: {collision_msg.normalforce}")

		ef = self.map_link_to_ef(collision_msg.linkid)
		if ef.orientation is None:
			self.get_logger().warn(f"Orientation for ef '{ef.name}' is not initialized!")
			return

		collision_direction = np.array([collision_msg.contactnormal.x, collision_msg.contactnormal.y, collision_msg.contactnormal.z])
		quaternion = Quaternion(ef.orientation)
		
		force_direction = quaternion.rotation_matrix.dot(collision_direction)

		forces = self.decompose(collision_msg.linkid, collision_msg.normalforce, force_direction)
		
		for muscle_id, force in forces.items():
			self.get_muscle_unit(muscle_id).set_motor_force(force)
			self.set_tendon_target_force(muscle_id, force)

		self.apply_min_force_timer.reset()

	def operator_ef_pos_listener(self, ef_pose):
		"""Callback of the pose subscriber. Sets the pose of the end effector given in the msg.

		Args:
			ef_pose: received PoseStamped msg.
		
		Returns:
			-

		"""
		ef_name = ef_pose.header.frame_id
		################### TEMPORAL MAPPING ############
		if ef_name == "hand_right":
			ef_name = "right_wrist"
		elif ef_name == "hand_left":
			ef_name = "left_wrist"
		else:
			return
		############################################
		#self.get_logger().info("Received pose for " + ef_name)
		end_effector = self.get_ef_name(ef_name)
		if end_effector is None:
			self.get_logger().warn(ef_name + " is not an end effector!")
			return

		link_pos = np.array([ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z])
		link_orn = np.array([ef_pose.pose.orientation.x, ef_pose.pose.orientation.y, ef_pose.pose.orientation.z, ef_pose.pose.orientation.w])

		end_effector.position = link_pos
		end_effector.orientation = link_orn

	def update(self):
		"""Updates ExoForce's state.
		
		Args:
			-

		Returns:
		   	-

		"""
		pass
