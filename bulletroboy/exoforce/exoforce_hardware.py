import time
from functools import partial

import numpy as np
from pyquaternion import Quaternion
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

		self.declare_parameters(
			namespace='',
			parameters=[
				('no_slack_force', 4.0),
				('tendon_init_time', 5.0),
				('max_force_apply_time', 1.0)]
			)
		
		# state variables
		self.active = False
		self.processing_flag = False

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
		
		# Target force publisher
		self.target_force_publisher = self.create_publisher(TendonUpdate, Topics.TARGET_FORCE, 20)

		# Force Control node start/stop clients
		self.start_force_control_client = self.create_client(Trigger, Services.START_FORCE_CONTROL)
		self.stop_force_control_client = self.create_client(Trigger, Services.STOP_FORCE_CONTROL)

		# Init operator client
		self.init_operator_client = self.create_client(InitExoforce, Services.INIT_OPERATOR)

		# Start exoforce topics
		self.create_subscription(InitExoforceRequest, Topics.INIT_EXOFORCE_REQ, self.start_exoforce, 1)
		self.init_response_publisher = self.create_publisher(ExoforceResponse, Topics.INIT_EXOFORCE_RES, 1)

		# Stop exoforce topics
		self.create_subscription(Empty, Topics.STOP_EXOFORCE_REQ, self.stop_exoforce, 1)
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
		self.processing_flag = False

	def start_exoforce(self, init_msg):
		"""Starts exoforce.
		
		Args:
			init_msg (InitExoforceRequest): Initialization message.

		Returns:
			-

		"""
		if self.processing_flag:
			return

		self.processing_flag = True
		self.get_logger().info("Starting Exoforce...")

		for ef_name in init_msg.ef_name:
			if ef_name not in [ef.name for ef in self.end_effectors]:
				err_msg = f"{ef_name} is not a cage end effector!"
				self.get_logger().error(f"Cannot start the Exoforce: {err_msg}")
				self.send_exoforce_response(self.init_response_publisher, False, err_msg)
				return

		self.get_logger().info("Requesting Force Control node to start...")
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

				# Applying no slack force to all tendons
				self.set_target_force(self.no_slack_force)

				# Requesting operator node to start
				self.init_operator(init_msg)

				# Sending initialization response
				self.get_logger().info("Exoforce succesfully started.")
				self.send_exoforce_response(self.init_response_publisher, True)

				# Waiting for pulling slack time
				time.sleep(self.tendon_init_time)

				# Activating exoforce
				self.active = True

	def init_operator(self, init_msg):
		"""Inits operator node.
		
		Args:
			init_msg (InitExoforceRequest): Initialization message.

		Returns:
			-

		"""
		self.get_logger().info("Requesting operator node to initialize...")

		request = InitExoforce.Request(ef_name=init_msg.ef_name,
										ef_enabled=init_msg.ef_enabled,
										ef_init_pose=init_msg.ef_init_pose,
										operator_height=init_msg.operator_height,
										roboy_link_information=init_msg.roboy_link_information)

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

	def stop_exoforce(self, _):
		"""Stops exoforce.
		
		Args:
			-

		Returns:
			-

		"""
		if self.processing_flag:
			return
		
		self.get_logger().info("Stopping Exoforce...")
		self.processing_flag = True
		self.active = False
		self.apply_min_force_timer.cancel()
		self.set_target_force(0.0)
		self.get_logger().info("Stopping Force Control node...")
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
				err_msg = f"Failed to stop force control node: {result.message}"
				self.get_logger().error(f"Cannot stop Exoforce: {err_msg}")
				self.send_exoforce_response(self.stop_response_publisher, False, err_msg)
			else:
				self.get_logger().info("Force Control node stopped.")
				self.get_logger().info("Exoforce stopped.")
				self.send_exoforce_response(self.stop_response_publisher, True)

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

		self.get_logger().info(f"Received collision")

		contact_points = []
		for cp in collision_msg.contact_points:
			cage_direction = self.get_direction_in_cage_frame(cp.linkid, np.array([cp.contactnormal.x, cp.contactnormal.y, cp.contactnormal.z]))
			if cage_direction is None:
				return
			else:
				contact_points.append({'link_id': cp.linkid, 'force_vector': cp.normalforce * cage_direction})

		forces = self.decompose(contact_points)
		
		for muscle_id, force in forces.items():
			self.get_muscle_unit(muscle_id).set_motor_force(force)
			self.set_tendon_target_force(muscle_id, force)

		self.apply_min_force_timer.reset()

	def get_direction_in_cage_frame(self, link_id, cp_direction):
		# ef = self.map_link_to_ef(link_id)
		# if ef is None:
		# 	self.get_logger().warn(f"Link '{link_id}' has no mapping to ef!")
		# 	return
		# elif ef.orientation is None:
		# 	self.get_logger().warn(f"Orientation for ef '{ef.name}' is not initialized!")
		# 	return
		# quaternion = Quaternion(ef.orientation)
		# return quaternion.rotation_matrix.dot(cp_direction)
		return cp_direction

	def update(self):
		"""Updates ExoForce's state.
		
		Args:
			-

		Returns:
		   	-

		"""
		pass
