import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from roboy_simulation_msgs.msg import Collision, TendonUpdate

from ..utils.utils import Services, Topics
from .exoforce import ExoForce

# This constants need to be defined in a conf file
INIT_FORCE = 20	# in Newtons
MIN_FORCE = 5	# in Newtons
FORCE_APPLY_TIME = 1 # in seconds
INIT_TIME_WAIT = 3 # in seconds

class ExoforceHW(ExoForce):
	def __init__(self, cage_conf):
		"""
		Args:
			cage_conf (CageConfiguration): Cage configuration defined in the configuration file.
		
		"""
		super().__init__(cage_conf, "exoforce")
		self.callback_group = ReentrantCallbackGroup()

		self.active = False
		self.apply_min_force_timer = self.create_timer(FORCE_APPLY_TIME, lambda: self.set_target_force(MIN_FORCE))
		self.apply_min_force_timer.cancel()

		self.create_subscription(Collision, Topics.MAPPED_COLLISIONS, self.collision_listener, 1)
		self.create_subscription(Collision, 'roboy/simulation/roboy/collision_hw', self.collision_listener, 1)
		# self.create_subscription(PoseStamped, Topics.OP_EF_POSES, self.ef_pos_listener, 1)
		self.create_subscription(PoseStamped, Topics.VR_HEADSET_POSES, self.operator_ef_pos_listener, 10, callback_group=self.callback_group)
		
		self.target_force_publisher = self.create_publisher(TendonUpdate, Topics.TARGET_FORCE, 1)

		self.start_exoforce()
		self.stop_exoforce()

	def start_exoforce(self):
		"""Starts exoforce.
		
		Args:
			-

		Returns:
			-

		"""
		self.set_target_force(MIN_FORCE)
		time.sleep(INIT_TIME_WAIT)
		self.active = True

	def stop_exoforce(self):
		"""Stops exoforce.
		
		Args:
			-

		Returns:
			-

		"""
		self.active = False
		self.apply_min_force_timer.cancel()
		self.set_target_force(0)

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
		force_msg = TendonUpdate(tendon_id, float(force))
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
