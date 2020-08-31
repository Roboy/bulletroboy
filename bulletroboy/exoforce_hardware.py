import numpy as np

from .exoforce import ExoForce

from roboy_simulation_msgs.msg import Collision
from roboy_middleware_msgs.msg import MotorCommand, MotorState
from roboy_middleware_msgs.srv import ControlMode
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class ExoforceHW(ExoForce):
	def __init__(self, cage_conf):
		"""
		Args:
			cage_conf (CageConfiguration): Cage configuration defined in the configuration file.
		
		"""
		super().__init__(cage_conf, "exoforce")
		self.callback_group = ReentrantCallbackGroup()

		self.create_subscription(Collision, '/roboy/simulation/exoforce/operator/collisions', self.collision_listener, 1)
		# self.create_subscription(PoseStamped, '/roboy/simulation/operator/pose/endeffector', self.ef_pos_listener, 1)
		self.create_subscription(PoseStamped, '/bullet_ik', self.ef_pos_listener, 1, callback_group=self.callback_group)
		self.create_subscription(MotorState, '/roboy/middleware/MotorState', self.motor_state_listener, 1)
		
		self.motor_command_publisher = self.create_publisher(MotorCommand, '/roboy/middleware/MotorCommand', 1)

		self.control_mode_client = self.create_client(ControlMode, '/roboy/middleware/ControlMode')
		# while not self.control_mode_client.wait_for_service(timeout_sec=1.0):
		# 	self.get_logger().info('roboy_plexus not available, waiting again...')
		self.get_logger().info('Succesfull connection to roboy plexus!')

		self.control_mode_req = ControlMode.Request()
		self.motor_command_msg = MotorCommand()

		self.calibrated = False

	def calibration(self):
		wp = [ef.world_point for ef in self.end_effectors]
		if np.any([p is None for p in wp]):
			return
		self.calibrated = True

		self.calibration_pos = np.array([0,0,0])
		print("\nInitializing VR trackers calibration:")
		input("- Touch motor 1 with the right hand and press enter...")
		print(str(self.get_ef("right_wrist").world_point))
		diff1 = self.get_ef("right_wrist").world_point - self.get_muscle_unit(1).motor.via_point.world_point
		input("- Touch motor 2 with the right hand and press enter...")
		print(str(self.get_ef("right_wrist").world_point))
		diff2 = self.get_ef("right_wrist").world_point - self.get_muscle_unit(2).motor.via_point.world_point
		input("- Touch motor 3 with the right hand and press enter...")
		print(str(self.get_ef("right_wrist").world_point))
		diff3 = self.get_ef("right_wrist").world_point - self.get_muscle_unit(3).motor.via_point.world_point

		self.calibration_pos = (diff1 + diff2 + diff3) / 3

		print("\nCalibration done!\n")

		self.get_logger().info("Calibration value: " + str(self.calibration_pos))

	def set_control_mode(self, mode, motor_ids=None, set_points=[]):
		self.control_mode_req.legacy = False
		self.control_mode_req.control_mode = mode
		self.control_mode_req.set_points = set_points
		self.control_mode_req.motor_id = motor_ids if motor_ids is not None else [muscle.id for muscle in self.muscle_units]

		self.future = self.control_mode_client.call_async(self.control_mode_req)

	def motor_state_listener(self, state):
		#print(state)
		pass
	
	def collision_listener(self, collision_msg):
		"""Collision listener.

		Args:
			collision_msg (Collision): Collision message.

		Returns:
			-
		
		"""
		self.get_logger().info(f"Received collision: link: {collision_msg.linkid} force: {collision_msg.normalforce}")
		# collision_direction = np.array([collision_msg.contactnormal.x, collision_msg.contactnormal.y, collision_msg.contactnormal.z])
			
		# _, rotation = p.getLinkState(self.operator.body_id, collision_msg.linkid)[:2]
		# rotation = np.array(p.getMatrixFromQuaternion(rotation)).reshape(3,3)
		
		# force_direction = rotation.dot(collision_direction)
		# forces = self.decompose(collision_msg.linkid, collision_msg.normalforce, force_direction)
		
		# for tendon in self.sim_tendons:
		# 	if tendon.tendon.id in forces:
		# 		force = forces[tendon.tendon.id]
		# 	else:
		# 		force = 0
		# 	self.update_tendon(tendon.tendon.id, force)

	def ef_pos_listener(self, ef_pose):
		"""Callback of the pose subscriber. Sets the pose of the link given in the msg.
		Args:
			ef_pose: received PoseStamped msg.
		
		Returns:
			-
		"""
		end_effector = ef_pose.header.frame_id
		################### TEST MAPPING ############
		if end_effector == "hand_right":
			end_effector = "right_wrist"
		elif end_effector == "hand_left":
			end_effector = "left_wrist"
		else:
			return
		############################################
		# self.get_logger().info("Received pose for " + end_effector)
		muscles = self.get_ef_muscle_units(end_effector)
		if len(muscles) == 0:
			self.get_logger().warn(end_effector + " is not an end effector!")
			return

		link_pos = np.array([ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z])
		for i, muscle in enumerate(muscles):
			if i == 0 and muscle.end_effector.world_point is None:
				self.get_logger().info("Got ef initial pos: " + muscle.end_effector.link)
			muscle.end_effector.world_point = link_pos

		if not self.calibrated:
			self.calibration()

	def update(self):
		"""Updates ExoForce's state.
		
		Args:
			-

		Returns:
		   	-

		"""	
		pass

	def update_tendon(self, id, force):
		"""Applies a force to a tendon.
		
		Args:
			id (int): Id of the tendon to update.
			force (float): Force applied to the tendon.

		Returns:
		   	-

		"""
		self.get_muscle_unit(id).set_motor_force(force)
		
	def send_motor_commands(self, motor_ids=None, set_points=None):
		if motor_ids is None:
			motor_ids = [muscle.id for muscle in self.muscle_units]
		if set_points is None:
			set_points = [0] * len(motor_ids)

		self.motor_command_msg.legacy = False
		self.motor_command_msg.motor = motor_ids
		self.motor_command_msg.setpoint = set_points

		self.motor_command_publisher.publish(self.motor_command_msg)

		

import os
import rclpy
from bulletroboy.exoforce import CageConfiguration
CONFIG_DEFAULT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "../config/cageConfiguration.xml"

def main(args=None):
	rclpy.init(args=args)

	# EXOFORCE SETUP
	initial_cage_conf = CageConfiguration(CONFIG_DEFAULT_PATH)
	exoforce = ExoforceHW(initial_cage_conf)
	executor = MultiThreadedExecutor()
	rclpy.spin(exoforce, executor)

	exoforce.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
