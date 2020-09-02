import numpy as np
import time

from .exoforce import ExoForce
from .utils import load_roboy_to_human_link_name_map

from roboy_simulation_msgs.msg import Collision
from roboy_middleware_msgs.msg import MotorCommand, MotorState
from roboy_middleware_msgs.srv import ControlMode
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# POS_CONTROL_THRESHOLD = 0.1
TENSION_PWM_FAST = 500
TENSION_PWM_SLOW = 200

class ExoforceHW(ExoForce):
	def __init__(self, cage_conf):
		"""
		Args:
			cage_conf (CageConfiguration): Cage configuration defined in the configuration file.
		
		"""
		super().__init__(cage_conf, "exoforce")
		self.link_names_map = load_roboy_to_human_link_name_map()
		self.callback_group = ReentrantCallbackGroup()

		# self.create_subscription(Collision, '/roboy/simulation/exoforce/operator/collisions', self.collision_listener, 1)
		# self.create_subscription(PoseStamped, '/roboy/simulation/operator/pose/endeffector', self.ef_pos_listener, 1)
		self.create_subscription(PoseStamped, '/bullet_ik', self.operator_ef_pos_listener, 10, callback_group=self.callback_group)
		# self.create_subscription(PoseStamped, '/roboy/simulation/roboy/ef_pose', self.roboy_ef_pos_listener, 10, callback_group=self.callback_group)
		self.create_subscription(MotorState, '/roboy/middleware/MotorState', self.motor_state_listener, 1, callback_group=self.callback_group)
		
		self.motor_command_publisher = self.create_publisher(MotorCommand, '/roboy/middleware/MotorCommand', 1)

		self.control_mode_client = self.create_client(ControlMode, '/roboy/middleware/ControlMode')
		while not self.control_mode_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('roboy_plexus not available, waiting again...')
		self.get_logger().info('Succesfull connection to roboy plexus!')

		self.control_mode_req = ControlMode.Request()
		self.motor_command_msg = MotorCommand()

		self.set_control_mode(3)

		self.init_tendons()
		choice = input("\nDo you want to initialize motor position calibration? (Yes/no): ")
		if choice in ["Yes","yes", "y", ""]:
			self.calibration_timer = self.create_timer(2, self.calibration)

	def init_tendons(self):
		choice = input("\nDo you want to initialize the tendons? (Yes/no): ")
		if choice in ["Yes","yes", "y", ""]:
			print("\nInitiating tendons:")
			for muscle in self.muscle_units:
				tendon = muscle.id
				print("\n- Tendon "+ str(tendon) + ":")
				input("Press enter to start...")
				self.send_motor_commands([tendon], [float(TENSION_PWM_FAST * muscle.direction)])
				input("Press enter to slowdown...")
				self.send_motor_commands([tendon], [float(TENSION_PWM_SLOW * muscle.direction)])
				input("Press enter to stop...")
				self.send_motor_commands([tendon], [0.0])
			print("\nTendons ready!\n")

	def calibration(self):
		if np.any([p is None for p in self.end_effectors.values()]):
			print("Waiting for end effector initial positions...")
			return

		print("\nInitializing motor position calibration:")
		for muscle in self.muscle_units:
			input("- Touch attachment point " + str(muscle.id) + " with the right hand tracker and press enter...")
			muscle.motor.via_point.world_point = self.end_effectors["right_wrist"]
			print("Motor " + str(muscle.id) + " position updated to: " + str(muscle.motor.via_point.world_point))

		print("\nMotor positions updated!\n")
		self.calibration_timer.destroy()

		#self.create_timer(0.1, self.print_distance)

	def print_distance(self):
		for muscle in self.muscle_units:
			dist = np.linalg.norm(muscle.motor.via_point.world_point - self.end_effectors["right_wrist"])
			if dist < 0.1:
				self.get_logger().info("Motor " + str(muscle.id) + " touched!")
			if muscle.id == 3:
				print(dist)

	# def pos_control(self):
	# 	for o_ef, r_ef in zip(self.end_effectors, self.roboy_end_effectors):
	# 		diff = np.absolute(o_ef - r_ef)
	# 		print(diff)
	# 		print(diff < POS_CONTROL_THRESHOLD)

	def motor_state_listener(self, state):
		for muscle, displacement in zip(self.muscle_units, state.displacement[:16]):
			if muscle.displacement is not None:
				muscle.speed = displacement - muscle.displacement
			muscle.displacement = displacement

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

	def operator_ef_pos_listener(self, ef_pose):
		"""Callback of the pose subscriber. Sets the pose of the link given in the msg.
		Args:
			ef_pose: received PoseStamped msg.
		
		Returns:
			-
		"""
		end_effector = ef_pose.header.frame_id
		################### TEMPORAL MAPPING ############
		if end_effector == "hand_right":
			end_effector = "right_wrist"
		elif end_effector == "hand_left":
			end_effector = "left_wrist"
		else:
			return
		############################################
		#self.get_logger().info("Received pose for " + end_effector)
		if end_effector not in self.end_effectors:
			self.get_logger().warn(end_effector + " is not an end effector!")
			return

		muscles = self.get_ef_muscle_units(end_effector)

		link_pos = np.array([ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z])
		for muscle in muscles:
			muscle.end_effector.world_point = link_pos

		if self.end_effectors[end_effector] is None:
			self.get_logger().info("Got ef initial pos: " + end_effector)
		self.end_effectors[end_effector] = link_pos

	def roboy_ef_pos_listener(self, ef_pose):
		if ef_pose.header.frame_id in self.link_names_map:
			oper_link = self.link_names_map[ef_pose.header.frame_id]
			pos = np.array([ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z])
			self.roboy_end_effectors[oper_link] = pos

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

	def set_control_mode(self, mode, motor_ids=None, set_points=[]):
		self.control_mode_req.legacy = False
		self.control_mode_req.control_mode = mode
		self.control_mode_req.motor_id = motor_ids if motor_ids is not None else [muscle.id for muscle in self.muscle_units]
		self.control_mode_req.set_points = set_points if motor_ids is not None else [0] * len(self.control_mode_req.motor_id)

		self.future = self.control_mode_client.call_async(self.control_mode_req)

	def send_motor_commands(self, motor_ids=None, set_points=None):
		if motor_ids is None:
			motor_ids = [muscle.id for muscle in self.muscle_units]
		if set_points is None:
			set_points = [0.0] * len(motor_ids)

		self.motor_command_msg.legacy = False
		self.motor_command_msg.motor = motor_ids
		self.motor_command_msg.setpoint = set_points

		self.motor_command_publisher.publish(self.motor_command_msg)

		

import os
import rclpy
from bulletroboy.exoforce import CageConfiguration
CONFIG_DEFAULT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "../config/realCageConfiguration.xml"

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
