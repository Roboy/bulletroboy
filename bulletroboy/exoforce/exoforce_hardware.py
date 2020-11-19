import numpy as np
from pyquaternion import Quaternion

from .exoforce import ExoForce
from ..utils.force_decomposition import decompose_force_link_to_ef
from ..utils.utils import load_roboy_to_human_link_name_map, Topics, Services

from roboy_simulation_msgs.msg import Collision
from roboy_middleware_msgs.msg import MotorCommand, MotorState
from roboy_middleware_msgs.srv import ControlMode
from geometry_msgs.msg import PoseStamped

from rclpy.callback_groups import ReentrantCallbackGroup

FAST_PWM_TENSION = 300
MIN_PWM_TENSION = 150
YES_OPTIONS = ["Yes","yes","y",""]

POS_CONTROL_THRESHOLD = 0.1

class ExoforceHW(ExoForce):
	def __init__(self, cage_conf):
		"""
		Args:
			cage_conf (CageConfiguration): Cage configuration defined in the configuration file.
		
		"""
		super().__init__(cage_conf, "exoforce", POS_CONTROL_THRESHOLD)
		self.link_names_map = load_roboy_to_human_link_name_map()
		self.callback_group = ReentrantCallbackGroup()

		self.create_subscription(Collision, Topics.MAPPED_COLLISIONS, self.collision_listener, 1)
		self.create_subscription(Collision, 'roboy/simulation/roboy/collision_hw', self.collision_listener, 1)
		# self.create_subscription(PoseStamped, Topics.OP_EF_POSES, self.ef_pos_listener, 1)
		self.create_subscription(PoseStamped, Topics.VR_HEADSET_POSES, self.operator_ef_pos_listener, 10, callback_group=self.callback_group)
		
		self.roboy_plexus_ready = False
		choice = input("Connect to roboy plexus? (Yes/no): ")
		if choice in YES_OPTIONS:
			self.create_subscription(MotorState, Topics.MOTOR_STATE, self.motor_state_listener, 1, callback_group=self.callback_group)
			self.motor_command_publisher = self.create_publisher(MotorCommand, Topics.MOTOR_COMMAND, 1)

			self.control_mode_client = self.create_client(ControlMode, Services.CONTROL_MODE)
			while not self.control_mode_client.wait_for_service(timeout_sec=1.0):
				self.get_logger().info('roboy_plexus not available, waiting again...')
			self.get_logger().info('Succesfull connection to roboy plexus!')
			self.roboy_plexus_ready = True

			self.control_mode_req = ControlMode.Request()
			self.motor_command_msg = MotorCommand()

			self.set_control_mode(3)

			self.init_tendons()

		choice = input("\nCalibrate motor positions? (Yes/no): ")
		if choice in YES_OPTIONS:
			self.calibration_timer = self.create_timer(2, self.calibration)
		else:
			self.start_exoforce()

		self.applying_force = False
		self.applying_force_timer = self.create_timer(1, self.stop_applying_force)
		self.applying_force_timer.cancel()

	def stop_applying_force(self):
		self.applying_force = False

	def start_exoforce(self):
		self.create_timer(0.5, self.update)
		choice = input("\nInitialize position control? (Yes/no): ")
		if choice in YES_OPTIONS:
			self.init_pos_control()

	def init_tendons(self):
		choice = input("\nInitialize tendons? (Yes/no): ")
		if choice in YES_OPTIONS:
			for ef in self.end_effectors:
				input("\n- Press enter to initialize " + ef + " tendons...")
				motor_ids = []
				set_points = []
				for muscle in self.get_ef_muscle_units(ef):
					motor_ids.append(muscle.id)
					set_points.append(float(FAST_PWM_TENSION * muscle.direction))
				self.send_motor_commands(motor_ids, set_points)
				input("- Press enter to finish")
				set_points = []
				for muscle in self.get_ef_muscle_units(ef):
					set_points.append(float(MIN_PWM_TENSION * muscle.direction))
				self.send_motor_commands(motor_ids, set_points)
			self.send_motor_commands([16],[float(MIN_PWM_TENSION)])
			print("\nTendons ready!\n")
	
	# def init_tendons(self):
	# 	choice = input("\nInitialize tendons? (Yes/no): ")
	# 	if choice in YES_OPTIONS:
	# 		print("\nInitiating tendons:")
	# 		for muscle in self.muscle_units:
	# 			tendon = muscle.id
	# 			print("\n- Tendon "+ str(tendon) + ":")
	# 			input("Press enter to start...")
	# 			self.send_motor_commands([tendon], [float(FAST_PWM_TENSION * muscle.direction)])
	# 			input("Press enter to slowdown...")
	# 			self.send_motor_commands([tendon], [float(MIN_PWM_TENSION * muscle.direction)])
	# 			# input("Press enter to stop...")
	# 			# self.send_motor_commands([tendon], [0.0])
	# 		print("\nTendons ready!\n")

	def calibration(self):
		print(self.end_effectors.values())
		if np.any([p is None for p in self.end_effectors.values()]):
			print("Waiting for end effector initial poses...")
			return

		print("\nInitializing motor position calibration:")
		for muscle in self.muscle_units:
			input("- Touch attachment point " + str(muscle.id) + " with the right hand tracker and press enter...")
			muscle.motor.via_point.world_point = self.end_effectors["right_wrist"]["position"]
			print("Motor " + str(muscle.id) + " position updated to: " + str(muscle.motor.via_point.world_point))

		print("\nMotor positions updated!\n")
		self.start_exoforce()
		self.calibration_timer.destroy()


	def print_distance(self):
		for muscle in self.muscle_units:
			dist = np.linalg.norm(muscle.motor.via_point.world_point - self.end_effectors["right_wrist"]["position"])
			if dist < 0.1:
				self.get_logger().info("Motor " + str(muscle.id) + " touched!")
			if muscle.id == 3:
				print(dist)

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

		ef = decompose_force_link_to_ef(collision_msg.linkid)
		############ TEMP WORKAROUND
		# for muscle in self.get_ef_muscle_units(ef):
		# 	muscle.end_effector.world_point = np.array([0.5, -0.5, -0.3])
		##########################################################################
		collision_direction = np.array([collision_msg.contactnormal.x, collision_msg.contactnormal.y, collision_msg.contactnormal.z])

		quaternion = Quaternion(self.end_effectors[ef]["orientation"])
		force_direction = quaternion.rotation_matrix.dot(collision_direction)

		forces = self.decompose(collision_msg.linkid, collision_msg.normalforce, force_direction)
		print(forces)
		
		for muscle in forces:
			self.get_muscle_unit(muscle).set_motor_force(forces[muscle])

		motor_ids = []
		set_points = []
		for muscle in self.get_ef_muscle_units(ef):
			motor_ids.append(muscle.id)
			if forces[muscle.id] == 0:
				#set_points.append(float(MIN_PWM_TENSION * muscle.direction * -1))
				set_points.append(0.0)
			else:
				set_points.append(self.get_set_point(forces[muscle.id]) * muscle.direction)

		for id, points in zip(motor_ids, set_points):
			self.get_logger().info("motor " + str(id) + ": " + str(points))

		self.send_motor_commands(motor_ids, set_points)
		self.applying_force = True
		self.applying_force_timer.reset()

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
		link_orn = np.array([ef_pose.pose.orientation.x, ef_pose.pose.orientation.y, ef_pose.pose.orientation.z, ef_pose.pose.orientation.w])
		for muscle in muscles:
			muscle.end_effector.world_point = link_pos

		if self.end_effectors[end_effector] is None:
			self.get_logger().info("Got ef initial pose: " + end_effector)
			self.end_effectors[end_effector] = {"position": None, "orientation": None}
		self.end_effectors[end_effector]["position"] = link_pos
		self.end_effectors[end_effector]["orientation"] = link_orn

	def update(self):
		"""Updates ExoForce's state.
		
		Args:
			-

		Returns:
		   	-

		"""
		if self.applying_force:
			return
		motor_ids = []
		set_points = []
		for muscle in self.muscle_units:
			motor_ids.append(muscle.id)
			speed = 0 if muscle.speed is None else muscle.speed
			if speed * muscle.direction < 0:
				set_point = 0.0
			else:
				set_point = float(MIN_PWM_TENSION * muscle.direction)
			set_points.append(set_point)
		self.send_motor_commands(motor_ids, set_points)

	def get_set_point(self, force):
		return float(force * 20 + 200)

	def set_control_mode(self, mode, motor_ids=None, set_points=[]):
		if not self.roboy_plexus_ready:
			return
		self.control_mode_req.legacy = False
		self.control_mode_req.control_mode = mode
		self.control_mode_req.motor_id = motor_ids if motor_ids is not None else [muscle.id for muscle in self.muscle_units]
		self.control_mode_req.motor_id.append(16)
		self.control_mode_req.set_points = set_points if motor_ids is not None else [0] * len(self.control_mode_req.motor_id)
		self.control_mode_req.set_points.append(0)

		self.future = self.control_mode_client.call_async(self.control_mode_req)

	def send_motor_commands(self, motor_ids=None, set_points=None):
		if not self.roboy_plexus_ready:
			return
		if motor_ids is None:
			motor_ids = [muscle.id for muscle in self.muscle_units]
		if set_points is None:
			set_points = [0.0] * len(motor_ids)

		self.motor_command_msg.legacy = False
		self.motor_command_msg.motor = motor_ids
		self.motor_command_msg.setpoint = set_points

		self.motor_command_publisher.publish(self.motor_command_msg)
