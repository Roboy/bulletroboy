import pybullet as p
import numpy as np
from numpy.linalg import norm

from roboy_simulation_msgs.msg import Collision
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from .exoforce import ExoForce
from ..operator.operator_simulation import Moves

from ..utils.utils import load_roboy_to_human_link_name_map

POS_CONTROL_THRESHOLD = 0.2

class ExoForceSim(ExoForce):
	"""ExoForce Child class. This class handles the simulation of the exoforce.

	"""
	def __init__(self, cage_conf, operator, mode):
		"""
		Args:
			cage_conf (CageConfiguration): Cage configuration defined in the configuration file.
			human_model (int): Id of the human model loaded in pybullet.
			mode (string): Mode in which the ExoForceSim will we executed.
		
		"""
		super().__init__(cage_conf, "exoforce_simulation", POS_CONTROL_THRESHOLD)
	
		self.link_names_map = load_roboy_to_human_link_name_map()
		self.mode = mode
		self.operator = operator
		self.init_sim()

		self.init_movement_params()

		if self.mode == "debug":
			self.init_debug_parameters()
		elif self.mode in ["tendon", "forces"]:
			self.create_subscription(Collision, '/roboy/simulation/exoforce/operator/collisions', self.collision_listener, 1)
			self.create_subscription(Float32, '/roboy/simulation/cage_rotation', self.cage_rotation_listener, 1)
		else:
			raise Exception(f"Mode [{mode}] not supported!")

		self.create_subscription(PoseStamped, '/roboy/simulation/operator/pose/endeffector', self.operator_ef_pos_listener, 1)
		
	
	def init_movement_params(self):
		'''Initializes movement buttons of the GUI.
		'''
		self.current_move = Moves.STAND_STILL
		self.stand = 0
		self.stand_id = p.addUserDebugParameter("Stand still", 1, 0, 0)
		self.arm_roll = 0 
		self.arm_roll_id = p.addUserDebugParameter("Arm roll", 1, 0, 0)
		self.catch = 0 
		self.catch_id = p.addUserDebugParameter("Catch", 1, 0, 0)
		self.hands_up = 0 
		self.hands_up_id = p.addUserDebugParameter("Hands up", 1, 0, 0)

	def init_sim(self):
		"""Initializes simulation.
		
		Args:
			-

		Returns:
		   -

		"""
		self.sim_tendons = []
		for tendon in self.get_tendons():
			new_tendon = TendonSim(tendon, self.operator)
			self.sim_tendons.append(new_tendon)

	def init_debug_parameters(self):
		"""Initializes pybullet debug parameters.
		
		Args:
			-

		Returns:
		   -

		"""
		for tendon_sim in self.sim_tendons:
			tendon_sim.force_id = p.addUserDebugParameter("Force in " + tendon_sim.name, 0, 1000, 0)
		self.cage_angle_id = p.addUserDebugParameter("Cage Angle", -180, 180, 0)

	def move_operator_sim(self):
		'''Moves the operator according to GUI buttons.
		'''
		count = p.readUserDebugParameter(self.stand_id)
		if(count > self.stand):
			self.stand = count
			self.current_move = Moves.STAND_STILL
		count = p.readUserDebugParameter(self.arm_roll_id)
		if(count > self.arm_roll):
			self.arm_roll = count
			self.current_move = Moves.ARM_ROLL
		count = p.readUserDebugParameter(self.catch_id)
		if(count > self.catch):
			self.catch = count
			self.current_move = Moves.CATCH
		count = p.readUserDebugParameter(self.hands_up_id)
		if(count > self.hands_up):
			self.hands_up = count
			self.current_move = Moves.HANDS_UP
		self.operator.move(self.current_move)
	
	def cage_rotation_listener(self, angle):
		"""Cage rotation listener.

		Args:
			angle (Float32): Angle message message.

		Returns:
			-
		
		"""
		self.rotate_cage(angle.data)
	
	def collision_listener(self, collision_msg):
		"""Collision listener.

		Args:
			collision_msg (Collision): Collision message.

		Returns:
			-
		
		"""
		self.get_logger().info(f"Received collision: link: {collision_msg.linkid} force: {collision_msg.normalforce}")
		
		self.draw_force(collision_msg)
		if self.mode == "tendon":
			collision_direction = np.array([collision_msg.contactnormal.x, collision_msg.contactnormal.y, collision_msg.contactnormal.z])
			
			_, rotation = p.getLinkState(self.operator.body_id, collision_msg.linkid)[:2]
			rotation = np.array(p.getMatrixFromQuaternion(rotation)).reshape(3,3)
			
			force_direction = rotation.dot(collision_direction)
			forces = self.decompose(collision_msg.linkid, collision_msg.normalforce, force_direction)
			
			for tendon in self.sim_tendons:
				if tendon.tendon.id in forces:
					force = forces[tendon.tendon.id]
				else:
					force = 0
				self.update_tendon(tendon.tendon.id, force)
		elif self.mode == "forces":
			force = collision_msg.normalforce
			vector = collision_msg.contactnormal

			force_vec = [force * vector.x, force * vector.y, force * vector.z]
			position_vec = [collision_msg.position.x, collision_msg.position.y, collision_msg.position.z]
			
			p.applyExternalForce(self.operator.body_id, collision_msg.linkid, force_vec, position_vec, p.LINK_FRAME)

	def draw_force(self, collision):
		"""Draw collision force as a debugLine.

		Args:
			collision (Collision): Collision to draw.

		Returns:
			-
		
		"""
		pos = np.array([collision.position.x, collision.position.y, collision.position.z])
		pos = [0,0,0]
		direction = np.array([collision.contactnormal.x,collision.contactnormal.y,collision.contactnormal.z]) * collision.normalforce
		p.addUserDebugLine(pos, pos + direction, [1, 0.4, 0.3], 2, 10, self.operator.body_id, collision.linkid)

	def update(self):
		"""Update ExoForce's state after a simulation step.
		
		Args:
			-

		Returns:
		   	-

		"""	
		self.operator.update_pose()

		for tendon_sim in self.sim_tendons:
			tendon_sim.tendon.update(self.operator)
			tendon_sim.update_lines()

		if self.mode == "debug":
			angle = p.readUserDebugParameter(self.cage_angle_id)
			self.rotate_cage(angle)

			for tendon_sim in self.sim_tendons:
				force = p.readUserDebugParameter(tendon_sim.force_id)
				self.update_tendon(tendon_sim.tendon.id, force)

		super().publish_state()

	def update_tendon(self, id, force):
		"""Applies a force to a tendon in the simulation.
		
		Args:
			id (int): Id of the tendon to update.
			force (float): Force applied to the tendon.

		Returns:
		   	-

		"""
		self.get_muscle_unit(id).set_motor_force(force)
		if force > 0: self.get_tendon_sim(id).apply_force(force)

	def get_tendon_sim(self, id):
		"""Gets ExoForceSim's tendon by id.
		
		Args:
			id (int): Id of the tendon simulation to search.

		Returns:
		   TendonSim: Tendon simulation with given id.

		"""
		tendon_sim = None
		for tendon in self.sim_tendons:
			if tendon.tendon.id == id:
				tendon_sim = tendon
				break
		return tendon_sim

	def operator_ef_pos_listener(self, ef_pose):
		"""Callback of the pose subscriber. Sets the pose of the link given in the msg.
		Args:
			ef_pose: received PoseStamped msg.
		
		Returns:
			-
		"""
		end_effector = ef_pose.header.frame_id
		#self.get_logger().info("Received pose for " + end_effector)
		if end_effector not in self.end_effectors:
			self.get_logger().warn(end_effector + " is not an end effector!")
			return

		link_pos = np.array([ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z])
		link_orn = np.array([ef_pose.pose.orientation.x, ef_pose.pose.orientation.y, ef_pose.pose.orientation.z, ef_pose.pose.orientation.w])
		# for muscle in muscles:
		# 	muscle.end_effector.world_point = link_pos

		if self.end_effectors[end_effector] is None:
			self.get_logger().info("Got ef initial pose: " + end_effector)
			self.end_effectors[end_effector] = {"position": None, "orientation": None}
		self.end_effectors[end_effector]["position"] = link_pos
		self.end_effectors[end_effector]["orientation"] = link_orn

class TendonSim():
	"""This class handles the simulation of each tendon attached to the operator.
	
	"""
	def __init__(self, tendon, operator):
		"""
		Args:
			tendon (Tendon): ExoForce's tendon to be simulated.
			operator (Operator): Operator object to which the tendon is attached.
		
		"""
		self.tendon = tendon
		self.name = "Tendon " + str(self.tendon.id)
		self.inactive_color = [0,0,255]
		self.active_color = [255,0,0]

		self.operator = operator

		self.force_id = None

		self.start_location = self.tendon.motor.via_point.world_point
		self.segments = []

		self.init_lines()

	def init_lines(self):
		"""Initializes debug lines for each via point.
		
		Args:
			-

		Returns:
		   -

		"""
		start = self.start_location
		for via_point in self.tendon.via_points:
			point = self.operator.get_link(via_point.link).get_center() + via_point.link_point
			segment = p.addUserDebugLine(start, point, lineColorRGB=self.inactive_color, lineWidth=2)
			self.segments.append(segment)
			start = point
		# self.debug_text = p.addUserDebugText(self.name, self.start_location, textColorRGB=self.inactive_color, textSize=0.8)

	def apply_force(self, force):
		"""Applies force to the simulated operator.
		
		Args:
			force (float): Force to be applied.

		Returns:
		   -

		Todo:
			* apply forces to all via points, currently the force is applied to the last via point (end effector)

		"""
		force_direction = np.asarray(self.start_location) - np.asarray(self.tendon.via_points[-1].world_point)
		force_direction /= norm(force_direction)

		p.applyExternalForce(objectUniqueId=self.operator.body_id,
				 		 linkIndex = self.operator.get_link_index(self.tendon.via_points[-1].link),
				 		 forceObj = force * force_direction,
				 		 posObj=self.start_location,
				 		 flags=p.WORLD_FRAME)

	def update_lines(self):
		"""Update debug lines position in simulation.
		
		Args:
			-

		Returns:
		   -

		"""
		start = self.start_location
		for segment, via_point in zip(self.segments, self.tendon.via_points):
			point = via_point.world_point
			color = self.active_color if self.tendon.motor.force > 0 else self.inactive_color
			p.addUserDebugLine(start, point, lineColorRGB=color, lineWidth=2, replaceItemUniqueId=segment)
			start = point
		# p.addUserDebugText(self.name, self.start_location, color, textSize=0.8, replaceItemUniqueId=self.debug_text)
