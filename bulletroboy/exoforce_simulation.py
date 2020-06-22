import pybullet as p
import numpy as np
from numpy.linalg import norm

import rclpy

from roboy_simulation_msgs.msg import TendonUpdate
from roboy_simulation_msgs.msg import Collision
from std_msgs.msg import Float32

from bulletroboy.exoforce import ExoForce
from bulletroboy.operator import Operator
from geometry_msgs.msg import Vector3
from bulletroboy.topics import TENDON_FORCE, COLLISION_EXOFORCE, CAGE_ROTATION


class ExoForceSim(ExoForce):
	"""ExoForce Child class. This class handles the simulation of the exoforce.

	"""
	def __init__(self, cage_conf, human_model, mode):
		"""
		Args:
			cage_conf (CageConfiguration): Cage configuration defined in the configuration file.
			human_model (int): Id of the human model loaded in pybullet.
			mode (string): Mode in which the ExoForceSim will we executed.
		
		"""
		super().__init__(cage_conf, "exoforce_simulation")

		self.mode = mode
		self.operator = Operator(human_model)
		self.init_sim()

		if self.mode == "debug":
			self.init_debug_parameters()
		else:
			if self.mode == "tendon":
				self.create_subscription(TendonUpdate, TENDON_FORCE, self.tendon_update_listener, 10)
			elif self.mode == "forces":
				self.create_subscription(Collision, COLLISION_EXOFORCE, self.forces_update_listener, 10)
			self.create_subscription(Float32, CAGE_ROTATION, self.cage_rotation_listener, 10)

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
			tendon_sim.force_id = p.addUserDebugParameter("Force in " + tendon_sim.name, 0, 200, 0)
		self.cage_angle_id = p.addUserDebugParameter("Cage Angle", -180, 180, 0)

	def tendon_update_listener(self, tendon_force):
		self.update_tendon(tendon_force.tendon_id, tendon_force.force)

	def cage_rotation_listener(self, angle):
		self.rotate_cage(angle.data)

	def forces_update_listener(self, forces):
		rclpy.logging._root_logger.info('Updating tendon')
		force = forces.normalforce
		vector = forces.contactnormal

		force_vec = [force * vector.x, force * vector.y, force * vector.z]
		position_vec = [forces.position.x, forces.position.y, forces.position.z]

		self.apply_force_on_op(forces.linkid, force_vec, position_vec)

	def apply_force_on_op(self, linkid, force_vec, position_on_link):
		body_id = self.operator.body_id
		link_pos = list(p.getLinkState(body_id, linkid)[0])
		force_position = [sum(x) for x in zip(link_pos, position_on_link)]
		print("force_position: ", force_position)

		p.applyExternalForce(body_id, linkid, force_vec, force_position, p.WORLD_FRAME)


	def update(self):
		"""Update ExoForce's state after a simulation step.
		
		Args:
			-

		Returns:
		   	-

		"""	
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
		self.get_muscle_unit(id).force = force
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
		self.debug_color = [0,0,255]

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
			point = self.operator.get_link_center(via_point.link) + via_point.link_point
			segment = p.addUserDebugLine(start, point, lineColorRGB=self.debug_color, lineWidth=2)
			self.segments.append(segment)
			start = point
		self.debug_text = p.addUserDebugText(self.name, self.start_location, textColorRGB=self.debug_color, textSize=0.8)

	def apply_force(self, force):
		"""Applies force to the simulated operator.
		
		Args:
			force (float): Force to be applied.

		Returns:
		   -

		Todo:
			* apply forces to all via points, currently the force is applied to the last via point (end effector)
			* move method to the Operator class

		"""
		force_direction = np.asarray(self.start_location) - np.asarray(self.tendon.attachtment_points[-1])
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
			p.addUserDebugLine(start, point, lineColorRGB=self.debug_color, lineWidth=2, replaceItemUniqueId=segment)
			start = point
		p.addUserDebugText(self.name, self.start_location, self.debug_color, textSize=0.8, replaceItemUniqueId=self.debug_text)

