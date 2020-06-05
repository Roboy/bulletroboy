import pybullet as p
import numpy as np
from numpy.linalg import norm

from roboy_simulation_msgs.msg import TendonUpdate
from std_msgs.msg import Float32

from bulletroboy.exoforce import ExoForce
from bulletroboy.operator import Operator


class ExoForceSim(ExoForce):
	def __init__(self, cage_conf, human_model, mode):
		"""
		ExoForce Child class. This class handles the simulation of the exoforce.
		"""
		super().__init__(cage_conf, "exoforce_simulation")

		self.mode = mode
		self.operator = Operator(human_model)

		self.init_sim()

		if self.mode == "debug":
			self.init_debug_parameters()
		else:
			if self.mode == "tendon":
				self.create_subscription(TendonUpdate, '/roboy/simulation/tendon_force', self.tendon_update_listener, 10)
			elif self.mode == "forces":
				# TODO: update subscriber with correct msg type
				self.create_subscription(TendonUpdate, '/roboy/simulation/operator_forces', self.forces_update_listener, 10)
			self.create_subscription(Float32, 'roboy/simulation/cage_rotation', self.cage_rotation_listener, 10)

	def init_sim(self):
		self.sim_tendons = []
		for tendon in self.get_tendons():
			new_tendon = TendonSim(tendon, self.operator)
			self.sim_tendons.append(new_tendon)

	def init_debug_parameters(self):
		for tendon_sim in self.sim_tendons:
			tendon_sim.force_id = p.addUserDebugParameter("Force in " + tendon_sim.name, 0, 200, 0)
		self.cage_angle_id = p.addUserDebugParameter("Cage Angle", -180, 180, 0)

	def tendon_update_listener(self, tendon_force):
		self.update_tendon(tendon_force.tendon_id, tendon_force.force)

	def cage_rotation_listener(self, angle):
		self.rotate_cage(angle.data)

	def forces_update_listener(self, forces):
        # TODO: implement force update
		pass

	def update(self):
		
		for tendon_sim in self.sim_tendons:
			tendon_sim.tendon.update(self.operator)
			tendon_sim.update_lines()

		if self.mode == "debug":
			angle = p.readUserDebugParameter(self.cage_angle_id)
			self.rotate_cage(angle)

			for tendon_sim in self.sim_tendons:
				force = p.readUserDebugParameter(tendon_sim.force_id)
				self.update_tendon(tendon_sim.tendon.id, force)

	def update_tendon(self, id, force):
		self.get_muscle_unit(id).force = force
		if force > 0: self.get_tendon_sim(id).apply_force(force)

	def get_tendon_sim(self, id):
		tendon_sim = None
		for tendon in self.sim_tendons:
			if tendon.tendon.id == id:
				tendon_sim = tendon
				break
		return tendon_sim


class TendonSim():
	def __init__(self, tendon, operator):
		"""
		This class handles the simulation of each tendon attached to the operator.
		"""
		self.tendon = tendon
		self.name = "Tendon " + str(self.tendon.id)
		self.debug_color = [0,0,255]

		self.operator = operator

		self.force_id = None

		self.start_location = self.tendon.motor.pos
		self.segments = []

		self.init_lines()

	def init_lines(self):
		start = self.start_location
		for via_point in self.tendon.via_points:
			point = self.operator.get_link_center(via_point['link']) + via_point['point']
			segment = p.addUserDebugLine(start, point, lineColorRGB=self.debug_color, lineWidth=2)
			self.segments.append(segment)
			start = point
		self.debug_text = p.addUserDebugText(self.name, self.start_location, textColorRGB=self.debug_color, textSize=0.8)

	def apply_force(self, force):
		# TODO: apply forces to all via points, currently the force is applied to the last via point
		force_direction = np.asarray(self.start_location) - np.asarray(self.tendon.attachtment_points[-1])
		force_direction /= norm(force_direction)

		p.applyExternalForce(objectUniqueId=self.operator.body_id,
			     	     linkIndex = self.operator.get_link_index(self.tendon.via_points[-1]['link']),
			     	     forceObj = force * force_direction,
			     	     posObj=self.start_location,
			     	     flags=p.WORLD_FRAME)

	def update_lines(self):
		start = self.start_location
		for segment, point in zip(self.segments, self.tendon.attachtment_points):
			p.addUserDebugLine(start, point, lineColorRGB=self.debug_color, lineWidth=2, replaceItemUniqueId=segment)
			start = point
		p.addUserDebugText(self.name, self.start_location, self.debug_color, textSize=0.8, replaceItemUniqueId=self.debug_text)

