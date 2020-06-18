from abc import ABC, abstractmethod
import pybullet as p
import time
import math
import numpy as np

from rclpy.node import Node
from roboy_control_msgs.msg import CageState, ViaPoint
from roboy_control_msgs.msg import MuscleUnit as MuscleUnitMsg
from geometry_msgs.msg import Point


class Tendon():
	def __init__(self, id, motor, via_points):
		"""
		This class handles each tendon attached to the operator.
		"""
		self.id = id
		self.motor = motor

		self.via_points = via_points # points relative to the link
		self.attachtment_points = [None for _ in self.via_points] # points in world space

	def update(self, operator):
		for i in range(len(self.attachtment_points)):
			self.attachtment_points[i] = operator.get_link_center(self.via_points[i]['link']) + self.via_points[i]['point']


class Cage():
	def __init__(self, height, radius, motors):
		"""
		This class handles the cage and the motors attached to it.
		"""
		self.origin = np.array([0, 0, 0])
		self.height = height
		self.radius = radius
		self.angle = 0

		self.motors = motors

	def rotate_cage(self, new_angle):

		for motor in self.motors:
			motor.rotate(self.angle - new_angle, self.origin)

		self.angle = new_angle


class Motor():
	def __init__(self, id, via_point):
		"""
		This class handles a motor attached to the cage.
		"""
		assert via_point['link'] == 'cage'

		self.id = id
		self.link = via_point['link']
		self.pos = via_point['point']
		self.force = 0

	def rotate(self, delta, pivot):

		x, y = self.pos[:2]
		offset_x, offset_y = pivot[:2]
		adjusted_x = (x - offset_x)
		adjusted_y = (y - offset_y)
		cos_rad = math.cos(math.radians(delta))
		sin_rad = math.sin(math.radians(delta))
		qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
		qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
		
		self.pos[0] = qx
		self.pos[1] = qy


class MuscleUnit():
	def __init__(self, id, via_points, parameters):
		"""
		This class handles a muscle unit including its tendon and motor.
		"""
		self.id = id
		self.via_points = via_points
		self.parameters = parameters
		self.motor = Motor(self.id, via_points[0])
		self.tendon = Tendon(self.id, self.motor, via_points[1:])

	def set_motor_force(self, force):
		self.motor.force = force


class ExoForce(Node, ABC):
	def __init__(self, cage_conf, node_name):
		"""
		Main ExoForce class. It handles the muscle units and the cage itself.
		"""
		super().__init__(node_name)
		self.muscle_units = [ MuscleUnit(muscle['id'], muscle['viaPoints'], muscle['parameters'])
								for muscle in cage_conf.muscle_units ]
		self.cage = Cage(cage_conf.cage_structure['height'], cage_conf.cage_structure['radius'], self.get_motors())

		self.cage_state_publisher = self.create_publisher(CageState, '/roboy/simulation/cage_state', 10)
	
	@abstractmethod
	def update(self):
		pass

	def rotate_cage(self, angle):
		self.cage.rotate_cage(angle)

	def tendon_update(self, id, force):
		self.get_muscle_unit(id).update(force)

	def get_muscle_unit(self, id):
		unit = None
		for muscle in self.muscle_units:
			if muscle.id == id:
				unit = muscle
				break
		return unit
	
	def get_motors(self):
		return [ muscle.motor for muscle in self.muscle_units ]

	def get_tendons(self):
		return [ muscle.tendon for muscle in self.muscle_units ]

	def get_msg(self):
		state_msg = CageState()
		state_msg.rotation_angle = float(self.cage.angle)
		for muscle in self.muscle_units:
			muscle_msg = MuscleUnitMsg()
			muscle_msg.id = muscle.id
			for via_point, world_point in zip(muscle.via_points, [muscle.motor.pos] + muscle.tendon.attachtment_points):
				via_point_msg = ViaPoint()
				via_point_msg.id = via_point['id']
				via_point_msg.position = Point()
				via_point_msg.position.x = world_point[0]
				via_point_msg.position.y = world_point[1]
				via_point_msg.position.z = world_point[2]
				via_point_msg.reference_frame = 'world'
				via_point_msg.link = via_point['link']
				muscle_msg.viapoints.append(via_point_msg)
			state_msg.muscleunits.append(muscle_msg)
			
		return state_msg

	def publish_state(self):
		self.cage_state_publisher.publish(self.get_msg())
