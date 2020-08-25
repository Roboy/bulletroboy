from abc import ABC, abstractmethod
import time
import math
import numpy as np

import xml.etree.ElementTree as ET
from xml.dom import minidom

from rclpy.node import Node
from roboy_control_msgs.msg import CageState, EndEffector, ViaPoint as ViaPointMsg, MuscleUnit as MuscleUnitMsg
from roboy_control_msgs.srv import GetCageEndEffectors
from geometry_msgs.msg import Point

from .force_decomposition import decompose_force_link_to_ef, decompose_force_ef_to_tendons

class CageConfiguration():
	"""This class handles the initial cage configuration.
		It reads and writes from a XML file.
	"""
	def __init__(self, filepath=None):
		"""
		Args:
			filepath (string): Path to XML configuration file. Default None.

		"""
		self.filepath = filepath
		self.cage_structure = {}
		self.muscle_units = []
		
		if self.filepath:
			self.load(filepath)

	def load(self, filepath):
		# parsing XML file
		tree = ET.parse(filepath)
		root = tree.getroot()

		# getting main items
		cage_xml = root.find('cageStructure')
		muscles_xml = root.find('muscleUnits')

		# parsing cage structure
		self.cage_structure['height'] = float(cage_xml.find('height').text)
		self.cage_structure['radius'] = float(cage_xml.find('radius').text)

		# parsing muscle units
		for muscle in muscles_xml:
			muscle_dict = {}
			muscle_dict['id'] = int(muscle.get('id'))
			muscle_dict['viaPoints'] = []
			for via_point_xml in muscle.find('viaPoints'):
				via_point = {}
				via_point['id'] = int(via_point_xml.get('id'))
				via_point['link'] = via_point_xml.get('link')
				via_point['point'] = np.array(list(map(float, via_point_xml.text.split(" "))))
				muscle_dict['viaPoints'].append(via_point)
			muscle_dict['parameters'] = {}
			for parameter in muscle.find('parameters'):
				muscle_dict['parameters'][parameter.tag] = parameter.text
			self.muscle_units.append(muscle_dict)

	def save(self, filepath=None):
		if filepath is None: filepath = self.filepath

		# main item
		cageConfiguration = ET.Element('cageConfiguration')

		# cageStructure
		cage_xml = ET.SubElement(cageConfiguration, 'cageStructure')
		cage_height = ET.SubElement(cage_xml, 'height')
		cage_radius = ET.SubElement(cage_xml, 'radius')
		cage_height.text = str(self.cage_structure['height'])
		cage_radius.text = str(self.cage_structure['radius'])

		# muscle Units
		muscles_xml = ET.SubElement(cageConfiguration, 'muscleUnits')
		for muscle in self.muscle_units:
			muscle_xml = ET.SubElement(muscles_xml, 'muscleUnit')
			muscle_xml.set('id', str(muscle['id']))
			muscle_via_points = ET.SubElement(muscle_xml, 'viaPoints')
			for via_point in muscle['viaPoints']:
				via_point_xml = ET.SubElement(muscle_via_points, 'viaPoint')
				via_point_xml.set('id', str(via_point['id']))
				via_point_xml.set('link', via_point['link'])
				via_point_xml.text = str(via_point['point'].tolist()).strip('[]').replace(',', '')
			muscle_parameters = ET.SubElement(muscle_xml, 'parameters')
			for k, v in muscle['parameters'].items():
				subelement = ET.SubElement(muscle_parameters, k)
				subelement.text = v

		# write to file
		mydata = ET.tostring(cageConfiguration).decode()
		reparsed = minidom.parseString(mydata)
		mydata = reparsed.toprettyxml(indent="\t")
		myfile = open(filepath, "w")
		myfile.write(mydata)
		myfile.close()

	def __str__(self):
		msg = f"\nCAGE CONFIGURATION: ({self.filepath})\n"
		msg += "-------------------\n"
		msg += "Cage Structure:\n"
		msg += f"\theight: {self.cage_structure['height']}\n"
		msg += f"\tradius: {self.cage_structure['radius']}\n"
		msg += "Muscle Units:\n"
		for muscle in self.muscle_units:
			msg += "\t-----------------\n"
			msg += f"\tid: {muscle['id']}\n"
			msg += "\tvia points:\n"
			for via_point in muscle['viaPoints']:
				msg += f"\t\t{via_point['id']} link: {via_point['link']:<20}\tpoint: {via_point['point']}\n"
			msg += "\tparameters:\n"
			for k, v in muscle['parameters'].items():
				msg += f"\t\t{k}: {v}\n"
		return msg


class ViaPoint():
	"""This class handles tendon's via points.

	"""
	def __init__(self, via_point):
		"""
		Args:
			via_point (dict): Via point's configuration defined in the configuration file.

		"""
		self.id = via_point['id']
		self.link = via_point['link']
		self.link_point = via_point['point']
		self.world_point = via_point['point']

	def to_msg(self, init_conf=False):
		"""Returns via point data as a ROS message.
		
		Args:
			init_conf (bool): If True the ROS message will include data from the configuration file,
				if False, it will include current state data.

		Returns:
		   ViaPointMsg: ROS via point message definition.

		"""
		via_point_msg = ViaPointMsg()
		via_point_msg.id = self.id
		via_point_msg.position.x, via_point_msg.position.y, via_point_msg.position.z = self.link_point if init_conf else self.world_point
		via_point_msg.link = self.link
		via_point_msg.reference_frame = self.link if init_conf else "world"
		return via_point_msg


class Tendon():
	"""This class handles each tendon attached to the operator.

	"""
	def __init__(self, id, motor, via_points):
		"""
		Args:
			id (int): Id of the via point.
			motor (Motor): Motor object to which the tendon is attached to.
			via_points (List[ViaPoint]): Tendon via points which are attached to the operator.

		"""
		self.id = id
		self.motor = motor
		self.via_points = via_points

	def update(self, operator):
		"""Updates via points' world points according to operator state.
		
		Args:
			operator (Operator): Operator object to which the tendon is attached.

		Returns:
			-

		Todo:
			* consider link orientation to get via point's world point

		"""
		for via_point in self.via_points:
			via_point.world_point = operator.get_link_center(via_point.link) + via_point.link_point


class Motor():
	"""This class handles a motor attached to the cage.

	"""
	def __init__(self, id, via_point):
		"""
		Args:
			id (int): Id of the motor.
			via_point (ViaPoint): Point in which the motor is attached to the cage.
		
		Raises:
			AssertionError: Via point link must be cage.
		
		"""
		assert via_point.link == "cage"

		self.id = id
		self.via_point = via_point
		self.force = 0.0

	def rotate(self, delta, pivot):
		"""Updates motor's world position with rotation angle around a pivot point.
		
		Args:
			delta (float): Rotation in degrees.
			pivot (List[float]): Vector3 point around which the rotation is happening.

		Returns:
			-

		"""

		x, y = self.via_point.world_point[:2]
		offset_x, offset_y = pivot[:2]
		adjusted_x = (x - offset_x)
		adjusted_y = (y - offset_y)
		cos_rad = math.cos(math.radians(delta))
		sin_rad = math.sin(math.radians(delta))
		qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
		qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
		
		self.via_point.world_point[0] = qx
		self.via_point.world_point[1] = qy


class MuscleUnit():
	"""This class handles a muscle unit including it's tendon and motor.

	"""
	def __init__(self, id, via_points, parameters):
		"""
		Args:
			id (int): Id of the motor.
			via_points (List[dict]): Via points' configuration defined in the configuration file.
			parameters (dict): Muscle unit's parameters defined in the configuration file.
		
		"""
		self.id = id
		self.via_points = [ViaPoint(via_point) for via_point in via_points]
		self.motor = Motor(self.id, self.via_points[0])
		self.tendon = Tendon(self.id, self.motor, self.via_points[1:])
		self.end_effector = self.via_points[-1]
		# parameters
		self.max_force = float(parameters['max_force'])

	def set_motor_force(self, force):
		"""Sets motor's current applied force.
		
		Args:
			force (float): Force applied by motor.

		Returns:
			-

		"""
		self.motor.force = force

	def to_msg(self, init_conf=False):
		"""Returns muscle unit data as a ROS message.
		
		Args:
			init_conf (bool): If 'True' the ROS message will include data from the configuration file,
				if 'False', it will include current state data.

		Returns:
		   MuscleUnitMsg: ROS muscle unit message definition.

		"""
		muscle_msg = MuscleUnitMsg()
		muscle_msg.id = self.id
		if init_conf: muscle_msg.max_force = self.max_force
		for via_point in self.via_points:
			via_point_msg = via_point.to_msg(init_conf)
			muscle_msg.viapoints.append(via_point_msg)
		return muscle_msg


class Cage():
	"""This class handles the cage and the motors attached to it.
	
	"""
	def __init__(self, height, radius, motors):
		"""
		Args:
			height (float): Height of the cage in meters.
			radius (float): Radius of the cage in meters.
			motors (List[Motor]): Tendons' motors in the cage.

		"""
		self.origin = np.array([0, 0, 0])
		self.height = height
		self.radius = radius
		self.angle = 0.0

		self.motors = motors

	def rotate_cage(self, new_angle):
		"""Updates cage angle and motors' world positions with new angle.
		
		Args:
			new_angle (float): New angle to which the cage is rotated in degrees.

		Returns:
			-

		"""
		for motor in self.motors:
			motor.rotate(self.angle - new_angle, self.origin)

		self.angle = new_angle


class ExoForce(Node, ABC):
	"""Main ExoForce class. It handles the muscle units and the cage itself.
	
	"""
	def __init__(self, cage_conf, node_name):
		"""
		Args:
			cage_conf (CageConfiguration): Cage configuration defined in the configuration file.
			node_name (string): Node name for ROS node's initialization.
		
		"""
		super().__init__(node_name)
		self.muscle_units = [ MuscleUnit(muscle['id'], muscle['viaPoints'], muscle['parameters'])
								for muscle in cage_conf.muscle_units ]
		self.cage = Cage(cage_conf.cage_structure['height'], cage_conf.cage_structure['radius'], self.get_motors())

		self.init_end_effectors()

		self.cage_state_publisher = self.create_publisher(CageState, '/roboy/simulation/cage_state', 1)
		self.initial_conf_service = self.create_service(GetCageEndEffectors, '/roboy/configuration/end_effectors', self.get_end_effectors_callback)
	
	def init_end_effectors(self):
		"""Initializes end effectors list.
		
		Args:
			-

		Returns:
		   -

		"""
		self.end_effectors = []
		for muscle in self.muscle_units:
			ef = muscle.end_effector.link
			if ef not in self.end_effectors: self.end_effectors.append(ef)

	@abstractmethod
	def update(self):
		pass

	def rotate_cage(self, angle):
		"""Applies cage rotation to a new angle.
		
		Args:
			angle (float): New angle for cage rotation in degress.

		Returns:
		   	-

		"""
		self.cage.rotate_cage(angle)

	def get_muscle_unit(self, id):
		"""Gets ExoForce's muscle unit by id.
		
		Args:
			id (int): Id of the muscle unit to search.

		Returns:
		   MuscleUnit: Muscle unit with given id.

		"""
		unit = None
		for muscle in self.muscle_units:
			if muscle.id == id:
				unit = muscle
				break
		return unit
	
	def get_ef_muscle_units(self, end_effector):
		"""Gets ExoForce's muscle units by end effector.
		
		Args:
			end_effector (string): End effector name's to search for muscle units.

		Returns:
		   	List[MuscleUnit]: Muscle units with tendons attached to the given end effector.

		"""
		muscle_units = []
		for muscle in self.muscle_units:
			if muscle.end_effector.link == end_effector:
				muscle_units.append(muscle)
		return muscle_units

	def get_motors(self):
		"""Gets ExoForce's motors.
		
		Args:
			-

		Returns:
			List[Motor]: ExoForce's motors.
		
		"""
		return [ muscle.motor for muscle in self.muscle_units ]

	def get_tendons(self):
		"""Gets ExoForce's tendons.
		
		Args:
			-

		Returns:
			List[Tendon]: ExoForce's tendons.
		
		"""
		return [ muscle.tendon for muscle in self.muscle_units ]

	def to_msg(self):
		"""Returns ExoForce's data as a ROS message.
		
		Args:
			-

		Returns:
		   CageState: ROS cage state message definition.

		"""
		state_msg = CageState()
		state_msg.rotation_angle = self.cage.angle
		for muscle in self.muscle_units:
			muscle_msg = muscle.to_msg()
			state_msg.muscleunits.append(muscle_msg)
			
		return state_msg

	def publish_state(self):
		"""Publishes ExoForce's state as a ROS message.
		
		Args:
			-

		Returns:
		   	-

		"""
		self.cage_state_publisher.publish(self.to_msg())

	def get_end_effectors_callback(self, request, response):
		"""ROS service callback to get end effectors initial configuration.

		"""
		for ef in self.end_effectors:
			end_effector_msg = EndEffector()
			end_effector_msg.name = ef
			for muscle in self.get_ef_muscle_units(ef):
				muscle_msg = muscle.to_msg(init_conf=True)
				end_effector_msg.muscle_units.append(muscle_msg)
			response.end_effectors.append(end_effector_msg)

		return response

	def decompose(self, link_id, collision_force, collision_direction):
		"""Decomposes force applied to link in operator.
		
		Args:
			link_id (int): Link id of the operator where the force is applied.
			collision_force (float): Value of the applied force.
			collision_direction array[3]: Force direction in world space coordinates.

		Returns:
		   	dict: Dictionary with the decomposed forces, the key is the tendon id.

		"""
		ef = decompose_force_link_to_ef(link_id)

		forces, msg = decompose_force_ef_to_tendons(collision_force, collision_direction, self.get_ef_muscle_units(ef)) if link_id is not None else {}
		
		if not forces:
			self.get_logger().warn(f"Force was not decomposed: force[{collision_force}] ef[{ef}] [{msg}]")

		return forces
