from abc import ABC, abstractmethod
import pybullet as p
import time
import math
import numpy as np

import xml.etree.ElementTree as ET
from xml.dom import minidom

from rclpy.node import Node
from roboy_control_msgs.msg import CageState, ViaPoint, EndEffector, MuscleUnit as MuscleUnitMsg
from roboy_control_msgs.srv import GetCageEndEffectors
from geometry_msgs.msg import Point


class CageConfiguration():
    def __init__(self, filepath=None):
        """
		This class handles the initial cage configuration.
		It can read and write from an XML file.
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
        self.cage_structure['height'] = int(cage_xml.find('height').text)
        self.cage_structure['radius'] = int(cage_xml.find('radius').text)

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
		self.motor = Motor(self.id, via_points[0])
		self.tendon = Tendon(self.id, self.motor, via_points[1:])
		self.end_effector = via_points[-1]["link"]
		# parameters
		self.max_force = float(parameters['max_force'])

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

		self.init_end_effectors()

		self.cage_state_publisher = self.create_publisher(CageState, '/roboy/simulation/cage_state', 10)
		self.initial_conf_service = self.create_service(GetCageEndEffectors, '/roboy/configuration/end_effectors', self.get_end_effectors_callback)
	
	def init_end_effectors(self):
		self.end_effectors = []
		for muscle in self.muscle_units:
			ef = muscle.end_effector
			if ef not in self.end_effectors: self.end_effectors.append(ef)

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
	
	def get_ef_muscle_units(self, end_effector):
		muscle_units = []
		for muscle in self.muscle_units:
			if muscle.end_effector == end_effector:
				muscle_units.append(muscle)
		return muscle_units

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
				via_point_msg.position.x, via_point_msg.position.y, via_point_msg.position.z = world_point
				via_point_msg.reference_frame = 'world'
				via_point_msg.link = via_point['link']
				muscle_msg.viapoints.append(via_point_msg)
			state_msg.muscleunits.append(muscle_msg)
			
		return state_msg

	def publish_state(self):
		self.cage_state_publisher.publish(self.get_msg())

	def get_end_effectors_callback(self, request, response):

		for ef in self.end_effectors:
			end_effector_msg = EndEffector()
			end_effector_msg.name = ef
			for muscle in self.get_ef_muscle_units(ef):
				muscle_msg = MuscleUnitMsg()
				muscle_msg.id = muscle.id
				muscle_msg.max_force = muscle.max_force
				for via_point in muscle.via_points:
					via_point_msg = ViaPoint()
					via_point_msg.id = via_point['id']
					via_point_msg.position.x, via_point_msg.position.y, via_point_msg.position.z = via_point['point']
					via_point_msg.reference_frame = 'link' if via_point['link'] != "cage" else 'world'
					via_point_msg.link = via_point['link']
					muscle_msg.viapoints.append(via_point_msg)
				end_effector_msg.muscle_units.append(muscle_msg)
			response.end_effectors.append(end_effector_msg)

		return response
