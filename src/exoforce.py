import pybullet as p
import time
import math
import numpy as np
from numpy.linalg import norm

import xml.etree.ElementTree as ET
from xml.dom import minidom


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
            muscle_dict['id'] = muscle.get('id')
            muscle_dict['viaPoints'] = []
            for via_point_xml in muscle.find('viaPoints'):
                via_point = {}
                via_point['id'] = via_point_xml.get('id')
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
            muscle_xml.set('id', muscle['id'])
            muscle_via_points = ET.SubElement(muscle_xml, 'viaPoints')
            for via_point in muscle['viaPoints']:
                via_point_xml = ET.SubElement(muscle_via_points, 'viaPoint')
                via_point_xml.set('id', via_point['id'])
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


class Operator():
	def __init__(self, body_id):
		"""
		This class handles the operator body and its links in the simulation.
		"""
		self.body_id = body_id
		self.links = self.get_links()

	def get_links(self):
		links = []
		for i in range(p.getNumJoints(self.body_id)):
			link = {}
			link['name'] = str(p.getJointInfo(self.body_id,i)[12], 'utf-8')
			link['id'] = i
			links.append(link)
		return links

	def get_link_center(self, link_name):
		center = None
		index = self.get_link_index(link_name)
		if index:
			center = np.asarray(p.getLinkState(self.body_id, self.links[index]['id'])[0])
		return center

	def get_link_index(self, link_name):
		index = None
		for i, link in enumerate(self.links):
			if link['name'] == link_name:
				index = i
				break
		return index


class Tendon():
	def __init__(self, id, motor, via_points, operator):
		"""
		This class handles each tendon attached to the operator.
		"""
		self.id = id
		self.name = "Tendon " + str(self.id)
		self.debug_color = [0,0,255]
		self.motor = motor

		self.via_points = via_points
		self.operator = operator

		self.forceId = None

		self.start_location = self.motor.pos
		self.end_location = self.via_points[-1]['point']
		self.segments = []

		self.init_tendon()

	def init_tendon(self):
		start = self.start_location
		for via_point in self.via_points:
			point = self.get_point(via_point)
			segment = p.addUserDebugLine(start, point, lineColorRGB=self.debug_color, lineWidth=2)
			self.segments.append(segment)
			start = point

		self.debug_text = p.addUserDebugText(self.name, self.start_location, textColorRGB=self.debug_color, textSize=0.8)

	def get_point(self, via_point):
		return self.operator.get_link_center(via_point['link']) + via_point['point']

	def apply_force(self, force):
		# TODO: apply forces to all via points, currently the force is applied to the last via point

		force_direction = np.asarray(self.start_location) - np.asarray(self.end_location)
		force_direction /= norm(force_direction)

		p.applyExternalForce(objectUniqueId=self.operator.body_id,
			     	     linkIndex = self.operator.get_link_index(self.via_points[-1]['link']),
			     	     forceObj = force * force_direction,
			     	     posObj=self.start_location,
			     	     flags=p.WORLD_FRAME)

	def update_lines(self):
		start = self.start_location
		for segment, via_point in zip(self.segments, self.via_points):
			point = self.get_point(via_point)
			p.addUserDebugLine(start, point, lineColorRGB=self.debug_color, lineWidth=2, replaceItemUniqueId=segment)
			self.end_location = point

	def update(self, force):
		self.update_lines()
		p.addUserDebugText(self.name, self.start_location, self.debug_color, textSize=0.8, replaceItemUniqueId=self.debug_text)
		self.apply_force(force)


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
		self.pos = via_point['point']

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
	def __init__(self, id, via_points, parameters, operator):
		"""
		This class handles a muscle unit including its tendon and motor.
		"""
		self.id = id
		self.via_points = via_points
		self.parameters = parameters
		self.motor = Motor(self.id, via_points[0])
		self.tendon = Tendon(self.id, self.motor, via_points[1:], operator)

	def update(self, force):
		self.tendon.update(force)


class ExoForce():
	def __init__(self, cage_conf, operator):
		"""
		Main ExoForce class. It handles the muscle units and the cage itself.
		"""
		self.operator = operator

		self.muscle_units = [ MuscleUnit(muscle['id'], muscle['viaPoints'], muscle['parameters'], self.operator)
								for muscle in cage_conf.muscle_units ]
		self.cage = Cage(cage_conf.cage_structure['height'], cage_conf.cage_structure['radius'], self.get_motors())

	def update(self, new_angle, motor_forces):

		self.cage.rotate_cage(new_angle)

		for muscle, force in zip(self.muscle_units, motor_forces):
			muscle.update(force)

	def get_cage(self):
		return self.cage

	def get_muscle_units(self):
		return self.muscle_units
	
	def get_motors(self):
		return [ muscle.motor for muscle in self.muscle_units ]

	def get_tendons(self):
		return [ muscle.tendon for muscle in self.muscle_units ]


class Movements():
	def __init__(self, body_id):
		"""
		This class defines 2 movements: Re-definable inverse kinematic ones & harcoded simple ones. 
		"""
		self.body_id = body_id
		self.links = self.get_links()		# delete this later

	def get_EF(self, link_name):
                freeJoints = []
                numJoints = p.getNumJoints(self.body_id)

                for i in range(numJoints):
                    info = p.getJointInfo(self.body_id,i)
                    if info[2] == p.JOINT_REVOLUTE:
                        freeJoints.append(i)
                    if info[12] == link_name:
                            endEffectorId = i

                return endEffectorId, freeJoints

	def get_links(self):				#TODO: delete this later.
		links = []
		for i in range(p.getNumJoints(self.body_id)):
			link = {}
			link['name'] = str(p.getJointInfo(self.body_id,i)[12], 'utf-8')
			link['id'] = i
			links.append(link)
		return links

	def get_link_index(self, link_name):		#TODO: delete this later.
		index = None
		for i, link in enumerate(self.links):
			if link['name'] == link_name:
				index = i
				break
		return index	


	def one_end_efector(self, link_name, pos, maxIter):		# Uses inverse kinematics.
		endEffectorId, freeJoints = get_EF(link_name)
		while(iter < maxIter):
		    jointPoses = p.calculateInverseKinematics(self.body_id, endEffectorId, pos)
		    for i in range(len(freeJoints)):
          		p.resetJointState(self.body_id, freeJoints[i], jointPoses[i])


	def two_end_effectors(self, link_names, postions, maxIter):	# Not yet implemented.
		return 0


	def simple_move(self, case, t):			# Hardcoded simple movements.
		spine_link = self.get_link_index('human/spine_2')
		spine_side_link = self.get_link_index('human/spine_0')
		left_shoulder_1 = self.get_link_index('human/left_shoulder_1')
		right_shoulder_1 = self.get_link_index('human/right_shoulder_1')
		left_shoulder_0 = self.get_link_index('human/left_shoulder_0')
		right_shoulder_0 = self.get_link_index('human/right_shoulder_0')

		if case == 'spine_swing':		# May be useful for "chest kick" into simulation (?)
                    p.resetJointState(self.body_id, spine_link, math.sin(t))

		elif case == 'forearm_roll':		# May be useful when checking tendon collisions (fix)
                    p.resetJointState(self.body_id, left_shoulder_1, -1.75*math.pi/4)
                    p.resetJointState(self.body_id, right_shoulder_1, 1.75*math.pi/4)

                    left_elbow = self.get_link_index('human/left_elbow')
                    right_elbow = self.get_link_index('human/right_elbow')

                    p.resetJointState(self.body_id, left_elbow, -abs(math.sin(t)*2*math.pi/4))
                    # p.resetJointState(self.body_id, right_elbow, abs(math.cos(t+math.pi/2)*2*math.pi/4))
                    p.resetJointState(self.body_id, left_shoulder_0, (math.cos(t)*2*math.pi/4))
                    # p.resetJointState(self.body_id, right_shoulder_0, (math.cos(t)*2*math.pi/4))

		elif case == 'arm_roll': 	# May be useful to test roboys shoulder limitations in cage?
                    p.resetJointState(self.body_id, left_shoulder_1, math.sin(t))
                    p.resetJointState(self.body_id, left_shoulder_0, math.sin(t+math.pi/2))
                    p.resetJointState(self.body_id, right_shoulder_1, -math.sin(t))
                    p.resetJointState(self.body_id, right_shoulder_0, math.sin(t+math.pi/2))

		elif case == 'side_swing':	# To test connection between figure and cage orientation.
		    p.resetJointState(self.body_id, spine_side_link, math.sin(t))
		    # TODO: Connect chest orientation to cage rotation!
		    

		else:
		    print('ERROR: No definition set for case = ', case)


