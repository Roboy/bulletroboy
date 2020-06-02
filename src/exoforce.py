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

	def update(self, new_angle, motor_forces, automatic_cage_rotation):

		if automatic_cage_rotation == True:
			chest_id = self.operator.get_link_index('human/spine')
			chest_state = p.getLinkState(self.operator.body_id, chest_id)
			chest_cart_orientation = chest_state[1][2]
			new_angle = np.arcsin(chest_cart_orientation)*180/math.pi

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
		This class defines 2 types of movements:
		-  Re-definable inverse kinematic ones (one_end_effector(), multiple_end_effectors())
		-  Harcoded simple ones (simple_move() --> 4 predefined movements)
		"""
		self.body_id = body_id
		self.op = Operator(self.body_id)


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


	def apply_chest_and_neck_constraints(self):
		p.createConstraint(self.body_id, self.op.get_link_index('human/spine_1'), -1,
				      self.op.get_link_index('human/spine_2'), p.JOINT_FIXED,
				      [0, 0, 0], [0, 0, 0], [0, 0, 1])

		p.createConstraint(self.body_id, self.op.get_link_index('human/neck'), -1, 					      self.op.get_link_index('human/spine_2'), p.JOINT_FIXED,
				      [0, 0, 0], [0, 0, 0], [0, 0, 1])


	def one_end_effector(self, link_name, pos, maxIter, chest_constraint):
		endEffectorId, freeJoints = self.get_EF(link_name)
		if chest_constraint == True:
		    self.apply_chest_and_neck_constraints()
		iter = 0
		while(iter <= maxIter):
		    jointPoses = p.calculateInverseKinematics(self.body_id, endEffectorId, pos)
		    for i in range(len(freeJoints)):
          		p.resetJointState(self.body_id, freeJoints[i], jointPoses[i])
		    iter = iter + 1


	def multiple_end_effectors(self, link_names, positions, maxIter, chest_constraint):
		# This function can lead to problems if the wrong positions are chosen. Hence:
		# TODO: Implement code that makes sure that the function only reacts to adequate position inputs.
		endEffectorIds = []
		for i in range(len(link_names)):
		    endEffectorId_i, freeJoints = self.get_EF(link_names[i])
		    endEffectorIds.append(endEffectorId_i)
		if chest_constraint == True:
		    self.apply_chest_and_neck_constraints()
		iter = 0
		while(iter <= maxIter):
		    jointPoses = p.calculateInverseKinematics2(self.body_id, endEffectorIds, positions)
		    for j in range(len(freeJoints)):
		         p.resetJointState(self.body_id, freeJoints[i], jointPoses[i])
		    iter = iter + 1


	def simple_move(self, case):
		t = time.time()
		spine_link = self.op.get_link_index('human/spine_2')
		spine_side_link = self.op.get_link_index('human/spine')
		left_shoulder_1 = self.op.get_link_index('human/left_shoulder_1')
		right_shoulder_1 = self.op.get_link_index('human/right_shoulder_1')
		left_shoulder_0 = self.op.get_link_index('human/left_shoulder_0')
		right_shoulder_0 = self.op.get_link_index('human/right_shoulder_0')
		left_elbow_link = self.op.get_link_index('human/left_elbow')
		right_elbow_link = self.op.get_link_index('human/right_elbow')

		if case == 'spine_swing':
                    p.resetJointState(self.body_id, spine_link, math.sin(t))

		elif case == 'forearm_roll':
                    p.resetJointState(self.body_id, left_shoulder_1, -1.75*math.pi/4)
                    p.resetJointState(self.body_id, right_shoulder_1, 1.75*math.pi/4)
                    p.resetJointState(self.body_id, left_elbow_link, (((math.sin(3*t)+1)/8) + (11/8))*math.pi)
                    p.resetJointState(self.body_id, left_shoulder_0, ((-(math.cos(3*t)+1)/8) + (1/8))*math.pi)
                    p.resetJointState(self.body_id, right_elbow_link, -(((math.sin(3*t+math.pi)+1)/8) + (11/8))*math.pi)
                    p.resetJointState(self.body_id, right_shoulder_0, ((-(math.cos(3*t+math.pi)+1)/8) + (1/8))*math.pi)

		elif case == 'arm_roll':
                    p.resetJointState(self.body_id, left_shoulder_1, math.sin(t))
                    p.resetJointState(self.body_id, left_shoulder_0, math.sin(t+math.pi/2))
                    p.resetJointState(self.body_id, right_shoulder_1, -math.sin(t))
                    p.resetJointState(self.body_id, right_shoulder_0, math.sin(t+math.pi/2))

		elif case == 'side_swing':
                    spine_state = p.getLinkState(self.body_id, spine_side_link)
                    print('spine_state is:  ', spine_state, '\n')
                    p.resetJointState(self.body_id, spine_side_link, math.sin(t))
		    # TODO: Connect chest orientation to cage rotation!
		    

		else:
		    print('ERROR: No definition set for case = ', case)


