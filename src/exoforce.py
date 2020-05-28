import os

import pybullet as p
import pybullet_data
import math
import numpy as np
from numpy.linalg import norm
from sympy import Plane
from xml_handler import CageConfiguration

#######################################################
###############                         ###############
###############    CLASS DEFINITIONS    ###############
###############                         ###############
#######################################################


class HumanFigure():
	def __init__(self, body_id):
		self.body_id = body_id

	def find_link_number(self, link_name):
		numJoints = p.getNumJoints(self.body_id)
		link = None
		for i in range(numJoints):
			info = p.getJointInfo(self.body_id,i)
			if info[12] == str.encode(link_name):
				link = i
		return link


class Tendon():
	def __init__(self, name, muscle, human, link_name, via_point):
		self.name = name
		self.muscle = muscle
		self.human_id = human.body_id
		self.link = human.find_link_number(link_name)
		self.via_point = np.array(via_point)
		self.forceId = None
		self.start_location = np.asarray(p.getLinkState(self.human_id, self.link)[0]) + self.via_point
		self.end_location = self.muscle.pos
		self.tendon = p.addUserDebugLine(self.start_location, self.end_location, lineColorRGB=[0,0,255], lineWidth=2)
		self.text = p.addUserDebugText(self.name, self.end_location, textColorRGB=[0,0,255], textSize=0.8)

	def apply_force(self, force):
		force_direction = np.asarray(self.end_location) - np.asarray(self.start_location)
		force_direction = force_direction / norm(force_direction)

		p.applyExternalForce(objectUniqueId=self.human_id,
			     	     linkIndex = self.link,
			     	     forceObj = force * force_direction,
			     	     posObj=self.start_location,
			     	     flags=p.WORLD_FRAME)

	def update(self, force):
		self.start_location = np.asarray(p.getLinkState(self.human_id, self.link)[0]) + self.via_point

		p.addUserDebugLine(self.start_location,self.end_location,lineColorRGB=[0,0,255], lineWidth=2, replaceItemUniqueId=self.tendon)
		p.addUserDebugText(self.name, self.end_location, textColorRGB=[0,0,255], textSize=0.8, replaceItemUniqueId=self.text)

		self.apply_force(force)


class Cage():
	def __init__(self, height, radius, muscle_units):

		self.origin = [0, 0, 0]
		self.height = height
		self.radius = radius
		self.angle = 0

		self.muscle_units = [ Muscle(muscle['name'], muscle['height'], muscle['angle'], self.radius, muscle['parameters']) for muscle in muscle_units ]

	def rotate_cage(self, new_angle):

		for muscle in self.muscle_units:
			muscle.rotate(self.angle - new_angle, self.origin)

		self.angle = new_angle


class Muscle():
	def __init__(self, name, height, angle, radius, parameters):
		
		self.name = name
		self.height = height
		self.angle = angle
		self.radius = radius
		self.parameters = parameters

		self.pos = self.get_pos()

	def get_pos(self):

		x = self.radius * math.cos(math.radians(self.angle))
		y = self.radius * math.sin(math.radians(self.angle))
		z = self.height

		return [x, y, z]

	def rotate(self, delta, origin):

		self.angle += delta

		x, y = self.pos[:2]
		offset_x, offset_y = origin[:2]
		adjusted_x = (x - offset_x)
		adjusted_y = (y - offset_y)
		cos_rad = math.cos(math.radians(delta))
		sin_rad = math.sin(math.radians(delta))
		qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
		qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y
		
		self.pos[0] = qx
		self.pos[1] = qy


class ExoForce():
	def __init__(self, cage_conf, human):

		self.human = human
		self.cage = Cage(cage_conf.cage_structure['height'], cage_conf.cage_structure['radius'], cage_conf.muscleUnits)
		self.tendons = [ Tendon(tendon['name'], self.get_muscle(tendon['muscle']), self.human, tendon['link'], tendon['viaPoint'])
							for tendon in cage_conf.tendons ]

	def step(self, new_angle, motor_forces):

		self.cage.rotate_cage(new_angle)

		for tendon, force in zip(self.tendons, motor_forces):
			tendon.update(force)

	def get_cage(self):
		return self.cage

	def get_tendons(self):
		return self.tendons

	def get_muscle_units(self):
		return self.cage.muscle_units

	def get_tendon(self, tendon_name):
		for tendon in self.tendons:
			if tendon.name == tendon_name:
				return tendon
		return None

	def get_muscle(self, muscle_name):
		for muscle in self.cage.muscle_units:
			if muscle.name == muscle_name:
				return muscle
		return None


#######################################################################################
###############                                                         ###############
###############                     SIMULATION SETUP                    ###############
###############                                                         ###############
#######################################################################################

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
human = p.loadURDF(os.path.dirname(os.path.realpath(__file__))+"/../models/human.urdf", [0, 0, 1], useFixedBase=1)

# COMMON WORLD SETUP
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)     # Don't change. Else p.applyExternalForce() won't work.

# EXOFORCE SETUP
initial_cage_conf = CageConfiguration("../config/cageConfiguration.xml")
human_figure = HumanFigure(human)

exoforce = ExoForce(initial_cage_conf, human_figure)

for tendon in exoforce.tendons:
	tendon.forceId = p.addUserDebugParameter("Force in " + tendon.name, 0, 200, 0)

cage_angle_id = p.addUserDebugParameter("Cage Angle", -180, 180, 0)

######################################################################################
###########                                                              #############
###########                          RUN SIMULATION                      #############
###########                                                              #############
######################################################################################

try:
	while True:
		cage_angle = p.readUserDebugParameter(cage_angle_id)

		motor_forces = []
		for tendon in exoforce.tendons:
			motor_forces.append(p.readUserDebugParameter(tendon.forceId))
		
		exoforce.step(cage_angle, motor_forces)

		p.stepSimulation()

except KeyboardInterrupt:
	pass
	