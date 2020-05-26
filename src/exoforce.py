#######################################################################################
###############                                                         ###############
###############    BASIC SETUP, LOADING THE BODY & CLASS DEFINITIONS    ###############
###############                                                         ###############
#######################################################################################

import argparse
import os

import pybullet as p
import pybullet_data
import math
import time
from numpy import sum
from numpy.linalg import norm
from sympy import Point3D, Line3D, Plane

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
human = p.loadURDF(os.path.dirname(os.path.realpath(__file__))+"/../models/human.urdf", [0, 0, 1], useFixedBase=1)


class humanFigure():
	def __init__(self, body_id):
		self.body_id = body_id

	def find_link_number(self, link_name):
		numJoints = p.getNumJoints(self.body_id)
		link = None
		for i in range(numJoints):
			info = p.getJointInfo(self.body_id,i)
			if info[12] == link_name:
				link = i;
		return link


class Tendon():
	def __init__(self, body_id, link, end_displacement, i):
		self.body_id = body_id
		self.link = link
		self.i = i
		self.end_displacement = end_displacement
		self.forceId = p.addUserDebugParameter(str("tendon " + str(self.i)), 0,200,0)
		self.start_location = p.getLinkState(self.body_id, self.link)[0]
		self.end_location = [self.start_location[0]+self.end_displacement[0],
				     self.start_location[1]+self.end_displacement[1],
				     self.start_location[2]+self.end_displacement[2]]
		self.tendon = p.addUserDebugLine(self.start_location,
		                      		 self.end_location,
                                      		 lineColorRGB=[0,0,255],
                                      		 lineWidth=2)

	def decompose_force(self):
		motor_force = p.readUserDebugParameter(self.forceId)
		force_vector = [a_i - b_i for a_i, b_i in zip(self.end_location,self.start_location)]
		self.force_components = [motor_force * x/norm(force_vector) for x in force_vector]

	def apply_force(self):
		p.applyExternalForce(objectUniqueId=self.body_id,
			     	     linkIndex = self.link,
			     	     forceObj = self.force_components,
			     	     posObj=self.start_location,
			     	     flags=p.WORLD_FRAME)

	def update(self):
		self.start_location = p.getLinkState(self.body_id, self.link)[0]
		p.addUserDebugLine(self.start_location,self.end_location,lineColorRGB=[0,0,255],
				   lineWidth=2, replaceItemUniqueId=self.tendon)
		self.decompose_force()
		self.apply_force()
		
#######################################################################################
###############                                                         ###############
###############               LINK, WORLD AND TENDON SETUP              ###############
###############                                                         ###############
#######################################################################################


# LINK DEFINITIONS
hf = humanFigure(human)

lefthand_link = hf.find_link_number(b'human/left_wrist')
righthand_link = hf.find_link_number(b'human/right_wrist')

# COMMON WORLD SETUP
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)     # Don't change. Else p.applyExternalForce() won't work.

# TENDONS:
tendon_list = []

tendon_list.append(Tendon(human, lefthand_link, [0.5, 0.5, 1.0], 1))
tendon_list.append(Tendon(human, lefthand_link, [0.5, 0.5, -1.0], 2))
tendon_list.append(Tendon(human, righthand_link, [0.5, -0.5, 1.0], 3))
tendon_list.append(Tendon(human, righthand_link, [0.5, -0.5, -1.0], 4))


######################################################################################
###########                                                              #############
###########      ACTUAL SIMULATION (UPDATING DEBUGLINES AND FORCES)      #############
###########                                                              #############
######################################################################################

try:
	while True:
		for i in range(len(tendon_list)):
			tendon_list[i].update()

		p.stepSimulation()

except KeyboardInterrupt:
	pass
	
	
