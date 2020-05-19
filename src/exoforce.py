#######################################################################################
###############                                                         ###############
###############             BASIC SETUP AND LOADING THE BODY            ###############
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


#######################################################################################
###############                                                         ###############
###############             EXTRA SETUP AND BODY INFORMATION            ###############
###############                                                         ###############
#######################################################################################


# IMPORTANT JOINT INDICES:

"""
00: Waist
01: Base - between the feet (on the floor)
04: Upper Chest
07: Neck
08: Head
12: Right Shoulder
13: Right Elbow
16: Right Wrist	 -------------------------------------------->   righthand_link
21: Left Shoulder
22: Left Elbow
25: Left Wrist	--------------------------------------------->   lefthand_link
30: Right Leg (Point where it joins the waist)
31: Right Knee
33: Right Foot
38: Left Leh (Point where it joins the waist)
39: Left Knee
41: Left Foot
"""

# LINK DEFINITIONS
righthand_link = 16
lefthand_link = 25

# COLOR DEFINITION
line_color = [0,0,255]

# NUMBER OF JOINTS = 43 (However most of them are visualized at the base... strange)
joints = p.getNumJoints(human)
print("The number of Joints is: ", joints)

for i in range(joints):
	info = p.getJointInfo(human, i)
	print(info)
info = p.getJointInfo(human, lefthand_link)
print(info)

# NUMBER OF CONSTRAINTS = 0 (Initially - later constraints can be set)
cons = p.getNumConstraints()
print("The number of Constraints is: ", cons)

for i in range(cons):
	info = p.getConstraintInfo(i)
	print(info)

# GET LINK AND JOINT STATES
linkstate = p.getLinkState(human, lefthand_link)
print("The link state or link lefthand_link is: ", linkstate)
print("\n")

jointinfo = p.getJointInfo(human, lefthand_link)
print("The joint info or link lefthand_link is: ", jointinfo)
print("\n")

# NUMBER OF BODIES = 2 (0 = plane, 1 = human)
bodies = p.getNumBodies()
print("The number of bodies is: ", bodies)
"""for i in range(bodies):
	info = p.getBodyInfo(i)
	body_id = p.getBodyUniqueId(i)
	print("Info: ",info)
	print("This body's Id is: ", body_id)"""


# COMMON WORLD SETUP
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)  # Requirement for p.applyExternalForce() to work     # XXX
color = [1, 0, 0]




######################################################################################
###########                                                              #############
###########    DEBUGLINES, PARAMS, CONSTRAINTS AND FORCE DEFINITIONS     #############
###########                                                              #############
######################################################################################


# LINK POSITION DEFINITIONS:
lefthand_pos = p.getLinkState(human, lefthand_link)[0]
righthand_pos = p.getLinkState(human, righthand_link)[0]

# TENDON ORIGIN DEFINITIONS:
leftHand_tendon1_origin = [lefthand_pos[0]+0.5, lefthand_pos[1]+0.5, 2.0]
leftHand_tendon2_origin = [lefthand_pos[0]+0.5, lefthand_pos[1]+0.5, 0.0]
rightHand_tendon1_origin = [righthand_pos[0]+0.5, righthand_pos[1]-0.5, 2.0]
rightHand_tendon2_origin = [righthand_pos[0]+0.5, righthand_pos[1]-0.5, 0.0]

# FORCE DEFINITIONS:
forceId1 = p.addUserDebugParameter("leftHand_tendon1", 0,200,0)
forceId2 = p.addUserDebugParameter("leftHand_tendon2", 0,200,0)
forceId3 = p.addUserDebugParameter("rightHand_tendon1", 0,200,0)
forceId4 = p.addUserDebugParameter("rightHand_tendon2", 0,200,0)

# DEBUGLINE PRE-DEFINITIONS:
leftHand_tendon1 = p.addUserDebugLine(lineToXYZ=lefthand_pos,
		                      lineFromXYZ=leftHand_tendon1_origin,
                                      lineColorRGB=line_color,
                                      lineWidth=2)
leftHand_tendon2 = p.addUserDebugLine(lineToXYZ=lefthand_pos,
		   		      lineFromXYZ=leftHand_tendon2_origin,
                                      lineColorRGB=line_color,
                                      lineWidth=2)
rightHand_tendon1 = p.addUserDebugLine(lineToXYZ=righthand_pos,
		                       lineFromXYZ=rightHand_tendon1_origin,
                                       lineColorRGB=line_color,
                                       lineWidth=2)
rightHand_tendon2 = p.addUserDebugLine(lineToXYZ=righthand_pos,
		   		       lineFromXYZ=rightHand_tendon2_origin,
                                       lineColorRGB=line_color,
                                       lineWidth=2)

# CONSTRAINTS:

"""Code below fixes the chest in place (In case it is needed for testing)
p.createConstraint(human,		# ParentBodyUniqueId
		   2,			# ParentLinkIndex
		   -1,			# childBodyUniqueId
		   3,			# childLinkIndex
		   p.JOINT_FIXED,	# Join Type: JOINT_GEAR
		   [0, 0, 0],		# Joint Axis
		   [0, 0, 0],		# ParentFramePosition
		   [0, 0, 1])		# childFramePosition"""

"""Code below (together with the one above) keeps the head fixed as well:
p.createConstraint(human,
		   8,
		   -1,
		   3,
		   p.JOINT_FIXED,
		   [0, 0, 0],
		   [0, 0, 0],
		   [0, 0, 1])	"""	



######################################################################################
###########                                                              #############
###########    ACTUAL SIMULATION ---> DEBUGLINES AND EXTERNAL FORCES     #############
###########                                                              #############
######################################################################################



for _ in range(100000000):
	
	# GET LINK CURRENT LINK POSITIONS:
	lefthand_pos = p.getLinkState(human, lefthand_link)[0]
	righthand_pos = p.getLinkState(human, righthand_link)[0]

	# READ MOTOR FORCES
	motor_force1 = p.readUserDebugParameter(forceId1)
	motor_force2 = p.readUserDebugParameter(forceId2)
	motor_force3 = p.readUserDebugParameter(forceId3)
	motor_force4 = p.readUserDebugParameter(forceId4)

	# UPDATE DEBUGLINES:
	p.addUserDebugLine(lineToXYZ=lefthand_pos,		        # XXX
			   lineFromXYZ=leftHand_tendon1_origin,		# XXX
	                   lineColorRGB=line_color,
	                   lineWidth=2,
			   replaceItemUniqueId=leftHand_tendon1)        # XXX

	p.addUserDebugLine(lefthand_pos,leftHand_tendon2_origin,line_color,2,replaceItemUniqueId=leftHand_tendon2)
	p.addUserDebugLine(righthand_pos,rightHand_tendon1_origin,line_color,2,replaceItemUniqueId=rightHand_tendon1)
	p.addUserDebugLine(righthand_pos,rightHand_tendon2_origin,line_color,2,replaceItemUniqueId=rightHand_tendon2)
	
	# DEFINE FORCE VECTORS (DIRECTION: LINE BETWEEN TENDON ORIGIN AND LINK POSITION):
	force_vector1 = [a_i - b_i for a_i, b_i in zip(leftHand_tendon1_origin,lefthand_pos)]
	force_vector2 = [a_i - b_i for a_i, b_i in zip(leftHand_tendon2_origin,lefthand_pos)]
	force_vector3 = [a_i - b_i for a_i, b_i in zip(rightHand_tendon1_origin,righthand_pos)]
	force_vector4 = [a_i - b_i for a_i, b_i in zip(rightHand_tendon2_origin,righthand_pos)]

	# PREPARE FORCE COMPONENTS (forceObj variable for force function):
	force_components1 = [motor_force1 * x/norm(force_vector1) for x in force_vector1]
	force_components2 = [motor_force2 * x/norm(force_vector2) for x in force_vector2]
	force_components3 = [motor_force3 * x/norm(force_vector3) for x in force_vector3]
	force_components4 = [motor_force4 * x/norm(force_vector4) for x in force_vector4]

	
	# PULLING THE TENDONS (EXTERNAL FORCE)
	p.applyExternalForce(objectUniqueId=human,              # XXX
			     linkIndex = lefthand_link,
			     forceObj = force_components1,      # XXX
			     posObj=lefthand_pos,		# XXX
			     flags=p.WORLD_FRAME)		# XXX

	p.applyExternalForce(human,lefthand_link,force_components2,lefthand_pos,p.WORLD_FRAME)
	p.applyExternalForce(human,righthand_link,force_components3,righthand_pos,p.WORLD_FRAME)
	p.applyExternalForce(human,righthand_link,force_components4,righthand_pos,p.WORLD_FRAME)

	p.stepSimulation()
