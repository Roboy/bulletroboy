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

lefthand_link = 25
# position, orientation = p.getBasePositionAndOrientation(human)
# orientation


# NUMBER OF JOINTS = 43 (However most of them are visualized at the base... strange)
joints = p.getNumJoints(human)
print("The number of Joints is: ", joints)

for i in range(joints):
	info = p.getJointInfo(human, i)
	print(info)
info = p.getJointInfo(human, lefthand_link)
print(info)

# NUMBER OF CONSTRAINTS = 0 (Unless I uncomments createConstraint function)
cons = p.getNumConstraints()
print("The number of Constraints is: ", cons)

for i in range(cons):
	info = p.getConstraintInfo(i)
	print(info)

# GET LINK STATES, JOINT STATES
linkstate = p.getLinkState(human, lefthand_link)
print("The link state or link lefthand_link is: ", linkstate)
print("\n")

jointinfo = p.getJointInfo(human, lefthand_link)
print("The joint info or link lefthand_link is: ", jointinfo)

# NUMBER OF BODIES = 2 (0 = plane, 1 = human)
bodies = p.getNumBodies()
print("The number of bodies is: ", bodies)
"""for i in range(bodies):
	info = p.getBodyInfo(i)
	body_id = p.getBodyUniqueId(i)
	print("Info: ",info)
	print("This body's Id is: ", body_id)"""


# COMMON SETUP
p.setGravity(0, 0, -9.81)
# p.setTimeStep(-0.0001)
p.setRealTimeSimulation(0) # XXX
color = [1, 0, 0]

# IMPORTANT JOINT INDICES:

"""
00: Waist
01: Base - between the feet (on the floor)
04: Upper Chest
07: Neck
08: Head
12: Right Shoulder
13: Right Elbow
16: Right Wrist
21: Left Shoulder
22: Left Elbow
lefthand_link: Left Wrist
30: Right Leg (Point where it joins the waist)
31: Right Knee
33: Right Foot
38: Left Leh (Point where it joins the waist)
39: Left Knee
41: Left Foot
"""




#################################################################################
#########                                                         ###############
#########     ALONA: THE QUESTIONS ARE ABOUT THE CODE BELOW :)    ###############
#########                                                         ###############
#################################################################################



# THIS DEBUGLINE IS ATTACHED TO THE WRIST BUT POINTS ARE DEFINED IN LINK FRAME
lefthand_pos = p.getLinkState(human, lefthand_link)[0]
leftHand_tendon1 = p.addUserDebugLine(lineToXYZ=lefthand_pos,	# starting point = wrist link frame origin
		   lineFromXYZ=[1,1,1],				# ending point : how to set it to world frame?
                   lineColorRGB= [1,0,0],
                   lineWidth=3)
leftHand_tendon2 = p.addUserDebugLine(lineToXYZ=lefthand_pos,	# starting point = wrist link frame origin
		   lineFromXYZ=[1,1,-1],			# ending point : how to set it to world frame?
                   lineColorRGB= [1,0,0],
                   lineWidth=3)

# THE USERDEBUGLINE BELOW HAS BOTH STARTING AND ENDING POINTS FIXED IN THE WORLD FRAME
"""
p.addUserDebugLine(lineFromXYZ=[0,0.8,1.35],
		   lineToXYZ=[0,0.8,2],
                   lineColorRGB= [1,0,0],
                   lineWidth=3)"""

# IF I UNCOMMENT THE CONSTRAINT (CREATE IT), THEN THE FIGURE LOSES CONTROL
"""
p.createConstraint(human,		# ParentBodyUniqueId
		   22,			# ParentLinkIndex
		   lefthand_link,			# childBodyUniqueId
		   lefthand_link,			# childLinkIndex
		   4,			# Join Type: JOINT_GEAR
		   [0,0,1],		# Joint Axis
		   [0,0,0],		# ParentFramePosition
		   [0,0,0])		# childFramePosition"""

# THIS IS THE BIGGEST ISSUE, THE FIGURE DOES NOT REACT TO THE FUNCTION APPLYEXTERNALFORCE

# p.applyExternalForce(objectUniqueId=1,
# 		     linkIndex = lefthand_link,
# 		     forceObj = [100,100,100],
# 		     posObj=[0,0,0],
# 		     flags=2)		# 2 Flags: WORLD_FRAME and LINK_FRAME

# THE CODE LINE BELOW RECEIVES THE SAME REACTION AS THE CREATECONSTRAINT FUNCTION:
# p.setJointMotorControlArray(human, (21,22,lefthand_link), p.POSITION_CONTROL)

lefthand_pos = p.getLinkState(human, lefthand_link)[0]

leftHand_tendon1_origin = [lefthand_pos[0]+0.5, lefthand_pos[1]+0.5, 2.0]
leftHand_tendon2_origin = [lefthand_pos[0]+0.5, lefthand_pos[1]+0.5, 0.0]
# motor_force = 50
forceId1 = p.addUserDebugParameter("leftHand_tendon1", 0,200,0)
forceId2 = p.addUserDebugParameter("leftHand_tendon2", 0,200,0)

for _ in range(100000000):
	# import pdb; pdb.set_trace()
	lefthand_pos = p.getLinkState(human, lefthand_link)[0]
	motor_force1 = p.readUserDebugParameter(forceId1)
	motor_force2 = p.readUserDebugParameter(forceId2)


	p.addUserDebugLine(lineToXYZ=lefthand_pos,		        # XXX
			   lineFromXYZ=leftHand_tendon1_origin,		# XXX
	                   lineColorRGB= [0,0,255],
	                   lineWidth=2,
			   replaceItemUniqueId=leftHand_tendon1)         # XXX

	p.addUserDebugLine(lineToXYZ=lefthand_pos,		        # XXX
			   lineFromXYZ=leftHand_tendon2_origin,		# XXX
	                   lineColorRGB= [0,0,255],
	                   lineWidth=2,
			   replaceItemUniqueId=leftHand_tendon2)         # XXX

	force_vector1 = [a_i - b_i for a_i, b_i in zip(leftHand_tendon1_origin,lefthand_pos)]
	force_components1 = [motor_force1 * x/norm(force_vector1) for x in force_vector1]

	force_vector2 = [a_i - b_i for a_i, b_i in zip(leftHand_tendon2_origin,lefthand_pos)]
	force_components2 = [motor_force2 * x/norm(force_vector2) for x in force_vector2]

	p.applyExternalForce(objectUniqueId=human,              # XXX
			     linkIndex = lefthand_link,
			     forceObj = force_components1,      # XXX
			     posObj=lefthand_pos,			# XXX
			     flags=p.WORLD_FRAME)		# XXX

	p.applyExternalForce(objectUniqueId=human,              # XXX
			     linkIndex = lefthand_link,
			     forceObj = force_components2,      # XXX
			     posObj=lefthand_pos,			# XXX
			     flags=p.WORLD_FRAME)		# XXX

	p.stepSimulation()
