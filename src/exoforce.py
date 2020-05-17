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

hand_link = 25
# position, orientation = p.getBasePositionAndOrientation(human)
# orientation


# NUMBER OF JOINTS = 43 (However most of them are visualized at the base... strange)
joints = p.getNumJoints(human)
print("The number of Joints is: ", joints)

for i in range(joints):
	info = p.getJointInfo(human, i)
	print(info)
info = p.getJointInfo(human, hand_link)
print(info)

# NUMBER OF CONSTRAINTS = 0 (Unless I uncomments createConstraint function)
cons = p.getNumConstraints()
print("The number of Constraints is: ", cons)

for i in range(cons):
	info = p.getConstraintInfo(i)
	print(info)

# GET LINK STATES, JOINT STATES
linkstate = p.getLinkState(human, hand_link)
print("The link state or link hand_link is: ", linkstate)
print("\n")

jointinfo = p.getJointInfo(human, hand_link)
print("The joint info or link hand_link is: ", jointinfo)

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
hand_link: Left Wrist
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
hand_pos = p.getLinkState(human, hand_link)[0]
tendon = p.addUserDebugLine(lineToXYZ=hand_pos,				# starting point = wrist link frame origin
		   lineFromXYZ=[1,1,1],			# ending point : how to set it to world frame?
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
		   hand_link,			# childBodyUniqueId
		   hand_link,			# childLinkIndex
		   4,			# Join Type: JOINT_GEAR
		   [0,0,1],		# Joint Axis
		   [0,0,0],		# ParentFramePosition
		   [0,0,0])		# childFramePosition"""

# THIS IS THE BIGGEST ISSUE, THE FIGURE DOES NOT REACT TO THE FUNCTION APPLYEXTERNALFORCE

# p.applyExternalForce(objectUniqueId=1,
# 		     linkIndex = hand_link,
# 		     forceObj = [100,100,100],
# 		     posObj=[0,0,0],
# 		     flags=2)		# 2 Flags: WORLD_FRAME and LINK_FRAME

# THE CODE LINE BELOW RECEIVES THE SAME REACTION AS THE CREATECONSTRAINT FUNCTION:
# p.setJointMotorControlArray(human, (21,22,hand_link), p.POSITION_CONTROL)
hand_pos = p.getLinkState(human, hand_link)[0]
tendon_origin = [hand_pos[0]+0.5, hand_pos[1]+0.5, 3.0]
# motor_force = 50
forceId = p.addUserDebugParameter("motor_force", 0,200,0)

for _ in range(100000000):
	# import pdb; pdb.set_trace()
	hand_pos = p.getLinkState(human, hand_link)[0]
	motor_force = p.readUserDebugParameter(forceId)

	p.addUserDebugLine(lineToXYZ=hand_pos,		# XXX
			   			lineFromXYZ=tendon_origin,		# XXX
	                    lineColorRGB= [0,0,255],
	                    lineWidth=2,
					    replaceItemUniqueId=tendon) # XXX

	force_vector = [a_i - b_i for a_i, b_i in zip(tendon_origin,hand_pos)]
	force_components = [motor_force * x/norm(force_vector) for x in force_vector]
	p.applyExternalForce(objectUniqueId=human, # XXX
			     linkIndex = hand_link,
			     forceObj = force_components, # XXX
			     posObj=hand_pos,			# XXX
			     flags=p.WORLD_FRAME )		# XXX

	p.stepSimulation()
