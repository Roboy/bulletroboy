import argparse
import os

import pybullet as p
import pybullet_data
import math
import time
from numpy import sum
from numpy.linalg import norm
from sympy import Point3D, Line3D, Plane


class HumanFigure():
    def __init__(self, body_id):
        self.body_id = body_id
        self.t = 0.
        self.prevPose = [0, 0, 0]
        self.prevPose1 = [0, 0, 0]
        self.hasPrevPose = 0
        self.ikSolver = 0
        #trailDuration is duration (in seconds) after debug lines will be removed automatically
        #use 0 for no-removal
        self.trailDuration = 15
        numJoints = p.getNumJoints(self.body_id)
        self.freeJoints = []

        for i in range(numJoints):
            info = p.getJointInfo(self.body_id,i)
            if info[2] == p.JOINT_REVOLUTE:
                self.freeJoints.append(i)
            if info[12] == b'human/left_hand':
                self.endEffectorId = i;
                #print("EF id: " + str(i))

    def accurateCalculateInverseKinematics(self, targetPos, threshold, maxIter):
      closeEnough = False
      iter = 0
      dist2 = 1e30
      while (not closeEnough and iter < maxIter):
        jointPoses = p.calculateInverseKinematics(self.body_id, self.endEffectorId, targetPos)
        #import pdb; pdb.set_trace()
        for i in range(len(self.freeJoints)):
          p.resetJointState(self.body_id, self.freeJoints[i], jointPoses[i])
        ls = p.getLinkState(self.body_id, self.endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
      #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
      return jointPoses

    def drawDebugLines(self, targetPos):
        # drawing debug lines
        ls = p.getLinkState(self.body_id, self.endEffectorId)
        if (self.hasPrevPose):
            p.addUserDebugLine(self.prevPose, targetPos, [0, 0, 0.3], 1, self.trailDuration)
            p.addUserDebugLine(self.prevPose1, ls[4], [1, 0, 0], 1, self.trailDuration)
        self.prevPose = targetPos
        self.prevPose1 = ls[4]
        self.hasPrevPose = 1


p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")

human = p.loadURDF(os.path.dirname(os.path.realpath(__file__))+"/../models/human.urdf", [0, 0, 1], useFixedBase=1)

lefthand_link = 25
righthand_link = 16
# position, orientation = p.getBasePositionAndOrientation(human)
# orientation


# NUMBER OF JOINTS = 43 (However most of them are visualized at the base... strange)
joints = p.getNumJoints(human)
print("The number of Joints is: ", joints)

"""for i in range(joints):
	info = p.getJointInfo(human, i)
	print(info)"""
info = p.getJointInfo(human, lefthand_link)
print(info)

# NUMBER OF CONSTRAINTS = 0 (Unless I uncomments createConstraint function)
cons = p.getNumConstraints()
print("The number of Constraints is: ", cons)

"""for i in range(cons):
	info = p.getConstraintInfo(i)
	print(info)"""

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

righthand_pos = p.getLinkState(human, righthand_link)[0]
rightHand_tendon1 = p.addUserDebugLine(lineToXYZ=righthand_pos,	# starting point = wrist link frame origin
		   lineFromXYZ=[1,1,1],				# ending point : how to set it to world frame?
                   lineColorRGB= [1,0,0],
                   lineWidth=3)
rightHand_tendon2 = p.addUserDebugLine(lineToXYZ=righthand_pos,	# starting point = wrist link frame origin
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

"""Code below keeps chest in place
p.createConstraint(human,		# ParentBodyUniqueId
		   2,			# ParentLinkIndex
		   -1,			# childBodyUniqueId
		   3,			# childLinkIndex
		   p.JOINT_FIXED,	# Join Type: JOINT_GEAR
		   [0, 0, 0],		# Joint Axis
		   [0, 0, 0],		# ParentFramePosition
		   [0, 0, 1])		# childFramePosition"""

"""Code below keeps head from falling
p.createConstraint(human,8,-1,3,p.JOINT_FIXED,[0, 0, 0],[0, 0, 0],[0, 0, 1.0])	"""	


# THIS IS THE BIGGEST ISSUE, THE FIGURE DOES NOT REACT TO THE FUNCTION APPLYEXTERNALFORCE

# p.applyExternalForce(objectUniqueId=1,
# 		     linkIndex = lefthand_link,
# 		     forceObj = [100,100,100],
# 		     posObj=[0,0,0],
# 		     flags=2)		# 2 Flags: WORLD_FRAME and LINK_FRAME

# THE CODE LINE BELOW RECEIVES THE SAME REACTION AS THE CREATECONSTRAINT FUNCTION:
# p.setJointMotorControlArray(human, (21,22,lefthand_link), p.POSITION_CONTROL)

link = []
link.append(lefthand_link)
link.append(righthand_link)

"""link_pos = []
link_pos.append(p.getLinkState(human, link[0])[0])"""

lefthand_pos = p.getLinkState(human, link[0])[0]
righthand_pos = p.getLinkState(human, link[1])[0]

#print("HERE COMES THE LINK_POS[0]", link_pos[0])
#print('\n')
#print("HERE COMES THE LEFTHAND_POS", lefthand_pos)
#print('\n')

leftHand_tendon1_origin = [lefthand_pos[0], lefthand_pos[1]+0.5, 2.0]
leftHand_tendon2_origin = [lefthand_pos[0], lefthand_pos[1]+0.5, 0.0]

rightHand_tendon1_origin = [righthand_pos[0], righthand_pos[1]-0.5, 2.0]
rightHand_tendon2_origin = [righthand_pos[0], righthand_pos[1]-0.5, 0.0]

forceId1 = p.addUserDebugParameter("leftHand_tendon1", 0,200,0)
forceId2 = p.addUserDebugParameter("leftHand_tendon2", 0,200,0)
forceId3 = p.addUserDebugParameter("rightHand_tendon1", 0,200,0)
forceId4 = p.addUserDebugParameter("rightHand_tendon2", 0,200,0)

for _ in range(100000000):
	# import pdb; pdb.set_trace()
	lefthand_pos = p.getLinkState(human, lefthand_link)[0]
	righthand_pos = p.getLinkState(human, righthand_link)[0]

	motor_force1 = p.readUserDebugParameter(forceId1)
	motor_force2 = p.readUserDebugParameter(forceId2)
	motor_force3 = p.readUserDebugParameter(forceId3)
	motor_force4 = p.readUserDebugParameter(forceId4)

	# t = time.time()
	# pos = [0.5, 0.2 * math.cos(t), 0.2 * math.sin(t) + 1.2]
	# threshold = 0.001
	# maxIter = 100
	# hf = HumanFigure(human)
	# hf.accurateCalculateInverseKinematics(pos, threshold, maxIter)


	p.addUserDebugLine(lineToXYZ=lefthand_pos,		        # XXX
			   lineFromXYZ=leftHand_tendon1_origin,		# XXX
	                   lineColorRGB= [0,0,255],
	                   lineWidth=2,
			   replaceItemUniqueId=leftHand_tendon1)        # XXX

	p.addUserDebugLine(lineToXYZ=lefthand_pos,		        
			   lineFromXYZ=leftHand_tendon2_origin,		
	                   lineColorRGB= [0,0,255],
	                   lineWidth=2,
			   replaceItemUniqueId=leftHand_tendon2)    

	p.addUserDebugLine(lineToXYZ=righthand_pos,		        
			   lineFromXYZ=rightHand_tendon1_origin,		
	                   lineColorRGB= [0,0,255],
	                   lineWidth=2,
			   replaceItemUniqueId=rightHand_tendon1) 

	p.addUserDebugLine(lineToXYZ=righthand_pos,		        
			   lineFromXYZ=rightHand_tendon2_origin,		
	                   lineColorRGB= [0,0,255],
	                   lineWidth=2,
			   replaceItemUniqueId=rightHand_tendon2)  

	force_vector1 = [a_i - b_i for a_i, b_i in zip(leftHand_tendon1_origin,lefthand_pos)]
	force_vector2 = [a_i - b_i for a_i, b_i in zip(leftHand_tendon2_origin,lefthand_pos)]
	force_vector3 = [a_i - b_i for a_i, b_i in zip(rightHand_tendon1_origin,righthand_pos)]
	force_vector4 = [a_i - b_i for a_i, b_i in zip(rightHand_tendon2_origin,righthand_pos)]

	force_components1 = [motor_force1 * x/norm(force_vector1) for x in force_vector1]
	force_components2 = [motor_force2 * x/norm(force_vector2) for x in force_vector2]
	force_components3 = [motor_force3 * x/norm(force_vector3) for x in force_vector3]
	force_components4 = [motor_force4 * x/norm(force_vector4) for x in force_vector4]

	p.applyExternalForce(objectUniqueId=human,              # XXX
			     linkIndex = lefthand_link,
			     forceObj = force_components1,      # XXX
			     posObj=lefthand_pos,		# XXX
			     flags=p.WORLD_FRAME)		# XXX

	p.applyExternalForce(objectUniqueId=human,              
			     linkIndex = lefthand_link,
			     forceObj = force_components2,      
			     posObj=lefthand_pos,		
			     flags=p.WORLD_FRAME)

	p.applyExternalForce(objectUniqueId=human,              
			     linkIndex = righthand_link,
			     forceObj = force_components3,      
			     posObj=righthand_pos,		
			     flags=p.WORLD_FRAME)

	p.applyExternalForce(objectUniqueId=human,              
			     linkIndex = righthand_link,
			     forceObj = force_components4,      
			     posObj=righthand_pos,		
			     flags=p.WORLD_FRAME)		

	p.stepSimulation()
