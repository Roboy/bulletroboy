import pybullet as p
import time
import math
import numpy as np
from enum import Enum

from ..utils.utils import draw_AABB
from .operator import Operator, Link

class OperatorSim(Operator):
	"""This class handles the operator body and its links in the simulation.

	"""
	def __init__(self, body_id):
		"""
		Args:
			body_id (int): Pybullet body indentifier.

		"""
		super().__init__()
		self.body_id = body_id

	def start_node(self):
		self.movements = Movements(self)
		self.prevPose = [0, 0, 0]
		self.trailDuration = 5

		p.createConstraint(self.body_id, -1, -1, -1, p.JOINT_FIXED, [0,0,0],[0,0,0],[0,0,0],[0,0,0,1])
		p.createConstraint(self.body_id, self.get_link_index('chest'), self.body_id, -1, p.JOINT_FIXED, [0, 0, 90], [0, 0, 1], [0, 0, 1])
		self.init_joint_motors()

		self.start_publishing()

	def init_joint_motors(self):
		"""Initializes joint motors.
		
		Args:
			-

		Returns:
		   -

		"""
		for j in range(p.getNumJoints(self.body_id)):
			p.setJointMotorControlMultiDof(self.body_id, j, p.POSITION_CONTROL, [0,0,0,1], positionGain=0.1, force=[200])

	def get_link_bb_dim(self, link_id):
		"""Gets link bounding box dimensions.
		
		Args:
			link_id (int): Index of the link to search.

		Returns:
			3darray[float]: x, y, and z dimensions of the bounding box.

		"""
		aabb = p.getAABB(self.body_id, link_id)
		aabb = np.array(aabb)
		return aabb[1] - aabb[0]

	def init_links(self):
		"""Initiates pybullet's links in operator body.
		
		Args:
			-

		Returns:
		   	-

		"""
		self.links = []
		for key in self.link_map:
			human_name = self.link_map[key]
			roboy_name = key
			id = self.get_link_index(human_name)
			dims = self.get_link_bb_dim(id)
			init_pose = p.getLinkState(self.body_id, id)[:2]

			self.links.append(Link(id, human_name, roboy_name, dims, init_pose))

			# self.draw_LF_coordinate_systems(id)
			# utils.draw_AABB(p,p.getAABB(self.body_id, i))

	def get_link_index(self, link_name):
		"""Gets link's index given it's name.
		
		Args:
			link_name (string): Name of the link to search.

		Returns:
		   	int: Index of the given link.

		"""
		index = None
		for i in range(p.getNumJoints(self.body_id)):
			if link_name == str(p.getJointInfo(self.body_id,i)[12], 'utf-8'):
				index = i
				break
		return index
	
	def update_pose(self):
		for link in self.links:
			pos, orn = p.getLinkState(self.body_id, link.id)[0:2]
			link.set_pose(pos, orn)

	def move(self, case):
		"""Applies a movement to the operator.
		
		Args:
			case (int): Movement to apply.

		Returns:
		   	-

		"""
		self.movements.simple_move(case)
	
	def draw_LF_coordinate_systems(self, link_id):
		"""Draws the coordinate system of the link.
		Args: 
			link_id : id of the link.
		"""
		p.addUserDebugLine([0,0,0],[0.3,0,0],[1,0,0],lineWidth= 3, parentObjectUniqueId=self.body_id, parentLinkIndex=link_id)
		p.addUserDebugLine([0,0,0],[0,0.3,0],[0,1,0],lineWidth= 3, parentObjectUniqueId=self.body_id, parentLinkIndex=link_id)
		p.addUserDebugLine([0,0,0],[0,0,0.3],[0,0,1],lineWidth= 3, parentObjectUniqueId=self.body_id, parentLinkIndex=link_id)

	def drawDebugLines(self, ef, pos_or):
		# drawing debug lines
		if(ef == 'left_wrist'):
			p.addUserDebugLine(self.prevPose, pos_or, [0, 0, 0.3], 1, self.trailDuration)
			self.prevPose = pos_or


class Moves(Enum):
	"""This enum class handles the posible moves the operator can perform.

	"""
	STAND_STILL = 1
	SPINE_SWING = 2
	SIDE_SWING = 3
	FOREARM_ROLL = 4
	ARM_ROLL = 5
	CATCH = 6
	HANDS_UP = 7

class Movements():
	"""This class defines 2 types of movements:
		- Re-definable inverse kinematic ones (one_end_effector(), two_end_effectors())
		- Harcoded simple ones (simple_move() --> 4 predefined movements)

	"""
	def __init__(self, operator):
		"""
		Args:
			operator (Operator): Operator object to which the movements will be applied to.

		"""
		self.op = operator

		self.neck = self.op.get_link_index('neck')
		self.chest = self.op.get_link_index('chest')
		self.left_shoulder = self.op.get_link_index('left_shoulder')
		self.right_shoulder = self.op.get_link_index('right_shoulder')
		self.left_elbow = self.op.get_link_index('left_elbow')
		self.right_elbow = self.op.get_link_index('right_elbow')
		self.left_wrist = self.op.get_link_index('left_wrist')
		self.right_wrist = self.op.get_link_index('right_wrist')


	def get_EF_id(self, link_name):
		"""Gets the id of the endEffector and the list of free revolute joints.

		Args:
			link_name (string): Link name to search.
		
		Returns:
		   	-

		"""
		freeJoints = []
		numJoints = p.getNumJoints(self.op.body_id)

		for i in range(numJoints):
			info = p.getJointInfo(self.op.body_id,i)
			if info[2] == p.JOINT_REVOLUTE:
				freeJoints.append(i)
			if info[12] == link_name:
				endEffectorId = i

		return endEffectorId, freeJoints

	def apply_chest_and_neck_constraints(self):
		"""Keeps the chest and neck in a straight position. In specific cases it makes the movement more ergonomic and human-like.

		Args:
			-
		
		Returns:
		   	-

		"""
		# Constraint on the Chest
		p.createConstraint(self.op.body_id, self.chest, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
		# Constraint on the Neck
		p.createConstraint(self.op.body_id, self.neck, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])


	def simple_move(self, case):
		"""Defines a series of hardcoded movements for a direct use. (Movements Library)

		Args:
			case (Moves): The Moves enum corresponding with the movement to be applied.
		
		Returns:
		   -

		"""
		t = time.time()

		if case == Moves.STAND_STILL:
			left_elbow_pos = 0
			right_elbow_pos = 0
			left_shoulder_quat = p.getQuaternionFromEuler([0, 0, 0])
			right_shoulder_quat = p.getQuaternionFromEuler([0, 0, 0])
			chest_quat = p.getQuaternionFromEuler([0, 0, 0])

		if case == Moves.SPINE_SWING:
			left_elbow_pos = 0
			right_elbow_pos = 0
			left_shoulder_quat = p.getQuaternionFromEuler([math.pi/2, 0, 0])
			right_shoulder_quat = p.getQuaternionFromEuler([-math.pi/2, 0, 0])
			chest_quat = p.getQuaternionFromEuler([0, math.sin(t), 0])

		elif case == Moves.SIDE_SWING:
			left_elbow_pos = 0
			right_elbow_pos = 0
			left_shoulder_quat = p.getQuaternionFromEuler([math.pi/2, 0, 0])
			right_shoulder_quat = p.getQuaternionFromEuler([-math.pi/2, 0, 0])
			chest_quat = p.getQuaternionFromEuler([math.sin(t), 0, 0])

		elif case == Moves.FOREARM_ROLL:
			left_elbow_pos =  math.pi/2 + 2*math.sin(t)/3
			right_elbow_pos = math.pi/2 - 2*math.sin(t)/3
			left_shoulder_quat = p.getQuaternionFromEuler([0, -math.pi/2 - 3*math.cos(t)/2, math.pi/2])
			right_shoulder_quat = p.getQuaternionFromEuler([0, math.pi/2 - 3*math.cos(t)/2, math.pi/2])
			chest_quat = p.getQuaternionFromEuler([0, 0, 0])

		elif case == Moves.ARM_ROLL:
			left_elbow_pos = 0
			right_elbow_pos = 0
			left_shoulder_quat = p.getQuaternionFromEuler([math.sin(t+math.pi/2)+math.pi/3, math.sin(t), 0])
			right_shoulder_quat = p.getQuaternionFromEuler([math.sin(t+math.pi/2)-math.pi/3, math.sin(t), 0])
			chest_quat = p.getQuaternionFromEuler([0, 0, 0])

		elif case == Moves.CATCH:
			left_elbow_pos = 0
			right_elbow_pos = 0
			left_shoulder_quat = p.getQuaternionFromEuler([0, 0, math.pi/2])
			right_shoulder_quat = p.getQuaternionFromEuler([0, 0, math.pi/2])
			chest_quat = p.getQuaternionFromEuler([0, 0, 0])

		elif case == Moves.HANDS_UP:
			left_elbow_pos = 0
			right_elbow_pos = 0
			left_shoulder_quat = p.getQuaternionFromEuler([math.pi, 0, 0])
			right_shoulder_quat = p.getQuaternionFromEuler([math.pi, 0, 0])
			chest_quat = p.getQuaternionFromEuler([0, 0, 0])

		p.setJointMotorControl2(self.op.body_id, self.left_elbow, p.POSITION_CONTROL, left_elbow_pos)
		p.setJointMotorControl2(self.op.body_id, self.right_elbow, p.POSITION_CONTROL, right_elbow_pos)
		p.setJointMotorControlMultiDof(self.op.body_id, self.left_shoulder, p.POSITION_CONTROL, left_shoulder_quat)
		p.setJointMotorControlMultiDof(self.op.body_id, self.right_shoulder, p.POSITION_CONTROL, right_shoulder_quat)
		p.setJointMotorControlMultiDof(self.op.body_id, self.chest, p.POSITION_CONTROL, chest_quat)
