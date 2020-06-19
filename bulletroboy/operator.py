import pybullet as p
import time
import math
import rclpy
import numpy as np

from rclpy.node import Node
from numpy.linalg import norm
from termcolor import colored
from roboy_simulation_msgs.msg import TendonUpdate
from geometry_msgs.msg import PoseStamped
from bulletroboy.srv import HumanLinkIdFromName


class Operator(Node):
	def __init__(self, body_id):
		"""
		This class handles the operator body and its links in the simulation.
		"""
		super().__init__("operator_node")		
		self.body_id = body_id
		self.links = self.get_links()
		self.movements = Movements(self)
		self.ef_publisher = self.create_publisher(PoseStamped, '/roboy/simulation/operator/pose/endeffector', 10)
		self.srv = self.create_service(HumanLinkIdFromName, 'human_link_id_from_link_name', self.human_link_id_from_link_name)
		

	def human_link_id_from_link_name(self, request, response):
		link = list(filter(lambda link: link['name'] == request.human_link_name, self.get_links()))
		assert len(link) == 1
		response.human_link_id = link[0]
		return response 

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
	
	def move(self, case):
		self.movements.simple_move(case)

	def publish_state(self, ef_names = np.array(['human/left_hand','human/right_hand'])):

		for ef in ef_names:
		   msg = PoseStamped()
		   ef_id = self.get_link_index(ef)
		   link_info = p.getLinkState(self.body_id, ef_id)[:2]
		   link_pos = link_info[0]
		   link_orn = link_info[1]

		   msg.header.frame_id = ef

		   msg.pose.position.x = link_pos[0]
		   msg.pose.position.y = link_pos[1]
		   msg.pose.position.z = link_pos[2]

		   msg.pose.orientation.x = link_orn[0]
		   msg.pose.orientation.y = link_orn[1]
		   msg.pose.orientation.z = link_orn[2]
		   msg.pose.orientation.w = link_orn[3]

		   self.ef_publisher.publish(msg)


class Movements():
	def __init__(self, operator, link = b'human/left_hand'):
		"""
		This class defines 2 types of movements:
		-  Re-definable inverse kinematic ones (one_end_effector(), two_end_effectors())
		-  Harcoded simple ones (simple_move() --> 4 predefined movements)
		"""

		self.op = operator
		self.body_id = operator.body_id
		self.link = link


	def get_EF_id(self, link_name):
		"""
		Gets the id of the endEffector and the list of free revolute joints 
		"""
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
		"""
		Keeps the chest and neck in a straight position. In speific cases it makes the 			movement more ergonomic and human-like.
		"""

		# Constraint on the Chest
		p.createConstraint(self.body_id, self.op.get_link_index('human/spine_1'), -1,
				   self.op.get_link_index('human/spine_2'), p.JOINT_FIXED,
				   [0, 0, 0], [0, 0, 0], [0, 0, 1])

		# Constraint on the Neck
		p.createConstraint(self.body_id, self.op.get_link_index('human/neck'), -1,
				   self.op.get_link_index('human/spine_2'), p.JOINT_FIXED,
				   [0, 0, 0], [0, 0, 0], [0, 0, 1])


	def one_end_effector(self, pos, maxIter = 100, chest_constraint = True):
		"""
		Uses Inverse Kinematics to compute the JointPoses for the position of the 
		End-Effector defined in the Movements class.
		
		Args:
		   pos (list of 3 floats): x,y,z coordinates of the point that the end_effector is to be positioned at. If the movement is to be dynamic, write the xyz vector in terms of t.
		   maxIter (int): maximum number of iterations for the inverse kinematic computation.
		   chest_constraint (bool): chest and neck constraints for a stable and realistic movement.
		
		Raises:
		   -

		Returns:
		   -
		"""

		iter = 0
		endEffectorId, freeJoints = self.get_EF_id(self.link)
		
		# Chest and Neck constraints
		if chest_constraint == True:
			self.apply_chest_and_neck_constraints()
		
		while(iter <= maxIter):
			jointPoses = p.calculateInverseKinematics(self.body_id, endEffectorId, pos)
			for i in range(len(freeJoints)):
				p.resetJointState(self.body_id, freeJoints[i], jointPoses[i])
			iter = iter + 1



	def two_end_effectors(self, positions, maxIter = 100, chest_constraint = True):
		"""
		Uses Inverse Kinematics to compute the JointPoses for the position of both hands.
		
		Args:
		   pos (list of 6 floats): x,y,z coordinates of the desired position of the left_hand followed by the x,y,z coordinates of the desired position of the right_hand.
		   maxIter (int): maximum number of iterations for the inverse kinematic computation.
		   chest_constraint (bool): chest and neck constraints for a stable and realistic movement.
		
		Raises:
		   -

		Returns:
		   -
		"""

		iter = 0

		# Gets the hand link ids and the free revolute joints
		_, freeJoints = self.get_EF_id(b'human/left_hand')
		endEffectorIds = [self.op.get_link_index('human/left_hand'),
                                  self.op.get_link_index('human/right_hand')]
		
		# Chest and Neck constraints
		if chest_constraint == True:
			self.apply_chest_and_neck_constraints()

		while(iter <= maxIter):
			jointPoses = p.calculateInverseKinematics2(self.body_id, endEffectorIds, positions)
			for i in range(len(freeJoints)):
				p.resetJointState(self.body_id, freeJoints[i], jointPoses[i])
			iter = iter + 1




	def simple_move(self, case):
		"""
		Defines a series of hardcoded movements for a direct use. (Movements Library)

		Args:
		   case (int): the constant corresponding with the movement to be applied
		
		Raises:
		   -

		Returns:
		   -
		"""
		
		t = time.time()

		# Get link Ids
		spine_link = self.op.get_link_index('human/spine_2')
		spine_side_link = self.op.get_link_index('human/spine')
		left_shoulder_1 = self.op.get_link_index('human/left_shoulder_1')
		right_shoulder_1 = self.op.get_link_index('human/right_shoulder_1')
		left_shoulder_0 = self.op.get_link_index('human/left_shoulder_0')
		right_shoulder_0 = self.op.get_link_index('human/right_shoulder_0')
		left_elbow_link = self.op.get_link_index('human/left_elbow')
		right_elbow_link = self.op.get_link_index('human/right_elbow')

		# Use input to move the figure with the chosen movement.
		# If a non-existent case is used as input, print warning.
		if case == 1:
                    p.resetJointState(self.body_id, spine_link, math.sin(t))

		elif case == 2:
                    p.resetJointState(self.body_id, left_shoulder_1, -1.75*math.pi/4)
                    p.resetJointState(self.body_id, right_shoulder_1, 1.75*math.pi/4)
                    p.resetJointState(self.body_id, left_elbow_link, (((math.sin(3*t)+1)/8) + (11/8))*math.pi)
                    p.resetJointState(self.body_id, left_shoulder_0, ((-(math.cos(3*t)+1)/8) + (1/8))*math.pi)
                    p.resetJointState(self.body_id, right_elbow_link, -(((math.sin(3*t+math.pi)+1)/8) + (11/8))*math.pi)
                    p.resetJointState(self.body_id, right_shoulder_0, ((-(math.cos(3*t+math.pi)+1)/8) + (1/8))*math.pi)

		elif case == 3:
                    p.resetJointState(self.body_id, left_shoulder_1, math.sin(t))
                    p.resetJointState(self.body_id, left_shoulder_0, math.sin(t+math.pi/2))
                    p.resetJointState(self.body_id, right_shoulder_1, -math.sin(t))
                    p.resetJointState(self.body_id, right_shoulder_0, math.sin(t+math.pi/2))

		elif case == 4:
                    p.resetJointState(self.body_id, spine_side_link, math.sin(t))
