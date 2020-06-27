import pybullet as p
import time
import math
import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from roboy_simulation_msgs.msg import TendonUpdate


class Operator(Node):
	"""This class handles the operator body and its links in the simulation.

	"""
	def __init__(self, body_id):
		"""
		Args:
			body_id (int): Pybullet body indentifier.

		"""
		super().__init__("operator_node")		
		self.body_id = body_id
		self.links = self.get_links()
		self.movements = Movements(self)
		self.ef_publisher = self.create_publisher(PoseStamped, '/roboy/simulation/operator/pose/endeffector', 10)
		
		p.createConstraint(self.body_id, -1, -1, -1, p.JOINT_FIXED, [0,0,0],[0,0,0],[0,0,0],[0,0,0,1])
		self.init_joint_motors()
	
	def init_joint_motors(self):
		"""Initializes joint motors.
		
		Args:
			-

		Returns:
		   -

		"""
		for j in range(p.getNumJoints(self.body_id)):
			p.setJointMotorControlMultiDof(self.body_id, j, p.POSITION_CONTROL, [0,0,0,1], positionGain=0.1, force=[200])

	def get_links(self):
		"""Gets pybullet's links in operator body.
		
		Args:
			-

		Returns:
		   	List[dict]: 'name' and 'id' for each link in the operator's pybullet body.

		"""
		links = []
		for i in range(p.getNumJoints(self.body_id)):
			link = {}
			link['name'] = str(p.getJointInfo(self.body_id,i)[12], 'utf-8')
			link['id'] = i
			links.append(link)
		return links

	def get_link_center(self, link_name):
		"""Gets link's center point in world frame.
		
		Args:
			link_name (string): Name of the link to search.

		Returns:
		   	List[dict]: 'name' and 'id' for each link in the operator's pybullet body.

		"""
		center = None
		index = self.get_link_index(link_name)
		if index:
			center = np.asarray(p.getLinkState(self.body_id, self.links[index]['id'])[0])
		return center

	def get_link_index(self, link_name):
		"""Gets link's index given it's name.
		
		Args:
			link_name (string): Name of the link to search.

		Returns:
		   	int: Index of the given link.

		"""
		index = None
		for i, link in enumerate(self.links):
			if link['name'] == link_name:
				index = i
				break
		return index
	
	def move(self, case):
		"""Applies a movement to the operator.
		
		Args:
			case (int): Movement to apply.

		Returns:
		   	-

		"""
		self.movements.simple_move(case)

	def publish_state(self, ef_names=['left_wrist','right_wrist']):
		"""Publishes the end effectors' state as a ROS message.
		
		Args:
			-

		Returns:
		   	-

		"""
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
	def __init__(self, operator, link = b'left_wrist'):
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
		_, freeJoints = self.get_EF_id(b'left_wrist')
		endEffectorIds = [self.op.get_link_index('left_wrist'),
                                  self.op.get_link_index('right_wrist')]
		
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
		root_link = self.op.get_link_index('root')				# FIXED
		chest_link = self.op.get_link_index('chest')				# SPHERICAL
		left_shoulder = self.op.get_link_index('left_shoulder')			# SPHERICAL
		right_shoulder = self.op.get_link_index('right_shoulder')		# SPHERICAL
		left_elbow_link = self.op.get_link_index('left_elbow')			# REVOLUTE
		right_elbow_link = self.op.get_link_index('right_elbow')		# REVOLUTE

		# SPINE_SWING
		if case == 1:
			p.resetJointState(self.body_id, left_elbow_link, 0.0)
			p.resetJointState(self.body_id, right_elbow_link, 0.0)
			p.resetJointStateMultiDof(self.body_id, left_shoulder, p.getQuaternionFromEuler([math.pi/2,0,0]))
			p.resetJointStateMultiDof(self.body_id, right_shoulder, p.getQuaternionFromEuler([-math.pi/2,0,0]))
			p.resetJointStateMultiDof(self.body_id, chest_link, p.getQuaternionFromEuler([0,math.sin(t),0]))

		# FOREARM_ROLL
		elif case == 2:
			left_shoulder_quat = p.getQuaternionFromEuler([0,((-(math.cos(3*t)+1)/8) + (13/8))*math.pi,math.pi/2])
			right_shoulder_quat = p.getQuaternionFromEuler([0,-(((math.cos(3*t)+1)/8) + (11/8))*math.pi,math.pi/2])

			p.resetJointStateMultiDof(self.body_id, left_shoulder, left_shoulder_quat)
			p.resetJointStateMultiDof(self.body_id, right_shoulder, right_shoulder_quat)
			p.resetJointState(self.body_id, left_elbow_link, -(((math.sin(3*t)+1)/8) + (11/8))*math.pi)
			p.resetJointState(self.body_id, right_elbow_link, -(((math.sin(3*t+math.pi)+1)/8) + (11/8))*math.pi)

		# ARM ROLL
		elif case == 3:
			left_shoulder_quat = p.getQuaternionFromEuler([math.sin(t+math.pi/2)+math.pi/2,math.sin(t),0])
			right_shoulder_quat = p.getQuaternionFromEuler([math.sin(t+math.pi/2)+math.pi/2,-math.sin(t)-math.pi,0])
			p.resetJointState(self.body_id, left_elbow_link, 0)
			p.resetJointState(self.body_id, right_elbow_link, 0)
			p.resetJointStateMultiDof(self.body_id, left_shoulder, left_shoulder_quat)
			p.resetJointStateMultiDof(self.body_id, right_shoulder, right_shoulder_quat)

		# SIDE_SWING
		elif case == 4:
			p.resetJointState(self.body_id, left_elbow_link, 0.0)
			p.resetJointState(self.body_id, right_elbow_link, 0.0)
			p.resetJointStateMultiDof(self.body_id, left_shoulder, p.getQuaternionFromEuler([math.pi/2,0,0]))
			p.resetJointStateMultiDof(self.body_id, right_shoulder, p.getQuaternionFromEuler([-math.pi/2,0,0]))
			p.resetJointStateMultiDof(self.body_id, chest_link, p.getQuaternionFromEuler([math.sin(t),0,0]))
