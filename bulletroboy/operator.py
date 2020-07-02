import pybullet as p
import time
import math
import rclpy
import numpy as np
from enum import Enum

from rclpy.node import Node
from roboy_control_msgs.srv import GetLinkPose
from geometry_msgs.msg import PoseStamped
from roboy_simulation_msgs.msg import TendonUpdate
from roboy_simulation_msgs.srv import LinkInfoFromName

class Operator(Node):
	"""This class handles the operator body and its links in the simulation.

	"""
	def __init__(self, body_id, node=None):
		"""
		Args:
			body_id (int): Pybullet body indentifier.

		"""
		if (node==None):
			super().__init__("operator_node")	
			self.node = self	
		else:
			self.node = node

		self.body_id = body_id
		self.links = self.get_links()
		self.movements = Movements(self)

		self.prevPose = [0, 0, 0]
		self.trailDuration = 5

		self.ef_publisher = self.node.create_publisher(PoseStamped, '/roboy/simulation/operator/pose/endeffector', 10)
		self.link_info_service = self.node.create_service(LinkInfoFromName, '/roboy/simulation/operator/link_info_from_name', self.get_link_info_from_name)
		self.initial_pose_service = self.node.create_service(GetLinkPose, '/roboy/simulation/operator/initial_link_pose', self.initial_link_pose_callback)

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

	def get_link_info_from_name(self, request, response):
		"""ROS service callback to get link info from link name.

		"""
		#self.node.get_logger().info("received call")
		link = list(filter(lambda link: link['name'] == request.link_name, self.get_links()))
		assert len(link) == 1
		response.link_id = link[0]['id']
		response.dimensions.x, response.dimensions.y, response.dimensions.z = link[0]['dims']
		#self.node.get_logger().info("responding")
		return response 

	def get_links(self):
		"""Gets pybullet's links in operator body.
		
		Args:
			-

		Returns:
		   	List[dict]: 'name', 'id' and 'dims' for each link in the operator's pybullet body.

		"""
		links = []
		for i in range(p.getNumJoints(self.body_id)):
			name = str(p.getJointInfo(self.body_id,i)[12], 'utf-8')
			if name == 'left_wrist':
				self.node.get_logger().info("EF hand_left id: " + str(i))
			if name == 'right_wrist':
				self.node.get_logger().info("EF hand_right id: " + str(i))
			if name == 'neck':
				self.node.get_logger().info("EF neck id: " + str(i))

			link = {}
			link['name'] = str(p.getJointInfo(self.body_id,i)[12], 'utf-8')
			link['dims'] = self.get_link_bb_dim(i)
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

	def drawDebugLines(self, ef, pos_or):
		# drawing debug lines
		if(ef == 'left_wrist'):
			p.addUserDebugLine(self.prevPose, pos_or, [0, 0, 0.3], 1, self.trailDuration)
			self.prevPose = pos_or
		ef_id = self.get_link_index(ef)
		p.addUserDebugLine([0,0,0],[0.3,0,0],[1,0,0],parentObjectUniqueId=self.body_id, parentLinkIndex=ef_id)
		p.addUserDebugLine([0,0,0],[0,0.3,0],[0,1,0],parentObjectUniqueId=self.body_id, parentLinkIndex=ef_id)
		p.addUserDebugLine([0,0,0],[0,0,0.3],[0,0,1],parentObjectUniqueId=self.body_id, parentLinkIndex=ef_id)
	def publish_state(self, ef_names=['left_wrist','right_wrist']):
		"""Publishes the end effectors' state as a ROS message.
		
		Args:
			-

		Returns:
		   	-

		"""
		for ef in ef_names:
		   #self.get_logger().info('Sending Endeffector pose: ' + ef)
		   msg = PoseStamped()
		   ef_id = self.get_link_index(ef)
		   link_info = p.getLinkState(self.body_id, ef_id)[4:6]
		   link_pos = link_info[0]
		   link_orn = link_info[1]

		   self.drawDebugLines(ef, link_pos)

		   msg.header.frame_id = ef

		   msg.pose.position.x = link_pos[0]
		   msg.pose.position.y = link_pos[1]
		   msg.pose.position.z = link_pos[2]

		   msg.pose.orientation.x = link_orn[0]
		   msg.pose.orientation.y = link_orn[1]
		   msg.pose.orientation.z = link_orn[2]
		   msg.pose.orientation.w = link_orn[3]

		   self.ef_publisher.publish(msg)

	def initial_link_pose_callback(self, request, response):
		self.node.get_logger().info(f"Service Initial Link Pose: request received for {request.link_name}")
		head = self.get_link_index(request.link_name)
		head_info = p.getLinkState(self.body_id, head)[:2]
		link_pos = head_info[0]
		link_orn = head_info[1]

		response.pose.position.x = link_pos[0]
		response.pose.position.y = link_pos[1]
		response.pose.position.z = link_pos[2]

		response.pose.orientation.x = link_orn[0]
		response.pose.orientation.y = link_orn[1]
		response.pose.orientation.z = link_orn[2]
		response.pose.orientation.w = link_orn[3]

		return response


class Moves(Enum):
	"""This enum class handles the posible moves the operator can perform.

	"""
	SPINE_SWING = 1
	SIDE_SWING = 2
	FOREARM_ROLL = 3
	ARM_ROLL = 4


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

	def one_end_effector(self, pos, maxIter=100, chest_constraint=True):
		"""Uses Inverse Kinematics to compute the JointPoses for the position of the 
		   End-Effector defined in the Movements class.
		
		Args:
		   pos (list of 3 floats): x,y,z coordinates of the point that the end_effector is to be positioned at. If the movement is to be dynamic, write the xyz vector in terms of t.
		   maxIter (int): maximum number of iterations for the inverse kinematic computation.
		   chest_constraint (bool): chest and neck constraints for a stable and realistic movement.
		
		Returns:
			-

		"""
		_, freeJoints = self.get_EF_id(b'left_wrist')
		endEffectorId = self.left_wrist
		
		# Chest and Neck constraints
		if chest_constraint == True:
			self.apply_chest_and_neck_constraints()
		
		for _ in range(maxIter):
			jointPoses = p.calculateInverseKinematics(self.op.body_id, endEffectorId, pos)
			for i in range(len(freeJoints)):
				p.resetJointState(self.op.body_id, freeJoints[i], jointPoses[i])

	def two_end_effectors(self, positions, maxIter=100, chest_constraint=True):
		"""Uses Inverse Kinematics to compute the JointPoses for the position of both hands.
		
		Args:
		   pos (list of 6 floats): x,y,z coordinates of the desired position of the left_hand followed by the x,y,z coordinates of the desired position of the right_hand.
		   maxIter (int): maximum number of iterations for the inverse kinematic computation.
		   chest_constraint (bool): chest and neck constraints for a stable and realistic movement.
		
		Returns:
			-

		"""
		# Gets the hand link ids and the free revolute joints
		_, freeJoints = self.get_EF_id(b'left_wrist')
		endEffectorIds = [self.left_wrist, self.right_wrist]
		
		# Chest and Neck constraints
		if chest_constraint == True:
			self.apply_chest_and_neck_constraints()

		for _ in range(maxIter):
			jointPoses = p.calculateInverseKinematics2(self.op.body_id, endEffectorIds, positions)
			for i in range(len(freeJoints)):
				p.resetJointState(self.op.body_id, freeJoints[i], jointPoses[i])

	def simple_move(self, case):
		"""Defines a series of hardcoded movements for a direct use. (Movements Library)

		Args:
			case (Moves): The Moves enum corresponding with the movement to be applied.
		
		Returns:
		   -

		"""
		t = time.time()

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
			right_shoulder_quat = p.getQuaternionFromEuler([math.sin(t+math.pi/2)+math.pi/3, -math.sin(t)-math.pi, 0])
			chest_quat = p.getQuaternionFromEuler([0, 0, 0])
	
		p.setJointMotorControl2(self.op.body_id, self.left_elbow, p.POSITION_CONTROL, left_elbow_pos)
		p.setJointMotorControl2(self.op.body_id, self.right_elbow, p.POSITION_CONTROL, right_elbow_pos)
		p.setJointMotorControlMultiDof(self.op.body_id, self.left_shoulder, p.POSITION_CONTROL, left_shoulder_quat)
		p.setJointMotorControlMultiDof(self.op.body_id, self.right_shoulder, p.POSITION_CONTROL, right_shoulder_quat)
		p.setJointMotorControlMultiDof(self.op.body_id, self.chest, p.POSITION_CONTROL, chest_quat)
