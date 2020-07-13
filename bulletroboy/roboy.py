import pybullet as p
import time
import math
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision
from geometry_msgs.msg import PoseStamped
from bulletroboy.link_mapping import OPERATOR_TO_ROBOY_NAMES

class BulletRoboy(Node):
    """
    This class represents the Roboy in simulation. It handles movements of the roboy as well as publishing and receiving ros messages.
    """
    def __init__(self, body_id):
        super().__init__("bullet_roboy")
        self.body_id = body_id
        self.t = 0.

        # VARIABLES FOR DEBUG LINES
        self.prevPose = [0, 0, 0]
        self.prevPose1 = [0, 0, 0]
        self.hasPrevPose = 0
        #trailDuration is duration (in seconds) after debug lines will be removed automatically
        #use 0 for no-removal
        self.trailDuration = 5
        
        numJoints = p.getNumJoints(self.body_id)
        self.freeJoints = []
        self.endEffectors = {}

        for i in range(numJoints):
            info = p.getJointInfo(self.body_id,i)
            if info[2] == p.JOINT_REVOLUTE:
                self.freeJoints.append(i)
            if info[12] == b'hand_left':
                self.endEffectors['hand_left'] = i
                self.get_logger().info("EF hand_left id: " + str(i))
            if info[12] == b'hand_right':
                self.endEffectors['hand_right'] = i
                self.get_logger().info("EF hand_right id: " + str(i))

        #Publishers and subscribers

        #Joint state publisher
        timer_period = 0.1 # seconds
        self.joint_names = []
        for i in range(p.getNumJoints(self.body_id)):
            ji = p.getJointInfo(self.body_id,i)
            self.joint_names.append(ji[1].decode("utf-8"))
        self.joint_publisher = self.create_publisher(JointState, '/roboy/simulation/joint_state', 1)
        self.timer = self.create_timer(timer_period, self.joint_state_timer_callback)
        
        #Collision publisher
        self.collision_publisher = self.create_publisher(Collision, '/roboy/simulation/collision', 1)

        #Operator EF pose subscriber
        self.ef_pose_subscription = self.create_subscription(PoseStamped, '/roboy/simulation/operator/pose/endeffector', self.move, 10)

    def ef_pose_callback(self, ef_pose):
        """Callback function of the endeffector subscription. Processes the msg received and moves the link accordingly.

        Args:
            ef_pose: end effector pose received from the operator.
        """
        #self.get_logger().info('Endeffector pose received: ' + ef_pose.header.frame_id)

        #process message
        ef_name = OPERATOR_TO_ROBOY_NAMES[ef_pose.header.frame_id]
        ef_id = self.endEffectors[ef_name]
        
        link_pos = [ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z]
        link_orn= [ef_pose.pose.orientation.x, ef_pose.pose.orientation.y, ef_pose.pose.orientation.z, ef_pose.pose.orientation.w]

        #move
        self.move(ef_id, link_pos, link_orn)
        if(ef_name == 'hand_right'):
            self.drawDebugLine(ef_id, link_pos)

    def move(self, ef_id, target_pos, target_orn):
        """Moves ef given to target pose.

        Args:
            ef_id: end effector to move.
            target_pos: position to move ef to.
            target_orn: orientation to move ef to.
        """
        jointPoses = p.calculateInverseKinematics(self.body_id, ef_id, target_pos)#, target_orn)           
        for i in range(len(self.freeJoints)):
            jointInfo = p.getJointInfo(self.body_id, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                p.setJointMotorControl2(bodyIndex=self.body_id, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[qIndex-7])
            
        return jointPoses


    def adapt_pos_to_roboy(self, pos, orn):
        new_pos = np.array(pos)
        new_orn = orn
        return new_pos, new_orn

    def axis_angle_to_quaternion(self, ax, ay, az, angle):
        qx = ax * math.sin(angle/2)
        qy = ay * math.sin(angle/2)
        qz = az * math.sin(angle/2)
        qw = math.cos(angle/2)
        return qx, qy, qz, qw
    
    def quaternion_multiply(self, quaternion0, quaternion1):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    def drawDebugLine(self, link_id, target_pos):
        """Draws debug lines for target postions and actual positions of the link.

        Args:
            link_id: link to track.
            target_pos: target position of the link.
        """
        ls = p.getLinkState(self.body_id, link_id)
        if(self.hasPrevPose):
            p.addUserDebugLine(self.prevPose, target_pos, [0, 0, 0.3], 1, self.trailDuration)
            p.addUserDebugLine(self.prevPose1, ls[4], [1, 0, 0], 1, self.trailDuration)
        self.prevPose = target_pos
        self.prevPose1 = ls[4]
        self.hasPrevPose = 1  
    def joint_state_timer_callback(self):

        """Callback function for the timer, publishes joint message every time it gets triggered by timer. 
        """
        msg = JointState()
        msg.effort = [0.0]*len(self.joint_names)
        msg.velocity = [0.0]*len(self.joint_names)
        for i in range(p.getNumJoints(self.body_id)):
            js = p.getJointState(self.body_id, i)
            msg.position.append(js[0])
            msg.name.append(self.joint_names[i])
        self.joint_publisher.publish(msg)

    def publish_collision(self, collision):
        """Processes the collision message and publishes it.

        Args:
            collision: item of the contact points list output of pybullet getcontactPoints.
        """
        msg = Collision()

        #collision[3] == linkIndexA in PyBullet docu
        msg.linkid = collision[3]

        #collision[5] == positionOnA in PyBullet docu
        pos_in_lf = self.get_pos_in_link_frame(collision[3], collision[5])
        msg.position.x = pos_in_lf[0]
        msg.position.y = pos_in_lf[1]
        msg.position.z = pos_in_lf[2]

        #collision[7] == contactNormalOnB in PyBullet docu
        msg.contactnormal.x = collision[7][0]
        msg.contactnormal.y = collision[7][1]
        msg.contactnormal.z = collision[7][2]

        #collision[8] == contactDistance in PyBullet docu
        msg.contactdistance = collision[8]

        #collision[9] == normalForce in PyBullet docu
        msg.normalforce = collision[9]

        self.get_logger().info("Publishing collision in link %i" % msg.linkid)

        self.collision_publisher.publish(msg)
    
    def get_pos_in_link_frame(self, link, position):
        """Changes the position of the collision from world's coordinates system to link frame.

        Args:
            link: link id in which a collision happened.
            position: vector that would be changed.

        Returns:
            The position in link frame.
        """

        frame_pos = [0,0,0]
        frame_orn = [0,0,0,0]

        if(link == -1):
            frame_pos, frame_orn = (p.getBasePositionAndOrientation(self.body_id))[:2]
        else:
            #[0] == linkWorldPosition in PyBullet docu
            #[1] == linkWorldOrientation in PyBullet docu
            frame_pos, frame_orn = (p.getLinkState(self.body_id, link))[:2]

        _, inv_frame_orn = p.invertTransform(frame_pos, frame_orn)

        rotation = np.array(p.getMatrixFromQuaternion(inv_frame_orn))
        rotation = rotation.reshape(3,3)
        translation = np.array(frame_pos)

        return rotation.dot(position) + translation
