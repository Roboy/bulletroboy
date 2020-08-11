import pybullet as p
import time
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision
from geometry_msgs.msg import PoseStamped
from roboy_simulation_msgs.srv import LinkInfoFromId
from roboy_control_msgs.srv import GetLinkPose
import bulletroboy.utils as utils

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
        self.roboy_to_human_link_names_map = utils.load_roboy_to_human_link_name_map()
        self.human_to_roboy_link_names_map = {v: k for k, v in self.roboy_to_human_link_names_map.items()}

        self.links = []
        self.freeJoints = []
        self.endEffectors = {}
        self.init_urdf_info()
        
        #Publishers and subscribers

        #Joint state publisher
        timer_period = 0.01 # seconds
        self.joint_names = []
        for i in range(p.getNumJoints(self.body_id)):
            ji = p.getJointInfo(self.body_id,i)
            self.joint_names.append(ji[1].decode("utf-8"))
        self.joint_publisher = self.create_publisher(JointState, '/roboy/simulation/joint_state', 1)
        self.timer = self.create_timer(timer_period, self.joint_state_timer_callback)
        
        #Collision publisher
        self.collision_publisher = self.create_publisher(Collision, 'roboy/simulation/roboy/collision', 1)

        #Operator EF pose subscriber
        self.ef_pose_subscription = self.create_subscription(PoseStamped, '/roboy/exoforce/pose/endeffector', self.ef_pose_callback, 10)

        #Services and clients

        #LinkNameFromId service
        self.link_info_service = self.create_service(LinkInfoFromId, '/roboy/simulation/roboy/link_info_from_id', self.link_info_from_id_callback)
        
        #Initial pose service
        self.initial_pose_service = self.create_service(GetLinkPose, '/roboy/simulation/roboy/initial_link_pose', self.initial_link_pose_callback)

    def init_urdf_info(self):
        """Gets links, free joints, endeffectors and initial link poses in roboy's body.
        Args:
            -
        Returns:
            -
        """
        link = {}
        link['name'] = 'torso'
        link['dims'] = self.get_link_bb_dim(-1)
        link['init_pose'] = p.getBasePositionAndOrientation(self.body_id)
        link['id'] = -1 
        self.links.append(link)
        for i in range(p.getNumJoints(self.body_id)):
            info = p.getJointInfo(self.body_id,i)
            link = {}
            name = str(p.getJointInfo(self.body_id,i)[12], 'utf-8')
            link['name'] = name
            link['dims'] = self.get_link_bb_dim(i)
            link['init_pose'] = p.getLinkState(self.body_id, i)[:2]
            link['id'] = i
            self.links.append(link)
            if info[2] == p.JOINT_REVOLUTE:
                self.freeJoints.append(i)
            if name == 'hand_left':
                self.endEffectors['hand_left'] = i
                self.get_logger().info("EF hand_left id: " + str(i))
                self.get_logger().info("Initial orientation: " + str(link['init_pose'][1][0]) 
                                                + "   " + str(link['init_pose'][1][1]) 
                                                + "   " + str(link['init_pose'][1][2]) 
                                                + "   " + str(link['init_pose'][1][3]))
                
            if name == 'hand_right':
                self.endEffectors['hand_right'] = i
                self.get_logger().info("EF hand_right id: " + str(i))
                self.get_logger().info("Initial orientation: " + str(link['init_pose'][1][0]) 
                                                + "   " + str(link['init_pose'][1][1]) 
                                                + "   " + str(link['init_pose'][1][2]) 
                                                + "   " + str(link['init_pose'][1][3]))
            if name == 'head':
                self.get_logger().info("EF head id: " + str(i))

            if name in self.roboy_to_human_link_names_map.keys():
                self.draw_LF_coordinate_system(i) 

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

    def get_link_info_from_name(self, link_name):
        """Returns the item in the links list that contains information about the link with the name given.
        Args:
            link_name: name of link
        Returns:
            link from links list
        """
        link = list(filter(lambda link: link['name'] == link_name, self.links))
        assert len(link) == 1
        return link[0]

    def get_link_info_from_id(self, link_id):
        """Returns the item in the links list that contains information about the link with the id given.
        Args:
            link_id: id of link
        Returns:
            link from links list
        """
        link = list(filter(lambda link: link['id'] == link_id, self.links))
        assert len(link) == 1
        return link[0]

    def link_info_from_id_callback(self, request, response):
        """ROS service callback to get link info from link id.
        """
        self.get_logger().info('Link Info From Id Service: received request for id' + str(request.link_id))
        link = self.get_link_info_from_id(request.link_id)
        response.link_name = link['name']
        response.dimensions.x, response.dimensions.y, response.dimensions.z = link['dims']
                
        return response

    def get_link_name_from_id(self, link_id):
        """Returns link name from link id.
        """
        link_name = None
        for link in self.links:
            if link['id'] == link_id:
                link_name = link['name']
                break
        return link_name

    def get_link_id_from_name(self, link_name):
        """Returns link id from link name.
        """
        link_id = None
        if link_name == "torso":
            return -1
        for i, link in enumerate(self.links):
            if link['name'] == link_name:
                link_id = i
                break
        return link_id

    def draw_LF_coordinate_system(self, link_id):
        """Draws the coordinate system of the link.
        Args: 
            link_id : id of the link.
        """
        p.addUserDebugLine([0,0,0],[0.3,0,0],[1,0,0],lineWidth= 3, parentObjectUniqueId=self.body_id, parentLinkIndex=link_id)
        p.addUserDebugLine([0,0,0],[0,0.3,0],[0,1,0],lineWidth= 3, parentObjectUniqueId=self.body_id, parentLinkIndex=link_id)
        p.addUserDebugLine([0,0,0],[0,0,0.3],[0,0,1],lineWidth= 3, parentObjectUniqueId=self.body_id, parentLinkIndex=link_id)

    def initial_link_pose_callback(self, request, response):
        """Callback function of the initial link pose service.

        Args:
            request: name of the link.
            response: pose of the link (position and orientation).
        Returns:
            the response
        """
        self.get_logger().info(f"Service Initial Link Pose: request received for {request.link_name}")
        
        link = self.get_link_info_from_name(request.link_name)
        
        link_pos = link['init_pose'][0]
        link_orn = link['init_pose'][1]

        response.pose.position.x = link_pos[0]
        response.pose.position.y = link_pos[1]
        response.pose.position.z = link_pos[2]

        response.pose.orientation.x = link_orn[0]
        response.pose.orientation.y = link_orn[1]
        response.pose.orientation.z = link_orn[2]
        response.pose.orientation.w = link_orn[3]
        self.get_logger().info(f"Service Initial Link Pose: sending response")

        return response

    def ef_pose_callback(self, ef_pose):
        """Callback function of the endeffector subscription. Processes the msg received and moves the link accordingly.

        Args:
            ef_pose: end effector pose received from the operator.
        """
        self.get_logger().info('Endeffector pose received: ' + ef_pose.header.frame_id)
        
        #process message
        ef_name = ef_pose.header.frame_id
        ef_id = self.endEffectors[ef_name]
        
        link_pos = [ef_pose.pose.position.x, ef_pose.pose.position.y, ef_pose.pose.position.z]
        link_orn = [ef_pose.pose.orientation.x, 
                        ef_pose.pose.orientation.y, 
                        ef_pose.pose.orientation.z, 
                        ef_pose.pose.orientation.w]

        # link_orn = self.get_adapted_link_orientation(ef_id,
        #                                             [ef_pose.pose.orientation.x, ef_pose.pose.orientation.y, 
        #                                                 ef_pose.pose.orientation.z, ef_pose.pose.orientation.w])
        
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
        jointPoses = p.calculateInverseKinematics(self.body_id, ef_id, target_pos, target_orn)           
        for i in range(len(self.freeJoints)):
            jointInfo = p.getJointInfo(self.body_id, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                p.setJointMotorControl2(bodyIndex=self.body_id, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[qIndex-7])
            
        return jointPoses

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
        """Publishes collision as a ROS Message.

        Args:
            collision (list): item of the contact points list output of pybullet getcontactPoints.

        Returns:
            -

        """
        if collision[9] > 0:
            msg = Collision()

            #collision[3] == linkIndexA in PyBullet docu
            msg.linkid = collision[3]

            #collision[5] == positionOnA in PyBullet docu
            pos_in_lf = self.get_vector_in_link_frame(collision[3], collision[5])
            msg.position.x = pos_in_lf[0]
            msg.position.y = pos_in_lf[1]
            msg.position.z = pos_in_lf[2]

            #collision[7] == contactNormalOnB in PyBullet docu
            normal_in_lf = self.get_vector_in_link_frame(collision[3], collision[7])
            msg.contactnormal.x = normal_in_lf[0]
            msg.contactnormal.y = normal_in_lf[1]
            msg.contactnormal.z = normal_in_lf[2]

            #collision[8] == contactDistance in PyBullet docu
            msg.contactdistance = collision[8]

            #collision[9] == normalForce in PyBullet docu
            msg.normalforce = collision[9]

            self.get_logger().info("Publishing collision in link %i" % msg.linkid)
            self.draw_force(msg)

            self.collision_publisher.publish(msg)

    def draw_force(self, collision):
        """Draw collision force as a debugLine.

        Args:
            collision (Collision): Collision to draw.

        Returns:
            -
        
        """
        pos = np.array([collision.position.x, collision.position.y, collision.position.z])
        direction = np.array([collision.contactnormal.x,collision.contactnormal.y,collision.contactnormal.z]) * collision.normalforce
        p.addUserDebugLine(pos, pos + direction, [1, 0.4, 0.3], 2, 2, self.body_id, collision.linkid)
    
    def get_vector_in_link_frame(self, link, vector):
        """Transforms a vector from world's coordinates system to link frame.

        Args:
            link (int): Link id.
            vector (array[3]): Vector in world's coordinates system.

        Returns:
            array[3]: The vector in link frame.
        
        """
        if(link == -1):
            frame_pos, frame_orn = (p.getBasePositionAndOrientation(self.body_id))[:2]
        else:
            #[0] == linkWorldPosition in PyBullet docu
            #[1] == linkWorldOrientation in PyBullet docu
            frame_pos, frame_orn = (p.getLinkState(self.body_id, link))[:2]

        translation, rotation = p.invertTransform(frame_pos, frame_orn)

        rotation = np.array(p.getMatrixFromQuaternion(rotation))
        rotation = rotation.reshape(3,3)
        translation = np.array(translation)

        return rotation.dot(vector) + translation
