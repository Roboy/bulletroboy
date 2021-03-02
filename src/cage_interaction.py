import rospy
from roboy_simulation_msgs.msg import Collision
from sensor_msgs.msg import JointState
import pybullet as p

CONF_PATH = r'conf.yml'


class CageInteraction:
    '''
    This Class manages all the interactions between the Roboy simulation and the cage 
    '''
    def __init__(self, roboy_body_id):
        with open(CONF_PATH) as file:
            self.body_id = roboy_body_id
            self.parent_link_map = yaml.load(file, Loader=yaml.FullLoader)
            self.links = {}
            self.init_urdf_info()
		    self.collision_publisher = rospy.Publisher(topic_root+"/collisions", JointState, queue_size=1)


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
		link['id'] = -1 
		self.links.append(link)
		for i in range(p.getNumJoints(self.body_id)):
			info = p.getJointInfo(self.body_id,i)
			link = {}
			name = str(p.getJointInfo(self.body_id,i)[12], 'utf-8')
			link['name'] = name
			link['dims'] = self.get_link_bb_dim(i)
			link['id'] = i
			self.links.append(link)
			if info[2] == p.JOINT_REVOLUTE:
				self.freeJoints.append(i)
			if name == 'hand_left':
				self.end_effectors[name] = i
				
			if name == 'hand_right':
				self.end_effectors[name] = i

    def get_link_info_from_id(self, link_id):
		"""Returns the item in the links list that contains information about the link with the id given.
		Args:
			link_id: id of link
		Returns:
			link from links list
		"""
		link = list(filter(lambda link: link['id'] == link_id, self.links))
		assert len(link) == 1, "Found result length is not 1 : link_id = {}, result_length = {}".format(link_id, len(link))
		return link[0]

    def get_link_info_from_name(self, link_name):
        """Returns the item in the links list that contains information about the link with the name given.
        Args:
            link_name: name of link
        Returns:
            link from links list
        """
        link = list(filter(lambda l: l['name'] == link_name, self.links))

        assert len(link) == 1, "Found result length is not 1 : link_name = {}, result_length = {}".format(link_name, len(link))
        return link[0]

    def publish_collision(self, collision):
        """Publishes collision as a ROS Message.

        Args:
            collision (list): item of the contact points list output of pybullet getcontactPoints.

        Returns:
            -

        """
        if collision[9] > 0:
            link = self.get_link_info_from_id(collision[3])
            if	link['parent_name'] == link['name']:
                link_id = collision[3]

            else :
                link_id = self.get_link_info_from_name(link['parent_name'])['id']
            
            msg = Collision()

            #collision[3] == linkIndexA in PyBullet docu
            msg.linkid = link_id

            #collision[5] == positionOnA in PyBullet docu
            pos_in_lf = self.get_vector_in_link_frame(link_id, collision[5])
            msg.position.x = pos_in_lf[0]
            msg.position.y = pos_in_lf[1]
            msg.position.z = pos_in_lf[2]

            #collision[7] == contactNormalOnB in PyBullet docu
            normal_in_lf = self.get_vector_in_link_frame(link_id, collision[7])
            msg.contactnormal.x = normal_in_lf[0]
            msg.contactnormal.y = normal_in_lf[1]
            msg.contactnormal.z = normal_in_lf[2]

            #collision[8] == contactDistance in PyBullet docu
            msg.contactdistance = collision[8]

            #collision[9] == normalForce in PyBullet docu
            msg.normalforce = collision[9]

            rospy.loginfo("Publishing collision in link %i" % msg.linkid)

            self.collision_publisher.publish(msg)

    def get_vector_in_link_frame(py, link, vector):
        """Transforms a vector from world's coordinates system to link frame.

        Args:
            link (int): Link id.
            vector (array[3]): Vector in world's coordinates system.

        Returns:
            array[3]: The vector in link frame.
        
        """
        if(link == -1):
            frame_pos, frame_orn = (py.getBasePositionAndOrientation(self.body_id))[:2]
        else:
            #[0] == linkWorldPosition in PyBullet docu
            #[1] == linkWorldOrientation in PyBullet docu
            frame_pos, frame_orn = (py.getLinkState(self.body_id, link))[:2]

        pos_in_LF, orn_in_LF = py.invertTransform(frame_pos, frame_orn)

        pos_in_LF, orn_in_LF = py.multiplyTransforms(pos_in_LF,orn_in_LF,vector,[0,0,0,1])

        return pos_in_LF