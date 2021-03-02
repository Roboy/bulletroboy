import rospy

from roboy_simulation_msgs.msg import Collision, ContactPoint
from sensor_msgs.msg import JointState
import pybullet as p
import yaml
from rospkg import RosPack

class CageInteraction:
    '''
    This Class manages all the interactions between the Roboy simulation and the cage 
    '''
    def __init__(self, roboy_body_id, topic_root):
        rp = RosPack()
        conf_path = rp.get_path('bulletroboy') + "/src/conf.yml"
        with open(conf_path) as file:
            self.parent_link_map = yaml.load(file, Loader=yaml.FullLoader)['parent_link_map']
        self.body_id = roboy_body_id
        self.links = []
        self.init_urdf_info()
        self.collision_publisher = rospy.Publisher("/roboy/simulation/collision", Collision, queue_size=1)


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
        parent_name = self.parent_link_map.get(link['name'])
        if not parent_name :
            rospy.loginfo("Link torso is not mapped to a parent link. Will be mapped to itself.")
            parent_name = link['name']
        link['parent_name'] = parent_name
        self.links.append(link)
        for i in range(p.getNumJoints(self.body_id)):
            info = p.getJointInfo(self.body_id,i)
            link = {}
            name = str(info[12], 'utf-8')
            link['name'] = name
            link['dims'] = self.get_link_bb_dim(i)
            link['id'] = i
            parent_name = self.parent_link_map.get(name)
            if not parent_name :
                rospy.loginfo("Link {} with id {} is not mapped to a parent link. Will be mapped to itself.".format(name, i))
                parent_name = name
            link['parent_name'] = parent_name
            self.links.append(link)

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
        contact_pts = []
        for pt in collision:
            if pt[9] > 0:
                link = self.get_link_info_from_id(pt[3])
                if link['parent_name'] == link['name']:
                    link_id = pt[3]            
                else :
                    link_id = self.get_link_info_from_name(link['parent_name'])["id"]

                contact_pt = ContactPoint()

                #pt[3] == linkIndexA in PyBullet docu
                contact_pt.linkid = link_id

                #pt[5] == positionOnA in PyBullet docu
                pos_in_lf = self.get_vector_in_link_frame(link_id, pt[5])
                contact_pt.position.x = pos_in_lf[0]
                contact_pt.position.y = pos_in_lf[1]
                contact_pt.position.z = pos_in_lf[2]

                #pt[7] == contactNormalOnB in PyBullet docu
                normal_in_lf = self.get_vector_in_link_frame(link_id, pt[7])
                contact_pt.contactnormal.x = normal_in_lf[0]
                contact_pt.contactnormal.y = normal_in_lf[1]
                contact_pt.contactnormal.z = normal_in_lf[2]
                
                #pt[8] == contactDistance in PyBullet docu
                contact_pt.contactdistance = pt[8]

                #pt[9] == normalForce in PyBullet docu
                contact_pt.normalforce = pt[9]

                contact_pts.append(contact_pt)

        if not contact_pts:
            return
        
        msg = Collision()
        msg.header.stamp = rospy.Time.now()
        msg.contact_points = contact_pts
        
        self.collision_publisher.publish(msg)

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

        pos_in_LF, orn_in_LF = p.invertTransform(frame_pos, frame_orn)

        pos_in_LF, orn_in_LF = p.multiplyTransforms(pos_in_LF,orn_in_LF,vector,[0,0,0,1])

        return pos_in_LF