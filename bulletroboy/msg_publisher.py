import pybullet as p
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Collision

class BulletRoboyNode(Node):
    def __init__(self, bulletBodyId):
        super().__init__("bullet_roboy_node")
        self.body_id = bulletBodyId
        self.joint_publisher = self.create_joint_publisher()
        self.collision_publisher = self.create_publisher(Collision, '/roboy/simulation/collision', 1)

    def create_joint_publisher(self):
        publisher = self.create_publisher(JointState, '/roboy/simulation/joint_state', 1)
        timer_period = 0.1 # seconds
        self.time = self.create_timer(timer_period, self.timer_callback)

        self.joint_names = []
        for i in range(p.getNumJoints(self.body_id)):
            ji = p.getJointInfo(self.body_id,i)
            self.joint_names.append(ji[1].decode("utf-8"))
        return publisher

    def timer_callback(self):
        msg = JointState()
        msg.effort = [0.0]*len(self.joint_names)
        msg.velocity = [0.0]*len(self.joint_names)
        for i in range(p.getNumJoints(self.body_id)):
            js = p.getJointState(self.body_id, i)
            msg.position.append(js[0])
            msg.name.append(self.joint_names[i])
        self.joint_publisher.publish(msg)

    def send_collision(self, collision):
        msg = Collision()
        msg.externalbody = collision[2]
        msg.linkroboy = collision[3]
        msg.linkexternalbody = collision[4]
        msg.positiononroboy.x = collision[5][0]
        msg.positiononroboy.y = collision[5][1]
        msg.positiononroboy.z = collision[5][2]
        msg.positiononexternalbody.x = collision[6][0]
        msg.positiononexternalbody.y = collision[6][1]
        msg.positiononexternalbody.z = collision[6][2]
        msg.contactnormalonexternalbody.x = collision[7][0]
        msg.contactnormalonexternalbody.y = collision[7][1]
        msg.contactnormalonexternalbody.z = collision[7][2]
        msg.contactdistance = collision[8]
        msg.normalforce = collision[9]
        msg.lateralfriction1 = collision[10]
        msg.lateralfrictiondir1.x = collision[11][0]
        msg.lateralfrictiondir1.y = collision[11][1]
        msg.lateralfrictiondir1.z = collision[11][2]
        msg.lateralfriction2 = collision[12]
        msg.lateralfrictiondir2.x = collision[13][0]
        msg.lateralfrictiondir2.y = collision[13][1]
        msg.lateralfrictiondir2.z = collision[13][2]
        self.collision_publisher.publish(msg)

    
