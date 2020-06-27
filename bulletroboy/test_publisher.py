
import rclpy
import numpy as np

from roboy_simulation_msgs.msg import TendonUpdate, Collision
from roboy_control_msgs.msg import CageState, MuscleUnit, ViaPoint
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32
from bulletroboy.operator import Operator
import time

def main():
    rclpy.init()
    node = rclpy.create_node("test_publisher")
    #force_publisher = node.create_publisher(TendonUpdate, '/roboy/simulation/tendon_force', 10)
    #angle_publisher = node.create_publisher(Float32, '/roboy/simulation/cage_rotation', 10)
    #cage_state_publisher = node.create_publisher(CageState, '/roboy/simulation/cage_state', 10)
    collision_message_publisher = node.create_publisher(Collision, 'roboy/exoforce/collisions', 10)

    while True:

        # msg = TendonUpdate()
        # msg.tendon_id = 0
        # msg.force = 200.0
        # force_publisher.publish(msg)

        # angle = Float32()
        # angle.data = 58.0
        # angle_publisher.publish(angle)

        # state = CageState()
        # state.rotation_angle = 47.8
        # for i in range(3):
        #     muscleunit = MuscleUnit()
        #     muscleunit.id = i
        #     for i in range(2):
        #         point = ViaPoint()
        #         point.id = i
        #         point.position = Point()
        #         point.position.x = 0.0
        #         point.position.y = 0.0
        #         point.position.z = 0.0
        #         point.reference_frame = 'world'
        #         point.link = 'hand'
        #         muscleunit.viapoints.append(point)
        #     state.muscleunits.append(muscleunit)

        # cage_state_publisher.publish(state)

        t = time.time()
        contact_info = Collision()
        contact_info.linkid = 8
        contact_info.position.x = 0.0
        contact_info.position.y = 0.0
        contact_info.position.z = 0.0
        contact_info.contactnormal.x = 0.0
        contact_info.contactnormal.y = -1.0
        contact_info.contactnormal.z = 0.0
        contact_info.normalforce = 5000.0

        collision_message_publisher.publish(contact_info)
        exit()

        time.sleep(0.1)


if __name__ == '__main__':
    main()

