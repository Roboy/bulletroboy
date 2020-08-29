
import rclpy
import numpy as np
from roboy_simulation_msgs.msg import TendonUpdate, Collision
from roboy_control_msgs.msg import CageState, MuscleUnit, ViaPoint
from geometry_msgs.msg import Point, Vector3, PoseStamped
from std_msgs.msg import Float32
from bulletroboy.operator import Operator
import time

def main():
    rclpy.init()
    node = rclpy.create_node("test_publisher")
    #force_publisher = node.create_publisher(TendonUpdate, '/roboy/simulation/tendon_force', 1)
    #angle_publisher = node.create_publisher(Float32, '/roboy/simulation/cage_rotation', 1)
    cage_state_publisher = node.create_publisher(CageState, '/roboy/simulation/cage_state', 1)
    collision_message_publisher = node.create_publisher(Collision, 'roboy/exoforce/collisions', 1)
    robot_collision_publisher = node.create_publisher(Collision, 'roboy/simulation/roboy/collision', 1)
    roboy_driver_publisher = node.create_publisher(PoseStamped, '/bullet_ik', 1)

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

        # t = time.time()
        # contact_info = Collision()
        # contact_info.linkid = 8
        # contact_info.position.x = 0.0
        # contact_info.position.y = 0.0
        # contact_info.position.z = 0.0
        # contact_info.contactnormal.x = 0.0
        # contact_info.contactnormal.y = -1.0
        # contact_info.contactnormal.z = 0.0
        # contact_info.normalforce = 5000.0

        # collision_message_publisher.publish(contact_info)
        # exit()

        # robotCollision = Collision()
        # robotCollision.linkid = 1
        # robotCollision.position = Vector3()
        # robotCollision.contactnormal = Vector3()
        # robotCollision.contactdistance = float(5)
        # robotCollision.normalforce = float(3)

        # robot_collision_publisher.publish(robotCollision)

        ef_pos = [-0.0273208, 0.20801295999999994, 0.938560576]
        ef_orn = [-0.0, -0.0, 0.7071067811865475, 0.7071067811865476]

        msg = PoseStamped()
        msg.header.frame_id = 'hand_left'

        msg.pose.position.x = ef_pos[0]
        msg.pose.position.y = ef_pos[1]
        msg.pose.position.z = ef_pos[2]

        msg.pose.orientation.x = ef_orn[0]
        msg.pose.orientation.y = ef_orn[1]
        msg.pose.orientation.z = ef_orn[2]
        msg.pose.orientation.w = ef_orn[3]

        roboy_driver_publisher.publish(msg)

        ef_pos = [-0.0273208, -0.20801296, 0.938560576]
        ef_orn = [0.7071067811865475, -0.0, -0.0, 0.7071067811865476]

        msg = PoseStamped()
        msg.header.frame_id = 'hand_right'

        msg.pose.position.x = ef_pos[0]
        msg.pose.position.y = ef_pos[1]
        msg.pose.position.z = ef_pos[2]

        msg.pose.orientation.x = ef_orn[0]
        msg.pose.orientation.y = ef_orn[1]
        msg.pose.orientation.z = ef_orn[2]
        msg.pose.orientation.w = ef_orn[3]

        roboy_driver_publisher.publish(msg)

        ef_pos = [0.0, 1.249000902703301e-16, 1.72141112]
        ef_orn = [0.7071067811865475, -0.0, -0.0, 0.7071067811865476]

        msg = PoseStamped()
        msg.header.frame_id = 'head'

        msg.pose.position.x = ef_pos[0]
        msg.pose.position.y = ef_pos[1]
        msg.pose.position.z = ef_pos[2]

        msg.pose.orientation.x = ef_orn[0]
        msg.pose.orientation.y = ef_orn[1]
        msg.pose.orientation.z = ef_orn[2]
        msg.pose.orientation.w = ef_orn[3]

        roboy_driver_publisher.publish(msg)
        time.sleep(0.1)


if __name__ == '__main__':
    main()

