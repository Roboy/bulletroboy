
import rclpy
from roboy_simulation_msgs.msg import TendonUpdate
from std_msgs.msg import Float32
import time

def main():
    rclpy.init()
    node = rclpy.create_node("test_publisher")
    force_publisher = node.create_publisher(TendonUpdate, '/roboy/simulation/tendon_force', 10)
    angle_publisher = node.create_publisher(Float32, 'roboy/simulation/cage_rotation', 10)

    while True:

        msg = TendonUpdate()
        msg.tendon_id = 0
        msg.force = 200.0
        force_publisher.publish(msg)

        angle = Float32()
        angle.data = 58.0
        angle_publisher.publish(angle)

        time.sleep(0.1)


if __name__ == '__main__':
    main()

