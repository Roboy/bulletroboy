import rclpy
from roboy_control_msgs.srv import GetCageEndEfectors
import time

def main():
    rclpy.init()
    node = rclpy.create_node("test_client")
    cli = node.create_client(GetCageEndEfectors, '/roboy/configuration/end_efectors')

    while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')

    req = GetCageEndEfectors.Request()
    res = cli.call_async(req)

    while rclpy.ok():
        rclpy.spin_once(node)
        if res.done():
            try:
                response = res.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                for ef in response.end_efectors:
                    node.get_logger().info(f"{ef.name} tendons {len(ef.muscle_units)}:")
                    for muscle in ef.muscle_units:
                        node.get_logger().info(f"id [{muscle.id}] max_force [{muscle.max_force}] points [{muscle.viapoints[0].position}]")

            break

if __name__ == '__main__':
    main()

