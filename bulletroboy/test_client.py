import rclpy
from roboy_control_msgs.srv import GetCageEndEffectors
from roboy_simulation_msgs.srv import OperatorHead
import time

def main():
    rclpy.init()
    node = rclpy.create_node("test_client")
    #cli = node.create_client(OperatorHead, '/roboy/configuration/end_effectors')
    cli = node.create_client(OperatorHead, '/roboy/simulation/operator/initial_headpose')
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    #req = GetCageEndEffectors.Request()
    req = OperatorHead.Request()
    res = cli.call_async(req)
    print("Before rclpy.ok()")

    while rclpy.ok():
        print("Before spin")
        rclpy.spin_once(node)
        print("Waiting response")
        if res.done():
            print("inside res.done")

            try:
                response = res.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                for ef in response.end_effectors:
                    node.get_logger().info(f"{ef.name} tendons {len(ef.muscle_units)}:")
                    for muscle in ef.muscle_units:
                        node.get_logger().info(f"muscle [{muscle.id}] max_force [{muscle.max_force}]")
                        for via_point in muscle.viapoints:
                            node.get_logger().info(f"via_point [{via_point.id}] reference_frame [{via_point.reference_frame}] link [{via_point.link}] point [{via_point.position}]")

            break

if __name__ == '__main__':
    main()

