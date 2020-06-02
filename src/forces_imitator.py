import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from sensor_msgs.msg import ContactPoints

class ForcesImitator(Node):
    def __init(self, initialJointState):
        self.jointState = initialJointState
        self.contactPoints = ContactPoints()

        self.result = ContactPoints()
        
        self.jointSubscription = self.create_subscription(
            JointState,
            '/roboy/simulation/joint_state',
            self.listener_joints_callback,
            1)

        self.environmentSubscription = self.create_subscription(
            ContactPoints,
            '/roboy/simulation/environment_collision',
            self.listener_environment_callback,
            1)

        self.robotCollisionSubscription = self.create_subscription(
            ContactPoints,
            '/roboy/simulation/robot_collision_feedback',
            self.listener_robot_callback,
            1)

        self.publisher = self.create_publisher(ContactPoints, '/roboy/kinematic/forces_imitator', 1)
        timer_period = 0.1 # seconds
        self.time = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.publisher.publish(self.result)


    def listener_joints_callback(self, msg):
        self.jointState = msg
        self.update()
        self.get_logger().info('I heard: "%s"' % msg.data)

    def listener_environnment_callback(self, msg):
        self.contactPoints = msg
        self.update()
        self.get_logger().info('I heard: "%s"' % msg.data)

    def listener_robot_callback(self, msg):
        self.contactPoints = msg
        self.update()
        self.get_logger().info('I heard: "%s"' % msg.data)

    def update(self):
         self.get_logger().info('I heard:')
    

def main(args=None):
    rclpy.init(args=args)

    forcesImitator = ForcesImitator(JointState())
    rclpy.spin(forcesImitator)
    forcesImitator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()