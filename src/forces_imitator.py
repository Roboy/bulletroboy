import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from sensor_msgs.msg import ContactPoints

class ForcesIimitatorPublisher(Node):

    def __init__(self, initialMsg):
        super().__init__("forces_imitator_publisher")
        self.msg = initialMsg
        self.publisher = self.create_publisher(ContactPoints, '/roboy/kinematic/forces_imitator', 1)
        timer_period = 0.1 # seconds
        self.time = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.publisher.publish(msg)

class ForcesImitator():
    def __init(self, initialJointState):
        self.jointState = initialJointState
        self.publisher = ForcesImitatorPublisher(ContactPoints())
        self.contactPoints = ContactPoints()

    def updateJointState(jointState):
        self.jointState = jointState
        self.update()

    def updateCollisionPredictionContactPoints(contactPoints):
        self.contactPoints = contactPoints
        self.update()

    def updateRobotFeedbackContactPoints(contactPoints):
        self.contactPoints = contactPoints
        self.update()

    def update():
        
        
        

class JointSubscriber(Node):

    def __init__(self, forcesImitator):
        super().__init__('joint_subscriber')
        self.forcesImitator = forcesImitator
        self.subscription = self.create_subscription(
            JointState,
            '/roboy/simulation/joint_state',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.forcesImitator.updateJointState(msg)
        self.get_logger().info('I heard: "%s"' % msg.data)

class EnvironmentCollisionSubscriber(Node):

    def __init__(self, forcesImitator):
        super().__init__('environment_collision_subscriber')
        self.forcesImitator = forcesImitator
        self.subscription = self.create_subscription(
            ContactPoints,
            '/roboy/simulation/environment_collision',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.forcesImitator.updateCollisionPredictionContactPoints(msg)
        self.get_logger().info('I heard: "%s"' % msg.data)

class RobotCollisionSubscriber(Node):

    def __init__(self, forcesImitator):
        super().__init__('robot_collision_subscriber')
        self.forcesImitator = forcesImitator
        self.subscription = self.create_subscription(
            ContactPoints,
            '/roboy/simulation/robot_collision_feedback',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.forcesImitator.updateRobotFeedbackContactPoints(msg)
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    forcesImitator = ForcesImitator(JointState())
    joint_subscriber = JointSubscriber(forcesImitator)
    robot_feedback_subscriber = RobotCollisionSubscriber(forcesImitator)
    environment_collision_subscriber = EnvironmentCollisionSubscriber(forcesImitator)

    rclpy.spin(joint_subscriber)
    rclpy.spin(robot_feedback_subscriber)
    rclpy.spin(environment_collision_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_subscriber.destroy_node()
    robot_feedback_subscriber.destroy_node()
    environment_collision_subscriber.destory_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()