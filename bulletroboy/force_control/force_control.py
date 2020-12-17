from rclpy.node import Node

class ForceControl(Node):
    def __init__(self):
        super().__init__("force_control")
        