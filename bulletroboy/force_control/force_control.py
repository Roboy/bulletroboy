from rclpy.node import Node

from roboy_simulation_msgs.msg import TendonUpdate
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.srv import ControlMode

from ..utils.load_cell import LoadCell, ConnectionError
from ..utils.utils import Topics, Services

class TendonForceController:
	"""This class handles the force controller for one individual tendon.

	"""
	def __init__(self, conf):
		self.id = conf['controller_id']
		self.set_point = 0
		self.kp = conf['kp']
		self.ki = conf['ki']
		self.kd = conf['kd']
		self.direction = conf['direction']
		self.force_sensor = LoadCell(conf['load_cell_conf'])

	@property
	def ready(self):
		self.force_sensor.getAttached()

	def connectToSensor(self):
		self.force_sensor.openChannel()

	def getPwmSetPoint(self):
		return 0 * self.direction

class ForceControl(Node):
	"""This class handles the force control ROS node.

	"""
	def __init__(self):
		super().__init__("force_control")
		self.declare_parameters(
			namespace='',
			parameters=[
				('refresh_rate', None),
				('controllers_id', None),
				('serials', None),
				('channels', None),
				('kp', None),
				('ki', None),
				('kd', None),
				('direction', None)
			]
		)
		self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().integer_value
		controllers_conf = self.get_controllers_conf()

		self.controllers = []
		for conf in controllers_conf:
			self.controllers.append(TendonForceController(conf))

		self.create_subscription(TendonUpdate, Topics.SET_TENDON_FORCE, self.set_force_set_point, 1)

		self.start_controllers()
		self.connect_to_roboy_plexus()
		self.star_node()

	def get_controllers_conf(self):
		controllers_id = self.get_parameter('controllers_id').get_parameter_value().integer_array_value
		serials = self.get_parameter('serials').get_parameter_value().integer_array_value
		channels = self.get_parameter('channels').get_parameter_value().integer_array_value
		kp = self.get_parameter('kp').get_parameter_value().double_array_value
		ki = self.get_parameter('ki').get_parameter_value().double_array_value
		kd = self.get_parameter('kd').get_parameter_value().double_array_value
		direction = self.get_parameter('direction').get_parameter_value().integer_array_value

		load_cells_conf = [{'serial': s, 'channel': c} for s, c in zip(serials, channels)]

		return [{'controller_id': index, 'kp': p, 'ki': i , 'kd': d, 'direction': di, 'load_cell_conf': conf} for index, p, i, d, di, conf in zip(controllers_id, kp, ki, kd, direction, load_cells_conf)]
		
	def connect_to_roboy_plexus(self):
		self.motor_command_publisher = self.create_publisher(MotorCommand, Topics.MOTOR_COMMAND, 1)

		self.control_mode_client = self.create_client(ControlMode, Services.CONTROL_MODE)
		while not self.control_mode_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('roboy_plexus not available, waiting again...')
		self.get_logger().info('Succesfull connection to roboy plexus!')

		self.control_mode_req = ControlMode.Request()
		self.motor_command_msg = MotorCommand()

		self.set_control_mode(3)

	def start_controllers(self):
		for controller in self.controllers:
			try:
				controller.connectToSensor()
			except ConnectionError as e:
				self.get_logger().warn(f"Failed to connect to load cell {controller.id}")
				self.get_logger().debug(f"Connection error to load cell {controller.id}: {e.message}")

	def star_node(self):
		self.create_timer(1 / self.refresh_rate, self.control_loop)

	def get_controller(self, id):
		for controller in self.controllers:
			if controller.id == id:
				return controller
		return None

	def set_force_set_point(self, msg):
		self.get_controller(msg.id).set_point = msg.force

	def control_loop(self):
		motor_ids = []
		set_points = []
		for controller in self.controllers:
			if controller.ready:
				motor_ids.append(controller.id)
				set_points.append(controller.getPwmSetPoint())

		self.send_motor_commands(motor_ids, set_points)

	def set_control_mode(self, mode, motor_ids=None, set_points=[]):
		self.control_mode_req.legacy = False
		self.control_mode_req.control_mode = mode
		self.control_mode_req.motor_id = motor_ids if motor_ids is not None else [controller.id for controller in self.controllers]
		self.control_mode_req.set_points = set_points if motor_ids is not None else [0] * len(self.control_mode_req.motor_id)

		self.control_mode_client.call_async(self.control_mode_req)

	def send_motor_commands(self, motor_ids=None, set_points=None):
		if motor_ids is None:
			motor_ids = [controller.id for controller in self.controllers]
		if set_points is None:
			set_points = [0.0] * len(motor_ids)

		self.motor_command_msg.legacy = False
		self.motor_command_msg.motor = motor_ids
		self.motor_command_msg.setpoint = set_points

		self.motor_command_publisher.publish(self.motor_command_msg)
