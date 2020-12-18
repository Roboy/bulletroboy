from rclpy.node import Node
from ..utils.load_cell import LoadCell, ConnectionError

class TendonForceController:
	"""This class handles the force controller for one individual tendon.

	"""
	def __init__(self, conf):
		self.id = conf['controller_id']
		self.set_point = 0
		self.kp = conf['kp']
		self.ki = conf['ki']
		self.kd = conf['kd']
		self.force_sensor = LoadCell(conf['load_cell_conf'])

	def connectToSensor(self):
		self.force_sensor.openChannel()

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
				('kd', None)
			]
		)
		self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().integer_value
		controllers_conf = self.get_controllers_conf()

		self.controllers = []
		for conf in controllers_conf:
			self.controllers.append(TendonForceController(conf))

	def get_controllers_conf(self):
		controllers_id = self.get_parameter('controllers_id').get_parameter_value().integer_array_value
		serials = self.get_parameter('serials').get_parameter_value().integer_array_value
		channels = self.get_parameter('channels').get_parameter_value().integer_array_value
		kp = self.get_parameter('kp').get_parameter_value().double_array_value
		ki = self.get_parameter('ki').get_parameter_value().double_array_value
		kd = self.get_parameter('kd').get_parameter_value().double_array_value

		load_cells_conf = [{'serial': s, 'channel': c} for s, c in zip(serials, channels)]

		return [{'controller_id': index, 'kp': p, 'ki': i , 'kd': d, 'load_cell_conf': conf} for index, p, i, d, conf in zip(controllers_id, kp, ki, kd, load_cells_conf)]
		
	def start_controllers(self):
		for controller in self.controllers:
			try:
				controller.connectToSensor()
			except ConnectionError as e:
				self.get_logger().warn(f"Failed to connect to load cell {controller.id}")
				self.get_logger().debug(f"Connection error to load cell {controller.id}: {e.message}")
