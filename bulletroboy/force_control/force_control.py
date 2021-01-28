from rclpy.node import Node

from roboy_simulation_msgs.msg import TendonUpdate
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.srv import ControlMode

from ..utils.load_cell import LoadCell, ConnectionError
from ..utils.utils import Topics, Services

class TendonForceController:
	"""This class handles the force controller for one individual tendon.

	"""
	def __init__(self, conf, logger):
		"""
		Args:
			conf (dict): Dictionary with controller configuration:
							'controller_id' (int)	: Id of the corresponding motor.
							'kp' (float)			: Proportional gain.
							'ki' (float)			: Integral gain.
							'kd' (float)			: Derivative gain.
							'direction' (int)		: Direction of rotation of motor on cage.
							'pwm_limit' (float)		: Maximum PWM value for the motors.
							'load_cell_conf' (dict)	: Load cell configuration.
			logger (Logger): ROS logger object.
		
		"""
		self.logger = logger
		
		self.target_force = 0
		self.ready = False
		self.detached = False

		self.id = conf['controller_id']
		self.kp = conf['kp']
		self.ki = conf['ki']
		self.kd = conf['kd']
		self.direction = conf['direction']
		self.pwm_limit = conf['pwm_limit']

		self.p_error = 0
		self.i_error = 0
		self.d_error = 0

		self.force_sensor = LoadCell(conf['load_cell_conf'])
		self.force_sensor.setOnAttachHandler(self.onAttach)
		self.force_sensor.setOnDetachHandler(self.onDetach)

	def onAttach(self, channel):
		"""Event handler for the attach event of the LoadCell object.

		Args:
			-

		Returns:
			-
		
		"""
		self.ready = self.force_sensor.ready
		self.detached = False
		self.logger.info(f"Load Cell [{self.id}] atached!")
	
	def onDetach(self, channel):
		"""Event handler for the detach event of the LoadCell object.

		Args:
			-

		Returns:
			-
		
		"""
		self.detached = True
		self.logger.info(f"Load Cell [{self.id}] detached!")

	def connectToSensor(self):
		"""Opens and attaches the LoadCell object to a load cell channel.

		Args:
			-

		Returns:
			-
		
		"""
		self.force_sensor.openChannel()

	def getPwmSetPoint(self):
		"""Returns pwm control value calculated from current error.

		Args:
			-

		Returns:
			(float): PWM set point.
		
		"""
		if self.detached:
			self.ready = False
			return 0.0

		current_force = self.force_sensor.readForce()

		current_error = self.target_force - current_force

		self.d_error = self.p_error - current_error
		self.p_error = current_error
		self.i_error += current_error

		pwm_set_point = self.p_error * self.kp + self.i_error * self.ki + self.d_error * self.kd
		pwm_set_point = max(pwm_set_point, self.pwm_limit) * self.direction

		self.logger.info(f"Tendon [{self.id}] target [{self.target_force}] actual [{current_force}] pwm [{pwm_set_point}]")

		return pwm_set_point

class ForceControl(Node):
	"""This class handles the force control ROS node.

	"""
	def __init__(self):
		"""
		Args:
			-
		
		"""
		super().__init__("force_control")
		self.declare_parameters(
			namespace='',
			parameters=[
				('refresh_rate', None),
				('controllers_id', None),
				('cal_offset', None),
				('cal_factor', None),
				('serials', None),
				('channels', None),
				('kp', None),
				('ki', None),
				('kd', None),
				('direction', None),
				('pwm_limit', None)
			]
		)
		self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().integer_value
		controllers_conf = self.get_controllers_conf()

		self.controllers = []
		for conf in controllers_conf:
			self.controllers.append(TendonForceController(conf, self.get_logger()))

		self.create_subscription(TendonUpdate, Topics.TENDON_FORCE, self.set_target_force, 1)

		self.start_controllers()
		self.init_roboy_plexus()
		self.star_node()

	def get_controllers_conf(self):
		"""Reads node's parameters and builds list of configuration dictionaries.

		Args:
			-

		Returns:
			(list[dict]): List of controllers configurations.
		
		"""
		controllers_id = self.get_parameter('controllers_id').get_parameter_value().integer_array_value
		cal_offsets = self.get_parameter('cal_offset').get_parameter_value().double_array_value
		cal_factors = self.get_parameter('cal_factor').get_parameter_value().double_array_value
		serials = self.get_parameter('serials').get_parameter_value().integer_array_value
		channels = self.get_parameter('channels').get_parameter_value().integer_array_value
		kp = self.get_parameter('kp').get_parameter_value().double_array_value
		ki = self.get_parameter('ki').get_parameter_value().double_array_value
		kd = self.get_parameter('kd').get_parameter_value().double_array_value
		direction = self.get_parameter('direction').get_parameter_value().integer_array_value
		pwm_limit = self.get_parameter('pwm_limit').get_parameter_value().double_array_value

		load_cells_conf = [{'tendon_id': i, 'cal_offset': o, 'cal_factor': f, 'serial': s, 'channel': c} for i, o, f, s, c in zip(controllers_id, cal_offsets, cal_factors, serials, channels)]

		return [{'controller_id': index, 'kp': p, 'ki': i , 'kd': d, 'direction': di, 'pwm_limit': l,'load_cell_conf': conf} for index, p, i, d, di, l, conf in zip(controllers_id, kp, ki, kd, direction, pwm_limit, load_cells_conf)]
		
	def init_roboy_plexus(self):
		"""Initiliazes objects to interface with the roboy plexus.

		Args:
			-

		Returns:
			-
		
		"""
		self.motor_command_publisher = self.create_publisher(MotorCommand, Topics.MOTOR_COMMAND, 1)

		self.control_mode_client = self.create_client(ControlMode, Services.CONTROL_MODE)
		while not self.control_mode_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('roboy_plexus not available, waiting again...')
		self.get_logger().info('Succesfull connection to roboy plexus!')

		self.control_mode_req = ControlMode.Request()
		self.motor_command_msg = MotorCommand()

	def start_controllers(self):
		"""Connects controller with load cells.

		Args:
			-

		Returns:
			-
		
		"""
		for controller in self.controllers:
			try:
				controller.connectToSensor()
			except ConnectionError as e:
				self.get_logger().warn(f"Failed to connect to load cell {controller.id}")
				self.get_logger().debug(f"Connection error to load cell {controller.id}: {e.message}")

	def star_node(self):
		"""Starts control loop.

		Args:
			-

		Returns:
			-
		
		"""
		self.set_control_mode(3)
		self.create_timer(1 / self.refresh_rate, self.control_loop)

	def get_controller(self, id):
		"""Returns controller with requested id.

		Args:
			id (int): Id of desired controller.

		Returns:
			(TendonForceController): Contoller object.
		
		"""
		for controller in self.controllers:
			if controller.id == id:
				return controller
		return None

	def set_target_force(self, msg):
		"""Set tendon target force callback.

		Args:
			msg (TendonUpdate): Received ROS Message.

		Returns:
			-
		
		"""
		controller = self.get_controller(msg.id)
		if controller is None:
			self.get_logger().warn(f"Trying to set target force for tendon ({msg.id}) without controller.")
		else:
			if msg.force < 0:
				self.get_logger().warn(f"Trying to set negative target force for tendon ({msg.id}).")
			else:
				controller.target_force = msg.force

	def control_loop(self):
		"""Control loop timer callback.

		Args:
			-

		Returns:
			-
		
		"""
		motor_ids = []
		set_points = []
		for controller in self.controllers:
			if controller.ready:
				motor_ids.append(controller.id)
				set_points.append(controller.getPwmSetPoint())

		self.send_motor_commands(motor_ids, set_points)

	def set_control_mode(self, mode, motor_ids=None, set_points=[]):
		"""Set motors control mode.

		Args:
			mode (int)				: Control mode:
										0: ENCODER0_POSITION: position controller using encoder0
										1: ENCODER1_POSITION: position controller using encoder1
										2: DISPLACEMENT: position controller using displacement
										3: DIRECT_PWM: directly controlling the pwm value of the motor
			motor_ids (list[int]) 	: Motor list to change mode, if None, all motors will be updated.
			set_points (list[float]): Set points for the corresponding motor list, if empty all motors will be set to 0.

		Returns:
			-
		
		"""
		self.control_mode_req.legacy = False
		self.control_mode_req.control_mode = mode
		self.control_mode_req.motor_id = motor_ids if motor_ids is not None else [controller.id for controller in self.controllers]
		self.control_mode_req.set_points = set_points if motor_ids is not None else [0] * len(self.control_mode_req.motor_id)

		self.control_mode_client.call_async(self.control_mode_req)

	def send_motor_commands(self, motor_ids=None, set_points=None):
		"""Set motors set point.

		Args:
			motor_ids (list[int]) 	: Motor list to set, if None, all motors will be updated.
			set_points (list[float]): Set points for the corresponding motor list, if None all motors will be set to 0.

		Returns:
			-
		
		"""
		if motor_ids is None:
			motor_ids = [controller.id for controller in self.controllers]
		if set_points is None:
			set_points = [0.0] * len(motor_ids)

		self.motor_command_msg.legacy = False
		self.motor_command_msg.motor = motor_ids
		self.motor_command_msg.setpoint = set_points

		self.motor_command_publisher.publish(self.motor_command_msg)
