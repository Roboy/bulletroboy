import os
import yaml
import io
import warnings


class Topics:

	# link poses topics
	VR_HEADSET_POSES 		= "/bullet_ik"
	OP_EF_POSES 			= "/roboy/simulation/operator/pose/endeffector"
	MAPPED_OP_REF_POSE 		= "/roboy/exoforce/pose/endeffector/right"
	MAPPED_OP_LEF_POSE 		= "/roboy/exoforce/pose/endeffector/left"
	ROBOY_EF_POSES 			= "/roboy/simulation/roboy/ef_pose"

	# roboy
	JOINT_STATES			= "/roboy/simulation/joint_state"

	# roboy plexus
	MOTOR_STATE 			= "/roboy/middleware/MotorState"
	MOTOR_COMMAND 			= "/roboy/middleware/MotorCommand"

	# collision
	ROBOY_COLLISIONS 		= "/roboy/simulation/collision"
	MAPPED_COLLISIONS 		= "/exoforce/simulation/collision"

	# cage
	CAGE_STATE				= "/exoforce/simulation/cage_state"
	CAGE_ROTATION 			= "/exoforce/simulation/cage_rotation"
	CAGE_END_EFFECTORS		= "/exoforce/configuration/end_effectors"

	# force
	TARGET_FORCE			= "/exoforce/force/target"


class Services:

	# roboy plexus services
	CONTROL_MODE 			= "/roboy/middleware/ControlMode"

	# links services
	LINK_INFO_FROM_NAME 	= "/roboy/simulation/operator/link_info_from_name"
	OP_INITIAL_LINK_POSE 	= "/roboy/simulation/operator/initial_link_pose"
	LINK_INFO_FROM_ID		= "/roboy/simulation/roboy/link_info_from_id"
	ROBOY_INITIAL_LINK_POSE	= "/roboy/simulation/roboy/initial_link_pose"

	INITIAL_HEAD_POSE		= "/roboy/simulation/exoforce/operator_initial_head_pose"

	# parameters
	STATE_MAPPER_GET		= "/state_mapper/get_parameters"

	# force
	START_FORCE_CONTROL		= "/exoforce/force/start"
	STOP_FORCE_CONTROL		= "/exoforce/force/stop"


def parse_launch_arg(arg, default_value, logger):
	"""Parse node args received from ros launcher.

	Parameters:
		arg (string): Argument received from launcher.
		default_value (string): Default value.
		logger (Logger): Node logger object.

	Returns:
		string: argument value
	"""
	if arg == '':
		return default_value
	else:
		if os.path.exists(arg):
			return arg
		else:
			logger(f"File '{arg}' does not exist, using default value '{default_value}")
			return default_value

def call_service(client, request, logger):
	"""Synchronous call to a service.

	Parameters:
		client (Client): The client used to communicate with the server
		request (SrvTypeRequest): The service request msg
		logger (Logger): Logger object

	Returns:
		SrvTypeResponse: Response msg from the call
	"""
	while not client.wait_for_service(timeout_sec=1.0):
		logger.info(f"'{client.srv_name}' service not available, waiting again...")
	response = client.call(request)
	return response
		
def call_service_async(client, request, callback, logger):
	while not client.wait_for_service(timeout_sec=1.0):
		logger.info(f"'{client.srv_name}' service not available, waiting again...")
	future = client.call_async(request)
	future.add_done_callback(callback)

def draw_AABB(pybullet, aabb, link_name):
	aabbMin = aabb[0]
	aabbMax = aabb[1]
	f = [aabbMin[0], aabbMin[1], aabbMin[2]]
	t = [aabbMax[0], aabbMin[1], aabbMin[2]]
	pybullet.addUserDebugLine(f, t, [1, 0, 0])
	f = [aabbMin[0], aabbMin[1], aabbMin[2]]
	t = [aabbMin[0], aabbMax[1], aabbMin[2]]
	pybullet.addUserDebugLine(f, t, [0, 1, 0])
	f = [aabbMin[0], aabbMin[1], aabbMin[2]]
	t = [aabbMin[0], aabbMin[1], aabbMax[2]]
	pybullet.addUserDebugLine(f, t, [0, 0, 1])

	f = [aabbMin[0], aabbMin[1], aabbMax[2]]
	t = [aabbMin[0], aabbMax[1], aabbMax[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMin[0], aabbMin[1], aabbMax[2]]
	t = [aabbMax[0], aabbMin[1], aabbMax[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMax[0], aabbMin[1], aabbMin[2]]
	t = [aabbMax[0], aabbMin[1], aabbMax[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMax[0], aabbMin[1], aabbMin[2]]
	t = [aabbMax[0], aabbMax[1], aabbMin[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMax[0], aabbMax[1], aabbMin[2]]
	t = [aabbMin[0], aabbMax[1], aabbMin[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMin[0], aabbMax[1], aabbMin[2]]
	t = [aabbMin[0], aabbMax[1], aabbMax[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMax[0], aabbMax[1], aabbMax[2]]
	t = [aabbMin[0], aabbMax[1], aabbMax[2]]
	pybullet.addUserDebugLine(f, t, [0, 0, 0])
	f = [aabbMax[0], aabbMax[1], aabbMax[2]]
	t = [aabbMax[0], aabbMin[1], aabbMax[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])
	f = [aabbMax[0], aabbMax[1], aabbMax[2]]
	t = [aabbMax[0], aabbMax[1], aabbMin[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])
	pybullet.addUserDebugText(link_name, [x + 0.005 for x in f], [0, 0, 0])	

# The following functions where replaced with a param file

def load_roboy_to_human_link_name_map():
	"""Fetches the link name map"""
	warnings.warn("load_roboy_to_human_link_name_map is deprecated", DeprecationWarning)

	script_dir = os.path.dirname(__file__)
	rel_path = "../../resource/roboy_to_human_linkname_map.yaml"
	abs_file_path = os.path.join(script_dir, rel_path)
	with open(abs_file_path) as f:
		linkNameMaps = yaml.safe_load(f)
		return linkNameMaps.get("roboyToHumanLinkNameMap")

def dump_op_link_dims(dims_dict):
	"""Saves the link dims dict"""
	warnings.warn("dump_op_link_dims is deprecated", DeprecationWarning)

	script_dir = os.path.dirname(__file__)
	rel_path = "../../resource/operator_link_standard_dimentions.yaml"
	abs_file_path = os.path.join(script_dir, rel_path)
	with io.open(abs_file_path, 'w') as f:
		yaml.dump({'operatorLinkDimentions':dims_dict}, f)

def load_op_link_dims():
	"""Fetches the link dims dict"""
	warnings.warn("load_op_link_dims is deprecated", DeprecationWarning)

	script_dir = os.path.dirname(__file__)
	rel_path = "../../resource/operator_link_standard_dimentions.yaml"
	abs_file_path = os.path.join(script_dir, rel_path)
	with open(abs_file_path) as f:
		dims_dict = yaml.load(f)
		print(dims_dict)
		return dims_dict.get("operatorLinkDimentions")
