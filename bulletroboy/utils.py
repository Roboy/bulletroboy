import os
import yaml
import io

def call_service(node, client, msg):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("service not available, waiting again...")
    response = client.call(msg)
    node.get_logger().info("Got response")
    return response
        
def call_service_async(self, client, req):
    while not client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("service not available, waiting again...")
    self.future = self.client.call_async(req)
        
def load_roboy_to_human_link_name_map():
    """Fetches the link name map"""
    script_dir = os.path.dirname(__file__)
    rel_path = "../resource/roboy_to_human_linkname_map.yaml"
    abs_file_path = os.path.join(script_dir, rel_path)
    with open(abs_file_path) as f:
        linkNameMaps = yaml.safe_load(f)
        return linkNameMaps.get("roboyToHumanLinkNameMap")

def dump_op_link_dims(dims_dict):
    """Saves the link dims dict"""
    script_dir = os.path.dirname(__file__)
    rel_path = "../resource/operator_link_standard_dimentions.yaml"
    abs_file_path = os.path.join(script_dir, rel_path)
    with io.open(abs_file_path, 'w') as f:
        yaml.dump({'operatorLinkDimentions':dims_dict}, f)

def load_op_link_dims():
    """Fetches the link dims dict"""
    script_dir = os.path.dirname(__file__)
    rel_path = "../resource/operator_link_standard_dimentions.yaml"
    abs_file_path = os.path.join(script_dir, rel_path)
    with open(abs_file_path) as f:
        dims_dict = yaml.load(f)
        return dims_dict.get("operatorLinkDimentions")

def draw_AABB(pybullet, aabb):
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
	pybullet.addUserDebugLine(f, t, [1.0, 0.5, 0.5])
	f = [aabbMax[0], aabbMax[1], aabbMax[2]]
	t = [aabbMax[0], aabbMin[1], aabbMax[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])
	f = [aabbMax[0], aabbMax[1], aabbMax[2]]
	t = [aabbMax[0], aabbMax[1], aabbMin[2]]
	pybullet.addUserDebugLine(f, t, [1, 1, 1])