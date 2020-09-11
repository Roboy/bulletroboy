import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
import os
import yaml

class CageConfiguration():
	"""This class handles the initial cage configuration.
		It reads and writes from a XML file.
	"""
	def __init__(self, filepath=None):
		"""
		Args:
			filepath (string): Path to XML configuration file. Default None.

		"""
		self.filepath = filepath
		self.cage_structure = {}
		self.muscle_units = []
		
		if self.filepath:
			self.load(filepath)

	def load(self, filepath):
		# parsing XML file
		tree = ET.parse(filepath)
		root = tree.getroot()

		# getting main items
		cage_xml = root.find('cageStructure')
		muscles_xml = root.find('muscleUnits')

		# parsing cage structure
		self.cage_structure['height'] = int(cage_xml.find('height').text)
		self.cage_structure['radius'] = int(cage_xml.find('radius').text)

		# parsing muscle units
		for muscle in muscles_xml:
			muscle_dict = {}
			muscle_dict['id'] = int(muscle.get('id'))
			muscle_dict['viaPoints'] = []
			for via_point_xml in muscle.find('viaPoints'):
				via_point = {}
				via_point['id'] = int(via_point_xml.get('id'))
				via_point['link'] = via_point_xml.get('link')
				via_point['point'] = np.array(list(map(float, via_point_xml.text.split(" "))))
				muscle_dict['viaPoints'].append(via_point)
			muscle_dict['parameters'] = {}
			for parameter in muscle.find('parameters'):
				muscle_dict['parameters'][parameter.tag] = parameter.text
			self.muscle_units.append(muscle_dict)

	def save(self, filepath=None):
		if filepath is None: filepath = self.filepath

		# main item
		cageConfiguration = ET.Element('cageConfiguration')

		# cageStructure
		cage_xml = ET.SubElement(cageConfiguration, 'cageStructure')
		cage_height = ET.SubElement(cage_xml, 'height')
		cage_radius = ET.SubElement(cage_xml, 'radius')
		cage_height.text = str(self.cage_structure['height'])
		cage_radius.text = str(self.cage_structure['radius'])

		# muscle Units
		muscles_xml = ET.SubElement(cageConfiguration, 'muscleUnits')
		for muscle in self.muscle_units:
			muscle_xml = ET.SubElement(muscles_xml, 'muscleUnit')
			muscle_xml.set('id', str(muscle['id']))
			muscle_via_points = ET.SubElement(muscle_xml, 'viaPoints')
			for via_point in muscle['viaPoints']:
				via_point_xml = ET.SubElement(muscle_via_points, 'viaPoint')
				via_point_xml.set('id', str(via_point['id']))
				via_point_xml.set('link', via_point['link'])
				via_point_xml.text = str(via_point['point'].tolist()).strip('[]').replace(',', '')
			muscle_parameters = ET.SubElement(muscle_xml, 'parameters')
			for k, v in muscle['parameters'].items():
				subelement = ET.SubElement(muscle_parameters, k)
				subelement.text = v

		# write to file
		mydata = ET.tostring(cageConfiguration).decode()
		reparsed = minidom.parseString(mydata)
		mydata = reparsed.toprettyxml(indent="\t")
		myfile = open(filepath, "w")
		myfile.write(mydata)
		myfile.close()

	def __str__(self):
		msg = f"\nCAGE CONFIGURATION: ({self.filepath})\n"
		msg += "-------------------\n"
		msg += "Cage Structure:\n"
		msg += f"\theight: {self.cage_structure['height']}\n"
		msg += f"\tradius: {self.cage_structure['radius']}\n"
		msg += "Muscle Units:\n"
		for muscle in self.muscle_units:
			msg += "\t-----------------\n"
			msg += f"\tid: {muscle['id']}\n"
			msg += "\tvia points:\n"
			for via_point in muscle['viaPoints']:
				msg += f"\t\t{via_point['id']} link: {via_point['link']:<20}\tpoint: {via_point['point']}\n"
			msg += "\tparameters:\n"
			for k, v in muscle['parameters'].items():
				msg += f"\t\t{k}: {v}\n"
		return msg

def call_service(node, client, msg):
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("service not available, waiting again...")
        response = client.call(msg)
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
