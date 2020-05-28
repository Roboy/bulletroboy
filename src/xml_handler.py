import xml.etree.ElementTree as ET
from xml.dom import minidom

class CageConfiguration():
    def __init__(self, filepath=None):
        self.filepath = filepath
        self.cage_structure = {}
        self.tendons = []
        self.muscleUnits = []

        if self.filepath:
            self.load(filepath)

    def load(self, filepath):
        # parsing XML file
        tree = ET.parse(filepath)
        root = tree.getroot()

        # getting main items
        cage_xml = root.find('cageStructure')
        muscles_xml = root.find('muscleUnits')
        tendons_xml = root.find('tendons')

        # parsing cage structure
        self.cage_structure['height'] = int(cage_xml.find('height').text)
        self.cage_structure['radius'] = int(cage_xml.find('radius').text)

        # parsing muscle units
        for muscle in muscles_xml:
            muscle_dict = {}
            muscle_dict['name'] = muscle.get('name')
            muscle_dict['height'] = int(muscle.find('height').text)
            muscle_dict['angle'] = int(muscle.find('angle').text)
            muscle_dict['parameters'] = muscle.find('parameters').attrib
            self.muscleUnits.append(muscle_dict)

        # parsing tendons
        for tendon in tendons_xml:
            tendon_dict = {}
            tendon_dict['name'] = tendon.get('name')
            tendon_dict['muscle'] = tendon.find('muscle').get('name')
            tendon_dict['link'] = tendon.find('link').get('name')
            tendon_dict['viaPoint'] = list(map(float, tendon.find('link').find('viaPoint').text.split(" ")))
            self.tendons.append(tendon_dict)

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
        for muscle in self.muscleUnits:
            muscle_xml = ET.SubElement(muscles_xml, 'muscleUnit')
            muscle_xml.set('name', muscle['name'])
            muscle_height = ET.SubElement(muscle_xml, 'height')
            muscle_angle = ET.SubElement(muscle_xml, 'angle')
            muscle_parameters = ET.SubElement(muscle_xml, 'parameters')
            muscle_height.text = str(muscle['height'])
            muscle_angle.text = str(muscle['angle'])
            for k, v in muscle['parameters'].items():
                muscle_parameters.set(k, v)

        # tendons
        tendons_xml = ET.SubElement(cageConfiguration, 'tendons')
        for tendon in self.tendons:
            tendon_xml = ET.SubElement(tendons_xml, 'tendon')
            tendon_xml.set('name', tendon['name'])
            muscle = ET.SubElement(tendon_xml, 'muscle')
            link = ET.SubElement(tendon_xml, 'link')
            muscle.set('name', tendon['name'])
            link.set('name', tendon['link'])
            viaPoint = ET.SubElement(link, 'viaPoint')
            viaPoint.text = str(tendon['viaPoint']).strip('[]').replace(',', '')

        # write to file
        mydata = ET.tostring(cageConfiguration).decode()
        reparsed = minidom.parseString(mydata)
        mydata = reparsed.toprettyxml(indent="\t")
        myfile = open(filepath, "w")
        myfile.write(mydata)
        myfile.close()


if __name__ == '__main__':

    cage_config = CageConfiguration('../config/cageConfiguration.xml')

    cageStruct = cage_config.cage_structure
    muscleUnits = cage_config.muscleUnits
    tendons = cage_config.tendons

    print("Cage Structure:")
    print("\theight: {}".format(cageStruct["height"]))
    print("\tradius: {}".format(cageStruct["radius"]))

    print("\nMuscle Units:")
    for muscle in muscleUnits:
        print("\t[{}]:".format(muscle["name"]))
        print("\t\theight: {}".format(muscle["height"]))
        print("\t\tangle: {}".format(muscle["angle"]))
        print("\t\tparameters: {}".format(muscle["parameters"]))

    print("\nTendons:")
    for tendon in tendons:
        print("\t[{}]:".format(tendon["name"]))
        print("\t\tmuscle: {}".format(tendon["muscle"]))
        print("\t\tlink: {}".format(tendon["link"]))
        print("\t\tviaPoint: {}".format(tendon["viaPoint"]))

    cage_config.save('../config/test_output.xml')
