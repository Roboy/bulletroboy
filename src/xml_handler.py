import xml.etree.ElementTree as ET
from xml.dom import minidom

def read_cage_conf(filepath):
    """
    cageConf, muscleUnits, tendons = read_cage_conf('cageConfiguration.xml')

    Reads exoforce cage configuration file.
    
    Parameters
    ----------
    - filepath: path to XML conf file

    Returns
    -------
    - cageStruc: dictionary
        {
        'height': cage height (cm)
        'radius': cage radius (cm)
        }
    - muscleUnits: list of dictionaries
        [
            {
                'name': muscle unit name
                'height': attatchment height in the cage (cm)
                'angle': attatchment angle in cage (°)
            }
            ...
        ] 
    - tendons: list of dictionaries
        [   
            {
                'name': tendon name
                'link': name of attachtment link
                'viaPoint': attachtment point in the link
            }
        ]
    """
    # parsing XML file
    tree = ET.parse(filepath)
    root = tree.getroot()

    # getting main items
    cage_xml = root.find('cageStructure')
    muscles_xml = root.find('muscleUnits')
    tendons_xml = root.find('tendons')

    # parsing cage structure
    cageStruc = {}
    cageStruc['height'] = int(cage_xml.find('height').text)
    cageStruc['radius'] = int(cage_xml.find('radius').text)

    # parsing muscle units
    muscleUnits = []
    for muscle in muscles_xml:
        muscle_dict = {}
        muscle_dict['name'] = muscle.get('name')
        muscle_dict['height'] = int(muscle.find('height').text)
        muscle_dict['angle'] = int(muscle.find('angle').text)
        muscleUnits.append(muscle_dict)

    # parsing tendons
    tendons = []
    for tendon in tendons_xml:
        tendon_dict = {}
        tendon_dict['name'] = tendon.get('name')
        tendon_dict['link'] = tendon.find('link').get('name')
        tendon_dict['viaPoint'] = list(map(float, tendon.find('link').find('viaPoint').text.split(" ")))
        tendons.append(tendon_dict)

    return cageStruc, muscleUnits, tendons

def write_cage_conf(filepath, cageStruc, muscleUnits, tendons):
    """
    write_cage_conf('cageConfiguration.xml', cageStruc, muscleUnits, tendons)

    Writes exoforce cage configuration file.
    
    Parameters
    ----------
    - filepath: path to XML conf file
    - cageStruc: dictionary
        {
        'height': cage height (cm)
        'radius': cage radius (cm)
        }
    - muscleUnits: list of dictionaries
        [
            {
                'name': muscle unit name
                'height': attatchment height in the cage (cm)
                'angle': attatchment angle in cage (°)
            }
            ...
        ] 
    - tendons: list of dictionaries
        [   
            {
                'name': tendon name
                'link': name of attachtment link
                'viaPoint': attachtment point in the link
            }
        ]
    """
    # main item
    cageConfiguration = ET.Element('cageConfiguration')

    # cageStructure
    cage_xml = ET.SubElement(cageConfiguration, 'cageStructure')
    cage_height = ET.SubElement(cage_xml, 'height')
    cage_radius = ET.SubElement(cage_xml, 'radius')
    cage_height.text = str(cageStruc['height'])
    cage_radius.text = str(cageStruc['radius'])

    # muscle Units
    muscles_xml = ET.SubElement(cageConfiguration, 'muscleUnits')
    for muscle in muscleUnits:
        muscle_xml = ET.SubElement(muscles_xml, 'muscleUnit')
        muscle_xml.set('name', muscle['name'])
        muscle_height = ET.SubElement(muscle_xml, 'height')
        muscle_angle = ET.SubElement(muscle_xml, 'angle')
        muscle_height.text = str(muscle['height'])
        muscle_angle.text = str(muscle['angle'])

    # tendons
    tendons_xml = ET.SubElement(cageConfiguration, 'tendons')
    for tendon in tendons:
        tendon_xml = ET.SubElement(tendons_xml, 'tendon')
        tendon_xml.set('name', tendon['name'])
        link = ET.SubElement(tendon_xml, 'link')
        link.set('name', tendon['link'])
        viaPoint = ET.SubElement(link, 'viaPoint')
        viaPoint.set('type', 'FIXPOINT')
        viaPoint.text = str(tendon['viaPoint']).strip('[]').replace(',', '')

    # write to file
    mydata = ET.tostring(cageConfiguration).decode()
    reparsed = minidom.parseString(mydata)
    mydata = reparsed.toprettyxml(indent="\t")
    myfile = open(filepath, "w")
    myfile.write(mydata)
    myfile.close()


if __name__ == '__main__':

    cageConf, muscleUnits, tendons = read_cage_conf('../config/cageConfiguration.xml')

    print("Cage Structure:")
    print("\theight: {}".format(cageConf["height"]))
    print("\tradius: {}".format(cageConf["radius"]))

    print("\nMuscle Units:")
    for muscle in muscleUnits:
        print("\t[{}]:".format(muscle["name"]))
        print("\t\theight: {}".format(muscle["height"]))
        print("\t\tangle: {}".format(muscle["angle"]))

    print("\nTendons:")
    for tendon in tendons:
        print("\t[{}]:".format(tendon["name"]))
        print("\t\tlink: {}".format(tendon["link"]))
        print("\t\tviaPoint: {}".format(tendon["viaPoint"]))

    # write_cage_conf('../config/test_output.xml', cageConf, muscleUnits, tendons)
