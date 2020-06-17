import yaml

class LinksMapper(object):
    def __init__(self, humandURDFFileName, roboyURDFFileName):
        """
        This class maps the ids of human links to robot links and vice versa
        """
        super().__init__()

        with open('../resources/roboy_to_human_linknam_map.yaml') as f:
            linkNameMaps = yaml.safe_load(f)
            self.roboyToHumanLinkNameMap = linkNameMaps.get("roboyToHumanLinkNameMap")

    def humanLinkName(self, robotLinkName):
        return None

    def robotLinkId(self, humanLinkIndex):
        return None
