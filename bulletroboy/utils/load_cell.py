
class ConnectionError(Exception):
    pass

# TODO: Implement LoadCell class for interfacing with the load cells.

class LoadCell:
    def __init__(self, conf):
        pass

    def openChannel(self):
        pass

    def readForce(self):
        return 0

    def getAttached(self):
        return False

    def setOnAttachHandler(self, handler):
        pass

    def setOnDetachHandler(self, handler):
        pass