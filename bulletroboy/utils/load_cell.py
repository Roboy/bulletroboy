from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *


class ConnectionError(Exception):
	"""Raised when connection to load cell channels fails."""
	def __init__(self, message):
		self.message = message


class LoadCell(VoltageRatioInput):
	"""This class handles a single load cell channel.
	
	"""
	def __init__(self, conf):
		"""
		Args:
			conf (dict): Dictionary with initial channel configuration:
							'serial' (int): 	Serial number of the phidget to which the cell is connected.
							'channel' (int): 	Number of the phidget channel to which the cell is connected.
		
		"""
		super().__init__()
		self.setDeviceSerialNumber(conf['serial'])
		self.setChannel(conf['channel'])

	def openChannel(self):
		"""Opens and attaches to phidget channel.
		
		Args:
			-

		Returns:
		   	-

		"""
		try:
			self.openWaitForAttachment(Phidget.DEFAULT_TIMEOUT)
		except PhidgetException as e:
			raise ConnectionError(e.details)

	def readForce(self):
		"""Reads force value from load cell.
		
		Args:
			-

		Returns:
		   	float: Force value in Newtons.

		"""
		return 0.0



# provisional code for class testing

def onVoltageRatioChange(channel, voltageRatio):
	print(f"VoltageRatio [{channel.id}]: {voltageRatio}")

configuration = [
	{
		'motor_id': 0,
		'serial': 25345234,
		'channel': 0
	},
	{
		'motor_id': 1,
		'serial': 25345234,
		'channel': 1
	},
	{
		'motor_id': 2,
		'serial': 25345234,
		'channel': 2
	},
	{
		'motor_id': 3,
		'serial': 25345234,
		'channel': 3
	},
]

if __name__ == "__main__":

	channels = []
	for i, conf in enumerate(configuration):
		new_channel = LoadCell(conf)
		new_channel.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
		try:
			new_channel.openChannel()
			channels.append(new_channel)
		except ConnectionError as e:
			print(f"Failed to open load cell {i}: {e.message}")
			
	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	# Close your Phidgets once the program is done.
	for channel in channels:
		channel.close()
