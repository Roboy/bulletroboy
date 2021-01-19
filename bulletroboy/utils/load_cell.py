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
							'tendon_id' 	(int): Tendon id to which the cell is attached.
							'cal_offset'  (float): Calibration offset value of the load cell.
							'cal_factor'  (float): Calibration factor value of the load cell.
							'serial' 		(int): Serial number of the phidget to which the cell is connected.
							'channel' 		(int): Number of the phidget channel to which the cell is connected.
		
		"""
		super().__init__()
		self.id = conf['tendon_id']
		self.cal_offset = conf['cal_offset']
		self.cal_factor = conf['cal_factor']
		self.setDeviceSerialNumber(conf['serial'])
		self.setChannel(conf['channel'])

	def getAddress(self):
		return f"{self.getDeviceSerialNumber()}/{self.getChannel()}"

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

	def closeChannel(self):
		self.close()

	def readForce(self):
		"""Reads force value from load cell in Newtons.
		
		Args:
			-

		Returns:
		   	float: Force value in Newtons.

		"""
		force = None
		if self.cal_factor is not None and self.cal_offset is not None:
			force = self.cal_factor * (self.getVoltageRatio() + self.cal_offset)
		return force



# provisional code for class testing

def onVoltageRatioChange(channel, voltageRatio):
	print(f"Force [{channel.id}] ({channel.getAddress()}): {channel.readForce():.1f} N")

phidget_serial = 585078
cal_offsets = []
cal_factors = []

configuration = [{'tendon_id': i, 'cal_offset': o, 'cal_factor': f, 'serial': phidget_serial, 'channel': i} for i, (o, f) in enumerate(zip(cal_offsets, cal_factors))]

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
		channel.closeChannel()
