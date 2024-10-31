# ble_scan_connect.py:
from bluepy.btle import Peripheral, UUID
from bluepy.btle import Scanner, DefaultDelegate
import struct

class ScanDelegate(DefaultDelegate):
	def __init__(self):
		DefaultDelegate.__init__(self)
	def handleDiscovery(self, dev, isNewDev, isNewData):
		if isNewDev:
			print ("Discovered device", dev.addr)
		elif isNewData:
			print ("Received new data from", dev.addr)
			
class NotificationDelegate(DefaultDelegate):
	def __init__(self):
		DefaultDelegate.__init__(self)
	def handleNotification(self, cHandle, data):
		if cHandle == ch.getHandle():
			x, y, z = struct.unpack('<hhh', data)
			print(f"x, y, z = {x}, {y}, {z}")
scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(10.0)
n=0
addr = []
for dev in devices:
	if dev.addr == "f6:4c:0b:d8:93:d4":
		print ("%d: Device %s (%s), RSSI=%d dB" % (n, dev.addr,dev.addrType, dev.rssi))
		addr.append(dev.addr)
		n += 1
		for (adtype, desc, value) in dev.getScanData():
			print (" %s = %s" % (desc, value))
#number = input('Enter your device number: ')
#print ('Device', number)
#num = int(number)
#print (addr[num])
num = 0
#
print ("Connecting...")
dev = Peripheral(addr[num], 'random')
#
print ("Services...")
for svc in dev.services:
	print (str(svc.uuid))
#
try:
	dev.setDelegate(NotificationDelegate())
	testService = dev.getServiceByUUID(UUID("00000000-0001-11e1-9ab4-0002a5d5c51b"))
	for ch in testService.getCharacteristics():
		print (str(ch))
#	
	descriptors = dev.getDescriptors()
	#for desc in descriptors:
	#	print(desc)
	ch = dev.getCharacteristics(uuid=UUID("00e00000-0001-11e1-ac36-0002a5d5c51b"))[0]
	cccd = ch.getDescriptors(UUID(0x2902))[0]
	cccd.write(b"\x01\x00", withResponse = True)
	ch2 = dev.getCharacteristics(uuid=UUID("00000000-0001-11e1-ac36-0002a5d5c51b"))[0]
	sampling_frequency = 2
	ch2.write(sampling_frequency.to_bytes(1, byteorder='little'), withResponse=True)
	##cccd_handle = ch.getHandle() + 1
	##dev.writeCharacteristic(cccd_handle, b'\x01\x00', withResponse = True)
	
#
	while True:
		continue
finally:
	dev.disconnect()
