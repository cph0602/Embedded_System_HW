# ble_scan_connect.py:
from bluepy.btle import Peripheral, UUID
from bluepy.btle import Scanner, DefaultDelegate

class ScanDelegate(DefaultDelegate):
	def __init__(self):
		DefaultDelegate.__init__(self)
	def handleDiscovery(self, dev, isNewDev, isNewData):
		if isNewDev:
			print ("Discovered device", dev.addr)
		elif isNewData:
			print ("Received new data from", dev.addr)
scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(10.0)
n=0
addr = []
for dev in devices:
	print ("%d: Device %s (%s), RSSI=%d dB" % (n, dev.addr,dev.addrType, dev.rssi))
	addr.append(dev.addr)
	n += 1
	for (adtype, desc, value) in dev.getScanData():
		print (" %s = %s" % (desc, value))
number = input('Enter your device number: ')
print ('Device', number)
num = int(number)
print (addr[num])
#
print ("Connecting...")
dev = Peripheral(addr[num], 'public')
#
print ("Services...")
for svc in dev.services:
	print (str(svc))
#
try:
	testService = dev.getServiceByUUID(UUID(0x180D))
	for ch in testService.getCharacteristics():
		print (str(ch))
#	
	descriptors = dev.getDescriptors()
	#for desc in descriptors:
	#	print(desc)
	ch = dev.getCharacteristics(uuid=UUID(0x2A37))[0]
	cccd = ch.getDescriptors(UUID(0x2902))[0]
	cccd.write(b"\x01\x00", withResponse = True)
	#cccd_handle = ch.getHandle() + 1
	#dev.writeCharacteristic(cccd_handle, b'\x01\x00', withResponse = True)
	if (ch.supportsRead()):
		print (ch.read())
#
	while True:
		if dev.waitForNotifications(1.0):
			print("Notification received")
finally:
	dev.disconnect()
