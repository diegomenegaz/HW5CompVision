
from maestro import Controller
import time

HEADVERTPORT = 4
HEADHORPORT = 3
MPORT = 0
MPORT2 = 1
TPORT = 1
FORWARD = 8000
REVERSE = 1000
STOP = 6000
TURN_L = 3000
TURN_R = 7000
class MovementControl:
	_instance = None
	@staticmethod
	def getInst():
		if MovementControl._instance == None:
			MovementControl._instance = MovementControl()
		return MovementControl._instance
	def __init__(self):
		self.m = Controller()
		pass
	def spin_L(self,duration=1):
		print("Spinning Left")
		self.m.setTarget(TPORT,TURN_L) 
		time.sleep(duration)
		self.stop()
	def spin_R(self,duration=1):
		print("Spinning Right")
		self.m.setTarget(TPORT, TURN_R)
		time.sleep(duration)
		self.stop()
	def forward(self,duration=1):
		self.m.setTarget(MPORT, FORWARD)
		self.m.setTarget(MPORT2, REVERSE)
		time.sleep(duration)
		self.stop()
	def reverse(self,duration=1):
		self.m.setTarget(MPORT,REVERSE)
		self.m.setTarget(MPORT2,FORWARD)
		time.sleep(duration)
		self.stop()
	def stop(self):
		print("BREAKSSS")
		self.m.setTarget(MPORT, STOP)
		self.m.setTarget(TPORT, STOP)
	def level_head(self):
		self.m.setTarget(HEADVERTPORT, 4000)
		self.m.setTarget(HEADHORPORT, 5000 + 1000)

movement = MovementControl().getInst()
movement.forward()
