from redRobotSim import *
from time import clock

class Mode:
	SIMULATION = 0
	ACTUAL = 1

class Switch:
	def __init__(self, sim = 0):
		self.mode = Mode.SIMULATION
		self.sim = sim
		self.lastTime = 0

	def movePosX(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveX(1)
			self.lastTime = clock()

	def moveNegX(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveX(-1)
			self.lastTime = clock()

	def movePosY(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveY(1)
			self.lastTime = clock()

	def moveNegY(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveY(-1)
			self.lastTime = clock()

	def getPos(self):
		if (self.mode == Mode.SIMULATION):
			return self.sim.getCurrPosCamera()


	def inMotion(self):
		if (self.mode == Mode.SIMULATION):
			return (clock() - self.lastTime < 1)

