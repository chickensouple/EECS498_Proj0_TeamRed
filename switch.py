from redRobotSim import *
from time import clock
from mode import *

class Switch:
	def __init__(self, mode, sim, actual):
		self.mode = mode
		self.sim = sim
		self.actual = actual
		self.lastTime = 0

	def movePosX(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveX(1)
			self.lastTime = clock()
		elif (self.mode == Mode.ACTUAL):
			self.actual.motorPlan.setMotorNum(0)
			self.actual.motorPlan.setAngleIncrement(0.2)
			self.actual.motorPlan.start()

	def moveNegX(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveX(-1)
			self.lastTime = clock()
		elif (self.mode == Mode.ACTUAL):
			self.actual.motorPlan.setMotorNum(0)
			self.actual.motorPlan.setAngleIncrement(-0.2)
			self.actual.motorPlan.start()

	def movePosY(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveY(1)
			self.lastTime = clock()
		elif (self.mode == Mode.ACTUAL):
			self.actual.motorPlan.setMotorNum(1)
			self.actual.motorPlan.setAngleIncrement(0.2)
			self.actual.motorPlan.start()

	def moveNegY(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveY(-1)
			self.lastTime = clock()
		elif (self.mode == Mode.ACTUAL):
			self.actual.motorPlan.setMotorNum(1)
			self.actual.motorPlan.setAngleIncrement(-0.2)
			self.actual.motorPlan.start()

	def getPos(self):
		"""
		Gets position in real coordinates
		"""
		if (self.mode == Mode.SIMULATION):
			return self.sim.particleFilter.getState().pos
		elif (self.mode == Mode.ACTUAL):
			return self.actual.particleFilter.getState().pos

	def inMotion(self):
		if (self.mode == Mode.SIMULATION):
			return (clock() - self.lastTime < 1)
		elif (self.mode == Mode.ACTUAL):
			return self.motorPlan.isRunning()
