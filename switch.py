from redRobotSim import *
from time import clock
from common import *

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
			self.sim.positionFilter.setControlInput(Directions.PosX)
		elif (self.mode == Mode.ACTUAL):
			self.actual.motorPlan.setMotorNum(0)
			self.actual.motorPlan.setAngleIncrement(0.2)
			self.actual.motorPlan.start()
			self.actual.positionFilter.setControlInput(Directions.PosX)

	def moveNegX(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveX(-1)
			self.lastTime = clock()
			self.sim.positionFilter.setControlInput(Directions.NegX)
		elif (self.mode == Mode.ACTUAL):
			self.actual.motorPlan.setMotorNum(0)
			self.actual.motorPlan.setAngleIncrement(-0.2)
			self.actual.motorPlan.start()
			self.actual.positionFilter.setControlInput(Directions.NegX)

	def movePosY(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveY(1)
			self.lastTime = clock()
			self.sim.positionFilter.setControlInput(Directions.PosY)
		elif (self.mode == Mode.ACTUAL):
			self.actual.motorPlan.setMotorNum(1)
			self.actual.motorPlan.setAngleIncrement(0.2)
			self.actual.motorPlan.start()
			self.actual.positionFilter.setControlInput(Directions.PosY)

	def moveNegY(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveY(-1)
			self.lastTime = clock()
			self.sim.positionFilter.setControlInput(Directions.NegY)
		elif (self.mode == Mode.ACTUAL):
			self.actual.motorPlan.setMotorNum(1)
			self.actual.motorPlan.setAngleIncrement(-0.2)
			self.actual.motorPlan.start()
			self.actual.positionFilter.setControlInput(Directions.NegY)

	def getPos(self):
		"""
		Gets position in real coordinates
		"""
		if (self.mode == Mode.SIMULATION):
			return self.sim.positionFilter.getState().pos
		elif (self.mode == Mode.ACTUAL):
			return self.actual.positionFilter.getState().pos

	def inMotion(self):
		if (self.mode == Mode.SIMULATION):
			return (clock() - self.lastTime < 2.5)
		elif (self.mode == Mode.ACTUAL):
			return self.motorPlan.isRunning()
