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
			self.actual.movePosXPlan.start()

	def moveNegX(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveX(-1)
			self.lastTime = clock()
		elif (self.mode == Mode.ACTUAL):
			self.actual.moveNegX.start()

	def movePosY(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveY(1)
			self.lastTime = clock()
		elif (self.mode == Mode.ACTUAL):
			# self.actual.movePosYPlan.start()
			pass

	def moveNegY(self):
		if (self.mode == Mode.SIMULATION):
			self.sim.moveY(-1)
			self.lastTime = clock()
		elif (self.mode == Mode.ACTUAL):
			# self.actual.moveNegYPlan.start()
			pass

	def getPos(self):
		"""
		Gets position in real coordinates
		"""
		if (self.mode == Mode.SIMULATION):
			return self.sim.pos
		elif (self.mode == Mode.ACTUAL):
			return self.actual.particleFilter.getState()

	def inMotion(self):
		if (self.mode == Mode.SIMULATION):
			return (clock() - self.lastTime < 1)
		elif (self.mode == Mode.ACTUAL):
			in_motion = False
			in_motion = in_motion or self.actual.movePosXPlan.running()
			in_motion = in_motion or self.actual.moveNegXPlan.running()
			# in_motion = in_motion or self.actual.movePosYPlan.running()
			# in_motion = in_motion or self.actual.moveNegYPlan.running()
			return in_motion

