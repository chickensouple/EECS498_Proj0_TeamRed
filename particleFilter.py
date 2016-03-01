from directions import *
from state import *
from constants import *
from numpy.random import randn

class Particle:
	def __init__(self, pos=[0, 0], yaw=0, prob = 1):
		self.state = RobotState(pos, yaw)
		self.prob = prob

class ParticleFilter:
	def __init__(self):
		# noise constants
		self.xNoise = 0.1 # cm
		self.yNoise = 0.1 # cm
		self.xYawBias = 0.001 # radians
		self.yYawBias = 0.001 # radians
		self.yawBias = 0.002 # radians

		self.numParticles = 80
		self.particles = []
		self.wheelSideLength = wheelSideLength;
		for i in range(self.numParticles):
			self.particles.append(RobotState())

	def update(self, direction):
		# move each particle with noise
		for particle in self.particles:
			if (direction == Directions.PosX):
				particle.state.pos[0] += self.wheelSideLength + randn() * self.xNoise
				particle.state.yaw += self.xYawBias
			elif (direction == Directions.NegX):
				particle.state.pos[0] -= self.wheelSideLength + randn() * self.xNoise
				particle.state.yaw -= self.xYawBias
			elif (direction == Directions.PosY):
				particle.state.pos[1] += self.wheelSideLength + randn() * self.yNoise
				particle.state.yaw += self.yYawBias
			elif (direction == Directions.NegY):
				particle.state.pos[1] -= self.wheelSideLength + randn() * self.yNoise
				particle.state.yaw -= self.yYawBias
			particle.state.yaw += randn() * self.yawBias

	def correct(self):
		# update each particles probability
		# normalize()
		# drawRandomSamples()
		pass

	def getPosition(self):
		pass

