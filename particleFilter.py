from directions import *
from state import *
from constants import *
from numpy.random import randn
from numpy import *
from numpy.linalg import *
from coordinateFrames import *

class Particle:
	def __init__(self, pos=[0, 0], yaw=0, prob = 0):
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

		self.tagLength = tagLength

		self.numChosenParticles = 80
		self.numRandomParticles = 20
		self.numTotalParticles = self.numChosenParticles + self.numRandomParticles
		self.particles = []
		self.wheelSideLength = wheelSideLength;

		probability = 1 / self.numTotalParticles
		for i in range(self.numTotalParticles):
			initialPos = [0, 0]
			initialYaw = 0
			self.particles.append(Particle(initialPos, initialYaw, probability))

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

	def correct(self, sensor, waypoints):
		"""
		sensor is a list of front and back sensors [f, b] still in raw sensor form
		waypoints is list of waypoints in real coordinates
		"""
		waypointLine = waypoints[1] - waypoints[0]

		sensorReal = self.convertSensor(sensor, waypoints)
		sensorFrontDist = lineToPtDist(waypoints[1], waypoints[0], sensorReal[0])
		sensorBackDist = lineToPtDist(waypoints[1], waypoints[0], sensorReal[1])

		for particle in self.particles:
			rotatedLength = coordinateFrames.rotateCCW([self.tagLength/2, 0], -particle.state.yaw)
			frontLoc = particle.state.pos + rotatedLength
			backLoc = particle.state.pos - rotatedLength

			frontDist = lineToPtDist(waypoints[1], waypoints[0], frontLoc)
			backDist = lineToPtDist(waypoints[1], waypoints[0], backLoc)
			sensorModel(particle, 
				array([sensorFrontDist, sensorBackDist]),
				array([frontDist, backDist]))



		# update each particles probability
		# normalize()
		# drawRandomSamples()
		pass


	@staticmethod
	def sensorModel(particle, sensorDists, particleDists):
		# distsDiff measures how close the sensor values are
		# to what the distance of this particle to the line
		distsDiff = norm(sensorDists - particleDists)
		scalar = 0.1

		# the probability we multiply our particle prob by,
		# p = p(sensor value | particle location) will be
		# e^(-distsDiff * scalar)
		# so that when distsDiff is 0, p=1
		# and for distsDiff > 0, p < 1 and drops off rapidly
		# how rapid the drop depends on the scalar
		sensorProb = exp(-distsDiff * scalar)
		particle.prob *= sensorProb



	def getPosition(self):
		pass

	def convertSensor(self, sensor, waypoints):
		return sensor

