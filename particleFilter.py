from directions import *
from state import *
from constants import *
from numpy.random import randn, uniform
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
		self.yawNoise = 0.002 # radians

		self.tagLength = tagLength

		self.numChosenParticles = 80
		self.numRandomParticles = 20
		self.numTotalParticles = self.numChosenParticles + self.numRandomParticles
		self.particles = []
		self.wheelSideLength = wheelSideLength;

		self.equalProbability = 1 / self.numTotalParticles
		for i in range(self.numTotalParticles):
			initialPos = [0, 0]
			initialYaw = 0
			self.particles.append(Particle(initialPos, initialYaw, self.equalProbability))

		self.mostProbable = self.particles[0]


	def setLoc(self, pos, yaw):
		for particle in self.particles:
			particle.prob = self.equalProbability
			particle.state.pos = pos
			particle.state.yaw = yaw

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
			particle.state.yaw += randn() * self.yawNoise

	def correct(self, sensor, waypoints):
		"""
		sensor is a list of front and back sensors [f, b] still in raw sensor form
		waypoints is list of waypoints in real coordinates
		"""
		waypointLine = waypoints[1] - waypoints[0]

		sensorReal = ParticleFilter.convertSensor(sensor)
		sensorFrontDist = lineToPtDist(waypoints[1], waypoints[0], sensorReal[0])
		sensorBackDist = lineToPtDist(waypoints[1], waypoints[0], sensorReal[1])

		# updating each particle's probability with sensor model
		totalProb = 0
		for particle in self.particles:
			rotatedLength = coordinateFrames.rotateCCW([self.tagLength/2, 0], -particle.state.yaw)
			frontLoc = particle.state.pos + rotatedLength
			backLoc = particle.state.pos - rotatedLength

			frontDist = lineToPtDist(waypoints[1], waypoints[0], frontLoc)
			backDist = lineToPtDist(waypoints[1], waypoints[0], backLoc)
			sensorModel(particle, 
				array([sensorFrontDist, sensorBackDist]),
				array([frontDist, backDist]))
			totalProb += particle.prob

		# normalizing particle probabilities so they add to 1
		scalar = 1.0 / totalProb
		for particle in self.particles:
			particle.prob *= scalar

		# sort particles so greatest probabilities are at lower indices
		# this step is so the drawing new random samples will be faster
		self.particles.sort(key=lambda x: x.prob, reverse=True)

		self.mostProbable = self.particles[0]
		
		newParticles = []

		# draw new random samples
		for i in range(self.numChosenParticles):
			randNum = uniform()

			cumulativeProb = 0
			drewParticle = False
			for particle in self.particles:
				cumulativeProb += particle.prob
				if (cumulativeProb > randNum):
					newParticle = Particle(particle.pos, particle.yaw, self.equalProbability)
					newParticles.append(newParticle)
					drewParticle = True
					break
			if (not drewParticle):
				newParticles.append(self.mostProbable)


		# draw scattering of particles around most probable
		for i in range(self.numRandomParticles):
			randPos = randn(2) * 1.5
			randYaw = randn() * 0.1

			newParticle = Particle(self.mostProbable.state.pos + randPos,
				self.mostProbable.state.yaw + randYaw, 
				self.equalProbability)
			newParticles.append(newParticle)

		self.particles = newParticles

	@staticmethod
	def sensorModel(particle, sensorDists, particleDists):
		# distsDiff measures how close the sensor values are
		# to what the distance of this particle to the line
		distsDiff = norm(sensorDists - particleDists)
		scalar = 0.2231

		# the probability we multiply our particle prob by,
		# p = P(sensor value | particle location), will be
		# e^(-distsDiff * scalar)
		# so that when distsDiff is 0, p=1
		# and for distsDiff > 0, p < 1 and drops off rapidly
		# how rapid the drop depends on the scalar
		sensorProb = exp(-distsDiff * scalar)
		particle.prob *= sensorProb

	def getPosition(self):
		pass

	@staticmethod
	def convertSensor(sensor):
		# converts sensor values into real distances
		# model for sensor is
		# y: real dist
		# x: sensor dist
		# y = 0.0007762568 * x^2 - 0.4104391031*x + 55.0759770855
		return 0.0007762568 * sensor * sensor - \
			0.4104391031 * sensor + 55.0759770855

