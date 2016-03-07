from directions import *
from state import *
from constants import *
from numpy.random import randn, uniform
from numpy import *
from numpy.linalg import *
from coordinateFrames import *
from joy import *
import pdb

class Particle:
  def __init__(self, pos=[0, 0], yaw=0, prob = 0):
    self.state = RobotState(pos, yaw)
    self.prob = prob

class ParticleFilter(Plan):
  def __init__(self, app, *arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    # noise constants
    self.xNoise = 0.1 # cm
    self.yNoise = 0.1 # cm
    self.yawNoise = 0.002 # radians

    self.tagLength = tagLength

    self.numChosenParticles = 100
    self.numRandomParticles = 10
    self.numTotalParticles = self.numChosenParticles + self.numRandomParticles
    self.particles = []
    self.wheelSideLength = wheelSideLength;
    self.coordinateFrames = CoordinateFrames()

    self.equalProbability = 1.0 / self.numTotalParticles
    for i in range(self.numTotalParticles):
      initialPos = [0, 0]
      initialYaw = 0
      self.particles.append(Particle(initialPos, initialYaw, self.equalProbability))

    self.mostProbable = self.particles[0]
    self.sensor = []
    self.waypoints = []

  def printParticles(self):
    for particle in self.particles:
      print("pos: " + str(particle.state.pos) + "\tyaw: " + str(particle.state.yaw) + "\tprob: " + str(particle.prob))

  def setSensorAndWaypoints(self, sensor, waypoints):
    self.sensor = sensor
    if (len(waypoints) != len(self.waypoints)):
      self.coordinateFrames.calculateRealToWaypointTransformation(waypoints[0], waypoints[1])
    self.waypoints = waypoints

  def behavior(self):
    """
    sensor is a list of front and back sensors [f, b] still in raw sensor form
    waypoints is list of waypoints in real coordinates
    """

    sensorReal = array(ParticleFilter.convertSensor(self.sensor))

    # updating each particle's probability with sensor model
    totalProb = 0
    for particle in self.particles:
      rotatedLength = CoordinateFrames.rotateCCW([self.tagLength/2, 0], -particle.state.yaw)
      frontLoc = particle.state.pos + rotatedLength
      backLoc = particle.state.pos - rotatedLength
      frontDist = frontLoc[1]
      backDist = backLoc[1]

      ParticleFilter.sensorModel(particle, 
        sensorReal,
        array([frontDist, backDist]))
      totalProb += particle.prob

    yield

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
          newParticle = Particle(particle.state.pos, particle.state.yaw, self.equalProbability)
          newParticles.append(newParticle)
          drewParticle = True
          break
      if (not drewParticle):
        newParticles.append(self.mostProbable)

    yield

    # draw scattering of particles around most probable
    for i in range(self.numRandomParticles):
      randPos = randn(2) * 5
      randYaw = randn() * 0.05

      newParticle = Particle(self.mostProbable.state.pos + randPos,
        self.mostProbable.state.yaw + randYaw, 
        self.equalProbability)
      newParticles.append(newParticle)

    self.particles = newParticles

  def setState(self, pos, yaw):
    """
    input is position and yaw in real coordinates
    """
    for particle in self.particles:
      randPos = randn(2) * 5
      randYaw = randn() * 0.05

      particle.prob = self.equalProbability
      particle.state.pos = self.coordinateFrames.convertRealToWaypoint(pos) + randPos
      particle.state.yaw = -self.coordinateFrames.getRealToWaypointYaw() + yaw + randYaw


  def update(self, direction):
    # move each particle with noise
    for particle in self.particles:
      travelDist = [0., 0.]
      if direction == (Directions.PosX):
        travelDist[0] += self.wheelSideLength + randn() * self.xNoise
      elif direction == (Directions.NegX):
        travelDist[0] -= self.wheelSideLength + randn() * self.xNoise
      elif direction == (Directions.PosY):
        travelDist[1] += self.wheelSideLength + randn() * self.yNoise
      elif direction == (Directions.NegY):
        travelDist[1] -= self.wheelSideLength + randn() * self.yNoise

      rotatedTravelDir = CoordinateFrames.rotateCCW(travelDist, -particle.state.yaw)
      particle.state.pos += array(rotatedTravelDir)
      particle.state.yaw += randn() * self.yawNoise

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

  def getState(self):
    realPos = self.coordinateFrames.convertWaypointToReal(self.mostProbable.state.pos)
    return RobotState(realPos, self.mostProbable.state.yaw)

  @staticmethod
  def convertSensor(sensor):
    # converts sensor values into real distances
    # model for sensor is
    # y: real dist
    # x: sensor dist
    # y = 0.0007762568 * x^2 - 0.4104391031*x + 55.0759770855
    return 0.0007762568 * sensor * sensor - \
      0.4104391031 * sensor + 55.0759770855

