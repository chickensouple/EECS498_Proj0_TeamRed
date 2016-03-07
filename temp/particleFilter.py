from common import *
from numpy.random import randn, uniform
from numpy import *
from numpy.linalg import *
from coordinateFrames import *
from joy import *
import pdb

class Particle:
  def __init__(self, d=0, yaw=0, prob=0):
    self.d = d
    self.yaw = yaw
    self.prob = prob

class ParticleFilter:
  """
  Particle Filter that estimates
  (r, d, theta) with respect to the current waypoint line
  Particles consist of (d, theta)
  """
  def __init__(self, core):
    # noise constants
    self.xNoise = 0.1 # cm
    self.yNoise = 0.1 # cm
    self.yawNoise = 0.002 # radians
	
    self.coordinateFrames = core.coordinateFrames

    # particles
    self.numChosenParticles = 100
    self.numRandomParticles = 40 # must be multiple of 4!
    self.numTotalParticles = self.numChosenParticles + self.numRandomParticles
    self.particles = []


    self.equalProbability = 1.0 / self.numTotalParticles
    for i in range(self.numTotalParticles):
      initialPos = 0
      initialYaw = 0
      self.particles.append(Particle(initialPos, initialYaw, self.equalProbability))

    self.mostProbable = self.particles[0]
    self.sensor = []
    self.waypoints = []

    # state
    self.r = 0


  def setState(self, pos, yaw):
    """
    input is position and yaw in real coordinates
    """
    [r, d] = self.coordinateFrames.convertRealToWaypoint(pos)
    self.r = r
    for particle in self.particles:
      randPos = randn() * 5
      randYaw = randn() * 0.05

      particle.prob = self.equalProbability
      particle.d = d + randPos
      particle.yaw = -self.coordinateFrames.getRealToWaypointYaw() + yaw + randYaw

  def printParticles(self):
    for particle in self.particles:
      print("pos: " + str(particle.d) + "\tyaw: " + str(particle.state.yaw) + "\tprob: " + str(particle.prob))


  def setSensorAndWaypoints(self, sensor, waypoints):
    self.sensor = sensor
    if (len(waypoints) != len(self.waypoints)):
      self.coordinateFrames.calculateRealToWaypointTransformation(waypoints[0], waypoints[1])
    self.waypoints = waypoints

  def correct(self):
    # TODO: take into account when sensor is -1
    totalProb = 0
    sensorReal = array(ParticleFilter.convertSensor(self.sensor))

    for particle in self.particles:
      rotatedLength = CoordinateFrames.rotateCCW([Constants.tagLength, 0], -particle.yaw)
      frontDist = abs(particle.d + rotatedLength[1])
      backDist = abs(particle.d - rotatedLength[1])

      ParticleFilter.sensorModel(particle, sensorReal, array([frontDist, backDist]))
      totalProb += particle.prob

    pdb.set_trace()

    self.particles.sort(key=lambda x: x.prob, reverse=True)
    self.mostProbable = self.particles[0]

    solutions = ParticleFilter.generateSolutions(sensorReal)

    for i in range(self.numRandomParticles):
      idx = len(self.particles) - i

      randPos = randn() * 3
      randYaw = randn() * 0.05

      solutionNum = floor(i / len(solutions))
      solution = solutions[solutionNum]
    
      totalProb += self.equalProbability - self.particles[idx].prob

      self.particles[idx] = Particle(solution[0] + randPos, solution[1] + randYaw, self.equalProbability)

    scalar = 1.0 / totalProb
    for particle in self.particles:
      particle.prob *= scalar



  @staticmethod
  def generateSolutions(sensor):
    solutions = []

    f = sensor[0]
    b = sensor[1]

    d1 = (f+b)/2
    theta1 = atan2((f-b), self.tagLength)

    d2 = (f-b)/2
    theta2 = atan2((f+b), self.tagLength)

    solutions.append([d1, theta1])
    solutions.append([-d1, -theta1])

    solutions.append([d2, theta2])
    solutions.append([-d2, -theta2])

    return solutions

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

  def actionModel(self, direction):
    travelDist = array([0., 0.])
    noiseVar = 0
    if direction == (Directions.PosX):
      travelDist[0] += self.wheelSideLength
      noiseVar = self.xNoise
    elif direction == (Directions.NegX):
      travelDist[0] -= self.wheelSideLength
      noiseVar = self.xNoise
    elif direction == (Directions.PosY):
      travelDist[1] += self.wheelSideLength
      noiseVar = self.yNoise
    elif direction == (Directions.NegY):
      travelDist[1] -= self.wheelSideLength
      noiseVar = self.yNoise
    travelDir = travelDist / self.wheelSideLength
 

    rotatedTravelDist = CoordinateFrames.rotateCCW(travelDist, -self.mostProbable.yaw)

    # move each particle with noise
    for particle in self.particles:
      particleTravelDist += travelDist + (travelDir * noiseVar * randn())
      rotatedTravelDist = CoordinateFrames.rotateCCW(particleTravelDist, particle.yaw)
      particle.d += rotatedTravelDist[1]
      particle.yaw += randn() * self.yawNoise

  def getState(self):
    realPos = self.coordinateFrames.convertWaypointToReal([self.r, self.mostProbable.d])
    return RobotState(realPos, self.mostProbable.yaw)

  @staticmethod
  def convertSensor(sensor):
    # converts sensor values into real distances
    # model for sensor is
    # y: real dist
    # x: sensor dist
    # y = 0.0007762568 * x^2 - 0.4104391031*x + 55.0759770855
    return 0.0007762568 * sensor * sensor - \
      0.4104391031 * sensor + 55.0759770855

