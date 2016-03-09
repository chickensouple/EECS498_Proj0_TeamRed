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


    self.alpha = 0.4
	
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

    self.particles[0].d = d
    self.particles[0].yaw = -self.coordinateFrames.getRealToWaypointYaw() + yaw
    self.mostProbable = self.particles[0]

  def printParticles(self):
    for particle in self.particles:
      print("pos: " + str(particle.d) + "\tyaw: " + str(particle.yaw) + "\tprob: " + str(particle.prob))


  def setSensorAndWaypoints(self, sensor, waypoints):
    self.sensor = sensor
    if (len(waypoints) != len(self.waypoints)):
      self.coordinateFrames.calculateRealToWaypointTransformation(waypoints[0], waypoints[1])
      self.setState(waypoints[0], 0)
    self.waypoints = waypoints

  def correct(self):
    totalProb = 0
    sensorReal = array(ParticleFilter.convertSensor(self.sensor))

    # try to localize R
    # self.correctR(sensorReal)

    for particle in self.particles:
      rotatedLength = CoordinateFrames.rotateCCW([Constants.tagLength, 0], particle.yaw)
      frontDist = abs(particle.d + rotatedLength[1])
      backDist = abs(particle.d - rotatedLength[1])

      self.sensorModel(particle, sensorReal, array([frontDist, backDist]))
      totalProb += particle.prob


    self.particles.sort(key=lambda x: x.prob, reverse=True)
    # self.mostProbable.d = self.mostProbable.d + self.alpha * (self.particles[0].d - self.mostProbable.d)
    # self.mostProbable.yaw = self.mostProbable.yaw + self.alpha * (self.particles[0].yaw - self.mostProbable.yaw)
    self.mostProbable = self.particles[0]


    # normalize probabilities so they add to one
    scalar = 1.0 / totalProb
    for particle in self.particles:
      particle.prob *= scalar


    # draw new samples
    newParticles = []
    for i in range(self.numChosenParticles):
      randNum = uniform()

      cumulativeProb = 0
      drewParticle = False
      for particle in self.particles:
        cumulativeProb += particle.prob
        if (cumulativeProb > randNum):
          newParticle = Particle(particle.d, particle.yaw, self.equalProbability)
          newParticles.append(newParticle)
          drewParticle = True
          break
      if (not drewParticle):
        newParticles.append(self.mostProbable)

    # draw scattering of particles around each solution
    solutions = ParticleFilter.generateSolutions(sensorReal)
    numParticlesPerSolution = self.numRandomParticles / len(solutions)
    for i in range(self.numRandomParticles):
      randPos = randn() * 1.5
      randYaw = randn() * 0.03

      solutionNum = int(floor(i / numParticlesPerSolution))
      solution = solutions[solutionNum]

      newParticles.append(Particle(solution[0] + randPos, solution[1] + randYaw, self.equalProbability))


    self.particles = newParticles



    # solutions = ParticleFilter.generateSolutions(sensorReal)

    # # assign new probabilities so that they 
    # # are self.equalProbability (totalProb may not add up to 1 right now)
    # newProb = self.equalProbability * totalProb

    # numParticlesPerSolution = self.numRandomParticles / len(solutions)
    # for i in range(self.numRandomParticles):
    #   idx = len(self.particles) - i - 1

    #   randPos = randn() * 1.5
    #   randYaw = randn() * 0.03

    #   solutionNum = int(floor(i / numParticlesPerSolution))

    #   solution = solutions[solutionNum]
    
    #   totalProb += newProb - self.particles[idx].prob

    #   self.particles[idx] = Particle(solution[0] + randPos, solution[1] + randYaw, newProb)

    # # scale probabilities so they add to one
    # scalar = 1.0 / totalProb
    # for particle in self.particles:
    #   particle.prob *= scalar

    # print("prob: " + str(self.particles[0].prob))



  def correctR(self, realSensor):
    # modify self.r if necessary
    rotatedLength = CoordinateFrames.rotateCCW([Constants.tagLength, 0], self.mostProbable.yaw)

    # locations of the front and back sensor of most probable particle
    frontR = self.r + rotatedLength[0]
    backR = self.r - rotatedLength[0]
    waypointDist = norm(array(self.waypoints[1]) - array(self.waypoints[0]))

    if (sensorReal[1] == -1 and sensorReal[0] == -1):
      # we are completely not between waypoints
      if ((backR > 0 or backR < waypointDist) or
        (frontR > 0 or frontR < waypointDist)):
        # our estimate is that one of our sensors is still
        # in range of waypoints

        # determine which end we are off of by looking at our current estimate
        if (self.r < waypointDist / 2):
          # closest to start
          # decrease r
          self.r -= (waypointDist) / 10
        else:
          # closest to end
          # increase r
          self.r += (waypointDist) / 10
    elif (sensorReal[1] == -1 or sensorReal[0] == -1):
      # one sensor is no longer in range of waypoints
      if (backR > 0 and frontR > 0 and backR < waypointDist and frontR < waypointDist):
        # if we think we are in range of waypoints

        # find which end we are closest to
        if (self.r < waypointDist / 2):
          # we are close to start
          # move r to within range [0, rotatedLength[1]]
          self.r = uniform() * rotatedLength[1]
        else:
          # we are close to end
          # move r to within range
          self.r = waypointDist - (uniform * rotatedLength[1])

      if ((backR < 0 or backR > waypointDist) and 
        (frontR < 0 and frontR > waypointDist)):
        # our estimate is completely off


        # find which end we are closest to
        if (self.r < waypointDist / 2):
          # we are close to start
          # move r to within range [0, rotatedLength[1]]
          self.r = uniform() * rotatedLength[1]
        else:
          # we are close to end
          # move r to within range
          self.r = waypointDist - (uniform * rotatedLength[1])
    elif (backR < 0 or backR > waypointDist or 
      frontR < 0 or frontR > waypointDist):
      # we estimate that our sensors are not between waypoints
      # however we are getting measurements

      # find which end we are closest to
      if (self.r < waypointDist / 2):
        # we are close to start
        # move r to within range [0, rotatedLength[1]]
        self.r = uniform() * rotatedLength[1]
      else:
        # we are close to end
        # move r to within range
        self.r = waypointDist - (uniform * rotatedLength[1])


    # elif (sensorReal[1] != -1 and backR < 0):
    #   # we are actually in between waypoints
    #   # but we think we are too close to the start waypoint
    #   # increase r
    # elif (sensorReal[1] == -1 and backR > 0):
    #   # we think we are between waypoints but we are actually
    #   # too close to start of waypoint
    #   # decrease r so that it is within start position
    #   self.r = 
    # elif (sensorReal[0] != -1 and frontR > waypointDist):
    #   # we think we are very close to end, but actually
    #   # we are at not as large of an R
    #   # decrease r
    # elif (sensorReal[0] == -1 and frontR < waypointDist):
    #   # we think we are not yet close to end waypoint,
    #   # but actually we are
    #   # increase r so that it is within end position




  @staticmethod
  def generateSolutions(sensor):
    solutions = []

    f = sensor[0]
    b = sensor[1]

    d1 = (f+b)/2
    theta1 = atan2((f-b), 2 * Constants.tagLength)

    d2 = (f-b)/2
    theta2 = atan2((f+b), 2 * Constants.tagLength)

    solutions.append([d1, theta1])
    solutions.append([-d1, -theta1])

    solutions.append([d2, theta2])
    solutions.append([-d2, -theta2])

    return solutions

  def sensorModel(self, particle, sensorDists, particleDists):
    # distsDiff measures how close the sensor values are
    # to what the distance of this particle to the line
    if (sensorDists[0] == -1):
      distsDiff = sqrt(2) * abs(sensorDists[1] - particleDists[1])
    elif (sensorDists[1] == -1):
      distsDiff = sqrt(2) * abs(sensorDists[0] - particleDists[0])
    elif (sensorDists[0] == -1 and sensorDists[1] == -1):
      distsDiff = 0
    else:
      distsDiff = norm(sensorDists - particleDists)

    # Yawdiff measures how close the yaw of the particle is to our 
    # estimated yaw
    yawScalar = 50
    # yawDiff = yawScalar * abs(particle.yaw - self.mostProbable.yaw)
    yawDiff = yawScalar * abs(particle.yaw + self.coordinateFrames.getRealToWaypointYaw())

    # scalar = 0.2231
    scalar = 0.11

    # the probability we multiply our particle prob by,
    # p = P(sensor value | particle location), will be
    # e^(-distsDiff * scalar)
    # so that when distsDiff is 0, p=1
    # and for distsDiff > 0, p < 1 and drops off rapidly
    # how rapid the drop depends on the scalar
    sensorProb = exp(-(distsDiff + yawDiff) * scalar)
    particle.prob *= sensorProb

  def actionModel(self, direction):
    travelDist = array([0., 0.])
    noiseVar = 0
    if direction == (Directions.PosX):
      travelDist[0] += Constants.wheelSideLength
      noiseVar = self.xNoise
    elif direction == (Directions.NegX):
      travelDist[0] -= Constants.wheelSideLength
      noiseVar = self.xNoise
    elif direction == (Directions.PosY):
      travelDist[1] += Constants.wheelSideLength
      noiseVar = self.yNoise
    elif direction == (Directions.NegY):
      travelDist[1] -= Constants.wheelSideLength
      noiseVar = self.yNoise
    travelDir = travelDist / Constants.wheelSideLength
 

    rotatedTravelDist = CoordinateFrames.rotateCCW(travelDist, self.mostProbable.yaw)

    self.r += rotatedTravelDist[0]

    # move each particle with noise
    for particle in self.particles:
      particleTravelDist = travelDist + (travelDir * noiseVar * randn())
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
    # # y = 0.0007762568 * x^2 - 0.4104391031*x + 55.0759770855
    # ret = 0.0007762568 * sensor * sensor - \
    #   0.4104391031 * sensor + 55.0759770855

    ret = -0.074 * sensor + 17.4935
    if (ret[0] < 0):
      ret[0] = 0
    if (ret[1] < 0):
      ret[1] = 0

    if (sensor[0] < 5):
      ret[0] = -1
    if (sensor[1] < 5):
      ret[1] = -1
    return ret


